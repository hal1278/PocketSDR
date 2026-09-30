//
//  Pocket SDR C Library - Antenna Array Functions
//
//  Author:
//  T.TAKASU
//
//  History:
//  2026-04-17  1.0  new
//  2026-04-25  1.1  switch from batch LS to per-epoch EKF
//
#include "pocket_sdr.h"
#include <float.h>

#define DPI         (2.0 * PI)
#define YAW_STEP    15.0        // yaw search step (deg)
#define MAX_ITER    15          // max LSQ iterations for init
#define CONV_THRES_A 1e-6       // angle update convergence (rad)
#define CONV_THRES_B 1e-5       // bias update convergence (m)
#define MIN_EL      15.0        // elevation mask (deg)
#define MAX_RMS     0.020       // max RMS of residuals (m)
#define NX          (3+SDR_MAX_RFCH)
#define MAX_NV      ((SDR_MAX_RFCH-1)*MAXOBS)
#define VAR_INIT_RPY  1.0       // initial covariance for rpy (rad^2)
#define VAR_INIT_BIAS 1.0       // initial covariance for bias (m^2)
#define VAR_PROC_RPY  1e-4      // process noise variance per epoch (rad^2)
#define VAR_PROC_BIAS 1e-8      // process noise variance per epoch (m^2)
#define VAR_MEAS      1e-4      // measurement variance (m^2)
#define ARRAY_W_SCALE 256       // array weight Q-factor (Q8)
#define ARRAY_FREQ    1.57542e9 // array L1 frequency (Hz)
#define ARRAY_FREQ_TOL 1e6      // RF CH center freq tolerance (Hz)
#define CLIP(x, mn, mx) ((x) < (mn) ? (mn) : ((x) > (mx) ? (mx) : (x)))
#define STATIC_MIN_EPOCHS 3
#define STATIC_MIN_SPAN 2.0
#define STATIC_MAX_SPAN 30.0
#define STATIC_MAX_EPOCHS 32
#define STATIC_MAX_ROWS 2048
#define STATIC_MAX_COND 1e6
static int calib_diag_active = 0, calib_diag_done = 0;
static int calib_diag_successes = 0;
static int calib_diag_last_pairs = -1;

typedef struct {
    gtime_t time;
    double los[3], lambda, phase_sd;
    int sat, rfch, epoch;
} sdr_calib_meas_t;

typedef struct {
    gtime_t time;
    int count;
} sdr_calib_epoch_t;

struct sdr_calib_static_tag {
    sdr_calib_meas_t *meas, *pending;
    sdr_calib_epoch_t epochs[STATIC_MAX_EPOCHS];
    double *work, *residual;
    double seed_bias[SDR_MAX_RFCH];
    int first, nmeas, ep_first, nepoch, total_epochs;
    int nsat, last_rms_valid;
    double span, last_rms;
};

// rotation matrix and partials by Euler angles (Z-Y-X) ------------------------
static void euler_rot(const double *rpy, double *R, double *Dr, double *Dp,
    double *Dy)
{
    double cr = cos(rpy[0]), sr = sin(rpy[0]), cp = cos(rpy[1]);
    double sp = sin(rpy[1]), cy = cos(rpy[2]), sy = sin(rpy[2]);
    
    // Rz(yaw) * Ry(pitch) * Rx(roll) (X:forward,Y:left,Z:up)
    R [0] =  cy*cp; R [3] =  cy*sp*sr - sy*cr; R [6] =  cy*sp*cr + sy*sr;
    R [1] =  sy*cp; R [4] =  sy*sp*sr + cy*cr; R [7] =  sy*sp*cr - cy*sr;
    R [2] = -sp;    R [5] =  cp*sr;            R [8] =  cp*cr;
    if (!Dr) return;
    
    // Dr = dR/droll, Dp = dR/dpitch, Dy = dR/dyaw
    Dr[0] =  0.0;   Dr[3] =  cy*sp*cr + sy*sr; Dr[6] = -cy*sp*sr + sy*cr;
    Dr[1] =  0.0;   Dr[4] =  sy*sp*cr - cy*sr; Dr[7] = -sy*sp*sr - cy*cr;
    Dr[2] =  0.0;   Dr[5] =  cp*cr;            Dr[8] = -cp*sr;
    Dp[0] = -cy*sp; Dp[3] =  cy*cp*sr;         Dp[6] =  cy*cp*cr;
    Dp[1] = -sy*sp; Dp[4] =  sy*cp*sr;         Dp[7] =  sy*cp*cr;
    Dp[2] = -cp;    Dp[5] = -sp*sr;            Dp[8] = -sp*cr;
    Dy[0] = -sy*cp; Dy[3] = -sy*sp*sr - cy*cr; Dy[6] = -sy*sp*cr + cy*sr;
    Dy[1] =  cy*cp; Dy[4] =  cy*sp*sr - sy*cr; Dy[7] =  cy*sp*cr + sy*sr;
    Dy[2] =  0.0;   Dy[5] =  0.0;              Dy[8] =  0.0;
}

// LOS unit vector (receiver -> satellite) in ENU frame ------------------------
static int los_vec(gtime_t time, int sat, const nav_t *nav, const double *pos,
    const double *rr, double *e)
{
    double rs[6], dts[2], var, e_ecef[3], azel[2];
    int svh;
    
    if (!satpos(time, time, sat, EPHOPT_BRDC, nav, rs, dts, &var, &svh)) {
        return 0;
    }
    if (svh || geodist(rs, rr, e_ecef) <= 0.0) return 0;
    if (satazel(pos, e_ecef, azel) < MIN_EL * D2R) return 0;
    ecef2enu(pos, e_ecef, e);
    return 1;
}

// SD model term -e'*(R*b) and partials wrt roll/pitch/yaw ---------------------
static double sd_proj(const double *e, const double *b, const double *R,
    const double *Dr, const double *Dp, const double *Dy, double *H)
{
    double v[3], vr[3], vp[3], vy[3];
    
    matmul("NN", 3, 1, 3, 1.0, R,  b, 0.0, v );
    matmul("NN", 3, 1, 3, 1.0, Dr, b, 0.0, vr);
    matmul("NN", 3, 1, 3, 1.0, Dp, b, 0.0, vp);
    matmul("NN", 3, 1, 3, 1.0, Dy, b, 0.0, vy);
    H[0] = -dot(e, vr, 3);
    H[1] = -dot(e, vp, 3);
    H[2] = -dot(e, vy, 3);
    return -dot(e, v, 3);
}

// build measurements and design matrix ----------------------------------------
static int build_meas(const sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr, const double *x, double *v, double *H)
{
    double pos[3], R[9], Dr[9], Dp[9], Dy[9];
    int nv = 0, f = array->freq;
    
    ecef2pos(rr, pos);
    euler_rot(x, R, Dr, Dp, Dy);
    
    for (int i = 0; i < nobs; i++) {
        const obsd_t *p = obs + i;
        double freq, lam, e[3];
        
        // search reference (CH1)
        if (p->rcv != 1 || p->L[f] == 0.0 || !array->ant_ena[0]) continue;
        if (!los_vec(p->time, p->sat, nav, pos, rr, e)) continue;
        if ((freq = sat2freq(p->sat, p->code[f], nav)) == 0.0) continue;
        lam = CLIGHT / freq;
        
        // residuals of SD phase
        for (int j = 0; j < nobs && nv < MAX_NV; j++) {
            const obsd_t *q = obs + j;
            double b_body[3], proj;
            int k = q->rcv - 1;
            
            if (q->sat != p->sat || k <= 0 || !array->ant_ena[k]) continue;
            if (q->L[f] == 0.0) continue;
            
            for (int l = 0; l < 3; l++) {
                b_body[l] = array->ant_pos[k][l] - array->ant_pos[0][l];
            }
            proj = sd_proj(e, b_body, R, Dr, Dp, Dy, H + nv * NX);
            v[nv] = lam * (q->L[f] - p->L[f]) - (proj - x[3+k]);
            v[nv] -= lam * floor(v[nv] / lam + 0.5); // -lam / 2 <= v < lam / 2
            H[3+k+NX*(nv++)] = -1.0;
        }
    }
    return nv;
}

// log phase model terms for a fitted initialization epoch -------------------
static void diag_phase_terms(const sdr_array_t *array, const obsd_t *obs,
    int nobs, const nav_t *nav, const double *rr, double start_yaw)
{
    double pos[3], x0[NX] = {0}, R0[9], R1[9], Dr[9], Dp[9], Dy[9];
    x0[2] = start_yaw;
    ecef2pos(rr, pos);
    euler_rot(x0, R0, Dr, Dp, Dy);
    euler_rot(array->x, R1, Dr, Dp, Dy);
    fprintf(stderr, "  fitted_state: roll=%.9g pitch=%.9g yaw=%.9g",
        array->x[0], array->x[1], array->x[2]);
    for (int k = 1; k < array->nrfch; k++) fprintf(stderr,
        " bias_ch%d=%.9g", k + 1, array->x[3+k]);
    fprintf(stderr, "\n");
    int row = 0, f = array->freq;
    for (int i = 0; i < nobs; i++) {
        const obsd_t *p = obs + i;
        double e[3], freq;
        if (p->rcv != 1 || p->L[f] == 0.0 || !array->ant_ena[0]) continue;
        if (!los_vec(p->time, p->sat, nav, pos, rr, e)) continue;
        if ((freq = sat2freq(p->sat, p->code[f], nav)) == 0.0) continue;
        double lam = CLIGHT / freq;
        for (int j = 0; j < nobs && row < MAX_NV; j++) {
            const obsd_t *q = obs + j;
            int k = q->rcv - 1;
            double b[3], h[3];
            if (q->sat != p->sat || k <= 0 || !array->ant_ena[k]) continue;
            if (q->L[f] == 0.0) continue;
            for (int l = 0; l < 3; l++) b[l] = array->ant_pos[k][l] -
                array->ant_pos[0][l];
            double meas = lam * (q->L[f] - p->L[f]);
            double geom0 = sd_proj(e, b, R0, Dr, Dp, Dy, h);
            double geom1 = sd_proj(e, b, R1, Dr, Dp, Dy, h);
            double pre = meas - geom0;
            double bias = array->x[3+k], post = meas - (geom1 - bias);
            pre -= lam * floor(pre / lam + 0.5);
            post -= lam * floor(post / lam + 0.5);
            char sat[16];
            satno2id(p->sat, sat);
            fprintf(stderr, "  CALPAIR row=%d sat=%s ref=%d ch=%d code_ref=%s code_ch=%s LLI_ref=%d LLI_ch=%d lam=%.9g meas=%.9g geom0=%.9g pre=%.9g geom=%.9g bias=%.9g model=%.9g post=%.9g\n",
                row++, sat, p->rcv, q->rcv, code2obs(p->code[f]),
                code2obs(q->code[f]), p->LLI[f], q->LLI[f], lam, meas,
                geom0, pre, geom1, bias, geom1 - bias, post);
        }
    }
}

// initialize attitude and biases by LSQ ---------------------------------------
static int lsq_init(const sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr, double *x, double *rms)
{
    int mode = array->calib_mode;
    for (int iter = 0; iter < MAX_ITER; iter++) {
        double v[MAX_NV+SDR_MAX_RFCH+SDR_MAX_RFCH+3] = {0};
        double H[NX*(MAX_NV+SDR_MAX_RFCH+SDR_MAX_RFCH+3)] = {0};
        double dx[NX], Q[NX*NX];
        
        int nv = build_meas(array, obs, nobs, nav, rr, x, v, H);
        if (nv == 0) return 0;
        int nphase = nv;
        
        for (int i = 0; i < SDR_MAX_RFCH; i++) { // avoid rank-deficient
            if (i == 0 || !array->ant_ena[i]) H[3+i+NX*(nv++)] = 1e-12;
        }
        if (mode == SDR_CALIB_BIAS) { // freeze rpy at zero
            for (int i = 0; i < 3; i++) {
                H[i+NX*nv] = 1.0;
                v[nv++] = -x[i];
            }
        } else if (mode == SDR_CALIB_RPY) { // freeze bias at current value
            for (int i = 3; i < NX; i++) {
                H[i+NX*nv] = 1.0;
                v[nv++] = 0.0;
            }
        }
        if (calib_diag_active && iter == 0) {
            fprintf(stderr, "  build_meas: phase_nv=%d rows=%d nx=%d\n", nphase, nv, NX);
            for (int i = 0; i < NX; i++) {
                double col = 0.0;
                for (int j = 0; j < nv; j++) col += H[i+NX*j] * H[i+NX*j];
                fprintf(stderr, "    column %d norm2=%.9g\n", i, col);
            }
            if (fabs(x[2] + PI) < 1e-6) for (int j = 0; j < nphase; j++) {
                fprintf(stderr, "    row %d v=%.9g H=", j, v[j]);
                for (int i = 0; i < NX; i++) fprintf(stderr, " %.9g", H[i+NX*j]);
                fprintf(stderr, "\n");
            }
        }
        int status = lsq(H, v, NX, nv, dx, Q);
        if (calib_diag_active) fprintf(stderr, "    iter=%d ls_status=%d prefit_rms=%.9g\n",
            iter, status, sqrt(dot(v, v, nphase) / nphase));
        if (status) break;
        
        for (int i = 0; i < NX; i++) x[i] += dx[i];
        
        if (norm(dx, 3) < CONV_THRES_A && norm(dx + 3, NX - 3) < CONV_THRES_B) {
            x[2] -= DPI * floor(x[2] / DPI + 0.5); // -PI <= yaw < PI
            *rms = sqrt(dot(v, v, nv) / nv);
            return 1;
        }
    }
    return 0;
}

// initialize KF states --------------------------------------------------------
static void kf_init(sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr)
{
    int mode = array->calib_mode;
    double best_yaw = 0.0;
    array->rms = 1e3;
    
    if (mode == SDR_CALIB_BIAS) { // rpy is fixed at 0; no yaw search needed
        double x[NX] = {0}, rms = 0.0;
        int ok = lsq_init(array, obs, nobs, nav, rr, x, &rms);
        if (calib_diag_active) fprintf(stderr, "  candidate bias: ok=%d rms=%.9g\n", ok, rms);
        if (ok) {
            matcpy(array->x, x, NX, 1);
            array->rms = rms;
        }
    } else { // BOTH or RPY: search over yaw initial values
        for (double y = -PI; y < PI; y += YAW_STEP * D2R) {
            double x[NX] = {0.0, 0.0, y}, rms = 0.0;
            if (mode == SDR_CALIB_RPY) { // preserve loaded biases
                for (int i = 3; i < NX; i++) x[i] = array->x[i];
            }
            int ok = lsq_init(array, obs, nobs, nav, rr, x, &rms);
            if (calib_diag_active) fprintf(stderr, "  yaw=%.1f ok=%d rms=%.9g\n", y * R2D, ok, rms);
            if (!ok) continue;
            if (rms < array->rms) {
                matcpy(array->x, x, NX, 1);
                array->rms = rms;
                best_yaw = y;
            }
        }
    }
    if (calib_diag_active) fprintf(stderr, "  best_rms=%.9g max_rms=%.9g accepted=%d nep=%d\n",
        array->rms, MAX_RMS, array->rms <= MAX_RMS, array->nep);
    if (calib_diag_active && array->rms < 1e3) {
        diag_phase_terms(array, obs, nobs, nav, rr, best_yaw);
        double v[MAX_NV], H[NX*MAX_NV] = {0};
        int nv = build_meas(array, obs, nobs, nav, rr, array->x, v, H);
        fprintf(stderr, "  best_solution: yaw=%.6f nv=%d\n", array->x[2] * R2D, nv);
        for (int i = 0; i < nv; i++) fprintf(stderr,
            "    postfit_residual %d=%.9g\n", i, v[i]);
    }
    if (array->rms > MAX_RMS) {
        array->calib_valid = 0;
        array->calib_source = SDR_CALIB_SRC_NONE;
        return;
    }
    
    memset(array->P, 0, sizeof(double) * NX * NX);
    for (int i = 0; i < NX; i++) {
        if (mode == SDR_CALIB_BIAS && i < 3) continue; // rpy frozen at 0
        if (mode == SDR_CALIB_RPY  && i >= 3) continue; // bias frozen
        array->P[i+i*NX] = (i < 3) ? VAR_INIT_RPY : VAR_INIT_BIAS;
    }
    array->calib_valid = 1;
    array->calib_source = SDR_CALIB_SRC_CONTINUOUS;
}

// predict and update KF states ------------------------------------------------
static void kf_update(sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr)
{
    int mode = array->calib_mode;
    double v[MAX_NV], H[NX*MAX_NV] = {0};
    
    // predict: P += Q (random walk; identity transition); skip frozen states
    if (mode == SDR_CALIB_BIAS) {
        array->x[0] = array->x[1] = array->x[2] = 0.0;
        for (int i = 3; i < NX; i++) array->P[i+i*NX] += VAR_PROC_BIAS;
    } else if (mode == SDR_CALIB_RPY) {
        for (int i = 0; i < 3; i++) array->P[i+i*NX] += VAR_PROC_RPY;
    } else {
        for (int i = 0; i < NX; i++) {
            array->P[i+i*NX] += (i < 3) ? VAR_PROC_RPY : VAR_PROC_BIAS;
        }
    }
    int nv = build_meas(array, obs, nobs, nav, rr, array->x, v, H);
    if (nv < 1) return;
    
    // KF measurement update
    double *R = (double *)sdr_malloc(sizeof(double) * nv * nv);
    for (int i = 0; i < nv; i++) R[i+i*nv] = VAR_MEAS;
    int info = filter(array->x, array->P, H, v, R, NX, nv);
    sdr_free(R);
    if (info) return;
    
    array->x[2] -= DPI * floor(array->x[2] / DPI + 0.5); // -PI <= yaw < PI
    
    // post-fit residual RMS
    nv = build_meas(array, obs, nobs, nav, rr, array->x, v, H);
    double rms = nv > 0 ? sqrt(dot(v, v, nv) / nv) : 0.0;
    
    if (rms > MAX_RMS) { // fallback to init KF
        memset(array->P, 0, sizeof(double) * NX * NX);
        array->nep = 0;
        array->calib_valid = 0;
        array->calib_source = SDR_CALIB_SRC_NONE;
    } else {
        array->rms = rms;
        array->nep++;
        array->calib_valid = 1;
        array->calib_source = SDR_CALIB_SRC_CONTINUOUS;
    }
}

// allocate the bounded static calibration workspace ---------------------------
static sdr_calib_static_t *static_cal_new(void)
{
    sdr_calib_static_t *cal = (sdr_calib_static_t *)sdr_malloc(sizeof(*cal));
    cal->meas = (sdr_calib_meas_t *)sdr_malloc(sizeof(*cal->meas) *
        STATIC_MAX_ROWS);
    cal->pending = (sdr_calib_meas_t *)sdr_malloc(sizeof(*cal->pending) *
        MAX_NV);
    cal->work = (double *)sdr_malloc(sizeof(double) * STATIC_MAX_ROWS * NX);
    cal->residual = (double *)sdr_malloc(sizeof(double) * STATIC_MAX_ROWS);
    return cal;
}

// release the static calibration workspace ------------------------------------
static void static_cal_free(sdr_calib_static_t *cal)
{
    if (!cal) return;
    sdr_free(cal->meas);
    sdr_free(cal->pending);
    sdr_free(cal->work);
    sdr_free(cal->residual);
    sdr_free(cal);
}

// clear collected static measurements and statistics --------------------------
static void static_cal_clear(sdr_calib_static_t *cal)
{
    if (!cal) return;
    cal->first = cal->nmeas = cal->ep_first = cal->nepoch = 0;
    cal->total_epochs = cal->nsat = cal->last_rms_valid = 0;
    cal->span = cal->last_rms = 0.0;
}

// generate new antenna array --------------------------------------------------
sdr_array_t *sdr_array_new(int nrfch, int freq)
{
    sdr_array_t *array = (sdr_array_t *)sdr_malloc(sizeof(sdr_array_t));
    array->freq = freq;
    array->nrfch = nrfch;
    array->calib_mode = SDR_CALIB_BOTH;
    array->calib_alg = SDR_CALIB_ALG_CONTINUOUS;
    array->calib_source = SDR_CALIB_SRC_NONE;
    for (int i = 0; i < SDR_MAX_RFCH; i++) {
        array->ant_ena[i] = (i < nrfch);
    }
    return array;
}

// free antenna array ----------------------------------------------------------
void sdr_array_free(sdr_array_t *array)
{
    if (!array) return;
    static_cal_free(array->static_cal);
    sdr_free(array);
}

static void reset_state(sdr_array_t *array);

// set element positions and enables for antenna array -------------------------
int sdr_array_ant_pos(sdr_array_t *array, const double *ant_pos,
    const int *ant_ena)
{
    if (!ant_ena[0]) return 0; // CH1 must be enabled
    int changed = 0;
    for (int i = 0; i < array->nrfch; i++) {
        if (array->ant_ena[i] != (ant_ena[i] != 0)) changed = 1;
        for (int j = 0; j < 3; j++) {
            if (array->ant_pos[i][j] != ant_pos[i*3+j]) changed = 1;
        }
    }
    if (!changed) return 1;

    matcpy(&array->ant_pos[0][0], ant_pos, array->nrfch * 3, 1);
    for (int i = 0; i < array->nrfch; i++) {
        array->ant_ena[i] = ant_ena[i] ? 1 : 0;
    }
    if (array->calib_alg == SDR_CALIB_ALG_STATIC &&
        array->calib_mode == SDR_CALIB_RPY) array->calib_run = 0;
    reset_state(array);
    static_cal_free(array->static_cal);
    array->static_cal = array->calib_alg == SDR_CALIB_ALG_STATIC &&
        array->calib_run ? static_cal_new() : NULL;
    return 1;
}

// reset antenna array states --------------------------------------------------
static void reset_state(sdr_array_t *array)
{
    memset(array->x, 0, sizeof(array->x));
    memset(array->P, 0, sizeof(array->P));
    array->nep = 0;
    array->rms = 0.0;
    array->calib_valid = 0;
    array->calib_source = SDR_CALIB_SRC_NONE;
}

// run control of antenna array calibration ------------------------------------
int sdr_array_run(sdr_array_t *array, int run)
{
    if (array->nrfch < 2) return 0;
    if (run != 0 && run != 1 && run != 2) return 0;
    if (run == 1 && array->calib_alg == SDR_CALIB_ALG_STATIC) {
        if (array->calib_mode == SDR_CALIB_RPY && !array->calib_valid)
            return 0;
        if (!array->static_cal) array->static_cal = static_cal_new();
        double bias[SDR_MAX_RFCH] = {0};
        if (array->calib_mode == SDR_CALIB_RPY) {
            for (int k = 1; k < array->nrfch; k++) {
                bias[k] = array->x[3+k];
            }
        }
        static_cal_clear(array->static_cal);
        matcpy(array->static_cal->seed_bias, bias, SDR_MAX_RFCH, 1);
    }
    if (run == 1 || run == 2) reset_state(array);
    if (run == 2 || (run == 1 &&
        array->calib_alg != SDR_CALIB_ALG_STATIC)) {
        static_cal_free(array->static_cal);
        array->static_cal = NULL;
    }
    if (run == 0 || run == 1) array->calib_run = run;
    if (run == 2) array->calib_run = 0;
    return 1;
}

// set calibration mode (SDR_CALIB_BOTH / BIAS / RPY) ---------------------------
int sdr_array_set_mode(sdr_array_t *array, int mode)
{
    if (mode != SDR_CALIB_BOTH && mode != SDR_CALIB_BIAS && mode != SDR_CALIB_RPY)
        return 0;
    if (array->calib_run && array->calib_alg == SDR_CALIB_ALG_STATIC &&
        array->calib_mode != mode) return 0;
    array->calib_mode = mode;
    return 1;
}

// select a supported calibration algorithm ------------------------------------
int sdr_array_set_alg(sdr_array_t *array, int alg)
{
    if (!array || array->calib_run ||
        (alg != SDR_CALIB_ALG_CONTINUOUS && alg != SDR_CALIB_ALG_STATIC))
        return 0;
    array->calib_alg = alg;
    return 1;
}

// calibrate antenna array -----------------------------------------------------
static void calib_continuous(sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr)
{
    if (!array->calib_run || nobs <= 0 || norm(rr, 3) < 1e-6) return;
    int pairs = 0;
    for (int i = 0; i < nobs; i++) {
        if (obs[i].rcv != 1 || obs[i].L[array->freq] == 0.0) continue;
        for (int j = 0; j < nobs; j++) {
            if (obs[j].sat == obs[i].sat && obs[j].rcv >= 2 &&
                obs[j].rcv <= array->nrfch && obs[j].L[array->freq] != 0.0) pairs++;
        }
    }
    if (!calib_diag_done && pairs != calib_diag_last_pairs) fprintf(stderr,
        "calib_try: nobs=%d pairs=%d rr_norm=%.3f mode=%d ena8=%d\n",
        nobs, pairs, norm(rr, 3), array->calib_mode, array->ant_ena[7]);
    calib_diag_last_pairs = pairs;
    calib_diag_active = calib_diag_successes < 3 && pairs >= 38;
    if (calib_diag_active) {
        fprintf(stderr, "calibration epoch: run=%d mode=%d nrfch=%d freq=%d nobs=%d rr=%.4f,%.4f,%.4f nav_n=%d nav_ng=%d\n",
            array->calib_run, array->calib_mode, array->nrfch, array->freq,
            nobs, rr[0], rr[1], rr[2], nav->n, nav->ng);
        for (int i = 0; i < array->nrfch; i++) fprintf(stderr,
            "  antenna ch=%d ena=%d pos=%.6f,%.6f,%.6f\n", i + 1,
            array->ant_ena[i], array->ant_pos[i][0], array->ant_pos[i][1],
            array->ant_pos[i][2]);
        for (int i = 0; i < nobs; i++) {
            char sat[16];
            satno2id(obs[i].sat, sat);
            fprintf(stderr, "  obs sat=%s ch=%d code=%s L=%.9f P=%.3f LLI=%d\n",
                sat, obs[i].rcv, code2obs(obs[i].code[array->freq]),
                obs[i].L[array->freq], obs[i].P[array->freq],
                obs[i].LLI[array->freq]);
        }
        double pos[3];
        ecef2pos(rr, pos);
        for (int i = 0; i < nobs; i++) {
            if (obs[i].rcv != 1) continue;
            char sat[16];
            double rs[6], dts[2], var, e[3], azel[2] = {0};
            int svh = -1, week, raw_pairs = 0;
            satno2id(obs[i].sat, sat);
            int sp = satpos(obs[i].time, obs[i].time, obs[i].sat,
                EPHOPT_BRDC, nav, rs, dts, &var, &svh);
            double dist = sp ? geodist(rs, rr, e) : 0.0;
            if (dist > 0.0) satazel(pos, e, azel);
            double tow = time2gpst(obs[i].time, &week);
            for (int j = 0; j < nobs; j++) {
                if (obs[j].sat == obs[i].sat && obs[j].rcv >= 2 &&
                    obs[j].L[array->freq] != 0.0) raw_pairs++;
            }
            fprintf(stderr, "  ref sat=%s tow=%.3f week=%d satpos=%d svh=%d dist=%.3f el=%.3f freq=%.3f raw_pairs=%d\n",
                sat, tow, week, sp, svh, dist,
                azel[1] * R2D, sat2freq(obs[i].sat,
                obs[i].code[array->freq], nav), raw_pairs);
        }
    }
    
    // initialized if any diagonal of P is set; covers BOTH (P[0]),
    // BIAS (P[3+3*NX], rpy frozen at 0), RPY (P[0], bias frozen)
    if (array->P[0] == 0.0 && array->P[3+3*NX] == 0.0) {
        kf_init(array, obs, nobs, nav, rr);
        if (calib_diag_active && array->rms < 1e3) calib_diag_successes++;
    } else {
        kf_update(array, obs, nobs, nav, rr);
    }
    calib_diag_active = 0;
}

// drop the oldest retained static epoch ---------------------------------------
static void static_evict(sdr_calib_static_t *cal)
{
    if (!cal->nepoch) return;
    const sdr_calib_epoch_t *ep = cal->epochs + cal->ep_first;
    cal->first = (cal->first + ep->count) % STATIC_MAX_ROWS;
    cal->nmeas -= ep->count;
    cal->ep_first = (cal->ep_first + 1) % STATIC_MAX_EPOCHS;
    cal->nepoch--;
}

// refresh window diversity and duration statistics ----------------------------
static void static_window_stat(sdr_calib_static_t *cal)
{
    unsigned char seen[MAXSAT + 1] = {0};
    cal->nsat = 0;
    for (int j = 0; j < cal->nmeas; j++) {
        const sdr_calib_meas_t *m = cal->meas +
            (cal->first + j) % STATIC_MAX_ROWS;
        if (m->sat >= 1 && m->sat <= MAXSAT && !seen[m->sat]) {
            seen[m->sat] = 1;
            cal->nsat++;
        }
    }
    cal->span = cal->nepoch > 1 ? timediff(
        cal->epochs[(cal->ep_first + cal->nepoch - 1) % STATIC_MAX_EPOCHS].time,
        cal->epochs[cal->ep_first].time) : 0.0;
}

// materialize and retain one eligible physical phase epoch --------------------
static int static_collect(sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr)
{
    sdr_calib_static_t *cal = array->static_cal;
    double pos[3];
    int n = 0, f = array->freq;
    if (!cal || !array->ant_ena[0]) return 0;
    ecef2pos(rr, pos);
    for (int i = 0; i < nobs; i++) {
        const obsd_t *p = obs + i;
        double e[3], freq;
        if (p->rcv != 1 || p->L[f] == 0.0 ||
            !los_vec(p->time, p->sat, nav, pos, rr, e) ||
            (freq = sat2freq(p->sat, p->code[f], nav)) <= 0.0) continue;
        double lam = CLIGHT / freq;
        for (int j = 0; j < nobs && n < MAX_NV; j++) {
            const obsd_t *q = obs + j;
            int k = q->rcv - 1;
            if (q->sat != p->sat || k <= 0 || k >= array->nrfch ||
                !array->ant_ena[k] || q->L[f] == 0.0) continue;
            sdr_calib_meas_t *m = cal->pending + n;
            m->time = p->time;
            matcpy(m->los, e, 3, 1);
            m->lambda = lam;
            m->phase_sd = lam * (q->L[f] - p->L[f]);
            m->sat = p->sat;
            m->rfch = k;
            m->epoch = cal->total_epochs + 1;
            if (isfinite(m->phase_sd)) n++;
        }
    }
    if (!n) return 0;
    gtime_t time = cal->pending[0].time;
    if (cal->nepoch) {
        const sdr_calib_epoch_t *last = cal->epochs +
            (cal->ep_first + cal->nepoch - 1) % STATIC_MAX_EPOCHS;
        double dt = timediff(time, last->time);
        if (fabs(dt) <= 1e-3) return 0;
        if (dt < 0.0) static_cal_clear(cal);
    }
    while (cal->nepoch && (cal->nepoch >= STATIC_MAX_EPOCHS ||
        cal->nmeas + n > STATIC_MAX_ROWS ||
        timediff(time, cal->epochs[cal->ep_first].time) > STATIC_MAX_SPAN)) {
        static_evict(cal);
    }
    int first = (cal->first + cal->nmeas) % STATIC_MAX_ROWS;
    for (int i = 0; i < n; i++) {
        cal->pending[i].epoch = cal->total_epochs + 1;
        cal->meas[(first + i) % STATIC_MAX_ROWS] = cal->pending[i];
    }
    sdr_calib_epoch_t *ep = cal->epochs +
        (cal->ep_first + cal->nepoch) % STATIC_MAX_EPOCHS;
    ep->time = time;
    ep->count = n;
    cal->nmeas += n;
    cal->nepoch++;
    cal->total_epochs++;
    static_window_stat(cal);
    return 1;
}

// map the calibration mode and enable mask to physical unknowns ---------------
static int static_active(const sdr_array_t *array, int *state)
{
    int n = 0;
    if (array->calib_mode != SDR_CALIB_BIAS) {
        for (int i = 0; i < 3; i++) state[n++] = i;
    }
    if (array->calib_mode != SDR_CALIB_RPY) {
        for (int k = 1; k < array->nrfch; k++) {
            if (array->ant_ena[k]) state[n++] = 3 + k;
        }
    }
    return n;
}

// rebuild wrapped physical residuals and the active Jacobian ------------------
static int static_build(const sdr_array_t *array, sdr_calib_static_t *cal,
    const int *state, int na, const double *x, double *rms)
{
    double R[9], Dr[9], Dp[9], Dy[9], ss = 0.0;
    euler_rot(x, R, Dr, Dp, Dy);
    for (int j = 0; j < cal->nmeas; j++) {
        const sdr_calib_meas_t *m = cal->meas +
            (cal->first + j) % STATIC_MAX_ROWS;
        double b[3], h[3];
        for (int i = 0; i < 3; i++) {
            b[i] = array->ant_pos[m->rfch][i] - array->ant_pos[0][i];
        }
        double proj = sd_proj(m->los, b, R, Dr, Dp, Dy, h);
        double v = m->phase_sd - (proj - x[3+m->rfch]);
        v -= m->lambda * floor(v / m->lambda + 0.5);
        if (!isfinite(v)) return 0;
        cal->residual[j] = v;
        ss += v * v;
        for (int c = 0; c < na; c++) {
            cal->work[j*na+c] = state[c] < 3 ? h[state[c]] :
                (state[c] == 3 + m->rfch ? -1.0 : 0.0);
        }
    }
    if (rms) *rms = sqrt(ss / cal->nmeas);
    return 1;
}

// solve and condition-check the same normalized system by pivoted QR ---------
static int static_qr(sdr_calib_static_t *cal, int m, int n, double *step)
{
    double scale[NX], y[NX] = {0}, rmax = 0.0;
    int perm[NX];
    double *A = cal->work, *v = cal->residual;
    if (m <= n || n < 1 || n > NX) return 0;
    for (int c = 0; c < n; c++) {
        double norm_col = 0.0;
        for (int i = 0; i < m; i++) {
            if (!isfinite(A[i*n+c])) return 0;
            norm_col = hypot(norm_col, A[i*n+c]);
        }
        if (!(norm_col > 0.0) || !isfinite(norm_col)) return 0;
        scale[c] = norm_col;
        perm[c] = c;
        for (int i = 0; i < m; i++) A[i*n+c] /= norm_col;
    }
    for (int k = 0; k < n; k++) {
        int pivot = k;
        double best = -1.0;
        for (int c = k; c < n; c++) {
            double norm_col = 0.0;
            for (int i = k; i < m; i++) {
                norm_col = hypot(norm_col, A[i*n+c]);
            }
            if (norm_col > best) {
                best = norm_col;
                pivot = c;
            }
        }
        if (!(best > 0.0) || !isfinite(best)) return 0;
        if (pivot != k) {
            for (int i = 0; i < m; i++) {
                double tmp = A[i*n+k];
                A[i*n+k] = A[i*n+pivot];
                A[i*n+pivot] = tmp;
            }
            int tmp = perm[k];
            perm[k] = perm[pivot];
            perm[pivot] = tmp;
        }
        double x0 = A[k*n+k];
        double alpha = -copysign(best, x0);
        double den = x0 - alpha;
        double tau = (alpha - x0) / alpha;
        A[k*n+k] = alpha;
        for (int i = k + 1; i < m; i++) A[i*n+k] /= den;
        for (int c = k + 1; c < n; c++) {
            double dotv = A[k*n+c];
            for (int i = k + 1; i < m; i++) {
                dotv += A[i*n+k] * A[i*n+c];
            }
            dotv *= tau;
            A[k*n+c] -= dotv;
            for (int i = k + 1; i < m; i++) {
                A[i*n+c] -= A[i*n+k] * dotv;
            }
        }
        double dotv = v[k];
        for (int i = k + 1; i < m; i++) dotv += A[i*n+k] * v[i];
        dotv *= tau;
        v[k] -= dotv;
        for (int i = k + 1; i < m; i++) v[i] -= A[i*n+k] * dotv;
        rmax = fmax(rmax, fabs(alpha));
    }
    double tol = 64.0 * DBL_EPSILON * (m > n ? m : n) * rmax;
    double rnorm = 0.0, invnorm = 0.0;
    for (int c = 0; c < n; c++) {
        if (fabs(A[c*n+c]) <= tol) return 0;
        double sum = 0.0;
        for (int i = 0; i <= c; i++) sum += fabs(A[i*n+c]);
        rnorm = fmax(rnorm, sum);
    }
    for (int c = 0; c < n; c++) {
        double z[NX] = {0};
        for (int i = n - 1; i >= 0; i--) {
            double rhs = i == c ? 1.0 : 0.0;
            for (int j = i + 1; j < n; j++) rhs -= A[i*n+j] * z[j];
            z[i] = rhs / A[i*n+i];
        }
        double sum = 0.0;
        for (int i = 0; i < n; i++) sum += fabs(z[i]);
        invnorm = fmax(invnorm, sum);
    }
    if (!isfinite(rnorm * invnorm) || rnorm * invnorm > STATIC_MAX_COND)
        return 0;
    for (int i = n - 1; i >= 0; i--) {
        double rhs = v[i];
        for (int j = i + 1; j < n; j++) rhs -= A[i*n+j] * y[j];
        y[i] = rhs / A[i*n+i];
    }
    for (int i = 0; i < n; i++) {
        step[perm[i]] = y[i] / scale[perm[i]];
        if (!isfinite(step[perm[i]])) return 0;
    }
    return 1;
}

// solve the bounded static window without changing the published array state -
static void static_solve(sdr_array_t *array)
{
    sdr_calib_static_t *cal = array->static_cal;
    int state[NX], na = static_active(array, state);
    if (cal->nepoch < STATIC_MIN_EPOCHS || cal->span < STATIC_MIN_SPAN ||
        cal->nmeas <= na) return;
    double best[NX] = {0}, best_rms = 1e3;
    int starts = array->calib_mode == SDR_CALIB_BIAS ? 1 :
        (int)ceil(360.0 / YAW_STEP);
    for (int s = 0; s < starts; s++) {
        double x[NX] = {0}, step[NX] = {0};
        if (array->calib_mode != SDR_CALIB_BIAS) {
            x[2] = -PI + s * YAW_STEP * D2R;
        }
        if (array->calib_mode == SDR_CALIB_RPY) {
            for (int k = 1; k < array->nrfch; k++) {
                x[3+k] = cal->seed_bias[k];
            }
        }
        for (int iter = 0; iter < MAX_ITER; iter++) {
            if (!static_build(array, cal, state, na, x, NULL) ||
                !static_qr(cal, cal->nmeas, na, step)) break;
            for (int c = 0; c < na; c++) x[state[c]] += step[c];
            double da = 0.0, db = 0.0;
            for (int c = 0; c < na; c++) {
                if (state[c] < 3) da += step[c] * step[c];
                else db += step[c] * step[c];
            }
            if (sqrt(da) >= CONV_THRES_A || sqrt(db) >= CONV_THRES_B)
                continue;
            x[2] -= DPI * floor(x[2] / DPI + 0.5);
            double rms = 0.0, check[NX] = {0};
            if (static_build(array, cal, state, na, x, &rms) &&
                static_qr(cal, cal->nmeas, na, check) && rms < best_rms) {
                matcpy(best, x, NX, 1);
                best_rms = rms;
            }
            break;
        }
    }
    cal->last_rms_valid = best_rms < 1e3;
    if (cal->last_rms_valid) cal->last_rms = best_rms;
    if (!cal->last_rms_valid || best_rms > MAX_RMS) return;
    matcpy(array->x, best, NX, 1);
    array->rms = best_rms;
    array->calib_valid = 1;
    array->calib_source = SDR_CALIB_SRC_STATIC;
    array->calib_run = 0;
}

// collect and solve one static calibration epoch -------------------------------
static void calib_static(sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr)
{
    if (!array->calib_run || !obs || !nav || !rr || nobs <= 0 ||
        norm(rr, 3) < 1e-6) return;
    if (static_collect(array, obs, nobs, nav, rr)) static_solve(array);
}

// dispatch the selected array calibration algorithm ---------------------------
void sdr_array_calib(sdr_array_t *array, const obsd_t *obs, int nobs,
    const nav_t *nav, const double *rr)
{
    if (array->calib_alg == SDR_CALIB_ALG_CONTINUOUS) {
        calib_continuous(array, obs, nobs, nav, rr);
    } else if (array->calib_alg == SDR_CALIB_ALG_STATIC) {
        calib_static(array, obs, nobs, nav, rr);
    }
}

// set antenna array states ----------------------------------------------------
int sdr_array_set(sdr_array_t *array, const double *rpy, const double *bias)
{
    if (array->nrfch < 2) return 0;
    if (array->calib_alg == SDR_CALIB_ALG_STATIC) array->calib_run = 0;
    reset_state(array);
    static_cal_free(array->static_cal);
    array->static_cal = NULL;
    matcpy(array->x, rpy, 3, 1);
    matcpy(array->x + 3, bias, array->nrfch, 1);
    for (int i = 0; i < NX; i++) {
        array->P[i+i*NX] = (i < 3) ? 1e-2 : 1e-4;
    }
    array->calib_valid = 1;
    array->calib_source = SDR_CALIB_SRC_EXTERNAL;
    return 1;
}

// get antenna array states ----------------------------------------------------
int sdr_array_stat(sdr_array_t *array, double *rpy, double *bias, double *rms,
    int *nep)
{
    matcpy(rpy, array->x, 3, 1);
    bias[0] = 0.0;
    matcpy(bias + 1, array->x + 4, array->nrfch - 1, 1);
    *rms = array->rms;
    *nep = array->nep;
    return array->calib_run;
}

// copy calibration lifecycle and collection status ----------------------------
int sdr_array_get_status(const sdr_array_t *array, sdr_array_status_t *status)
{
    if (!array || !status) return 0;
    memset(status, 0, sizeof(*status));
    status->run = array->calib_run;
    status->mode = array->calib_mode;
    status->alg = array->calib_alg;
    status->valid = array->calib_valid;
    status->source = array->calib_source;
    status->nep = array->nep;
    status->rms = array->rms;
    if (array->static_cal) {
        const sdr_calib_static_t *cal = array->static_cal;
        status->static_total_epochs = cal->total_epochs;
        status->static_window_epochs = cal->nepoch;
        status->static_meas = cal->nmeas;
        status->static_sats = cal->nsat;
        status->static_span = cal->span;
        status->static_last_rms = cal->last_rms;
        status->static_last_rms_valid = cal->last_rms_valid;
    }
    return 1;
}

// save calibration state to file ----------------------------------------------
int sdr_array_save(sdr_array_t *array, const char *file)
{
    double rpy[3], bias[SDR_MAX_RFCH], rms;
    int nep;
    
    sdr_array_stat(array, rpy, bias, &rms, &nep);
    if (!array->calib_valid) return 0;
    
    FILE *fp = fopen(file, "w");
    if (!fp) return 0;
    fprintf(fp, "# Pocket SDR Array Calibration\n");
    fprintf(fp, "# epochs=%d rms=%.4fm freq=%d\n", nep, rms, array->freq);
    if (array->calib_source == SDR_CALIB_SRC_STATIC && array->static_cal) {
        fprintf(fp, "# algorithm=static static_epochs=%d static_meas=%d\n",
            array->static_cal->nepoch, array->static_cal->nmeas);
    }
    fprintf(fp, "# Roll Pitch Yaw (deg)\n");
    fprintf(fp, "%.3f %.3f %.3f\n", rpy[0] * R2D, rpy[1] * R2D, rpy[2] * R2D);
    fprintf(fp, "# Bias CH1..CHn (m)\n");
    for (int i = 0; i < array->nrfch; i++) {
        fprintf(fp, "%s%.4f", i ? " " : "", bias[i]);
    }
    fprintf(fp, "\n");
    fclose(fp);
    return 1;
}

// load calibration state from file --------------------------------------------
int sdr_array_load(sdr_array_t *array, const char *file)
{
    FILE *fp = fopen(file, "r");
    if (!fp) return 0;
    
    double rpy[3] = {0}, bias[SDR_MAX_RFCH] = {0};
    char buff[1024];
    int line = 0;
    while (line < 2 && fgets(buff, sizeof(buff), fp)) {
        char *p = strchr(buff, '#');
        if (p) *p = '\0';
        for (p = buff; *p && (*p == ' ' || *p == '\t'); p++);
        if (!*p || *p == '\n' || *p == '\r') continue;
        if (line == 0) {
            if (sscanf(buff, "%lf %lf %lf", rpy, rpy + 1, rpy + 2) < 3) {
                fclose(fp);
                return 0;
            }
            for (int i = 0; i < 3; i++) rpy[i] *= D2R;
        } else {
            int n = 0;
            for (char *s = buff, *e; n < array->nrfch; s = e) {
                bias[n] = strtod(s, &e);
                if (e == s) break;
                n++;
            }
            if (n < 1) { fclose(fp); return 0; }
        }
        line++;
    }
    fclose(fp);
    if (line < 2) return 0;
    if (!sdr_array_set(array, rpy, bias)) return 0;
    array->calib_run = 0;
    array->calib_source = SDR_CALIB_SRC_LOADED;
    return 1;
}

// save antenna geometry to file -----------------------------------------------
int sdr_array_geom_save(const char *file, const double *ant_pos, int n)
{
    FILE *fp = fopen(file, "w");
    if (!fp) return 0;
    
    fprintf(fp, "# Pocket SDR Array Geometry in body-frame, m)\n");
    for (int i = 0; i < n; i++) {
        fprintf(fp, "%.6f %.6f %.6f\n", ant_pos[i*3], ant_pos[i*3+1], ant_pos[i*3+2]);
    }
    fclose(fp);
    return n;
}

// load antenna geometry from file ---------------------------------------------
int sdr_array_geom_load(const char *file, double *ant_pos, int max_ant)
{
    FILE *fp = fopen(file, "r");
    if (!fp) return 0;
    
    char buff[256];
    int n = 0;
    while (n < max_ant && fgets(buff, sizeof(buff), fp)) {
        char *p = strchr(buff, '#');
        if (p) *p = '\0';
        double pos[3];
        if (sscanf(buff, "%lf %lf %lf", pos, pos + 1, pos + 2) == 3) {
            matcpy(ant_pos + n * 3, pos, 3, 1);
            n++;
        }
    }
    fclose(fp);
    return n;
}

// free array CH beam state -----------------------------------------------------
void sdr_arch_free(sdr_arch_t *arch)
{
    arch->az = arch->el = arch->scale = 0.0;
    memset(arch->w, 0, sizeof(arch->w));
}

// check if RF CH is tunable to L1 ---------------------------------------------
static int is_l1_ch(const sdr_rcv_t *rcv, int i)
{
    double f_eff = rcv->rfch[i].fo;
    if (rcv->rfch[i].IQ == 1) f_eff += rcv->fs * 0.25;
    return fabs(f_eff - ARRAY_FREQ) <= ARRAY_FREQ_TOL;
}

// bit-range expansion gain ----------------------------------------------------
static double bit_gain(const sdr_rcv_t *rcv, const int *ant_ena)
{
    int bits_min = 4;
    
    for (int i = 0; i < rcv->nrfch; i++) {
        if (!is_l1_ch(rcv, i) || !ant_ena[i]) continue;
        if (rcv->rfch[i].bits > 0 && rcv->rfch[i].bits < bits_min) {
            bits_min = rcv->rfch[i].bits;
        }
    }
    return (bits_min < 4) ? 7.0 / (double)((1 << bits_min) - 1) : 1.0;
}

// beam steering vector in body frame ------------------------------------------
static void steer_body(const double *rpy, double az, double el, double *e_body)
{
    double R[9], e_enu[3];
    euler_rot(rpy, R, NULL, NULL, NULL);
    e_enu[0] = sin(az) * cos(el);
    e_enu[1] = cos(az) * cos(el);
    e_enu[2] = sin(el);
    matmul("TN", 3, 1, 3, 1.0, R, e_enu, 0.0, e_body);
}

// generate array response for a candidate sky direction ----------------------
void sdr_array_steering(const sdr_array_t *array, double az, double el,
    double freq, sdr_cpx_t *a)
{
    double e_body[3], lam = CLIGHT / freq;
    steer_body(array->x, az, el, e_body);
    for (int i = 0; i < array->nrfch; i++) {
        double b[3], phi;
        for (int j = 0; j < 3; j++) {
            b[j] = array->ant_pos[i][j] - array->ant_pos[0][j];
        }
        phi = DPI * (dot(e_body, b, 3) + array->x[3+i]) / lam;
        a[i][0] = array->ant_ena[i] ? (float)cos(phi) : 0.0f;
        a[i][1] = array->ant_ena[i] ? (float)sin(phi) : 0.0f;
    }
}

// build LUT for weight multiplication -----------------------------------------
static void build_lut(sdr_arch_t *arch, int nrfch)
{
    for (int j = 0; j < nrfch; j++) {
        int32_t wr = arch->w[j*2], wi = arch->w[j*2+1];
        for (int b = 0; b < 256; b++) {
            int32_t I = SDR_CPX8_I((sdr_cpx8_t)b);
            int32_t Q = SDR_CPX8_Q((sdr_cpx8_t)b);
            arch->LUT[j][b].I = I * wr - Q * wi;
            arch->LUT[j][b].Q = I * wi + Q * wr;
        }
    }
}

// set array CH beam -----------------------------------------------------------
void sdr_arch_set_beam(sdr_arch_t *arch, const sdr_rcv_t *rcv, double az,
    double el, double scale)
{
    arch->az = az;
    arch->el = el;
    const sdr_array_t *array = rcv->array;
    
    int nant = 0; // effective antenna count
    if (array && scale > 0.0) {
        for (int i = 0; i < rcv->nrfch; i++) {
            if (is_l1_ch(rcv, i) && array->ant_ena[i]) nant++;
        }
    }
    scale = (nant > 0) ? 1.0 / nant : 0.0;
    arch->scale = scale;
    memset(arch->w, 0, sizeof(arch->w));
    if (scale <= 0.0) {
        build_lut(arch, rcv->nrfch); // zero LUT
        return;
    }
    double amp = scale * bit_gain(rcv, array->ant_ena) * ARRAY_W_SCALE;
    sdr_cpx_t a[SDR_MAX_RFCH];
    sdr_array_steering(array, az, el, ARRAY_FREQ, a);
    
    for (int i = 0; i < rcv->nrfch; i++) {
        if (!is_l1_ch(rcv, i) || !array->ant_ena[i]) continue;
        
        double wr = amp * a[i][0], wi = -amp * a[i][1];
        
        arch->w[i*2  ] = (int16_t)floor(CLIP(wr, -32768.0, 32767.0) + 0.5);
        arch->w[i*2+1] = (int16_t)floor(CLIP(wi, -32768.0, 32767.0) + 0.5);
    }
    // build LUT for weight multiplication
    build_lut(arch, rcv->nrfch);
}

// get array CH beam ------------------------------------------------------------
void sdr_arch_get_beam(const sdr_arch_t *arch, double *az, double *el)
{
    *az = arch->az;
    *el = arch->el;
}

// combine RF CHs into array CH -------------------------------------------------
void sdr_arch_combine(const sdr_arch_t *arch, const sdr_rcv_t *rcv, int base)
{
    if (arch->scale <= 0.0) return;
    int nrfch = rcv->nrfch, N = rcv->N;
    int m = (int)(arch - rcv->arch);
    if (m < 0 || m >= rcv->narch || nrfch < 2) return;
    
    sdr_cpx8_t *out = rcv->buff[nrfch+m]->data + base;
    const sdr_cpx8_t *in[SDR_MAX_RFCH];
    
    for (int i = 0; i < nrfch; i++) {
        in[i] = rcv->buff[i]->data + base;
    }
    for (int i = 0; i < N; i++) {
        int32_t I = 0, Q = 0;
        for (int j = 0; j < nrfch; j++) {
            I += arch->LUT[j][in[j][i]].I;
            Q += arch->LUT[j][in[j][i]].Q;
        }
        I += (I >= 0) ? ARRAY_W_SCALE / 2 : -ARRAY_W_SCALE / 2;
        Q += (Q >= 0) ? ARRAY_W_SCALE / 2 : -ARRAY_W_SCALE / 2;
        I /= ARRAY_W_SCALE;
        Q /= ARRAY_W_SCALE;
        out[i] = SDR_CPX8(CLIP(I, -8, 7), CLIP(Q, -8, 7));
    }
}
