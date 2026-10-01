// Pocket SDR common-reference spatial snapshots and Bartlett scan.
#include "pocket_sdr.h"

#define MIN_TICK_STEP 5
#define RF_TOL 1.0

struct sdr_spatial_tag {
    sdr_mutex_t mtx;
    int ch, count;
    const sdr_spatial_alg_t *alg;
    sdr_spatial_cfg_t cfg;
    sdr_spatial_map_t map;
    float *sum, *work;
};

// correlate configurable delay taps using one tracking replica --------------
static int snapshot_delays(const sdr_rcv_t *rcv, const sdr_ch_t *ch, int ix,
    int ndelay, double delay_min, double delay_max,
    sdr_spatial_snapshot_t *snap)
{
    if (!rcv || !ch || !snap || !ch->trk || ch->state != SDR_STATE_LOCK ||
        ch->rf_ch < 0 || ch->rf_ch >= rcv->nrfch || rcv->nrfch < 2 ||
        strcmp(ch->sig, "L1CA") || !ch->trk->code || !ch->trk->code_sum ||
        !ch->trk->wrap_pol_valid || abs(ch->trk->wrap_pol) != 1 ||
        ndelay < 1 || ndelay > SDR_SPATIAL_NDELAY) {
        return 0;
    }
    double ref_fo = rcv->rfch[ch->rf_ch].fo;
    for (int i = 0; i < rcv->nrfch; i++) {
        if (!rcv->buff[i] || fabs(rcv->rfch[i].fo - ref_fo) > RF_TOL ||
            rcv->rfch[i].IQ != rcv->rfch[ch->rf_ch].IQ) return 0;
    }
    memset(snap, 0, sizeof(*snap));
    snap->time = ch->time;
    snap->freq = ch->fc;
    snap->cn0 = ch->cn0;
    snap->ch = ch->no;
    snprintf(snap->sat, sizeof(snap->sat), "%s", ch->sat);
    snprintf(snap->sig, sizeof(snap->sig), "%s", ch->sig);
    snap->nant = rcv->nrfch;
    snap->ndelay = ndelay;
    double pos[SDR_SPATIAL_NDELAY];
    double samples_per_chip = ch->fs * ch->T / ch->len_code;
    for (int d = 0; d < snap->ndelay; d++) {
        snap->delay[d] = ndelay == 1 ? delay_min : delay_min +
            (delay_max - delay_min) * d / (ndelay - 1);
        pos[d] = snap->delay[d] * samples_per_chip;
    }
    for (int i = 0; i < snap->nant; i++) {
        sdr_cpx_t C[2];
        sdr_corr_std(rcv->buff[i], ix, ch->N, ch->fs, ch->fi + ch->fd,
            ch->phi, ch->trk->code, ch->trk->code_sum, NULL, NULL,
            ch->trk->code_scale, ch->coff * ch->fs, pos, snap->ndelay,
            ch->trk->wrap_pol,
            snap->corr[i], C);
    }
    return 1;
}

// extract the default 21-tap common-reference snapshot -----------------------
int sdr_spatial_snapshot(const sdr_rcv_t *rcv, const sdr_ch_t *ch, int ix,
    sdr_spatial_snapshot_t *snap)
{
    return snapshot_delays(rcv, ch, ix, 21, -1.0, 1.0, snap);
}

// scan each delay with conventional array beamforming -------------------------
int sdr_spatial_bartlett(const sdr_spatial_snapshot_t *snap,
    const sdr_array_t *array, const sdr_spatial_grid_t *grid, float *power)
{
    if (!snap || !array || !grid || !power || snap->nant != array->nrfch ||
        snap->ndelay < 1 || snap->ndelay > SDR_SPATIAL_NDELAY ||
        grid->naz < 1 || grid->nel < 1 ||
        grid->naz > SDR_SPATIAL_NAZ || grid->nel > SDR_SPATIAL_NEL ||
        !isfinite(snap->freq) || snap->freq <= 0.0) return 0;
    int nant = 0;
    for (int i = 0; i < snap->nant; i++) nant += array->ant_ena[i] != 0;
    if (nant < 2) return 0;
    for (int el = 0; el < grid->nel; el++) {
        for (int az = 0; az < grid->naz; az++) {
            sdr_cpx_t a[SDR_MAX_RFCH];
            int cell = el * grid->naz + az;
            sdr_array_steering(array, (grid->az0 + az * grid->daz) * D2R,
                (grid->el0 + el * grid->del) * D2R, snap->freq, a);
            for (int d = 0; d < snap->ndelay; d++) {
                double re = 0.0, im = 0.0;
                for (int i = 0; i < snap->nant; i++) {
                    re += a[i][0] * snap->corr[i][d][0] +
                        a[i][1] * snap->corr[i][d][1];
                    im += a[i][0] * snap->corr[i][d][1] -
                        a[i][1] * snap->corr[i][d][0];
                }
                power[cell * snap->ndelay + d] =
                    (float)((re * re + im * im) / (nant * nant));
            }
        }
    }
    return 1;
}

// resolve the current spatial algorithm --------------------------------------
const sdr_spatial_alg_t *sdr_spatial_algorithm(const char *name)
{
    static const sdr_spatial_alg_t alg = {"Bartlett", SDR_SPATIAL_CAP_CORR,
        sdr_spatial_bartlett};
    return name && !strcmp(name, alg.name) ? &alg : NULL;
}

// create fixed-size spatial processor ----------------------------------------
sdr_spatial_t *sdr_spatial_new(void)
{
    sdr_spatial_t *sp = (sdr_spatial_t *)sdr_malloc(sizeof(*sp));
    sp->cfg.grid.naz = SDR_SPATIAL_NAZ;
    sp->cfg.grid.nel = SDR_SPATIAL_NEL;
    sp->cfg.grid.daz = sp->cfg.grid.del = 5.0;
    sp->cfg.ndelay = 21;
    sp->cfg.delay_min = -1.0;
    sp->cfg.delay_max = 1.0;
    sp->cfg.sample_step = 20;
    sp->cfg.average_count = 10;
    sp->alg = sdr_spatial_algorithm("Bartlett");
    size_t n = SDR_SPATIAL_NAZ * SDR_SPATIAL_NEL * SDR_SPATIAL_NDELAY;
    sp->sum = (float *)sdr_malloc(n * sizeof(float));
    sp->work = (float *)sdr_malloc(n * sizeof(float));
    sdr_mutex_init(&sp->mtx);
    return sp;
}

// configure the spatial grid, delays, and update cadence ---------------------
int sdr_spatial_config(sdr_spatial_t *sp, const sdr_spatial_cfg_t *cfg)
{
    if (!sp || !cfg || cfg->grid.naz < 1 || cfg->grid.nel < 2 ||
        cfg->grid.naz > SDR_SPATIAL_NAZ ||
        cfg->grid.nel > SDR_SPATIAL_NEL ||
        !isfinite(cfg->grid.az0) || !isfinite(cfg->grid.el0) ||
        !isfinite(cfg->grid.daz) || !isfinite(cfg->grid.del) ||
        cfg->grid.daz <= 0.0 || cfg->grid.del <= 0.0 ||
        !isfinite(cfg->delay_min) || !isfinite(cfg->delay_max) ||
        cfg->ndelay < 1 ||
        cfg->ndelay > SDR_SPATIAL_NDELAY ||
        cfg->delay_min > cfg->delay_max || cfg->delay_min < -10.0 ||
        cfg->delay_max > 10.0 ||
        cfg->sample_step < MIN_TICK_STEP ||
        cfg->sample_step % MIN_TICK_STEP ||
        cfg->average_count < 1 || cfg->average_count > 100) return 0;
    sdr_mutex_lock(&sp->mtx);
    sp->cfg = *cfg;
    sp->count = 0;
    sp->map.seq = 0;
    memset(sp->sum, 0, sizeof(float) * SDR_SPATIAL_NAZ * SDR_SPATIAL_NEL *
        SDR_SPATIAL_NDELAY);
    sdr_mutex_unlock(&sp->mtx);
    return 1;
}

// release spatial processor --------------------------------------------------
void sdr_spatial_free(sdr_spatial_t *sp)
{
    if (!sp) return;
    sdr_free(sp->sum);
    sdr_free(sp->work);
    sdr_free(sp);
}

// select the common tracking reference and algorithm -------------------------
int sdr_spatial_select(sdr_spatial_t *sp, int ch, const char *alg)
{
    const sdr_spatial_alg_t *a = sdr_spatial_algorithm(alg);
    if (!sp || ch < 0 || !a) return 0;
    sdr_mutex_lock(&sp->mtx);
    sp->ch = ch;
    sp->alg = a;
    sp->count = 0;
    sp->map.seq = 0;
    memset(sp->sum, 0, sizeof(float) * SDR_SPATIAL_NAZ * SDR_SPATIAL_NEL *
        SDR_SPATIAL_NDELAY);
    sdr_mutex_unlock(&sp->mtx);
    return 1;
}

// accumulate spatial power and publish an azimuth/elevation map --------------
void sdr_spatial_tick(sdr_spatial_t *sp, sdr_rcv_t *rcv,
    const sdr_ch_t *ch, int64_t cycle, int ix)
{
    if (!sp || cycle % MIN_TICK_STEP != 0) return;
    sdr_mutex_lock(&sp->mtx);
    if (cycle % sp->cfg.sample_step || sp->ch != ch->no ||
        ch->lock * ch->T < 2.0 || !rcv->array) {
        sdr_mutex_unlock(&sp->mtx);
        return;
    }
    sdr_spatial_snapshot_t snap;
    if (!snapshot_delays(rcv, ch, ix, sp->cfg.ndelay, sp->cfg.delay_min,
        sp->cfg.delay_max, &snap)) {
        sdr_mutex_unlock(&sp->mtx);
        return;
    }
    sdr_array_t array;
    sdr_mutex_lock(&rcv->mtx);
    array = *rcv->array;
    array.static_cal = NULL;
    sdr_mutex_unlock(&rcv->mtx);
    if (sp->alg->process(&snap, &array, &sp->cfg.grid, sp->work)) {
        int n = sp->cfg.grid.naz * sp->cfg.grid.nel * snap.ndelay;
        for (int i = 0; i < n; i++) sp->sum[i] += sp->work[i];
        if (++sp->count >= sp->cfg.average_count) {
            sp->map.ch = snap.ch;
            sp->map.time = snap.time;
            sp->map.cn0 = snap.cn0;
            sp->map.nsnap = sp->count;
            sp->map.naz = sp->cfg.grid.naz;
            sp->map.nel = sp->cfg.grid.nel;
            sp->map.az0 = sp->cfg.grid.az0;
            sp->map.el0 = sp->cfg.grid.el0;
            sp->map.daz = sp->cfg.grid.daz;
            sp->map.del = sp->cfg.grid.del;
            for (int c = 0; c < sp->cfg.grid.naz * sp->cfg.grid.nel; c++) {
                float peak = 0.0f;
                for (int d = 0; d < snap.ndelay; d++) {
                    float p = sp->sum[c * snap.ndelay + d] / sp->count;
                    if (p > peak) peak = p;
                }
                sp->map.power[c] = peak;
            }
            sp->map.seq++;
            sp->count = 0;
            memset(sp->sum, 0, n * sizeof(float));
        }
    }
    sdr_mutex_unlock(&sp->mtx);
}

// copy the latest completed spatial map --------------------------------------
int sdr_spatial_get(sdr_spatial_t *sp, sdr_spatial_map_t *map)
{
    if (!sp || !map) return 0;
    sdr_mutex_lock(&sp->mtx);
    int ok = sp->map.seq > 0;
    if (ok) *map = sp->map;
    sdr_mutex_unlock(&sp->mtx);
    return ok;
}
