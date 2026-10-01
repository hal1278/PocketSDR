// Phase 1 held-out STATIC calibration and Bartlett/LOS replay measurement.
#include "pocket_sdr.h"

#define CAL_END 239.0
#define REF_START 260.0
#define REF_END 270.0
#define MEAS_END 340.0

// read the receiver's current IF cycle ---------------------------------------
static double replay_time(sdr_rcv_t *rcv)
{
    sdr_mutex_lock(&rcv->mtx);
    double time = rcv->ix * SDR_CYC;
    sdr_mutex_unlock(&rcv->mtx);
    return time;
}

// check a satellite's current PVT and ephemeris status -----------------------
static int valid_los(sdr_rcv_t *rcv, const char *sat)
{
    char line[256], id[16];
    double az, el;
    int pvt, obs, eph, svh, fcn;
    if (!sdr_rcv_sat_stat(rcv, sat, line, sizeof(line))) return 0;
    if (sscanf(line, "%15s %lf %lf %d %d %d %d %d", id, &az, &el,
        &pvt, &obs, &eph, &svh, &fcn) != 8) return 0;
    return pvt && eph && el >= 0.0;
}

// select one fixed physical-RF L1 C/A reference with the best C/N0 -----------
static sdr_ch_t *select_reference(sdr_rcv_t *rcv)
{
    sdr_ch_t *best = NULL;
    double best_cn0 = -1e9;
    for (int i = 0; i < rcv->nch; i++) {
        sdr_ch_t *ch = rcv->th[i]->ch;
        char sat[16];
        double cn0;
        int eligible;
        sdr_mutex_lock(&ch->mtx);
        eligible = ch->state == SDR_STATE_LOCK && ch->rf_ch >= 0 &&
            ch->rf_ch < 7 && !strcmp(ch->sig, "L1CA") &&
            ch->lock * ch->T >= 2.0 && ch->trk->wrap_pol_valid;
        cn0 = ch->cn0;
        snprintf(sat, sizeof(sat), "%s", ch->sat);
        sdr_mutex_unlock(&ch->mtx);
        if (eligible && cn0 > best_cn0 && valid_los(rcv, sat)) {
            best = ch;
            best_cn0 = cn0;
        }
    }
    return best;
}

// find the dominant cell of one published Bartlett map -----------------------
static void dominant_peak(const sdr_spatial_map_t *map, double *az, double *el)
{
    int best = 0;
    for (int i = 1; i < map->naz * map->nel; i++) {
        if (map->power[i] > map->power[best]) best = i;
    }
    *az = map->az0 + (best % map->naz) * map->daz;
    *el = map->el0 + (best / map->naz) * map->del;
}

// compute angular separation on the unit sphere ------------------------------
static double angular_error(double az1, double el1, double az2, double el2)
{
    az1 *= D2R; el1 *= D2R; az2 *= D2R; el2 *= D2R;
    double dot = sin(el1) * sin(el2) + cos(el1) * cos(el2) *
        cos(az1 - az2);
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;
    return acos(dot) * R2D;
}

// sample the predicted LOS at the time a map is published --------------------
static int map_los(sdr_rcv_t *rcv, const sdr_ch_t *ch,
    const sdr_spatial_map_t *map, double *az, double *el, double *age)
{
    if (!valid_los(rcv, ch->sat)) return 0;
    int sat = satid2no(ch->sat);
    if (!sat) return 0;
    sdr_mutex_lock(&rcv->pvt->mtx);
    *az = rcv->pvt->ssat[sat-1].azel[0] * R2D;
    *el = rcv->pvt->ssat[sat-1].azel[1] * R2D;
    *age = fabs(map->time - rcv->pvt->ix * SDR_CYC);
    sdr_mutex_unlock(&rcv->pvt->mtx);
    return *age <= 2.0 && *el >= 0.0;
}

// replay once with bounded calibration and a later held-out LOS segment ------
int main(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "usage: %s input.bin geometry.geom los.csv calibration.txt\n",
            argv[0]);
        return 2;
    }
    const int prns[] = {5, 7, 11, 15, 20, 21, 29, 30};
    const char *sigs[8] = {"L1CA", "L1CA", "L1CA", "L1CA", "L1CA",
        "L1CA", "L1CA", "L1CA"};
    int prn_list[8], types[SDR_MAX_STR] = {0};
    const char *paths[SDR_MAX_STR] = {0};
    double fo[SDR_MAX_RFCH] = {0};
    int IQ[SDR_MAX_RFCH] = {0}, bits[SDR_MAX_RFCH] = {0};
    double ant_pos[SDR_MAX_RFCH * 3] = {0};
    int ant_ena[SDR_MAX_RFCH] = {0};
    for (int i = 0; i < 8; i++) prn_list[i] = prns[i];
    int nant = sdr_array_geom_load(argv[2], ant_pos, SDR_MAX_RFCH);
    if (nant != 7) {
        fprintf(stderr, "expected seven enabled geometry rows; found %d\n",
            nant);
        return 2;
    }
    for (int i = 0; i < nant; i++) ant_ena[i] = 1;
    FILE *csv = fopen(argv[3], "w");
    if (!csv) { perror(argv[3]); return 2; }
    fprintf(csv, "time_s,sat,bb_ch,rf_ch,cn0_dbhz,peak_az_deg,peak_el_deg,"
        "los_az_deg,los_el_deg,los_age_s,error_deg\n");
    sdr_func_init("");
    sdr_rcv_t *rcv = sdr_rcv_open_file(sigs, prn_list, 8, SDR_FMT_RAW32,
        16e6, fo, IQ, bits, 0.0, 1.0, argv[1], types, paths,
        "-RFCH L1CA:1-7");
    if (!rcv) { fclose(csv); return 2; }
    int ok = sdr_rcv_array_ant_pos(rcv, ant_pos, ant_ena) &&
        sdr_rcv_array_set_alg(rcv, SDR_CALIB_ALG_STATIC) &&
        sdr_rcv_array_set_mode(rcv, SDR_CALIB_BOTH) &&
        sdr_rcv_array_run(rcv, 1);
    if (!ok) {
        fprintf(stderr, "failed to start STATIC calibration\n");
        sdr_rcv_close(rcv); fclose(csv); return 2;
    }
    int calib_checked = 0, selected = 0, nmap = 0, last_seq = 0;
    int ret = 0;
    double frozen[3 + SDR_MAX_RFCH];
    sdr_ch_t *reference = NULL;
    while (rcv->state && replay_time(rcv) < MEAS_END) {
        double t = replay_time(rcv);
        if (!calib_checked && t >= CAL_END) {
            sdr_rcv_array_run(rcv, 0);
            sdr_array_status_t status;
            sdr_rcv_array_get_status(rcv, &status);
            calib_checked = 1;
            fprintf(stderr, "calibration t=%.3f valid=%d source=%d run=%d "
                "rms=%.6f epochs=%d measurements=%d\n", t, status.valid,
                status.source, status.run, status.rms,
                status.static_window_epochs, status.static_meas);
            if (!status.valid || status.source != SDR_CALIB_SRC_STATIC ||
                status.run) { ret = 3; break; }
            sdr_mutex_lock(&rcv->mtx);
            memcpy(frozen, rcv->array->x, sizeof(frozen));
            sdr_mutex_unlock(&rcv->mtx);
            if (!sdr_rcv_array_save(rcv, argv[4])) { ret = 3; break; }
        }
        if (calib_checked && !selected && t >= REF_START && t < REF_END) {
            reference = select_reference(rcv);
            if (reference) {
                selected = sdr_spatial_select(rcv->spatial, reference->no,
                    "Bartlett");
                fprintf(stderr, "reference t=%.3f sat=%s bb=%d rf=%d "
                    "cn0=%.1f\n", t, reference->sat, reference->no,
                    reference->rf_ch + 1, reference->cn0);
            }
        }
        if (calib_checked && t >= REF_END && !selected) { ret = 4; break; }
        if (selected && t >= REF_END) {
            sdr_spatial_map_t map;
            if (sdr_spatial_get(rcv->spatial, &map) && map.seq > last_seq) {
                last_seq = map.seq;
                if (map.time >= REF_END && map.time < MEAS_END) {
                    double peak_az, peak_el, los_az, los_el, age;
                    if (map_los(rcv, reference, &map, &los_az, &los_el, &age)) {
                        dominant_peak(&map, &peak_az, &peak_el);
                        double error = angular_error(peak_az, peak_el,
                            los_az, los_el);
                        fprintf(csv, "%.3f,%s,%d,%d,%.2f,%.1f,%.1f,%.6f,"
                            "%.6f,%.3f,%.6f\n", map.time, reference->sat,
                            reference->no, reference->rf_ch + 1, map.cn0,
                            peak_az, peak_el, los_az, los_el, age, error);
                        nmap++;
                    }
                }
            }
        }
        sdr_sleep_msec(20);
    }
    if (!calib_checked && ret == 0) ret = 3;
    if (!selected && ret == 0) ret = 4;
    if (!nmap && ret == 0) ret = 5;
    if (calib_checked && ret != 3) {
        sdr_array_status_t status;
        sdr_rcv_array_get_status(rcv, &status);
        sdr_mutex_lock(&rcv->mtx);
        int frozen_ok = !memcmp(frozen, rcv->array->x, sizeof(frozen));
        sdr_mutex_unlock(&rcv->mtx);
        if (!status.valid || status.source != SDR_CALIB_SRC_STATIC ||
            status.run || !frozen_ok) ret = 6;
        fprintf(stderr, "frozen=%d final_valid=%d final_source=%d "
            "final_run=%d maps=%d\n", frozen_ok, status.valid,
            status.source, status.run, nmap);
    }
    sdr_rcv_close(rcv);
    fclose(csv);
    return ret;
}
