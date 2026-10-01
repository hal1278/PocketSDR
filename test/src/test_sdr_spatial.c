// Numerical tests for the shared array model and delay-aware Bartlett scan.
#include "test_sdr.h"

// pack signed I/Q samples for a synthetic spatial snapshot -------------------
static sdr_cpx8_t pack_cpx8(int re, int im)
{
    return (sdr_cpx8_t)((((uint8_t)im & 0x0F) << 4) |
        ((uint8_t)re & 0x0F));
}

// make a three-dimensional eight-element array -------------------------------
static sdr_array_t *make_array(void)
{
    sdr_array_t *array = sdr_array_new(8, 0);
    double pos[SDR_MAX_RFCH * 3];
    int ena[SDR_MAX_RFCH];
    for (int i = 0; i < 8; i++) {
        pos[3*i] = (i & 1) ? 0.19 : 0.0;
        pos[3*i+1] = (i & 2) ? 0.19 : 0.0;
        pos[3*i+2] = (i & 4) ? 0.19 : 0.0;
        ena[i] = 1;
    }
    TEST_ASSERT_TRUE(sdr_array_ant_pos(array, pos, ena));
    return array;
}

// synthesize a spatial component into one code-delay tap ----------------------
static void add_component(sdr_spatial_snapshot_t *snap,
    const sdr_array_t *array, double az, double el, int delay, double amp)
{
    sdr_cpx_t a[SDR_MAX_RFCH];
    sdr_array_steering(array, az * D2R, el * D2R, snap->freq, a);
    for (int i = 0; i < snap->nant; i++) {
        snap->corr[i][delay][0] += (float)(amp * a[i][0]);
        snap->corr[i][delay][1] += (float)(amp * a[i][1]);
    }
}

// find the strongest cell after taking the maximum over delays ---------------
static int best_cell(const float *power, int naz, int nel, int ndelay)
{
    int best = 0;
    float maxp = -1.0f;
    for (int c = 0; c < naz * nel; c++) {
        float p = 0.0f;
        for (int d = 0; d < ndelay; d++) {
            if (power[c * ndelay + d] > p) p = power[c * ndelay + d];
        }
        if (p > maxp) { maxp = p; best = c; }
    }
    return best;
}

// reduce one spatial cell over code delay ------------------------------------
static float map_value(const float *power, int cell, int ndelay)
{
    float peak = 0.0f;
    for (int d = 0; d < ndelay; d++) {
        if (power[cell * ndelay + d] > peak) peak = power[cell * ndelay + d];
    }
    return peak;
}

// check that a direction is a local maximum in the 2-D map ------------------
static int local_peak(const float *power, int az, int el, int naz,
    int ndelay)
{
    float p = map_value(power, el * naz + az, ndelay);
    for (int de = -1; de <= 1; de++) {
        for (int da = -1; da <= 1; da++) {
            if (!da && !de) continue;
            int a = (az + da + naz) % naz;
            if (map_value(power, (el + de) * naz + a, ndelay) >= p) return 0;
        }
    }
    return 1;
}

// verify one and two separated delay/direction components --------------------
static void test_bartlett_peaks(void)
{
    sdr_array_t *array = make_array();
    sdr_spatial_snapshot_t snap = {0};
    sdr_spatial_grid_t grid = {72, 19, 0, 0, 5, 5};
    float *power = (float *)calloc(72 * 19 * 21, sizeof(float));
    snap.freq = 1575.42e6;
    snap.nant = 8;
    snap.ndelay = 21;
    TEST_ASSERT_TRUE(sdr_spatial_algorithm("Bartlett") != NULL);
    TEST_ASSERT_TRUE(sdr_spatial_algorithm("MUSIC") == NULL);
    add_component(&snap, array, 40, 25, 6, 1.0);
    TEST_ASSERT_TRUE(sdr_spatial_bartlett(&snap, array, &grid, power));
    TEST_ASSERT_EQ_INT(5 * 72 + 8, best_cell(power, 72, 19, 21));
    TEST_ASSERT_NEAR(1.0, power[(5 * 72 + 8) * 21 + 6], 1e-5);
    add_component(&snap, array, 215, 65, 15, 0.8);
    TEST_ASSERT_TRUE(sdr_spatial_bartlett(&snap, array, &grid, power));
    TEST_ASSERT_EQ_INT(5 * 72 + 8, best_cell(power, 72, 19, 21));
    TEST_ASSERT_NEAR(1.0, power[(5 * 72 + 8) * 21 + 6], 1e-5);
    TEST_ASSERT_NEAR(0.64, power[(13 * 72 + 43) * 21 + 15], 1e-5);
    TEST_ASSERT_TRUE(power[(13 * 72 + 43) * 21 + 15] >
        power[(13 * 72 + 43) * 21 + 6]);
    TEST_ASSERT_TRUE(local_peak(power, 8, 5, 72, 21));
    TEST_ASSERT_TRUE(local_peak(power, 43, 13, 72, 21));
    free(power);
    sdr_array_free(array);
}

// verify runtime grid and cadence configuration boundaries ------------------
static void test_spatial_config(void)
{
    sdr_spatial_t *sp = sdr_spatial_new();
    sdr_spatial_cfg_t cfg = {{36, 10, 0, 0, 10, 10},
        31, 25, 8, -1.5, 1.5};
    TEST_ASSERT_TRUE(sdr_spatial_config(sp, &cfg));
    TEST_ASSERT_TRUE(sdr_spatial_select(sp, 1, "Bartlett"));
    cfg.sample_step = 7;
    TEST_ASSERT_EQ_INT(0, sdr_spatial_config(sp, &cfg));
    cfg.sample_step = 25;
    cfg.ndelay = SDR_SPATIAL_NDELAY + 1;
    TEST_ASSERT_EQ_INT(0, sdr_spatial_config(sp, &cfg));
    sdr_spatial_free(sp);
}

// require the tracked sign across all physical RF correlations ---------------
static void test_snapshot_wrap_polarity(void)
{
    const int N = 16;
    sdr_rcv_t *rcv = (sdr_rcv_t *)calloc(1, sizeof(*rcv));
    sdr_ch_t ch = {0};
    sdr_trk_t trk = {0};
    sdr_spatial_snapshot_t snap;
    int8_t code[N * SDR_N_CODES];
    int32_t sums[(N + 1) * SDR_N_CODES] = {0};
    double pos = 0.0;
    sdr_cpx_t forced[1], independent[1], parts[2];

    TEST_ASSERT_TRUE(rcv != NULL);
    rcv->nrfch = 2;
    for (int i = 0; i < 2; i++) {
        rcv->buff[i] = sdr_buff_new(N, 2);
        rcv->rfch[i].fo = 0.0;
        rcv->rfch[i].IQ = 2;
        for (int s = 0; s < N; s++) {
            int sign = i == 0 && s >= N / 2 ? -1 : 1;
            rcv->buff[i]->data[s] = pack_cpx8(3 * sign, 0);
        }
    }
    for (int k = 0; k < SDR_N_CODES; k++) {
        for (int s = 0; s < N; s++) {
            code[k * N + s] = 1;
            sums[k * (N + 1) + s + 1] = s + 1;
        }
    }
    ch.state = SDR_STATE_LOCK;
    ch.rf_ch = 0;
    ch.N = N;
    ch.fs = N;
    ch.T = 1.0;
    ch.len_code = N;
    ch.coff = 0.5;
    ch.fc = 1575.42e6;
    ch.trk = &trk;
    strcpy(ch.sig, "L1CA");
    trk.code = code;
    trk.code_sum = sums;
    trk.code_scale = 1;
    trk.wrap_pol = -1;

    TEST_ASSERT_EQ_INT(0, sdr_spatial_snapshot(rcv, &ch, 0, &snap));
    trk.wrap_pol_valid = 1;
    TEST_ASSERT_EQ_INT(1, sdr_spatial_snapshot(rcv, &ch, 0, &snap));
    for (int i = 0; i < 2; i++) {
        sdr_corr_std(rcv->buff[i], 0, N, N, 0.0, 0.0, code, sums,
            NULL, NULL, 1, N / 2.0, &pos, 1, -1, forced, parts);
        TEST_ASSERT_NEAR(forced[0][0], snap.corr[i][10][0], 1e-6);
        TEST_ASSERT_NEAR(forced[0][1], snap.corr[i][10][1], 1e-6);
    }
    sdr_corr_std(rcv->buff[1], 0, N, N, 0.0, 0.0, code, sums,
        NULL, NULL, 1, N / 2.0, &pos, 1, 0, independent, parts);
    TEST_ASSERT_TRUE(fabsf(independent[0][0] - snap.corr[1][10][0]) > 1.0f);
    trk.wrap_pol = 1;
    TEST_ASSERT_EQ_INT(1, sdr_spatial_snapshot(rcv, &ch, 0, &snap));
    sdr_corr_std(rcv->buff[0], 0, N, N, 0.0, 0.0, code, sums,
        NULL, NULL, 1, N / 2.0, &pos, 1, 1, forced, parts);
    TEST_ASSERT_NEAR(forced[0][0], snap.corr[0][10][0], 1e-6);
    trk.wrap_pol = 0;
    TEST_ASSERT_EQ_INT(0, sdr_spatial_snapshot(rcv, &ch, 0, &snap));
    for (int i = 0; i < 2; i++) sdr_buff_free(rcv->buff[i]);
    free(rcv);
}

// run the spatial estimator tests --------------------------------------------
int main(void)
{
    sdr_func_init("");
    TEST_RUN(test_bartlett_peaks);
    TEST_RUN(test_spatial_config);
    TEST_RUN(test_snapshot_wrap_polarity);
    return 0;
}
