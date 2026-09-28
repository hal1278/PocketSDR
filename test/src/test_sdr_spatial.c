// Numerical tests for the shared array model and delay-aware Bartlett scan.
#include "test_sdr.h"

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

// run the spatial estimator tests --------------------------------------------
int main(void)
{
    TEST_RUN(test_bartlett_peaks);
    TEST_RUN(test_spatial_config);
    return 0;
}
