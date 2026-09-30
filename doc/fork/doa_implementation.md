# DOA / Spatial Processing Implementation

## 1. Purpose and relation to the design document

This document records the DOA/spatial-processing implementation currently
present in this fork.

The initial design is preserved separately in
[`doa_design.md`](doa_design.md). That document describes the intended first
architecture. This document describes what has actually been implemented,
what is still provisional, and which interfaces should be revised before
adding more spatial algorithms.

Array-calibration work is documented separately in
[`array_calibration.md`](array_calibration.md).

## 2. Current processing path

The current implementation uses the ordinary receiver path for both live input
and IF-file replay:

```text
live front end / IF file
        |
        v
      sdr_rcv
        |
        v
 selected tracked L1 C/A channel
        |
 common code/carrier replica
        |
        v
 per-RF-channel correlation at multiple delays
        |
        v
 C[antenna][delay]
        |
        v
 shared sdr_array_t steering model
        |
        v
 Bartlett scan
        |
        v
 max over delay
        |
        v
 azimuth/elevation power map
        |
        v
 binary WebSocket frame
        |
        v
 Web UI Spatial page
```

The implementation is centered in:

- `src/sdr_spatial.c` for common-reference snapshot extraction, Bartlett
  processing, accumulation, and map publication,
- `src/sdr_array.c` for the shared array steering model,
- `src/sdr_rcv.c` for receiver integration,
- `src/sdr_web.c` for WebSocket transport and control,
- `html/js/pages/spatial.js` for the current Spatial page.

## 3. Common-reference spatial snapshots

The spatial processor does not stack prompt correlators from independently
tracked RF channels.

Each tracking channel has its own tracking-loop/NCO state. Using those complex
outputs directly would mix physical inter-antenna phase with independent
receiver phase references.

Instead, the selected tracking channel provides one common code/carrier state.
`snapshot_delays()` applies that reference to synchronized IF buffers from
all RF channels using the existing PocketSDR correlator.

The current implementation accepts only a locked `L1CA` reference channel.
All participating RF channels must have compatible LO frequency and IQ mode.

The snapshot contains:

- selected channel, satellite, and signal identifiers,
- carrier frequency and C/N0,
- one complex correlation vector per code-delay tap,
- one row per RF channel.

The current default delay grid is 21 taps from -1 to +1 chip.

## 4. Shared array model

The existing array model is reused rather than duplicated for spatial
processing.

`sdr_array_steering()` provides the steering vector used by both the array
implementation and the Bartlett scan. It uses:

- `sdr_array_t.ant_pos`,
- `sdr_array_t.ant_ena`,
- the current roll/pitch/yaw state,
- per-channel range/phase bias state,
- carrier frequency.

This shared function is the boundary that should also be used by later
spatial estimators.

For the current physical seven-element array on an eight-channel front end,
the intended runtime state is:

```text
nrfch   = 8
ant_ena = [1, 1, 1, 1, 1, 1, 1, 0]
```

A numeric geometry-file row is currently interpreted as an enabled antenna.
Therefore a final `0 0 0` row does not mean "unused CH8"; a seven-element
geometry file should contain seven numeric rows unless an explicit geometry
enable-mask format is added.

## 5. Bartlett implementation

The current estimator computes conventional beamforming power for each
azimuth/elevation/delay cell:

```text
P(az, el, tau) = |a(az, el)^H C(tau)|^2 / N_ant^2
```

Several snapshots are accumulated in power. When the configured accumulation
count is reached, the published 2-D map is:

```text
H(az, el) = max_tau mean_n P_n(az, el, tau)
```

The current defaults are:

- azimuth/elevation spacing: 5 degrees,
- maximum grid dimensions: 72 x 19,
- delay taps: 21,
- delay range: -1 to +1 chip,
- spatial sample step: 20 receiver cycles,
- accumulation count: 10 snapshots.

The delay dimension is retained internally but the current Web UI publishes
only the reduced azimuth/elevation map.

## 6. Spatial algorithm dispatch

A static dispatch entry exists and currently resolves only `Bartlett`.

The current interface is stateless from the algorithm point of view:

```c
int (*process)(const sdr_spatial_snapshot_t *,
               const sdr_array_t *,
               const sdr_spatial_grid_t *,
               float *);
```

This is sufficient for Bartlett but should be revised before adding covariance-
or history-dependent algorithms. MVDR/Capon and MUSIC need accumulated complex
spatial statistics rather than only an instantaneous map.

The intended next interface is stateful in concept:

```text
create
reset
update(snapshot)
make_map
free
```

Static linking is sufficient; runtime dynamic plugins are not required.

Expected later algorithms include:

- MVDR/Capon,
- coherence-aware MUSIC,
- later parametric estimators such as SAGE/maximum-likelihood methods.

## 7. Web UI

The current Spatial page provides:

- a selector for locked L1 C/A reference channels,
- an algorithm selector containing Bartlett,
- a rectangular azimuth/elevation heatmap,
- relative-power display normalized to the brightest cell,
- current C/N0, receiver time, and map sequence,
- predicted satellite LOS overlay when PVT and ephemeris status are available.

The map is transferred through the binary WebSocket path. Control and status
remain JSON.

A second skyplot view is desirable. The existing Receiver page already
contains sky-coordinate projection and array-gain heatmap logic. That
rendering code should be factored into shared Web UI utilities rather than
reimplemented independently in the Spatial page.

The rectangular view should remain available for algorithm diagnostics even
after a skyplot view is added.

## 8. Calibration semantics

The spatial processor currently copies and uses the current `sdr_array_t`
state. It does not require a separately asserted "calibration valid" state.

As a consequence, a spatial map can be produced with zero-initialized
attitude/bias state. Such a map is still a Bartlett scan against the nominal
array model, but its absolute azimuth/elevation should not be treated as
calibrated.

The fork should therefore add explicit calibration validity/provenance rather
than infer validity from `calib_run` or `nep` alone.

Calibration algorithm changes are deliberately kept outside the spatial
processor and are specified in
[`array_calibration.md`](array_calibration.md).

## 9. Validation

The primary real-data fixture for this work is the coherent eight-channel L1
recording:

```text
data/260824_8ch_16Mubuntu.bin
data/260824_8ch_16Mubuntu.tag
```

The existing array beamforming path has been used successfully with this
recording.

The real recording is suitable for validating:

- synchronized RF-channel handling,
- common-reference correlation,
- shared geometry/steering behavior,
- replay/live common processing,
- backend/UI transport,
- calibrated LOS alignment once calibration is valid.

It is not assumed to contain a guaranteed resolvable two-path case. Synthetic
two-component tests remain the deterministic validation path for multiple
Bartlett peaks.

## 10. Remaining work

The principal planned changes after the current Bartlett implementation are:

- add explicit calibration validity/provenance,
- revise the spatial algorithm interface to own persistent algorithm state,
- share skyplot rendering primitives between Receiver and Spatial pages,
- validate calibrated peak direction against predicted GNSS LOS,
- add MVDR/Capon,
- add coherence-aware MUSIC,
- optionally expose the delay dimension,
- consider peak extraction/classification only after the underlying spatial
  estimates are validated.

These changes should remain separable from the selectable array-calibration
work.
