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

The selected BB channel is only the current implementation's common tracking
reference. It is not the physical unit for which a DOA map is conceptually
estimated. The map combines the physical RF-channel antenna inputs and estimates
one spatial response for the selected GNSS signal. If the same satellite/signal
is tracked by multiple BB channels on different physical RF channels, those BB
channels are alternative common-reference tracking sources for the same spatial
target rather than distinct DOA targets.

The long-term selection key should therefore be the GNSS signal identity,
currently represented by the satellite ID plus signal ID (for example
`G12/L1CA`). The backend should select and retain a suitable locked physical-RF
BB channel as the common reference, exposing its BB/RF channel numbers as
diagnostic state rather than using the BB channel number as the primary UI
identity.

Array channels (ARCH) are synthesized beamformed outputs and are not antenna
inputs to the spatial snapshot. Spatial processing correlates the physical RF
buffers `0 .. nrfch-1`; the current UI deliberately excludes BB channels whose
source channel is an ARCH output.

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

Time accumulation is currently performed outside the algorithm by summing
instantaneous Bartlett power maps. This is sufficient for Bartlett, but it is
not the correct abstraction for covariance- or history-dependent algorithms.
MVDR/Capon and MUSIC require accumulation of complex spatial statistics such as
the antenna covariance matrix, not merely averaging already formed power maps.

Before adding those algorithms, the algorithm boundary should become stateful
in concept:

```text
create
reset
update(snapshot, array state as required)
make_map(array model, grid)
free
```

Algorithm-specific accumulation, including Bartlett power averaging and future
covariance accumulation, should belong to the algorithm instance. The common
spatial layer should remain responsible for target/reference selection,
snapshot production, update cadence, and published-map transport.

Static linking is sufficient; runtime dynamic plugins are not required.

Expected later algorithms include:

- MVDR/Capon,
- coherence-aware MUSIC,
- later parametric estimators such as SAGE/maximum-likelihood methods.

## 7. Web UI

The current Spatial page provides:

- a selector that currently lists locked physical-RF L1 C/A BB channels,
- an algorithm selector containing Bartlett,
- a rectangular azimuth/elevation heatmap,
- relative-power display normalized to the brightest cell,
- current C/N0, receiver time, and map sequence,
- predicted satellite LOS overlay when PVT and ephemeris status are available,
- array-calibration validity/provenance.

The map is transferred through the binary WebSocket path. Control and status
remain JSON.

The BB-channel selector is an implementation leak. The DOA/spatial result is
formed from multiple physical RF antenna channels, so the primary selector
should identify the spatial target, not the BB channel used internally as the
common replica source. For the current GNSS-only implementation the target key
should be `satellite + signal`. The UI should show the automatically selected
reference as secondary diagnostic information, for example:

```text
Target: G12 / L1CA
Reference: BB CH 37 / RF CH 1
Elements: RF CH 1-7
```

The backend should choose one eligible locked physical-RF BB channel for that
target and retain it until it is no longer usable, rather than exposing
multiple otherwise equivalent DOA choices for the same satellite/signal. A
manual reference override may be useful later for diagnostics, but should not
be the normal selection model. ARCH channels are beamformed derived channels
and should not appear as spatial-input elements.

The primary Spatial visualization should be a skyplot using the same projection,
azimuth/elevation grid, cardinal labels, satellite marker style, constellation
colors, and PVT/elevation semantics as the Receiver page. The predicted LOS
should be drawn with the same satellite symbol used by Receiver rather than the
current Spatial-specific white circle.

The Receiver and Spatial pages should not maintain independent copies of this
drawing logic. Common sky-coordinate projection, grid, and satellite-marker
rendering should be factored into shared Web UI utilities. The Spatial page then
adds the spatial-power heatmap beneath the ordinary satellite/LOS marker.

The current rectangular azimuth/elevation map should remain available as a
diagnostic view. In particular, it is useful for inspecting raw grid output and
algorithm behavior even though the skyplot is the more natural user-facing
representation.

## 8. Calibration semantics

Explicit calibration validity and provenance are now implemented in the common
array state and reported through `sdr_array_status_t`. Spatial processing still
uses the current `sdr_array_t` model, but the Web UI can distinguish accepted
calibration from a nominal zero/unaccepted state.

The Spatial page reports either an accepted source such as `Continuous`,
`Static`, `Loaded`, or `External`, or explicitly warns that the map uses a
nominal uncalibrated array direction. A spatial map may still be produced while
uncalibrated for diagnostics; its absolute azimuth/elevation must not be treated
as calibrated.

Geometry and calibration remain separate. A geometry file can be loaded as
receiver configuration, while saved calibration is currently loaded explicitly
through the array calibration controls rather than automatically at receiver
startup.

Calibration algorithm design and the selectable continuous/static estimator are
documented separately in
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

Manual inspection of the current Spatial UI with the real data indicates that
the dominant displayed peak is broadly consistent with the predicted satellite
LOS. This is useful evidence that the end-to-end sign/orientation convention is
plausible, but it is not yet a quantitative calibrated-LOS regression test.
A reproducible numerical angular-error check should still be added.

## 10. Remaining work

The principal planned changes after the current Bartlett implementation are:

- change Spatial selection from BB-channel identity to a spatial target keyed by
  satellite + signal, while retaining BB/RF reference-channel identity as
  diagnostic state,
- make the physical RF/antenna-element inputs explicit in status/UI and keep
  ARCH outputs out of the target/input semantics,
- revise the spatial algorithm interface to own persistent algorithm state
  before adding covariance-based estimators,
- factor Receiver skyplot projection/grid/satellite rendering into shared Web UI
  utilities and use the same representation as the primary Spatial view,
- retain the rectangular azimuth/elevation map as an algorithm-diagnostic view,
- quantify calibrated dominant-peak error against predicted GNSS LOS in the
  deterministic real-data replay,
- review common-reference code-wrap/data-polarity handling so one reference
  polarity decision is applied coherently across antenna channels,
- add MVDR/Capon,
- add coherence-aware MUSIC,
- optionally expose the delay dimension,
- consider peak extraction/classification only after the underlying spatial
  estimates are validated.

These changes should remain separable from the selectable array-calibration
work.
