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
`G12/L1CA`). The backend should choose one suitable locked physical-RF BB
channel when the target is selected and expose its BB/RF channel numbers as
diagnostic state rather than using the BB channel number as the primary UI
identity.

Automatic reference fallback is deliberately deferred. In the first target-based
implementation, the chosen reference should remain fixed while it is usable; if
it becomes unusable, map production should stop and report the unavailable
reference rather than silently switching to another BB/RF channel. This keeps
the target/reference semantic cleanup independent from the larger runtime
resilience change.

Array channels (ARCH) are synthesized beamformed outputs and are not antenna
inputs to the spatial snapshot. Spatial processing correlates the physical RF
buffers `0 .. nrfch-1`; the current UI deliberately excludes BB channels whose
source channel is an ARCH output.

The snapshot contains:

- selected channel, satellite, and signal identifiers,
- carrier frequency and C/N0,
- one complex correlation vector per code-delay tap,
- one row per RF channel.

### Code-wrap polarity coherence

The `pol` argument of `sdr_corr_std()` controls how the two parts of a
correlation window are combined when the window crosses a primary-code-period
boundary. With `pol == 0`, the correlator already determines the sign from the
reference-channel correlation. This is a local correlator decision; it is not
the same state as decoded navigation-data polarity or secondary-code polarity.

For spatial processing, that decision must be common across antenna channels.
Independent automatic polarity decisions on each RF channel can introduce a
spurious element-wise 180-degree phase reversal and corrupt the array manifold.

The intended responsibility split is:

```text
sdr_func.c
    correlator AUTO/FORCE polarity handling
    expose the polarity actually used

sdr_ch.c
    ordinary reference-channel tracking performs AUTO detection
    retain the polarity used for the current tracking cycle

sdr_spatial.c
    apply that one reference polarity to every RF-channel correlation
    used in the common-reference snapshot

sdr_array.c
    unchanged; array geometry/calibration is not responsible for
    correlator-window polarity
```

The normal single-channel tracking path should not gain an array-specific
cross-channel branch. The additional runtime work should be limited to exposing
and storing the sign already computed by the existing correlator, then passing
it to the existing per-RF spatial correlations. No additional reference
correlation or per-antenna polarity vote is required.

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

The first target-based backend should choose one eligible locked physical-RF BB
channel when the target is selected and retain it as a fixed reference. If that
reference becomes unavailable, the first implementation should stop map
production and report the condition instead of automatically switching
references. Automatic fallback can be added later together with runtime element
availability handling. A manual reference override may be useful later for
diagnostics, but should not be the normal selection model. ARCH channels are
beamformed derived channels and should not appear as spatial-input elements.

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

## 10. Implementation roadmap

The work after the current Bartlett MVP should be split into independently
verifiable phases. Reference auto-switching and degraded-array operation are
explicitly deferred until the simpler target/reference semantics and algorithm
boundaries are stable.

### Phase 1: fix common-reference polarity and establish a numerical baseline

- extend the low-level standard-correlator path so AUTO polarity can expose the
  sign actually used without changing the existing normal callers,
- retain that sign in the reference tracking state for the current cycle,
- apply the same sign to all RF-channel correlations in
  `sdr_spatial_snapshot()`,
- do not add a second polarity estimator in `sdr_array.c` or an
  array-specific branch to the ordinary tracking control flow,
- keep existing Bartlett numerical behavior otherwise unchanged,
- replay the known-good eight-channel IF recording with valid calibration and
  quantify dominant-peak angular error relative to predicted GNSS LOS.

Manual inspection already indicates broad peak/LOS agreement; this phase turns
that observation into a reproducible numerical regression surface.

### Phase 2: share the Receiver skyplot representation

- factor sky projection, azimuth/elevation grid, cardinal labels, and satellite
  marker drawing from the Receiver page into shared Web UI utilities,
- use the same satellite symbol and constellation/PVT conventions for the
  predicted LOS in Spatial,
- make the skyplot the primary Spatial view with the power map underneath the
  normal satellite marker,
- retain the current rectangular azimuth/elevation grid as a diagnostic view,
- verify that the Receiver-page appearance does not regress.

This phase is UI-only with respect to the spatial estimator and should not
change Bartlett results.

### Phase 3: separate spatial target identity from the common-reference channel

- change normal Spatial selection from BB-channel identity to
  `satellite + signal`,
- choose one eligible locked physical-RF BB channel when the target is selected,
- expose the chosen `BB CH / RF CH` as diagnostic state,
- expose the physical RF antenna-element set separately from the reference,
- keep ARCH outputs out of spatial-input semantics,
- keep the selected reference fixed; if it becomes unusable, stop publishing new
  maps and report the condition rather than automatically switching references.

The initial implementation may require the reference to be one of the active
spatial elements. The public/internal model should nevertheless keep reference
identity and element membership as separate concepts so that restriction can be
removed later if needed.

### Phase 4: make the spatial algorithm interface stateful

Refactor the current stateless `process(snapshot, array, grid, power)` boundary
without changing Bartlett output. The intended lifecycle is:

```text
create
reset
update(snapshot)
make_map(array model, grid)
free
```

Move Bartlett power accumulation into the Bartlett algorithm instance. This
provides the correct boundary for future algorithms that need different
persistent statistics, especially covariance accumulation for MVDR/Capon and
MUSIC.

Existing Bartlett synthetic tests and the real-data LOS regression from Phase 1
should gate this refactor.

### Phase 5: degraded-array operation

After the preceding behavior is stable:

- distinguish configured antenna enablement from runtime RF-channel
  availability,
- derive an effective element mask for each usable snapshot,
- continue spatial estimation when a subset of physical elements is unavailable
  if the remaining geometry is sufficient,
- normalize the estimator using the effective element set,
- reset persistent algorithm state whenever the effective array geometry/mask
  changes materially,
- expose active/unavailable RF elements in status/UI.

The current implementation rejects the whole snapshot if any participating RF
buffer/LO/IQ condition is invalid, so this is a real data-path change rather
than only a UI enhancement.

### Phase 6: reference fallback

Only after degraded-array operation is defined should automatic reference
fallback be added.

- if the fixed reference becomes unusable, select another eligible locked BB
  channel for the same `satellite + signal`,
- permit the replacement reference to use any compatible physical RF channel
  allowed by the final reference/element rules,
- reset algorithm accumulation on the transition,
- expose the transition in status/logging.

Frequent best-C/N0 reference hopping is not intended; a usable current
reference should remain stable.

### Phase 7: additional estimators

With the stateful algorithm API and runtime-element semantics established:

- add MVDR/Capon,
- add coherence-aware MUSIC,
- optionally expose the delay dimension,
- consider later parametric estimators such as SAGE/maximum-likelihood methods,
- add peak extraction/classification only after the underlying spatial
  estimates are validated.

Calibration algorithm changes remain separate from this roadmap. Geometry and
accepted array calibration continue to be supplied through the shared
`sdr_array_t` model.
