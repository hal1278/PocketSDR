# DOA and Multipath Visualization Design

## 1. Purpose

This document defines the initial design for direction-of-arrival (DOA) processing and real-time multipath visualization in PocketSDR.

The first implementation target is intentionally narrow:

- use a coherent multi-antenna PocketSDR input,
- select one tracked GNSS signal at a time,
- estimate a spatial power map in real time,
- display the map as a heatmap in the Web UI,
- make two spatial peaks visible when a direct path and a sufficiently separated reflected path are present,
- do not classify or automatically declare multipath at this stage.

The design must not prevent later addition of other DOA algorithms, more complete array calibration, raw-IF processing, automatic detection, or multi-satellite processing.

## 2. Scope

### 2.1 Initial scope

The initial implementation shall support:

- PocketSDR live front-end input and IF-file replay through the existing receiver path,
- one selected satellite/signal for spatial processing,
- coherent per-antenna correlator snapshots derived from a common tracking reference,
- delay-aware Bartlett beam scanning,
- an azimuth/elevation heatmap updated at a lower rate than the tracking loop,
- reuse of the existing array geometry, attitude, and per-channel phase-bias calibration,
- a stable algorithm interface so that MVDR/Capon, MUSIC, and later estimators can be added without changing the UI/data-source boundary.

The first validation target is GPS L1 C/A unless another signal is explicitly selected during implementation.

### 2.2 Out of scope for the first implementation

The first implementation does not require:

- automatic multipath detection or thresholding,
- automatic peak association,
- multipath mitigation in the navigation solution,
- simultaneous spatial processing of all tracked satellites,
- raw-IF wideband direction finding,
- complex-gain or measured-array-manifold calibration,
- MUSIC or SAGE,
- production-quality uncertainty estimation.

These are future extensions and must remain possible within the interfaces defined here.

## 3. Design constraints

### 3.1 Common-reference spatial phase

The complex prompt outputs of independently tracked RF channels shall not be used directly as the spatial snapshot.

Each PocketSDR tracking channel owns an independent FLL/PLL/DLL state. Therefore, even when several channels track the same satellite, their complex prompt phases are referenced to independent carrier NCO states. Directly stacking these correlator outputs would mix the physical inter-antenna phase with receiver-loop phase states.

Instead, one tracking channel is selected as the reference. Its code and carrier states define a common replica, which is applied to the synchronized IF samples from all antenna RF channels. The resulting correlator outputs retain the inter-antenna spatial phase required for beamforming and DOA estimation.

### 3.2 Replay and live processing

Replay and live processing shall use the same receiver and spatial-processing pipeline.

The DOA implementation must not define separate algorithm paths for live input and IF-file replay. Input-mode differences shall remain below the receiver/spatial-snapshot boundary.

### 3.3 Existing PocketSDR array model

The first implementation shall reuse the existing PocketSDR array information:

- antenna element positions,
- enabled-element mask,
- array attitude,
- RF-channel phase bias,
- calibration save/load mechanism where practical.

The first implementation shall not replace the existing calibration estimator merely to support the initial heatmap.

The steering-vector calculation should, however, be factored so that the existing beamformer, the new Bartlett estimator, and future algorithms use the same array-model definition.

## 4. Processing architecture

```text
Live front-end / IF file
          |
          v
    existing sdr_rcv
          |
          v
 existing tracking
(selected reference channel)
          |
   common code/carrier replica
          |
          v
+-----------------------------+
| spatial snapshot extraction |
|                             |
| correlate each antenna      |
| at multiple code delays     |
+--------------+--------------+
               |
               v
     C[antenna][delay]
               |
               v
        shared ArrayModel
               |
               v
      SpatialAlgorithm API
               |
      Bartlett initially
               |
               v
       P[az][el][delay]
               |
        max/selection over delay
               |
               v
          H[az][el]
               |
          WebSocket
               |
               v
        Web UI heatmap
```

## 5. Spatial snapshot extraction

### 5.1 Snapshot definition

A spatial snapshot contains complex correlations for one selected satellite/signal, evaluated across all active antenna channels and a configurable set of code-delay taps.

Conceptually:

```c
typedef struct {
    double time;
    int sat;
    int sig;
    double freq;

    int nant;
    int ndelay;
    const double *delay;
    const sdr_cpx_t *corr; /* [nant][ndelay] */
} sdr_spatial_snapshot_t;
```

Exact field types and ownership rules may be adjusted to match PocketSDR conventions during implementation.

### 5.2 Common replica

For the selected signal, the reference tracking channel provides:

- code timing,
- Doppler/carrier frequency,
- carrier phase/NCO state,
- code sequence and signal-specific tracking state.

The same replica is then correlated against each RF channel at the same receiver epoch.

This is a new spatial-processing path. It should reuse existing PocketSDR correlation kernels rather than introduce a second correlator implementation.

### 5.3 Delay taps

The first implementation should use a modest code-delay search interval and number of taps sufficient to expose a delayed reflected path while keeping CPU cost bounded.

An initial engineering default may be approximately:

- delay range: about +/-1 chip around the tracked direct-path delay,
- delay taps: approximately 21-41,

but these are initial defaults rather than interface constraints.

The existing diagnostic correlator limit (`SDR_N_CORRX`) must not become a public DOA-interface limit. Spatial processing should own its required delay-grid configuration even if it reuses the same low-level correlator implementation.

## 6. Array model

The spatial estimator needs a steering vector for an arbitrary candidate azimuth/elevation and signal frequency.

The common array model should expose the equivalent of:

```c
steering_vector(array_model, az, el, freq, weights);
```

The initial model includes:

- antenna positions in the existing array body frame,
- the calibrated array orientation,
- per-channel phase bias,
- signal carrier frequency.

The first implementation does not require amplitude calibration.

The interface must permit later extension to:

- frequency-dependent phase/delay calibration,
- complex gain calibration,
- measured array manifold,
- antenna position refinement.

## 7. Spatial algorithm interface

The spatial estimator shall be selected independently from the data source and UI.

A static dispatch table is sufficient. Runtime dynamic loading is not required.

Conceptually:

```c
typedef struct {
    const char *name;
    unsigned capabilities;
    int  (*init)(...);
    int  (*process)(const sdr_spatial_snapshot_t *snapshot,
                    const sdr_array_model_t *array,
                    const sdr_spatial_grid_t *grid,
                    sdr_spatial_map_t *map);
    void (*free)(...);
} sdr_spatial_alg_t;
```

The exact API should follow existing PocketSDR C conventions and avoid unnecessary allocation in the real-time path.

Planned algorithms are:

1. Bartlett / conventional beamforming -- initial implementation,
2. MVDR/Capon,
3. MUSIC with an explicit coherence-handling method such as spatial smoothing where appropriate,
4. later parametric estimators such as SAGE/maximum-likelihood methods.

Algorithms may declare required input capabilities. This allows a future raw-IF estimator without changing the current correlator-snapshot interface.

## 8. Initial Bartlett estimator

For antenna vector `C(tau)` and steering vector `a(az, el)`, the initial spatial spectrum is based on conventional beamforming:

```text
P(az, el, tau) = average_n |a(az, el)^H C_n(tau)|^2
```

Several consecutive snapshots may be accumulated in power to reduce display variance.

The initial 2-D heatmap is then:

```text
H(az, el) = max_tau P(az, el, tau)
```

The delay dimension remains available internally so that a later UI can inspect `az/el/delay` jointly instead of discarding delay information.

No peak detector is required in the first stage.

## 9. Grid and update rate

The spatial scan grid and display update rate are configuration parameters rather than tracking-loop constants.

Initial values should favor implementation speed and predictable CPU use:

- coarse spatial grid during bring-up, for example 5 deg azimuth/elevation spacing,
- finer 1-2 deg spacing after profiling if feasible,
- heatmap update approximately 2-5 Hz,
- accumulation interval approximately 20-100 ms as an initial range.

The algorithm shall not attempt to push one WebSocket map per 1 ms tracking epoch.

## 10. Web UI

A new DOA/Spatial page should be added to the existing PocketSDR Web UI rather than introducing another GUI framework.

The initial page requires only:

- satellite/signal selector,
- current algorithm selector,
- azimuth/elevation heatmap,
- color-scale control or fixed documented dB normalization,
- selected-signal C/N0 and basic state,
- predicted satellite LOS marker when navigation/receiver position is available,
- current update time/status.

The UI shall not label a secondary peak as "multipath" automatically in the initial implementation.

Heatmap samples should use the existing binary WebSocket path or an equivalent compact binary message rather than JSON arrays of floating-point map values.

Control/status metadata may remain JSON.

## 11. Calibration strategy

### 11.1 Initial implementation

Use the existing PocketSDR calibration output unchanged where possible.

The initial path assumes that the existing carrier-phase array calibration provides sufficiently accurate:

- orientation,
- per-channel phase bias,
- array geometry.

The purpose of the first implementation is to establish whether distinct spatial peaks can be observed, not to redesign calibration simultaneously.

### 11.2 Later calibration improvements

The shared array-model boundary must support future replacement or augmentation by:

- weighted/robust phase calibration,
- batch offline calibration,
- frequency-dependent delay/phase calibration,
- complex channel-gain calibration,
- measured array-manifold calibration.

Calibration algorithms and spatial algorithms should remain independently selectable.

## 12. Replay and validation

IF-file replay shall be the primary bring-up and regression path because it provides deterministic input while exercising the same receiver and spatial-processing implementation as live reception.

Validation should proceed in the following order:

1. verify common-reference correlator phase on a known single-source condition,
2. verify that the Bartlett maximum agrees approximately with known GNSS LOS,
3. verify repeatability during IF replay,
4. introduce or capture a controlled reflected path,
5. confirm that two stable/repeatable peaks become visible under sufficiently separated conditions,
6. switch the same configuration to the live front end.

A two-peak result is not guaranteed for arbitrary multipath. Resolvability depends on array aperture and geometry, direction separation, path delay, relative power, signal bandwidth, integration interval, and calibration error.

## 13. Proposed source-code boundaries

Names are provisional until implementation begins, but responsibility should remain separated approximately as follows:

```text
src/sdr_array.c
    existing array state/calibration
    common array-model / steering-vector functions

src/sdr_spatial.c
    common-reference snapshot extraction
    delay-grid management
    spatial algorithm dispatch
    Bartlett implementation initially

src/sdr_rcv.c
    receiver-facing wrappers / selected-signal access

src/sdr_web.c
    WebSocket control/status/data adapter

html/js/pages/spatial.js   (name provisional)
    heatmap and controls
```

If implementation shows that snapshot extraction couples too strongly to channel tracking internals, a smaller helper may remain in `sdr_ch.c`; the spatial estimator itself should remain outside the tracking-loop implementation.

## 14. Implementation stages

### Stage 1: data path

- add common-reference multi-antenna correlator snapshot extraction,
- expose a stable snapshot structure,
- support one selected tracked signal,
- verify phase consistency from IF replay.

### Stage 2: first spatial estimator

- factor/reuse array steering-vector calculation,
- implement Bartlett scan,
- add time/power accumulation,
- produce a 2-D az/el map.

### Stage 3: UI

- add WebSocket map transport,
- add signal and algorithm selection,
- render real-time heatmap,
- optionally overlay expected satellite LOS.

This completes the first target when two resolvable peaks can be visually observed in an appropriate direct-plus-reflected-path test.

### Stage 4: algorithm extensions

After the first target is demonstrated:

- MVDR/Capon,
- coherence-aware MUSIC,
- optional 3-D az/el/delay inspection,
- improved calibration,
- automatic peak extraction/detection only if required later.

## 15. Non-blocking assumptions for the first implementation

Unless measurements during bring-up require otherwise, the implementation may initially assume:

- fixed array geometry during a run,
- an already calibrated array,
- one selected L1-band GNSS signal,
- one selected satellite at a time,
- no automatic source-count estimation,
- no requirement to distinguish direct and reflected peaks automatically,
- heatmap latency of a few hundred milliseconds is acceptable.

These assumptions are implementation defaults, not permanent format/API restrictions.
