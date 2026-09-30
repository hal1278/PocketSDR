# Array Calibration Design for the Fork

## 1. Scope

This document records calibration findings from the DOA development work and
defines the fork-side calibration architecture to implement next.

It does not replace the upstream PocketSDR documentation. The existing
PocketSDR calibration path remains available as a selectable compatibility
algorithm.

The new algorithm introduced by this fork is initially static-only. A future
extension may use the same static initialization and then continue with a
dynamic attitude estimator.

## 2. Existing PocketSDR calibration behavior

The current array calibration estimates:

- roll, pitch, and yaw,
- relative RF-channel carrier-phase/range biases.

For each satellite, carrier-phase single differences between RF CH1 and other
enabled RF channels are compared with the phase predicted from satellite LOS,
array geometry, attitude, and channel bias.

Initialization performs iterative LS from yaw starts separated by 15 degrees.
If initialization passes the residual check, the current implementation
continues with a random-walk EKF.

The current residual acceptance threshold is:

```text
MAX_RMS = 0.020 m
```

This threshold should remain unchanged for the new work unless independent
evidence justifies changing it.

## 3. Geometry enable-mask failure found during replay

The initial Web replay used a geometry file containing eight numeric rows even
though the physical array has seven active elements. The eighth row was
`(0,0,0)`.

The geometry loader enables every parsed numeric row. The resulting state was:

```text
nrfch   = 8
ant_ena = [1, 1, 1, 1, 1, 1, 1, 1]
```

CH8 had no tracking observations. Its enabled bias state therefore produced an
all-zero LS design-matrix column, making the initialization solve singular.

With the geometry corrected to seven numeric rows, the intended state is:

```text
nrfch   = 8
ant_ena = [1, 1, 1, 1, 1, 1, 1, 0]
```

This removes the CH8 rank deficiency.

The front end is still an eight-RF-channel receiver; `nrfch=8` is not itself
an error. The inactive element must simply remain disabled.

## 4. Wrapped-phase initialization failure found after fixing geometry

After CH8 was disabled, a separate failure remained.

A recorded failed epoch had:

- a full-rank active-state matrix,
- valid PVT and broadcast satellite positions,
- valid CH1-to-array carrier-phase pairs,
- 17 accepted phase measurements,
- measurements from G15, G20, and G21.

G07 was present on CH1 but had no CH2-CH7 pair in that epoch.

The yaw search converged to a wrapped-phase branch with approximately 46 mm
reported RMS, so the existing 20 mm limit correctly rejected it.

The carrier data themselves support a much better solution:

- applying a successful state to the same failed-epoch measurements gives
  about 5 mm post-fit RMS,
- adding the missing G07 pairs to the recorded failed epoch gives about
  4.5 mm,
- adding only the G07 CH1-to-CH7 pair gives about 4.35 mm.

Thus the failure is not the carrier-observation noise floor, a bad RF channel,
or a rank deficiency. It is a solution-basin ambiguity in the current
single-epoch wrapped-phase initialization.

Full matrix rank is therefore necessary but not sufficient for reliable
initialization.

A specific satellite such as G07 must not be hard-coded. In this recording,
G07 simply provides additional independent LOS information that selects the
low-residual wrapped-phase branch.

## 5. Reference replay

A headless run through the current `python/pocket_sdr.py` bindings with the
same IF interval and seven-element geometry reached approximately 4.43 mm RMS
with one accepted epoch.

The instrumented C replay reached approximately the same 4-5 mm solution when
its observation set entered the correct branch.

This supports treating the remaining issue as robustness of initialization
against the instantaneous observation set, rather than as a separate
Python-versus-Web calibration algorithm.

## 6. Calibration algorithm selection

Calibration algorithm selection shall be separate from the existing
calibration mode.

The existing mode answers "which state components are estimated":

```text
BOTH
BIAS
RPY
```

The algorithm selector answers "how calibration is initialized and evolved".

The intended architecture is:

| Algorithm | Initialization | After successful initialization | Status |
| --- | --- | --- | --- |
| `CONTINUOUS` | current single-epoch yaw-search LS | current EKF | retain |
| `STATIC` | multi-epoch static solve | freeze accepted state | implement next |
| `STATIC_DYNAMIC` | same robust static initialization | track dynamic attitude | future |

The intended API names are:

```c
#define SDR_CALIB_ALG_CONTINUOUS     0
#define SDR_CALIB_ALG_STATIC         1
#define SDR_CALIB_ALG_STATIC_DYNAMIC 2 /* reserved */
```

`CONTINUOUS` describes the current upstream-compatible behavior without
overloading the word "original", which is reserved in the fork documentation
for upstream/original PocketSDR material.

Algorithm selection remains orthogonal to `BOTH/BIAS/RPY`.

## 7. Static calibration model

The new static algorithm assumes that, over its calibration observation
window:

```text
array attitude is constant
RF-channel biases are constant
```

The receiver position does not have to be numerically identical at each epoch
provided each observation is modeled using the corresponding receiver
position and satellite LOS.

Significant rotation of the physical array during the collection window
violates the static model.

The static algorithm is intended first for stationary installation and DOA
validation, where robust initialization is more important than continuous
attitude tracking.

## 8. Static calibration lifecycle

The intended lifecycle is:

```text
NONE
  |
  | Start
  v
COLLECTING
  |
  | sufficient multi-epoch information
  v
SOLVING
  |
  +-- invalid/ambiguous --> continue collecting
  |
  +-- accepted
          |
          v
        VALID
          |
          v
     state remains fixed
```

A rejected candidate must not be published as calibrated state.

After a static solution is accepted:

- roll/pitch/yaw are frozen,
- RF-channel biases are frozen,
- calibration estimation stops,
- the state remains available to beamforming and spatial processing.

The first static implementation therefore does not continue the existing EKF
after successful initialization.

## 9. Multi-epoch initialization

The static algorithm should combine carrier-phase constraints from multiple
epochs while solving one common attitude/bias state.

Conceptually:

```text
minimize over x

sum over epochs, satellites, and antenna pairs

wrap_lambda(
    measured carrier-phase single difference
    - predicted array phase(x)
)^2
```

The purpose of the multi-epoch window is to make initialization less sensitive
to short-lived satellite/RF-channel dropouts and to provide enough independent
LOS information to distinguish wrapped-phase branches.

A fixed minimum satellite count may be useful as a conservative collection
gate, but it should not be treated as the fundamental observability
criterion. Directional diversity and the actual measurement Jacobian contain
more information than satellite count alone.

## 10. Validation and acceptance

The new static solver should not accept a candidate merely because the design
matrix is full rank.

At minimum, acceptance should consider:

- sufficient usable measurements,
- active-state rank and conditioning,
- independent LOS/directional information,
- convergence of the wrapped-phase solution,
- post-fit residual RMS.

The existing `MAX_RMS=0.020 m` limit should initially remain as a final
residual gate. The observed 3-5 cm wrong branch is evidence for retaining a
strong rejection criterion, not for relaxing it.

If no candidate is acceptable, the static algorithm should continue collecting
rather than fall back to a high-residual state.

## 11. Calibration state and provenance

The current `calib_run` and `nep` fields do not fully describe calibration
validity.

For example, a loaded state can have useful attitude/bias values without the
same epoch-count semantics as an estimated state, while a zero-initialized
array can still be consumed by other code.

The fork should expose explicit calibration state/provenance. Conceptually:

```text
NONE
COLLECTING
VALID
LOADED
```

Intermediate solver states may remain internal.

Consumers such as beamforming and the Spatial page should be able to
distinguish a calibrated absolute array model from a nominal/uncalibrated one.

## 12. Future static-dynamic algorithm

A future mode may reuse the robust static solution as the initial condition for
dynamic operation.

The likely physical split is:

```text
static initialization
        |
        v
RPY_0 + RF-channel biases
        |
        +--> RF-channel biases remain fixed or very slowly varying
        |
        +--> roll/pitch/yaw become dynamic states
```

RF-channel biases are receiver-chain calibration parameters and should not
normally follow platform motion. Attitude must be allowed to evolve when the
array rotates.

The exact dynamic model and estimator are intentionally deferred. The first
new algorithm remains static-only so that wrapped-phase initialization can be
validated independently of dynamic-state modeling.

## 13. Implementation constraints for the next step

The next implementation should:

- preserve the existing/upstream-compatible calibration path,
- add a separately selectable static algorithm,
- keep algorithm selection independent of `BOTH/BIAS/RPY`,
- collect multi-epoch calibration measurements only for the static algorithm,
- freeze the accepted static state,
- expose explicit calibration validity/provenance,
- keep `MAX_RMS` unchanged initially,
- avoid satellite-specific or fixture-specific heuristics,
- validate with deterministic IF replay before considering the static path a
  default.

The future static-dynamic algorithm should fit into the same selector without
requiring the static algorithm to be redesigned.


## 14. Internal calibration state

The implementation should keep calibration control, validity, and provenance
as separate state rather than encoding all semantics in `calib_run`,
`nep`, or covariance contents.

Conceptually, `sdr_array_t` should contain state equivalent to:

```c
int calib_run;       /* estimator is currently consuming observations */
int calib_mode;      /* SDR_CALIB_BOTH / BIAS / RPY */
int calib_alg;       /* CONTINUOUS / STATIC / ... */
int calib_valid;     /* x[] contains an accepted calibrated state */
int calib_source;    /* NONE / CONTINUOUS / STATIC / LOADED / EXTERNAL */
```

This supports the following externally meaningful states:

```text
never calibrated:
    run=0, valid=0, source=NONE

CONTINUOUS initializing:
    run=1, valid=0

CONTINUOUS tracking:
    run=1, valid=1, source=CONTINUOUS

STATIC collecting/solving:
    run=1, valid=0

STATIC complete:
    run=0, valid=1, source=STATIC

loaded calibration:
    run=0, valid=1, source=LOADED

direct sdr_array_set() state:
    valid=1, source=EXTERNAL
```

`LOADED` applies only to a successful `sdr_array_load()` call. A failed
`CONTINUOUS` initialization or EKF fallback may leave modified values in
`x[]`; it must clear validity and provenance without changing the existing
numerical path.

The existing `nep` field may retain its current meaning for
`CONTINUOUS`. Static collection statistics should be represented separately
instead of redefining `nep` ambiguously.

Useful static statistics include:

```text
number of collected epochs
number of retained physical phase measurements
number of distinct contributing satellites
collection time span
last attempted candidate RMS
```

## 15. Static calibration context

Multi-epoch storage should not be embedded as a large fixed array in
`sdr_array_t`.

The current spatial path copies the small public array state by value before
using it. A large calibration buffer inside that structure would make this
copy expensive and would unnecessarily couple spatial processing to
calibration internals.

Instead, the static algorithm should own a private context through a pointer,
for example:

```c
typedef struct sdr_calib_static_tag sdr_calib_static_t;

/* in sdr_array_t */
sdr_calib_static_t *static_cal;
```

The context is created/freed with the array object and reset when a new static
calibration run begins.

The Spatial path must clear the private pointer in its by-value snapshot and
must never own or free the calibration context.

Conceptually it stores:

```c
struct sdr_calib_static_tag {
    sdr_calib_meas_t *meas;
    int nmeas;
    int nmax;
    int nepoch;
    double t_first;
    double t_last;
    double last_rms;
};
```

Exact field names and allocation strategy may follow existing PocketSDR coding
style.

## 16. Materialized calibration measurements

The static algorithm should not retain whole `obsd_t` epochs, pointers to
`nav_t`, or receiver/PVT objects.

At each normal `sdr_array_calib()` call, the observation/navigation data
needed for calibration should be reduced immediately to self-contained
measurement rows.

A conceptual row is:

```c
typedef struct {
    double time;
    double los[3];       /* ENU receiver-to-satellite unit vector */
    double lambda;       /* carrier wavelength */
    double phase_sd;     /* measured CHk-CH1 phase difference, metres */
    int sat;
    int rfch;            /* secondary RF channel / antenna index */
    int epoch;
} sdr_calib_meas_t;
```

The row must contain enough information to rebuild the same model currently
used by `build_meas()` without later dependence on mutable navigation data.

Satellite-health, elevation, carrier-frequency, reference-channel, and valid
carrier-phase checks should continue to follow the current calibration
semantics.

Geometry and enabled-element state remain part of the common array object and
are not duplicated into every measurement row.

## 17. Static collection policy

While `STATIC` calibration is running, every eligible PVT epoch contributes
materialized CH1-to-secondary-channel phase measurements.

Collection must be bounded. The implementation should use a finite retained
time window and/or finite maximum measurement count rather than allowing
memory and solve cost to grow indefinitely.

The initial implementation may use internal engineering defaults for:

```text
minimum collection interval
maximum retained interval
maximum retained measurements
```

These are tuning parameters, not part of the first public API. Exact default
values should be selected from deterministic replay tests.

When the retained window is full, old measurements should be discarded so
that the static assumption applies to a bounded interval.

No satellite-specific condition, including any rule involving G07, may be used.

## 18. Solver structure

The first `STATIC` solver should intentionally reuse the current calibration
model and local LS behavior. The primary algorithmic change is that each solve
uses measurements from multiple epochs.

Current behavior:

```text
one epoch
  -> build_meas()
  -> yaw multistart
  -> iterative wrapped LS
  -> optional EKF
```

New static behavior:

```text
bounded multi-epoch measurement window
  -> build_static_meas()
  -> same yaw multistart
  -> iterative wrapped LS
  -> validate
  -> freeze accepted state
```

The initial yaw search should remain:

```text
-180, -165, ..., +165 degrees
```

with roll and pitch initially zero for `BOTH`/`RPY`, matching the existing
initializer.

Roll/pitch global search, alternative optimizers, robust losses, and changed
phase-unwrapping schemes should not be introduced in the first static
implementation. Keeping the numerical model constant makes it possible to
attribute any improvement specifically to multi-epoch observation diversity.

The existing `sd_proj()` geometry/sign convention should be reused rather
than reimplemented.

## 19. Static residual model

For retained physical measurement row `j`, the static solver evaluates the
same wrapped residual model as the current implementation:

```text
r_j = wrap_lambda_j(
          z_j
        - (geometry_j(attitude) - bias_rfch_j)
      )
```

where `geometry_j` uses the stored LOS, current antenna baseline geometry,
and candidate attitude.

All physical phase rows should initially have equal weight.

Weighting by satellite, RF channel, C/N0, epoch, or pair frequency may be
investigated later if replay evidence shows a need. It should not be added
merely to force the current fixture toward a known answer.

## 20. Solve gating and observability

A fixed satellite count is not the definition of observability.

A static solve may be attempted after:

- a minimum collection interval has elapsed,
- enough physical measurements are available for the active states,
- the active-state design matrix is numerically solvable.

Full rank is a precondition for attempting/accepting a solution, but it is not
sufficient to declare calibration valid.

Build a Jacobian containing only active physical states, normalize its
parameter columns, and use a small-matrix rank/conditioning method that needs
no new LAPACK or BLAS dependency. Solve the same scaled system and unscale
the update. The `CONTINUOUS` solver arithmetic remains unchanged.

The implementation should retain room for a stronger conditioning or LOS-
diversity metric once actual replay behavior is measured.

## 21. Candidate isolation and acceptance

Static candidate states must not be written into the public `array->x` while
they are being tested.

The flow should be:

```text
local candidate state
        |
        v
wrapped LS iterations
        |
        v
post-fit physical residuals
        |
        v
validation
   |         |
 reject    accept
   |         |
collect     publish x[]
more        mark valid
```

This prevents a wrong wrapped-phase branch from being temporarily consumed by
beamforming or spatial processing.

For the static solver, RMS should be computed from physical carrier-phase rows
only. Pseudo-constraint rows used to freeze/regularize states must not be
included in the RMS denominator.

This also removes the diagnostic discrepancy observed in the failed epoch,
where the current LS reported about 46.14 mm while the physical measurement
rows had about 48.78 mm RMS.

The initial final residual gate remains:

```text
physical RMS <= MAX_RMS = 0.020 m
```

## 22. Static completion semantics

When a static candidate is accepted:

```text
array->x            = accepted candidate
array->rms          = physical post-fit RMS
array->calib_valid  = 1
array->calib_source = STATIC
array->calib_run    = 0
```

The accepted attitude and RF-channel biases remain fixed.

The static algorithm does not need an EKF covariance for subsequent tracking.
The current use of non-zero entries of `P` as an implicit
"initialized/not-initialized" discriminator should therefore be confined to
the `CONTINUOUS` implementation and must not become common calibration
lifecycle logic.

## 23. Algorithm dispatch

The public calibration entry point should become an algorithm dispatcher:

```c
void sdr_array_calib(...)
{
    if (!array->calib_run || ...)
        return;

    switch (array->calib_alg) {
    case SDR_CALIB_ALG_CONTINUOUS:
        calib_continuous(...);
        break;

    case SDR_CALIB_ALG_STATIC:
        calib_static(...);
        break;
    }
}
```

The existing `kf_init()`/`kf_update()` behavior can remain inside
`calib_continuous()` with minimal numerical change.

A later `STATIC_DYNAMIC` implementation can reuse the static solver and then
enter a separate dynamic tracker without restructuring the selector.

## 24. Start, stop, clear, and load semantics

Control behavior should be explicit:

| Operation | CONTINUOUS | STATIC |
| --- | --- | --- |
| Start | reset estimator state, begin current init/EKF path | clear static collection, begin collecting |
| Stop | stop updates and retain last accepted state | stop collection; remain invalid if no static solution was accepted |
| Clear | clear calibration validity/state | clear calibration validity/state and collection buffer |
| Successful solve | continue EKF | publish state and stop automatically |
| Load | load accepted state, not running | same; loaded state is independent of selected algorithm |

A failed static candidate must not survive a Stop as a valid state.

For `STATIC` with `RPY`, Start requires an accepted bias state. Copy those
biases into the private context before resetting the estimator state and use
that seed throughout the solve.

## 25. Geometry changes

Changing antenna positions or the enabled-element mask changes the array model
to which the calibration state applies.

Therefore a successful call that changes geometry/enables should invalidate
the current calibration and clear any static collection context.

Conceptually:

```text
geometry / enable-mask change
        |
        +--> calibration validity = false
        +--> static collection cleared
```

A calibration file may then be loaded explicitly if the user knows that it is
compatible with the new geometry.

This rule also makes the CH8 enable/disable state part of calibration
provenance rather than an incidental UI setting.

## 26. Web UI and status reporting

The Array page should expose algorithm and mode separately, for example:

```text
Algorithm: [Continuous | Static]
Mode:      [Both | Bias | Att]
```

During static calibration, useful status is:

```text
CALIB: COLLECTING
EPOCHS: <retained/collected epochs>
MEAS:   <physical phase rows>
SATS:   <distinct contributing satellites>
RMS:    <last candidate RMS or --->
```

After acceptance:

```text
CALIB: VALID (STATIC)
EPOCHS: ...
MEAS:   ...
SATS:   ...
RMS:    0.00xx m
```

The Spatial page only needs calibration validity/provenance; it should not
contain algorithm-specific calibration logic.

An uncalibrated spatial heatmap may still be displayed, but the UI should make
clear that its absolute direction is based on a nominal, uncalibrated array
state.

## 27. Save/load compatibility

The existing calibration file format should remain readable.

Additional fork metadata may be added as comment lines, for example:

```text
# algorithm=static
# rms=0.0046
# epochs=18
```

Loading an old file must continue to work.

A successfully loaded state should be represented as:

```text
calib_valid  = 1
calib_source = LOADED
calib_run    = 0
```

The algorithm currently selected for the next calibration run does not change
the provenance of a loaded state.

## 28. Future STATIC_DYNAMIC integration

The static solver should produce an accepted calibration result that is usable
as a standalone output and as a future dynamic-estimator initial condition.

Conceptually:

```text
static multi-epoch solve
        |
        v
accepted RPY_0 + RF biases + validation statistics
        |
        +--> STATIC: freeze all accepted states
        |
        +--> STATIC_DYNAMIC:
                freeze or very slowly vary RF biases
                track roll/pitch/yaw dynamically
```

The first implementation should therefore keep static solving and dynamic
tracking as separate components rather than building dynamic assumptions into
the static solver.

## 29. First implementation boundary

The first implementation following this document is complete when:

- `CONTINUOUS` preserves the current upstream-compatible behavior,
- `STATIC` is separately selectable,
- static measurements are accumulated across a bounded multi-epoch window,
- a candidate is solved using the existing wrapped-phase model and yaw
  multistart,
- rejected candidates remain private,
- accepted static calibration freezes automatically,
- calibration validity/provenance is explicit,
- geometry changes invalidate calibration,
- existing calibration files remain loadable,
- deterministic IF replay demonstrates that temporary observation-set
  dropouts no longer force the known high-residual wrapped-phase branch.

More sophisticated search, weighting, robust losses, and dynamic tracking are
outside this first implementation boundary.
