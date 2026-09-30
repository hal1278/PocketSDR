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
| existing/upstream-compatible | current single-epoch yaw-search LS | current EKF | retain |
| static | multi-epoch static solve | freeze accepted state | implement next |
| static-dynamic | same robust static initialization | track dynamic attitude | future |

Exact enum/API names are provisional. The important requirement is that this
selector remains orthogonal to `BOTH/BIAS/RPY`.

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
