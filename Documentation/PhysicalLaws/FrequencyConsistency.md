# FrequencyConsistency

Identifier: `IAMSPEED.PHYS.FREQUENCY_CONSISTENCY.V1`.
CompositionLevel: System. Status: **draft, not qualified**.
Target line: IAmSpeed **0.4.n** stabilization. Network restoration belongs to **0.5**.

## Chosen contract

For an admitted physical model and declared geometry/motion domain, the same
physical experiment at **60, 100, 120, 240, 300 and 360 Hz** should produce very
similar collision and motion behavior. In particular, low-frequency
`ImpactPoint` and `ImpactNormal` must remain close to the geometric oracle or
a demonstrably converged fine-step reference. This is bounded physical
consistency, not bit identity between different integration rates.

This target complements [the existing laws](README.md). It does not replace
ContactDetection, NormalResponse, Friction, Sliding, RestingEquilibrium,
BoxStandingStill, BoxEquilibrium, Rolling, Support, MultiContactEquilibrium or
Suspension, nor relax their accepted assertions.

## Preconditions and episode identity

A case declares the supported body pair, finite surface domain, world geometry,
material/force model, mass, inertia, COM, units, initial pose and linear/angular
state, physical duration, input timeline and numeric context. Geometry and
physical coefficients stay the same at every rate. An unsupported pair or a
query-budget exhaustion is an explicit result, never a certified miss.

The rate is fixed for an episode and belongs to its configuration/checkpoint
identity. A frame number at 60 Hz and the same number at 360 Hz refer to
different physical times. Snapshot restoration must not silently reinterpret
one rate's state or input/history indices at another rate.

First strict fixtures can place command changes on shared 50 ms boundaries.
Separately test phase-shifted input admission and report its timing error;
do not attribute that error to the collision kernel. Forces and damping use
declared physical units; they are not retuned to match each frequency.

## Observables and comparison classes

| Observable | Required measurement |
| --- | --- |
| Contact validity | Hit/miss, admissible provider/feature, ordered events; no missed or invented obstacle hidden by an average |
| Impact time | Absolute physical event time, including intra-frame TOI, in seconds; normalized per-frame alpha alone is insufficient |
| ImpactPoint | World-space distance in cm and scale-relative error at the matching event |
| ImpactNormal | Angular error in degrees, consistent orientation and coherent contact witnesses |
| Penetration | Maximum/residual depth at publication and along the declared CCD trajectory; clear endpoints alone do not prove a clear segment |
| Response | Post-impact linear/angular velocity, impulse, restitution and energy balance under the chosen law |
| Persistent motion | Pose/quaternion angle, slip, support, rolling, suspension and release at common physical times |

For a smooth face, the point and normal must approach the same independent
geometric result within the case's bound. At a real edge, exactly simultaneous
contacts or a genuine physical bifurcation, define the admissible feature/
normal set and canonical tie-break rather than manufacture a smooth normal.
Perturbation tests must distinguish numerical instability from a physically
unstable configuration such as a box at its tipping boundary.

Measure first impact separately from a long chaotic multi-impact trajectory.
Retain the first divergent event/field, maxima and percentiles, geometry,
inputs and configuration. A close terminal pose does not excuse a wrong CCD
event, a penetrated segment or an invalid normal response.

## References and tolerances

Prefer independent analytic oracles for solvable cases. For composed cases,
check convergence of a fine-step reference with at least one still finer step.
A run at 9 kHz is not automatically an oracle or proof of real-time throughput.
The 9 kHz objective is separate from this six-rate acceptance matrix.

Every blocking case defines its tolerance **before qualification**: unit,
geometry scale, numerical/geometric error, publication quantization, duration,
and observable. An unspecified tolerance or incomplete reference keeps the case
unqualified. Do not widen tolerances after a failure or reuse one global depth
allowance for contact shells, exact rest, geometric kernels and publication.

## Initial case families

- `IAMSPEED.PHYS.FREQUENCY_CONSISTENCY.V1.SPHERE_PLANE`: analytic first impact
  under fixed translation, oblique and phase variants.
- `.BOX_FINITE_PLANE`: finite OBB witnesses, rotation/COM, real edge and miss
  controls, symmetry and off-grid geometry.
- `.CURVED_CONTACT`: declared smooth providers, varying normals, contact
  transport, tangence and true endpoint release.
- `.RESPONSE`: restitution, normal impulse and passive energy checks.
- `.FRICTION_ROLLING`: stopping distance, slip/roll and incline transitions.
- `.SUPPORT_EQUILIBRIUM`: supported rest, acquisition where guaranteed, changed
  load, impulse, finite support loss and multi-contact feasibility.
- `.SUSPENSION`: support, travel, damping, bump/rebound and chassis reaction.
- `.SENSORS`: complete crossing, ordered begin/continuous/end and filtering.

Apply cases to the actual admitted SubBody domains. Static rings and prescribed
cylinder obstacles do not acquire unsupported free rigid-body behavior merely
because their types are present in the API. Existing pure geometry tests,
component tests and native host tests remain distinct proof levels.

## Determinism and execution mode

At each individual rate, compare real-time and Fast canonical progression with
the same consumed commands and causal state; camera follows its mode contract.
Also compare scalar and admitted parallel execution, identifying requested,
admitted, participating and useful consumed work separately. Exact ordering/FP
requirements within a supported numeric configuration remain in force.

A restore/replay test compares against uninterrupted progression **at the same
rate**. These tests establish foundations useful to future netcode; they do
not certify operational multiplayer, prediction or network reconciliation.

## Current implementation and coverage

The [canonical frame context](../../Source/IAmSpeed/World/Simulation/CanonicalFrameContext.h)
currently fixes 300 Hz. The
[simulation driver](../../Source/IAmSpeed/World/Simulation/SpeedSimulation.cpp),
[contact transport](../../Source/IAmSpeed/Components/ISpeedComponent.cpp),
[box support reacquisition](../../Source/IAmSpeed/SubBodies/Solid/BoxSubBody.cpp)
and timing/fixture consumers need a coherent rate audit before the other rates
can be advertised. Changing only the old host physics setting is insufficient.

The [ContactDetection catalogue](initial-laws.md#open-wheel-ccd-debt) also
records open wheel-query overlap evidence. Cross-frequency work must retain
those failures and add generic reproductions; it does not retroactively
qualify wheel non-penetration.

Existing 300 Hz Normal/Fast/parallel fixtures are scoped evidence for their
listed sources and cases. Long tests containing 9000 frames do not prove a
9000 Hz timestep. The six-rate whole-engine matrix is **NOT_RUN**.
