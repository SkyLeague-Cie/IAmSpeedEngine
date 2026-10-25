# Collision matrix 0.4.2

Status: implemented and measured; strict matrix remains **FAIL / NOT_QUALIFIED**.
Latest evidence: [measured physics corrections](CollisionMatrix042PhysicsResults20261025.md). No inherited 0.4.1 pass
qualifies this matrix. The production canonical clock remains 300 Hz.

The CI matrix runs sealed cases through `AFastSimulation` in FastBatch, in one
Unreal world. A joined boundary unregisters each previous actor and its contact
pairs before replacing its bodies and static query authority. Canonical frame
numbers remain monotonic; an explicit test episode records its first frame and
physical step. Changing frequency never reinterprets an earlier frame's time.
After a failed episode, a fresh FastSimulation driver replaces the terminal one
in that same world. Production terminal latches are never cleared, and no
receipt is fabricated for an unpublished frame. Attempted and completed cases,
expected and actual frames, driver generations and terminal failures are
reported separately; a loop reaching its last row does not prove coverage.
This test-only episode hook does not qualify production multi-rate gameplay,
cross-rate rollback, hardware, audio, rendering or Netcode.

The declared sparse matrix contains 4,800 cases: three dynamic fixture shapes
(Sphere, Box, Wheel), five peer roles (analytic plane, static Ring, Sphere, Box,
prescribed Cylinder), eight authored variants, eight frequencies and five common
translations. Wheel uses an actual `USWheelSubBody`, real Chaos wheel and
suspension storage and the native `ISpeedWheeledComponent` support solver; this
unit fixture does not qualify an entire car. Free dynamic Ring/Cylinder and
Wheel/Wheel are outside the admitted set, not successful misses.

The same authored relative configuration is exercised at 60, 100, 120, 240, 300,
360, 720 and 1440 Hz. The highest tested rate, 1440 Hz, is the comparison
reference; comparison with 720 Hz also exposes reference convergence. Equal frame indices at different rates are
never compared as equal physical times. End states are sampled at 50 ms.
Linear and angular velocities, initial orientation and relative offset vary in
an explicit sparse design; this is not the complete Cartesian product.

For translation invariance, move both the actors and the analytic environment
by a common offset. Compare `ImpactPoint - offset`; compare ImpactNormal, impact
time in seconds and post-impact linear/angular velocities without translation.
On-grid offsets and a fractional-grid offset are distinguished. Translation is
not a test in which the environment remains fixed. Large-coordinate precision
is reported, not hidden by a success label.

First selected contacts are copied from the actual CCD resolution, before and
after response. The observer performs no replacement query and supplies no
synthetic hit. Raw rows retain the local case frame, within-frame elapsed time,
geometry witnesses, feature identity, and both actors' post-response velocities.
End-state comparisons are separate from immediate impact response.

Fixture planes explicitly opt into analytic authority, just like the existing
native plane law fixtures. The diagnostic-only default is not a blocking floor.
The short smoke checks two joined cases in the same world, including a common
on-grid translation; it does not qualify the complete matrix.

Tolerances are frozen before native qualification in the native test and exported
as `spec.json`. The position grid is
0.01 cm (`FKinematicState::PositionQuantizationScale`). A frequency comparison
must account explicitly for the worst-case accumulated half-grid errors of
the two finite runs, multiplied by sqrt(3) for its Euclidean bound. The geometric
part is 0.02 cm; the normal bound is 0.002 radians. This bound is separate from
the observed discrepancy. Translation uses 0.0001 cm, 1 microsecond, 0.000002
radians, 0.01 cm/s and 0.001 rad/s; fractional-grid cases keep the strict gate and
remain failures if the existing absolute quantization changes the trajectory.
Normal
orientation, contact presence/topology, unsupported admissions and iteration
exhaustion remain independent gates; they cannot be waived by a position bound.
Analytic first-impact oracles are required for non-rotating axis-aligned cases.
The fine-rate reference alone is never an independent physical oracle.

Ring is admitted as a static analytic torus, built from `URingSubBody`.
Cylinder is a prescribed obstacle; free dynamic cylinders and moving rings are
not claimed. Wheel cases must use real wheel and suspension storage. Unsupported
roles must be identified explicitly rather than counted as successful misses.
Open Wheel CCD debts and existing laws remain unchanged.

All native pair response containers explicitly block. The base sub-body default
ignores dynamic channels; Cylinder checks both containers. Leaving that default
in a blocking fixture would test filtering rather than a blocking collision.
The original failing run is retained before this fixture correction.

Wheel suspension support is exported separately in `support-samples.csv`, from
the actual native frame-end support solve. Its `sample_seconds` is a frame sample;
`query_parameter` is the unmodified native support-query value. Neither is relabeled
as a CCD impact time. `events.csv` and its strict gates continue to require a real
selected CCD resolution. A support sample cannot waive a missing CCD contact.

The CI also exports independent diagnostics on failure. They separate incomplete
episodes, missing reference contacts, input equivalence and measured point,
normal, time and response differences. Categories overlap and grant no PASS.
Common translated initial positions are checked independently of output
invariance. `execution.json` counts completed cases and actual frames, rather
than treating an attempted loop as complete physical coverage.

Stationary prescribed Cylinder owners reject generic state writes and bypass
free-body quantization, matching the production prescribed-owner contract.
The artifact gate independently requires their final relative position and
linear/angular velocity to remain the authored zero values. A deflected fixture
is identified as a fixture defect, not accepted as a mechanical track result.

Paired suspension diagnostics compare first observed support points/normals and
report the difference between frame-sample times. They also compare both actors'
velocities at the common 50 ms endpoint. These additional measurements introduce
no new acceptance threshold or PASS, and exclude incomplete, mis-timed or
deflected-prescription cases. They do not reinterpret a support time as CCD TOI.
