# Collision matrix 0.4.2

Status: implementation in progress, **NOT_QUALIFIED**. No inherited 0.4.1 pass
qualifies this matrix. The production canonical clock remains 300 Hz.

The CI matrix runs sealed cases through `AFastSimulation` in FastBatch, in one
Unreal world. A joined boundary unregisters each previous actor and its contact
pairs before replacing its bodies and static query authority. Canonical frame
numbers remain monotonic; an explicit test episode records its first frame and
physical step. Changing frequency never reinterprets an earlier frame's time.
This test-only episode hook does not qualify production multi-rate gameplay,
cross-rate rollback, hardware, audio, rendering or Netcode.

The same authored relative configuration is exercised at 60, 100, 120, 240, 300,
360 and 720 Hz. The 720 Hz case is the comparison reference; an additional 1440
Hz run checks reference convergence. Equal frame indices at different rates are
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
geometry witnesses, feature identity, and both actors' pre/post velocities.
End-state comparisons are separate from immediate impact response.

Tolerances must be frozen before native qualification. The position grid is
0.01 cm (`FKinematicState::PositionQuantizationScale`). A frequency comparison
must account explicitly for the worst-case accumulated half-grid errors of
the two finite runs; this bound is separate from the observed discrepancy.
Translation tests on grid-aligned offsets use a tighter numeric bound. Normal
orientation, contact presence/topology, unsupported admissions and iteration
exhaustion remain independent gates; they cannot be waived by a position bound.
Analytic first-impact oracles are required for non-rotating axis-aligned cases.
The fine-rate reference alone is never an independent physical oracle.

Ring is admitted as a static analytic torus, built from `URingSubBody`.
Cylinder is a prescribed obstacle; free dynamic cylinders and moving rings are
not claimed. Wheel cases must use real wheel and suspension storage. Unsupported
roles must be identified explicitly rather than counted as successful misses.
Open Wheel CCD debts and existing laws remain unchanged.
