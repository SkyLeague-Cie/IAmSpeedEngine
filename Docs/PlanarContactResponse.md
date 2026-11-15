# Coupled speed limits and regularized planar hitbox contacts

The default planar box/static-world impact response now solves normal impulses,
Coulomb tangent impulses and game speed-limit reactions in one iteration. Trial
velocities remain local to the solver. The canonical state is updated only after
the contact residual and both speed bounds have passed. This does not defer speed
limits to the end of a frame and does not advance collision detection at a trial
velocity above a limit.

## Defaults and comparison controls

- `p.IAmSpeed.Contact.CoupledSpeedLimits=1`.
- `p.IAmSpeed.Box.ContactResponseTime=0.001` seconds.
- Setting the response time to zero selects rigid contacts with coupled limits.
- Setting coupled limits to zero selects the historical planar impulse path.

The integration currently covers the existing free-body box/analytic-plane
response. Existing admission checks still exclude dynamic pairs, varying normals,
and bodies with other active physical constraints. It does not claim to replace
the complete wheel, ball/car or coupled-body solver.

## Normal compliance model

This is a regularized rigid-body contact constraint, not a volumetric deformation
model. The fixed material horizon T is independent of the simulation frame dt.
For each normal row with effective inverse mass K, effective spring and damping
are k=1/(K*T*T), c=2/(K*T). The backward response over T gives CFM=K/3.

An incoming vertex at positive gap g can join the contact candidate set only if
g <= max(0,-v_point.normal)*T. Each candidate is separately checked against the
same admitted analytic surface. No TOI, pose or quaternion is snapped or shifted.
Its velocity floor is -g/T. At actual geometric contact the floor is zero, so the
regularization cannot make the body cross the contact plane. The normal update
combines the compliant restitution target and this unilateral floor.

This explicitly changes the impact law and contact activation model. Input
restitution is retained as a target bias, not a guarantee that the compliant
outgoing point speed equals the rigid Newton restitution target. T=1 ms is an
initial material model choice, not a parameter identified from Rocket League.

## Friction and speed constraints

The point velocity is v + omega cross r. The tangent update uses its complete
two-dimensional effective-mass matrix and an accumulated circular Coulomb budget
|Jt| <= mu*Jn. Normal impulses are nonnegative.

Linear speed is projected onto its admissible ball. Angular speed is projected
onto its admissible ball in the inertia metric. Their accumulated reactions are
removed before re-projecting at the next iteration; they are not spent again as
new independent impulses. Speed-limit reactions are external game constraints
and are returned separately from normal and tangential contact impulses.

The solver has a finite 512-iteration budget. It rejects nonfinite inputs,
non-positive inverse inertia, invalid mass/limits and an unresolved residual.
A refusal is explicitly logged; the existing caller retains its fallback path.
Tests must count refusals rather than treating fallback as successful use of the
new solver.

## Validation so far

The compiled native self-tests cover hard and soft edge solves, energy, contact
order, translations, three scene rotations, invalid mass, and joint linear and
angular limits (11 checks). Five fresh native captures cover hard Pitch, default
Pitch/Roll, and NormalLanding/TapJump guards, all with zero solver refusals.

At the first Pitch impact the regularized candidate set contains both nearly
simultaneous front-bottom corners. The measured yaw velocity immediately after
the event is approximately 0.003902 rad/s instead of 1.049805 rad/s historically.
Full trajectory agreement with RL is not acquired: Pitch orientation improves,
while its maximum COM error worsens. RL inertia, native position datum and contact
manifold remain unknown. No fitting, time shift, reference replacement or inferred
RL torque is part of this implementation.

## Box equilibrium regression correction (5 November 2026)

The first generic bounded iteration lost the existing certified plastic
edge/face endpoints. A small iterative residual was not equivalent to an exact
zero mode: it could sustain rocking or exhaust the CCD event budget. An endpoint
from the analytical Coulomb solver is now accepted only after checking the same
normal complementarity, including CFM and gap floors, and both velocity limits.
Otherwise the bounded iteration remains responsible for the result.

The actual manifold's existing transform-roundoff classification is preserved.
A corner already certified touching is not recast as separated because its
computed distance contains a positive rounding residual.

Near-corner speculation is restricted further: when the exact actual-contact
response is already admissible under both speed limits, separated corners do
not add speculative impulses. Ordinary CCD owns their actual arrival. The
clustered/cap-coupled response remains when a bound would be active or the exact
endpoint cannot be certified. Thus the 1 ms option is not an unconditional
replacement of free-box collision mechanics.

Native validation, unchanged BoxEquilibrium.CanonicalDrops criteria, 215 cells:
initial defaults 101 errors; coupled rigid 24; legacy control 0; first endpoint
correction 10; complete correction 0. The final run has no observed penetration,
no CCD-budget errors and no rejected bounded solves. Every cell holds its
required terminal interval (at least 900 stable frames in the recorded run).
The 11 native bounded-contact checks also pass. An affected actual PitchLanding
guard keeps the first-impact yaw near 0.003902 rad/s, while its complete trajectory
can change; no transfer of the previous RL curve qualification is implied.

Evidence: private scopes Box3031, Box3032, Box3033, Box3036, Box3039 and Box3040;
summary Box3041CIRegressionFixed. The headless adapter calls the existing test
without modifying assertions. Its recorded error count, not process exit alone,
decides success. Editor startup failure Box3026 and harness compilation failure
Box3028 remain preserved. This is focused private native validation, not full CI
or a common publication.
