# Collision matrix 0.4.2 — causal production corrections

Status: source candidate, native validation pending. The earlier baseline and
prescribed-fixture reports remain separate failed executions. No numerical
gate, production frequency, public interface or snapshot provider is changed.

## Large-coordinate quantization

`RoundVectorToNetQuantize` narrowed its rounded grid index to `float` and
performed the division in that precision. At large world coordinates this
loses the declared 0.01 cm position grid and makes whole-grid translations
change local trajectories. The correction keeps double precision through the
same half-up rounding and division. Engine `RoundToInt(double)` returns int64;
this is a floating-point narrowing defect, not a demonstrated int32 overflow.

The new regression checks the half-grid error bound and equivariance under
whole-grid offsets through positive/negative 1e8 cm, at scales 100 and 10000.
It does not claim invariance under a fractional offset of an absolute grid.

## Sphere/sphere coarse-frame crossing

The previous sphere/sphere sweep required overlap at the end of the frame.
A fast pair can enter and leave between two separated endpoints. For example,
a relative start of -40 cm, combined radius 25 cm and speed 4600 cm/s has a
first contact after 15/4600 seconds inside a 60 Hz frame, yet is separated at
both frame endpoints. Its old endpoint rule returned a miss.

For zero relative acceleration, the correction solves the continuous relative
quadratic over the complete frame. It uses the stable near-root expression,
tests the root against the real frame interval, and computes normal and both
witness points from relative motion. A common translation is subtracted before
advancing the pair, avoiding extra large-coordinate cancellation. Shared
acceleration is supported because the relative path is still affine.

The existing accelerated bracket is retained. Its accepted midpoint now sets
both bracket endpoints: previously the returned time averaged that accepted
root with a stale upper endpoint. Accelerated enter/leave paths with separated
endpoints remain outside this limited correction.

New native regressions cover the independent first-impact distance/speed
oracle at all eight matrix frequencies and five translations, common velocity,
grazing contact, a true near miss, and the accelerated accepted-root branch.
Portable scalar reproductions demonstrate the two causes but do not establish
an Unreal or matrix PASS. The native regression IDs are
`IAmSpeed.PhysicalLaws.QuantizationLargeCoordinates042` and
`IAmSpeed.PhysicalLaws.SphereSphereContinuous042`.

## Stationary axes and fractional translations

The full a402 native matrix has 64 translation-point failures in each of
Sphere/Plane and Box/Plane. A body's zero-velocity, zero-acceleration tangent
coordinate was rounded onto an absolute world grid even though the integrator
had not moved it. For example a common Y offset of -.004 cm was rounded to
zero for the body while the analytic environment kept its authored offset.
This injects relative motion before any physical contact.

`ISpeedComponent::QuantizeKinematicState` now preserves each unmoving location
axis exactly. It tests the velocity before rounding and the live acceleration;
small real velocities or a new acceleration do not qualify. The canonical
quantizer/cache and the moving-axis grid are unchanged, and existing geometric
support preservation still applies. No new persistent state or origin anchor
is introduced. This limited correction does not promise fractional-translation
invariance for moving axes or collision-response invariance in general.

`IAmSpeed.PhysicalLaws.StationaryAxesTranslation042` checks preservation at
ordinary, fractional and large coordinates, moving-axis quantization, and
revocation when acceleration or a sub-grid velocity starts. Its native
qualification must be reported for the exact newly built source tuple.
