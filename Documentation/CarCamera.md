# Car camera policy (additive extraction)

`FSpeedCarCameraAim` is a values-only ground/aerial heading and rear-view kernel.
It advances once per consecutive physical frame at 1/300s, from copied car
orientation/velocity, ground/rear input, settings and complete direction history.
Rear view preserves the car direction history and adds 180 degrees of yaw to
the current car-view rotation. There is no ball, game target, live component,
world query or configuration read in this kernel.

This first checkpoint is not connected to ASpeedCar. It supplies neither a
final camera pose nor collision, command transport, output codec or authenticated
rollback. Copying its state is an in-process test/initialization operation, not
permission to import arbitrary wire state. Invalid input or a frame gap closes
the timeline and leaves the caller's output untouched.

The source-only connection preparation separates `FSpeedCarCameraAimHistory`
(eight direction fields) from the standalone `FSpeedCarCameraAimState` timeline.
`AdvancePolicy` advances caller-owned history at fixed1/300 without standalone
admission checks. Its outer caller must own validation and failure ordering;
it must not persist a second aim timeline. The temporary arithmetic context
does not register a producer or retain state between calls. The guarded `Step`
entry and its arithmetic are unchanged. This seam is not activated in a game
solver yet, and the new boundary native tests have not been compiled or run.

The next integration will place the generic arm/producer and component under
ASpeedCar, with virtual specialization for game targeting. Pose computation
stays on the canonical lane; the GT component reads a matching immutable
car-relative snapshot. Publication and physical hashes remain separate as
specified in PresentationBoundary.md. No activation or runtime equivalence is
claimed from this additive source checkpoint.
