# IAmSpeed 0.4.n stabilization milestones

Netcode restoration belongs to **0.5**. This plan prepares independently usable
physics foundations for IAmSpeedDemo and consuming games such as Sky League.
It does not assign a release date or transfer qualification between sources.

## First deliverable: an independent 300 Hz demo

Restore the v0.1 demo in its own repository with the selected IAmSpeed source,
a native Pawn/Controller/GameMode, one canonical simulation owner, device input
and one camera endpoint. Preserve the legacy content while migrating its launch
path. Admit a small finite Analytic world through the existing immutable bake
contract; a visible mesh without admitted collision data is not a playable demo.

Acceptance requires a fresh native build, matching authored/baked/cooked source
inventory, driving and braking with keyboard/gamepad, stable publication and
camera, pause/focus/reconnect checks, repeated reset with old owners joined and
new input epochs, and a human controller trial. No Sky League map, gameplay
class or input implementation may be required. This milestone is **not yet
qualified**; source migration and a statically applicable patch are preparation.

## Second deliverable: portable physical contract qualification

Start with [the current laws](PhysicalLaws/README.md), their real admitted
SubBody pairs and independent geometric oracles. Separate geometry queries,
component behavior, native host execution and human playability. Keep the open
wheel CCD overlaps described in [ContactDetection](PhysicalLaws/initial-laws.md#open-wheel-ccd-debt)
as failures to reproduce and resolve, rather than declaring non-penetration
from the architecture alone.

Freeze each case's geometry, physical model, observable, comparison rule and
tolerance before qualification. Prioritize first impact, bounded support and
release, response/energy, friction/rolling, suspension and sensors. Preserve
miss controls, initial overlap handling, high linear/angular speeds, off-grid
poses, symmetry and the first divergent field. Existing CI failures are not
waived by a new suite.

## Third deliverable: a coherent episode rate

Qualify [FrequencyConsistency](PhysicalLaws/FrequencyConsistency.md) at
60/100/120/240/300/360 Hz. First reconcile every consumer of physical time:
forward/replay frame context, real-time pacing, Fast windows, consumed commands,
history/snapshot identity and capacity, contact transport, timing and readers.
Changing only Unreal's physics setting is insufficient while the canonical
context fixes 300 Hz. Keep the current 300 Hz numeric path as the control.

At a fixed rate, retain the existing strict Normal/Fast, scalar/parallel and
restore/replay requirements. Between rates, compare events and states in
physical time with per-case bounds, especially ImpactPoint and ImpactNormal;
do not demand cross-rate bit identity. A 9 kHz throughput goal is separate from
reference convergence and from these six supported target rates.

## Integration boundary

Each source lot has its own build/tests and review. Documentation can ship
before the demo, and the first 300 Hz playable demo can ship before six-rate
qualification. A future networking lead can consume these contracts for 0.5;
the stabilization work does not claim operational multiplayer.
