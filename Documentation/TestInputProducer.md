# Exact-frame test producer — unit 1

`Tests/TestInputProducer.h` provides `FTestInputProducer final : IInputProducer`.
It is test support, not a shipped input policy or a default controller source.
The caller explicitly supplies an existing Device/AI/Network identity, first
canonical frame and a complete scenario of `FInputFrame` values. Identity is
scenario provenance; the test run must identify itself as synthetic externally.

Creation validates the entire timeline before returning a producer: nonempty,
valid payloads, matching explicit identity, contiguous consumption frames,
nondecreasing source sequence and no terminal-frame overflow. It copies all
entries into private immutable storage. There is no public submission/mutation,
interpolation, clock access or missing-frame hold. Payload values, ordered edges,
reset markers and both frame numbers are preserved exactly.

Forward requests must begin at FirstFrame and remain contiguous. Replay returns
the exact original snapshot within the last 256 consumed frames. Older replay,
gaps and requests outside the sealed timeline fail without advancing anything.
The entire scenario remains stored, but its replay window matches the common
input history. `Skip` deliberately returns false without advancing: a legacy
sealed override cannot silently suppress this producer and claim parity.

The native probe connects this producer to the real `FInputStream::Consume`,
`ReadDrivingInputTargets`, `PublishCompleted`, and `FInputPresentationBindings`.
The wheeled component now uses that same small target-decoding helper. It accepts
only a valid exact-frame snapshot; invalid/missing input leaves neutral targets
and a false validity flag, preserving the existing component diagnostic. The
helper adds no slew, device selection, gameplay rule or alternative physics loop.
Game-owned example Jump/Boost slots travel unchanged through frame/history and
presentation. Their actual game physics handlers are not migrated by this unit.

The comparison fixture drives real `FDeviceInputSession` with injected samples,
including two edges between polls and pause/resume fresh-held cancellation. An
independently authored test scenario must match all canonical frame fields and
history hashes. Fast and coalesced presentation schedules observe different
serial subsets but leave identical input histories; callbacks cannot advance
producers, change history/publication or inject device values. Component writer
guards and the actual Tick/publication/legacy source wiring remain checked by
the source-boundary suite.

Native completion is explicitly simulated, not physical vehicle simulation.
The actual component still applies targets before stepping and publishes inputs
only after canonical physical snapshot publication succeeds. Native tests cannot
qualify the Unreal compilation, physics law or whole-game CI execution. No UE,
Docker, real runtime or hardware is exercised in this unit.

The legacy path remains default. `ConfigureInputProducer` remains opt-in before
possession; EnhancedInput does not feed this source. The old SLTest sealed
override is untouched. A separately reviewed unit must wire CI scenarios into
the producer route, account for existing frame offsets and bypass-slew fixtures,
and prove game-action migration without running both routes. Only after an
authorized UE 5.8.2 gate can that full integration be claimed.
