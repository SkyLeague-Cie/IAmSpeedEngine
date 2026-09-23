# Portable session lifecycle protocol

`World/Simulation/Lifecycle/SessionLifecycle.h` is an additive, inactive protocol
model. Nothing in the existing worker, module, controller or physical simulation
includes or activates it. `Tests/SessionLifecycleProbe.cpp` contains authored
deterministic traces; they have not been compiled or executed at this checkpoint.

## Authority and ownership

One serialized owner calls all methods except `RequestStop`. Only that stop
method is intended for concurrent callers. A lock-free atomic word combines the
stop latch and admission bit: an admitted reservation wins before stop, or stop
wins and admission fails. A stop after admission preserves the reservation for
resolution. This is source-level protocol design, not a tested OS worker or an
Unreal lifetime guarantee. Unsupported platforms without lock-free uint32
atomics fail at compilation; no portability qualification is implied.

Participant retire calls belong to the serialized owner. No claim is made that
GT, hotplug and reaper callers can concurrently mutate this object. A future
host must serialize those requests while retaining the object, and must prove
its own delivery and shutdown guarantees before integration.

Fixed participant storage bounds configuration. Registration is sealed at
Start; adding runtime devices or physics participants is not implemented here.
A detached slot may be reused during configuration only, with an incremented
generation. Exhaustion rejects registration without modifying an existing slot.
Stop uses a dedicated atomic flag; retirement uses its participant's own state.
Neither operation competes for a queue slot or allocates. This model contains
no general command queue; it does not prove overflow safety for a future one.

## States and witnesses

Session: Configured -> Running -> StopRequested -> Draining -> Stopped ->
ThreadExited -> Reaped. Cancelling an unstarted configuration may also go from
Configured to StopRequested. StopRequested is the observable atomic latch while
the owner has not entered Draining. Repeated requests return false without
undoing the original request. Pause closes ordinary admission but remains a
separate flag, never proof of stopping; stop works while paused and forbids resume.

Participant: Active -> RetireRequested -> Draining -> Detached. Session draining
requests retirement of every still-active participant. Detachment is rejected
while an unresolved reservation includes that participant. The remaining active
participants can continue after individual retirement when the session is not
stopping. There is at most one whole-session reservation; its fixed mask names
all participants active at admission.

Reservation: Reserved -> Committed -> Published -> Completed. Reserved or
Committed may instead become Aborted, which also closes session admission.
Published cannot abort. These are explicit caller-supplied witnesses, not an
implementation of application, global publication, rollback or resource removal.
The host must deliver truthful witnesses from its real transaction. This model
does not call the existing V2 stream or A5b1 hooks and cannot prove their
allocation-free finalization.

Stopped requires every participant Detached and every reservation resolved.
ThreadExited is accepted only after Stopped; Reaped only after ThreadExited.
The future service must obtain actual exit/join evidence, not infer it from
these method names or from `bRunning=false`. Session identity includes an opaque
shared marker and a nonzero epoch; equal numeric epochs from different objects
do not match. Participant generations and reservation serials reject stale
acknowledgments. Counters do not wrap into reused identities.

`FCodeLease` is a portable marker, not a loaded-module pin. The protocol retains
its shared lifetime until Reaped. It does not authorize unloading a kernel,
destructor or provider module. A future service must prove code-module shutdown
ordering separately. An unstarted, uncancelled configuration may be discarded;
destroying an activated/cancelled object before Reaped is an owner error and
terminates instead of silently releasing a potentially live lease. There is no
implicit stop, join, spin or cleanup retry in the destructor.

## Authored verification scope

The standalone probe models stop before admission and at reserved, committed,
published and completed phases; mandatory resolution before detachment/stopping;
pre-publication abort and post-publication abort rejection; individual retirement
while other participants continue; pause/stop separation; default, foreign,
stale-generation and stale-reservation rejection; duplicate acknowledgments;
fixed-capacity overflow and the 64th mask bit; and sole-owner lease survival
through Stopped and ThreadExited until Reaped. It uses no sleeps or threads.

Not covered: real concurrent scheduling, OS worker lifetime, hotplug delivery,
UObject/ISpeedComponent/USSubBody ownership, network callbacks, actual module
unload, real physics, camera producers, bindings or mapping parity. Those remain
separate source and runtime gates. No V2 activation or forced-teardown safety
claim follows from this portable unit, even after its eventual tests pass.
