# Boolean delivery foundations — source continuation

This continuation does not activate a V2 controller, Windows acquisition or game
consumer. Those integration gates remain open. Portable evidence is recorded by
the parent task; it does not qualify an Unreal target or a mechanic effect.

`FInputEdgeIdentity` is authored from an immutable valid frame and transition
ordinal. It includes producer kind/id, stream epoch, physical frame, source
sequence, ordinal and action. A producer must never reuse its epoch for a new
stream lifetime. Different producers cannot alias their edges even if all their
clock values coincide. Started/Completed presentation events carry this same
identity; held Triggered has no edge identity. Observers cannot use the identity
as permission to apply an authoritative effect again.

V2 publication has three explicit operations. PreparePublication allocates both
future publication and continuity copies before the global snapshot is made
visible. IsPublicationPrepared only validates the exact reservation. After
global publication, FinalizePublication performs checked noexcept swaps and
scalar updates, with no allocation or callbacks. Old swapped-out frame storage
survives until a later preparation/destruction. The convenience PublishCompleted
wrapper still serves portable callers and is unsuitable for a global commit
hook. The host integration must use the split operations.

Stop after reservation/preparation preserves the owner's capability for drain.
Abort invalidates preparation without publishing. Wrong/stale tokens cannot
affect another reservation. Consume stages all potentially throwing copies and
the token before installing history/reservation, so allocation failure cannot
strand an inaccessible token. Dispatcher continuity is copied before callbacks
and swapped only with cursor advancement; resynchronization also stages its copy.

InputTransactionProbe injects each acquisition/preparation allocation failure,
forbids allocations during finalization, and checks dispatch copy failures do
not follow already delivered callbacks. BooleanCapacityProbe covers 255/256/257
publications, 63/64/65 transitions and 255/256/257 raw changes. Overflow rejects
the whole batch/frame and reports resync rather than acknowledging a suffix.
Resync is a fresh held baseline without invented edges. A presentation gap never
changes the already completed authoritative transaction outcome.

The same-frame portable press/release tests establish ordered event transport,
not game eligibility, actual jump/flip, powerslide forces or camera behavior.
Native game tests and integration remain mandatory.

## Portable lifecycle boundary

V2 producers now expose explicit quiescent pause and cancellation operations.
Sealed timelines can report Unaffected; device raw sources must positively
implement pause/resume or are rejected. Applied device lifecycle changes require
a fresh raw Resync baseline while retaining the next physical address.
Stream lifecycle calls reject outstanding reservations without touching the
source. Paused Consume returns Paused without polling/advancing. Old held state
is hidden until a fresh publication; an observation cursor preceding the pause
barrier must explicitly resynchronize rather than replay pre-pause edges.
Cancellation executes outside the no-allocation finalizer, after drain/abort;
successful cancellation is idempotent and rejected cancellation can be retried.

These APIs are tested with a synthetic raw source only. Windows raw-source
adaptation, controller/world quiescence, physical latch neutralization and a
pause-independent control-plane remain unimplemented. No Skip implementation is
added: host ownership of a sealed test source versus a device stream still needs
explicit migration, rather than silently skipping real input history.
