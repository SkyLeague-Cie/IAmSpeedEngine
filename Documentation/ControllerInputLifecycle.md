# Controller input lifecycle — L2 source proposal

Not compiled or executed. L1 remains separately source-accepted. No mapping,
device backend, OS callback shutdown or default activation changes are included.

## Protocol

`ESimulationQuiescence` separates BoundaryAcknowledged, AlreadyStopped,
TimedOut and Failed. `TryPauseOwnedSimulation` requests pause and uses the
worker's bounded acknowledgement. A stopped/unstarted/missing owned worker
returns AlreadyStopped; an active legacy Unreal async host without an owned
boundary returns Failed. Neither state authorizes mutation of physical state.
Timeout/failure retains the pause request and cannot trigger automatic resume.
Existing no-producer pause behavior remains unchanged.

For an installed producer, controller pause obtains quiescence before calling
the stream's lifecycle control, then requests Unreal pause. If Unreal rejects
pause, source resume establishes a new neutral generation before resuming the
worker. Successful unpause similarly resets the source before worker resume.
World-pause synchronization respects a latched lifecycle fault. A later explicit
pause retry can obtain a valid acknowledgement; an unpause after a failed
external-world synchronization must obtain that acknowledgement too. A repeated
already-paused Unreal request must never be mistaken for a rejected pause and
resume its worker. Source success alone cannot clear a failed owner ack; a failed request is not silently
converted into success by a repeated same-state call.

`EInputLifecycleResult` is Applied, UnaffectedByPolicy or Rejected. The base
producer returns Rejected: missing lifecycle implementation must fail closed.
Only FQueuedInputProducer (including AI/network subclasses) and the sealed
FTestInputProducer explicitly return UnaffectedByPolicy to preserve their
frame-addressed authored timelines. FDeviceInputSession implements real reset.
FDeviceInputProducer, FDeviceDiscovery and the Windows acquisition/discovery/
selected-source classes remain Rejected until D1 supplies explicit live hooks.
The counted W4/U3/L1 and standalone probe wrappers are not qualified controller
lifecycle sources; their existing tests do not install them through this route.
The independent V2 producer API is unchanged. No current parent 9b506 source
installs another custom producer via ConfigureInputProducer; later CI adapters
must declare a policy when integrated. Gating never rewinds a sealed timeline.

The stream serializes control with Produce/publication. Paused/closed streams
reject acquisition and publication. Control epochs invalidate unpublished old
frames without editing historical values or Latest. Mandatory Deactivate closes
the stream before cancelling its source, including during presentation cleanup.
The session closes admission even if generation exhaustion makes reset fail.
Rejected control leaves the stream closed to polling/publication and is visible
to the controller. Backend handles and callback-unregistration retries belong
to D1, not this values-only cancellation hook.

Possession establishes a fresh baseline. Unpossession and EndPlay close source
and stream before controller ownership is released; retained shared references
remain alive but inactive. The controller's presentation tick skips a suspended
stream. No lifecycle path calls a physical input setter, polls a device, edits
WheeledUserInput or invents a completed frame. The physical targets become
neutral on the first real resumed frame if no fresh reading exists; a fresh
held reading applies on its first eligible real frame. Historical snapshots
remain observations of past frames throughout pause and teardown.

## Authored automation

Exact selection: `IAmSpeed.Simulation.ControllerInputLifecycle`, manifest
`Tests/ControllerInputLifecycleAutomation.json`. It uses a real world, game
mode, controller possession, device session, stream, owned worker and canonical
consumer. A counted forwarding source records lifecycle calls and injects one
explicit Rejected response. There is no TestInputProducer substitution.

Three real frames prove held baseline, neutral resume without a fresh reading,
and immediate fresh-held target. The actual controller pause/resume route
proves source generation invalidation and stream gating without extra polling.
A separate no-game-mode world exercises Unreal's genuine pause rejection:
standalone GameMode ignores `bPauseable=false`, so that property cannot author
the required rejection. Its controller has a real PlayerState and its pawn is
fully spawned; absence of GameMode is the isolated missing dependency. A controlled in-flight worker delays acknowledgement
to exercise timeout, no lifecycle mutation/no automatic resume, then an explicit
successful retry. Joined-worker cancellation covers AlreadyStopped. Rejected
source control must remain closed and can be retried explicitly.

Unpossession and the actual EndPlay hook close retained streams and preserve
historical copies. EndPlay is invoked directly in this pre-BeginPlay fixture;
this is not a qualification of all engine destruction orderings or GC races.
The existing owner reference keeps source state alive during in-flight use;
this does not qualify Windows backend shutdown or module unloading.

Source guards are authored separately and remain NOT_RUN until permitted.
Compilation and native runtime remain NOT_RUN. Plan one reviewed L1+L2 build,
then two separately selected fresh runtime reports and scoped verdicts.
