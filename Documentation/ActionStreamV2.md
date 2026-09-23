# Common canonical stream and presentation — A5a transaction source checkpoint

Status: A5a source prepared, not compiled or executed. A3/A4 historical evidence does not qualify the changed transaction API. All types remain in V2;
the existing player path, A1 schema/fingerprint, A2 response and v1 fixtures
are unchanged. No controller, platform backend or game asset activates this.

Both `FDeviceInputProducer` and the sealed `FTestInputProducer` implement the
same `IInputProducer`. Device acquisition exposes a value-only `IRawInputSource`
polled by the physical consumer; its readings traverse the A2 mapper. The test
producer begins after mapping and never invokes a second physics implementation.
Both feed the same `FInputStream::Consume` and `AssembleDrivingTargets` call.

Configure producers, catalogue, epoch, identity and first canonical frame before
activation. Presentation bindings are registered and sealed while the stream
is configurable. The stream checks source/consumer fingerprints before becoming
active. A single physical lane owns Consume, ConfirmPhysicalCommit and
PublishCompleted. A real host must assign all targets together, apply its
existing slew once, confirm success with the Ready reservation token, and publish with that same token only after the whole physical
frame completes. The native test models that grouped assignment; it does not
claim that a UE component already calls this API.

Consume validates frame/epoch/identity/contract, reset baseline and cross-frame
activity continuity. It records the immutable frame and assembles a complete
target tuple. Failure returns neutral invalid targets and permanently deactivates
the stream; it never calls legacy input. Installation of a fresh stream is the
recovery boundary. A missing commit acknowledgement prevents publication. Failed
physical commit permanently deactivates; successful publication must precede the
next forward consumption. A repeated pending Consume returns AlreadyPending
with no frame and neutral invalid targets, so it cannot authorize another apply.
Retained completed history returns the distinct PublishedReplay status, requiring
an explicit rollback/replay decision; it does not poll devices or add a new
publication. Exceptions from any producer, including raw Poll and mapping, are
caught at the stream boundary and permanently deactivate it with neutral invalid
targets, no history/publication and no retry. Frame wrap fails closed and
publication serial exhaustion permanently deactivates.
The source-call reservation remains under a recursive boundary mutex solely to
detect same-thread producer reentry without deadlock. Reentrant mutation calls
fail closed; Deactivate can cancel the reservation. GetContract and Produce
exceptions are contained, and returned data is discarded after cancellation.
Other threads remain serialized by the mutex; this is not an unlocked producer
call with a lifecycle race. Producers must still terminate their own work; the
stream cannot make an arbitrary blocking implementation bounded.

The consumed history and completed-publication journal are separate bounded
256-entry arrays. `ReadPublishedSince({epoch,serial})` copies a coherent batch
under one lock. Missing retained history yields Overflow with no partial batch;
wrong epoch/inactive yields Detached; a future cursor is invalid. ReadRecorded
exists for diagnostics only and is never used for presentation dispatch.

`BindAction(name,id,state,weakReceiver,method)` registers a presentation handler
for Started, Triggered or Completed. `BindReset` cancels presentation-held state
without fabricating Completed events. Dispatch validates the entire published
batch, copies the binding list, releases all stream locks, and processes frames
in order: reset notifications, then transitions in their recorded order, with
matching handlers in registration order. After all edges, only latest-frame
active actions receive Triggered, once per new latest publication. Transition
callbacks receive their recorded transition value, including a press/release
that ended before the presentation tick.

`BindSnapshotAction` observes the newest publication even if neutral. The old
three-argument BindAction overload is a documented deprecated semantic alias;
it is intentionally warning-free for existing /WX clients. Neither overload
silently becomes a Triggered handler. V1 API remains untouched, and A3 explicitly
tests the original snapshot observation schedule 0/63/95.

Bindings are presentation-lane owned, weak receivers are skipped after expiry,
and bind/unbind during or after sealing is refused. Unpossession/deactivation
inside a callback stops remaining callbacks and discards the old cursor. A
binding object is permanently associated with one stream: it cannot advance a
new possession's cursor. A throwing handler deactivates the stream rather than
replaying an already delivered prefix. Canonical mutation APIs reject work under
the presentation scope; arbitrary user callbacks are not a general world sandbox.

Overflow/invalid batches deliver no callbacks and return ResyncRequired. An
explicit Resynchronize adopts the latest published baseline without inventing
events; subsequent new publications resume dispatch. Repeated ticks with no new
publication do nothing. Consumers must handle this status explicitly.

Prepared tests exercise raw and independently authored sealed frames through the
same stream/sink/consumer, publication-before-commit rejection, full payload
equality, transition ordering, latest-only triggering, snapshot compatibility,
replay, overflow/resync, weak expiry, callback mutation/reentrancy, detach and
exceptions, malformed producers and neutral failure. They do not qualify UE
lifecycle, cross-toolchain float parity or real device support.

Final product migration must remove every physical writer driven by UE action
bindings, controller handlers and mapping contexts, including game-owned actions.
This portable checkpoint prepares their replacement but removes none prematurely.
Presentation-only UI/camera controls require separate classification. Controller
integration, mechanic migration and final legacy removal remain qualified future
work; they are not acceptance claims for A3.


## A5a opaque reservation, deferred stop and explicit abort

Ready alone grants an optional FReservationToken. The token contains private
stream epoch/frame plus a unique shared identity retained by all copies, so a
stale capability cannot become valid through allocation address reuse. There is
no public constructor from frame/epoch. ConfirmPhysicalCommit, PublishCompleted
and Abort require this token; frame-number overloads are removed. Wrong/empty/
foreign/stale or duplicate finalize tokens return false without disturbing a
valid reservation. PublishedReplay grants no token and cannot authorize a new
physical apply. Token copies represent the same owner capability; hosts must
not distribute them to presentation code.

| State/event | Result |
| --- | --- |
| Configurable or ActiveIdle + RequestStop/Deactivate | Stopped immediately |
| Ready reservation | Reserved; complete tuple may be applied once |
| Reserved + Confirm(token,true) | Committed; no second confirmation |
| Reserved/Committed + RequestStop | Draining, IsActive false, presentation detached, no new Consume |
| Committed + Publish(token) | Exactly one completed record; ActiveIdle, or Stopped if draining |
| Reserved + Confirm(token,false) | Terminal Aborted(ApplicationFailed), no publication |
| Reserved/Committed + Abort(token,reason) | Terminal Aborted(reason), no publication |
| Stop before Ready returns | No token/tuple, no physical authority |
| Wrong/duplicate token | No state change and no success |

No waits or cross-frame mutex ownership: the stream's recursive Gate protects
each atomic state transition, then releases. The physical lane keeps the token
between Ready, grouped application, Confirm and completed-frame Publish. The
presentation lane may RequestStop but cannot finalize under its read-only scope.
GetState exposes Draining versus Stopped; the host must delay installing a new
physical owner until the old stream reaches Stopped. The stream cannot police
activation of an unrelated independently constructed stream. No global registry
or hidden blocking has been added.

ReadCompleted(frame) copies only retained completed journal entries, even after
stop. ReadOutcome reports the last finalization (epoch/frame, Completed with no
reason or Aborted with explicit reason). ReadRecorded remains diagnostic consumed
history and never proves completion. ReadLatest/ReadPublishedSince detach from
presentation at stop request; the separate witness is for lifecycle evidence.
Successful publication survives a pending stop. Callback exceptions still request
stop through Deactivate, which now preserves any in-flight reservation.

Explicit Abort is terminal, even without a prior stop request. It cannot publish
or restart the stream. SnapshotPublicationFailed is available for the later
simulation integration: an unsuccessful global snapshot makes that canonical
frame non-authoritative and must abort before worker exit. This portable change
does not install that simulation notification or implement restoration. A lost
worker cannot magically complete a token; the host must report success or abort.
Publication serial exhaustion is rejected before granting a new Ready tuple.

Prepared tests: InputTransactionProbe uses deterministic no-sleep interleavings
at acquisition/Ready/application/confirmation/publication, duplicate stop,
wrong-stream/same-address tokens, stale frame/epoch tokens, abort reasons,
completion witness after stop and callback throw while another frame is reserved.
ActionStreamProbe and ActionFamiliesProbe are migrated to the token API (including
rejection of duplicate publication rather than the old idempotent-success API).
Production wrappers, simulation and Build.cs remain unchanged and NOT_INTEGRATED.
Exception containment semantics remain unchanged; future explicit module exception
policy is a separate UE integration change.
