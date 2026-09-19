# Common canonical stream and presentation — A3 source checkpoint

Status: source prepared, not compiled or executed. All types remain in V2;
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
existing slew once, confirm success, and publish only after the whole physical
frame completes. The native test models that grouped assignment; it does not
claim that a UE component already calls this API.

Consume validates frame/epoch/identity/contract, reset baseline and cross-frame
activity continuity. It records the immutable frame and assembles a complete
target tuple. Failure returns neutral invalid targets and permanently deactivates
the stream; it never calls legacy input. Installation of a fresh stream is the
recovery boundary. A missing commit acknowledgement prevents publication. Failed
physical commit permanently deactivates; successful publication must precede the
next forward consumption. Retained replay does not poll devices or add a new
publication. Frame and publication serial wrap fail closed.

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
