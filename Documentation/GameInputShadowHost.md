# Windows diagnostic host/factory — opt-in source checkpoint

`Input/Windows/GameInputShadowHost.h` is an optional Windows-only leaf. No
Unreal module, controller, vehicle, build rule or runtime caller includes it.
Legacy remains the production default. This checkpoint is source-only pending
review/build: neither native execution nor Unreal/hardware acceptance is claimed.

The factory defaults to disabled and rejects invalid identity, mapper and
activity configuration before invoking the supplied backend factory. There is
no `GameInputCreate`, runtime loader or built-in real backend. Tests inject fake
GameInput interfaces; callers must provide isolated backend/discovery ownership,
pure bounded nonthrowing mappers and a run-unique producer identity. The host
does not invent thresholds or a focus policy. Automatic selection uses the
existing approved activity policy only when its explicit configuration is given.

The host exclusively retains one selected source and one private diagnostic
stream. It deliberately does not implement `IInputProducer` and exposes no
mutable stream/source/API handle, so it cannot be passed to
`ConfigureInputProducer` as a vehicle source. It adds no thread or collector.
An acquisition owner calls `BindOwnerThread` once on its live worker thread;
another thread cannot bind, begin, abort or complete frames. Owner threads may
not exit/be recycled while the host is live; there is no ownership-transfer API.

`BeginFrame(N)` consumes once and retains one pending diagnostic transaction.
Repeating that pending N returns the same recorded copy without another read.
Another frame cannot begin before completion or abort. `CompleteFrame(N)` is
called only after the corresponding canonical physical publication succeeded;
it publishes the matching input copy. `AbortFrame(N)` prevents incomplete
physics from appearing as completed input. Aborted/completed/control-invalidated
frames cannot be reopened through BeginFrame; observers can still read their
immutable history copies. Source forward-contiguity rules remain authoritative.

`ReadLatest` and `ReadRecorded` return copies only; observers never poll OS or
choose devices. The shared stream's publication serial follows completed
diagnostic frames. It is separate from vehicle input and physical authority.
Current shared history bounds remain unchanged; this is not a rollback journal
serialization implementation.

Host control locking serializes pause with polling and completion. `SetPaused`
calls the source reset immediately, invalidates pending diagnostic publication,
and returns a monotonic control epoch as a synchronous acknowledgment. It does
not wait for the next frame. Device revision/generation checking remains in
the source; the host epoch is not substituted for those tokens. A cached
pre-pause frame cannot be resurrected for publication; its historical copy
remains readable. Already published snapshots remain past observations, never
a claim of fresh input while paused. Actual stopped-worker/focus/possession
integration is not wired by this header.

Selection/lock requests retain the source's forward-boundary policy. Lifecycle
and polling reject synchronous presentation mutation. Shutdown is mandatory
cleanup even during presentation: close the host and stream, then unregister
the source. On failure the source/context remain owned for explicit retry and
no further polling/control is permitted. The existing destructor fail-fast
fallback remains; callers must resolve shutdown before destruction and never
destroy from callbacks. Mappers cannot reenter host methods. Lock order is host
Gate -> stream (where applicable) -> source Gate -> discovery/session locks;
SDK lifecycle callbacks never acquire host Gate.

`WindowsShadowHostProbe.cpp` prepares fake-backend tests for disabled/invalid
creation, explicit single owner, private completed publication, pending replay,
abort, pause between consumption/publication, fresh-held resume, explicit versus
automatic control and failed shutdown retention/retry. Test thresholds are
fixtures, not product defaults. Compile with the existing GameInput v3 headers,
MSVC14.38 C++17 /W4 /WX and `InputPresentationScope.cpp`; no GameInput import
library or runtime is needed for these fakes. Regression probes for selected
source, activity, discovery and portable producer remain separate required checks.

The accepted hardware F9 probe exercised only current keyboard-any-device
readings, visible foreground UI pump and its own ~5ms worker. Before any real
host activation, selected-device current/next, history/overflow, focus/pump/GT
stalls and lifecycle/mapping must be qualified. Physics300Hz is not a measured
GameInput rate. Background/focus policy requires an explicit product decision.
Full UE wrappers/module/package inclusion, shadow scheduling and developer
Freeplay acceptance require later mandates. No code here changes production
bindings, physical actions, server behavior or console platform support.
