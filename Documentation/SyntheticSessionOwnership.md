# Synthetic session ownership

This test-only model composes the unchanged portable session lifecycle protocol
with independently retained scalar state. It is not a production session host,
Unreal module service, physics participant, thread or reaper implementation.
No existing source includes it. Its probe is authored but not compiled/executed
at the source-review checkpoint.

`Tests/Support/SyntheticSessionService.h` contains a bounded service table. Each
occupied slot holds a shared session reference before activation. A session owns
its participant, lifecycle protocol and scalar input/publication state. Clients
are move-only weak endpoints; destroying one closes its endpoint and requests
stop without resolving work or releasing the service's controlling reference.
Only `ReapReady` releases that reference after fake lane exit and protocol Reaped.
Temporary internal shared references do not escape into the client API.

The entire model is serialized by explicit test calls. Admission captures a
scalar command. Apply writes it once; Publish makes a scalar snapshot visible;
Complete resolves the reservation. Each step is nonthrowing and allocation-free
after configuration. Wrong/foreign/stale tokens cannot mutate the scalar state.
Pre-publication Abort preserves the previous published snapshot and makes the
session terminal. Applied-but-aborted state is not reused. Global publication
cannot abort and must finish Complete, including after client destruction.

Held frames use no sleep, blocking wait or actual worker: the test simply does
not call the next advance method. Detach and Stopped reject unresolved work.
`ReturnFakeLane` is an explicit modeled return after Stopped; it does not infer
thread exit from a stop request. A never-started lane has no work and follows the
same empty-lane fence without pretending an OS thread was started. The reaper
scans all slots and skips unexited sessions, so one held session cannot prevent
another from being reaped.

Configuration allocation happens only in Create, after capacity and input
validation. Fixed-slot exhaustion cannot consume stop/retire capacity. Session
epoch, slot generation and the protocol's opaque session identity reject stale
or foreign addresses, including equal numeric epochs in different services.
No actor, network pointer, contact, world query, arbitrary callback or mutable
external physical object can be registered.

Lifetime instrumentation uses shared trace records that outlive the observed
objects. Clients and participants own their observer lifetime explicitly; tests
also retain it through final assertions. The weak lifetime witness exposes no
session API and tests never retain a locked strong witness. FCodeLease remains
the portable marker from the protocol, not a DLL pin. Tests can drop their own
lease reference and observe that it remains alive through Stopped/exit and
expires only on reaping.

Service destruction with any unreaped slot is a fail-fast fixture error. There
is no implicit cleanup, join or intentional leak. Tests must complete the
explicit drain/exit/reap sequence, including cancellation before Start.

`Tests/SyntheticSessionProbe.cpp` authors client destruction at idle, Reserved,
Committed, Published and resolved barriers; retained-frame completion and
pre-publication abort; paused stop; weak/lease/destructor timing; bounded
capacity and two independently retiring sessions; invalid/foreign/stale
addresses/tokens and duplicate transitions; move-only endpoint ownership;
immutable admitted input despite a subsequent client write; and never-started
cancellation. These traces do not qualify concurrent execution, OS workers,
module unload order, forced UObject teardown, production physics, hardware,
mapping parity or removal of legacy UE bindings. Those gates remain separate.
