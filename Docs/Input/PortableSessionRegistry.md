# Portable controller session registry

This source unit implements a portable endpoint, not an Unreal integration.
`FInputSessionCommands` accepts inert descriptors from its publisher thread.
`FInputSessionRegistry` constructs Device/Test producers and their mediators on
its worker thread, with unique source ownership. No callback factory, UObject
or GameInput handle is accepted through the command API.

Admission compares canonical little-endian payload bytes plus a stable digest.
Identical command IDs return the existing receipt; different bytes reject.
Enqueued is not Applied. A bounded mailbox/receipt store rejects overflow rather
than evicting live requests. Read retries allocate no producer. Worker shutdown
closes pending receipts, prevents further admission, and retires sources on the
same worker. Failure to close the worker ledger must be handled before destroying
the registry; destruction with live unretirable ownership terminates explicitly.

The registry publishes bindings sorted by session and actor ID, forbids duplicate
actor/producer/controller ownership and aliased raw journals, and records epochs.
The one installed snapshot is shared by the session's actor list. Component-side
mapping/slew/queues are not implemented or enabled here.

Device processing retains an immutable per-action slew policy and filter state.
ExactScenario accepts no slew steps, allocates no filter state and installs the
authored canonical values and ordered edges unchanged. ExactRemote's no-slew
policy is expressible/validated, but the registry refuses a real Network factory;
neither Network V2 nor AI is enabled by this unit. A Test cannot impersonate the
Device policy by supplying an EProducerKind::Device identity.

Device bind starts paused. Resume requests a fresh raw-journal generation and
waits for readiness before the session resumes; no premature Poll occurs while
waiting. Superseding pause/detach/stop is serviced during that wait. ExactScenario
inspects its sealed reset/baseline without advancing Produce or any cursor. If
the next authored frame is not a baseline, resume rejects instead of inventing
one. Reset/reconnect can retire and replace a session with a new epoch/source;
they cannot reuse a retired journal physical lane or rewind an old producer.

PrepareFrame polls once per ready session. InstallAll exposes immutable views,
then BeginAll admits the portable step. The caller must execute actual actor
installation, gameplay/integration and participant validation before CompleteAll.
Any rejection aborts and makes the registry terminal. A terminal view cannot be
mistaken for the old active registry, and partial PauseAll receipts include each
session's reached phase. These are portable transaction proofs, not a substitute
for the future pre-gravity Unreal result gate.

InitialBoundary closes capabilities and retains an initial receipt distinct from
CompletedFrameN. StepN receives a fresh capability using the retained snapshot,
without repolling, processing or initial application again. A later failed step
preserves historical initial evidence while disabling its active use. Pause
between initial application and step quarantines rather than silently rewinding;
recovery needs a new epoch/registry boundary.

The bounded retained-replay cursor supplies historical registry/actor generations
and immutable input views without consulting a live producer. Device Applied is
checked against its retained Before/policy; ExactScenario has no filter history.
It does not restore a real world, run gameplay or reconcile presentation. Those
features remain outside this MVP portable unit. Actor generation resolution must
be supplied by the future worker simulation adapter; IDs here are value evidence.

Current tests cover actual portable Device/Test construction, source fences,
late readings via the existing owner/journal probes, exact axes and short edges,
duplicate commands, failure/retirement, fresh resume, immutable fanout, initial
receipt reuse and retained replay. They do not qualify native GameInput affinity,
UE frame rate independence, Jump/Flip eligibility, real camera transitions,
multiplayer behavior or CI migration. No UserInput field or Shot977 code changes
are included. Legacy streams/hosts remain unchanged and are not automatically
switched to this registry.
