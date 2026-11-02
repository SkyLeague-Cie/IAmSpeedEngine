# Client prediction implementation campaign

The new human mandate requires local predicted physics, same-frame divergence measurement,
authoritative rollback and bounded canonical resimulation with two real clients and a dedicated server.
The previous presentation-only campaign remains independently closed; its results do not qualify this work.

Ownership: IAS owns protocol, full state, inputs, physics worker and restore; Netcode Sky League owns
gameplay integration, camera/HUD and parent pins. A predicted-client profile must be separate from
PresentationClient; no existing guard is weakened. The server remains authority; each predicted client
has one physical writer for its complete causal world. Start with the full two-car world, not a guessed
single-car island. Predict the opponent only from information actually received, never test schedule knowledge.

The historical divergence helper compares the same frame, world incarnation, branch, authority epoch,
schema and registry generation. It measures position, linear velocity, orientation and angular velocity
for each actor, while also comparing full canonical actor and world payload bytes. Hidden mechanical
or gameplay differences require reconciliation even when visible poses coincide. Diagnostic quaternion
normalization never changes physics. Publication serials are local and are not compared across processes.
History has explicit frame and byte budgets; expiry, future authority, roster changes and malformed state
are distinct outcomes. No fabricated zero-error result is returned for unavailable history.

This helper neither authenticates a sender nor proves payload completeness. Its caller must obtain
local state from the completed native publisher and server state from the authenticated original connection,
with certified complete native serializers. Current native complete-restore providers are absent.
RequestRollbackAndResimulation still rejects live input sessions. No native guard is opened by this tranche.

Next implementation: complete stock-car/subbody/common/contact/world checkpoint providers and boundary
lease; atomic input+world restore; exact retained local inputs and explicitly predicted remote inputs;
bounded Fast catch-up through the same physical pipeline; resimulation receipts and final-only presentation.
No rollback acknowledgement may be issued before the actual restored/replayed publication is observed.
Missing history requires an explicit full resynchronization, not a teleport labelled rollback.

Acceptance needs actual client prediction before server delivery; forced incorrect remote predictions and
collisions; detected historical mismatches; authenticated restore/replay receipts; post-replay equality
of full causal state/contacts/inputs; no duplicated effects; finite natural process drains and exact-source audits.
Loss/delay/reordering trials and human camera/control continuity are separate measured gates.

RemoteRequestPrediction implements bounded hold-last-REQUEST speculation with explicit evidence frame,
not hold-last-applied filtering. Predictions are a distinct type with no counterfeit device sequence,
raw edge or reset. Only earlier received server commands may seed them. Late changed requests and
even within-frame taps with unchanged final values invalidate the affected historical frame. The host
must then restore/replay through the native transaction; the helper cannot acknowledge that operation.
Confirmed data and speculation have finite capacity; no silent eviction of commands needed for correction.

RollbackReplayCoordinator now implements correction ordering and bounded replay through a mandatory
native adapter interface: authenticated origin token, same-frame historical comparison, current target
and replay budget, prepare before mutation, restore/read-back equality, consecutive canonical steps,
history replacement and one final presentation release. Adapter failure after mutation retires the
timeline; there is no partial-world fallback. Portable coordinator tests use an arithmetic world fixture.
No real native adapter/provider is supplied or qualified yet, so this is NOT working client rollback.

The next native candidate adds a production Forward/Resimulation purpose to the canonical frame
context, independent of Shipping/test builds. It leaves frame, fixed delta, physical time and every
restore admission guard unchanged. Existing fixture tags are preserved. Pacing diagnostics consume
the purpose to distinguish unpaced replay from real-time starts. SL external effects may inspect
the same purpose, but physical gameplay must still execute during replay. This tag is not an
authority, ownership, lifetime, world-restore or event-deduplication certificate.


Native storage implementation is advancing separately from that portable coordinator.
RestorableSuspension uses a genuine derived Engine object, owns its configuration,
and serializes every protected spring/smoothing field without calling mutating getters.
RestorableWheel covers all44 native dynamic fields and frozen configuration. Its current
recipe explicitly refuses a nonempty opaque Engine FGraph; it never approximates that
configuration. Staged tokens reject native mutation, wrong targets, repeated commits,
configuration mutation and same-address lifetime replacement. The native fixture also
checks600 subsequent original Engine wheel steps after correction. These are child
providers; none establishes an exclusive world/lifecycle lease or a network admission.

NativeVehicleStorageCheckpoint additionally stages four wheel and four suspension
states and installs them into the original native BASE objects via legal complete
copy assignment, retaining the original configuration pointers. Existing subbody
bindings therefore keep their exact addresses, including after preparation objects
are retired. This fixed recipe refuses other vehicle subsystems rather than omitting
their causal state. Its child commit does not publish an atomic component/world;
whole-world ownership, Common/subbody/contact/input providers and integration remain
required. Fresh native tests on each exact candidate are mandatory; source assertions
and an earlier suspension test do not qualify this new storage participant.


BranchInputFilter adds a separate replayable input-processing domain. Local requests
retain their original immutable V2 Device frame; confirmed remote requests retain
the received packet; speculative holds retain only FSpeculativeRequest. No V2 frame,
Device generation, source sequence, raw reset/edge or transport credit is invented
for a forecast. The same pure authoritative quantized processing function executes
once from the restored Applied baseline toward the held Requested values. Frame
order, immutable local/remote role, contract/policy, horizon, owner thread, revision
and native filter lifetime are checked. Restoring this filter state alone neither
authenticates a server nor restores/adopts a physical world or an active registry.
The new native unit fixture is NOT_RUN until the exact next candidate is built.


## Private native road kernel â€” source prototype, native tests not yet run

`FPrivateRoadPhysicsWorld` now constructs actual private native cars, four native
wheel/suspension systems per car, and an authored finite analytical road. It
consumes the typed branch filter output through a dedicated native movement
subtype and the unchanged canonical 300 Hz solver. Its public outputs contain
immutable values; no actor/component/world pointer is returned. The factory
retains graph/configuration ownership and checks native lifetime before a step.
This experimental kernel is GT-only, has no Engine world context, normal tick,
BeginPlay or Chaos physical owner, and rejects presentation-client mode. The
network subobject registration policy is disabled at construction, before
registration; any actual registered network component/history makes admission
fail. The subtype permanently refuses external state projection and the legacy
pose/mechanics rollback. These guards do not change the existing client profile.

The native test exercises two independent two-car worlds and a third fresh
world replaying retained commands, including steering changes, throttle/brake,
GC while alive, wheel values, graph retirement and retained immutable values.
Fresh-origin reconstruction is explicitly distinct from authenticated historical
snapshot restore. Native build/test is NOT_RUN on this new source until its own
frozen tuple has been compiled and executed. Native36 belongs to the preceding
input-history/filter source and cannot qualify this kernel.

The kernel does not authenticate a local Device frame or received authority
packet, implement source continuity admission, supply a complete restore lease,
adopt a corrected connected-client branch, or implement an SL world. It refuses
menu/reset and unknown game-action values/edges. The private subtype deliberately
does not manufacture completed V2 world/input receipts or legacy Requested input
from already filtered Applied values. Physical failure retires the branch;
there is no partial or pose-only fallback. This is a concrete native execution
building block, not completion of the requested dedicated/two-client rollback.


The first native private-road execution on f62d779 failed before its first
physical frame because ASpeedCar intentionally applies the road profile only to
the exact generic movement class. The specialized private movement constructor
now explicitly adopts the same existing ApplyStockRoadCar recipe. The rejection
and failed test remain recorded; no coefficients were retuned, and the corrected
source requires its own native execution before any kernel PASS is claimed.


## Finite native predicted session and late-request reconciliation

The private road kernel is now verified on IAS e504d18:37 native tests plus the
separate Pending1 cohort passed, zero runtime errors/warnings, genuine modules,
natural Job0 twice and final audits clean. The kernel test ran actual two-car
graphs, independent exact trajectories, GC and fresh-origin replay. The earlier
f62d779 construction failure remains preserved.

`FPrivateRoadPredictionSession` now owns a real native predicted graph with one
retained local Device domain and remote hold-last-REQUEST histories. Local raw
identity, baseline, generation, sequence and active-edge continuity are validated
without polling. Later remote requests can invalidate actual consumed forecasts.
A correction creates another real graph from the original seeds, replays retained
commands in bounded pulses, accepts new confirmations while staged, restarts a
stale staged version, catches the current live tail, then adopts its immutable
publication and corresponding histories together. Original Device history and
incoming confirmation high-waters are retained. No fabricated raw metadata, ACK
or completed registry receipt is generated.

This is a finite native prototype: at most4096 original frames, no silent history
eviction, GT physical ownership, original-seed reconstruction rather than an
arbitrary-frame fullstate snapshot restore. The numerical peer supplied to its
ingress is a binding check, not authentication. It does not admit a connected
PredictedClient profile, restore a complete server snapshot or qualify hardware,
network loss/delay, full-game state, cameras or effects. Those acceptance gates
remain open; no old restore guard or complete-coverage flag has been changed.

The new native regression deliberately withholds opponent changes, measures a
same-frame physical error, delivers late requests during bounded staged replay,
advances the live local tail, and requires exact corrected kinematics against an
independently simulated reference. This new code/test requires its own frozen
native build and execution; preceding37 PASS cannot qualify it.


The public SL source review identified valid generic contracts that the fixed
road kernel could misinterpret: swapped physical throttle/brake destinations,
and wider or different camera semantics under slots5/6. The factory now checks
its exact seven supported action semantics, versions, types, signs, ranges and
identity physical destinations BEFORE allocating a native world. Unsupported
extensions are refused, without clamping values or retuning coefficients.
Legitimate raw control rebinding and mapping revisions remain independent.
Native regression fixtures preserve generic contract validity, cover both
physical/camera counterexamples and require rejection before world allocation.
This correction requires its own frozen native test result; no earlier PASS
qualifies the revised admission code.


The native session now retains its actual immutable per-frame physical
publications beside original local inputs. Read-only CompareObservedKinematics
selects the exact observed historical frame, validates the complete semantic
roster/incarnations and completed COM coordinates, then reports position,
velocity, acceleration, quaternion-angle, angular-velocity and angular-acceleration
errors per actor. Nonfinite observations or overflowed diagnostics are refused. Future and
missing history are distinct results; visual-origin positions are not silently
converted or retimed. The caller must authenticate observations before use.
This is kinematic observation only, not a complete canonical-state comparison,
authority admission or pose-only correction fallback.

Staged reconstruction retains its own native physical history and swaps it with
the corrected graph/input histories at adoption. The native late-input regression
now also compares an actual reference frame60 while the client is at90, and
requires that same unchanged historical observation to match after correction.
This extension is NOT_RUN until its own exact source has been compiled/executed.


A second public SL source review found that exact pose equality must not be
inferred from rounded diagnostic norms/acos. The native historical comparison now
uses separate exact kinematic field equality, treating antipodal quaternions as
the same orientation. Vector diagnostics use hypot; rotation uses validated,
normalized copies and a stable quaternion chord atan2 expression. No tolerance
is invented and no captured state is modified. The existing fixture now checks
identical poses, antipodal rotation, normalized tiny nonzero rotation and vector
differences whose naive squared norm underflows. These numerical checks alone
do not qualify native physics; the same production helper is used by the actual
native historical/reconciliation test. Revised source is NOT_RUN until its own
exact native cohort executes.


The finite native transport probe now has a cooperative C++20 pump for a game
host. Each Tick returns at a completed native publication or protocol wait;
the actual original road graph, retained inputs and OS session stay owned by
the suspended coroutine. Options are copied into that frame. Tick/destruction
require the original GT, reentrant resume is refused, and destruction unwinds
real native RAII owners before publication/status storage retires. Cancellation
cannot emit success, ClientDone or a Job-drain receipt. ReadLatest returns only
the immutable actual value publication. The old commandlet entry point drives
the same pump to completion; it no longer owns a separate protocol algorithm.

Correction work per pulse is bounded to16frames by default, with an explicit
1..256work-budget option. Physical delta remains1/300s. This is scheduling work,
not an elapsed-frame startup gate, a larger timestep or localized resimulation.
The protocol still has finite240-frame unit-authored inputs and original-birth
correction. A game host, real acquisition, complete native restore, interaction
closure and real two-playable-client/dedicated-server tests are still required.
This cooperative source is NOT_RUN until its own native build and three-process
qualification; preceding native53 or54 results do not qualify it.

### Private native mutation and retirement refusals (source, not admitted restore)

The private native subtype rejects the public legacy snapshot entry before any
common cache, static-support, pose, frozen, mechanic or sub-body write, even
when checks are disabled. The ordinary valid legacy path retains its existing
behavior. The world legacy caller validates every component before its first
mutation; private components reject that preflight too. The void compatibility
entry is not a success receipt, a complete restore provider or an atomic cohort.
The native negative now uses the actual component-captured, successfully decoded
mechanic payload rather than only an invalid empty envelope. Development
automation alone does not constitute a checks-disabled native lane.

Private phase-observer installation is refused before writing the actual base
storage. This does not revoke a previously retained view or drain a callback
already in flight. Original native wheel and suspension storage remains alive
through synchronous world cleanup callbacks, then surviving UObject bindings
are cleared before storage retirement. Future escaped readers and writers are
still unqualified; complete restore admission remains closed.

The cooperative probe now exposes an original-GT Cancel operation with a
distinct Cancelled result. It clears the immutable publication and marks the
pump active before detaching/destroying the actual coroutine handle. Native
cleanup callbacks can query values, but cannot resume the retiring coroutine.
Actual ClientDone attempt/datagram counters are values-only diagnostics, not
process-exit or Job-drain evidence. The new unit covers before-first-pulse and
foreign-thread refusal; the real suspended-native-world cleanup case still
requires an actual process/Game harness. This source is not native-qualified
until its own fresh candidate runs.

A new native-only retained-graph experiment uses three actors in one correction
world: two initially approaching cars and a distant moving third actor. A real
oscillating brake request is withheld on the live branch. The test requires an
actual cross-car native constraint (not floor support), measures the same-frame
kinematic error, and advances the original graph held atK10 through11..180 with
the retained authority requests. All three actors must match the independent
reference in six COM fields, filters and startup gates. The third actor also
replays170frames: this is intentionally NOT a localized-replay success claim.
It exercises interaction-sensitive reconstruction before any island admission;
no coefficients, tolerances, presets or demolition rules change. Native result
NOT_RUN on this source until a new frozen candidate executes.

Native30 preserved a real regression failure: the new contact test incorrectly
required dynamic component constraints, but the stock solid-subbody profile
disables their optional registration. Absence of those constraints does not
prove absence of physical impacts. The new source records values from the real
private component RcvImpactOnSubBody callback after invoking the unchanged
super implementation exactly once. Only two genuine members of the same active
native roster/frame can form a dependency pair. Fixed-roster pair capacity is
reserved before physics; overflow explicitly marks the observation incomplete.
No constraint flag, impulse, collision rule or tolerance changes. The retained
interaction test now requires actual cross-car callbacks and their replayed
per-frame dependency sequence to match the independent reference. This is
contact observation, not complete contact restoration or localized admission.
Native31 execution is required; Native30 FAIL remains intact.


### Contact fixture correction after native31 (source only)

Native31 preserves the original failure. Its scenario supplied initial velocity
before SetCannotMoveLocal, whose actual FreezeMovement zeros all velocities;
brake-only packets therefore did not drive the approaching cars. RcvImpactOnSubBody
also is not called by the actual box/box solve. The new private hitbox subtype
observes genuine blocking box/box resolution dispatch and calls Super once,
without changing forces or enabling optional constraints. This is a conservative
dependency witness, including zero-impulse contacts, not an impulse receipt.
The new scenario starts at rest with the default closed synchronization gate,
then supplies real throttle from frame1 and withholds only later brake changes.
Native trajectory diagnostics precede assertions. All three actors still replay;
no full-state lease, localized rollback or network qualification is inferred.


Native33 genuinely moved the cars, but its0.6s interaction observation found
nearest COM distance131.24427900637943cm from an initial140cm, actor0 maximum
3D travel83.794849641695194cm, and zero actual blocking box contact dispatch.
The hard contact requirement failed; this is not an impact success. Native34
therefore observes the SAME geometry/inputs/default synchronization for3s,
with explicit1024prediction/2048retained-frame UNIT bounds and no eviction.
CanMove still releases at common1; the longer observation is NOT a freeze,
countdown, warmup or adopted production horizon. Per-actor COM/velocity and
actual hitbox center/extent values at completed cuts diagnose approach versus
vertical/support motion. Contact remains mandatory, full native reconstruction
still replays all3actors, and no complete state or localized claim is added.


Source40 preserves native35's box-only FAIL (zero box/box dispatch over3s).
The NEW RetainedVehicleInteractionNativeExperiment observes actual private
box/solid resolution dispatch and actual wheel/other-car-box contact registration,
with distinct bounded counters. Super runs once; no impulse/geometry/input/startup
change. Registration is not an impulse or complete-contact certificate. A fourth
genuine control world changes only the opponent's lateral proximity; actor101's
original seed/input/gate are unchanged. Require a real different-car event AND
over1cm actor101 difference versus that control before accepting vehicle coupling.
Then the original retained graph must replay890frames and match all3actors and
the independently observed event sequence. All3 replay, so this is NOT localized.
The old test/source FAIL is not reclassified. No restore/borrow guard opens.


Source41 selects a separate finite interacting network fixture explicitly.
Two stationary native cars activate through the same actual0→1 synchronization
gate, approach using unit-authored throttle/brake requests, and BOTH clients
measure same-frame170 divergence before withheld remote requests are delivered.
The final actual native graph of each of the3 processes must contain observed
different-car box/wheel contact event frames. Registration remains distinct from
impulse; these counters are not complete contact/state certificates. Replay is
still FROM BIRTH and global: this is a required real-network coupling test, NOT
localized K rollback or hardware admission. Default separated fixture stays
available. Source40's aggregate motion diagnostic label is corrected; its old
raw output and distinct exact CausalControl counters remain preserved.


Source42 adds actual owner-lane values for a DECLARED PARTIAL native subset:
sleep/identity-quantization caches and all named wheel/suspension values in the
existing diagnostic enumerators, with actual owned configuration values and
normalized setup aliases. Unknown pointers, nonfinite values, missing native
storage or unsupported nonempty slip graph yield UNAVAILABLE, never zero.
Atomic/oscillating units report unknown/differing/neutral-response frame counts;
the retained graph compares this declared subset in addition to six COM fields.
The observer does NOT call guarded common Capture with a fake lease and does
NOT serialize complete subbody/contact/world/configuration/lifecycle state.
It is not a complete causal fingerprint, authoritative checkpoint, reader
exclusion or restore provider. All restore/lifetime guards remain closed.


Source44 begins a PARTIAL separation of physical storage from its Unreal facade.
Five component value blocks (base game/physics, wheeled game/physics/input) are
freshly default-initialized once by the original private component after native
construction; no mutable facade field is imported into the new storage. Only
the original explicit seed/policy and subsequent real steps initialize/evolve it.
Class-owned native operations route through virtual storage hooks; legacy
components keep their original fields and behavior. The private factory, step
and initial-publication release install the original routing scope under their
gate. Outside that scope, native getters/setters reach separate facade values.
No publication contains a mutable view of the owned blocks.

The proposed native boundary experiment retains a facade reference obtained by
Engine enumeration and writes pose/inputs/allowed angular velocity through
normal entries. It also registers a real FOutputDevice, enables the existing
UpdateFrameState physical debug log, then attempts another pose write and view
retention from its actual synchronous callback within the original Step call.
That debug output site formats immutable native values before suspending the
private routing for its external output devices. Physical1 and2 compare against
a genuine unattacked graph, including declared partial cache/wheel values.
Until executed this is only proposed source evidence. Other callback sites are
not qualified by this one site's guard or test.

This is NOT a complete causal owner or boundary lease. Configuration/public
properties, shared common caches, subbodies/contact/world state, timeline fields,
callbacks that might run within an original scope, registry/lifecycle/source
closure and previously escaped views from other domains remain unqualified.
The scope is routing, not a certificate of exclusion. No restore provider is
admitted and ProtectsEscapedMutableViews stays false. Global/from-birth replay
is unchanged; no localized, hardware or complete-state claim is added.


Source45 extends the PARTIAL private value routing to five common blocks:
ordered physical constraints, sleep, identity quantization, the borrowed
static-world pointer and its frame horizon. Original native operations use
private default-initialized blocks; facade calls use legacy storage. Actual
private cache diagnostics read the routed original values during their step.
The real output-device experiment additionally calls the supported public COM
velocity setter between calls and inside the existing synchronous log callback.
Source41 compilation failed because the test attempted a protected allowed-
angular-velocity setter. These unsupported calls are removed; no field/method
visibility is changed. Source41 runtime remains NOT_RUN, failure preserved.

This does NOT supply complete state/reference/configuration coverage, exclude
all other callbacks or lifecycle operations, or authorize guarded common
capture/restore. Existing guarded common providers still require a genuine
complete boundary lease before their first native address read. Their legacy
field access must be integrated with an actual future owner/provider before
admission; no current private lease/provider can bypass that gate. Subbody,
world, configuration, timeline/source and old-view closure remain incomplete.
ProtectsEscapedMutableViews stays false; no localized/hardware claim is added.


Source46 corrects the fixture after native42 failed (55Success/1Fail, all
original drains/audits clean). The deprecated live throttle/brake/steering
setters intentionally emit ensures and refuse; these guards remain untouched.
The test uses supported public pose/unfreeze/velocity entries on the facade,
while actual native input still comes only from retained input evidence.
The real output device opts into Engine's unbuffered dispatch through
CanBeUsedOnMultipleThreads; Serialize returns on foreign threads BEFORE reading
fixture state. The existing diagnostic category is enabled at Log and restored,
which keeps warnings/errors visible. Actual callback count must still be1;
missing/deferred output remains FAIL, never a synthetic callback invocation.
No complete state, lifetime, localized or hardware admission is introduced.


Source47 begins PARTIAL subbody core value separation. Seven values (kinematics
and the future/current/ground/box/sphere/wheel hit caches) route through virtual
storage hooks. Legacy adapters keep their fields. Actual private final box AND
wheel subtypes allocate fresh values without facade import; the original
factory binds their access before the first physical step. Wheel setup/native
configuration lifetime is unchanged. Unknown exact wheel subtype prefetch stays
opted out; no parallel-query/performance qualification is inferred.

The existing actual unbuffered physical log callback now additionally obtains
the hitbox through its real public configuration entry, mutates facade pose/hit
through ordinary subbody setters, and retains both references. Real physical1/2
and hitbox centers compare with an untouched graph; the native per-frame hit
reset must not reset the retained facade hit. The deliberately invalid facade
hit is an ATTACK, never a real-contact/impulse/coverage witness. Until a new
native run executes this experiment it remains proposed source only.

Parent/registry association, geometry/scene transforms, ignored/external lists,
additional solid/wheel/suspension state and configuration, world/timeline/source
and all other callback/lifecycle closure remain incomplete. No complete causal
fingerprint, restore provider/lease, localized replay or hardware admission is
introduced. ProtectsEscapedMutableViews remains false. Native43's scoped
success is retained independently and cannot qualify these new subtypes.
