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


Source48 separates fifteen declared wheel runtime values through const/nonconst
storage hooks. Legacy fields/defaults and physics laws are retained. Actual
private wheel subtypes allocate fresh values; native steps select those blocks,
while outside calls and retained render references reach the facade. The real
existing GLog callback additionally writes wheel jump/ground-count/unilateral-
support/angular velocity/render entries, and retains the render reference.
Actual native frames1/2 compare both COM/cache and the newly enumerated runtime
values with an untouched graph. Missing/nonfinite runtime observations are
UNKNOWN, not zero. This proposed experiment requires fresh native execution.

This hunk does not own wheel setup pointers, solid/body configuration, all
contact/scene/world/source domains or all synchronous callbacks. It does not
admit a complete fingerprint, boundary lease or localized restoration. All
historical failures remain; no existing native result qualifies this new hunk.


Source49 separates the twenty-one declared solid scalar/inertia/contact-policy
values behind inherited storage hooks. Reflected legacy fields and their
defaults stay in place. Private box/wheel allocations are fresh. The original
native Initialize routines apply the actual factory setup under the original
routing scope before first Step; no mutable facade state is imported. Private
wheel rest-force policy matches its inherited constructor. A free gutter
helper's same-named boolean parameter is only renamed, preserving its behavior.

The real existing output callback additionally uses public mass/index/contact
policy setters on the facade. Actual native solid values are copied field by
field at original Steps and must equal the untouched independent control after
the attack. Missing/nonfinite diagnostics remain UNKNOWN. This is a declared
subset, not complete shape/configuration/contact/registry/lifetime coverage.
Complete owner/restore admission remains closed. No localized or hardware claim
is introduced; new native qualification is mandatory.


Source50 preserves the inherited native wheel constructor's default mass in the
fresh private solid block. A single shared DefaultWheelMass constant supplies
both the unchanged legacy field initializer and the private wheel birth policy;
no reflected facade value is imported and no numeric setting is retuned. The
original native stock configuration methods intentionally return invalid, so
re-running Initialize cannot supply constructor-only defaults. Native45's
actual factory-guard failure remains recorded. Post-initialization recipe,
binding and non-solid witnesses stay mandatory; no guard is weakened.


Source51 keeps the strict render-reference witness while observing native
motion over a finite input-driven window. Native46 passed55 tests but this
witness failed because RollRotation was unchanged at frame2. The test now
continues actual retained throttle requests, compares native COM and declared
state with an untouched control at EVERY observed frame, requires both an
actual native render change and positive physical forward velocity, and checks
that the retained facade stays unchanged. Missing motion by frame120 fails.
Reported forward-step counts are actual, not fixed assertions. This observation
deadline never opens CanMove: its original synchronization activation remains
frame1 and is checked throughout. No physics law, startup warmup, restore gate
or numerical comparison tolerance changes. Earlier failures remain preserved.


Source52 prepares the original road epoch boundary as one development hunk.
The native bridge registry/world and driver controls/journals/publication tail
have fresh private storage. Component configuration/history/input controls and
body geometry/reference lists, box contact caches, wheel configuration and
vehicle simulation bindings join that storage. The actual physical and legacy
reference vehicle/config instances are independent; Engine AccessSetup must
never mutate a config shared with the original graph. The unchanged stock
profile program initializes fresh values without importing a mutable facade.

During Step, diagnostic values are queued; actual output-device callbacks run
after the native math stack, original storage route and real FGCScopeGuard
have ended. Publication follows those callbacks and real lifetime rechecks.
The strict test invokes actual CollectGarbage from that real callback and
requires it to return. Legacy registry mutations are now tested as mutations
of distinct facade storage, while direct original driver execution refuses
without its actual owner context. Earlier registry-refusal and native failure
artifacts retain their original results and are not reclassified.

This development hunk is NOT yet native-qualified. It does not provide a
complete state serializer, authoritative restore at K, source/lifecycle lease,
localized replay, Device integration or human local-feel acceptance. Legacy
CVar reads still require the explicit authoritative immutable epoch recipe;
a GT capture does not prove equal worker values or exclude foreign writers.
Complete restore guards and ProtectsEscapedMutableViews remain CLOSED. Even
successful compilation or partial native equality must not admit restoration.


Source53 fixes native build48's actual UHT declaration error: the original
Sphere Radius UPROPERTY and field are adjacent; storage hooks precede them.
Corresponding source headers precede deferred-diagnostic includes under the
existing IWYU policy. No header, guard, compiler severity or runtime test is
disabled. Native48 original exit6/real Job0 twice/audits0 and all32 warnings
remain preserved; no native/runtime stage ran on that failed frozen tuple.


Source54 fixes native49's actual C2248: the sealed native driver reads a
protected selection predicate; the original store/binder remain private.
This does not issue a lease or permit external StepCanonicalFrame. Native49
exit6/70.219s, both real Job0 observations, source/protected/Engine audits0,
compiler diagnostics and all5 warnings remain recorded. Runtime NOT_RUN.


Source55 preserves the actual ECanonicalEdge::InputJournal enum identity.
That audit label is not the homonymous driver data member and must never route
through InputJournalDriverStorage(). The recorded qualified-use census has
exactly this one namespace/enum collision; native member routing remains.
Native50's actual C2838/C2664, exit6/128.625s, true Job0 twice/audits0 and all
warnings remain preserved. No native/runtime tests ran on that failed tuple.


Source56 places original wheel allocations and constructor-only policies in
the actual constructor; the const storage selector remains read-only. The
generator had accidentally attached them to the following method declaration.
Native51's actual C2761/syntax cascade, exit6/287.907s, original Job0 twice and
all clean source/protected/Engine audits remain recorded; runtime NOT_RUN.
Default legacy local position/rotation access keeps its exact original
GetRelativeLocation/GetRelativeRotation().Quaternion expressions; private
epochs consume their separately owned local pose. No numeric tuning or
restoration capability is introduced.


Source57 preserves the free gutter helper's pointer parameter and the two
actual Speed::SBox value/query fields; these are not UObject body members.
Native52's actual C2664/C2039, exit6/310.219s, natural Job0 twice/source+protected+
Engine audits0 and all diagnostics remain preserved. Its separate preparation
label-check incident stays provisional and is not erased by this correction.
No native/runtime stage ran; no equation/guard/restore capability changed.


Source58 confines the sphere body-setup helper local to that helper. The scene
proxy uses the existing public GetRadius accessor; CalcBounds reads the routed
physical radius in its own scope. Native53's three actual C2039/C2065 errors,
original exit6/338.922s and natural Job0 twice with zero source/protected/Engine
audit drift remain failures. No runtime ran and no native law, value, guard,
complete restoration capability or localized replay claim changes here.


Source59 initializes the fresh original recorded-history arrays with the SAME
constructor policy as the legacy native domain: native capacity and INDEX_NONE
frame markers, default native state values. A shared typed helper executes that
policy in both births. It does not import facade history, record synthetic
frames, add warmup or replace missing history with a success value. Native54
compiled/loaded/query0, then genuinely crashed3 in the first oscillating case:
SetOwner hitbox inertia queried frame0 against an empty original history array.
All raw diagnostics/drains/audits and the incomplete suite remain FAIL; subsequent
network/Game stages did not run. No restore/localized capability is admitted.


Source60 preserves a closed startup publication across rejected movement
attempts: the actual Next/Requires/Released gate is read under original routing
before any native diagnostic stage opens. No physics/input counter/publication
changes occur and no elapsed attempts release CanMove. Native55's 512-attempt
failure previously retired the branch through the generic failed-stage tail.
Its 54Success/2FAIL and the separate real physical-callback test failure remain
preserved. The callback test now records each actual Step result/reason and GC
return count independently, without short-circuiting the reference Step.
No assertion, native law, restore guard or lifetime requirement is relaxed.


Source61 fixes the independent physical-log callback fixture's ingress history.
Its earlier owner had actually predicted frame1 before a changed frame1 request
arrived; reusing that history for a NEW independent graph correctly left a
pending invalidation. Native56 therefore refused input preparation in BOTH
graphs, before any physical callback/GC. The new independent scenario owns a
fresh real history and confirms frame0/frame1 in order. It does not clear the
earlier invalidation, forge rollback acknowledgement or relax any filter guard.
Native56's55Success/1FAIL stays preserved; its512-attempt CanMove gate succeeds.
The uncommitted root/settings integration draft is separate and not applied.


Source62 extends that same independent ingress chain through actual frame2
and every subsequent motion frame. Four remaining old-history Confirm/Predict
uses are corrected. Prepared57/source61 was explicitly cancelled by its owner
BEFORE any Engine/coordinator launch, with matching frozen inputs and original
preparation34895/exit0 retained. It is NOT_RUN, not a native failure or success,
and no Job0 observation or native process exit is invented. The last real
native result remains56/55Success1FAIL. No production filter/ACK guard changes.


Source63 integrates the reviewed development epoch hunk, not a completed
restore provider. Original driver/bridge/component/subbody allocations have
real shared C++ roots and aliasing members; original and reference native
vehicle configs are distinct and survive their real native base destructors.
Retirement uses explicit phases, original weak serials and typed root release,
never a premature Closed flag or a synthetic worker/source drain. The actual
constructor/base-destructor/config-destructor entry diagnostics are values,
not a lease. Native callback/GC/destruction-order assertions are NOT_RUN here.

The controlled conformance host explicitly chooses all178 declared CVar values
before birth.281 real readers use those immutable values inside native setup,
Step and activation, without a live getter fallback. Legacy getters remain the
original lazy expressions elsewhere. UObject allocations occur before that
scope; real diagnostic callbacks dispatch after math/routing/GC exclusion.
Acquired graph lifetimes are rechecked after callbacks, including prior cars,
bridge, driver, body pins and the final registry/counter. Activation reads back
the actual native CanMove/freeze/countdown/roster before releasing the gate.
Initial startup comparison includes the full declared-slot canonical bytes;
the existing IAS fragmentation retains1232-byte datagrams and bounded32768
assembled messages. Lane1 is mandatory for this original synchronous profile.

Tests add genuine birth output GC and prepared-wheel reachability, an actual
driver Destroy+GC negative construction, a real optional activation diagnostic
callback that mutates the legacy facade, and actual native/config destruction
order. Live stiffness changes only during Local Step; a fourth separately
configured native graph must show physical sensitivity to changed stiffness.
These are unexecuted assertions, not qualified results or178-reader coverage.

Complete native configuration/units/source identity, module/lifecycle exclusion,
all callback and Engine causal-state coverage, authoritative restore atK and
localized dependency closure remain missing. Complete restoration providers,
ProtectsEscapedMutableViews and lease admission stay CLOSED. No source62 disk
failure or older native PASS is transferred. Real Device/renderer/local-feel
and two-client localized resimulation remain required after native validation.


Source64 updates the existing partial native observer to read the actual
retained configuration root after Source63's ownership migration. The physical
factory already uses that same exact NativeConfiguration Wheel/Suspension;
only two obsolete observer member names remained. Native59's C2039/C2672,
original6/natural Job0 twice and zero audits stay FAIL; no runtime ran. No native
equation, configured value, assertion, coverage or restoration guard changes.


Source65 fixes native60's genuine failures without relaxing input or startup
guards. Predict advances its ingress cursor, so neutral evidence is read once
and shared immutably with both native controls. Activation reads actual prepared
physics while CanMove is STILL closed by its sync bit, then success-only commits
that bit and verifies real CanMove without an intervening callback. Requiring
CanMove before that commit was circular. The negative DestroyActor fixture now
supplies/removes a real temporary Engine world context before genuine GC; the
earlier context warning remains a FAIL, never filtered or expected away.
Native60:46Success/10FAIL with all warnings/errors retained. Actual birth GC
returned, eight prepared wheels were reachable, facade callback ran and actual
driver destruction+GC refused construction. Those scoped observations are not
full callback/lifetime/restore or localized qualification. Later stages NOT_RUN.


Source66 fixes actual native61 retry02 process startup: the local-session
reassembly mask was uint8 even though the explicit recipe admitted32768bytes,
more than eight1232-byte datagrams. It now uses uint64 with a compile-time
capacity proof for the fixed admitted maximum. CRC, exact part size/count,
original process pins, replay window, duplicate mismatch refusal and message
budget stay enforced. The previous three real commandlet exits1 and startup
transport Invalid/PeerDead observations remain failed; no Game followed.
Fresh same-source65 native retry02 independently passed56tests0errors/warnings,
then genuine Pending BeginPlay1test0errors/warnings, actual Job0twice/audits0.
These results do not transfer to66. Its transport regression requires the real
three-process full-recipe startup exchange, not only the earlier small packets.
Commandlet boot also loaded an unrelated sports presentation asset without its
Editor import modules:11warnings per child are preserved. The next commandlet
launch selects the existing private procedural harness flag before CDO birth;
normal Device/render gameplay retains its original asset selection.
Complete epoch capture/lease, authoritative restoreK and localized rollback
remain unimplemented/closed. Current correction is global replay from birth.


Source67 adds a normalized field-by-field capture of the six retained original
body value structs (125 fields plus nested hit/render values) and the bound
original flat plane. Unknown/stale references remain unknown; they never become
zeroes or complete-state admission. Registration checks actual retained roots,
weak birth identities, original native allocation addresses and every current
alias before assigning semantic reference keys. The new paired native measures
also compare these original body domains against matched and neutral controls.
This is not a complete component/driver/world/source/configuration capture and
does not restore any physical object or open a lease.

A real Engine ObjectHandleRead callback fixture records original wheel reads
under the native scope and retains only their observed view identities. A
positive distinct-from-facade witness demonstrates the currently OPEN escape
path, without mutating physics. It cannot certify exclusion. Complete restore
providers and ProtectsEscapedMutableViews remain closed. Full source/module/
lifecycle/callback closure and same-graph authoritative restoreK remain work.

Exact previous source66 native62:56 tests and Pending1 passed0errors/warnings.
Original process launch was refused by a build-sized disk gate before any
managed job; that NOT_RUN failure is preserved. Fresh same-DLL runtime-only
continuation passed3 commandlets,2actual Game clients plus dedicated server,
and expected9/10 cancellation. Atframe170 measured position errors were
216.0001757046008cm and123.7260579355192cm; actual global from-birth corrections
3/6 reached exact six-COM comparison at239,215contactframes. Every original
job naturally drained with final audits0. No Device/render/localized/restoreK
qualification transfers to67. No earlier FAIL/UNKNOWN is relabeled.


Source68 corrects a demonstrated CODEC bounds defect in source67: this Engine
stores64 collision responses, while ECC_MAX also includes a deprecated marker.
GetResponse indexes the native array directly, so encoding through ECC_MAX read
one byte beyond it. The new bound comes from the actual EnumArray extent with
a compile-time marker/layout check. No physical channel, state or tolerance is
removed. A native regression varies each of all64 channels independently and
changes adjacent sentinels while requiring identical encoded response values.
Source63 native49Success/7FAIL/0warnings and all57 body differences remain kept.
Their runtime attribution to this defect must be confirmed by a fresh native
paired run, not inferred from the source fix alone. Old six-COM measures remain
scoped; complete state/restore/localization admission stays closed.

Source67 also genuinely observed ObjectHandleRead tracking enabled,251 scoped
target reads and4 original native views distinct from the facade, with no
physical mutation. This is positive evidence of an OPEN callback/view path,
not a protection certificate. That path still needs actual closure before a
complete restore lease can be admitted. Native63 later stages were NOT_RUN.


Source69 replaces physical wheel/hitbox TObjectPtr reads with routed read views.
On the original branch, the exact Engine no-resolve/no-read helper is used only
after checking the actual resolved handle; unknown handles refuse birth/Step.
Ordinary public getters and the legacy branch retain Engine tracking. Original
body capture uses the same no-resolve operation. The host records actual weak
wheel/hitbox role identities before SetOwner/Initialize and checks the current
original handles against those birth roles before physical frame admission.
Short GC exclusion and existing original pins are still required.

The genuine Engine callback test covers four wheels plus the hitbox during the
tested native Step/capture paths. Its positive control measures the delta from
each deliberate ordinary Get separately for each live target. This does not
certify all factory/CDO/Engine facade/virtual/backend or module lifetime paths.
The prefetch path is unreachable behind the existing unconditional closed gate;
no claim transfers to an enabled prefetch branch. Full escaped-view protection,
complete restore lease, authoritative K restore and localized rollback remain
CLOSED. No callback registry/macro/global counter or Engine source is changed.

Exact prior source68 native64 freshly passed56native tests andPending1 with
zero errors/warnings,3commandlets,2actualGame clients anddedicated server, and
expected cancellation9/10. All managed original jobs naturally drained and final
audits were zero. Matched125-body-field/17-plane observations no longer differed
after the actual64-channel codec bound fix; the intentionally mismatched startup
still differed121/121. The finite Game fixture ended exact at239 with215contact
frames and3/6GLOBALFROMBIRTH corrections. This is not playable device/render,
full causal state, restoreK or localized qualification. All prior FAIL/UNKNOWN
remain. Source69 itself is not natively compiled or qualified by this receipt.


Source70 applies the same checked no-resolve original read to the two native
Chaos wheel-configuration reads in validation/binding. The legacy branch keeps
its genuine tracked Get. The original configuration weak birth pins and role
checks from source69 remain unchanged. A separate real Engine callback census
now observes ALL object reads under the declared recipe, from before Create
through the tested Step/capture. It records actual weak identities, bounded256
with explicit overflow, and reports names/classes only after math has ended.
Birth/Step/GT/compiled-tracking limits are explicit. This is a diagnostic to
identify remaining real edges, not a complete callback exclusion certificate.
SetOwner/Actor/CDO/virtual/backend/module/service lifetime paths remain open.

Exact prior source69/native65 passed56native tests andPending1,3commandlets,
2actualGame clients with dedicated server, and expected cancellation9/10. All
strict severities and source/protected/Engine audits were zero, with actual
natural original job drains. The genuine wheel4/hitbox1 callback test observed
zero scoped target reads and five deliberate per-target positive ordinary Gets.
Matched57body cases remained exact except the deliberately mismatched startup
control121/121. The Game fixture ended239exact/215contacts with3/6GLOBALFROMBIRTH
corrections. This is not playable device/render, full causal state, authoritative
K restore or localized qualification. All prior FAIL/UNKNOWN remain. Source70
has no native result at this commit and opens no complete restore guard.


Source70/native66 freshly passed56native tests,Pending1,3commandlets,2actualGame
clients anddedicated, and expected9/10cancellation; original58246 closed0 with
natural managed Job0twice and final audits0. The independent wheel4/hitbox1
witness stayed0scoped/5deliberate positives and matched body values stayed exact.
The new ALL-object census was INCOMPLETE: weak assignment in its callback calls
FObjectPtr.Get in this Engine and reenters the observer. It filled256duplicate
entries and overflowed. Counts470/164 are not qualified physical-read counts.
The raw report and explicit UNKNOWN receipt are preserved; neither native suite
success nor the earlier source review qualifies that diagnostic or a lease.

Source71 fixes only the observer. It records the actual object index, address
identity and actual Engine serial without Weak/FObjectKey/resolve construction
inside the callback. If the real serial is absent it invokes the real Engine
AllocateSerialNumber operation, explicitly counting that metadata call. Index,
pointer, serial, Garbage and Unreachable checks precede use; this creates no pin
or lifetime certificate. Descriptions are read back under a real short GC guard
outside the recipe/math, copied before framework logging. A separate known-world
tracked-read control checks one event and maximum observer depth1 in this exact
profile. It never suppresses a callback, resets a counter or demands zero
Birth/Step reads. Production physics and all full restore guards are unchanged.
Source71 is not compiled or qualified by this commit. Full causal capture,
remaining callback/host/module closure, authoritativeK restoration and localized
rollback plus playable two-client validation remain unfinished.


Source71/native67 actually failed compilation C2248: the test tried to construct
FOriginalRoadSettingsScope, whose constructor is private to the physical owner.
Actual UBT exit6/345.562s naturally drained with final audits0; original49017
closed1. No subsequent native/Pending/process/Game test ran. The complete failure
and the prior census66 UNKNOWN remain preserved; no previous PASS transfers.

Source72 keeps that constructor private and does not add a test friend. A
development-only, read-only method on the real FPrivateRoadPhysicsWorld owner
takes no caller recipe, pointer or callback, and returns only two observations:
whether its diagnostic was admitted and whether the tracked world read matched.
It checks the actual GT, gate, closed/failed/active state, roots and pins, holds
the real short GC guard, and enters its OWN declared-settings scope for the
single known-world Get. It never enters the physical storage route, steps or
restores physics, flushes an outbox or returns a native view/capability. Framework
checks and counter deltas remain outside that owner scope. This is only a test
profile read probe; it does not admit arbitrary callbacks/GC/teardown, a complete
lifetime lease, authoritativeK restoration or localized rollback. Source72
requires a fresh native build and real process tests; none are claimed here.


Source72/native68 actually completed: 56 native tests with zero errors/warnings,
Pending1, three commandlets, two actual Game clients and a dedicated process,
expected cancellation9/10, natural Job0 twice and all final audits0. Its corrected
one-car observer measured birth214/step164/control1/null48/unique41/no overflow
or recursion. These are diagnostic read events, not complete lifetime, source
coverage, authoritativeK restoration or localized resimulation. The old census66
UNKNOWN and compile67 FAIL remain preserved; no PASS transfers to source73.

Source73 preserves the ordinary raw FromAnalyticHit implementation and exact
callback ordering. A new overload copies an existing valid weak reference and
normalizes invalid references to null, matching the previous .Get() conversion.
It calls the raw null Reset path for the same geometric fields without creating
another tracked object read. Analytic support conversions, the static hitcount
map and selected wheel support hits reuse actual weak identities. No weak index
or serial is invented. IgnoredComponents stores raw pointers and is unchanged.

The existing three-car interaction experiment now explicitly selects the real
178-value recipe before all four native world births. A real read callback
observes sixty original subbodies and four actual floor components during the
native contact trajectory and retained replay, with an individual ordinary Get
positive control for every target. Birth and other object types are outside this
specific witness; counts do not certify non-escape or a complete restore lease.
This changes the test profile and requires fresh compilation and runtime results.
The physical correction remains GLOBAL FROM BIRTH; full causal capture, retained
authoritativeK restore, localized contact-island replay, network loss redundancy
and fresh two-client Device/render acceptance remain unfinished.


Source73/native69 completed on its exact frozen source: 56 native tests and one
Pending test, three commandlets, two actual Game clients with a dedicated process
and expected cancellation9/10, real natural drains and final audits0. Its35
oscillation cases covered21455 matched native frames with zero measured six-COM
differences. These are authored native inputs, not network-loss or Device/render
qualification. Existing FAIL/UNKNOWN history is unchanged.

The one-car observer reduced Step reads164 to152; its original floor reads no
longer appeared in that window. The newly explicit three-car recipe nevertheless
measured6176 subbody and134 floor read callbacks across the four real fixture
worlds,375 contact-event frames, with64 individual ordinary-read positive
controls. Thus the old five-target zero-read observation did not cover interacting
contacts. No complete non-escape/lifetime/restore qualification follows from it.

Source74 returns from ApplyPendingOps only after an actual locked empty queue
observation, before any adapter access. That lock ends before the existing real
worker suspension/ACK for nonempty work. Active-frame checks, ACK failure and the
second locked move remain unchanged. A concurrent enqueue after the empty
observation stays queued for a later boundary; this does not give callers an
adapter-exclusion lease after return.

The four original wheel setup classes are checked against the actual base class
captured and pinned before prepared-wheel callbacks. Original resolved handles
are read without load/resolve/read hooks; ordinary legacy class bool evaluation
retains its original short-circuit position. Actual class and graph validity are
checked after class pin/read callbacks and before the next child construction.
This is an identity guard, not a complete configuration or lifetime certificate.

A Windows diagnostic captures24-PC prefixes for actual interacting read events
with a real pre-resolved and pinned system function. All original target events
remain counted. A fixed64-prefix table reports failed captures, capacity overflow,
depth truncation and nested sampler omissions explicitly. Prefixes can merge
deeper callers beyond the cap. Module mapping/output happens after native math;
PDB resolution is separate, local and requires real DLL/CodeView/PDB identity
matching. The OS stack capture remains a diagnostic participant, not a guarantee
of zero internal OS allocation/locks or physics lifetime protection. Instrumented
timing is not a pure performance benchmark. This source needs fresh native tests;
source73 PASS does not transfer. Full causal capture, authoritativeK restoration,
localized rollback and final playable two-client acceptance remain unfinished.


Source74/native70 completed with56 native tests0errors/warnings, Pending1,
three commandlets, two actual Game clients plus dedicated and expected9/10
cancellation. Original90574 and both coordinators exited0 with natural Job0
twice and all final audits0.35 oscillations/21455 frames still measured zero
six-COM/filter/startup differences. All measurements remain exact-source scoped.
No localized or fresh playable Device acceptance follows from those fixtures.

Source74 reduced one-car original-recipe Step read callbacks152 to10 (actual
World7 and Component3); birth reads177. Interacting contacts still generated6176
subbody and134 floor read events. A real24-PC-prefix sampler accounted for6310
events in17 prefixes without capture failures, recursion or capacity overflow.
All prefixes reached the depth cap; deeper callers remain unknown. Offline local
PDB lookup used actual matching PE/PDB GUID+age and the PDB really loaded by
DbgHelp, real original-project DLL hashes, and a read-only PE file handle. The
first two helper results stayed UNKNOWN; no matching guard was weakened. Lookup
remains conditional because module mapping happened after capture and does not
prove the loader incarnation during the earlier event. Return PCs can resolve
to the source line after an actual weak assignment. Exact source inspection
confirmed the weak reconstruction sites before preparing this change.

Source75 copies real private body weak identities from the already acquired
birth role references, outside recipe/storage routing and under the actual short
GC guard. Private getters return only that weak identity and refuse a changed
or stale original; ordinary base implementations retain normal raw weak creation.
The resolver, persistent hit and contact observations reuse existing weak values.
Raw contact API signatures remain available. Weak contact overloads take an
immutable local snapshot for the full synchronous constraint/manifold loop, so
later changes to a caller's mutable hit cell cannot change the stored peer.
No pointer to that local snapshot escapes. Raw physics math/order is unchanged.

The ignored-component lookup uses a raw comparison only for a valid weak
reference. Null, stale and pending-garbage references keep the actual native weak
equality, which can equate two invalid references. The existing startup test
adds real objects, MarkAsGarbage, genuine CollectGarbage and eight validity cases
before any original recipe math, without dereferencing a destroyed raw pointer.
These controls qualify reference semantics, not complete physics lifetime.

New private weak metadata is identity/configuration data, not a restoration
capability or exclusion lease. Full causal capture, module/service participation,
factory callback closure, the remaining World/Component reads, authoritativeK
restoration and localized rollback still need work. Providers and restoration
guards remain closed. Source75 requires its own complete fresh native/Game tests;
no source74 PASS transfers and no zero contact-read result is claimed here.


Source75/native71 completed its own full bounded chain: actual56 native tests
without errors/warnings, Pending1, three commandlets, two actual Game clients
and a dedicated host, plus expected9/10 cancellation. Original17165 and both
coordinators exited0; actual managed Job0 was observed twice with final source,
protected and Engine audits empty. Eight real weak-equality cases included
MarkAsGarbage, genuine garbage collection and stale identities.35 oscillating
input scenarios/21455 frames again measured all six kinematic differences zero.
Default CanMove refused512 attempts until actual initial publication and common
activation1, with no timer release. These are exact-source programmed fixtures.

The four-world/three-car native collision fixture measured375 contact-event
frames. Its64 original body/floor targets now produce0 tracked body reads and
0 tracked floor reads under recipe, while all64 ordinary positive controls
still fire. No read callback is globally disabled. This is the measured effect
of the source75 weak-identity changes; other objects are still unmeasured.
One-car census remains177 birth reads and10 Step reads (World7/Component3).

Source76 keeps the actual topology checks while replacing those ordinary
World/VehicleMovement reads with genuine native handle readback. PersistentLevel
and Level.OwningWorld are actual resolved fields, with original weak identities
and pins. Actor/Level direct outer equality is deliberately limited to this
private runtime SpawnActor profile; streamed/reparented/replaced graphs refuse.
The real native VehicleMovementComponent TObjectPtr field is located once using
authentic FObjectProperty metadata: owner, actual native non-Blueprint class,
CPF_TObjectPtr, ArrayDim, element size, class containment, alignment and value
class are checked. No guessed offset, FField pseudo-weak or replacement field
is used. Only its const actual address is retained, with original container and
class identities. The ordinary getter is a real birth-time positive read.

All callback-capable weak acquisition, pins, metadata lookup and the ordinary
getter execute outside GC exclusion and outside recipe/storage routing. Only
after the last getter does a short GC guard revalidate original weak Actor,
Component and classes before reading the real cell with NoResolve. Garbage,
replacement and newer class versions refuse. GetOuter/GetClass remain Engine
non-access-tracked native getters; arbitrary unresolved/reloaded host paths and
module/service lifetime are not thereby certified.

The collision fixture now counts every actual scoped object read, including
other objects and nulls, before selecting its64 targets. Its partition must
equal body+floor+other+null, and trackingCompiled is reported. No all-object-zero
result is assumed. Source76 needs a fresh complete native/Game chain; no75 PASS
transfers. Factory callbacks, full typed causal state, genuine service/module
participation, authoritativeK restore/readback and localized island resimulation
remain open. Restoration providers/guards stay closed. Existing fast replay is
still global, and these NullRHI fixtures are not a fresh Device/render playtest.


Source76/native72 completed its own56 native tests with zero errors/warnings,
Pending1, three commandlets, two actual Game hosts plus dedicated, and expected
cancellation9/10. Original60492/coordinators20208+40232 exited0, actual Job0
twice and final audits empty.35 oscillations/21455 frames still measured six
kinematic differences zero. One-car Step reads fell10 to2 (World), birth177,
control1/null48/40 identities/depth1/no overflow. No full closure follows.

Crucially, the real ALL collision counter measured81385 scoped reads:56117
other-object and25268 null reads, while the64 selected body/floor targets still
measured0 with64 ordinary positive controls. This newly measured scope stays
OPEN. The target-only zero is not a substitute for all-object coverage.

Source77 adds a readonly resolved-null predicate over actual native handle
fields. It rejects live/unresolved handles rather than resolving or inventing
an absent participant. Five actual private network fields and actual public
World.NetDriver use that predicate. A real null Get remains a tracked positive
control; the helper is tested on genuine null/live fields with ALL read events
counted. No fabricated unresolved fixture is used or claimed qualified.

The private native driver reuses the same actual weak Driver/World/Level and
Outer/PersistentLevel/OwningWorld predicate for its inherited virtual GetWorld
calls while the genuine original GT route is active. Invalid originals return
null/refuse, and ordinary Engine behavior remains outside that route. No
arbitrary remembered world is accepted in place of the real topology fields.

The Windows sampler now observes EVERY scoped GT object/null read before
selection, with body/floor/other/null categories. Its256 fixed prefixes of24 PCs
retain explicit capture failures, reentrant sample skips and capacity overflow;
their sum must reconcile with the ALL counter. The actual system capture entry
is acquired/pinned before observer registration; module mapping/symbol work
stays after all math. Prefix suffixes and post-capture loader mapping remain
unknown, and instrumented timings are not an uninstrumented performance claim.

Source77 needs fresh native/Game qualification. Factory177, SetOwner callbacks
under GC/math, real service/module/deleter lifetimes, full typed causal capture,
same-graph authoritativeK restoration and conservative localized island replay
remain open. Actual service retention must distinguish an admitted internal
context from callbacks, serialize admission/closing/lazy pool initialization,
provide nonblocking stop and an explicitly neutral single join coordinator,
and test irreversible shutdown in a fresh process without a global reset.
No restoration guard is opened by this diagnostic change; no76 PASS transfers.


Native73 did NOT qualify source77. Actual compilation failed with C3248:
AActor::GetWorld is declared final in the genuine Engine Actor.h. The attempted
override was invalid and should have been caught by source review. Actual UBT6,
original31163/pipeline/continuation exit1, natural Job0 twice and final audits0
are retained; no later native/network stage ran. No Engine declaration changed.

Source78 removes that override entirely. ResolveOriginalPhysicalWorld is a
distinct private nonvirtual method, called only at AdvancePrivate after its GT
and Access checks. It retains exactly the same genuine original driver and
World/Level/Outer handle predicate. The Engine/base-driver GetWorld behavior,
canonical frame context and success-only physical counter are untouched. Other
World reads can still exist and remain measured; no Step-zero is predicted.

Fresh native74 is required. ALL81385 from source76/native72 and FAIL73 remain
durable, not transferred to source78. Full restoration guards remain closed;
callback, service/module lifetime, complete causal state and localized rollback
still require real implementation and two-client/dedicated qualification.


Source78/native74 completed56 native tests0errors/warnings, Pending1,
three commandlets, two actual Game hosts plus dedicated and expected9/10
cancellation. Original93118 and coordinators closed0, real Job0 twice/final
audits empty.35 oscillations/21455frames still measured six kinematic differences
zero. One-car Step reads are now0 with original world control1; birth177/null45
remain. Actual ALL interaction74177=52513other+21664null,64body/floor targets0
with64 ordinary positives,375 contact-event frames. All47 prefixes were sampled
without loss/overflow/reentrant skip, but all24-PC suffixes remain UNKNOWN.

Actual bounded local lookup74-01 closed0 with real Job0 twice and unchanged
eleven upstream files.1128 PC candidates and seven genuine current PE/PDB pairs
matched strict actual loaded-PDB checks. Mapping remains conditional because
module lookup occurred after capture; loader incarnation is not certified.
Exact source inspection showed legacy HasAuthority/GetAuthGameMode causing21664
null reads, the ground-sweep subsystem lookup causing46152 reads, and private
contact-observation movement getters causing6361. Return PCs can map to the
following line; the geometric VisitPlanarCandidates math was not blamed/changed.

Source79 reads the actual native World.AuthorityGameMode cell through checked
birth metadata, with callback-capable operations outside GC/recipe/route and
short final original-identity/cell readback. Null is an actual required private
profile condition, not a guessed IAS authority role. A dedicated original hook
in the true base HasAuthority method validates it; nonnull/unresolved or changed
origins refuse. Ordinary legacy behavior remains.

The native ground sweep now resolves its genuine IAS epoch/source participant
against the parent's actual frame StaticWorld. It verifies the original backend
Data/Index/Generation observation captured from the real constructor before
ownership transfer, retained original immutable data, original source ID, exact
single-plane geometry and actual selected epoch source-map/floor binding before
VisitPlanarCandidates. A changed world installed jointly in both fields refuses.
The fixed private owner thunk returns only a predicate, no original view. The
default legacy lookup stays at the same point/order. An observational query
generation and data retention are NOT backend/service lifetime certificates.

Private collision observations reuse actual born actor/component/class identities
and readonly movement cells, with genuine C++ adjustment to ISpeedComponent.
They do not cast a C++ interface as UObject or extend raw-pointer lifetime.
The solver's Super operation remains once. Original landing diagnostics queue
one owned frame/time record through the real outbox; no Engine display, GetRole,
captured callback or physical view is retained. Flush remains after physical
scope/GC/route exit, with failures preserved. The legacy screen branch is kept.

Fresh native75 is required. No zero-all-read, full callback closure, complete
causal snapshot, same-graph authoritativeK restore or localized rollback is
claimed. Factory callbacks and real service/module/deleter participation remain
open; restoration guards stay closed. Network correction is still global.
FAIL73, all earlier FAIL/UNKNOWN and the incomplete historical parent-directory
prechecks15-18 remain durable; later current directory audits do not rewrite them.


Source79/native75 FAILED compilation with actual C2679 in the original private
world birth identity check. MakeShared returns TSharedRef, whose Get() is a
reference, whereas TSharedPtr.Get() is a pointer. Source80 compares the genuine
immutable backend Data identity to &Data.Get(), without altering identity,
geometry, source-map, lifetime or restoration guards. Original23903 and both
coordinators closed1, UBT6/344.218seconds; real Job0 twice and final source,
protected and full Engine audits empty. Query/native/Pending/process/Game stages
were NOT_RUN. This failure remains evidence; source review had missed the type.

Source80 also adds finite per-actor/frame kinematic diagnostic records to the
programmed original-process probe. For240 frames it copies the real first
forward publication after AdvanceLocal and before that pulse's correction,
including actual requested/applied inputs. It keeps original-server observation
bytes, accepts identical repeats and refuses conflicting authority at one frame.
Final records pair the same actor/incarnation/frame and actual consumed-server
request journal. Six COM numerical errors and independent exact verdict are
emitted for owner and remote separately. Missing local/authority frames and
matched coverage remain explicit. There is no invented unobserved state or
whole-session maximum with partial coverage, and input-value association is not
isolated causal attribution. This is a finite programmed fixture, not player
hardware or a controlled loss test. First forward may inherit corrections from
previous pulses; it is not a never-corrected trajectory. Added copies/allocations
may perturb wallclock and scheduling despite unchanged message content/rate
parameters and physical step order, so no unchanged timing/loss profile is claimed.

Fresh native76 compilation and complete finite three-Game chain are required.
No source78/native74 PASS is transferred to80. Full causal state, authoritativeK
restore, callback/service/module/deleter lifetime admission and localized rollback
remain open and guarded. Current reconciliation remains GLOBAL_FROM_BIRTH.
All earlier FAIL/UNKNOWN including73 and75 remain preserved.


Source80/native76 completed build0,56 native tests0errors/warnings, Pending1,
three commandlets, two simultaneous actual Game client hosts plus a dedicated
process and expected cancellation. Original9813 and coordinators closed0,
real Job0 twice and final source/protected/Engine audits empty.35 oscillations
cover21455frames with all six measured COM differences zero after synchronization.
Default startup gate refused512 attempts until real publication/activation1.
The interacting recipe-GT scoped read census is now0 object/null events with
tracking enabled and64 genuine positive controls,375 contact-event frames.
This is the measured recipe scope, NOT a whole callback/factory/lifetime proof.
One-car recipe-only birth177/null41 remain. All FAIL73/75 and UNKNOWN preserved.

The actual finite Game run now retains240/240 paired authority/first-forward
frames for each client, no missing observations. First-forward position maxima
over those frames are431.5385455139685cm localA,411.53127184241754cm remoteB
onA,255.21254257253062cm remoteA and274.6549326156487cm localB onB.
Actual requested input mismatches are0 for owners,122 remoteB frames and121
remoteA frames, in the deliberate withholding/contact fixture. Final compared
kinematics at239 are exact after3/4GLOBAL_FROM_BIRTH corrections. These are
programmed inputs, not playable device/local-feel acceptance. Input-value
association is not isolated causal attribution; first forward may inherit prior
pulse corrections, and added instrumentation affects scheduling.

Source81 widens the actual primary read census to the UNION of selected recipe
and original native storage routing context. A read-only GT boolean observes
the genuine translation-unit routing pointer; it exposes no owner/view/scope
installer and grants no permission, lifetime or exclusion. Route-only birth,
step, control and null counts are reported separately. The historical
ActualRecipeObjectReadCensus marker now denotes the wider UNION, so its new
totals are not directly comparable with177recipe-only. Facade operations outside
both contexts and all non-GT reads remain unobserved by this census.

Before observer registration the actual ntdll stack-capture function is resolved
and PINned. Every primary union-scope object/null event gets a bounded address
sample, keyed by phase/route-only/null,24PC/512prefixes. Capture failure,
overflow and reentrant sample skips remain explicit and reconcile with all
actual census events. Identity accounting still uses the actual object array
and serial readback, not manufactured weak generations. The hook is actually
removed before reporting names/modules and before retirement/negative-birth
fixtures; positive controls precede removal. Retirement and subsequent invalid
birth are explicitly NOT observed by these primary counters. Current module
lookups after capture are conditional and do not certify loader incarnation or
source attribution; capped suffixes remain UNKNOWN. No symbol lookup, log,
object name or heap allocation is called by the address sampler. Instrumentation
adds work and does not certify unchanged performance or complete callbacks.

Fresh native77 is required. No76 success transfers to this source. This change
does not alter physical math, startup/retirement routing, provider guards or
restoration admission. Actual factory callback partition, real service/module/
deleter participation, full typed causal state, authoritativeK restoration and
localized contact-island replay remain required. Reconciliation remains GLOBAL.


Source81/native77 FAILED compilation: actual C2653/C3861 in the newly widened
observer. The namespace alias P existed only inside RunTest, whereas the observer
struct lives in the anonymous namespace outside that function. Source82 spells
the actual full Speed::Netcode::Prediction namespace at both diagnostic queries.
These are the only two test-code edits; the header declaration/implementation,
routing, scope/capture/removal logic, physical math and guards are unchanged.
Original45621 and both original coordinators closed1, actual UBT6, realJob0 twice
and final source/protected/Engine audits empty. All later query/native/Pending/
process/Game stages remain NOT_RUN. Source81 reviews had missed this C++ scope
error; the compile failure and all older FAIL/UNKNOWN remain preserved.

Fresh native78 is required, no76 success transfers. The reviewed helper77 is
bound to actual77 CLOSED0 and therefore MUST NOT run against failed77 or later78.
A separate reviewed78 adaptation must bind the genuine new source, original
closure, metadata and upstream hashes. No runtime factory prefix, complete
callback/service lifetime, authoritativeK restore or localized rollback is
claimed from this compile fix. All incomplete restoration guards remain closed.


Source82/native78 completed56 native tests0errors/warnings, Pending1,
three commandlets, two simultaneous actual Game clients plus dedicated and
expected cancellation. Original79488 closed0 with all real original children,
managedJob0 twice and empty final source/protected/Engine audits.35 oscillations
cover21455frames, all six measured kinematic differences0 after synchronization.
The widened primary recipe-or-original-route census measured178birth objects,
41nulls and1deliberate control, noStep reads. Route-only subset was1floor weak
assignment.91actualprefixes capture220events with0capturefailure/overflow/reentry;
151events have24PC-capped suffixes. Retirement, negativebirth, nonGT and ordinary
facade outside both contexts remain unobserved, not a wholefactory/lifetime proof.

The actual bounded readonly78 helper completed synchronously with exit0,
genuineJob0 twice and six reserved tool/review/manifest plus eleven upstream
file hashes stable.2121PC records,1994 candidates and9modules were retained;
127unresolved PC records and all capped suffixes remain UNKNOWN. Current
module/PDB mapping after capture does not certify source or loader incarnation.
These candidates plus exact code inspection identify reference/facade owner,
wheel setup, scene offset and weak-root operations inside the original recipe.

Source83 is a local experimental partition, submitted to Netcode SL for exact
source review. It keeps the diagnostic outbox active but moves the reference/
facade setup before the GC, original recipe and original storage scopes.
It uses an owned topology IDENTITY list, checked against real prior hitbox/
wheel birth weak references and actual field NoResolve readback after callbacks
and before native entry, and refuses missing native backing. Callbackful setup
no longer retains borrowed facade container iterators in this owner loop.
Native physical values are still constructed from defaults and the same
explicit declared native program, never imported from facade state. The actual
legacy solid recipe remains comparison evidence only; a mismatch refuses.
The initial floor map copies its genuine existing floor weak identity.

This does NOT complete factory callback closure: the second native SetOwner,
gravity CDO, late PinTree and driver/network admission remain known open
domains. Their guards and retention points remain unchanged. No zero-count
or historical177comparison is assumed after changing scope. Fresh native79,
GC/negativebirth/isolation controls and exact review are still required. Full
causal restoration, real service/module/deleter participation and localized
rollback remain guarded and unfinished; current replay is GLOBAL_FROM_BIRTH.
All earlier FAIL73/75/77 and UNKNOWN remain preserved.


Source83/native79 completed the finite programmed chain with actual original
launcher39653 exit0, both original coordinators0, genuine job drains and empty
final input/protected/Engine audits. The exact head passed56 native tests and
one separate pending-world test, three commandlets, two simultaneous actual
Game clients plus dedicated, and expected cancellation.35 oscillating cases
cover21455frames with six COM differences0 after initial synchronization.
The network fixture pairs240frames/client for both actors; its first-forward
errors remain nonzero under deliberate withholding and contact, with exact
final compared kinematics only after global-from-birth corrections. These
results do not qualify full-state restoration, localized rollback or hardware.

Source84/native80 FAILED compilation with C2665 in SSubBody.cpp(76,60).
The permanent ignored-component storage is actually TArray<UPrimitiveComponent*>
in both the base body and the owned native domain. Its original AddUnique(raw)
did not reconstruct any weak identity. The attempted weak-return insertion was
therefore a type error and the raw-to-weak callback attribution was incorrect.
The implementation review missed this actual container type; that finding and
the failed source84 cohort remain preserved. Conditional stack attribution to
a SetOwner caller is not proof that its raw ignored-list insertion is a weak
construction site.

Source85 restores the entire SSubBody.cpp file byte-for-byte to source83's
original worktree hash, verified against the frozen native79 vendor with only
its CRLF line endings normalized to LF. It does not add weak.Get(), conversions,
identity callbacks,
changes to list order, null handling or external virtual overrides. The source84
private-original GetSubBodyConfig override and its explicit full config include
remain: that profile still returns the actual base's identical default-invalid
value, while ordinary facade calls use Super. They have not passed native80.

Actual original launcher36047 and both original coordinators closed1, UBTexit6,
natural drain368.282s, actual job0 twice and final source/protected/Engine audits
empty. Query,56native tests,PendingWorld,commandlets,Game and cancellation80 were
NOT_RUN after compile failure. Fresh native81 must verify the current remaining
config change, native physical witness and GC/stale/negative-birth/isolation
controls and the full finite three-Game chain. No79 pass transfers. Factory
owner/tick/registration/gravity/driver/service callbacks remain open; full
lifetime, authoritativeK restore and localized admission guards stay closed.


Source85/native81 compiled and linked with genuine BuildId and completed the
query. All56 tests reported Success and0 errors, but2 LogHttp timeout warnings
to the Editor Home Screen's Google connectivity-check URL caused strict native
qualification to be REJECTED. Original20920/coordinators closed1, native process
exit0 with actual job0 twice and empty final audits. Pending, commandlets and
Game/cancellation were NOT_RUN. Raw35 oscillation messages remain unqualified;
their values are not promoted to zero-divergence acceptance or historical PASS.

Source86 reuses a genuinely retained original strong root when its original
weak birth identity still resolves to the same current object. It avoids
reacquiring duplicate roots through Pin, retains first acquisitions and the
PinTree traversal/readbacks, and does not reconstruct identity from raw address
alone. New first acquisitions remain callback-capable. Duplicate-root counts
and final release ordering can change, so actual GC/stale/destruction/negative
birth/config-lifetime/retirement/native-witness tests and the complete finite
three-Game chain must be rerun; no source-only count or performance effect is
claimed. This is not whole factory, service/module/deleter or restoration closure.

The next isolated host profile will disable the unrelated Editor Home Screen
at startup using its supported HomeScreen.EnableHomeScreen=0 config variable.
It will retain the2 warnings/rejection81 and all strict warning validation;
it will not filter LogHttp, disable IAS sockets, retune physics or patch Engine.
The actual startup value must be observed in the new process. No pending change
to a test profile grants runtime qualification. Full-state restoreK and localized
rollback remain unfinished and all incomplete admission guards stay closed.


Runtime83 freshly qualified source86 using the exact unchanged build82 DLLs as
inputs, with NO build83/native82 acceptance invented. Real original66497 closed0,
56 strict native tests and Pending1 passed, three native commandlets, two actual
Game client processes and one dedicated, and negative cancellation all drained
their original jobs with empty source/protected/Engine audits. The35 oscillating
unit cases covered21455 paired frames with all six COM maxima0; this is isolated
unit evidence, not a network/input-source/full hidden-state certificate. Actual
startup HomeScreen query returned false. FAIL80/81/82 remain preserved. The
network correction remains GLOBAL_FROM_BIRTH, not authoritativeK/localized.

The next source adds an explicit, disabled-by-default IAS-only controlled egress
loss fixture. A selected ORIGINAL UDP fragment is discarded before its OS send;
that is counted separately from an actual successful send and returns InjectedLoss.
Schedules advance independently per original destination and kind, so alternating
two peers cannot systematically sacrifice just one peer. This finite probe selects
only owner-input, authority-input and observable-kinematics kinds2/3/4; startup,
retention ACK, close and completion controls are excluded. This is not observed
external, random, burst, delayed or reordered network loss. It grants no authority.

The optional retry limit1/2/4/8/16 keeps the OLDEST unacknowledged original input
records first; zero preserves the previous fixture batching. No source/sequence,
edge, consumed frame or ACK meaning is changed. Effective counts, complete actor
journals, first-forward divergence and correction counts must be measured in NEW
three-Game process runs across limits/loss profiles. Configuration alone is not
coverage. Native compilation/regression and those controlled-loss runs are NOT_RUN
at this source edit. Observable kinematics are not complete physical state; no
physical-state redundancy, complete restoreK, service/source/lifetime admission,
localized rollback or live-device match follows from this egress fixture.

The first loss draft incorrectly treated reliable-outbox slot iteration as
chronological retry order. SL review supplied a real ACK/reuse counterexample:
seq10slot0/seq11slot1, ACK10 then Queue12 gives12-before11. Revision02 sorts only
the caller-owned retry COPY when a bounded limit is enabled, before truncation.
Zero retains prior order. The existing journal native test now exercises the
real outbox hole/reuse, negative physical-order control, original encoded bytes,
bounded oldest prefix and unchanged original outbox/ACK behavior. This test is
NOT_RUN until the new compile/native campaign; no model-only PASS is claimed.
Periodic fragment loss can phase-lock a multi-fragment message if its fragment
count is divisible by Every; finite failures must be retained, never assert
fairness/liveness or add a bypass. Loss profiles remain controlled experiments.


Native84 source87 genuinely completed its56/Pending/probe/baseline/cancellation
and9 actual three-Game controlled-loss profiles, original13508 CLOSED0 consumed.
All original Jobs drained twice with empty audits. Both clients retained all240
authority inputs per actor and final COM matched. The legacy latest-only
observation path retained only192..224 of240 frame pairs under discard schedules;
those missing values remain UNKNOWN. No earlier PASS transfers to this source.

Source88 proposes a bounded journal for native OBSERVABLE VALUE observations,
with separate wire magic from full-state/full-roster transport. It does not put
pose bytes inside FWorldState or assert the full-roster flag. Original session,
epoch, native kernel value identity, sequence and frame accompany raw observable
bytes; CRC is integrity only. Duplicate retention is idempotent; conflicts,
foreign streams, overflow, malformed batches and corrupt records refuse. ACK
means original bytes retained, never simulation consumption or source/lifetime
admission. Original frame order selects unacknowledged records for bounded
grouping. Disabled mode keeps the legacy latest-only protocol. Enabled clients
and server finish only after actual complete240-frame retention/ACK, with no
missing-to-zero conversion. Tests and new three-Game profiles remain NOT_RUN.

This is observable state delivery, not complete causal physical snapshots.
Full authoritativeK capture/restore, real opaque ingress/service/module/lifetime
participation and conservative localized resimulation remain closed/unqualified.
Source87's periodic-loss/aggregate-kind/pre-release/single-fixture limitations,
all historical FAIL/UNKNOWN and the unfinished playable-device goal persist.


## Proposed source89: opaque original same-host ingress (native NOT_RUN)

The real FNativeLocalSession alone can export one FNativeLocalIngress. The
export retains that original FImpl, socket and original process pins. Its
constructor is private; no numeric peer/session/CRC can fabricate it. The
claim is irreversible: double acquisition and facade Receive refuse after it,
even when an export is destroyed. A new socket/session cannot be rebound.

Receive runs on the original owner thread. Only actual transport Ready can
return an immutable FNativeLocalReceivedMessage containing original bytes;
other outcomes clear output. The receipt retains the original source storage
but is data only after closure. No SL callback or UObject is invoked under Gate.

Close is idempotent and may run on another thread. Gate linearizes it against
actual Receive: an operation completing under Gate before Close may return
Ready, while every Receive acquiring Gate after Close returns Closed. Close
waits for that critical section, not arbitrary post-unlock return stacks.
Each operation holds the original shared FImpl through its own return path.
Destroying the facade closes the shared original and surviving exports cannot
resurrect it. Module retention through all operations/export/receipt destructors
remains the host's explicit obligation; this code does not manufacture a loader,
service/global lifetime lease, remote-host authentication, game source admission,
complete causal physical state, authoritative-K restore or localized rollback.

This source is a proposed tranche only. Actual exclusive-consumer, wrong-lane,
distinct real sessions/peers, close/receive ordering and surviving-export controls
must be executed in genuine native processes before qualification. Existing
source88/native86/Game86 receipts do not transfer to it.

Self-review89r02: the first legacy facade Receive also irreversibly claims that
receive mode; later opaque acquisition refuses. No silent handover from an
already consumed legacy stream. Wrong-owner calls never claim either mode.


Source89r03 integrates the proposed ingress into both genuine process-probe
receive loops. Each actual opened channel must acquire one opaque consumer;
wrong-owner acquisition, second consumer and facade receive are refused before
its normal network run. On original successful shutdown it checks wrong-owner
Receive with an actual retained receipt prefilled, output clearing, original
assigned-peer provenance, surviving original bytes and repeated Closed results.
Clients destroy the still-bound facade without a prior explicit Close; the
server coordinates a real closer thread against owner Receive then checks closed
acquisition. Sidecars record actual original PID/peer/sequence/Ready count and
per-role outcomes. No particular race order/overlap or missing legacy-first
control is claimed. These controls are NOT_RUN until fresh compiled processes.
The original owner thread and called facade/export objects must stay alive
through every call/return; shared FImpl does not legalize concurrent destruction
of the same C++ object or prevent std::thread::id reuse after the original thread
exits. Host module lifetime remains required through all object destructors.


## Source90 candidate: read-only original component values

The private native completed publication additionally captures the eleven
top-level members of its actual FOwnedComponentValues: published binding, base
game/physics state, wheeled game/physics/input state, ordered constraints, actual
sleep and quantization caches, actual static resting-world binding and support
horizon. Typed nested fields are serialized without pointers or padding. A
constraint PairKey is canonicalized only after validating its exact native
HIGH32|LOW32 derivation against the original weak references. Unknown/stale
references, foreign static worlds, changed bindings and nonfinite values leave
this domain explicitly unavailable. They never become zero divergence.

Native per-input tests require actual paired component captures on every frame,
and an independent neutral trajectory must expose each input, including camera
state which need not change COM kinematics. Separate typed codec refusal controls
do not qualify a physical snapshot or owner. This candidate does not supply the
driver/native-vehicle/world/configuration closure, a restore boundary lease,
full-state synchronization, authoritative-K restore or localized rollback.
Existing restoration guards are unchanged. Fresh native results are required;
source89/native87 results do not transfer. Native90 is NOT_RUN at preparation.


## Source93 finite native code and service retention

The explicitly selected `-IASFiniteProcessCodePins` profile in a finite private commandlet or non-Editor Game pins the actual modules returned by Windows at each PIN call and retains the actual configured executor before the original probe coroutine is born. The roots outlive the coroutine, publications and pump field finalizers. Acquired/released sidecars contain actual PID/module bases and service observations. Initial roster incarnations, pre-existing callback provenance and future module loads are not covered. The profile is refused in a shared Editor. This does not open restoration guards or qualify complete initial state, authoritative-K restore or localized rollback. A separate fresh-process control exercises irreversible executor shutdown; it is never a registered Automation test. New native compile and process controls remain NOT_RUN at preparation; Windows91r07 is not Unreal qualification.
