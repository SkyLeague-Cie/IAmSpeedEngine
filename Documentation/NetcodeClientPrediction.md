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


## Proposed source99 private birth access split (native NOT_RUN)

The original private factory captures the actual actor rotation/location and
performs facade admission/tick calls before entering the original storage route.
The native owner then applies those copied actor values and the same shared
subbody/inertia operations; it registers with its already-acquired bridge
without querying UWorld from that registration entry. The ordinary public
SetOwner path retains its Engine tick/registration route and operation order.
The private IAmSpeed-only component's async Engine tick is disabled again after
the ordinary reference setup, and the original value initializer does not enable it.

This closes specific actor/world lookup and tick/registration crossings in
private birth. Subbody initialization, other getters, allocator/reentrant paths,
source/configuration lifetime and every actual escaping view remain to audit and
protect. No complete restore lease, complete hidden state, authoritative-K restore
or localized rollback is supplied. Native results on source96 do not transfer.


## Proposed source100 local subbody pose boundary (native NOT_RUN)

The native base subbody initializer and owner-kinematics update use explicit
physical local pose operations. Their default implementations perform the same
scene-component calls as before. Private box/wheel branches use their owned
local offset and original Euler value. The native exact Euler-to-quaternion
cache still receives Euler values; no quaternion-to-Euler round trip is added.
The owned Euler coordinates are included in declared body-value encoding with
finite checks. This extends the declared six-struct member count to126; it is
not a claim of complete causal state.

The native registry boundary test extends its actual GLog callback attack with
supported scene offset/rotation writes on the escaped hitbox and wheel. It must
observe both writes and compare subsequent actual COM and encoded body values
against an independent untouched physical graph through the final motion frame.
The facade base-kinematics must respond to the same scene writes as a positive
control. Existing callback, GC return,
retained-reference and native motion checks remain required. A separate typed
codec control proves Euler-coordinate sensitivity and nonfinite refusal; that
control does not measure scene isolation. These changes do not close every
borrow, configuration/source/lifetime path or authorize restoration. This
fixed stock private profile authors identity local rotations; changed valid
subbody configurations and Engine rotation-cache history are not qualified. Native
results from source99 or earlier never transfer to this proposed source.


## Source102 proposed finite code-admission refusal diagnostics (native NOT_RUN)

Native94 passed the scene-pose isolation control and normal finite two-client /
dedicated fixture, but its cancellation cohort failed: role7 refused code-pin
admission before its first physical tick. The current combined error does not
distinguish code-pin status from original-PID mismatch. That subcause remains
UNKNOWN; the whole native94 chain remains FAIL and all actual drains are retained.

The proposed diagnostic includes the original pin status/error/process, expected
process and observed module count from that same refusing report. The predicate,
immediate refusal, module-pin algorithm and service lifetime are unchanged. There
is no retry, roster-stability assumption, missing-state substitution or positive
lease. A later successful cohort cannot erase this failure. Numeric diagnostics
are observations only, never full state or authoritative-K/localized admission.

### Component kinematic reference separation (source103 draft)

The public current/frame COM getters on the wheeled adapter read the actual
legacy fields, including qualified base calls during private original routing.
Public signatures remain const references; these are mutable facade views, not
immutable snapshots. A module-internal, non-exported reader with private entry
points retains the original live-reference operations at their existing native
call sites. Its private virtual dispatch is access control, not an owner lease.

Actual-owner diagnostics compare six direct/interface/qualified-base current
and frame references with the real legacy and original field addresses. The
registry boundary scenario checks these counts and retains its physical-control
comparisons after real facade mutations. New native execution is still required.

This tranche covers the named component getters only. Subbody/sweeper getters,
public GetPhys submember references, existing protected storage hooks, mutable
operations and escaped wheel/configuration/source views remain open. Complete
causal state, source/lifetime exclusion, authoritative-K restore, localized
resimulation and rendered device-client acceptance are not admitted. All restore
guards stay closed. No native95 result transfers to this changed source.

The internal native-read overrides live only on the final private road subtype.
Ordinary and custom wheeled adapters retain the original public virtual getter
dispatch through the ISpeedComponent default. The six observations above query
the current capture frame; they do not qualify all historical ring hit/miss or
overwrite branches. Other alias families and restore guards remain unchanged.

Source103/native96 did not compile: the moved history reader and its capture
diagnostic used SpeedConstants without including its declaring SpeedConstant.h.
Native compiler exit6, outer wrapper1, natural Job drain and zero source/Engine
audit drift are preserved. QA and runtime96 were not run. Source104 adds that
existing declaration header explicitly; it does not hard-code a history size,
change a reader body or relax any restoration guard. A fresh tuple is required.

### Subbody and sweeper kinematic reference separation (source105 draft)

The final private box/wheel types select legacy backing for public body getters,
including qualified base and sweeper-interface calls under original routing.
Default custom body-storage and sweeper getter dispatch is preserved. Native
call sites retain the old live storage reads using private, non-exported reader
entries; two final private types specialize the internal sweeper reader.

The actual owner records five public paths on each original box/four wheel
objects, checking twenty-five addresses against real facade and native backing. The
registry boundary scenario checks those values while retaining native control
comparisons after real facade mutations. This draft is not applied or tested.
GetPhys submember references, existing protected hooks, mutable operations and
other aliases, complete state/source/lifetime and authoritative-K/localized
restoration remain unqualified; all restoration guards remain closed.

### Bounded actual process PIN acquisition (source106 draft, NOT applied/native)

Native97 cancellation refused at code admission with actual status RosterChanged,
error0, equal actual/expected PIDs and423 original observed records. The report
does not identify whether the later count or same-count bases changed. Native94
fine cause remains UNKNOWN; neither failure is erased by a future success.

This draft preallocates1024 module records and four raw pass observations before
the first PIN. It records each real initial/After enumeration, byte count,
errors and transition. Later inputs are explicitly the preceding actual After
observation. Only a strict superset of already successful matched PIN bases may
be expanded; only new bases are PINned. Failed PINs, removals/substitutions,
invalid sets, capacity and pass-budget exhaustion refuse. Success requires exact
equality of the final currently observed numeric base set and actual PIN set.
No Engine callback, C++ allocation or physics operation occurs in the PIN loop.

This is still only retention at the actual PIN calls. Initial incarnations,
preexisting callback provenance, future modules and full source/lifetime/restore
permission remain UNKNOWN/unadmitted. Real finite Windows stable/growth/refused
PIN/budget controls and the fresh native tuple are NOT yet implemented or run.
Pure set tests would qualify only set predicates, never a Windows loader lease.

The source106 experiment also retains raw pass observations in successful
finite-process reports and a separate refusal report before any service/world
admission. Development-only, thread-local kernel event controls let a finite
Windows fixture load/unload a real private DLL after actual input enumeration
and before PIN. No enumeration/PIN result is supplied by that control. A failed
event/finite wait returns a distinct refusal, never PinFailed or success. These
are Windows loader controls only and do not qualify Unreal/world restoration.
The actual harness and new native executions remain pending.

The raw pass/record JSON fragments are shared engine-independent inline code so
the finite Windows fixture can compile and parse the exact same renderer. A
prior unexecuted draft had malformed C++ quote escaping; it is preserved as a
preparation finding, not a native failure. Actual new compilation is required.

The unexecuted r04 draft used the SDK FALSE macro in a new BOOL comparison; Unreal hides that macro. The next draft uses !=0, preserving the actual BOOL result without depending on the standalone SDK wrapper. No r04 native failure or success is inferred.

### Private GetPhys submember routing (source107 draft, NOT applied/native)

Seven public reference getters and the public COM-location alias select legacy storage, including their actually
available direct/interface/qualified-base paths. Module-internal named physical
readers retain the original native references. Position, origin, velocity at
points, direction and acceleration reads must migrate coherently with the fixed
local COM offset; the single origin-formula implementation is shared unchanged.
Generic public-only overrides retain default virtual dispatch. Only the final
private native type selects original COM/local-COM backing through private reads.
The proposed owner diagnostic records24 actual submember/alias paths alongside COM6
and body/sweeper25 and retains existing reentrant positive facade/physics tests.
This draft is uncompiled and unexecuted; no source/borrow/lifetime, hidden-state,
authoritative-K or localized-restoration coverage follows. All guards stay closed.

The unexecuted r04 draft missed the physical GetPhysCOMLocation alias inside a planar roundoff correction. The native read now uses the same private COM reference, before adding the unchanged Correction and calling the original setter. Three real alias paths are included in the24 diagnostics. Other exact-class-gated joined/shadow kinematic paths remain outside this private-native migration scope; no exhaustive alias or restore lease follows.

Source107/native99 failed compilation at the private reader used by the unnamed
gutter damping helper. Native exit6, root1, natural drain, two Job0 observations
and zero input/protected/unexpected-output drift remain recorded; QA/runtime99
did not run. Source108 moves that exact helper body into a non-exported named
implementation friend, changes only its three call designators, and keeps all
parameters, settings, vector operations and logging. The reader stays private.
This is a compilation repair, not an authoritative-K, localized, causal-state
or source/lifetime qualification. A new frozen tuple must compile and run.

### Contact reference separation (source109 draft, NOT applied/native)

Public current/future hit references, their HasHit/WillHit/TOI predicates and
the physical-constraint array reference select facade storage only on the final
private road types. Ordinary/custom storage adapter dispatch is preserved.
Internal native readers retain exact live backing at the named physical calls;
no setter, hit acceptance, solver order, coefficient or equation is changed.

Actual-owner diagnostics check thirty body hit-reference paths and two component
constraint-reference paths against real facade/native addresses, plus fifteen
derived predicate checks. The existing real output-device facade mutations and
independent native-control comparison remain. An empty contact/constraint or
pointer-identity match never certifies actual contact or complete causal state.
Public mutable operations, wheel/config/source aliases and lifetime exclusion
remain open. No complete CanMove synchronization, authoritative-K/localized
restore or rendered/device client qualification follows; all guards stay closed.
HasActivePhysicalConstraintsOtherThan and its pose-dependent relevancy predicate
remain explicitly outside this getter-reference tranche. They keep their exact
existing storage/pose dispatch; no facade-array/native-pose hybrid is introduced.
New compilation and native tests are required. Source108/build100 is a distinct
frozen tuple; no historical result transfers to this draft.

Source109/native101 failed compilation at the newly added constraint-reference
diagnostic: PrivateRoadPhysicsWorld.cpp had only the forward declaration of
FComponentRead. Native exit6/root1, natural drain, two real Job0 observations
and zero input/protected/unexpected-output drift are retained; QA/runtime101
were not run. Source110 adds the defining ComponentKinematicRead.h directly
after the translation unit's own header. It changes no method, friend, equation,
contact order, predicate or restoration guard. A new frozen tuple must compile
and run; no source108/native100 result transfers to these changed sources.

### Explicit native operation and storage boundaries

The private road owner has a separate, non-exported operation object. It is
constructed only from the actual retained graph and its transferred owner lock;
a public frame context, thread-local routing flag or caller-supplied type id
cannot issue it. Typed storage bindings retain the real allocation's control
block and validate the actual adjusted subject, graph and operation before use.
This establishes a private C++ memory/access boundary, not a complete source,
UObject, module, worker, escaped-borrow or restoration lease.

The driver, bridge and pair solver propagate this operation through their
private publication and physical read/write paths. Legacy and custom adapters
retain their existing public/protected virtual behavior. Shared numerical
bodies preserve the original formulas, branch order, clamping and transaction
order. Frame-scoped collision-world pointers remain borrowed references.

This migration is incomplete: numerical component/sub-body/sweeper phases,
snapshot/participant callbacks, initialization and partial-birth retirement
still contain legacy paths. In particular, a protected Storage member pointer
can still reach original storage on unmigrated paths. Do not treat the private
operation, strong ownership, negative public-access controls or any successful
compile as closure of that channel or admission for restoration.

Complete causal-state/source/lifetime coverage, effective initial sync and
activation agreement, authoritative-frame restoration and localized resimulation
remain required. The prepared-restore guard stays closed. No historical native
test, programmed NullRHI run or observable-subset equality qualifies this tuple
as two playable clients with working localized rollback.

### Private sequential sub-body update and joined callback boundary

The next native tranche routes the sequential component-to-sub-body update
through the actual private operation. The real ordered sub-body list, origin,
six kinematic writes and wheel update after its inherited sphere operation use
the original shared numerical bodies. Authored local pose/Euler values, one
shared per-thread rotator cache and suspension reads retain their original
order. Typed raw storage validation precedes private dispatch.

Native joined installation constructs its fixed operation-bearing callback
inside a private entry and consumes it only in a private template in the same
translation unit. The public legacy TFunctionRef overload receives no native
operation. The existing joined consume helper retains its synchronous six
writes; this does not admit a callback or borrowed output to escape.

The concrete validation target is the real private road Step reaching these
new native update entries, followed by fresh same-frame input/body/component
comparisons and normal/exception last-wrapper retirement on the new tuple.
Passing pre-tranche tests does not qualify changed code. Source routing plus
state parity does not invent a measured branch counter or prove every optional
parallel/joined branch was taken.

SupportsCapture, shadow/CaptureAndCompute and parallel paths, further numeric
phases, initialization, snapshot/participant callbacks and partial-birth Close
still need source/lifetime/escaped-borrow review. Protected member-pointer and
mutable-view channels remain open wherever their actual callees are unmigrated.
Complete causal state, an effective full-state CanMove barrier, authoritative-K
restore and localized replay with two rendered playable clients plus a dedicated
server remain unqualified. The prepared-restore guard remains closed.

### Private shape-specific resolved-hit acceptance

The operation-bearing Step hit-acceptance route preserves the actual distinct
box and wheel virtual implementations. Box copy/reset, early return, sphere
cache and frame, per-component static budget, repeated-contact predicates and
peer acceptance share their original bodies with legacy policies. Wheel keeps
both hits and only ignores the current component or its future fallback. The
base public implementation remains unchanged; unsupported native types refuse.
The actual owner/peer backing and box parent backing are checked before that
private entry's first write. No operation-bearing public callback is created.

This remains a limited cache-routing boundary. Component mobility/type reads,
weak object resolution and source/lifetime coverage require further work;
typed backing and borrowed static-world identity do not provide that coverage.
Shape response/sweep/birth, protected mass/member-pointer channels and other
mutable views remain open. The restore guard stays closed. No historical
native result qualifies new bytes. Complete initial state synchronization,
authoritative-K localized replay and two rendered playable clients plus a
dedicated server remain unfinished.

The test-only controls execute eight authored cache fixtures through both actual
legacy and private native dispatches inside a real advancing Step operation.
They distinguish box sphere-cache/frame, static cap and repeated-contact cases,
wheel hit preservation/current-before-future fallback, and the unchanged base
behavior. A second actual graph supplies foreign body/peer/parent refusals;
only copied results escape. No foreign mutex is held across the real own Step;
the foreign owner is retained solely for genuine negative subjects. Named fixture
values are cleaned up before canonical
advancement and compared with an independent graph. This is not acquired source
coverage, full-state restoration or a new lifetime/exclusion certificate. These
controls still require compilation and actual native execution on the new tuple.

### Private birth initialization and concrete shape inertia

The next boundary carries the actual factory Birth operation through Base,
Solid, Box, Sphere and Wheel initialization and the concrete Box/Sphere initial
inverse-inertia dispatch. Seven original bodies are shared with legacy policies.
Box still performs its post-Solid configuration query and second inverse call,
including its invalid-config branch. Sphere also performs two inverse calls;
Wheel performs the third when its wheel configuration is valid. Legacy custom
configuration and inverse virtuals remain in their original order.

Private native body/parent and shape bindings are validated before dispatch.
The sealed native stock parent's exact declared default-invalid configurations
are returned only after those bindings are available; missing bindings refuse.
These values are not substitutes for unknown configuration. The key-free scalar
PrescribedMotion virtual and remaining source/lifetime boundaries stay open.

Prepared controls use genuine UCLASS fixtures for five valid/invalid legacy
configuration cases and record actual configuration/inverse order and the
extent/radius/mass visible at each call. Separate controls run inside the real
Birth operation, reject foreign body/parent/inverse subjects and a Step-only
route, and verify that the real Step rejects Birth-only initialization before
writing. A forced failure after one actual initializer on the already allocated
first-car graph returns through the existing factory/operation/owner unwind.
Only existing independent native/configuration counters are retained for the
post-return destruction observations. This does not cover early allocation or
constructor failures. No birth-table flag or restoration guard is changed.

This source requires fresh compilation and execution. These controls, typed
backing and limited initialization witnesses establish no complete causal state,
source/lifetime exclusion or restoration lease. Other mass/member-pointer,
response/sweep, constructor/binding, retirement and escaped-view paths remain
open. Full effective-state CanMove synchronization, authoritative-K localized
resimulation and two rendered playable clients with a dedicated server remain
unfinished. Earlier native results cannot qualify these new bytes.


The next private Birth binding route preflights the real currently owned vehicle,
the original captured wheel/configuration birth identities and all typed backing
parts before the shared binding body's first log/write. The factory records its
actual new allocation before Init transfers ownership. Legacy null/partial/full,
configuration-index/tail and Clear semantics remain in the same shared body;
actual Retirement helpers are unchanged. Native pointer inputs and real body
parents are checked against those current owned originals before writes.

Prepared controls exercise real legacy UCLASS objects, meaningful masses and
both Raise/Drop predicates; genuine factory Birth and Step operations; foreign
body/parent/vehicle/wheel/suspension/configuration subjects and an invalid fourth
roster/configuration before earlier writes. Named temporary fixture cleanup and
an independent COM comparison do not supply authoritative restoration. These
source controls are not executed at preparation. Existing keyed helper paths,
other numerical/birth/retirement accesses, complete causal state/source/lifetime
exclusion, full effective-state CanMove synchronization, authoritative-K local
resimulation and the final two rendered playable clients remain unfinished.

The private factory shares only the unchanged vehicle constructor program.
Its reference/facade caller explicitly retains ordinary wheel binding with the
reference configuration. Only the later original storage caller selects the
actual Birth binding route and its original captured identities. No permission
boolean or reference-to-native fallback selects those routes.

### First original owner Birth traversal (source candidate117)

The actual FIRST factory owner traversal now obtains a closed immutable roster
of original actor/component/hitbox/wheel/configuration weak births from its
genuine typed owner map. The record copies already observed weak identities and
is retained by an alias of the actual vehicle root; it is not physical state,
caller permission, a completeness mask or a source/lifetime lease. Same-Outer
reference configurations are not interchangeable with the original cohort.

Before owner/COM/pose writes, the specific Birth entry preflights the actual
complete roster and body/common/shape/runtime parts. Initially null parents are
allowed; foreign parents refuse. A distinct true no-vehicle clear keeps Chaos
configuration while clearing wheel/suspension pointers; the116 actual-vehicle
binder still refuses null. Legacy/custom dispatch, COM precedence, ordered
children and row-vector chassis R.Transposed()*STORED InvInertiaLocal*R share
their existing programs. The SECOND initializer loop stays after vehicle
binding and authored offsets/radius/travel. No global phase restriction is
added to ordinary or Retirement entries; four protected mass overrides and all
restoration guards remain unchanged.

Prepared controls distinguish ten first and ten second normal initializer
returns. A separate real constructor fixture uses actual nontrivial captured
actor pose, nonzero COM and rotated hitbox, nine before-write refusals including
same-owner reference configuration, an actual stored-tensor sentinel that
distinguishes recomputation, and all five child kinematics. That fixture MUST
abort after five first returns and before original vehicle construction/Step;
its actual reference-constructor/destructor returns are counted separately.
It is neither restoration nor complete coverage of constructor failures.
Real Step refusal and independent normal COM parity remain required, plus
ordinary/custom helper controls. New source/native qualification is required;
none of109's results transfer. Complete causal state, source/lifetime closure,
full effective-state CanMove, authoritative-K localized resimulation and two
rendered playable clients with a dedicated server remain unfinished.

### Proposed first-owner child velocity oracle correction (source119)

Fresh source118 diagnostics isolate the failed first-owner fixture to its child
kinematic comparison; the probe returns false after the real initializers and
nine before-write refusals. The original wheel update performs a second velocity
write from world child position minus parent position, after the base update.
The fixture incorrectly compared every child against the base-only expression.
The proposed diagnostic oracle retains the original exact wheel FP order, with
an independently computed expected position. Actual native and real legacy
wheel controls must distinguish it from the base-only shortcut. All other
strict first-owner assertions, physical programs, tolerances and restoration
guards remain unchanged. This correction still requires fresh native validation;
it supplies no complete causal-state, source/lifetime or localized rollback proof.


### Proposed Step mass response and reset propagation (source120)

This candidate carries the real private Step operation through both world roots:
component reset and resolved-hit dispatch. Thirteen box/sphere response/reset
methods and the component constraint-reset program share the original numerical
bodies with ordinary/custom legacy policies. The native reset relevance consumer
uses its existing operation-bearing constraint program. Twenty-six response mass
reads validate actual backing, non-null parent and parent roster membership.
The reset owner preflights all children before constraint or earlier-child writes.
Private constraint borrowing stays in the existing friend helper; no private
derived type or operation issuer is made public. The single box contact
observation remains after the single response. Wheel reset retains its distinct
current-hit behavior; its other legacy field accesses remain explicitly open.

Proposed native controls run inside actual factory Birth and canonical Step
operations. They compare real ordinary/native reset outputs from authored
persistent/transient constraints and hit canaries, distinguish Box from Wheel,
and reject foreign subjects/components/peers, null and wrong same-graph parents,
and late-invalid child rosters before named-buffer writes. Two nonblocking
responses must return unchanged. Named test-domain cleanup is neither a complete
world snapshot nor an authoritative restoration provider. These controls have
not run on this candidate; source reversibility checks and119 results are not
native120 qualification.

The four protected mass overrides remain unchanged. Constructor, public setup,
activation, retirement, other key-free operations, escaped views and complete
source/lifetime exclusion still require review. No restore guard opens. Full
causal initial sync/CanMove activation agreement, authoritative-K localized
resimulation and two rendered playable clients plus a dedicated server remain
unfinished. No physical coefficient, floating-point program, contact order,
geometry or tolerance is intentionally changed by this access migration.


The source120/build114 compilation failed with two C2248 diagnostics: the writer
helper could not call a private reader helper. Source121 keeps both helpers
private and adds a private static validator on ISpeedComponent, which already
has reader friendship and already grants writer friendship. The static bridge
delegates to the unchanged two typed raw-owner checks; it does not dereference
the caller pointer before those checks. Neither helper visibility nor any friend
list is expanded. In particular the private wheeled-domain type is still named
only by its existing authorized reader. The failed114 build, natural dual drains
and clean audits are preserved; QA and native tests did not run. This correction
requires a distinct fresh compilation and native run, including source120's
pending controls and measured trajectories. Earlier119 results do not qualify it.


The source121/build115 compilation advanced past the two private-access errors
but failed on six C2027/C3861 diagnostics at the three new component-reset probe
calls: the physics-world translation unit had only a forward declaration of
FComponentWrite. Source122 includes its existing private ComponentContactWrite.h
definition. Its existing friendship already admits these world-owned calls;
no helper interface, friend list, arithmetic, native control, or restoration
guard is changed. Build115 exited6 with natural dual drains and clean audits;
QA and native tests did not run. The failed114 and115 builds are retained.
A distinct fresh build and native run are required; earlier119 successes do
not qualify source122 or the still-pending mass-response controls.


### Observed consumed-input prefixes at first forward publication (candidate123)

The predicted session can copy its actual live immutable publication and the
same graph's full retained observed-input prefix0..F together under its GT Gate.
Graph adoption swaps physical histories, so concatenating earlier first-forward
records would not reconstruct the input history used by a later adopted graph.
The copy is bounded, value-only and carries actual kernel lifetime and completed
adoption count. It includes every action slot's Requested, Before and Applied,
active mask, filter values and observed CanMove/frozen/countdown/startup values.
Declared settings bytes remain only the existing declared subset. Actual reset
evidence is read from retained raw frames or actual received packets; speculative
holds have unknown reset evidence, serialized as null rather than invented false.
Request-origin metadata stays distinct from consumed-command-v2 producer origins.

The finite network probe captures this copy before its pulse correction and
records server inputs from each actual completed server Step/publication before
the packet-history push. Full payloads are emitted outside physics; no hash is
treated as equality. Missing, duplicate, rejected and overflow observations stay
explicitly UNKNOWN. Offline comparisons must separately report observed-value
prefix equality, origin metadata equality and role/origin consistency, with
reset-evidence unknown explicit. Equal observed values do not qualify equality
of Device evidence, the causal baseline, hidden state, source/lifetime or K.

The native regression is extended around real staging/adoption, preserved old
copies, nonzero camera slots beyond the old three-axis trace, actual reset and
unknown speculative reset, finite overflow and closed/unstarted sessions. These
changes and their network reports are NOT_RUN until a distinct reviewed native
tuple is compiled/executed. Added observation can change wall scheduling; new
network outcomes and egress counters must be measured rather than imported.
The four mass channels, restoration guards and physical programs are unchanged.


### Declared payload size and exact table correction (candidate124)

Source123's4096-byte diagnostic bound refuses the existing178-slot declared
conformance profile. Its Encode layout derives9389 bytes from the current names
and profile; that source derivation is not an observed native measurement. The
copy bound now matches the existing32768-byte startup publication contract.
The actual native regression selects that same existing profile in both graphs
and requires exact observed declared bytes throughout the retained prefix.

Probe JSON stores the complete declared byte payload once in a bounded table,
then uses checked indices in each input frame. Table lookup compares full bytes;
hashes are never payload equality. Missing indices and refused/overflow tables
remain UNKNOWN. The offline comparison must resolve exact payloads before value
comparison and keep unknowns explicit. This reduces repeated JSON bytes; capture
still copies full values and remains quadratic in finite first-forward frames.
Native tests, real serialized size/time, network and egress remain NOT_RUN on124.
No complete physical/configuration state, lease, activation or localized K
restoration follows from equality of these declared slots.


### Reference wheel binding Birth proof (candidate125 draft)

The private factory retains the actual reference allocation, simulation and
wheel/suspension array identities before Init transfers unique ownership. A
private stack witness ties these observations to the actual retained root,
component/actor and real body/configuration births. The reference binding entry
checks that the transferred allocation is still owned by the reference cells,
not the original backend of the same component, and validates the entire roster
before any index, pointer, pointed configuration or simulator/mass write. The
existing shared binding program runs in its existing order through fixed facade
hooks after those checks. Generic ordinary/legacy entries remain compatible.

This is a draft with native tests NOT_RUN and exact review pending. It does not
close the four ordinary Mass aliases, public Chaos-qualified SetupVehicle,
remaining retirement population evidence, complete state/source/lifetime,
initial network CanMove synchronization, authoritative K restore or localized
resimulation. Earlier124/118 native results do not qualify this candidate.


Candidate125 adds draft real-factory diagnostics for reference Birth: sixteen
foreign/mixed-backend/late-roster/target/index/parent refusals with distinct named
fixture sentinels, a nondefault reference binding positive, and an actual Step
rejection at the reference body Birth preflight. These tests are NOT_RUN until
the exact candidate is applied, built and executed under its own admission.
The scoped proof expires at Birth; no absent/noOp or foreign live Birth-operation
native coverage is fabricated. Retirement clears and original root disconnection
remain separate existing paths with their population evidence still OPEN.

Candidate125 R03 additionally compares actual reference binding values before
and after each real original binding call (including its existing diagnostics),
and refuses reference proof if either element allocation aliases the original
backend. Both-direction isolation tests remain NOT_RUN on this new candidate.

Candidate125 R04 corrects the positive reference control: before/after binding
it also compares the named original solid index/mass, all three original wheel
binding pointers, original simulator mass and original configuration travel,
plus the named foreign target mass/travel. This can expose an accidental double
write even when reference results are correct. No whole hidden-state claim.
The earlier incomplete positive footprint is retained in draft R02/R03 history.


### Actual Retirement facade clear boundary (candidate126, not native qualified)

The factory now proposes capturing the actual weak FImpl owner allocation and
control block with each real original body identity, once and before wheel
enrollment. The private Retirement clear validates the genuine stack operation,
owned mutex and active owner against that captured root. It avoids the storage
table lookup that deliberately rejects Retirement during an incomplete Birth;
the registration guard stays unchanged. After real world callbacks return and
the body weak serial/garbage flags resolve, StorageAccess resets before three
fixed direct facade pointer null assignments. Original retained C++ wheel
domains disconnect later, separately, with configurations alive through actual
native destructors. No caller target, mass/config write or TLS grant is added.

Draft native controls count actual weak visits, unresolved subjects, flag skips,
live survivors and clear returns separately from original root disconnects.
Normal/partial-birth/last-wrapper tests require nonzero genuine clear returns;
cleanup callbacks invalidate real wheel subjects and report whichever skip
branch actually runs. Absent flag/unresolved populations are not certified.
Actual Birth/Step must refuse the leaf before the named pointer footprint changes.
Under a real Retirement operation, the Root validator must first accept its own
actual root and refuse a real foreign root. This isolates the validator from
the foreign leaf's still-live StorageAccess precondition; foreign-leaf rejection
with all later preconditions satisfied remains NOT_PROVED. These new controls are NOT_RUN until a new
exact tuple is applied, built, admitted and executed;125/119 results do not pass126.

Ordinary physical bindings remain OPEN. The real early reference SetOwner calls
FLegacyOwnerBirthAccess::BindInitial before the later125 reference vehicle witness;
its no-vehicle Clear makes two ordinary nulls (wheel and suspension). Retirement
and the ordinary tail make three. A leaf-only refusal also follows SetIndex in
the whole binder. Future closure must migrate that real early Birth oracle and
guard the entire ordinary entry before getters/logs/index/effects; a permissive
null/type/TLS exception is not a replacement. Four Mass aliases, arbitrary public
targets, explicit Chaos-qualified SetupVehicle, complete state/source/lifetime,
initial network CanMove synchronization, authoritative K and localized rollback
with two rendered playable clients and a dedicated server remain unqualified.

Candidate126 R03 preserves three R02 review findings: the foreign leaf test was
masked by its unreset StorageAccess, positive counters did not observe actual
facade effects, and a postcallback footprint read omitted garbage-flag checks.
The new positive records each facade field's actual pre-clear presence, null
after return and nonnull-to-null transitions. At least one real present field
is required; a zero-present field does not acquire positive transition coverage.
Each postcallback diagnostic weak subject also resolves its garbage flags;
unresolved/flagged populations are counted and omitted from value reads, never
represented as null pointer values. Original pointer comparison remains separate.

Candidate126 R04 explicitly checks the foreign subject's stored original weak
serial/this/flags and the captured birth root against its genuine retained FImpl
allocation/control block before the validator control. The foreign fixture has
both a different allocation and control block. It does not independently prove
same-address/different-control-block rejection; that narrower case stays untested.


Source127 proposed EARLY reference owner migration (not yet native qualified).
The actual factory early SetOwner call uses a distinct call-scoped Birth witness
of the retained FImpl and its preexisting actor/component/five-body weak births.
There is no vehicle allocation at this point. The original legacy SetOwner,
first-owner and child arithmetic bodies are shared; the reference access checks
its real roster at callback boundaries and issues exactly two facade null writes
per wheel, keeping configuration. Null-to-null calls are not positive clears.
The initializer itself remains unchanged and OPEN, including its independent
binding/radius/mass/inverse writes. Four Mass overrides are unchanged.

This lot deliberately stops before permanent ordinary physical-leaf and whole
Bind/Clear/Setup refusal. The actual early call must first be independently
reviewed and tested on its exact tuple. Ordinary parent SetOwner, base-qualified
IAS/Chaos lifecycle, public arbitrary targets and other ambient channels remain
OPEN. Genuine NoOp controls for later refusal are not claimed by calls inside
actual operations. The new direct Retirement validator phase controls check all
other actual root/controlblock/mutex/active-operation predicates before invoking
it under real Birth/Step; previous whole-leaf refusals remain separate results.
No hidden-state/source/lifetime lease, network CanMove closure, authoritative K
restore, localized rollback or two rendered playable clients is certified.

127 R02 adds the actual captured world/bridge weak births to the early witness
and checks the real world and returned subsystem before registration. The legacy
first-owner oracle now invokes the actual ordinary SetOwner with real actor pose
APIs and custom body callbacks. NoOp binder controls remain deferred with the
permanent refusals. R01's uncompiled proposed footprint used unsupported composite
codecs; R02 uses named six-field values and the existing explicit array codec.
No R01 native result existed or is claimed.

127 R04 fixes the R03 shared children TArray iterator crossing virtual Update:
Origin is still computed once, cardinality captured once and each index resolved
fresh through its access before the next callback, preserving order/arithmetic.
The real custom wheel fixture reallocates the parent's actual TArray while its
old allocation remains live, keeps exactly the same pointer roster and observes
different GetData; subsequent actual custom updates must return in the same order.
This exercises the shared algorithm through genuine custom dispatch, not a custom
subclass falsely admitted into a final private graph. The ReadWheels TView already
uses an index iterator and does not retain an element borrow across Initialize;
the initial broader suspicion was withdrawn after its implementation was read.
All proposed controls still require fresh C++/UHT and native execution.


### Typed early reference initializer boundary (candidate131, native NOT_RUN)

The actual early reference owner now proposes a distinct call-scoped initializer
route through Box/Solid/SubBody and Wheel/Sphere/Solid/SubBody. It reuses each
existing numerical program and legacy reference backing, rather than substituting
the original Owned/Part initializer. The genuine factory root, active Birth,
component/actor/world/bridge and captured body weak births are checked before
the first parent write and across real configuration/inverse callbacks. Each
inverse callback returns one copied matrix; validation precedes reacquiring and
writing its destination. Ordinary Initialize entries check an empty default hook
first; final private bodies unconditionally refuse, including null and explicitly
base-qualified calls. Four Mass overrides and restoration guards are unchanged.

Draft native controls include27 true NoOp attempts on fixed private subjects:
nine virtual/qualified Box/Wheel/base forms with null/own/foreign parents. Two
actual retained graphs, absence of any operation and live routes are required;
six declared original/reference/facade footprints also observe vehicle allocations.
Separately, six typed refusals per actual actor execute after the genuine early
owner returns and before vehicle construction/early abort. Own validator positives
run before and after each refusal without reinitializing. Four copied buffers
cover five own and five foreign named bodies, their identity/parent/pose/Euler,
ignored arrays, solid/inertia and Box/Sphere/Wheel values. These four typed buffers
do not independently cover all pointed Chaos state: PWheel identity is recorded,
but a change confined to its pointed SetMassPerWheel state is not certified by
them. Ordinary NoOp allocation observations do not transfer to the typed controls.
Actual completed initializer/check counters are exported before diagnostic failure;
an empty or rejected capture cannot become a zero-divergence success.

All new C++/UHT and native controls remain NOT_RUN until the exact new tuple is
built and admitted. Planned typed rows are6 early-abort,6 original-abort and12
normal, with strict run/actor/entry/subject/parent identity and report/stdout checks.
Step/Retirement of this strictly early witness, isolated mixed-backend rejection
and inverse-callback invalidation on the exact final private class remain NOT_PROVED;
no witness is retained/reconstructed and no fake private subclass is admitted.

These child guards do not close ordinary parent SetOwner or InitializeBoundSubBodies,
which write before reaching children. Arbitrary targets, Chaos-qualified lifecycle,
complete causal state/source-lifetime, full network initial sync/default-closed
CanMove with agreed activation frame, authoritative-K restore and localized replay
with two rendered playable clients on the product dedicated server remain OPEN.
The current correction still replays from birth. Prior FAIL/UNKNOWN remain intact.


Candidate132 repairs the actual compile125 C2248: the closed early witness had
called the private root validator from a nonfriend class. Its first instruction
now calls a private static USpeedWheeledComponent helper, an existing authorized
friend, which executes the same RequireBirthRoot with that witness's ActualRoot.
This occurs before Component.Get, flags or any UObject dereference. No StorageAccess
friendship, TLS/public permission, root predicate, numerical program or test
expectation is relaxed. Compile125 remains FAIL6 and native125 NOT_RUN; candidate132
requires a distinct compilation and fresh native validation, with no prior PASS.

### Explicit Euler diagnostic fields (source133, native qualification pending)

Actual source132/build126 failed compilation with C2027 and C3861 because
initializer diagnostic captures requested an unsupported FRotator composite codec.
Source133 records Pitch, Yaw, then Roll in their actual scalar types at the eight
owned/reference/facade capture sites. Each existing Euler getter is evaluated once;
no quaternion conversion, normalization, padding, fallback codec or field omission
is introduced. Before/after comparisons, populations, physical programs, four Mass
overrides, test expectations and the closed restore guard are unchanged.
Compilation and native tests on this distinct tuple are still required. Build125
and126 remain FAIL; prior native124 results do not qualify this change. Full state,
source/lifetime exclusion, network activation and localized rollback remain open.

### Early typed diagnostic actor identity correction (native validation pending)

Actual source133/native127 recorded63 successful tests and one failed test with
24 errors. All24 typed early rows reported successful positive controls, refusals
and unchanged named buffers, but actor=0: the diagnostic read the physical registry
StableId before its later legitimate assignment. This remains a failed suite;
the next pending test was not run, and actor identity is not retrospectively fixed.

The distinct correction reads N.Seed.Actor, the genuine logical actor identifier
validated and retained before creation and early Birth controls. It does not
assign/register physical StableId early, invent an identity, remap old measurements
or relax Row.Actor!=0 or the strict per-actor checks. Physical creation, order,
programs, guards, four Mass overrides and the restore guard remain unchanged.
Fresh native execution must observe the corrected binding on the new tuple;
full-state/source-lifetime, CanMove, K and playable multiplayer gates remain open.

### Prospective ordinary parent initializer entry controls (unqualified draft)

The private final wheeled component refuses ordinary SetOwner and ordinary
InitializeBoundSubBodiesForPhysicalOwner at their first entry, before constructing
legacy access or the shared numerical program. Legacy/custom empty-default dispatch
and genuine typed reference/original factory programs remain unchanged.
Seven native attempts are authored: virtual and explicitly base-qualified SetOwner
with null/own/foreign actual actors, plus InitializeBound with no argument. Each
attempt reads the retained own/foreign owners with no actual operation and compares
six declared original/reference/facade buffers. Named component owner, COM settings,
six kinematics, chassis matrix, child arrays and the existing initializer/binding
footprint are included. Actual pending lists are copied under their real locks;
those locks end before attempts and test callbacks. Const FindStableId reads only
named membership; it never registers, applies, rebuilds or advances a frame.

This draft has zero native executions. Registry membership is partial, the private
UE async scheduling bit is UNCAPTURED, and pointed Chaos/full state/source/lifetime
and restoration remain open. No value is substituted for unavailable observation.
Existing positive legacy/custom and typed factory controls must run freshly, along
with independent same-frame input comparisons. Guards do not certify a complete
lease or authorize K restoration/two-client network acceptance.

The parent NoOp fixture observes four actual components and twenty distinct body
UObjects through twenty original and twenty reference body domains; this differs
from the ten-body scope of typed early initializer refusals. Pending-list copies
and named membership are partial observations. PublishedStableId is copied only
from original component storage; no facade back-reference equality is inferred.
