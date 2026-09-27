# A5b1 canonical publication infrastructure — source only

Base de5cd683e17aee6f756f9cab35089bf598541137. Tests prepared, uncompiled and
unexecuted. This does NOT connect V2 to controller/car/wheeled inputs, resolve
UObject lifetime or qualify any Unreal target.

ISpeedComponent gains three default no-op hooks: const validation, noexcept
commit and noexcept abort. FSimulationWorld owns their ordered orchestration;
USpeedWorldSubsystem forwards it. The stable Bodies list belongs to the admitted
canonical frame, so GT registration is deferred by the existing frame boundary.
No new lock is held across simulation, hooks or publication.

All participants are validated before the supplied global publisher is called.
A false validation (including one after an earlier successful participant) or
validation exception aborts all. Global publication false/exception also aborts
all with SnapshotPublicationFailed. No input commit has happened at that point.
After global success, each prevalidated commit hook must finalize without
allocation or exceptions. Its bool failure is an invariant violation, returned
explicitly as CommitInvariantFailed; already finalized inputs are NEVER aborted.
The caller logs that result and permanently terminates the simulation lane.

The hooks express an obligation on future implementors, not a proof that an
arbitrary V2 PublishCompleted call is allocation-free. V2 streams currently have
no hook implementation. Their future adapter must stage any required storage
before the irreversible global publication and prove the finalization contract.
noexcept makes a thrown hook a fatal language-level contract violation, not a
recoverable publisher exception. Abort hooks also must not allocate or throw.

StepCanonicalFrame routes its actual SnapshotBuffer.Publish through the new
world transaction. The worker checks serial exhaustion first. The existing
FSnapshotBuffer writes the inactive slot under its mutex: all validation and
potentially throwing snapshot/TArray copies occur before serial increment and
PublishedSlot exchange; the remaining work is scalar serial assignments and
iteration of existing output entries. Thus false/ordinary C++ exception before
the exchange leaves the previous snapshot/serial visible. Unreal fatal OOM or
process termination is not recoverable C++ exception evidence. Snapshot buffer
source itself is unchanged in this delta.

New authoritative commit happens immediately after global success. The legacy
OnCanonicalFramePublished observation follows, then camera output and frame-hash
append. A later observer/camera/hash exception terminates the lane but never
aborts the already authoritative input transaction. Both BeginCanonicalFrame
and post-admission C++ exceptions are contained. Scoped cleanup calls
EndCanonicalFrame and StaticWorldQueryAudit::EndFrame on every admitted exit.
Preparation/input failures abort before publication. A dedicated terminal latch
cannot be cleared by ordinary pause/resume or same-actor controlled restart;
failed state needs an explicitly new simulation owner/run. No rollback or
restoration of partially simulated physical state is claimed.

## Exception and binary scope

IAmSpeed.Build.cs explicitly enables bEnableExceptions=true. This changes
module-wide code generation/unwind tables and PCH compatibility, not just input
files. Windows exception paths require /EHsc in actual UBT commands. Final
Windows Game/Editor and supported server target builds remain required; any
5.7.4 gate is PROVISIONAL_ENGINE_5_7_4. No performance/size result is claimed.

Catch/throw implementations remain in private .cpp files. Public interfaces
add enums and virtual methods but no exception-bearing inline implementation;
dependent modules do not acquire an implicit requirement to compile these catch
bodies. The new ISpeedComponent virtual layout is NOT ABI-compatible with old
binary consumers: rebuild the module and every derived/using module together.
SkyLeague/SLCorePhysics already enable exceptions locally; SLTest does not and
is unchanged. Templates/headers in future V2 wrapper work need another audit.
No C++ exception should cross from the new private orchestration into an Unreal
worker or actor callback. noexcept hook violations remain fatal invariants.

## Prepared tests and limits

CanonicalPublicationTests.cpp uses value-only ISpeedComponent fixtures with the
actual FSimulationWorld routing and FSnapshotBuffer. It checks validation of all
participants, failure at either participant, validation throw, actual oversized
snapshot rejection, injected publisher throw before visibility, successful
publication ordering, and a post-global commit invariant failure. Previous
snapshot and serial survive failures; every participant aborts before publication;
no completed participant is aborted after global success. Test commit/abort hooks
use counters only and allocate nothing.

These tests do not instantiate a vehicle or execute an actual worker physics
frame. The actor terminal latch/RAII paths require a later focused host fixture
and source review; no claim is made from the helper tests alone. No profile,
assets, live device, parent fixture, wrapper opt-in or destruction code changed.
Default V1 hooks do nothing; successful V1 input observation occurs earlier in
the same completed-frame path, before camera/hash, which needs regression coverage
when a qualified engine runner is approved.
