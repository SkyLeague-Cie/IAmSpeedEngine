#pragma once
#include "CoreMinimal.h"
#include "NativeConfigurationLifetimeDiagnostics.h"
#include "BranchInputFilter.h"
#include "IAmSpeed/Netcode/Common/CommonComponentState.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "IAmSpeed/World/Simulation/SimulationFrameJournal.h"
#include <atomic>
#include <memory>
#include <variant>
#include "IAmSpeed/World/Simulation/RoadSettings/DeclaredRoadSettingsRecipe.h"

class USSubBody;class ISpeedComponent;
namespace Speed::Netcode::Prediction
{
class FPrivateNativeOperation;
// Observes this GT's actual original storage-routing pointer only. This is
// diagnostic context, NOT factory completion, permission, exclusion or a lease.
// Foreign callbacks can still run inside a route; they remain observable work.
IAMSPEED_API bool IsPrivateRoadOriginalStorageRouteForDiagnostics() noexcept;
enum class EPrivateRoadStartPolicy : uint8 { AwaitInitialPublication, ExplicitUnitRecipe };
struct FPrivateRoadCarSeed
{
    U64 Actor=0,Incarnation=0;
    SKinematic Initial;
    // Network profile starts closed; countdown is only an explicit unit recipe.
    EPrivateRoadStartPolicy StartPolicy=EPrivateRoadStartPolicy::AwaitInitialPublication;
    uint16 CountdownFrames=0;
    // Explicit supplied values; null preserves the older unqualified live-settings experiment.
    std::shared_ptr<const Speed::RoadSettings::FDeclaredRecipe> DeclaredSettings;
    EBranchFilterRole Role=EBranchFilterRole::RemoteRequests;
    std::shared_ptr<const BranchV2::FInputActionContract> Contract;
    BranchV2::FInputProcessingPolicy Processing;
    Speed::Input::FProducerIdentity Local;
    BranchV2::FStreamEpoch LocalEpoch;
    FBinding Remote;
    U64 PredictionHorizon=256;
};
struct FPrivateRoadRequest
{
    U64 Actor=0,Incarnation=0;
    std::variant<std::shared_ptr<const BranchV2::FInputFrame>,FPrediction> Evidence;
};
struct FPrivateRoadActorFrame
{
    U64 Actor=0,Incarnation=0,NativeStableId=0;
    FBranchFilterState Filter;
    std::shared_ptr<const FBranchProcessedInput> Input;
    bool CanMove=false,Frozen=false;
    uint16 CountdownFrames=0; // actual native values, diagnostics only
    // Identity observations at the real owner capture, under its original
    // routing scope. No reference escapes; these counts grant no lease.
    uint8 PublicComponentKinematicViews=0;
    uint8 PublicComponentKinematicFacadeViews=0;
    uint8 PublicComponentKinematicOriginalViews=0;
    uint8 PublicPhysReferenceViews=0;
    uint8 PublicPhysReferenceFacadeViews=0;
    uint8 PublicPhysReferenceOriginalViews=0;
    uint8 PublicBodyKinematicViews=0;
    uint8 PublicBodyKinematicFacadeViews=0;
    uint8 PublicBodyKinematicOriginalViews=0;
    uint8 PublicContactReferenceViews=0;
    uint8 PublicContactReferenceFacadeViews=0;
    uint8 PublicContactReferenceOriginalViews=0;
    uint8 PublicContactPredicateChecks=0;
    uint8 PublicContactPredicateMatches=0;
    // Declared PARTIAL observation only. Missing is UNKNOWN, never zero/full.
    bool NativePartialValuesAvailable=false;
    CommonState::FSleepValues NativeSleep;
    CommonState::FQuantizationValues NativeQuantization;
    std::vector<uint8> NativeWheelSuspensionValues; // normalized config references
    std::vector<FQuat> NativeWheelRenderRotations; // actual per-wheel diagnostic copies
    // Exact six original body structs (126 declared members plus nested hit/render values).
    // Component, driver, native vehicle, source and lifetime domains are NOT covered.
    bool OriginalBodyDomainsAvailable=false;
    std::vector<uint8> OriginalBodyDomainValues;
    // Actual eleven top-level private component value members. References use
    // the original graph table; missing/stale/nonfinite remains UNKNOWN.
    // This read-only domain is not the driver/world/config/lifetime closure.
    bool OriginalComponentDomainAvailable=false;
    std::vector<uint8> OriginalComponentDomainValues;
    bool NativeSolidInitializationWitness=false; // actual same-run recipe/value/binding witness only
    std::vector<uint8> NativeSolidValues; // twenty-one declared solid fields, not complete configuration
    std::vector<uint8> NativeWheelRuntimeValues; // fifteen declared runtime blocks only
    FVector NativeHitboxHalfExtent=FVector::ZeroVector;
    FVector NativeHitboxCenter=FVector::ZeroVector; // actual queried child storage, not reconstructed COM
};
// Observed actual private native box dispatches and wheel contact registrations.
// Counts are conservative dependencies, including zero impulse. Registration is
// not proof of impulse; these events do not certify complete contact coverage.
struct FPrivateRoadContactDispatchPair
{
    U64 ActorA=0,ActorB=0;uint32 Dispatches=0,BoxWheelDispatches=0,WheelRegistrations=0;
    bool operator==(const FPrivateRoadContactDispatchPair& R) const noexcept
    {return ActorA==R.ActorA&&ActorB==R.ActorB&&Dispatches==R.Dispatches&&BoxWheelDispatches==R.BoxWheelDispatches&&WheelRegistrations==R.WheelRegistrations;}
};
struct FPrivateRoadPublication
{
    U64 LocalKernelLifetime=0,Frame=0;
    bool AwaitingInitialPublication=false;
    std::vector<uint8> DeclaredSettingsIdentity; // full canonical declared-slot bytes, not full config
    uint32 NativeNetworkComponents=0; // actual object census, not just history
    uint32 NativeDebugDelegateBindings=0; // bound entries; not an in-flight-reader drain certificate
    bool RenderScenePresent=false,PhysicsScenePresent=false; // actual private world scene pointers
    U64 VehicleContactFrameCount=0; // actual events in this original graph, diagnostic only
    bool ContactDispatchObservationComplete=false; // bounded event storage only, not complete physical coverage
    std::vector<FPrivateRoadContactDispatchPair> ContactDispatchPairs;
    FSimulationSnapshot Physics;
    std::vector<FPrivateRoadActorFrame> Actors;
};
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
enum class EPrivateRoadBirthProbeMode : uint8 { Observe,FailAfterFirstInitializer,ProbeFirstOwnerAndAbort };
struct FPrivateRoadBirthInitializationProbe
{
    // Diagnostic VALUES only: entered/returned stages and actual pose API results.
    // These do not grant operation, source/lifetime or restore authority.
    FString FirstOwnerLastStage=TEXT("not-entered"),FirstOwnerCreateWhy;
    bool FirstOwnerProbeCallEntered=false,FirstOwnerProbeCallReturned=false,FirstOwnerProbeSucceeded=false;
    bool FirstOwnerActorLocationSet=false,FirstOwnerActorRotationSet=false;
    FVector FirstOwnerActorLocationReadback=FVector::ZeroVector;
    FRotator FirstOwnerActorRotationReadback=FRotator::ZeroRotator;
    uint32 InitializersReturned=0; // SECOND pass, after native vehicle/shape authoring
    uint32 FirstOwnerInitializersReturned=0,FirstOwnerInitialNullParents=0,FirstOwnerGuardRefusals=0;
    bool FirstOwnerProbeReturned=false,FirstOwnerAbortReached=false,FirstOwnerAllRefusalsBeforeWrite=false;
    bool FirstOwnerPoseAndCOMMatch=false,FirstOwnerClearKeepsConfigurations=false;
    bool FirstOwnerStoredChassisTensorMatch=false,FirstOwnerStoredTensorDistinguished=false,FirstOwnerChildrenMatch=false;
    bool FirstOwnerWheelVelocityOrderDistinguished=false;
    uint32 WheelBindingFactoryReturns=0,WheelBindingGuardRefusals=0,WheelBindingPositiveEntryReturns=0;
    bool WheelBindingProbeReturned=false,WheelBindingAllRefusalsBeforeWrite=false;
    bool WheelBindingPositiveValuesMatch=false,WheelBindingOverridePredicatesMatch=false;

    bool ActualBirthOperation=false,ForeignBodyRefused=false,ForeignParentRefused=false,ForeignInverseRefused=false;
    bool StepHitRefusedInBirth=false,OwnBodyUnchangedAfterRefusals=false;
    bool StepResetRefusedInBirth=false,StepResolveRefusedInBirth=false;
    bool FaultAfterFirstInitializerReached=false,CreateReturnedNull=false;
    // Only existing independent actual constructor/destructor counters escape.
    // No own graph, simulation, configuration, UObject or native view retained.
    std::vector<std::shared_ptr<const FNativeConfigurationLifetimeTrace>> LifetimeObservers;
};
struct FPrivateRoadHitAcceptanceCase
{
    FString Name;bool LegacyExpected=false,NativeExpected=false,LegacyNativeEqual=false;
};
struct FPrivateRoadHitAcceptanceProbe
{
    std::vector<FPrivateRoadHitAcceptanceCase> Cases;
    uint32 LegacyCalls=0,NativeCalls=0,Refusals=0;
    bool BaseBehaviorDistinguished=false,AllRefusalsBeforeWrite=false,ActualStepOperation=false;
    bool BirthInitializationRefusedDuringStep=false;
    bool BirthWheelBindingRefusedDuringStep=false;
    bool BirthOwnerTraversalRefusedDuringStep=false;
    uint32 MassBoundaryRefusals=0,MassLegacyResetReturns=0,MassNativeResetReturns=0,MassNoHitResolveReturns=0;
    bool MassRefusalsBeforeWrite=false,MassResetLegacyNativeEqual=false,MassResetDistinctBoxWheel=false,MassNoHitUnchanged=false;

};
#endif
// Experimental finite native flat-road kernel. Creates actual, private native
// cars and runs the unchanged canonical IAmSpeed solver at300Hz on the GT.
// This public API returns immutable VALUES. Engine/global enumeration and
// previously escaped native views require separate exclusion qualification.
// This factory does not authenticate seeds/input, select a connected client,
// supply a complete restore provider or certify lifecycle/view exclusion for
// an ISimulationRestoreBoundaryLease. Unsupported scene/game domains stay closed.
class IAMSPEED_API FPrivateRoadPhysicsWorld final
{
    struct FImpl;
    struct FRetainedNativeCall;
    std::shared_ptr<FImpl> Impl;
    // Private operation-memory retention only. No source/restore lease follows.
    static bool HasExactRegisteredRosterRetained(const std::shared_ptr<FImpl>& Root,const FPrivateNativeOperation& Operation,FString& Why);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    struct FHitAcceptanceDiagnosticContext;
    static bool ProbeHitAcceptanceRetained(FImpl& I,const FPrivateNativeOperation& Operation,FHitAcceptanceDiagnosticContext& Context);
    static bool ProbeMassResponseRetained(FImpl& I,const FPrivateNativeOperation& Operation,FHitAcceptanceDiagnosticContext& Context);
#endif
    static bool StepRetained(const std::shared_ptr<FImpl>& Root,const std::vector<FPrivateRoadRequest>& Requests,bool Resimulation,FString& Why
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        ,FHitAcceptanceDiagnosticContext* HitContext=nullptr
#endif
        );
    static bool ReleaseInitialPublicationRetained(const std::shared_ptr<FImpl>& Root,const std::shared_ptr<const FPrivateRoadPublication>& Baseline,FString& Why);
    static std::vector<FNativeConfigurationLifetimeValues> ReadNativeConfigurationLifetimeRetained(const std::shared_ptr<FImpl>& Root);
    static bool CloseRetained(const std::shared_ptr<FImpl>& Root);
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    struct FBirthInitializationDiagnosticContext;
    static bool ProbeBirthInitializationGuards(FImpl& I,const FPrivateNativeOperation& Operation,
        FBirthInitializationDiagnosticContext& Context,USSubBody* Body,ISpeedComponent* Parent);
#endif
    static std::unique_ptr<FPrivateRoadPhysicsWorld> CreateCore(std::vector<FPrivateRoadCarSeed> Seeds,FString& Why
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
        ,FBirthInitializationDiagnosticContext* BirthContext=nullptr
#endif
        );
    FPrivateRoadPhysicsWorld();

public:
    ~FPrivateRoadPhysicsWorld();
    FPrivateRoadPhysicsWorld(const FPrivateRoadPhysicsWorld&)=delete;
    FPrivateRoadPhysicsWorld& operator=(const FPrivateRoadPhysicsWorld&)=delete;
    static std::unique_ptr<FPrivateRoadPhysicsWorld> Create(std::vector<FPrivateRoadCarSeed> Seeds,FString& Why);
    bool Step(const std::vector<FPrivateRoadRequest>& Requests,bool Resimulation,FString& Why);
    std::shared_ptr<const FPrivateRoadPublication> ReadLatest() const noexcept;
    // Local-owner transition after the session has compared its ACTUAL startup
    // publication with the original server and received the common activation.
    // This is NOT a complete hidden-state/restore certificate or source admission.
    bool ReleaseInitialPublication(const std::shared_ptr<const FPrivateRoadPublication>& OriginalBaseline,FString& Why);
    // Immutable actual C++ return-site values; no source/lifetime exclusion or lease follows.
    std::vector<FNativeConfigurationLifetimeValues> ReadNativeConfigurationLifetimeForDiagnostics() const;
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    struct FOriginalWorldReadProbe {bool Admitted=false,SameOriginal=false;};
    // Fixed test-only observations/fault injection in the same real factory.
    // A failure mode can only abort; it never grants an operation or lease.
    static std::unique_ptr<FPrivateRoadPhysicsWorld> CreateWithBirthInitializationProbeForDiagnostics(
        std::vector<FPrivateRoadCarSeed> Seeds,FPrivateRoadPhysicsWorld& Foreign,EPrivateRoadBirthProbeMode Mode,
        FPrivateRoadBirthInitializationProbe& Result,FString& Why);
    // Fixed native cache fixtures inside a real advancing Step. Only copied
    // results escape; the caller cannot supply a native pointer or callback.
    bool StepWithHitAcceptanceProbeForDiagnostics(const std::vector<FPrivateRoadRequest>& Requests,
        bool Resimulation,FPrivateRoadPhysicsWorld& Foreign,FPrivateRoadHitAcceptanceProbe& Result,FString& Why);
    // Read-only diagnostic executed by the actual owner. No caller recipe,
    // pointer, callback, native view, physical route or restore authority.
    FOriginalWorldReadProbe ObserveOriginalWorldReadForDiagnostics();
    // Actual independent counter objects; retaining them does not retain any
    // native simulation/configuration or provide source/lifetime authority.
    std::vector<std::shared_ptr<const FNativeConfigurationLifetimeTrace>> CaptureNativeConfigurationLifetimeObserversForDiagnostics();
private:
    static FOriginalWorldReadProbe ObserveOriginalWorldReadRetained(const std::shared_ptr<FImpl>& Root);
public:
#endif
    bool Close(); // GT only; never destroys native storage in a physical/publication tail
};
}
