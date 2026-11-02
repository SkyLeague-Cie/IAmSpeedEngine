// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "IAmSpeed/World/Simulation/SimulationWorld.h"
#include "IAmSpeed/World/Simulation/NativeEpochStorageRoute.h"
#include "SpeedWorldSubsystem.generated.h"

namespace Speed::Netcode::Prediction {class FPrivateRoadPhysicsWorld;}
class ISpeedComponent;
#if !UE_BUILD_SHIPPING
namespace Speed { class FFramePointerRegistry; }
#endif
class USolidSubBody;
struct FCanonicalFrameContext;
namespace Speed::Input::V2 { struct FRegistryFrame; struct FInputRegistryView; class FInputSessionRegistry; }

enum class ECanonicalRunControlState : uint8
{
	Uncontrolled,
	WaitingForScenario,
	Ready,
	Complete,
};

struct FSpeedStepDiagnostics
{
	uint64 Serial = 0;
	unsigned int Frame = 0;
	int32 IterationCount = 0;
	int32 ComponentSweepCount = 0;
	int32 ResolvedEventCount = 0;
	uint32 StaticQueryCount = 0;
	uint32 LegacySweepCount = 0;
	uint32 AuthorityAttemptCount = 0;
	double StaticAuthorityMilliseconds = 0.0;
	double MaximumStaticAuthorityMilliseconds = 0.0;
	int32 MaximumIterationCount = 0;
	double PhysicalDeltaTimeMilliseconds = 0.0;
	double TotalMilliseconds = 0.0;
	double ResetMilliseconds = 0.0;
	double SweepMilliseconds = 0.0;
	double IntegrateMilliseconds = 0.0;
	double ResolveMilliseconds = 0.0;
	double ProjectionMilliseconds = 0.0;
	double PostMilliseconds = 0.0;
	bool bIterationLimitReached = false;
};

struct FPendingOp
{
    bool bAdd = false;
    ISpeedComponent* Comp = nullptr;
};

/**
 * USpeedWorldSubsystem : Subsystem responsible for managing and updating all speed components in the world.
 */
UCLASS()
class IAMSPEED_API USpeedWorldSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
#if WITH_DEV_AUTOMATION_TESTS
	friend struct FLocalNormalCameraFixtureAccess;
	friend class FIAmSpeedCollisionMatrix042Test;
#endif
#if WITH_DEV_AUTOMATION_TESTS
	friend class FIAmSpeedBoxEquilibriumWorldTest;
	friend class FIAmSpeedProducedInputWorkerOrderTest;
	friend class FIAmSpeedNativeCarsCamera1217;
 friend class FIAmSpeedNativeAnalyticContacts1221;
	friend class FIAmSpeedNativeAnalyticGrounded1368;
 friend class FIAmSpeedNativeAnalyticEightCars1534;
	friend class FSkyProducedJumpPowerslideWorkerTest;
	friend class FSkyProducedBooleanV2WorkerTest;
	friend class FSkyCameraWholePhysicalFrameCounterbalancedTest;
	friend class FSkyBumpWholePhysicalFrameDiagnosticTest;
	friend class FSkyCameraPacedPhysicalFrameCounterbalancedTest;
	friend class FSkyWaveDashAirPitchRearmTest;
	friend class FIAmSpeedProducedDeviceLifecycleTest;
	friend class FIAmSpeedControllerInputLifecycleTest;
	friend class FSLBallEpisodeAdmissionOwnerTest; // sole native floor certificate fixture
#endif
public:
	virtual void OnWorldBeginPlay(UWorld& InWorld) override;
	static bool AreUnilateralRollingPairsEnabled();
	static bool ShouldAcquireUnilateralRollingPair(float RelativeNormalSpeed);
    void RegisterSpeedComponent(ISpeedComponent* Comp);
    void UnregisterSpeedComponent(ISpeedComponent* Comp);
    void ApplyPendingOps();
    bool HasPhysicalRegistrationsForPresentationAdmission();
	#if !UE_BUILD_SHIPPING
	void RegisterFramePointerWitnessForTesting(Speed::FFramePointerRegistry& Registry,int32& ExpectedActors) const;
    void FrameAdapterCensusForTesting(TArray<FString>& OutRows) const;
    void DeclareExpectedFrameAnalogWitnessForTesting() const;
#endif
	void RegisterDynamicContactPair(
		USolidSubBody& BodyA,
		USolidSubBody& BodyB,
		const FVector& ContactPoint,
		const FVector& NormalBToA,
		float ImpactRelativeNormalSpeed);
	void ActivatePendingRollingContactPairAtTOI(
		const USolidSubBody& BodyA,
		const USolidSubBody& BodyB,
		float RemainingDt);
	bool IsDynamicContactPairOwnedByRollingManifold(
		const USolidSubBody& BodyA,
		const USolidSubBody& BodyB) const;
	ERollingManifoldContactState GetRollingManifoldContactState(
		const USolidSubBody& Body) const;

	bool NeutralizeCanonicalInputs(const Speed::Input::V2::FInputRegistryView& View);
	bool RetireInputProcessingOnWorker();
	bool ServiceInputRetirementsAtBoundary();
	bool InstallCanonicalInputs(const Speed::Input::V2::FRegistryFrame& Frame);
	bool StageCanonicalScenarioInputs(const FCanonicalFrameContext& Context,
		Speed::Input::V2::FInputSessionRegistry& Registry);
	bool PrepareCanonicalInputs(const FCanonicalFrameContext& Context);
#if !UE_BUILD_SHIPPING
    bool InspectBodyContactPairsForTesting(const USolidSubBody& Body,uint64 ExpectedPreparingFrame,Speed::FBodyContactPairInspection& Out) const;
#endif
	void PrepareCanonicalFrame(const FCanonicalFrameContext& Context);
	/** Validates every registered adapter before any adapter may prepare a frame. */
	bool ValidateSimulationBindings(FString& OutReason);
	/** Keeps the admitted registry fixed until the canonical frame has ended. */
	bool BeginCanonicalFrame(FString& OutReason);
	void EndCanonicalFrame() { bCanonicalFrameActiveEpochStorage() = false; }
	ECanonicalRunControlState GetCanonicalRunControlState();
#if !UE_BUILD_SHIPPING
    /** Owner-only observer over the already admitted registry. No pending-op
     * application, sort rebuild, ready-delay update or frame advancement. */
    ECanonicalRunControlState InspectAdmittedCanonicalRunControlForTesting() const;
#endif
	const Speed::Analytic::FAnalyticWorldData* GetAnalyticWorldData() const
	{
		return AnalyticWorldDataEpochStorage().Get();
	}
	const Speed::IStaticCollisionWorld* GetStaticCollisionWorld() const
	{
		return SimulationWorldEpochStorage().GetStaticCollisionWorld();
	}
	const FSpeedStepDiagnostics& GetLastStepDiagnostics() const
	{
		return LastStepDiagnosticsEpochStorage();
	}
	FSimulationSnapshot CaptureSimulationSnapshot(uint64 NumFrame, uint64 InputJournalHash, bool bIncludePresentation = false);
	bool CaptureReplayStateFields(TArray<FSimulationReplayStateRecord>& Out, uint64& UnsupportedStableId) const;
	void NotifyCanonicalFramePublished(uint64 NumFrame);
	ECanonicalPublicationResult PublishCanonicalFrame(uint64 Frame, TFunctionRef<bool()> Publish);
	void AbortCanonicalFrame(uint64 Frame, ECanonicalFrameAbortReason Reason) noexcept;
	/** Restores one validated canonical snapshot without advancing simulation time. */
	bool RestoreSimulationSnapshot(
		const FSimulationSnapshot& Snapshot,
		uint64 ExpectedInputJournalHash);
	/** Resolves the address assigned to a registered simulation component. */
	uint64 GetSimulationStableId(const ISpeedComponent& Component);
    ISpeedComponent* FindSimulationAdapter(uint64 StableId) const
    {
        return SimulationWorldEpochStorage().FindAdapterByStableId(StableId);
    }
	/** Applies the sealed input commands for a frame before component preparation. */
	bool ApplySimulationInputs(uint64 NumFrame, const Speed::SimulationBoundary::FInputJournal& Journal);
	TWeakObjectPtr<UPrimitiveComponent> FindAnalyticSourceComponent(
		uint64 SourceId) const;
    void Step(const float& Dt, const float& SimTime, const unsigned int& Frame);
private:
    // Whole private bridge storage is one fresh allocation. Original legacy
    // members below stay ordinary fields, including old caller views. Native
    // operations select the private epoch block internally; no facade import.
    struct FEpochBridgeStorage
    {
#if !UE_BUILD_SHIPPING
        uint64 TestingPreparingFrame=MAX_uint64;
#endif
        bool bCanonicalFrameActive=false;
        Speed::FSimulationWorld SimulationWorld;
        uint64 StepSerial=0;
        FSpeedStepDiagnostics LastStepDiagnostics;
        mutable bool bLoggedRollingOwnershipState=false;
        bool bLoggedRollingSolveState=false;
        TSharedPtr<const Speed::Analytic::FAnalyticWorldData> AnalyticWorldData;
        TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>> AnalyticSourceComponents;
        uint8 AnalyticWorldBuildAttempt=0;
        FCriticalSection PendingCS;
        TArray<FPendingOp> PendingOps;
    };
    std::shared_ptr<FEpochBridgeStorage> OriginalEpochBridgeStorage;
    std::weak_ptr<const Speed::NativeEpoch::IOriginalStorageRoute> OriginalEpochRoute;
    bool UsesOriginalEpochStorage() const
    {
        if(!IsInGameThread())return false;
        const auto Owner=OriginalEpochRoute.lock();
        return OriginalEpochBridgeStorage&&Owner&&Owner->SelectsOriginalStorage();
    }
    void BindOriginalEpochStorage(const std::shared_ptr<const Speed::NativeEpoch::IOriginalStorageRoute>& Owner)
    {
        check(Owner&&!OriginalEpochBridgeStorage&&OriginalEpochRoute.expired());
        OriginalEpochBridgeStorage=std::make_shared<FEpochBridgeStorage>();
        OriginalEpochRoute=Owner; // never copy any legacy/facade state
    }
#if !UE_BUILD_SHIPPING
    uint64& TestingPreparingFrameEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->TestingPreparingFrame:TestingPreparingFrame;}
    uint64 const& TestingPreparingFrameEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->TestingPreparingFrame:TestingPreparingFrame;}
#endif
    bool& bCanonicalFrameActiveEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->bCanonicalFrameActive:bCanonicalFrameActive;}
    bool const& bCanonicalFrameActiveEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->bCanonicalFrameActive:bCanonicalFrameActive;}
    Speed::FSimulationWorld& SimulationWorldEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->SimulationWorld:SimulationWorld;}
    Speed::FSimulationWorld const& SimulationWorldEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->SimulationWorld:SimulationWorld;}
    uint64& StepSerialEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->StepSerial:StepSerial;}
    uint64 const& StepSerialEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->StepSerial:StepSerial;}
    FSpeedStepDiagnostics& LastStepDiagnosticsEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->LastStepDiagnostics:LastStepDiagnostics;}
    FSpeedStepDiagnostics const& LastStepDiagnosticsEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->LastStepDiagnostics:LastStepDiagnostics;}
    bool& bLoggedRollingOwnershipStateEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->bLoggedRollingOwnershipState:bLoggedRollingOwnershipState;}
    bool& bLoggedRollingOwnershipStateEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->bLoggedRollingOwnershipState:bLoggedRollingOwnershipState;}
    bool& bLoggedRollingSolveStateEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->bLoggedRollingSolveState:bLoggedRollingSolveState;}
    bool const& bLoggedRollingSolveStateEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->bLoggedRollingSolveState:bLoggedRollingSolveState;}
    TSharedPtr<const Speed::Analytic::FAnalyticWorldData>& AnalyticWorldDataEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->AnalyticWorldData:AnalyticWorldData;}
    TSharedPtr<const Speed::Analytic::FAnalyticWorldData> const& AnalyticWorldDataEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->AnalyticWorldData:AnalyticWorldData;}
    TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>>& AnalyticSourceComponentsEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->AnalyticSourceComponents:AnalyticSourceComponents;}
    TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>> const& AnalyticSourceComponentsEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->AnalyticSourceComponents:AnalyticSourceComponents;}
    uint8& AnalyticWorldBuildAttemptEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->AnalyticWorldBuildAttempt:AnalyticWorldBuildAttempt;}
    uint8 const& AnalyticWorldBuildAttemptEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->AnalyticWorldBuildAttempt:AnalyticWorldBuildAttempt;}
    FCriticalSection& PendingCSEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->PendingCS:PendingCS;}
    FCriticalSection const& PendingCSEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->PendingCS:PendingCS;}
    TArray<FPendingOp>& PendingOpsEpochStorage() {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->PendingOps:PendingOps;}
    TArray<FPendingOp> const& PendingOpsEpochStorage() const {return UsesOriginalEpochStorage()?OriginalEpochBridgeStorage->PendingOps:PendingOps;}

#if !UE_BUILD_SHIPPING
    uint64 TestingPreparingFrame=MAX_uint64;
#endif
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    friend struct Speed::FBodyContactPairReadOnlyTestFixture;
#endif
	bool bCanonicalFrameActive = false; // owned by the simulation lane
	Speed::FSimulationWorld SimulationWorld;
	uint64 StepSerial = 0;
	FSpeedStepDiagnostics LastStepDiagnostics;
	mutable bool bLoggedRollingOwnershipState = false;
	bool bLoggedRollingSolveState = false;
	// Runtime data is immutable after validation. Baked assets already return a
	// finalized shared payload, so retaining it avoids a deep copy and a second
	// rebuild of every compact-provider acceleration structure at world start.
	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> AnalyticWorldData;
	// Runtime-only Unreal bridge. The analytical payload retains stable ids;
	// legacy consumers receive their source component without coupling the
	// generic query backend to UObjects.
	TMap<uint64, TWeakObjectPtr<UPrimitiveComponent>> AnalyticSourceComponents;
	uint8 AnalyticWorldBuildAttempt = 0;
	void BuildAnalyticWorldFromLoadedSources();

    void RebuildSortedIfNeeded();
	void AddComponent(ISpeedComponent& Comp);
	void RemoveComponent(ISpeedComponent& Comp);
	void SolveDynamicContactPairs(
		float Dt,
		uint64 PairKeyFilter = 0,
		bool bFilterByPairKey = false,
		bool bSolveFirstSeenFrame = false);
	void ProjectDynamicContactPairs();

    FCriticalSection PendingCS;
    TArray<FPendingOp> PendingOps;
};
