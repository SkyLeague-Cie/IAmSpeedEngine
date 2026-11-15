#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "NativeOrdinaryIntegrationValues.h"
namespace Speed::PreparationPrefix { class FCursor; }
namespace Speed::Collision { class FResolvedPairSet; }
struct SComponentTOI;
struct FCanonicalFrameContext;
class ASpeedPhysicsSimulation;class USpeedWorldSubsystem;class UWorld;class FSimulationWorker;class USSubBody;class USphereSubBody;class USolidSubBody;class ISpeedComponent;class UBallSubBody;class UBoxSubBody;
#if WITH_DEV_AUTOMATION_TESTS
class FPreparedPhysicalExecutionTest;
#endif
namespace Speed
{
struct SHitResult;
class FPreparedNativeFrameConsumer;class IStaticCollisionWorld;class FSimulationWorld;class FPreparedPhysicalQueryDomain;class FPreparedWorldSnapshot;struct FPreparedNativeOwnerBinding;
namespace Analytic {struct FWorldQuery;}
namespace Analytic::Certified {struct FDistanceBudget;}
namespace Analytic::Prepared {class FScene;}
namespace OnePausedWork {class FCanonicalFrameWindow;}
namespace PreparedExecution
{
struct FOrdinarySweepReturnState;struct FOrdinaryWorldSelectionState;class FOrdinaryOwnerProbePhaseScope;
struct FOrdinaryResponseState;struct FOrdinaryConsumedStateValues;struct FOrdinaryIntegrationState;
struct FOrdinaryNativeAdvanceTicket;struct FOrdinaryFinalState;class FClosedPlanarResponseEffect;
struct FFixedNoPoseSweepBaseline;
enum class ENativeFrameFinalOperation:uint8 {StaticProjectionOwner,DynamicProjectionBeforePost,PostOwner,MergeRollingPairs,SolvePairs,DynamicProjectionAfterPost};
enum class EOrdinaryBodyInertiaOrigin:uint8 {BoxRotatedLocalOperator,SphereCachedOperator};
// Values from the REAL World filter branch. SurfaceId0 has no pair filter;
// a false ResolverValid means its repeated-contact virtual was not called.
struct FOrdinaryWorldFilterValues
{
    bool ResolverValid=false,EligibleRepeated=false,Applied=false,PairWasMember=false;
    bool ProgressRead=false,OrdinaryRead=false,Allowed=false;
};
class FGroundSweepEvidence;class FComponentSweepEvidence;class FNativeSphereRequest;class FNativeSolidStaticRequest;class FOrdinaryGroundSweepEvidence;
class FOrdinarySleepingSupportObservation;struct FNativeSleepingSupportDecisionValues;
class FNativeSpherePostSyncRsCapture;class FPostSyncRsNativeState;struct FPostSyncRsStepObservation;
class FNativeSphereRsTrajectoryCapture;class FTerminalRsCaptureStop;struct FNativeRsTraceValues;struct FTerminalRsStepObservation;struct FTerminalConsumedPrefixAdvanceObservation;class FFirstRsResponseData;struct FTerminalFirstRsResponseObservation;struct FTerminalFirstRsResponseSyncObservation;enum class EFirstRsResponseCall:uint8;
// Refusal diagnostics ONLY. There is intentionally no Admitted enum value:
// generic declarations and pure census tests cannot mint a native capability.
enum class ECanonicalCensusRefusal:uint8
{PendingRegistry,DirtyOrder,IncompleteInventory,DuplicateOwner,MissingNativeMetadata,InvalidChildCensus,NoCompiledPhaseCoverage};
IAMSPEED_API ECanonicalCensusRefusal InspectCanonicalCensusForRefusal(
    TConstArrayView<FPreparedNativeOwnerBinding> Rows,TConstArrayView<ISpeedComponent*> Adapters,
    TConstArrayView<ISpeedComponent*> Ordered,bool HasPending,bool OrderDirty);
IAMSPEED_API bool RejectPreparedPendingMutation(bool PreparedWorkActive,bool HasPending) noexcept;
IAMSPEED_API bool RejectPreparedOrderMutation(bool PreparedWorkActive,bool OrderDirty) noexcept;
// Earliest native admission boundary. This first lot has NO positive phase
// contracts: every complete census still refuses before BeginCanonicalFrame.
// Strong UObject pins remain with the real GT owner, never this native scope.
class IAMSPEED_API FCanonicalFramePreflight final
{
    friend class ::ASpeedPhysicsSimulation;friend class FStepScope;
    static bool HasCurrentNativeRunControlContext(const ASpeedPhysicsSimulation&,const USpeedWorldSubsystem&,
        const FSimulationWorker*,uint64,uint64,float,float);

    TSharedPtr<const FPreparedNativeFrameConsumer,ESPMode::ThreadSafe> NativeConsumer;
    TSet<const ISpeedComponent*> PreparedOwners;
    bool NativeStepClosed=false,SnapshotClosureConsumed=false;
    TUniquePtr<PreparationPrefix::FCursor> PreparationCursor;
    TSharedPtr<const OnePausedWork::FCanonicalFrameWindow,ESPMode::ThreadSafe> Window;
    TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe> Snapshot;
    const ASpeedPhysicsSimulation* Host=nullptr;const USpeedWorldSubsystem* Bridge=nullptr;
    const FSimulationWorker* Worker=nullptr;
    // Ordinal/epoch reserved ONLY; this refusal lot installs no phase protocol.
    uint64 Generation=0,Frame=0,StepBefore=0,WindowSerial=0,Nonce=0,PhaseOrdinal=0,PhaseEpoch=0;
    uint32 ThreadId=0,DeltaBits=0,TimeBits=0;
    FCanonicalFramePreflight(bool Required,const ASpeedPhysicsSimulation&,const USpeedWorldSubsystem&,
        const FSimulationWorker*,uint64 Generation,uint64 Frame,float Delta,float SimTime);
public:
    static bool HasUnchangedRawPreparationCensus(const USpeedWorldSubsystem&, const FPreparedWorldSnapshot&, uint64 ExpectedStep);
    static bool DispatchNativePrepare(ISpeedComponent&,const FCanonicalFrameContext&);
    void RequireClosedForSnapshot();
    ~FCanonicalFramePreflight() noexcept;
    FCanonicalFramePreflight(const FCanonicalFramePreflight&)=delete;
    FCanonicalFramePreflight& operator=(const FCanonicalFramePreflight&)=delete;
    FCanonicalFramePreflight(FCanonicalFramePreflight&&)=delete;
    FCanonicalFramePreflight& operator=(FCanonicalFramePreflight&&)=delete;
};
// These native identities are lexical process-local witnesses, not authored
// surface IDs, serialized gameplay state or permission to consume geometry.
struct FStepIdentity
{
    const ASpeedPhysicsSimulation* Host=nullptr;const USpeedWorldSubsystem* Bridge=nullptr;const UWorld* World=nullptr;
    const FSimulationWorker* Worker=nullptr;const IStaticCollisionWorld* Provider=nullptr;const void* Lease=nullptr;
    uint64 InputGeneration=0,Frame=0,StepSerial=0,QuerySerial=0,QueryEpoch=0,NumericControl=0;
    uint32 ThreadId=0;int32 Iteration=0;float Delta=0,SimTime=0,Remaining=0,TimePassed=0;
    uint64 WindowSerial=0,WindowNonce=0;
};
#if WITH_DEV_AUTOMATION_TESTS
// Pure identity/value fixtures only; no actual setter, UObject, worker or frame.
IAMSPEED_API bool TestOrdinarySetFutureHitValueClosure(FString& Reason);
IAMSPEED_API bool TestOrdinaryPrimitiveValueClosure(FString& Reason);
#endif
IAMSPEED_API bool Requested();
// Failure-only: never resets or admits a frame.
IAMSPEED_API void PoisonNativeRotationFailure() noexcept;
IAMSPEED_API bool IsCurrentWorker(const FSimulationWorker* Expected) noexcept;
IAMSPEED_API bool ReadCurrentStep(FStepIdentity& Out) noexcept;
IAMSPEED_API bool NextSphereQuery(FStepIdentity& Out) noexcept;
// Ordinary FullFrame solid calls only; preserves terminal Sphere serial contracts.
IAMSPEED_API bool NextNativeSolidQuery(FStepIdentity& Out) noexcept;
IAMSPEED_API bool ReadCurrentSnapshot(TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe>& Out) noexcept;
IAMSPEED_API bool ReadCurrentInputs(TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe>& Scene,
    TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe>& Filters) noexcept;
// A float margin is read ONLY by the actual sphere member, never reconstructed
// from the inflated collision radius. Bit validation is FTZ/DAZ independent.
IAMSPEED_API bool IsSupportedSphereResponseMargin(float Margin) noexcept;
IAMSPEED_API bool ReadSphereMemberSweep(const USphereSubBody* Body,const void* Owner,uint64& Serial,float& Margin) noexcept;
class IAMSPEED_API FSphereMemberSweepScope final
{
    friend class ::USphereSubBody;friend class ::UBallSubBody;friend class FComponentSweepScope;
    friend IAMSPEED_API bool ReadSphereMemberSweep(const USphereSubBody*,const void*,uint64&,float&) noexcept;
    bool Active=false,Completed=false,Interrupted=false;const USphereSubBody* Body=nullptr;const void* Owner=nullptr;
    FStepIdentity Step;uint64 Serial=0;float Margin=0;
    TSharedPtr<FGroundSweepEvidence,ESPMode::ThreadSafe> PendingGround;
    TSharedPtr<FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> PendingOrdinaryGround;
    void FinishOrdinaryGround(bool,const SHitResult&,float);
    TSharedPtr<FOrdinarySleepingSupportObservation,ESPMode::ThreadSafe> PendingSleepingSupport;
    static void ObserveOrdinarySleepingSupport(const USphereSubBody*,float,
        TFunctionRef<bool(FNativeSleepingSupportDecisionValues&)>);
    void FinishSleepingSupport(bool,const SHitResult&,float);
    TSharedPtr<const FNativeRsTraceValues,ESPMode::ThreadSafe> Trace;
    FSphereMemberSweepScope(const USphereSubBody* InBody,const void* InOwner,float ActualVirtualMargin,const FNativeRsTraceValues* ActualTrace=nullptr);
    void CloseTerminalRsCapture(const FTerminalRsCaptureStop&,float RereadMargin,const FNativeRsTraceValues&);
    static bool IsPostSyncRsCaptureRequested();
    [[noreturn]] static void CapturePostSyncRsAndStop(TSharedPtr<const FNativeSphereRequest,ESPMode::ThreadSafe>);
    void ClosePostSyncRsCapture(const FTerminalRsCaptureStop&,float,const FNativeRsTraceValues&);
    bool HasClosedInterruptedPostSyncRsMember() const noexcept;
    void Finish(float RereadActualVirtualMargin,bool DeliveredHit,const SHitResult& Delivered,float DeliveredSeconds);
public:
    // Runs only in the actual live sphere member scope. Complete union
    // exclusion can be recorded; any candidate still requires contact proof.
    static bool IsTerminalRsCaptureRequested();
    static bool IsTerminalRsCapture();
    [[noreturn]] static void CaptureRsAndStop(TSharedPtr<const FNativeSphereRequest,ESPMode::ThreadSafe> Request);
    static bool RecordGroundSweep(TSharedPtr<const FNativeSphereRequest,ESPMode::ThreadSafe> Request,SHitResult& OutHit);
    static bool ReadBody(const USolidSubBody* RawSolid,const USphereSubBody*& Body,const void*& Owner) noexcept;
    ~FSphereMemberSweepScope() noexcept;
    FSphereMemberSweepScope(const FSphereMemberSweepScope&)=delete;
    FSphereMemberSweepScope& operator=(const FSphereMemberSweepScope&)=delete;
    FSphereMemberSweepScope(FSphereMemberSweepScope&&)=delete;
    FSphereMemberSweepScope& operator=(FSphereMemberSweepScope&&)=delete;
};
class IAMSPEED_API FWorkerCallScope final
{
    friend class ::FSimulationWorker;
#if WITH_DEV_AUTOMATION_TESTS
    friend class ::FPreparedPhysicalExecutionTest;
#endif
    const FSimulationWorker* Worker=nullptr;uint32 ThreadId=0;
    explicit FWorkerCallScope(const FSimulationWorker& InWorker);
public:
    ~FWorkerCallScope() noexcept;
    FWorkerCallScope(const FWorkerCallScope&)=delete;FWorkerCallScope& operator=(const FWorkerCallScope&)=delete;
    FWorkerCallScope(FWorkerCallScope&&)=delete;FWorkerCallScope& operator=(FWorkerCallScope&&)=delete;
};
class IAMSPEED_API FHostCallScope final
{
    friend class ::ASpeedPhysicsSimulation;
    bool Active=false;const ASpeedPhysicsSimulation* Host=nullptr;const USpeedWorldSubsystem* Bridge=nullptr;
    const UWorld* World=nullptr;const FSimulationWorker* Worker=nullptr;uint64 Generation=0,Frame=0;uint32 ThreadId=0;
    float Delta=0,SimTime=0;
    FHostCallScope(bool Required,const ASpeedPhysicsSimulation& InHost,const USpeedWorldSubsystem& InBridge,
        const FSimulationWorker* InWorker,uint64 InGeneration,uint64 InFrame,float InDelta,float InTime,bool OwnerAvailable);
    friend class FStepScope;
    friend class FCanonicalFramePreflight;friend class FOrdinaryOwnerProbePhaseScope;friend class FNativeWheelProbeScope;
    mutable uint64 OwnerProbePhaseOrdinal=0;
public:
    ~FHostCallScope() noexcept;
    FHostCallScope(const FHostCallScope&)=delete;FHostCallScope& operator=(const FHostCallScope&)=delete;
    FHostCallScope(FHostCallScope&&)=delete;FHostCallScope& operator=(FHostCallScope&&)=delete;
};
namespace CurvedBoxArrivalDetail { struct FCase976Step51RemainderAdmission; struct FCase585PostFifthRotationAdmission; }
class IAMSPEED_API FStepScope final
{
    friend struct CurvedBoxArrivalDetail::FCase976Step51RemainderAdmission;
    friend struct CurvedBoxArrivalDetail::FCase585PostFifthRotationAdmission;
    friend class ::USpeedWorldSubsystem;friend class ::ISpeedComponent;
    friend class FSphereMemberSweepScope;friend class FComponentSweepScope;friend class FOrdinaryOwnerProbePhaseScope;friend class FNativeOwnerPoseMutationScope;friend class FNativePlanarResponseCommitScope;
    friend class FNativeSphereProjectionScope;
    friend IAMSPEED_API bool ReadCurrentStep(FStepIdentity&) noexcept;friend IAMSPEED_API bool NextSphereQuery(FStepIdentity&) noexcept;
    friend IAMSPEED_API bool NextNativeSolidQuery(FStepIdentity&) noexcept;
    friend IAMSPEED_API bool ReadCurrentInputs(TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe>&,
        TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe>&) noexcept;
    friend IAMSPEED_API bool ReadCurrentSnapshot(TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe>&) noexcept;
    TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe> Snapshot;
    TSharedPtr<const OnePausedWork::FCanonicalFrameWindow,ESPMode::ThreadSafe> Window;
    bool Active=false,SweepPhase=false;FStepIdentity Identity;const FSimulationWorld* Registry=nullptr;
    TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe> Scene;
    TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe> Filters;
    FStepScope(bool Required,const USpeedWorldSubsystem& Bridge,const FSimulationWorld& InRegistry,
        const IStaticCollisionWorld* Provider,uint64 Serial,uint32 Frame,float Delta,float SimTime,
        TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe> InScene,
        TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe> InFilters);
    TUniquePtr<FOrdinarySweepReturnState> OrdinaryReturns;
    TUniquePtr<FOrdinaryResponseState> OrdinaryResponse;
    TArray<TSharedPtr<const FClosedPlanarResponseEffect,ESPMode::ThreadSafe>> ClosedPlanarValuesForObservation;
    bool HasClosedPlanarValuesForObservation(const UBoxSubBody*) const;
#if !UE_BUILD_SHIPPING
    TSharedPtr<FFixedNoPoseSweepBaseline,ESPMode::ThreadSafe> FixedNoPoseSweepBaseline;
    void CaptureFixedNoPoseSweepBaseline(int32 iteration,float remaining,float timePassed);
#endif
    TArray<TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe>,TInlineAllocator<4>> ClosedCurvedBoxGroundForObservation;
    bool HasClosedCurvedBoxGroundForObservation(const FNativeSolidStaticRequest&,const SHitResult&) const;
    bool IsPreparedCurvedBoxExactProgress(const ::SComponentTOI&,const USSubBody*,bool Consumed) const;
#if !UE_BUILD_SHIPPING
    TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> Case585FourthGround,Case585FifthGround;
    uint64 Case585Pair=0;
    bool Case585AcceptingFifth=false,Case585FifthResolved=false,Case585FifthDeltaVUsed=false;
    void BindCase585CurvedReimpact(const TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe>&);
#endif
    TUniquePtr<FOrdinaryIntegrationState> OrdinaryIntegration;
    TSharedPtr<const FPreparedNativeFrameConsumer,ESPMode::ThreadSafe> NativeConsumer;
    TUniquePtr<FOrdinaryNativeAdvanceTicket> NativeAdvance;
    TUniquePtr<FOrdinaryFinalState> OrdinaryFinal;
    void RequireNativeFrameContext() const;
    void ConsumeNativeAdvanceTicket(float,bool,TConstArrayView<::SComponentTOI>,float,float);
    bool DispatchNativeIntegration(ISpeedComponent&,float);
    bool DispatchNativeResponse(int32,USSubBody&,const SHitResult&,float,float);
    void RequireOrdinaryFinalContext() const;
    void BeginNativeFinalOperation(ENativeFrameFinalOperation,const ISpeedComponent* Owner=nullptr,int32 Index=INDEX_NONE,bool Called=true);
    void EndNativeFinalOperation();
    bool DispatchNativeFinalOwner(ISpeedComponent&,float,bool Post);
    void CloseNativeFrameStep(float ActualTime);

    void RequireOrdinaryFrameStateReadContext() const;
    void ReadOrdinaryFrameStateValues(FOrdinaryConsumedStateValues&) const;
    TArray<const ISpeedComponent*> GeneralBoxOwners;
    const FOrdinaryConsumedStateValues& GeneralBoxCurrentClosedBaseline(bool RequireClosedBatch=true) const;
    void RequireGeneralBoxUnchanged(const FOrdinaryConsumedStateValues&,const FOrdinaryConsumedStateValues&,
        const ISpeedComponent* Except=nullptr) const;
    void ObserveGeneralBoxStage(struct FOrdinaryIntegrationOwnerValues&,const struct FOrdinaryIntegrationStageValues&);
    void ObserveGeneralBoxChildSync(struct FOrdinaryIntegrationOwnerValues&);
    void RequireOrdinaryIntegrationContext(bool AllowExactPostProbe=false) const;
    bool HasOrdinaryIntegrationObservation() const;
    void BeginOrdinaryIntegration(float Seconds,bool HasSelection,TConstArrayView<::SComponentTOI>,float Horizon,float ActualBefore);
    void BeginOrdinaryIntegrationOwner(const ISpeedComponent*,int32);
    void EndOrdinaryIntegrationOwner(const ISpeedComponent*,int32,bool Called);
    void EndOrdinaryIntegration(float ActualAfter);
    void ReadOrdinaryConsumedState(FOrdinaryConsumedStateValues&) const;
    void RequireOrdinaryResponseContext() const;
    bool HasOrdinaryResponseObservation() const;
    void BeginOrdinaryResponseBatch(TConstArrayView<::SComponentTOI>,const Collision::FResolvedPairSet&,float Seconds,float Before,float ActualAfter,float SimTime);
    void ObserveOrdinaryResponseDecision(int32,const ::SComponentTOI&,const USSubBody*,bool Eligible,bool Allowed,const Collision::FResolvedPairSet&);
    void BeginOrdinaryResponseCall(int32 Call);
    void EndOrdinaryResponseCall(int32 Call);
    void EndOrdinaryResponseEntry(int32,bool Ignored,bool ExactRecordDecision,const Collision::FResolvedPairSet&);
    void EndOrdinaryResponseBatch();
    void ResetOrdinaryOwnerSweepReturns();
    void BeginOrdinaryOwnerSweep(const ISpeedComponent*,float LastSubDelta);
    static void ObserveOrdinarySweepReactions(const ISpeedComponent&,const FVector&,const FVector&);
    void EndOrdinaryOwnerSweep(const ISpeedComponent*,const ::SComponentTOI&);
    void CheckOrdinaryOwnerSweepReturns(TConstArrayView<TSharedPtr<const FComponentSweepEvidence,ESPMode::ThreadSafe>>) const;
    bool HasOpenOrdinaryWorldSelection() const noexcept;
    void BeginOrdinaryWorldSelection(const Collision::FResolvedPairSet&,float Horizon,float Before,float Delta,float Tolerance);
    void ObserveOrdinaryWorldCandidate(const ISpeedComponent*,const ::SComponentTOI&,const Collision::FResolvedPairSet&,
        const FOrdinaryWorldFilterValues&,bool HasClamped,float Clamped,TConstArrayView<::SComponentTOI> After,float Earliest);
    void ObserveOrdinaryWorldSortKey(const ::SComponentTOI&,const USSubBody*,uint32 ObjectId);
    void FinishOrdinaryWorldSelection(TConstArrayView<::SComponentTOI>,float Earliest,float Seconds,float Horizon,float Before,float Delta);
    void CheckOrdinaryWorldSelection(TConstArrayView<TSharedPtr<const FComponentSweepEvidence,ESPMode::ThreadSafe>>,
        float Seconds,bool HasSelection,TConstArrayView<::SComponentTOI>,float Horizon) const;
    TUniquePtr<FPostSyncRsNativeState> PostSyncRs;
    bool IsPostSyncRsPurpose() const;
    bool HasPostSyncRsIdentity() const;
    void CheckPostSyncRsOwners() const;
    void BeginPostSyncRsPrelude(float RawRemaining,float TimePassed,float LastSubDelta);
    void ObservePostSyncRsSupport(const ISpeedComponent*,float Before,float Actual);
    void BeginPostSyncRsOwner(const ISpeedComponent*,float LastSubDelta);
    void ValidatePostSyncRsReactions(const ISpeedComponent&,const FVector&,const FVector&);
    void CompletePostSyncRsRunner(const ISpeedComponent*,const ::SComponentTOI&);
    TSharedPtr<const FNativeSpherePostSyncRsCapture,ESPMode::ThreadSafe> ClosePostSyncRsCapture(const FTerminalRsCaptureStop&);
    TSharedPtr<FNativeSphereRsTrajectoryCapture,ESPMode::ThreadSafe> RsCapture;
    TSharedPtr<const FComponentSweepEvidence,ESPMode::ThreadSafe> RsRunnerEvidence;
    int32 RsSupportCount=0,RsSweepOwners=0;
    TArray<float> RsSupportBefore,RsSupportAfter;
    bool RsOwnerActive=false,RsReactionObserved=false;
    bool RsLastSubDeltaObserved=false;
    float RsLastSubDelta=0;
    const ISpeedComponent* RsOwner=nullptr;
    FKinematicState RsOwnerBefore;
    TArray<FKinematicState> RsBodiesBefore;
    FVector RsLinearReaction=FVector::ZeroVector,RsAngularReaction=FVector::ZeroVector;
    bool IsTerminalRsPurpose() const;
    void ObserveTerminalRsSupportInterval(const ISpeedComponent*,float Before,float After);
    void BeginTerminalRsSweepOwner(const ISpeedComponent*,float ActualLastSubDelta);
    void CompleteTerminalRsRunner(const ISpeedComponent*,const ::SComponentTOI&);
    static void ValidateTerminalRsReactions(const ISpeedComponent&,const FVector&,const FVector&);
    TSharedPtr<const FNativeSphereRsTrajectoryCapture,ESPMode::ThreadSafe> CloseTerminalRsCapture(const FTerminalRsCaptureStop&);
    float BeginTerminalConsumedPrefixAdvance();
    void BeginTerminalConsumedPrefixAdvanceOwner(const ISpeedComponent*,int32 Index);
    void EndTerminalConsumedPrefixAdvanceOwner(const ISpeedComponent*,int32 Index);
    void RecordTerminalConsumedPrefixAdvanceStates(const ISpeedComponent*,int32 Index,bool Returned);
    void EndTerminalConsumedPrefixAdvance();
    const USphereSubBody* BeginTerminalFirstRsResponse(float Seconds,float TimePassed,float SimTime);
    SHitResult BeginTerminalFirstRsResponseCall(const USSubBody*,int32 Call);
    void RecordTerminalFirstRsResponseStates(const USSubBody*,int32 Call,bool Returned);
    void EndTerminalFirstRsResponseCall(const USSubBody*,int32 Call);
    void EndTerminalFirstRsResponse();
    bool HasCurrentFirstRsResponseIdentity(bool SyncSite=false) const;
    void CheckFirstRsResponseOwners(int32 CompletedCalls,bool SyncSite=false,bool Synchronized=false) const;
    ISpeedComponent* BeginTerminalFirstRsResponseSynchronization();
    void RecordTerminalFirstRsResponseSynchronizationStates(const ISpeedComponent*,bool Returned);
    void EndTerminalFirstRsResponseSynchronization(const ISpeedComponent*);
    bool HasCurrentConsumedPrefixIdentity() const;
    void CheckConsumedPrefixOwners(int32 Completed) const;
    void BeginSweepIteration(int32 Iteration,float Remaining,float TimePassed);
    void InvalidateQueries();
    void RequireAdvanceEvidence(TConstArrayView<TSharedPtr<const FComponentSweepEvidence,ESPMode::ThreadSafe>> Bundles,float Seconds,
        bool HasActualSelection,TConstArrayView<::SComponentTOI> SelectedBatch,float SweepHorizon);
public:
    // Copies the real planar solver operands, then refuses changed General FLOOR pose before its first setter.
    static void ObserveUnclosedPlanarCommitValues(const ISpeedComponent&,const UBoxSubBody&,const SHitResult&,
        const FKinematicState& NativeBefore,const FKinematicState& LocalSolved);
    static bool TryProveNativeCurvedBoxPhysicalProjectionClear(const UBoxSubBody&,
        const SHitResult&,const IStaticCollisionWorld&,const Analytic::FWorldQuery&);
    // Reject-only observation of the first unpublished wheel-support pose.
    static bool TryRejectNativeCurvedBoxWheelSupportTranslation(const ISpeedComponent&,const FVector&);
#if !UE_BUILD_SHIPPING
    static bool MatchesCase205CoupledHitboxOverlap(const Analytic::FWorldQuery&);
    static bool TryProveCase585CoupledHitboxOverlapClear(
        const TSharedPtr<const FPreparedWorldSnapshot,ESPMode::ThreadSafe>&,
        const Analytic::FWorldQuery&,Analytic::Certified::FDistanceBudget&);
    static bool TryUseCase585FifthSoftWallDeltaV(const UBoxSubBody&,const SHitResult&);
    static bool TryKeepCase585CurvedComponent(const UBoxSubBody&,const SHitResult&,uint32 Count,uint32 OriginalCap);
#endif
    static void ObserveOrdinaryIntegrationPath(const ISpeedComponent&,const FNativeIntegrationPathValues&);
    static void ObserveOrdinaryIntegrationStage(const ISpeedComponent&,ENativeIntegrationStage,float Delta,int32 Index=INDEX_NONE,int32 Count=0,bool Projected=false);
    static void ObserveOrdinaryIntegrationChildSync(const ISpeedComponent&,const FKinematicState& ActualOrigin) noexcept;
    static void BeginOrdinaryIntegrationRestoration(const ISpeedComponent&) noexcept;
    static void EndOrdinaryIntegrationRestoration(const ISpeedComponent&,float Delta) noexcept;
    static void ObserveOrdinaryBodyInertiaValue(const USolidSubBody*,const FMatrix&,EOrdinaryBodyInertiaOrigin,const FQuat& Basis);
    ~FStepScope() noexcept;
    static bool ReadTerminalRsObservation(FTerminalRsStepObservation&);
    static bool ReadPostSyncRsObservation(FPostSyncRsStepObservation&);
    static bool ReadTerminalConsumedPrefixAdvanceObservation(FTerminalConsumedPrefixAdvanceObservation&);
    static bool ReadTerminalFirstRsResponseObservation(FTerminalFirstRsResponseObservation&);
    static bool ReadTerminalFirstRsResponseSyncObservation(FTerminalFirstRsResponseSyncObservation&);
    static void RequireTerminalFirstRsResponseHit(const USSubBody*,const SHitResult&,EFirstRsResponseCall,float Delta,float SimTime);
    FStepScope(const FStepScope&)=delete;FStepScope& operator=(const FStepScope&)=delete;
    FStepScope(FStepScope&&)=delete;FStepScope& operator=(FStepScope&&)=delete;
    static uint64 ReadOwnerStableId(const void* Owner) noexcept;
    static void RequireNativeContactPairTargets(const FSimulationWorld&,const USSubBody* A=nullptr,const USSubBody* B=nullptr);
};
}
}
