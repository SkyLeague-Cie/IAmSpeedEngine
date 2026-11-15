#pragma once
#include "PreparedSphereRsTrajectoryCapture.h"
#include "PreparedSphereGeneralRsTemporalPrefix.h"
#include "PreparedSphereGeneralFirstArrivalRs.h"
#include "IAmSpeed/World/Analytic/IndexedIntervalUnion.h"

namespace Speed::PreparedExecution
{
struct FPostSyncRsMathResult
{
    // Full-H partitions are distinct from a prefix's own consumed-c partition.
    Analytic::Certified::FReferenceCandidateSetPtr FullHorizonRsPartition,FullHorizonRcPartition;
    // Module-owned immutable dynamic source; never a mutation/advance capability.
    TSharedPtr<const void,ESPMode::ThreadSafe> FullHorizonDynamicSource;
    int32 FullHorizonDynamicBodies=0;
    bool FullHorizonRsAttempted=false,FullHorizonRcAttempted=false,DynamicDomainAttempted=false;
    bool StrictInitialAttempted=false,TemporalAttempted=false,FirstArrivalAttempted=false;
    FString FullHorizonRsReason,FullHorizonRcReason,DynamicDomainReason,InitialReason,TemporalReason,FirstArrivalReason;
    Analytic::Prepared::FReferenceSphereStrictSeparationPtr Initial;
    Analytic::Prepared::FReferenceSphereGeneralRsTemporalPrefixPtr TemporalPrefix;
    Analytic::Prepared::FReferenceSphereGeneralFirstArrivalRsPtr FirstArrival;
    Analytic::Prepared::FRsTemporalPrefixDiagnostic TemporalDiagnostic;
    Analytic::Certified::FReferenceSphereEvaluation FirstArrivalPartial;
    FString Reason;
    int32 BudgetBefore=0,BudgetAfter=0,Attempts=0,ClosureReserve=0;
    uint64 MaxObservedSourceUnits=0;
};
struct FPostSyncRsStepObservation
{
    FStepIdentity Step;
    uint64 ResponseSerial=0;
    float RawRemaining=0,TimePassed=0,LastSubDelta=0,Horizon=0;
    TArray<float> SupportBefore,SupportAfter;
    int32 SupportCount=0,SweepOwners=0,DiagnosticIteration=0,DiagnosticComponentSweeps=0;
    bool OwnerActive=false,ReactionObserved=false,MemberPresent=false,ComponentPresent=false;
    bool SweepPhase=false,CaptureClosed=false,RunnerInventorySealed=false;
    FVector LinearReaction=FVector::ZeroVector,AngularReaction=FVector::ZeroVector;
};
class FNativeSpherePostSyncRsCapture final
{
    friend class FSphereMemberSweepScope;friend class FComponentSweepScope;friend class FStepScope;
    enum class EStage:uint8 {Captured,MemberClosed,ComponentClosed,StepClosed};
    EStage Stage=EStage::Captured;
    FNativeSphereRequestPtr Request;
    FNativeRsTraceValues Trace;
    uint64 ComponentSerial=0,ResponseSerial=0;
    TSharedPtr<const FFirstRsResponseData,ESPMode::ThreadSafe> OriginatingResponseData;
    FPostSyncRsMathResult Math;
    bool MathAttempted=false;
    FNativeSpherePostSyncRsCapture()=default;
public:
    bool IsClosed() const {return Stage==EStage::StepClosed;}
    const auto& GetRequest() const {return Request;}
    const auto& GetTrace() const {return Trace;}
    uint64 GetComponentSerial() const {return ComponentSerial;}
    uint64 GetResponseSerial() const {return ResponseSerial;}
    const auto& GetResponseData() const {return OriginatingResponseData;}
    bool WasMathAttempted() const {return MathAttempted;}
    const auto& GetMath() const {return Math;}
};
using FNativeSpherePostSyncRsCapturePtr=TSharedPtr<const FNativeSpherePostSyncRsCapture,ESPMode::ThreadSafe>;

// This state is reachable only through the private current FStepScope. Its
// counters start once; no slot1 observation/request/ticket is reused or reset.
class FPostSyncRsNativeState final
{
    friend class FStepScope;friend class FSphereMemberSweepScope;friend class FComponentSweepScope;
    friend IAMSPEED_API bool ReadCurrentStep(FStepIdentity&) noexcept;
    friend IAMSPEED_API bool NextSphereQuery(FStepIdentity&) noexcept;
    bool Started=false,OwnerActive=false,ReactionObserved=false;
    uint64 ResponseSerial=0;
    TSharedPtr<const FFirstRsResponseData,ESPMode::ThreadSafe> Data;
    float RawRemaining=0,TimePassed=0,LastSubDelta=0,Horizon=0;
    int32 SupportCount=0,SweepOwners=0;
    TArray<float> SupportBefore,SupportAfter;
    const ISpeedComponent* Owner=nullptr;
    FKinematicState OwnerBefore;
    TArray<FKinematicState> BodiesBefore;
    FVector LinearReaction=FVector::ZeroVector,AngularReaction=FVector::ZeroVector;
    TSharedPtr<const FComponentSweepEvidence,ESPMode::ThreadSafe> RunnerEvidence;
    TSharedPtr<FNativeSpherePostSyncRsCapture,ESPMode::ThreadSafe> Capture;
    FPostSyncRsNativeState()=default;
};
}
