#pragma once
#include "PreparedSphereRequest.h"
#include "PreparedNativeTraceValues.h"
#include "PreparedSphereInitialTubeState.h"
#include "PreparedSphereFirstArrivalRs.h"
#include "PreparedSphereRsTemporalPrefix.h"
#include "PreparedRsPrefixAdvanceSource.h"
#include "PreparedFirstArrivalRsResponseQuality.h"

namespace Speed::PreparedExecution
{
class FNativeSphereRsTrajectoryCapture;
using FNativeSphereRsTrajectoryCapturePtr=TSharedPtr<const FNativeSphereRsTrajectoryCapture,ESPMode::ThreadSafe>;
struct FCapturedRsMathResult
{
    Analytic::Prepared::FReferenceSphereInitialTubeStatePtr Initial;
    Analytic::Prepared::FReferenceSphereFirstArrivalRsPtr FirstArrival;
    Analytic::Prepared::FReferenceSphereRsTemporalPrefixPtr TemporalPrefix;
    Analytic::Prepared::FRsTemporalPrefixDiagnostic TemporalDiagnostic;
    FRsPrefixAdvanceSourcePtr AdvanceSource;
    FFirstArrivalRsResponseQualityPtr RsQuality;
    Analytic::Certified::FReferenceSphereEvaluation FirstArrivalPartial;
    FString Reason;
    int32 BudgetBefore=0,BudgetAfter=0,Attempts=0,ClosureReserve=0,RetainedTemporalAttempt=0;
    uint64 MaxObservedSourceUnits=0;
};
// Plain observations. Copies never carry a live query/advance capability.
struct FTerminalRsStepObservation
{
    FStepIdentity Step;
    int32 SupportCount=0,SweepOwners=0,DiagnosticIteration=0,DiagnosticComponentSweeps=0;
    TArray<float> SupportBefore,SupportAfter;
    bool OwnerActive=false,ReactionObserved=false,MemberPresent=false,ComponentPresent=false;
    bool SweepPhase=false,CaptureClosed=false,RunnerInventorySealed=false,LastSubDeltaObserved=false;
    int32 RunnerBodies=0;
    float LastSubDelta=0;
    FVector LinearReaction=FVector::ZeroVector,AngularReaction=FVector::ZeroVector;
    double ResetMilliseconds=0,SweepMilliseconds=0;
};

// A private native control transfer. No caller can construct a stop from a
// request or a serialized diagnostic. It names the one actual live Step.
class FTerminalRsCaptureStop final
{
    friend class FSphereMemberSweepScope;
    friend class FComponentSweepScope;
    friend class FStepScope;
    enum class ESlot:uint8 {Initial,PostSync};
    ESlot Slot=ESlot::Initial;
    const FStepScope* Lease=nullptr;
    uint64 QuerySerial=0,MemberSerial=0;
    FTerminalRsCaptureStop(const FStepScope* step,uint64 query,uint64 member)
        :Lease(step),QuerySerial(query),MemberSerial(member){}
    FTerminalRsCaptureStop(const FStepScope* step,uint64 query,uint64 member,ESlot slot)
        :Slot(slot),Lease(step),QuerySerial(query),MemberSerial(member){}
public:
    FTerminalRsCaptureStop(const FTerminalRsCaptureStop&)=default;
};

// This immutable observation can become closed only after the real native
// member, component and Step have closed in order. It grants no hit, support,
// complete sweep inventory, integration or response capability.
class FNativeSphereRsTrajectoryCapture final
{
    friend class FSphereMemberSweepScope;
    friend class FComponentSweepScope;
    friend class FStepScope;
    enum class EStage:uint8 {Captured,MemberClosed,ComponentClosed,StepClosed};
    EStage Stage=EStage::Captured;
    FNativeSphereRequestPtr Request;
    FNativeRsTraceValues Trace;
    uint64 ComponentSerial=0;
    Analytic::Prepared::FReferenceSphereInitialTubeStatePtr Initial;
    Analytic::Prepared::FReferenceSphereFirstArrivalRsPtr FirstArrival;
    Analytic::Prepared::FReferenceSphereRsTemporalPrefixPtr TemporalPrefix;
    Analytic::Prepared::FRsTemporalPrefixDiagnostic TemporalDiagnostic;
    FRsPrefixAdvanceSourcePtr AdvanceSource;
    FFirstArrivalRsResponseQualityPtr RsQuality;
    Analytic::Certified::FReferenceSphereEvaluation FirstArrivalPartial;
    FString MathReason;
    int32 MathBudgetBefore=0,MathBudgetAfter=0,MathAttempts=0,MathClosureReserve=0,RetainedTemporalAttempt=0;
    uint64 MaxObservedSourceUnits=0;
    bool MathAttempted=false,InitialKnown=false,FirstArrivalKnown=false,PositiveConsumedPrefix=false;
    FNativeSphereRsTrajectoryCapture()=default;
public:
    bool IsClosed() const {return Stage==EStage::StepClosed;}
    const auto& GetRequest() const {return Request;}
    const auto& GetTrace() const {return Trace;}
    uint64 GetComponentSerial() const {return ComponentSerial;}
    const auto& GetInitial() const {return Initial;}
    const auto& GetFirstArrival() const {return FirstArrival;}
    const auto& GetTemporalPrefix() const {return TemporalPrefix;}
    const auto& GetAdvanceSource() const {return AdvanceSource;}
    const auto& GetRsQuality() const {return RsQuality;}
    const auto& GetFirstArrivalPartial() const {return FirstArrivalPartial;}
    bool HasBoundedFirstArrivalRsQuality() const
    {
        if(!Initial||!FirstArrival||!TemporalPrefix||!RsQuality||RsQuality->GetArrival()!=FirstArrival||
            FirstArrival->GetInitial()!=Initial||TemporalPrefix->GetInitial()!=Initial||
            RsQuality->GetStatus()!=ESphereResponseQualityStatus::BoundedWithinInputMargin||
            FirstArrival->GetReference().Status!=Analytic::Certified::EReferenceSphereStatus::ReferenceContactBounded||
            !FirstArrival->GetReference().ConsumedPrefixClear||FirstArrival->GetReference().Contact.Leaves.IsEmpty()||
            !TemporalPrefix->HasWholeRoundedPrefixProof())return false;
        const float a=FirstArrival->GetReference().ConsumedSeconds,b=TemporalPrefix->GetConsumedSeconds();
        uint32 x=0,y=0,h=0,k=0;const float ah=FirstArrival->GetHorizon(),bh=TemporalPrefix->GetHorizon();
        FMemory::Memcpy(&x,&a,4);FMemory::Memcpy(&y,&b,4);FMemory::Memcpy(&h,&ah,4);FMemory::Memcpy(&k,&bh,4);
        if(x!=y||h!=k||x<0x00800000u||x>=0x7f800000u)return false;
        const auto& actual=FirstArrival->GetConsumedPoint()->GetInput().State.Location;
        const auto& other=TemporalPrefix->GetConsumedPoint()->GetInput().State.Location;
        const auto& scalar=RsQuality->GetReboundedScalar();
        for(int32 d=0;d<3;++d)
            if(AnalyticCertified::Bits(actual[d])!=AnalyticCertified::Bits(other[d])||
                AnalyticCertified::Bits(actual[d])!=AnalyticCertified::Bits(scalar.Location[d]))return false;
        return RsQuality->MatchesConsumedValues(a,scalar.Location,scalar.ImpactPoint,scalar.NormalStored,
            scalar.NormalConsumed,RsQuality->GetResponsePoint());
    }

    // Last attempt can be Unknown while a preceding valid +0 is retained.
    const auto& GetLastTemporalDiagnostic() const {return TemporalDiagnostic;}
    int32 GetRetainedTemporalAttempt() const {return RetainedTemporalAttempt;}
    bool HasTemporalPrefix() const {return bool(TemporalPrefix);}
    int32 GetMathClosureReserve() const {return MathClosureReserve;}
    uint64 GetMaxObservedSourceUnits() const {return MaxObservedSourceUnits;}
    const FString& GetMathReason() const {return MathReason;}
    bool WasMathAttempted() const {return MathAttempted;}
    bool HasInitialClassification() const {return InitialKnown;}
    bool HasFirstArrival() const {return FirstArrivalKnown;}
    bool HasPositiveConsumedPrefix() const {return PositiveConsumedPrefix;}
    int32 GetMathBudgetBefore() const {return MathBudgetBefore;}
    int32 GetMathBudgetAfter() const {return MathBudgetAfter;}
    int32 GetMathAttempts() const {return MathAttempts;}
};
}
