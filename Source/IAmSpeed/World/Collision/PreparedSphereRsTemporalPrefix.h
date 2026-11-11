#pragma once
#include "PreparedSphereInitialTubeState.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"
namespace Speed::Analytic::Prepared
{
enum class ERsTemporalPrefixStage:uint8 {Prerequisites,Candidates,Sweep,Coplanarity,Floor,Prefix,RcCoverage,ConsumedPoint,Complete};
struct FRsTemporalPrefixDiagnostic
{
    ERsTemporalPrefixStage Stage=ERsTemporalPrefixStage::Prerequisites;
    C::ETensorSphereTimeStatus TimeStatus=C::ETensorSphereTimeStatus::Unknown;
    double LowerTime=0,UpperTime=1;
    int32 Candidates=0,Excluded=0,PrefixAttempts=0,Retreats=0;
    int32 BudgetBefore=0,BudgetAfterSweep=-1,BudgetAfterPrefix=-1,BudgetAfterRc=-1,BudgetAtExit=0;
    float InitialFloor=0;
    TArray<float> TriedSeconds;
    bool InitialSeparationProved=false,PrefixClear=false,RcCovered=false;
};
class FReferenceSphereRsTemporalPrefix;
using FReferenceSphereRsTemporalPrefixPtr=TSharedPtr<const FReferenceSphereRsTemporalPrefix,ESPMode::ThreadSafe>;
// A distinct mathematical temporal/rounded-prefix proof. It contains NO
// point/normal/scalar contact, selected owner, native advance or response grant.
class FReferenceSphereRsTemporalPrefix final
{
    FReferenceSphereInitialTubeStatePtr Initial,ConsumedPoint;
    C::FTensorSphereTimeBounds Temporal;
    C::FReferenceCandidateSetPtr RsCandidates,RcPartition;
    float Horizon=0,ConsumedSeconds=0;
    FReferenceSphereRsTemporalPrefix()=default;
public:
    IAMSPEED_API static bool TryEvaluate(FReferenceSphereInitialTubeStatePtr Initial,float ActualHorizon,
        const C::FReferenceSphereOptions& Options,C::FDistanceBudget& Budget,
        FReferenceSphereRsTemporalPrefixPtr& Out,FRsTemporalPrefixDiagnostic& Diagnostic,FString& Reason);
    const auto& GetInitial() const {return Initial;}
    const auto& GetConsumedPoint() const {return ConsumedPoint;}
    const auto& GetTemporal() const {return Temporal;}
    const auto& GetRsCandidates() const {return RsCandidates;}
    const auto& GetRcPartition() const {return RcPartition;}
    float GetHorizon() const {return Horizon;}
    float GetConsumedSeconds() const {return ConsumedSeconds;}
    bool HasWholeRoundedPrefixProof() const {return bool(ConsumedPoint)&&bool(RcPartition);}
};
}
