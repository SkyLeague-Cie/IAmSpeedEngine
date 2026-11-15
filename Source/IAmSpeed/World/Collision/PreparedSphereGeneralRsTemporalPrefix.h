#pragma once
#include "PreparedSphereStrictSeparation.h"
#include "PreparedSphereRsTemporalPrefix.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"
namespace Speed::Analytic::Prepared
{
class FReferenceSphereGeneralRsTemporalPrefix;
using FReferenceSphereGeneralRsTemporalPrefixPtr=TSharedPtr<const FReferenceSphereGeneralRsTemporalPrefix,ESPMode::ThreadSafe>;
// A distinct mathematical temporal/rounded-prefix proof. It contains NO
// point/normal/scalar contact, selected owner, native advance or response grant.
class FReferenceSphereGeneralRsTemporalPrefix final
{
    FReferenceSphereStrictSeparationPtr Initial,ConsumedPoint;
    C::FTensorSphereTimeBounds Temporal;
    C::FReferenceCandidateSetPtr RsCandidates,RcPartition;
    float Horizon=0,ConsumedSeconds=0;
    FReferenceSphereGeneralRsTemporalPrefix()=default;
public:
    IAMSPEED_API static bool TryEvaluate(FReferenceSphereStrictSeparationPtr Initial,float ActualHorizon,
        const C::FReferenceSphereOptions& Options,C::FDistanceBudget& Budget,
        FReferenceSphereGeneralRsTemporalPrefixPtr& Out,FRsTemporalPrefixDiagnostic& Diagnostic,FString& Reason);
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
