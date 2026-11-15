#pragma once
#include "PreparedSphereInitialTubeState.h"
#include "IAmSpeed/World/Analytic/ReferenceSphereConsumer.h"
namespace Speed::Analytic::Prepared
{
class FReferenceSphereFirstArrivalRs;
using FReferenceSphereFirstArrivalRsPtr=TSharedPtr<const FReferenceSphereFirstArrivalRs,ESPMode::ThreadSafe>;
// Values-only axial planar route from a separated point already inside Rc.
// The complete continuous/reference and rounded-prefix proofs use Rs, while
// a distinct complete rounded hull retains the real Rc. No native radius is
// replaced. No World hit, unique owner, force, phase or advance is admitted.
class FReferenceSphereFirstArrivalRs final
{
    FReferenceSphereInitialTubeStatePtr Initial,ConsumedPoint;
    C::FReferenceSphereEvaluation Reference;
    C::FReferenceCandidateSetPtr RcPartition;
    float Horizon=0;
    FReferenceSphereFirstArrivalRs()=default;
public:
    IAMSPEED_API static bool TryEvaluate(FReferenceSphereInitialTubeStatePtr Initial,float ActualHorizon,
        const C::FReferenceSphereOptions& Options,C::FDistanceBudget& Budget,
        FReferenceSphereFirstArrivalRsPtr& Out,FString& Reason,C::FReferenceSphereEvaluation* PartialReference=nullptr);
    const auto& GetInitial() const {return Initial;}
    const auto& GetReference() const {return Reference;}
    const auto& GetRcPartition() const {return RcPartition;}
    const auto& GetConsumedPoint() const {return ConsumedPoint;}
    float GetHorizon() const {return Horizon;}
};
}
