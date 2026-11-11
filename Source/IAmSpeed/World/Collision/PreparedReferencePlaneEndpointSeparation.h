#pragma once
#include "PreparedReferencePlaneInitialContact.h"
namespace Speed::Analytic::Prepared
{
class FReferencePlaneEndpointSeparation;
using FReferencePlaneEndpointSeparationPtr=TSharedPtr<const FReferencePlaneEndpointSeparation,ESPMode::ThreadSafe>;
// Mathematical separation at the EXACT SUPPLIED endpoint. Not a tangency,
// minimum translation, native quantizer/state witness, clear Rc prefix,
// material/filter owner, support, normal reacquisition or permission to apply.
// In particular lateral drift may move its projection outside the finite face.
class FReferencePlaneEndpointSeparation final
{
    FReferencePlaneInitialContactPtr Initial;
    C::FReferenceCandidateSetPtr Partition;
    C::FReferenceCandidateSet::FClosedPlaneOriginWitness Witness;
    FVector3d Endpoint=FVector3d::ZeroVector;
    FReferencePlaneEndpointSeparation()=default;
public:
    IAMSPEED_API static bool TryBound(FReferencePlaneInitialContactPtr Initial,const FVector3d& Endpoint,
        C::FDistanceBudget& Budget,FReferencePlaneEndpointSeparationPtr& Out,FString& Reason);
    const auto& GetInitial() const {return Initial;}
    const auto& GetCompletePartition() const {return Partition;}
    const auto& GetWitness() const {return Witness;}
    FVector3d GetEndpoint() const {return Endpoint;}
};
}
