#pragma once
#include "PreparedReferencePlaneEndpointSeparation.h"
#include "IAmSpeed/Base/Kinematic.h"
namespace Speed::Analytic::Prepared
{
enum class EPhysicalSphereInitialKind:uint8 {Separated,Tangency,Overlap};
struct FStaticSphereInitialValues
{
    FKinematicState State;
    float PhysicalRadius=0,StaticContactOffset=0,ActualMargin=0,QueryRadius=0;
    double SupportRadius=0;
};
class FReferenceSphereInitialEvaluation;
using FReferenceSphereInitialEvaluationPtr=TSharedPtr<const FReferenceSphereInitialEvaluation,ESPMode::ThreadSafe>;
// Values-only diagnostic. The Rs theorem classifies both physical Rp and
// static support Rs, then the ACTUAL native quantizer runs on a LOCAL copy.
// The second theorem proves Rs separation at that exact preview endpoint.
// Values cannot authenticate the supplied margin; the closed native caller
// must join its actual member reread. Input.Rotation is the retained LOCAL
// preview reference, not a witness of previous native rotation/history.
// No native request, applied state, Rc-clear prefix, hit/time, response arm,
// support normal, phase capability or permission to mutate is produced.
class FReferenceSphereInitialEvaluation final
{
    FStaticSphereInitialValues Input;
    FKinematicState Preview;
    EPhysicalSphereInitialKind PhysicalKind=EPhysicalSphereInitialKind::Separated;
    FReferencePlaneInitialContactPtr SupportInitial;
    FReferencePlaneEndpointSeparationPtr Endpoint;
    FReferenceSphereInitialEvaluation()=default;
public:
    IAMSPEED_API static bool TryEvaluate(FPreparedGeometrySelectionPtr Geometry,FStaticSphereInitialValues Values,
        C::FDistanceBudget& Budget,FReferenceSphereInitialEvaluationPtr& Out,FString& Reason);
    const auto& GetInput() const {return Input;}
    const auto& GetPreview() const {return Preview;}
    EPhysicalSphereInitialKind GetPhysicalKind() const {return PhysicalKind;}
    const auto& GetSupportInitial() const {return SupportInitial;}
    const auto& GetEndpoint() const {return Endpoint;}
};
}
