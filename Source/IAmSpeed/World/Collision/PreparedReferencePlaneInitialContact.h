#pragma once
#include "PreparedOriginPlaneContact.h"
namespace Speed::Analytic::Prepared
{
enum class EReferencePlaneInitialKind:uint8 {Tangency,Overlap};
class FReferencePlaneInitialContact;
using FReferencePlaneInitialContactPtr=TSharedPtr<const FReferencePlaneInitialContact,ESPMode::ThreadSafe>;
// Mathematical initial-distance/MTD result, distinct from FirstContact and an
// Rc-clear prefix. It owns the actual selected model AND complete partition.
// Radius is a supplied distance radius, not an assertion of Rp, Rs or margin.
// No filter/material authority, native request, hit, pose mutation or advance
// is granted. A quantized/native correction target needs a new covered segment.
class FReferencePlaneInitialContact final
{
    FPreparedGeometrySelectionPtr Geometry;
    C::FReferenceCandidateSetPtr Selected;
    C::FReferenceCandidateSet::FClosedPlaneOriginWitness Witness;
    FVector3d Center=FVector3d::ZeroVector,Target=FVector3d::ZeroVector,Normal=FVector3d::ZeroVector;
    double Radius=0,CoverageRadius=0,Depth=0;
    EReferencePlaneInitialKind Kind=EReferencePlaneInitialKind::Tangency;
    FReferencePlaneInitialContact()=default;
public:
    IAMSPEED_API static bool TryBound(FPreparedGeometrySelectionPtr geometry,const FVector3d& center,
        double radius,double coverageRadius,C::FDistanceBudget& budget,FReferencePlaneInitialContactPtr& out,FString& reason);
    const FPreparedGeometrySelectionPtr& GetGeometry() const {return Geometry;}
    const C::FReferenceCandidateSetPtr& GetSelected() const {return Selected;}
    const C::FReferenceCandidateSet::FClosedPlaneOriginWitness& GetWitness() const {return Witness;}
    FVector3d GetCenter() const {return Center;}
    FVector3d GetTangencyTarget() const {return Target;}
    FVector3d GetGeometricNormal() const {return Normal;}
    double GetRadius() const {return Radius;}
    double GetCoverageRadius() const {return CoverageRadius;}
    double GetMinimumTranslationLength() const {return Depth;}
    EReferencePlaneInitialKind GetKind() const {return Kind;}
};
}
