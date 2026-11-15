#pragma once
#include "CoreMinimal.h"
namespace Speed::Analytic::Prepared
{
enum class EReferenceSphereNormalEvolution:uint8 {Unknown,FixedNormal,RequiresReacquisition};
// Producer-defined algorithm contract, NOT an authority/normal uniqueness proof.
// V1: the finite-provider distance oracle bounds radial first-contact normals.
// A plane's finite edges need not have the face's constant normal. Thus the
// producer may conservatively require reacquisition for EVERY provider before
// any query/margin/error is evaluated, and forbid tangent-plane continuation.
// The coupled nonpenetrating sphere response arm is Center-Rp*NormalConsumed.
// ScalarSphereContactBounds and SphereStaticResponsePoint must qualify the
// actual bits together. InitialInsideTube never supplies penetration or MTD.
// This contract applies to sphere contact only, not box/wheel/support readiness.
struct FReferenceSphereResponseContract
{
    uint32 Version=0;int32 SourceFaceIndex=INDEX_NONE;
    EReferenceSphereNormalEvolution NormalEvolution=EReferenceSphereNormalEvolution::Unknown;
    bool RadialFirstContactNormal=false,PhysicalRadiusRadialArm=false,ForbidTangentPlaneContinuation=false;
    static FReferenceSphereResponseContract RadialReacquisitionV1(int32 sourceFace)
    {return {1,sourceFace,EReferenceSphereNormalEvolution::RequiresReacquisition,true,true,true};}
    bool StructurallyValid(int32 face) const
    {
        if(SourceFaceIndex!=face||Version>1||uint8(NormalEvolution)>uint8(EReferenceSphereNormalEvolution::RequiresReacquisition))return false;
        return Version!=0||(NormalEvolution==EReferenceSphereNormalEvolution::Unknown&&!RadialFirstContactNormal&&!PhysicalRadiusRadialArm&&!ForbidTangentPlaneContinuation);
    }
    bool IsRadialReacquisitionV1() const
    {return Version==1&&NormalEvolution==EReferenceSphereNormalEvolution::RequiresReacquisition&&RadialFirstContactNormal&&PhysicalRadiusRadialArm&&ForbidTangentPlaneContinuation;}
    bool operator==(const FReferenceSphereResponseContract& b) const
    {return Version==b.Version&&SourceFaceIndex==b.SourceFaceIndex&&NormalEvolution==b.NormalEvolution&&RadialFirstContactNormal==b.RadialFirstContactNormal&&PhysicalRadiusRadialArm==b.PhysicalRadiusRadialArm&&ForbidTangentPlaneContinuation==b.ForbidTangentPlaneContinuation;}
};
}
