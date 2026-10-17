#include "SWheelSubBody.h"
#include "RayWheelSubBody.h"
#include "CylinderSubBody.h"
#include "CylinderSweep.h"
#include "IAmSpeed/Components/ISpeedWheeledComponent.h"
#include <stdexcept>
#include <limits>
namespace
{
bool CylinderSuspensionHit(USSubBody& Wheel,TConstArrayView<TWeakObjectPtr<UCylinderSubBody>> Cylinders,
    const FVector& Start,const FVector& End,double Radius,const float Delta,SHitResult& Out)
{
    bool Found=false;double Best=1.;const double Length=(End-Start).Size();
    for(const auto& Reference:Cylinders)
    {
        auto* C=Reference.Get();if(!C||C->GetOwner()==Wheel.GetOwner()||Wheel.ComponentHasBeenIgnored(*C))continue;
        const auto Geometry=C->Geometry();const auto K=C->GetKinematicState();
        const double Bound=FMath::Sqrt(Geometry.Radius*Geometry.Radius+Geometry.HalfHeight*Geometry.HalfHeight)+Radius+Length;
        const double Roundoff=64.*std::numeric_limits<double>::epsilon()*(1.+Start.Size()+K.Location.Size()+Bound);
        if(FVector::Distance(Start,K.Location)>Bound+Roundoff)continue;
        const FTransform Pose(K.Rotation,K.Location);
        const auto Hit=Speed::Geometry::SweepCylinderShapeTrajectory(Geometry,true,Radius,FVector3d::ZeroVector,
            [&](double){return Pose;},[&](double T){return FTransform(FQuat::Identity,FMath::Lerp(Start,End,T));},Length,0.,0.);
        if(Hit.Result==Speed::Geometry::ECylinderSweepResult::Miss)continue;
        if(Hit.Result!=Speed::Geometry::ECylinderSweepResult::Hit)throw std::runtime_error("Cylinder suspension query unresolved");
        if(Found&&Hit.Time>=Best)continue;Best=Hit.Time;Found=true;
        Out=SHitResult(true,Hit.Contact.CylinderPoint,Hit.Contact.Normal,float(Hit.Time*Delta),C);Out.SubBody=C;
        Out.FrameTag=Wheel.GetParentComponent()->NumFrame();Out.ContactPointThis=Hit.Contact.ShapePoint;Out.ContactPointOther=Hit.Contact.CylinderPoint;
        Out.GeometricErrorBoundCm=2.e-4f;Out.bSurfaceNormalMayVary=true;
    }
    return Found;
}
}
bool USWheelSubBody::TryGetPrescribedSupportVelocity(const FVector& Point,FVector& Velocity) const
{
    const auto* Other=Cast<USolidSubBody>(CurrentHit.SubBody.Get());
    if(!Other||!Other->GetParentComponent()||!Other->GetParentComponent()->IsPrescribedMotion())return false;
    Velocity=Other->GetVelocityAtPoint(Point);return true;
}
bool USWheelSubBody::SweepSuspensionOnCylinders(SHitResult& Out,const float& Delta)
{
    if(!ParentComponent||ExternalCylinderSubBodies.IsEmpty()||Delta<=0.f)return false;
    const FTransform Chassis(ParentComponent->GetPhysRotation(),ParentComponent->GetPhysLocation());
    const FVector Up=Chassis.GetUnitAxis(EAxis::Z),Start=Chassis.TransformPosition(GetLocalOffset())+SuspensionMaxRaise()*Up;
    const FVector End=WorldPos()+(PredictNextDisplacement(Delta)-SpringDisplacement()-CollisionMargin())*Up;
    if(!CylinderSuspensionHit(*this,ExternalCylinderSubBodies,Start,End,Radius(),Delta,Out))return false;
    Out.ImpactNormal=Speed::QuantizeUnitNormal(Out.ImpactNormal);Out.Location=Out.ImpactPoint+Radius()*Out.ImpactNormal;
    SetOnGround(true);if(WheelComponent)WheelComponent->NotifyWheelOnGroundStateChanged();return true;
}
bool URayWheelSubBody::SweepSuspensionOnCylinders(SHitResult& Out,const float& Delta)
{
    FVector Start,End,Up;if(ExternalCylinderSubBodies.IsEmpty()||!BuildSuspensionRay(Delta,Start,End,Up))return false;
    if(!CylinderSuspensionHit(*this,ExternalCylinderSubBodies,Start,End,0.,Delta,Out))return false;
    AcceptSuspensionRayHit(Out,Up);SetOnGround(true);if(WheelComponent)WheelComponent->UpdateWheelOnGroundStates();return true;
}
