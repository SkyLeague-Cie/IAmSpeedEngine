#include "CylinderSubBody.h"
#include "CylinderSweep.h"
#include "BoxSubBody.h"
#include "SphereSubBody.h"
#include "SWheelSubBody.h"
#include "Math/QuatRotationTranslationMatrix.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include <limits>
#include <stdexcept>
namespace
{
struct FCylinderRigidTrajectory
{
    SKinematic Owner;
    FVector Offset;FQuat LocalRotation;
    double Delta=0.,Span=0.,Motion=0.,Acceleration=0.,Error=0.;
    FCylinderRigidTrajectory(const USolidSubBody& Body,double Dt,double ShapeRadius)
    {
        const auto& Parent=*Body.GetParentComponent();Owner=Parent.GetKinematicState();Delta=Dt;
        const auto& K=Body.GetKinematicState();Offset=Owner.Rotation.UnrotateVector(K.Location-Owner.Location);LocalRotation=Owner.Rotation.Inverse()*K.Rotation;
        Span=Offset.Size()+ShapeRadius;const double W=Owner.AngularVelocity.Size()+Owner.AngularAcceleration.Size()*Dt;
        Motion=Dt*(Owner.Velocity.Size()+Owner.Acceleration.Size()*Dt+W*Span);
        Acceleration=Dt*Dt*(Owner.Acceleration.Size()+(Owner.AngularAcceleration.Size()+W*W)*Span);
        Error=8.*double(std::numeric_limits<float>::epsilon())*(1.+Motion)+64.*std::numeric_limits<double>::epsilon()*(1.+Owner.Location.Size()+Span);
    }
    FTransform At(double T) const
    {const auto K=Owner.Integrate(float(T*Delta));return FTransform(K.Rotation*LocalRotation,K.Location+K.Rotation.RotateVector(Offset));}
};
}
UCylinderSubBody::UCylinderSubBody(const FObjectInitializer& I):Super(I)
{
    SubBodyType=ESubBodyType::Cylinder;bApplyRestForce=false;bEnableConstraintOnSubBodyHit=false;EnableFakePhysics=false;
    SetCollisionEnabled(ECollisionEnabled::NoCollision);SetCollisionObjectType(ECC_WorldDynamic);SetCollisionResponseToAllChannels(ECR_Block);
}
bool UCylinderSubBody::Configure(double R,double H)
{
    if(ParentComponent||!Speed::Geometry::FCylinderGeometry{R,H}.IsValid()||R>20000.||H>20000.)return false;
    CylinderRadius=R;CylinderHalfHeight=H;UpdateBounds();return true;
}
void UCylinderSubBody::Initialize(ISpeedComponent* Owner)
{
    Super::Initialize(Owner);
    if(!Owner||!Owner->IsPrescribedMotion()||Owner->GetPhysMass()!=0.f||!Geometry().IsValid()||!GetRelativeScale3D().Equals(FVector::OneVector,1.e-12))throw std::runtime_error("Cylinder admission requires a configured prescribed solid with rigid unit scale");
    bApplyRestForce=false;bEnableConstraintOnSubBodyHit=false;EnableFakePhysics=false;
}
FMatrix UCylinderSubBody::InitInvInertiaTensor() const
{
    if(Mass<=0.f)return FMatrix(ForceInitToZero);
    const double Side=double(Mass)*(3.*CylinderRadius*CylinderRadius+4.*CylinderHalfHeight*CylinderHalfHeight)/12.;
    const double Axis=double(Mass)*CylinderRadius*CylinderRadius*.5;
    FMatrix M(ForceInitToZero);M.M[0][0]=M.M[1][1]=1./Side;M.M[2][2]=1./Axis;M.M[3][3]=1.;return M;
}
FMatrix UCylinderSubBody::ComputeWorldInvInertiaTensor() const
{
    if(ParentComponent&&ParentComponent->IsPrescribedMotion())return FMatrix(ForceInitToZero);
    const FMatrix R=FQuatRotationMatrix(GetKinematicState().Rotation);return R.GetTransposed()*InvInertiaLocal*R;
}
FBoxSphereBounds UCylinderSubBody::CalcBounds(const FTransform& Pose) const
{
    const FVector Z=Pose.GetUnitAxis(EAxis::Z);FVector E;
    for(int32 I=0;I<3;++I)E[I]=CylinderRadius*FMath::Sqrt(FMath::Max(0.,1.-Z[I]*Z[I]))+CylinderHalfHeight*FMath::Abs(Z[I]);
    return FBoxSphereBounds(Pose.GetLocation(),E,FMath::Sqrt(CylinderRadius*CylinderRadius+CylinderHalfHeight*CylinderHalfHeight));
}
void UCylinderSubBody::AcceptHit()
{
    // The cylinder owns its new pair CCD. Do not suppress a genuine later
    // reacquisition after another impact in the same canonical frame.
    CurrentHit=FutureHit;
}
bool UCylinderSubBody::SweepTOI(const float& Remaining,float& OutTOI)
{
    FutureHit={};OutTOI=Remaining;if(!ParentComponent||Remaining<=0.f)return false;
    const auto C=Geometry();const double R=FMath::Sqrt(C.Radius*C.Radius+C.HalfHeight*C.HalfHeight);
    const FCylinderRigidTrajectory CT(*this,Remaining,R);
    auto Visit=[&](USolidSubBody* Other)
    {
        if(!Other||Other->GetParentComponent()==ParentComponent||!Other->GetParentComponent()||ComponentHasBeenIgnored(*Other)||Other->ComponentHasBeenIgnored(*this))return;
        if(GetCollisionResponseToChannel(Other->GetCollisionObjectType())!=ECR_Block||Other->GetCollisionResponseToChannel(GetCollisionObjectType())!=ECR_Block)return;
        const auto Shape=Other->GetCollisionShape();const bool Sphere=Shape.IsSphere();
        if(!Sphere&&!Shape.IsBox())throw std::runtime_error("Unsupported cylinder contact shape");
        const double Radius=Sphere?Shape.GetSphereRadius():0.;const FVector3d Extent=Sphere?FVector3d::ZeroVector:FVector3d(Shape.GetBox());
        const double BoundRadius=Sphere?Radius:Extent.Size();const FCylinderRigidTrajectory OT(*Other,Remaining,BoundRadius);
        if(FVector::Distance(GetKinematicState().Location,Other->GetKinematicState().Location)>R+BoundRadius+CT.Motion+OT.Motion+CT.Error+OT.Error)return;
        Speed::Geometry::FCylinderProjectedMotionBounds Projected;
        const double Dt=Remaining;
        Projected.TranslationVelocity=(OT.Owner.Velocity-CT.Owner.Velocity)*Dt;
        Projected.TranslationAcceleration=(OT.Owner.Acceleration-CT.Owner.Acceleration)*(Dt*Dt);
        const auto Spin=[&](const FCylinderRigidTrajectory& P){return P.Owner.AngularVelocity.Size()+P.Owner.AngularAcceleration.Size()*Dt;};
        Projected.RotationalVelocity=Dt*(Spin(CT)*CT.Span+Spin(OT)*OT.Span);
        Projected.RotationalAcceleration=Dt*Dt*((CT.Owner.AngularAcceleration.Size()+FMath::Square(Spin(CT)))*CT.Span+(OT.Owner.AngularAcceleration.Size()+FMath::Square(Spin(OT)))*OT.Span);
        Projected.TransformRoundoff=128.*std::numeric_limits<double>::epsilon()*(1.+CT.Owner.Location.Size()+OT.Owner.Location.Size()+CT.Span+OT.Span);
        const auto H=Speed::Geometry::SweepCylinderShapeTrajectory(C,Sphere,Radius,Extent,[&](double T){return CT.At(T);},[&](double T){return OT.At(T);},CT.Motion+OT.Motion,CT.Acceleration+OT.Acceleration,CT.Error+OT.Error,&Projected);
        if(H.Result==Speed::Geometry::ECylinderSweepResult::Miss)return;
        if(H.Result!=Speed::Geometry::ECylinderSweepResult::Hit)throw std::runtime_error("Cylinder trajectory unresolved; collision miss refused");
        const float T=float(H.Time*Remaining);if(T>=OutTOI)return;
        OutTOI=T;FutureHit=SHitResult(true,(H.Contact.CylinderPoint+H.Contact.ShapePoint)*.5,-H.Contact.Normal,T,Other);FutureHit.SubBody=Other;
        FutureHit.ContactPointThis=H.Contact.CylinderPoint;FutureHit.ContactPointOther=H.Contact.ShapePoint;FutureHit.GeometricErrorBoundCm=float(2.e-4+CT.Error+OT.Error);
        FutureHit.bStartPenetrating=H.Time==0.&&H.Contact.UpperSeparation<0.;FutureHit.PenetrationDepth=float(FMath::Max(0.,-H.Contact.LowerSeparation));FutureHit.bSurfaceNormalMayVary=true;FutureHit.FrameTag=ParentComponent->NumFrame();
    };
    for(const auto& P:ExternalBoxSubBodies)Visit(P.Get());
    for(const auto& P:ExternalSphereSubBodies)Visit(P.Get());
    for(const auto& P:ExternalWheelSubBodies)Visit(P.Get());
    return FutureHit.bBlockingHit;
}
void UCylinderSubBody::ResolveCurrentHitPrv(const float&,const float&)
{
    auto* Other=Cast<USolidSubBody>(CurrentHit.SubBody.Get());
    if(!Other||!USolidSubBody::ResolvePrescribedContact(*this,*Other,CurrentHit,CurrentHit.PenetrationDepth))throw std::runtime_error("Cylinder pair lost its admitted prescribed response");
}
