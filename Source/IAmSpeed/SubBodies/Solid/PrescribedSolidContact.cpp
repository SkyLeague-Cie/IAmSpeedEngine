#include "SolidSubBody.h"
#include "SWheelSubBody.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
bool USolidSubBody::ResolvePrescribedContact(USolidSubBody& A,USolidSubBody& B,const SHitResult& Hit,double Penetration)
{
    ISpeedComponent* PA=A.GetParentComponent();ISpeedComponent* PB=B.GetParentComponent();
    if(!PA||!PB||(!PA->IsPrescribedMotion()&&!PB->IsPrescribedMotion()))return false;
    if(PA==PB)return true;
    const FVector N=Hit.ImpactNormal.GetSafeNormal();if(N.IsNearlyZero())return false;
    const FVector Point=Hit.ImpactPoint;
    const SKinematic KA=PA->GetKinematicState(),KB=PB->GetKinematicState();
    const FMatrix IA=A.ComputeWorldInvInertiaTensor(),IB=B.ComputeWorldInvInertiaTensor();
    const float MA=PA->GetPhysMass(),MB=PB->GetPhysMass();
    // No gameplay shot/bump model on an authored mechanical obstacle. The
    // complete existing car/ball force and material profiles stay unchanged.
    if(Penetration>.05)
        SolveOverlap(*PA,MA,IA,KA,PB,MB,IB,KB,Point,N,float(Penetration),.05f,.8f,FMath::Max(A.GetImpactThreshold(),B.GetImpactThreshold()));
    SHitResult Reverse=Hit;Reverse.ImpactNormal=-N;Swap(Reverse.ContactPointThis,Reverse.ContactPointOther);Reverse.Component=&A;Reverse.SubBody=&A;
    A.SetHit(Hit);B.SetHit(Reverse);
    FVector JA,JB;
    const float E=MixRestitution(A.GetRestitution(),B.GetRestitution(),EMixMode::E_Max);
    const float Mu=MixFriction(A.GetDynamicFriction(),B.GetDynamicFriction(),EMixMode::E_Max);
    if(Speed::SImpulseSolver::ComputeCollisionImpulse(Point,N,KA,MA,IA,KB,MB,IB,E,Mu,JA,JB,FMath::Max(A.GetImpactThreshold(),B.GetImpactThreshold())))
    {A.ApplyImpulse(JA,Point);B.ApplyImpulse(JB,Point);}
    if(!PA->IsPrescribedMotion())A.RegisterContactAsConstraint(Point,N,&B,&B,float(FMath::Max(0.,Penetration)),Hit.TOI,false);
    if(!PB->IsPrescribedMotion())B.RegisterContactAsConstraint(Point,-N,&A,&A,float(FMath::Max(0.,Penetration)),Hit.TOI,false);
    // A TOI at zero does not run IntegrateKinematics, so its usual refresh
    // cannot publish a positional correction or impulse to collider caches.
    // Refresh both admitted owners before the next same-frame CCD query.
    PA->UpdateSubBodiesKinematics();PB->UpdateSubBodiesKinematics();
    return true;
}
