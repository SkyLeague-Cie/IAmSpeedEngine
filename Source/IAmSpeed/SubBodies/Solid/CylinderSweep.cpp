#include "CylinderSweep.h"
#include <limits>
namespace Speed::Geometry
{
FCylinderSweep SweepCylinderShapeTrajectory(const FCylinderGeometry& Cylinder,bool bSphere,double Radius,const FVector3d& HalfExtent,
    TFunctionRef<FTransform(double)> CylinderPoseAt,TFunctionRef<FTransform(double)> ShapePoseAt,
    double MotionBoundCm,double AccelerationBoundCm,double PoseErrorCm,const FCylinderProjectedMotionBounds* ProjectedBounds)
{
    FCylinderSweep Out;constexpr double Tolerance=1.e-4;constexpr int32 Budget=4096;
    if(!Cylinder.IsValid()||!FMath::IsFinite(MotionBoundCm)||MotionBoundCm<0.||!FMath::IsFinite(AccelerationBoundCm)||!FMath::IsFinite(PoseErrorCm)||PoseErrorCm<0.||
        (ProjectedBounds&&(ProjectedBounds->TranslationVelocity.ContainsNaN()||ProjectedBounds->TranslationAcceleration.ContainsNaN()||!FMath::IsFinite(ProjectedBounds->RotationalVelocity)||ProjectedBounds->RotationalVelocity<0.||!FMath::IsFinite(ProjectedBounds->RotationalAcceleration)||ProjectedBounds->RotationalAcceleration<0.||!FMath::IsFinite(ProjectedBounds->TransformRoundoff)||ProjectedBounds->TransformRoundoff<0.))||
        (bSphere?(!FMath::IsFinite(Radius)||Radius<0.):(HalfExtent.ContainsNaN()||HalfExtent.GetMin()<=0.)))return Out;
    struct FSample {FTransform CylinderPose,ShapePose;FCylinderContact Contact;};
    auto Sample=[&](double Time,FSample& S)
    {
        if(++Out.PoseEvaluations>Budget)return false;
        S.CylinderPose=CylinderPoseAt(Time);S.ShapePose=ShapePoseAt(Time);
        if(S.CylinderPose.ContainsNaN()||S.ShapePose.ContainsNaN()||!S.CylinderPose.GetScale3D().Equals(FVector::OneVector,1.e-12)||!S.ShapePose.GetScale3D().Equals(FVector::OneVector,1.e-12))return false;
        const auto P=S.CylinderPose.InverseTransformPosition(S.ShapePose.GetTranslation());
        const auto Q=S.CylinderPose.GetRotation().Inverse()*S.ShapePose.GetRotation();
        const bool Valid=bSphere?TryCylinderSphereContact(Cylinder,P,Radius,S.Contact):TryCylinderBoxContact(Cylinder,P,Q,HalfExtent,S.Contact,Tolerance/32.);
        if(!Valid||!S.Contact.bConverged)return false;
        S.Contact.Normal=S.CylinderPose.TransformVectorNoScale(S.Contact.Normal);
        S.Contact.CylinderPoint=S.CylinderPose.TransformPosition(S.Contact.CylinderPoint);
        S.Contact.ShapePoint=S.CylinderPose.TransformPosition(S.Contact.ShapePoint);return true;
    };
    auto Hit=[&](double T,const FSample& S){Out.Result=ECylinderSweepResult::Hit;Out.Time=T;Out.Contact=S.Contact;};
    auto PlaneGap=[&](const FSample& S,const FVector3d& N)
    {
        const auto NC=S.CylinderPose.InverseTransformVectorNoScale(N);
        const FVector3d C=S.CylinderPose.TransformPosition(Cylinder.Support(NC));
        FVector3d A;
        if(bSphere)A=S.ShapePose.GetTranslation()-N*Radius;
        else
        {
            const auto L=S.ShapePose.InverseTransformVectorNoScale(-N);
            const FVector3d V(L.X>=0.?HalfExtent.X:-HalfExtent.X,L.Y>=0.?HalfExtent.Y:-HalfExtent.Y,L.Z>=0.?HalfExtent.Z:-HalfExtent.Z);
            A=S.ShapePose.TransformPosition(V);
        }
        return FVector3d::DotProduct(A-C,N);
    };
    FSample First,Last;
    if(!Sample(0.,First)||!Sample(1.,Last)){Out.Result=ECylinderSweepResult::Unresolved;return Out;}
    // Match the float slop consumed by SolveOverlap: a double comparison
    // below that float bin can repeatedly request a correction that the
    // resolver correctly rounds to zero. No tolerance is increased.
    if(float(-First.Contact.UpperSeparation)>.05f){Hit(0.,First);return Out;}
    if(MotionBoundCm==0.){Out.Result=ECylinderSweepResult::Miss;return Out;}
    struct FInterval{double A,B;FSample SA,SB;};TArray<FInterval> Stack;Stack.Reserve(64);Stack.Add({0.,1.,First,Last});
    Out.Result=ECylinderSweepResult::Miss;
    while(!Stack.IsEmpty())
    {
        const FInterval I=Stack.Pop(EAllowShrinking::No);const double W=I.B-I.A;
        const double A=I.SA.Contact.LowerSeparation,B=I.SB.Contact.LowerSeparation;
        const double Roundoff=128.*std::numeric_limits<double>::epsilon()*(1.+FMath::Abs(A)+FMath::Abs(B)+MotionBoundCm*W);
        double Lower=FMath::Min(FMath::Min(A,B),.5*(A+B-MotionBoundCm*W))-2.*PoseErrorCm-Roundoff;
        if(AccelerationBoundCm>=0.)
        {
            // Minimum over material point pairs preserves semiconcavity:
            // fixed separating-plane gap >= endpoint chord - A*w*w/8.
            // Use each endpoint normal, never differentiate a changing witness.
            for(const FVector3d& N:{I.SA.Contact.Normal,I.SB.Contact.Normal})
            {
                const double GA=PlaneGap(I.SA,N),GB=PlaneGap(I.SB,N);
                double PlaneAcceleration=AccelerationBoundCm,PlaneError=PoseErrorCm;
                if(ProjectedBounds)
                {
                    const auto& P=*ProjectedBounds;
                    PlaneAcceleration=FMath::Abs(FVector3d::DotProduct(N,P.TranslationAcceleration))+P.RotationalAcceleration;
                    const double PlaneMotion=FMath::Abs(FVector3d::DotProduct(N,P.TranslationVelocity))+FMath::Abs(FVector3d::DotProduct(N,P.TranslationAcceleration))+P.RotationalVelocity;
                    PlaneError=8.*double(std::numeric_limits<float>::epsilon())*PlaneMotion+P.TransformRoundoff;
                }
                const double PlaneLower=FMath::Min(GA,GB)-PlaneAcceleration*W*W*.125-2.*PlaneError-Roundoff;
                Lower=FMath::Max(Lower,PlaneLower);
                // A whole interval whose certified penetration cannot exceed
                // the existing spatial error beyond its tolerated initial overlap may
                // release a certified nonclosing contact. The -.05 admission
                // and spatial tolerance remain unchanged.
                // This avoids exhaustively subdividing a box sliding along a
                // cylinder side while gravity acts parallel to that side.
                const auto& SA=I.SA;const auto& SB=I.SB;
                const FVector3d LA=SA.ShapePose.InverseTransformPosition(SA.Contact.ShapePoint),LB=SA.CylinderPose.InverseTransformPosition(SA.Contact.CylinderPoint);
                const FVector3d Relative=(SB.ShapePose.TransformPosition(LA)-SA.Contact.ShapePoint)-(SB.CylinderPose.TransformPosition(LB)-SA.Contact.CylinderPoint);
                if(SA.Contact.UpperSeparation<=Tolerance&&float(-SA.Contact.LowerSeparation)<=.05f&&PlaneLower>=FMath::Min(0.,GA)-Tolerance&&FVector3d::DotProduct(N,Relative)>=-Roundoff)
                    Lower=FMath::Max(Lower,Tolerance);
                // Exact affine, nonpenetrating support plane: touching while
                // separating/sliding needs no new event impulse.
                if(AccelerationBoundCm==0.&&PoseErrorCm==0.&&
                    ((GA>=0.&&GB>=0.)||(First.Contact.UpperSeparation<=0.&&GA>=-.05&&GB>=GA)))
                    Lower=FMath::Max(Lower,Tolerance);
            }
        }
        if(Lower>0.)continue;
        if(MotionBoundCm*W<=Tolerance)
        {
            const auto& SA=I.SA;const auto& SB=I.SB;
            const FVector3d LocalA=SA.ShapePose.InverseTransformPosition(SA.Contact.ShapePoint),LocalB=SA.CylinderPose.InverseTransformPosition(SA.Contact.CylinderPoint);
            const FVector3d Relative=(SB.ShapePose.TransformPosition(LocalA)-SA.Contact.ShapePoint)-(SB.CylinderPose.TransformPosition(LocalB)-SA.Contact.CylinderPoint);
            const double Closing=FVector3d::DotProduct(SA.Contact.Normal,Relative);
            if(float(-SA.Contact.UpperSeparation)<=.05f&&Closing>=0.)continue;
            if(SA.Contact.UpperSeparation<=2.*Tolerance){Hit(I.A,SA);return Out;}
            // Width alone cannot certify a hit in the presence of pose error.
            // Retain this earlier interval and refine; keep the same spatial
            // acceptance, pose budget and fail-closed collapse conditions.
        }
        const double Mid=(I.A+I.B)*.5;FSample M;
        if(Mid==I.A||Mid==I.B||!Sample(Mid,M)){Out.Result=ECylinderSweepResult::Unresolved;return Out;}
        Stack.Add({Mid,I.B,M,I.SB});Stack.Add({I.A,Mid,I.SA,M});
    }
    return Out;
}
}
