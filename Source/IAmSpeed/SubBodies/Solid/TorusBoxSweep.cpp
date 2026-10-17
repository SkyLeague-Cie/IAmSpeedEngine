#include "TorusBoxSweep.h"
#include <limits>

namespace Speed::Geometry
{
FTorusBoxSweep SweepTorusShapeBoundedTrajectory(const FTorusGeometry& Torus,
    bool bSphere,double Radius,const FVector3d& HalfExtent,TFunctionRef<FTransform(double)> PoseAt,
    double MotionBoundCm,double InitialOverlapTolerance,double AccelerationBoundCm,
    double PoseErrorCm,int32* PoseEvaluations)
{
    FTorusBoxSweep Out;
    if (PoseEvaluations) *PoseEvaluations=0;
    constexpr double ToleranceCm=1.e-4;
    constexpr int32 MaximumPoseEvaluations=4096;
    if (!Torus.IsValid() || (!bSphere && (HalfExtent.ContainsNaN() || HalfExtent.GetMin()<=0.)) ||
        (bSphere && (!FMath::IsFinite(Radius) || Radius<0.)) ||
        !FMath::IsFinite(MotionBoundCm) || MotionBoundCm<0. ||
        !FMath::IsFinite(InitialOverlapTolerance) || InitialOverlapTolerance<0. ||
        !FMath::IsFinite(AccelerationBoundCm) || !FMath::IsFinite(PoseErrorCm) || PoseErrorCm<0.) return Out;
    struct FSample { FTransform Pose; FTorusBoxDistance Distance; FVector3d Normal=FVector3d::ZeroVector; FVector3d Surface=FVector3d::ZeroVector; };
    int32 Evaluations=0;
    const auto Sample=[&](double Time,FSample& S)
    {
        if (PoseEvaluations) *PoseEvaluations=Evaluations+1;
        if (++Evaluations>MaximumPoseEvaluations) return false;
        S.Pose=PoseAt(Time);
        if (S.Pose.ContainsNaN() || !S.Pose.GetScale3D().Equals(FVector::OneVector,1.e-9)) return false;
        if (bSphere)
        {
            FTorusContact Contact;
            if (!TryTorusSphereContact(Torus,S.Pose.GetTranslation(),Radius,Contact)) return false;
            S.Distance.LowerSeparation=S.Distance.UpperSeparation=Contact.Separation;
            S.Distance.BoxPoint=S.Pose.GetTranslation()-Contact.Normal*Radius;
            S.Normal=Contact.Normal; S.Surface=Contact.SurfacePoint;
            return true;
        }
        if (!TryTorusBoxDistance(Torus,S.Pose.GetTranslation(),S.Pose.GetRotation(),HalfExtent,
            ToleranceCm/16.,8192,S.Distance) || !S.Distance.bConverged) return false;
        const FVector3d Offset=S.Distance.BoxPoint-S.Distance.CentrelinePoint;
        S.Normal=Offset.GetSafeNormal(1.e-12);
        S.Surface=S.Distance.CentrelinePoint+S.Normal*Torus.TubeRadius;
        return true;
    };
    FSample First,Last;
    if (!Sample(0.,First)) { Out.Result=ETorusSweepResult::Unresolved; return Out; }
    const auto Contact=[&](double Time,const FSample& S)
    {
        if (S.Normal.IsZero()) { Out.Result=ETorusSweepResult::Unresolved; return; }
        Out.Result=ETorusSweepResult::Hit;Out.Time=Time;Out.Separation=S.Distance.UpperSeparation;
        Out.Normal=S.Normal;
        Out.SurfacePoint=S.Surface;
        Out.BoxPoint=S.Distance.BoxPoint;
        Out.ErrorBoundCm=2.*ToleranceCm;
    };
    // An already intersecting pose is a t=0 contact, not a temporal search.
    // Subdividing a persistent initial overlap until every interval is tiny
    // can exhaust the CCD budget before any motion has occurred. The adapter
    // still independently certifies the whole-torus MTD before accepting it.
    if (First.Distance.UpperSeparation<0.) { Contact(0.,First); return Out; }
    if (!Sample(1.,Last)) { Out.Result=ETorusSweepResult::Unresolved; return Out; }
    if (MotionBoundCm==0.)
    {
        if (First.Distance.UpperSeparation < -InitialOverlapTolerance) Contact(0.,First);
        else Out.Result=ETorusSweepResult::Miss;
        return Out;
    }
    struct FInterval { double A,B; FSample SA,SB; };
    TArray<FInterval> Stack;Stack.Add({0.,1.,First,Last});
    Out.Result=ETorusSweepResult::Miss;
    while (!Stack.IsEmpty())
    {
        FInterval Node=Stack.Pop(EAllowShrinking::No);
        const double Width=Node.B-Node.A;
        // Distance to a fixed set is 1-Lipschitz under point motion. This
        // excludes a WHOLE time interval, not just its sampled endpoints.
        // Intersect BOTH endpoint Lipschitz cones. Their intersection is the
        // minimum possible clearance anywhere in this interval. The previous
        // max(endpoint)-L*width discarded the opposite cone and needlessly
        // exhausted the pose budget on near-tangent, separating contacts.
        const double DA=Node.SA.Distance.LowerSeparation,DB=Node.SB.Distance.LowerSeparation;
        double Lower=FMath::Min(FMath::Min(DA,DB),.5*(DA+DB-MotionBoundCm*Width))-2.*PoseErrorCm;
        if (AccelerationBoundCm>=0.)
        {
            const double ShapeRadius=bSphere ? Radius : HalfExtent.Size();
            const auto RadialLower=[&](const FSample& S)
            {
                const auto P=S.Pose.GetTranslation();
                return FMath::Sqrt(P.Y*P.Y+P.Z*P.Z)-ShapeRadius;
            };
            const double Rho=FMath::Max(RadialLower(Node.SA),RadialLower(Node.SB))-MotionBoundCm*Width-2.*PoseErrorCm;
            const double CircleDistance=Torus.TubeRadius+FMath::Max(DA,DB)-MotionBoundCm*Width-2.*PoseErrorCm;
            if (Rho>0. && CircleDistance>0.)
            {
                // Distance to a circular centreline has Hessian upper norm
                // <=1/rho+1/d away from its axis/centreline. Compose with the
                // point trajectory: f'' <= A+L^2*(1/rho+1/d). The minimum over
                // ALL material points keeps the endpoint chord lower bound,
                // including box feature changes (no witness-normal assumption).
                const double Curvature=AccelerationBoundCm+MotionBoundCm*MotionBoundCm*(1./Rho+1./CircleDistance);
                if (FMath::IsFinite(Curvature))
                    Lower=FMath::Max(Lower,FMath::Min(DA,DB)-Curvature*Width*Width*.125-2.*PoseErrorCm);
            }
        }
        Lower-=64.*std::numeric_limits<double>::epsilon()*(1.+FMath::Abs(DA)+FMath::Abs(DB)+MotionBoundCm*Width);
        if (Lower>0.) continue;
        if (MotionBoundCm*Width<=ToleranceCm)
        {
            const FVector3d Normal=Node.SA.Normal;
            if (Normal.IsZero()) { Out.Result=ETorusSweepResult::Unresolved;return Out; }
            // Release shallow separating/tangent contact using motion of the
            // SAME material point, never motion of the nearest-witness index.
            const FVector LocalPoint=Node.SA.Pose.InverseTransformPosition(Node.SA.Distance.BoxPoint);
            const double Closing=FVector3d::DotProduct(Normal,bSphere ?
                Node.SB.Pose.GetTranslation()-Node.SA.Pose.GetTranslation() :
                Node.SB.Pose.TransformPosition(LocalPoint)-Node.SA.Distance.BoxPoint);
            if (Node.SA.Distance.UpperSeparation>=-InitialOverlapTolerance && Closing>=0.) continue;
            if (Node.SA.Distance.UpperSeparation<=2.*ToleranceCm) { Contact(Node.A,Node.SA); return Out; }
            Out.Result=ETorusSweepResult::Unresolved;return Out;
        }
        const double Mid=(Node.A+Node.B)*.5; FSample Middle;
        if (Mid==Node.A || Mid==Node.B || !Sample(Mid,Middle))
        { Out.Result=ETorusSweepResult::Unresolved;return Out; }
        // LIFO: earlier interval must be processed before any later contact.
        Stack.Add({Mid,Node.B,Middle,Node.SB});
        Stack.Add({Node.A,Mid,Node.SA,Middle});
    }
    return Out;
}
FTorusBoxSweep SweepTorusShapeTrajectory(const FTorusGeometry& Torus,
    bool bSphere,double Radius,const FVector3d& HalfExtent,TFunctionRef<FTransform(double)> PoseAt,
    double MotionBoundCm,double InitialOverlapTolerance)
{
    return SweepTorusShapeBoundedTrajectory(Torus,bSphere,Radius,HalfExtent,PoseAt,
        MotionBoundCm,InitialOverlapTolerance,-1.);
}
FTorusBoxSweep SweepTorusBoxTrajectory(const FTorusGeometry& Torus,
    const FVector3d& HalfExtent,TFunctionRef<FTransform(double)> PoseAt,
    double MotionBoundCm,double InitialOverlapTolerance)
{
    return SweepTorusShapeTrajectory(Torus,false,0.,HalfExtent,PoseAt,MotionBoundCm,InitialOverlapTolerance);
}
}
