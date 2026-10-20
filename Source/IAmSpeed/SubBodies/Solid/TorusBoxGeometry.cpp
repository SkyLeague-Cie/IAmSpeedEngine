#include "TorusBoxGeometry.h"
#include <cmath>
#include <limits>

namespace Speed::Geometry
{
namespace
{
bool Finite(const FVector3d& V)
{ return FMath::IsFinite(V.X) && FMath::IsFinite(V.Y) && FMath::IsFinite(V.Z); }
double Length(const FVector3d& V) { return std::hypot(V.X,std::hypot(V.Y,V.Z)); }
struct FArc { double Start=0.,End=0.,Lower=0.; };

// Exact coordinate extrema of C+U*cos(t)+V*sin(t) over the arc interval.
void CoordinateRange(double C,double U,double V,double Start,double End,double& Min,double& Max)
{
    const auto Value=[=](double T){ return C+U*std::cos(T)+V*std::sin(T); };
    Min=FMath::Min(Value(Start),Value(End)); Max=FMath::Max(Value(Start),Value(End));
    double Peak=std::atan2(V,U); if (Peak<0.) Peak+=2.*PI;
    for (int32 I=0;I<2;++I)
    {
        double T=Peak+I*PI; if (T>=2.*PI) T-=2.*PI;
        if (T>=Start && T<=End) { const double X=Value(T); Min=FMath::Min(Min,X); Max=FMath::Max(Max,X); }
    }
}
}

bool TryTorusBoxDistance(const FTorusGeometry& Torus,
    const FVector3d& BoxCentre,const FQuat4d& BoxRotation,const FVector3d& HalfExtent,
    double ToleranceCm,int32 MaximumEvaluations,FTorusBoxDistance& Out)
{
    Out={};
    if (!Torus.IsValid() || !Finite(BoxCentre) || !Finite(HalfExtent) || HalfExtent.GetMin()<=0. ||
        BoxRotation.ContainsNaN() || !BoxRotation.IsNormalized() || !FMath::IsFinite(ToleranceCm) ||
        ToleranceCm<=0. || MaximumEvaluations<1 || MaximumEvaluations>65536) return false;
    const FVector3d C=BoxRotation.UnrotateVector(-BoxCentre);
    const FVector3d U=BoxRotation.UnrotateVector(FVector3d(0.,Torus.MajorRadius,0.));
    const FVector3d V=BoxRotation.UnrotateVector(FVector3d(0.,0.,Torus.MajorRadius));
    if (!Finite(C) || !Finite(U) || !Finite(V)) return false;
    double Best=TNumericLimits<double>::Max();
    FVector3d BestCircle=FVector3d::ZeroVector,BestBox=FVector3d::ZeroVector;
    const auto Evaluate=[&](double Angle)
    {
        const FVector3d Q=C+U*std::cos(Angle)+V*std::sin(Angle);
        const FVector3d B=Q.ComponentMax(-HalfExtent).ComponentMin(HalfExtent);
        const double Distance=Length(Q-B); ++Out.Evaluations;
        if (Distance<Best) { Best=Distance; BestCircle=Q; BestBox=B; }
    };
    const auto Bound=[&](double Start,double End)
    {
        FVector3d Gap=FVector3d::ZeroVector;
        for (int32 Axis=0;Axis<3;++Axis)
        {
            double Min=0.,Max=0.; CoordinateRange(C[Axis],U[Axis],V[Axis],Start,End,Min,Max);
            Gap[Axis]=FMath::Max(0.,FMath::Max(Min-HalfExtent[Axis],-HalfExtent[Axis]-Max));
        }
        // Squared distance to a convex box has a Lipschitz derivative along
        // the circular centreline. The Taylor enclosure is quadratic in arc
        // width, including when several circle angles share the same minimum.
        ++Out.Evaluations;
        const double Mid=(Start+End)*.5,Half=(End-Start)*.5;
        const FVector3d Q=C+U*std::cos(Mid)+V*std::sin(Mid);
        const FVector3d B=Q.ComponentMax(-HalfExtent).ComponentMin(HalfExtent);
        const FVector3d Difference=Q-B;
        const FVector3d Derivative=-U*std::sin(Mid)+V*std::cos(Mid);
        const double Slope=2.*FVector3d::DotProduct(Difference,Derivative);
        const double Radius=Torus.MajorRadius;
        const double SecondDerivativeBound=2.*Radius*Radius+
            2.*Radius*(Length(C)+Radius+Length(HalfExtent));
        const double Roundoff=128.*std::numeric_limits<double>::epsilon()*
            (1.+C.SizeSquared()+Radius*Radius+HalfExtent.SizeSquared());
        const double TaylorSquared=Difference.SizeSquared()-FMath::Abs(Slope)*Half-
            .5*SecondDerivativeBound*Half*Half-Roundoff;
        const double ArcBoxLower=FMath::Max(0.,Length(Gap)-64.*std::numeric_limits<double>::epsilon()*
            (1.+Length(C)+Radius+Length(HalfExtent)));
        return FMath::Max(ArcBoxLower,FMath::Sqrt(FMath::Max(0.,TaylorSquared)));
    };
    Evaluate(0.);
    TArray<FArc> Arcs;
    Arcs.Add({0.,2.*PI,Out.Evaluations<MaximumEvaluations ? Bound(0.,2.*PI) : 0.});
    double Lower=0.;
    while (!Arcs.IsEmpty())
    {
        int32 Winner=0;
        for (int32 I=1;I<Arcs.Num();++I) if (Arcs[I].Lower<Arcs[Winner].Lower) Winner=I;
        Lower=FMath::Min(Best,Arcs[Winner].Lower);
        if (Best-Lower<=ToleranceCm) { Out.bConverged=true; break; }
        if (Out.Evaluations+3>MaximumEvaluations) break;
        const FArc Arc=Arcs[Winner]; Arcs.RemoveAtSwap(Winner,EAllowShrinking::No);
        const double Mid=(Arc.Start+Arc.End)*.5;
        if (Mid==Arc.Start || Mid==Arc.End) { Arcs.Add(Arc); break; }
        Evaluate(Mid);
        const double L=Bound(Arc.Start,Mid),R=Bound(Mid,Arc.End);
        if (L<Best) Arcs.Add({Arc.Start,Mid,L});
        if (R<Best) Arcs.Add({Mid,Arc.End,R});
    }
    if (Arcs.IsEmpty()) { Lower=Best; Out.bConverged=true; }
    Out.LowerSeparation=Lower-Torus.TubeRadius;
    Out.UpperSeparation=Best-Torus.TubeRadius;
    Out.CentrelinePoint=BoxCentre+BoxRotation.RotateVector(BestCircle);
    Out.BoxPoint=BoxCentre+BoxRotation.RotateVector(BestBox);
    if (!Finite(Out.CentrelinePoint) || !Finite(Out.BoxPoint) ||
        !FMath::IsFinite(Out.LowerSeparation) || !FMath::IsFinite(Out.UpperSeparation)) { Out={}; return false; }
    return true;
}
}
