#include "TorusSphereSweep.h"
#include <cmath>

namespace Speed::Geometry
{
namespace
{
double Evaluate(const TArray<double>& Coefficients,double Time)
{
    double Value=0.;
    for (int32 I=Coefficients.Num()-1;I>=0;--I) Value=std::fma(Value,Time,Coefficients[I]);
    return Value;
}
void AddRoot(TArray<double>& Roots,double Root)
{
    Root=FMath::Clamp(Root,0.,1.);
    if (!Roots.ContainsByPredicate([Root](double Other){return FMath::Abs(Other-Root)<1.e-12;})) Roots.Add(Root);
}
void RootsInUnitInterval(TArray<double> Coefficients,TArray<double>& Roots)
{
    double Scale=0.; for (double C:Coefficients) Scale=FMath::Max(Scale,FMath::Abs(C));
    if (Scale==0.) return;
    for (double& C:Coefficients) C/=Scale;
    while (Coefficients.Num()>1 && FMath::Abs(Coefficients.Last())<1.e-15) Coefficients.Pop(EAllowShrinking::No);
    const int32 Degree=Coefficients.Num()-1;
    if (Degree==0) return;
    if (Degree==1)
    {
        const double Root=-Coefficients[0]/Coefficients[1];
        if (Root>=0. && Root<=1.) AddRoot(Roots,Root);
        return;
    }
    TArray<double> Derivative,Critical;
    for (int32 I=1;I<Coefficients.Num();++I) Derivative.Add(I*Coefficients[I]);
    RootsInUnitInterval(MoveTemp(Derivative),Critical);
    Critical.Add(0.); Critical.Add(1.); Critical.Sort();
    for (double T:Critical) if (FMath::Abs(Evaluate(Coefficients,T))<=1.e-13) AddRoot(Roots,T);
    for (int32 I=1;I<Critical.Num();++I)
    {
        double A=Critical[I-1],B=Critical[I],FA=Evaluate(Coefficients,A),FB=Evaluate(Coefficients,B);
        if ((FA<0.)==(FB<0.) || FA==0. || FB==0.) continue;
        for (int32 Iteration=0;Iteration<64;++Iteration)
        {
            const double Mid=(A+B)*.5,FM=Evaluate(Coefficients,Mid);
            if (FM==0.) { A=B=Mid; break; }
            if ((FM<0.)==(FA<0.)) { A=Mid;FA=FM; } else B=Mid;
        }
        AddRoot(Roots,(A+B)*.5);
    }
}
bool ClipSegment(const FVector3d& Start,const FVector3d& Delta,const FVector3d& Extent,double& Enter,double& Exit)
{
    Enter=0.;Exit=1.;
    for (int32 Axis=0;Axis<3;++Axis)
    {
        if (Delta[Axis]==0.) { if (FMath::Abs(Start[Axis])>Extent[Axis]) return false; continue; }
        double A=(-Extent[Axis]-Start[Axis])/Delta[Axis],B=(Extent[Axis]-Start[Axis])/Delta[Axis];
        if (A>B) Swap(A,B);
        Enter=FMath::Max(Enter,A);Exit=FMath::Min(Exit,B);
        if (Enter>Exit) return false;
    }
    return true;
}
}

FTorusSphereSweep SweepTorusSphere(const FTorusGeometry& Torus,const FVector3d& Start,
    const FVector3d& End,double Radius,double InitialOverlapTolerance)
{
    FTorusSphereSweep Out;
    FTorusContact Initial;
    if (!TryTorusSphereContact(Torus,Start,Radius,Initial) || !FMath::IsFinite(End.X) ||
        !FMath::IsFinite(End.Y) || !FMath::IsFinite(End.Z) ||
        !FMath::IsFinite(InitialOverlapTolerance) || InitialOverlapTolerance<0.) return Out;
    const FVector3d Delta=End-Start;
    if (Delta.ContainsNaN() || !FMath::IsFinite(Delta.SizeSquared())) return Out;
    constexpr double ContactTolerance=1.e-5;
    const double InitialClosing=FVector3d::DotProduct(Delta,Initial.Normal);
    if (Initial.Separation < -InitialOverlapTolerance ||
        (Initial.Separation<=0. && InitialClosing<0.))
    {
        // The filled-hole, on-axis initial case has no unique translation witness.
        if (Start.Y==0. && Start.Z==0. && Radius+Torus.TubeRadius>=Torus.MajorRadius)
        { Out.Result=ETorusSweepResult::Unresolved; return Out; }
        Out.Result=ETorusSweepResult::Hit; Out.Time=0.; Out.Contact=Initial;
        Out.bInitialOverlap=Initial.Separation<0.; return Out;
    }
    Out.Result=ETorusSweepResult::Miss;
    if (Delta.IsZero()) return Out;
    const double Expanded=Torus.TubeRadius+Radius;
    double Enter=0.,Exit=1.;
    if (!ClipSegment(Start,Delta,FVector3d(Expanded,Torus.MajorRadius+Expanded,Torus.MajorRadius+Expanded),Enter,Exit)) return Out;
    // Crop BEFORE forming the quartic: a very long flight must not generate
    // catastrophic coefficient cancellation around a millimetre-wide contact.
    const FVector3d P=(Start+Delta*Enter)/Torus.MajorRadius;
    const FVector3d V=Delta*((Exit-Enter)/Torus.MajorRadius);
    const double R=Expanded/Torus.MajorRadius;
    const double A0=P.SizeSquared()+1.-R*R,A1=2.*FVector3d::DotProduct(P,V),A2=V.SizeSquared();
    const double B0=P.Y*P.Y+P.Z*P.Z,B1=2.*(P.Y*V.Y+P.Z*V.Z),B2=V.Y*V.Y+V.Z*V.Z;
    TArray<double> Coefficients{A0*A0-4.*B0,2.*A0*A1-4.*B1,A1*A1+2.*A0*A2-4.*B2,2.*A1*A2,A2*A2};
    TArray<double> Roots; RootsInUnitInterval(MoveTemp(Coefficients),Roots);Roots.Sort();
    for (double Root:Roots)
    {
        const double Time=Enter+(Exit-Enter)*Root;
        FTorusContact Contact;
        if (!TryTorusSphereContact(Torus,Start+Delta*Time,Radius,Contact))
        { Out.Result=ETorusSweepResult::Invalid; return Out; }
        if (FMath::Abs(Contact.Separation)>ContactTolerance) continue;
        // A released separating contact must not hide a later re-entry on the
        // opposite side of the torus. Evaluate every ordered physical root.
        if (FVector3d::DotProduct(Delta,Contact.Normal)>=0.) continue;
        Out.Result=ETorusSweepResult::Hit;Out.Time=Time;Out.Contact=Contact;return Out;
    }
    return Out;
}
}
