#include <type_traits>
#include "HAL/IConsoleManager.h"
#include "Case202Serial3GraphEvidence.h"
#if PLATFORM_WINDOWS && defined(_M_X64)
#include "Case202Serial3LoadedImageVerifier.h"
#include "HAL/PlatformMisc.h"
#if __has_include("Case202Serial3ReviewerPublicKey.inl")
#include "Case202Serial3ReviewerPublicKey.inl"
#endif
#endif
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include <array>
#include "Misc/ScopedCVar.h"
#include "IAmSpeed/World/Simulation/FrameFloatingPointContext.h"
#endif

// Uncompiled geometry-only draft. Include outside namespaces in the IAS
// producer TU. The caller must bind the authentic FProduct to the selected
// reference, a complete competitor partition, the external world and policy.
#include "IAmSpeed/World/Analytic/NativeFloorBoxSupportBounds.h"
#include "IAmSpeed/World/Analytic/ReferenceAffineExtrusionMath.h"
#include <cmath>
#include <initializer_list>
#include "Case205TerminalRemainderObservation.inl"
#include "Case205ReferenceTimingProof.inl"

namespace Speed::Analytic::Certified::FactoredLowerCapGraphGap
{
namespace A=::AnalyticCertified;
using A::Interval;

struct FControl { Interval Y,Z; };

inline Interval DivideByPositive(Interval numerator,Interval denominator)
{
    if(!A::Valid(numerator)||!A::Valid(denominator)||!(denominator.Lower>0.))return {};
    volatile double lo=1./denominator.Upper,hi=1./denominator.Lower;
    Interval inverse;
    inverse.Known=A::Outward(lo,false,inverse.Lower)&&A::Outward(hi,true,inverse.Upper);
    return A::Multiply(numerator,inverse);
}

template<int N> inline Interval Evaluate(const Interval (&controls)[N],Interval parameter)
{
    if(!A::Valid(parameter)||parameter.Lower<0.||parameter.Upper>1.)return {};
    const auto complement=A::Subtract(Interval::Exact(1.),parameter);
    Interval row[N];
    for(int i=0;i<N;++i)row[i]=controls[i];
    for(int n=N-1;n>0;--n)for(int i=0;i<n;++i)
        row[i]=A::Add(A::Multiply(complement,row[i]),A::Multiply(parameter,row[i+1]));
    return row[0];
}

class FProfile
{
    Interval Origin,LastY,FactorY[5],FactorZ[3],FactorYHull;
    bool Known=false;
    template<class TPay> bool BoundPositiveOffset(Interval y,TPay& pay,Interval& v) const
    {
        if(!A::Valid(y)||!(y.Lower>0.))return false;
        v=DivideByPositive(y,FactorYHull);
        if(!A::Valid(v)||!(0.<v.Lower&&v.Upper<1.))return false;
        // Three bounded contractions are sufficient for the near-seam
        // capture. The count changes only precision, not containment.
        for(int i=0;i<3;++i)
        {
            if(!pay())return false;
            const auto implied=DivideByPositive(y,Evaluate(FactorY,v));
            if(!A::Valid(implied))return false;
            if(implied.Lower>v.Lower)v.Lower=implied.Lower;
            if(implied.Upper<v.Upper)v.Upper=implied.Upper;
            if(!A::Valid(v))return false;
        }
        return true;
    }

public:
    template<class TPay> static bool TryBuild(const FControl (&controls)[6],TPay&& pay,FProfile& out)
    {
        out={};
        FProfile result;
        for(int i=0;i<6;++i)
        {
            if(!pay()||!A::Valid(controls[i].Y)||!A::Valid(controls[i].Z)||controls[i].Z.Lower<0.)return false;
            if(i&&!(controls[i-1].Y.Upper<controls[i].Y.Lower))return false;
            if(i<3&&(controls[i].Z.Lower!=0.||controls[i].Z.Upper!=0.))return false;
        }
        if(A::Bits(controls[0].Y.Lower)!=A::Bits(controls[0].Y.Upper))return false;
        result.Origin=controls[0].Y;result.LastY=controls[5].Y;
        // (Y(v)-Y(0))/v has degree four, and its Bernstein controls are
        // (5/(j+1))*(Y[j+1]-Y[0]). The global hull is positive.
        for(int j=0;j<5;++j)
        {
            if(!pay())return false;
            const auto scaled=A::Multiply(Interval::Exact(5.),A::Subtract(controls[j+1].Y,result.Origin));
            const auto term=DivideByPositive(scaled,Interval::Exact(double(j+1)));
            if(!A::Valid(term)||!(term.Lower>0.))return false;
            result.FactorY[j]=term;
            if(!j)result.FactorYHull=term;
            else
            {
                if(term.Lower<result.FactorYHull.Lower)result.FactorYHull.Lower=term.Lower;
                if(term.Upper>result.FactorYHull.Upper)result.FactorYHull.Upper=term.Upper;
            }
        }
        // Z(v)/v^3 is quadratic because Z[0..2] are exactly zero.
        if(!pay())return false;
        result.FactorZ[0]=A::Multiply(Interval::Exact(10.),controls[3].Z);
        result.FactorZ[1]=A::Multiply(Interval::Exact(2.5),controls[4].Z);
        result.FactorZ[2]=controls[5].Z;
        if(!A::Valid(result.FactorZ[0])||!A::Valid(result.FactorZ[1]))return false;
        result.Known=true;out=result;return true;
    }

    template<class TPay> bool TryBoundGap(Interval worldY,Interval worldZ,TPay&& pay,Interval& gap) const
    {
        gap={};
        if(!Known||!A::Valid(worldY)||!A::Valid(worldZ)||
            !(Origin.Upper<worldY.Lower&&worldY.Upper<LastY.Lower)||!pay())return false;
        const auto y=A::Subtract(worldY,Origin);
        Interval v;
        if(!BoundPositiveOffset(y,pay,v))return false;
        if(!pay())return false;
        const auto chartZ=A::Multiply(A::Multiply(A::Square(v),v),Evaluate(FactorZ,v));
        gap=A::Subtract(worldZ,chartZ);
        return A::Valid(gap);
    }

    template<class TPay> bool TryBoundParameterRelativeY(Interval y,TPay&& pay,Interval& v) const
    {
        v={};
        const auto span=A::Subtract(LastY,Origin);
        return Known&&A::Valid(y)&&A::Valid(span)&&0.<y.Lower&&y.Upper<span.Lower&&pay()&&
            BoundPositiveOffset(y,pay,v);
    }

    template<class TPay> bool TryBoundGapRelativeY(Interval y,Interval worldZ,TPay&& pay,Interval& gap) const
    {
        gap={};
        Interval v;
        if(!A::Valid(worldZ)||!TryBoundParameterRelativeY(y,pay,v)||!pay())return false;
        const auto chartZ=A::Multiply(A::Multiply(A::Square(v),v),Evaluate(FactorZ,v));
        gap=A::Subtract(worldZ,chartZ);
        return A::Valid(gap);
    }
};
}

namespace Speed::Analytic::Certified::Case202SplitReference
{
namespace A=::AnalyticCertified;
using A::Interval;
using V3=Interval[3];
using Q4=Interval[4];

struct FInput
{
    double Position[3],Velocity[3],Acceleration[3],AngularVelocity[3],AngularAcceleration[3];
    double Rotation[4],LocalCOM[3],LocalOffset[3],HalfExtent[3];
};

struct FBounds
{
    V3 Raw,Normalized;
};

// Error-free two-sum, with each operation explicitly rounded in binary64.
// Callers require the checked nearest numeric contract and the range guards
// below. The nonzero limbs of this scoped capture are all normal.
inline bool TwoSum(double a,double b,double& sum,double& residual)
{
    volatile double s=a+b;
    volatile double bPart=s-a;
    volatile double aPart=s-bPart;
    volatile double aError=a-aPart;
    volatile double bError=b-bPart;
    volatile double error=aError+bError;
    sum=s;residual=error;
    return A::Finite(sum)&&A::Finite(residual);
}

inline bool NormalOrZero(double value)
{
    return value==0.||std::isnormal(value);
}

inline bool SupportedOperand(double value)
{
    const double magnitude=std::abs(value);
    return magnitude==0.||(magnitude>=0x1p-100&&magnitude<=0x1p20);
}

inline Interval ExactSum(const double (&terms)[4])
{
    double sum=0.;
    Interval errors=Interval::Exact(0.);
    for(double term:terms)
    {
        double next=0.,error=0.;
        if(!SupportedOperand(term)||!TwoSum(sum,term,next,error)||!NormalOrZero(error))return {};
        errors=A::Add(errors,Interval::Exact(error));
        sum=next;
    }
    return A::Add(Interval::Exact(sum),errors);
}

inline Interval NormMinusOne(const double (&q)[4])
{
    // Guard the inverted near-horizontal case. The products of X,Y and
    // their exact fused residuals cannot approach the subnormal range.
    if(!(0.5<=std::abs(q[0])&&std::abs(q[0])<=1.&&0.5<=std::abs(q[1])&&std::abs(q[1])<=1.)||
        !SupportedOperand(q[2])||!SupportedOperand(q[3])||
        std::abs(q[2])>0x1p-24||std::abs(q[3])>0x1p-24)return {};
    volatile double xx=q[0]*q[0],yy=q[1]*q[1];
    const double ex=std::fma(q[0],q[0],-xx),ey=std::fma(q[1],q[1],-yy);
    double main=0.,error=0.;
    if(!TwoSum(xx,yy,main,error)||main<0.5||main>2.||
        !NormalOrZero(ex)||!NormalOrZero(ey)||!NormalOrZero(error))return {};
    const auto tiny=A::Add(A::Square(Interval::Exact(q[2])),A::Square(Interval::Exact(q[3])));
    // Sterbenz makes main-1 exact under the guarded ratio.
    const auto part=A::Add(Interval::Exact(main-1.),Interval::Exact(error));
    return A::Add(A::Add(part,Interval::Exact(ex)),A::Add(Interval::Exact(ey),tiny));
}

inline void Cross(const V3& a,const V3& b,V3& out)
{
    for(int d=0;d<3;++d)
    {
        const int i=(d+1)%3,j=(d+2)%3;
        out[d]=A::Subtract(A::Multiply(a[i],b[j]),A::Multiply(a[j],b[i]));
    }
}

inline bool TryBound(const FInput& in,Interval seconds,int corner,FBounds& out)
{
    out={};
    if(corner<0||corner>=8||!A::Valid(seconds)||seconds.Lower<0.||seconds.Upper>0.005)return false;
    const auto normDelta=NormMinusOne(in.Rotation);
    if(!A::Valid(normDelta)||normDelta.Lower < -0x1p-40||normDelta.Upper > 0x1p-40)return false;
    const auto one=Interval::Exact(1.),two=Interval::Exact(2.);
    const auto norm=A::Add(one,normDelta);
    const auto time2=A::Square(seconds);
    V3 s,displacement,theta;
    double vertex[3];
    for(int d=0;d<3;++d)
    {
        for(double operand:{in.Position[d],in.Velocity[d],in.Acceleration[d],in.AngularVelocity[d],
            in.AngularAcceleration[d],in.LocalCOM[d],in.LocalOffset[d],in.HalfExtent[d]})
            if(!SupportedOperand(operand))return false;
        if(!(in.HalfExtent[d]>0.)||std::abs(in.AngularVelocity[d])>0x1p-24||
            std::abs(in.AngularAcceleration[d])>0x1p-24)return false;
        vertex[d]=(corner&(1<<d))?in.HalfExtent[d]:-in.HalfExtent[d];
        s[d]=A::Add(A::Subtract(Interval::Exact(in.LocalOffset[d]),Interval::Exact(in.LocalCOM[d])),
            Interval::Exact(vertex[d]));
        theta[d]=A::Add(A::Multiply(Interval::Exact(in.AngularVelocity[d]),seconds),
            A::Multiply(Interval::Exact(0.5*in.AngularAcceleration[d]),time2));
        displacement[d]=A::Add(A::Multiply(Interval::Exact(in.Velocity[d]),seconds),
            A::Multiply(Interval::Exact(0.5*in.Acceleration[d]),time2));
    }
    auto x=A::Add(A::Add(A::Square(theta[0]),A::Square(theta[1])),A::Square(theta[2]));
    if(!A::Valid(x)||x.Upper<0.||x.Upper>1.)return false;
    if(x.Lower<0.)x.Lower=0.; // Squared norm is nonnegative.
    const auto series=[&](double limit,double first,double second)
    {
        const auto linear=A::Subtract(Interval::Exact(limit),FactoredLowerCapGraphGap::DivideByPositive(x,Interval::Exact(first)));
        const auto quadratic=FactoredLowerCapGraphGap::DivideByPositive(A::Square(x),Interval::Exact(second));
        const auto upper=A::Add(linear,quadratic);
        return A::Valid(linear)&&A::Valid(upper)?Interval{linear.Lower,upper.Upper,true}:Interval{};
    };
    const auto scale=series(0.5,48.,3840.),cosine=series(1.,8.,384.);
    V3 expV,qV,cross;
    for(int d=0;d<3;++d){expV[d]=A::Multiply(theta[d],scale);qV[d]=Interval::Exact(in.Rotation[d]);}
    const auto qw=Interval::Exact(in.Rotation[3]);
    Cross(expV,qV,cross);
    V3 v;
    auto w=A::Multiply(cosine,qw);
    for(int d=0;d<3;++d)
    {
        v[d]=A::Add(A::Add(A::Multiply(cosine,qV[d]),A::Multiply(qw,expV[d])),cross[d]);
        w=A::Subtract(w,A::Multiply(expV[d],qV[d]));
    }
    V3 cross1,cross2;
    Cross(v,s,cross1);
    Cross(v,cross1,cross2);
    const auto twoOverNorm=FactoredLowerCapGraphGap::DivideByPositive(two,norm);
    for(int d=0;d<2;++d)
    {
        const auto base=A::Add(A::Add(Interval::Exact(in.Position[d]),s[d]),displacement[d]);
        const auto k=A::Add(A::Multiply(w,cross1[d]),cross2[d]);
        out.Raw[d]=A::Add(base,A::Multiply(two,k));
        out.Normalized[d]=A::Add(base,A::Multiply(twoOverNorm,k));
    }
    // Exploit ||exp(theta/2)Q||² = ||Q||² without treating the normalized
    // and raw rotations as the same geometry. Only small terms enter K.
    const auto zBase=A::Add(ExactSum({in.Position[2],-in.LocalOffset[2],in.LocalCOM[2],-vertex[2]}),displacement[2]);
    const auto axial=A::Multiply(A::Add(A::Square(v[2]),A::Square(w)),s[2]);
    const auto horizontal=A::Multiply(v[2],A::Add(A::Multiply(v[0],s[0]),A::Multiply(v[1],s[1])));
    const auto k=A::Add(A::Add(axial,horizontal),A::Multiply(w,cross1[2]));
    const auto rawBase=A::Subtract(zBase,A::Multiply(two,A::Multiply(normDelta,s[2])));
    out.Raw[2]=A::Add(rawBase,A::Multiply(two,k));
    out.Normalized[2]=A::Add(zBase,A::Multiply(twoOverNorm,k));
    for(int d=0;d<3;++d)if(!A::Valid(out.Raw[d])||!A::Valid(out.Normalized[d]))return false;
    return true;
}
}

namespace Speed::Analytic::Certified::Case202SplitReference
{
struct FRelativeYSum
{
    double High=0.;
    Interval Error=Interval::Exact(0.);
    bool AddExact(double x)
    {
        double hi,lo;
        if(!TwoSum(High,x,hi,lo)||!NormalOrZero(lo))return false;
        Error=A::Add(Error,Interval::Exact(lo));High=hi;
        return A::Valid(Error);
    }
    bool AddTriple(double a,double b,double c,double sign)
    {
        volatile double hi=a*b;
        const double lo=std::fma(a,b,-hi);
        for(double part:{double(hi),lo})
        {
            volatile double main=part*c;
            const double rem=std::fma(part,c,-main);
            if(!NormalOrZero(main)||!NormalOrZero(rem)||!AddExact(sign*main)||!AddExact(sign*rem))return false;
        }
        return true;
    }
    Interval Bound() const {return A::Add(Interval::Exact(High),Error);}
};

inline bool TryBoundNormalizedRelativeY(const FInput& in,Interval seconds,int corner,double origin,Interval& out)
{
    using G=FactoredLowerCapGraphGap::FProfile;
    namespace FG=FactoredLowerCapGraphGap;
    out={};
    if(corner<0||corner>=8||!SupportedOperand(origin)||!A::Valid(seconds)||seconds.Lower<0.||
        seconds.Upper>1.e-8)return false;
    for(double q:in.Rotation)if(!SupportedOperand(q))return false;
    FRelativeYSum base,k0;
    if(!base.AddExact(in.Position[1])||!base.AddExact(-origin))return false;
    V3 offset,theta;
    const auto time2=A::Square(seconds);
    for(int d=0;d<3;++d)
    {
        if(in.AngularAcceleration[d]!=0.||!SupportedOperand(in.AngularVelocity[d])||
            !SupportedOperand(in.LocalOffset[d])||!SupportedOperand(in.LocalCOM[d])||
            !SupportedOperand(in.HalfExtent[d])||!(in.HalfExtent[d]>0.))return false;
        const double vertex=(corner&(1<<d))?in.HalfExtent[d]:-in.HalfExtent[d];
        const double parts[]={in.LocalOffset[d],-in.LocalCOM[d],vertex};
        offset[d]=A::Add(A::Add(Interval::Exact(parts[0]),Interval::Exact(parts[1])),Interval::Exact(parts[2]));
        theta[d]=A::Multiply(Interval::Exact(in.AngularVelocity[d]),seconds);
        for(double s:parts)
        {
            if(d==1&&!base.AddExact(s))return false;
            // K_y = qw*qz*sx - qw*qx*sz + qy*qx*sx + qy*qz*sz - (qx^2+qz^2)*sy.
            const auto both=[&](int a,int b,double sign)
            {return k0.AddTriple(in.Rotation[a],in.Rotation[b],s,sign)&&
                    base.AddTriple(in.Rotation[a],in.Rotation[b],s,2.*sign);};
            if(d==0&&(!both(3,2,1.)||!both(1,0,1.)))return false;
            if(d==1&&(!both(0,0,-1.)||!both(2,2,-1.)))return false;
            if(d==2&&(!both(3,0,-1.)||!both(1,2,1.)))return false;
        }
    }
    auto angle2=A::Add(A::Add(A::Square(theta[0]),A::Square(theta[1])),A::Square(theta[2]));
    if(!A::Valid(angle2)||angle2.Upper>1.)return false;
    angle2.Lower=(std::max)(angle2.Lower,0.);
    const auto scaled=[&](double a,double b,double leading)
    {
        const auto low=A::Subtract(Interval::Exact(leading),FG::DivideByPositive(angle2,Interval::Exact(a)));
        const auto up=A::Add(low,FG::DivideByPositive(A::Square(angle2),Interval::Exact(b)));
        return A::Valid(low)&&A::Valid(up)?Interval{low.Lower,up.Upper,true}:Interval{};
    };
    const auto scale=scaled(48.,3840.,.5);
    const auto cosMinusOne=scaled(8.,384.,0.);
    V3 expV,qv,cross;
    for(int d=0;d<3;++d){expV[d]=A::Multiply(theta[d],scale);qv[d]=Interval::Exact(in.Rotation[d]);}
    Cross(expV,qv,cross);
    Q4 dq;
    const auto qw=Interval::Exact(in.Rotation[3]);
    dq[3]=A::Multiply(cosMinusOne,qw);
    for(int d=0;d<3;++d)
    {
        dq[d]=A::Add(A::Add(A::Multiply(cosMinusOne,qv[d]),A::Multiply(expV[d],qw)),cross[d]);
        dq[3]=A::Subtract(dq[3],A::Multiply(expV[d],qv[d]));
    }
    const auto deltaProduct=[&](int a,int b)
    {
        return A::Add(A::Add(A::Multiply(Interval::Exact(in.Rotation[a]),dq[b]),
                             A::Multiply(dq[a],Interval::Exact(in.Rotation[b]))),A::Multiply(dq[a],dq[b]));
    };
    const auto x=A::Multiply(A::Add(deltaProduct(3,2),deltaProduct(1,0)),offset[0]);
    const auto y=A::Multiply(A::Add(deltaProduct(0,0),deltaProduct(2,2)),offset[1]);
    const auto z=A::Multiply(A::Subtract(deltaProduct(1,2),deltaProduct(3,0)),offset[2]);
    const auto dk=A::Add(A::Subtract(x,y),z);
    const auto normDelta=NormMinusOne(in.Rotation);
    const auto norm=A::Add(Interval::Exact(1.),normDelta);
    const auto correction=FG::DivideByPositive(A::Multiply(A::Add(k0.Bound(),dk),normDelta),norm);
    const auto rotation=A::Multiply(Interval::Exact(2.),A::Subtract(dk,correction));
    const auto displacement=A::Add(A::Multiply(Interval::Exact(in.Velocity[1]),seconds),
                                    A::Multiply(Interval::Exact(.5*in.Acceleration[1]),time2));
    out=A::Add(base.Bound(),A::Add(rotation,displacement));
    return A::Valid(out);
}
}

namespace Speed::Analytic::Certified::BoxGraphFirstArrival
{
namespace A=::AnalyticCertified;
using AffineReferenceDetail::FProduct;
using GeneralPhysicalFloorClearDetail::FCell;

enum class EResult : uint8 { NotApplicable, Unproved, Work, Arithmetic, FirstAdjacent, NativeFirstSeparateReferences, Serial3NativeFirst, ReferenceFirst205 };
constexpr int32 MaxInverseDepth=42;
constexpr int32 MaxTimeCells=128;
constexpr int32 MaxFocusedOrdinaryTimeCells=256;
constexpr uint16 AllCorners=0xffff;

static TAutoConsoleVariable<int32> CVarMixedFloorCurveTrace(
    TEXT("p.IAmSpeed.AnalyticWorld.MixedFloorCurveTrace"),0,
    TEXT("Trace refused established-floor/curved Box certificates without changing the proof. Default OFF."));
static TAutoConsoleVariable<int32> CVarOrdinaryCurvedBoxTrace(
    TEXT("p.IAmSpeed.AnalyticWorld.OrdinaryCurvedBoxTrace"),0,
    TEXT("Trace refused ordinary curved Box certificates without changing the proof. Default OFF."));
static TAutoConsoleVariable<int32> CVarOrdinaryCurvedBoxReferenceSubcells(
    TEXT("p.IAmSpeed.AnalyticWorld.OrdinaryCurvedBoxReferenceSubcells"),0,
    TEXT("Allow sixteen proved reference subcells inside an adjacent ordinary curved Box interval, on the same global work and time-cell accounts. Default OFF."));

static TAutoConsoleVariable<int32> CVarOrdinaryCurvedBoxFocusedTimeCells(
    TEXT("p.IAmSpeed.AnalyticWorld.OrdinaryCurvedBoxFocusedTimeCells"),0,
    TEXT("Allow up to 256 time cells for a focused ordinary curved Box prefix on its unchanged work account. Default off."));

static TAutoConsoleVariable<int32> CVarOrdinaryCurvedBoxAdjacentReferenceChord(
    TEXT("p.IAmSpeed.AnalyticWorld.OrdinaryCurvedBoxAdjacentReferenceChord"),0,
    TEXT("Prove one unresolved continuous reference Box corner between adjacent native times using existing endpoint witnesses. Default off."));

static TAutoConsoleVariable<int32> CVarOrdinaryCurvedBoxAdjacentConsumedSingleton(
    TEXT("p.IAmSpeed.AnalyticWorld.OrdinaryCurvedBoxAdjacentConsumedSingleton"),0,
    TEXT("Refine the consumed square at an ordinary curved Box adjacent inside endpoint. Default off."));

static TAutoConsoleVariable<int32> CVarOrdinaryCurvedBoxRotationCorridorHalves(
    TEXT("p.IAmSpeed.AnalyticWorld.OrdinaryCurvedBoxRotationCorridorHalves"),0,
    TEXT("Retry ordinary curved Box pure-rotation separation on two halves of the original quaternion chord. Default off."));

struct FAccount
{
    FDistanceBudget& Budget;
    uint64 Began;
    int32 Limit;
    bool Stopped=false;
    bool Invalid=false;
    bool Pay()
    {
        if(Budget.Used<Began){Invalid=true;return false;}
        if(Budget.Used-Began>=uint64(Limit)||!Budget.Consume()){Stopped=true;return false;}
        return true;
    }
    EResult Failure() const
    {
        return Invalid?EResult::Arithmetic:Stopped?EResult::Work:EResult::Unproved;
    }
};

struct FYZ
{
    Interval Y,Z;
    bool Valid() const {return A::Valid(Y)&&A::Valid(Z);}
};

struct FGraph
{
    Interval X0,X1;
    FYZ P[6],D[5];
    bool AdjacentBandSeam=false,NominalInverseProposal=false;
    static constexpr double LeftSlope=167./256.;
    struct FAboveWitness {double YLimit=0.,ZFloor=0.;bool Known=false;};

    static bool Make(const FProduct& p,FAccount& work,FGraph& result,
        bool mixedConvexity=false,bool adjacentBandSeam=false)
    {
        if(p.FreeWorldAxis!=0||!p.FreeAlongU||p.Curve.DegreeU!=0||p.Curve.DegreeV!=5||
            p.Curve.ControlPoints.Num()!=6||!A::Valid(p.Start)||!A::Valid(p.End)||
            !(p.Start.Upper<p.End.Lower))return false;
        result.X0=p.Start;result.X1=p.End;
        for(int32 i=0;i<6;++i)
        {
            if(!work.Pay())return false;
            const auto& c=p.Curve.ControlPoints[i];
            if(!Detail::ValidPoint(c)||c.Axis[0].Lower!=0.||c.Axis[0].Upper!=0.)return false;
            result.P[i]={c.Axis[1],c.Axis[2]};
        }
        const auto exact=[](Interval value)
        {
            return A::Valid(value)&&A::Bits(value.Lower)==A::Bits(value.Upper);
        };
        const auto scaledDifference=[&](Interval a,Interval b,double scale)
        {
            // Subtracting equal exact controls is exactly zero, including
            // the seam's repeated Z=0 and its repeated first derivatives.
            if(mixedConvexity&&exact(a)&&exact(b)&&a.Lower==b.Lower)return Interval::Exact(0.);
            return A::Multiply(Interval::Exact(scale),A::Subtract(a,b));
        };
        const auto diff=[&](FYZ a,FYZ b,double scale)
        {
            return FYZ{scaledDifference(a.Y,b.Y,scale),scaledDifference(a.Z,b.Z,scale)};
        };
        for(int32 i=0;i<5;++i)
        {
            if(!work.Pay())return false;
            const auto d=diff(result.P[i+1],result.P[i],5.);
            if(!d.Valid()||!(d.Y.Lower>0.)||d.Z.Lower<0.)return false;
            result.D[i]=d;
        }
        if(adjacentBandSeam)
        {
            // The slope is a proposal. Prove the convex continuation at
            // the left edge using the complete original derivative net.
            for(const auto& derivative:result.D)
            {
                if(!work.Pay())return false;
                const auto excess=A::Subtract(derivative.Z,
                    A::Multiply(Interval::Exact(LeftSlope),derivative.Y));
                if(!A::Valid(excess)||!(excess.Lower>0.))return false;
            }
            result.AdjacentBandSeam=true;
        }
        FYZ second[4];
        for(int32 i=0;i<4;++i)
        {
            if(!work.Pay())return false;
            const auto dd=diff(result.D[i+1],result.D[i],4.);
            if(!dd.Valid()||(!mixedConvexity&&(dd.Y.Upper>0.||dd.Z.Lower<0.)))return false;
            second[i]=dd;
        }
        if(mixedConvexity)
        {
            // Convexity of Z as a function of Y requires Y'Z''-Z'Y'' >= 0.
            // Multiply the original degree-4/3 derivative nets in Bernstein
            // form. For coefficient k, the common divisor choose(7,k) is
            // positive, so only the sign of the directed numerator matters.
            constexpr int32 firstWeight[]={1,4,6,4,1};
            constexpr int32 secondWeight[]={1,3,3,1};
            const auto zero=[&](Interval value){return exact(value)&&value.Lower==0.;};
            const auto product=[&](Interval a,Interval b)
            {return zero(a)||zero(b)?Interval::Exact(0.):A::Multiply(a,b);};
            for(int32 k=0;k<8;++k)
            {
                Interval numerator=Interval::Exact(0.);
                for(int32 i=FMath::Max(0,k-3);i<=FMath::Min(4,k);++i)
                {
                    if(!work.Pay())return false;
                    const int32 j=k-i;
                    const auto term=scaledDifference(product(result.D[i].Y,second[j].Z),
                        product(result.D[i].Z,second[j].Y),double(firstWeight[i]*secondWeight[j]));
                    if(!A::Valid(term))return false;
                    if(!zero(term))numerator=zero(numerator)?term:A::Add(numerator,term);
                }
                if(!A::Valid(numerator)||numerator.Lower<0.)return false;
            }
        }
        return true;
    }

    bool SameProduct(const FProduct& p) const
    {
        const auto same=[](Interval a,Interval b)
        {return A::Valid(a)&&A::Valid(b)&&A::Bits(a.Lower)==A::Bits(b.Lower)&&A::Bits(a.Upper)==A::Bits(b.Upper);};
        if(p.FreeWorldAxis!=0||!p.FreeAlongU||p.Curve.DegreeU!=0||p.Curve.DegreeV!=5||
            p.Curve.ControlPoints.Num()!=6||!same(X0,p.Start)||!same(X1,p.End))return false;
        for(int32 i=0;i<6;++i)
        {
            const auto& c=p.Curve.ControlPoints[i];
            if(!Detail::ValidPoint(c)||c.Axis[0].Lower!=0.||c.Axis[0].Upper!=0.||
                !same(P[i].Y,c.Axis[1])||!same(P[i].Z,c.Axis[2]))return false;
        }
        return true;
    }

    template <int32 N>
    static bool EvaluatePaid(const FYZ (&net)[N],double t,FAccount& work,FYZ& out)
    {
        if(t==0.||t==1.){out=net[t==0.?0:N-1];return true;}
        const auto a=A::Interval::Exact(t),b=A::Subtract(A::Interval::Exact(1.),a);
        FYZ row[N];
        for(int32 i=0;i<N;++i)row[i]=net[i];
        for(int32 n=N-1;n>0;--n)for(int32 i=0;i<n;++i)
        {
            row[i].Y=A::Add(A::Multiply(b,row[i].Y),A::Multiply(a,row[i+1].Y));
            row[i].Z=A::Add(A::Multiply(b,row[i].Z),A::Multiply(a,row[i+1].Z));
        }
        out=row[0];
        if(!out.Valid()){work.Invalid=true;return false;}
        return true;
    }

    template <int32 N>
    static bool Evaluate(const FYZ (&net)[N],double t,FAccount& work,FYZ& out)
    {
        // One logical bounded Bernstein evaluation, including on memo hits.
        return work.Pay()&&EvaluatePaid(net,t,work,out);
    }

    class FPositionMemo final
    {
        const FYZ (&Net)[6];
        bool Enabled;
        uint64 Present=0;
        struct FBounds { double YLower,YUpper,ZLower,ZUpper; };
        FBounds Values[64]; // Read only after its Present bit is set.

        static int32 Slot(double t)
        {
            // Recognize the exact internal grid k/64, 1 <= k <= 63, by bits.
            // These are the first six levels shared by all [0,1] inversions.
            // Integer-only selection introduces no FP conversion or flags.
            const uint64 bits=A::Bits(t);
            if(bits&A::Sign)return INDEX_NONE;
            const int32 exponent=int32((bits>>52)&0x7ff);
            if(exponent<1017||exponent>1022)return INDEX_NONE;
            const uint64 significand=(bits&0x000fffffffffffffULL)|(1ULL<<52);
            const int32 shift=1069-exponent;
            if(significand&((1ULL<<shift)-1))return INDEX_NONE;
            return int32(significand>>shift);
        }

    public:
        FPositionMemo(const FGraph& graph,bool enabled):Net(graph.P),Enabled(enabled){}
        bool At(double t,FAccount& work,FYZ& out)
        {
            if(!Enabled)return FGraph::Evaluate(Net,t,work,out);
            if(!work.Pay())return false;
            const int32 slot=Slot(t);
            if(slot==INDEX_NONE)return FGraph::EvaluatePaid(Net,t,work,out);
            const uint64 bit=1ULL<<slot;
            if(Present&bit)
            {
                const auto& cached=Values[slot];
                out={{cached.YLower,cached.YUpper,true},{cached.ZLower,cached.ZUpper,true}};
                return true;
            }
            if(!FGraph::EvaluatePaid(Net,t,work,out))return false;
            Values[slot]={out.Y.Lower,out.Y.Upper,out.Z.Lower,out.Z.Upper};
            Present|=bit;
            return true;
        }
    };

    bool InOriginalPatch(const FIntervalPoint& point) const
    {
        return X0.Upper<point.Axis[0].Lower&&point.Axis[0].Upper<X1.Lower&&
            P[0].Y.Upper<point.Axis[1].Lower&&point.Axis[1].Upper<P[5].Y.Lower;
    }

    template<int32 N>
    static double EvaluateNominalY(const double (&controls)[N],double v)
    {
        double row[N];
        for(int32 i=0;i<N;++i)row[i]=controls[i];
        for(int32 n=N-1;n>0;--n)
            for(int32 i=0;i<n;++i)row[i]=(1.-v)*row[i]+v*row[i+1];
        return row[0];
    }

    bool ProposeInverseY(double target,FAccount& work,double& v) const
    {
        const double origin=.5*P[0].Y.Lower+.5*P[0].Y.Upper;
        double y[6],dy[5];
        for(int32 i=0;i<6;++i)y[i]=.5*P[i].Y.Lower+.5*P[i].Y.Upper-origin;
        for(int32 i=0;i<5;++i)dy[i]=5.*(y[i+1]-y[i]);
        if(!A::Finite(y[5])||!(y[5]>0.))return false;
        const double relative=target-origin;
        v=relative/y[5];
        // Numerical proposals only. Nothing here is an inverse bound or
        // authorizes a sign; the original directed net is evaluated below.
        for(int32 iteration=0;iteration<4;++iteration)
        {
            if(!(0.<v&&v<1.)||!work.Pay())return false;
            const double slope=EvaluateNominalY(dy,v);
            if(!A::Finite(slope)||!(slope>0.))return false;
            v-=(EvaluateNominalY(y,v)-relative)/slope;
        }
        return 0.<v&&v<1.;
    }

    bool Sign(const FIntervalPoint& point,bool above,FAccount& work,FPositionMemo& memo,
        FAboveWitness* witness=nullptr) const
    {
        if(witness)*witness={};
        if(!work.Pay())return false;
        const auto rememberAbove=[&](double yLimit,double zFloor)
        {
            if(witness)*witness={FMath::Min(yLimit,P[5].Y.Lower),zFloor,true};
            return true;
        };
        const auto y=point.Axis[1],z=point.Axis[2];
        if(!(y.Upper<P[5].Y.Lower))return false;
        if(above)
        {
            // The constant LEFT extension is convex and agrees with the
            // original profile. It is used solely for exclusion.
            if(y.Upper<=P[0].Y.Lower)
            {
                if(!AdjacentBandSeam)return z.Lower>P[0].Z.Upper&&rememberAbove(P[0].Y.Lower,P[0].Z.Upper);
                // The finite chart starts at P[0]. This virtual extension
                // is used solely for proving separation to its left.
                const auto delta=A::Subtract(Interval::Exact(y.Upper),P[0].Y);
                const auto ceiling=A::Add(P[0].Z,A::Multiply(Interval::Exact(LeftSlope),delta));
                return A::Valid(ceiling)&&z.Lower>ceiling.Upper&&rememberAbove(y.Upper,ceiling.Upper);
            }
            if(z.Lower>P[5].Z.Upper)return rememberAbove(P[5].Y.Lower,P[5].Z.Upper);
            if(z.Lower<=P[0].Z.Upper)return false;
        }
        else if(!(P[0].Y.Upper<y.Lower)||z.Upper>=P[5].Z.Lower)return false;
        else if(z.Upper<P[0].Z.Lower)return true;
        const double target=above?y.Upper:y.Lower;
        if(NominalInverseProposal)
        {
            double proposed=0.;
            if(ProposeInverseY(target,work,proposed))
                for(double offset:{0x1p-40,-0x1p-40,0x1p-43,-0x1p-43})
                {
                    const double candidate=proposed+(above?offset:-offset);
                    if(!(0.<candidate&&candidate<1.))continue;
                    FYZ q;
                    if(!memo.At(candidate,work,q))return false;
                    if(q.Y.Upper<=target)
                    {
                        if(!above&&q.Z.Lower>z.Upper)return true;
                        if(above&&q.Z.Lower>=z.Lower)return false;
                    }
                    else if(q.Y.Lower>=target)
                    {
                        if(above&&q.Z.Upper<z.Lower)return rememberAbove(q.Y.Lower,q.Z.Upper);
                        if(!above&&q.Z.Upper<=z.Upper)return false;
                    }
                }
            if(work.Stopped||work.Invalid)return false;
        }
        double lo=0.,hi=1.;
        for(int32 depth=0;depth<MaxInverseDepth;++depth)
        {
            const double t=.5*lo+.5*hi;
            FYZ q;
            if(!memo.At(t,work,q))return false;
            if(q.Y.Upper<=target)
            {
                if(!above&&q.Z.Lower>z.Upper)return true;
                // The lower-Y sample already reaches this Z. No later
                // upper-Y sample can establish this strict above bound.
                if(above&&q.Z.Lower>=z.Lower)return false;
                lo=t;
            }
            else if(q.Y.Lower>=target)
            {
                if(above&&q.Z.Upper<z.Lower)return rememberAbove(q.Y.Lower,q.Z.Upper);
                // The upper-Y sample is already below this Z. No earlier
                // lower-Y sample can establish this strict below bound.
                if(!above&&q.Z.Upper<=z.Upper)return false;
                hi=t;
            }
            else break; // Neither side was proved, never assign the midpoint.
        }
        return false;
    }

    bool BoundParameter(Interval y,FAccount& work,FPositionMemo& memo,Interval& out) const
    {
        if(!(P[0].Y.Upper<y.Lower&&y.Upper<P[5].Y.Lower))return false;
        const auto side=[&](double target,bool lower,double& answer)
        {
            double lo=0.,hi=1.;
            for(int32 depth=0;depth<MaxInverseDepth;++depth)
            {
                const double t=.5*lo+.5*hi;
                FYZ q;
                if(!memo.At(t,work,q))return false;
                if(q.Y.Upper<=target)lo=t;
                else if(q.Y.Lower>=target)hi=t;
                else break;
            }
            answer=lower?lo:hi;
            return true;
        };
        double lo=0.,hi=1.;
        if(!side(y.Lower,true,lo)||!side(y.Upper,false,hi)||!(0.<lo&&lo<=hi&&hi<1.))return false;
        out={lo,hi,true};
        return true;
    }
};

enum class ETraversalProposal : uint8
{
    Default,
    Owner88First,
    Owner88Remainder
};

class FProof final
{
    FGeneralBoxMotionInput Input;
    FGraph Graph;
    uint64 Numeric=0;
    bool ReuseExactEvaluations=false;
    uint32 ClearOrdinal=0,InsideOrdinal=0,SearchOrdinal=0;
    int32 Corner=INDEX_NONE;
    Interval U,V;
    FIntervalPoint SurfacePoint,OutwardRawNormal,Envelope,ClearNativeCenter;
    TArray<FCell,TInlineAllocator<32>> Prefix;
    FProof()=default;

    struct FSplitReferenceProof
    {
        FCell Raw,Normalized;
        uint32 CommonClear=0;
        bool Matches(uint32 clear,uint32 inside) const
        {
            return CommonClear>0&&CommonClear<Raw.First&&Raw.Last==Raw.First+1&&Raw.Last<clear&&
                Normalized.First==clear&&Normalized.Last==inside&&inside==clear+1;
        }
    };
    TOptional<FSplitReferenceProof> SplitReference;

    static bool Split202GraphSign(const FGraph& graph,const FactoredLowerCapGraphGap::FProfile& profile,
        const FIntervalPoint& point,bool above,FAccount& work)
    {
        if(!Detail::ValidPoint(point)||!work.Pay())return false;
        const auto y=point.Axis[1],z=point.Axis[2];
        if(above&&y.Upper<=graph.P[0].Y.Lower)return z.Lower>graph.P[0].Z.Upper;
        if(above&&z.Lower>graph.P[5].Z.Upper&&y.Upper<graph.P[5].Y.Lower)return true;
        Interval gap;
        const auto pay=[&](){return work.Pay();};
        return profile.TryBoundGap(y,z,pay,gap)&&(above?gap.Lower>0.:gap.Upper<0.);
    }

    struct FSerial3Proof
    {
        Case202Serial3::ChartIdentity Chart;
        Case202Serial3::RawInitialInside Raw;
        Case202Serial3::NormalizedAdjacent Normalized;
        Case202Serial3::NativeAdjacent Native;
        TSharedPtr<const Case202Serial3::LoadedModelAttestation,ESPMode::ThreadSafe> Seal;
        Case202Serial3::NativeModelIdentity Live;
    };
    TSharedPtr<const FSerial3Proof,ESPMode::ThreadSafe> Serial3;

// Included in the private FProof body of NativeCanonicalBoxSweepClearanceValues.inl.
#if !UE_BUILD_SHIPPING
    TOptional<Case205ReferenceTiming::FNativePrefixAndLaterWitness> Reference205;

    static bool TryBuildReference205(const FGeneralBoxMotionInput& input,const FProduct& product,
        const Prepared::FReferenceTransport& reference,const FGraph& graph,const FIntervalPoint& envelope,
        uint32 common,uint32 end,FAccount& work,FGraph::FPositionMemo& memo,uint64 numeric,bool reuse,
        TSharedPtr<const FProof,ESPMode::ThreadSafe>& out,const TCHAR*& stage)
    {
        namespace S=Case205ReferenceTiming;
        stage=TEXT("capture");
        const auto ordinal=[](uint32 bits,uint32& value)
        {return FloorNativeSweepDetail::Ordinal(KinematicCenterDetail::FromBits(bits),value);};
        uint32 referenceClear=0,referenceInside=0,expectedCommon=0,nativeInside=0;
        if(!ordinal(S::ReferenceClearBits,referenceClear)||!ordinal(S::ReferenceInsideBits,referenceInside)||
            !ordinal(S::NativeInsideBits,nativeInside)||!ordinal(0x2f343170u,expectedCommon)||
            common!=expectedCommon||referenceClear+1!=referenceInside||nativeInside>end||
            !PreparedExecution::Requested()||!Detail::ValidPoint(envelope))return false;
        const auto pay=[&](){return work.Pay();};
        const auto signOther=[&](const FIntervalPoint& point){return graph.Sign(point,true,work,memo);};
        const auto contains=[&](const FIntervalPoint& point)
        {
            if(!Detail::ValidPoint(point))return false;
            for(int32 d=0;d<3;++d)
                if(point.Axis[d].Lower<envelope.Axis[d].Lower||point.Axis[d].Upper>envelope.Axis[d].Upper)return false;
            return true;
        };
        S::FNativePrefixAndLaterWitness separate;
        if(!S::Prove(input,reference,product,numeric,pay,signOther,contains,separate,stage)||
            !separate.Matches(S::ReferenceClearBits,S::ReferenceInsideBits))return false;

        // Keep the original finite response chart, point, normal and clear
        // center calculation at the REFERENCE interval. The separately
        // proved native edge is never an operand of this response.
        stage=TEXT("response_chart");
        FGeneralBoxMotionBounds current,atClear;
        if(!Bounds(input,referenceClear,referenceInside,work,current)||
            !Bounds(input,referenceClear,referenceClear,work,atClear))return false;
        for(int32 d=0;d<3;++d)
            if(separate.ConsumedCenter.Axis[d].Lower<atClear.NativeCenter.Axis[d].Lower||
                separate.ConsumedCenter.Axis[d].Upper>atClear.NativeCenter.Axis[d].Upper)return false;
        const int32 selected=S::ActiveCorner;
        const auto corner=TimeDetail::Hull(current.Corners[selected],current.Corners[16+selected]);
        Interval u,v;
        if(!graph.InOriginalPatch(corner)||!graph.BoundParameter(corner.Axis[1],work,memo,v)||!work.Pay()||
            !AffineReferenceDetail::FreeParameter(product,corner,u)||!(0.<u.Lower&&u.Upper<1.))return false;
        FYZ lower,upper,dlo,dhi;
        if(!memo.At(v.Lower,work,lower)||!memo.At(v.Upper,work,upper)||
            !FGraph::Evaluate(graph.D,v.Lower,work,dlo)||!FGraph::Evaluate(graph.D,v.Upper,work,dhi))return false;
        const FIntervalPoint point{{corner.Axis[0],{lower.Y.Lower,upper.Y.Upper,true},{lower.Z.Lower,upper.Z.Upper,true}}};
        const FIntervalPoint normal{{A::Interval::Exact(0.),Interval{-dhi.Z.Upper,-dlo.Z.Lower,true},Interval{dhi.Y.Lower,dlo.Y.Upper,true}}};
        if(!Detail::ValidPoint(point)||!Detail::ValidPoint(normal)||!work.Pay()||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)
            return false;
        TSharedPtr<FProof,ESPMode::ThreadSafe> proof(new FProof);
        proof->Input=input;proof->Graph=graph;proof->Numeric=numeric;proof->Graph.NominalInverseProposal=false;
        proof->ReuseExactEvaluations=reuse;proof->ClearNativeCenter=atClear.NativeCenter;
        proof->ClearOrdinal=referenceClear;proof->InsideOrdinal=referenceInside;proof->SearchOrdinal=end;proof->Corner=selected;
        proof->U=u;proof->V=v;proof->SurfacePoint=point;proof->OutwardRawNormal=normal;proof->Envelope=envelope;
        proof->Reference205.Emplace(MoveTemp(separate));
        out=MoveTemp(proof);stage=TEXT("complete");return true;
    }

    bool MatchesReference205() const
    {
        return Reference205.IsSet()&&!SplitReference.IsSet()&&!Serial3&&Corner==Case205ReferenceTiming::ActiveCorner&&
            Prefix.IsEmpty()&&Reference205->Matches(
                KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(ClearOrdinal)),
                KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(InsideOrdinal)));
    }
#endif


public:
#if !UE_BUILD_SHIPPING
    static bool ProveCase205ActiveProjectionCorner(const Analytic::FWorldQuery& query,
        const Analytic::Prepared::FReferenceTransport& reference,const FProduct& product,
        const Analytic::Certified::FIntervalPoint& fixed,const Analytic::Certified::FIntervalPoint& native,
        const Analytic::Certified::FIntervalPoint& unit,Analytic::Certified::BoxGraphFirstArrival::FAccount& work)
    {
        namespace C=Analytic::Certified;
        namespace S=C::Case205ReferenceTiming;
        namespace Q=S::Q;
        const auto pay=[&](){return work.Pay();};
        S::FChart chart;
        const auto y=C::TimeDetail::Hull(C::TimeDetail::Hull(fixed,native),unit).Axis[1];
        if(!S::AuthenticateChart(reference,product,pay,chart)||
            !chart.SetWindow(S::FromInterval(y),.5*y.Lower+.5*y.Upper,pay)||!pay())return false;

        static_assert(S::ActiveCorner==1);
        const FVector local(query.HalfExtent.X,-query.HalfExtent.Y,-query.HalfExtent.Z);
        const auto actual=S::Point(query.Start+query.Rotation.RotateVector(local));
        S::FQuaternion raw{{Q::FromDouble(query.Rotation.X),Q::FromDouble(query.Rotation.Y),
            Q::FromDouble(query.Rotation.Z),Q::FromDouble(query.Rotation.W)}};
        auto normSquared=Q::FromDouble(0.);
        for(const auto& component:raw.Axis)normSquared=Q::Add(normSquared,Q::Square(component));
        const auto norm=Q::PositiveSqrt(normSquared);
        S::FQuaternion normalized;
        for(int32 d=0;d<4;++d)normalized.Axis[d]=Q::DividePositive(raw.Axis[d],norm);
        const auto localPoint=S::Point(local);
        auto original=S::Rotate(raw,localPoint),referenceUnit=S::Rotate(normalized,localPoint);
        Analytic::Private::FBoxSweepContext sat(query);
        auto cachedAxes=S::Point(query.Start);
        for(int32 d=0;d<3;++d)
        {
            const auto center=Q::FromDouble(query.Start[d]);
            original.Axis[d]=Q::Add(center,original.Axis[d]);
            referenceUnit.Axis[d]=Q::Add(center,referenceUnit.Axis[d]);
            for(int32 axis=0;axis<3;++axis)
                cachedAxes.Axis[d]=Q::Add(cachedAxes.Axis[d],
                    Q::Multiply(Q::FromDouble(sat.Axis[axis][d]),localPoint.Axis[axis]));
        }
        const auto proveCorner=[&](const S::FPoint& p,const C::FIntervalPoint& retained)
        {
            if(!work.Pay()||!S::InOriginalChart(chart,product,p))return false;
            for(int32 d=0;d<3;++d)
            {
                const auto bound=S::FromInterval(retained.Axis[d]);
                if(!Q::Valid(bound)||!Q::Valid(p.Axis[d])||Q::Compare(p.Axis[d].Lower,bound.Lower)<0||
                    Q::Compare(p.Axis[d].Upper,bound.Upper)>0)return false;
            }
            return S::StrictPositive(chart.Gap(p.Axis[1],p.Axis[2]));
        };
        return proveCorner(actual,native)&&proveCorner(original,fixed)&&proveCorner(referenceUnit,unit)&&proveCorner(cachedAxes,fixed);
    }
#endif
    FORCENOINLINE static bool TryBuildSerial3(const FGeneralBoxMotionInput& input,const FProduct& product,
        const FIntervalPoint& envelope,FDistanceBudget& budget,int32 maximumWork,bool reuse,
        const Case202Serial3::ChartIdentity& chart,
        const TSharedPtr<const Case202Serial3::LoadedModelAttestation,ESPMode::ThreadSafe>& seal,
        const Case202Serial3::NativeModelIdentity& live,TSharedPtr<const FProof,ESPMode::ThreadSafe>& out,
        const TCHAR*& stage)
    {
        namespace R=Case202SplitReference;
        namespace G=FactoredLowerCapGraphGap;
        namespace S=Case202Serial3;
        constexpr int32 corner=5,opposite=corner^4;
        stage=TEXT("attestation");
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        if(!seal||!seal->Matches(live)||live.NumericControl!=numeric||live.CosineReturnOneBelow!=0x1p-27||
            numeric!=0x0000027f00001f80ull||!PreparedExecution::Requested()||chart.ImmutableProduct!=&product||
            input.LocalRotation.X!=0.||input.LocalRotation.Y!=0.||input.LocalRotation.Z!=0.||input.LocalRotation.W!=1.||
            !Detail::ValidPoint(envelope)||maximumWork<=0)return false;

        const auto ordinal=[](uint32 bits,uint32& result)
        {return FloorNativeSweepDetail::Ordinal(KinematicCenterDetail::FromBits(bits),result);};
        const auto seconds=[](uint32 value){return FloorNativeSweepDetail::Seconds(value);};
        uint32 nativeClear=0,nativeInside=0,unitClear=0,unitInside=0,originalH=0;
        stage=TEXT("proposal");
        if(!ordinal(0x2f21df0bu,nativeClear)||!ordinal(0x2f21df0cu,nativeInside)||
            !ordinal(0x30e43e57u,unitClear)||!ordinal(0x30e43e58u,unitInside)||
            !FloorNativeSweepDetail::Ordinal(input.Horizon,originalH)||
            !(nativeInside==nativeClear+1&&nativeInside<unitClear&&unitInside==unitClear+1&&unitInside<=originalH))return false;

        FAccount work{budget,budget.Used,maximumWork};
        const auto pay=[&](){return work.Pay();};
        FGraph graph;
        stage=TEXT("chart");
        if(!FGraph::Make(product,work,graph,true))return false;
        G::FControl controls[6];
        for(int32 i=0;i<6;++i)controls[i]={graph.P[i].Y,graph.P[i].Z};
        G::FProfile profile;
        if(!G::FProfile::TryBuild(controls,pay,profile))return false;
        const auto gap=[&](const FIntervalPoint& point,Interval& out)
        {
            if(!Detail::ValidPoint(point)||!work.Pay())return false;
            if(point.Axis[1].Upper<=graph.P[0].Y.Lower)
            {
                out=A::Subtract(point.Axis[2],graph.P[0].Z);
                return A::Valid(out);
            }
            return profile.TryBoundGap(point.Axis[1],point.Axis[2],pay,out);
        };
        const auto copy=[](const Interval& in){return S::Range{in.Lower,in.Upper};};
        const auto point=[](const R::V3& in){return FIntervalPoint{{in[0],in[1],in[2]}};};
        const auto cellPoint=[](const S::V3& in)
        {return FIntervalPoint{{Interval{in[0].Lo,in[0].Hi,true},Interval{in[1].Lo,in[1].Hi,true},Interval{in[2].Lo,in[2].Hi,true}}};};

        R::FInput reference{};
        S::Input native{};
        for(int32 d=0;d<3;++d)
        {
            native.Position[d]=reference.Position[d]=input.Owner.Location[d];
            native.Velocity[d]=reference.Velocity[d]=input.Owner.Velocity[d];
            native.Acceleration[d]=reference.Acceleration[d]=input.Owner.Acceleration[d];
            native.Omega[d]=reference.AngularVelocity[d]=input.Owner.AngularVelocity[d];
            native.Alpha[d]=reference.AngularAcceleration[d]=input.Owner.AngularAcceleration[d];
            native.COM[d]=reference.LocalCOM[d]=input.LocalCOM[d];
            native.Offset[d]=reference.LocalOffset[d]=input.LocalOffset[d];
            native.HalfExtent[d]=reference.HalfExtent[d]=input.PhysicalHalfExtent[d];
        }
        const auto& q=input.Owner.Rotation;
        native.Q={q.X,q.Y,q.Z,q.W};
        for(int32 d=0;d<4;++d)reference.Rotation[d]=native.Q[d];

        TSharedPtr<FSerial3Proof,ESPMode::ThreadSafe> evidence(new FSerial3Proof);
        evidence->Chart=chart;evidence->Seal=seal;evidence->Live=live;
        evidence->Raw.Chart=chart;evidence->Raw.Corner=corner;evidence->Raw.Opposite=opposite;
        evidence->Normalized.Chart=chart;evidence->Normalized.Corner=corner;
        evidence->Normalized.Clear=unitClear;evidence->Normalized.Inside=unitInside;
        evidence->Native.Chart=chart;evidence->Native.Corner=corner;
        evidence->Native.Clear=nativeClear;evidence->Native.Inside=nativeInside;

        stage=TEXT("raw_initial_inside");
        for(int32 k:{corner,opposite})
        {
            R::FBounds bounds;Interval signedGap;
            if(!work.Pay()||!R::TryBound(reference,Interval::Exact(0.),k,bounds)||
                !graph.InOriginalPatch(point(bounds.Raw))||!gap(point(bounds.Raw),signedGap))return false;
            (k==corner?evidence->Raw.InsideGap:evidence->Raw.OppositeGap)=copy(signedGap);
        }

        stage=TEXT("normalized_monotone");
        if(!A::Valid(graph.P[0].Y)||graph.P[0].Y.Lower!=graph.P[0].Y.Upper)return false;
        const double origin=graph.P[0].Y.Lower;
        const Interval duration{0.,double(seconds(unitInside)),true};
        const auto duration2=A::Square(duration);
        R::FBounds whole;Interval relativeY,parameter;
        if(!work.Pay()||!R::TryBound(reference,duration,corner,whole)||!graph.InOriginalPatch(point(whole.Normalized))||
            !R::TryBoundNormalizedRelativeY(reference,duration,corner,origin,relativeY)||
            !profile.TryBoundParameterRelativeY(relativeY,pay,parameter))return false;
        Interval dy[5],dz[5];
        for(int32 i=0;i<5;++i){dy[i]=graph.D[i].Y;dz[i]=graph.D[i].Z;}
        const auto slope=G::DivideByPositive(G::Evaluate(dz,parameter),G::Evaluate(dy,parameter));
        R::V3 offset,omega,cross,velocity;
        for(int32 d=0;d<3;++d)
        {
            const auto p=A::Add(A::Add(Interval::Exact(reference.Position[d]),
                A::Multiply(Interval::Exact(reference.Velocity[d]),duration)),
                A::Multiply(Interval::Exact(.5*reference.Acceleration[d]),duration2));
            offset[d]=A::Subtract(whole.Normalized[d],p);
            omega[d]=Interval::Exact(reference.AngularVelocity[d]);
        }
        R::Cross(omega,offset,cross);
        for(int32 d=0;d<3;++d)
            velocity[d]=A::Add(A::Add(Interval::Exact(reference.Velocity[d]),
                A::Multiply(Interval::Exact(reference.Acceleration[d]),duration)),cross[d]);
        const auto derivative=A::Subtract(velocity[2],A::Multiply(slope,velocity[1]));
        if(!A::Valid(derivative)||derivative.Upper>=0.)return false;
        evidence->Normalized.GapDerivative=copy(derivative);
        for(int32 k=0;k<8;++k)
        {
            if(k==corner)continue;
            R::FBounds bounds;Interval signedGap;
            if(!work.Pay()||!R::TryBound(reference,duration,k,bounds)||!gap(point(bounds.Normalized),signedGap))return false;
            evidence->Normalized.OtherGaps[k]=copy(signedGap);
        }
        for(uint32 time:{unitClear,unitInside})
        {
            const auto at=Interval::Exact(seconds(time));
            R::FBounds bounds;Interval y,signedGap;
            if(!work.Pay()||!R::TryBound(reference,at,corner,bounds)||!graph.InOriginalPatch(point(bounds.Normalized))||
                !R::TryBoundNormalizedRelativeY(reference,at,corner,origin,y)||
                !profile.TryBoundGapRelativeY(y,bounds.Normalized[2],pay,signedGap))return false;
            (time==unitClear?evidence->Normalized.LastClearGap:evidence->Normalized.FirstInsideGap)=copy(signedGap);
        }

        const auto contains=[&](const FIntervalPoint& point)
        {
            if(!Detail::ValidPoint(point))return false;
            for(int32 d=0;d<3;++d)if(point.Axis[d].Lower<envelope.Axis[d].Lower||point.Axis[d].Upper>envelope.Axis[d].Upper)return false;
            return true;
        };
        // Bound both compiled x64 dot orders. Only the current live binary
        // attestation grants use of this reviewed rounded DAG.
        const auto boundCell=[&](uint32 first,uint32 last,S::BoundedPose (&out)[2])
        {
            return work.Pay()&&S::BoundReviewedCell(native,seconds(first),seconds(last),false,live.CosineReturnOneBelow,out[0])&&
                S::BoundReviewedCell(native,seconds(first),seconds(last),true,live.CosineReturnOneBelow,out[1]);
        };
        TArray<FCell,TInlineAllocator<32>> prefix,pending;
        pending.Add(FCell{0,nativeClear});
        stage=TEXT("native_clear_cover");
        while(!pending.IsEmpty())
        {
            const FCell cell=pending.Pop(EAllowShrinking::No);
            S::BoundedPose bounds[2];
            bool clear=boundCell(cell.First,cell.Last,bounds);
            S::NativeClearCell certificate;certificate.First=cell.First;certificate.Last=cell.Last;
            for(int32 k=0;clear&&k<8;++k)
            {
                const auto vertex=TimeDetail::Hull(cellPoint(bounds[0].Corners[k]),cellPoint(bounds[1].Corners[k]));
                Interval signedGap;
                clear=contains(vertex)&&gap(vertex,signedGap)&&signedGap.Lower>0.;
                if(clear)certificate.Gaps[k]=copy(signedGap);
            }
            if(clear)
            {
                if(evidence->Native.Cells>=evidence->Native.Cover.size())return false;
                evidence->Native.Cover[evidence->Native.Cells++]=certificate;prefix.Add(cell);continue;
            }
            if(work.Invalid||work.Stopped||cell.Last<=cell.First+1)return false;
            const uint32 middle=cell.First+(cell.Last-cell.First)/2;
            pending.Add(FCell{middle,cell.Last});pending.Add(FCell{cell.First,middle});
        }

        stage=TEXT("native_readback");
        FIntervalPoint clearCenter,lastPose[8];
        for(uint32 time:{nativeClear,nativeInside})
        {
            S::BoundedPose predicted[2];
            if(!boundCell(time,time,predicted))return false;
            auto pose=input.Owner.Integrate(seconds(time));
            pose.Location-=pose.Rotation.RotateVector(input.LocalCOM);
            pose.Location+=pose.Rotation.RotateVector(input.LocalOffset);
            pose.Rotation=pose.Rotation*input.LocalRotation;
            const double rotation[4]={pose.Rotation.X,pose.Rotation.Y,pose.Rotation.Z,pose.Rotation.W};
            if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)return false;
            for(const auto& p:predicted)
            {
                for(int32 d=0;d<3;++d)if(!p.Center[d].Exactly(pose.Location[d]))return false;
                for(int32 d=0;d<4;++d)if(!p.Rotation[d].Exactly(rotation[d]))return false;
            }
            if(time==nativeClear)clearCenter=Detail::Point(pose.Location);
            for(int32 k=0;k<8;++k)
            {
                if(!work.Pay())return false;
                FVector local;
                for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?input.PhysicalHalfExtent[d]:-input.PhysicalHalfExtent[d];
                const FVector vertex=pose.Location+pose.Rotation.RotateVector(local);
                for(const auto& p:predicted)for(int32 d=0;d<3;++d)
                    if(!p.Corners[k][d].Exactly(vertex[d]))return false;
                if(time==nativeInside)
                {
                    lastPose[k]=Detail::Point(vertex);
                    Interval signedGap;
                    if(!contains(lastPose[k])||!gap(lastPose[k],signedGap))return false;
                    evidence->Native.FirstInsideGaps[k]=copy(signedGap);
                }
            }
        }
        if(!S::VerifyAttestedGraphFacts(seal.Get(),live,chart,evidence->Raw,evidence->Normalized,evidence->Native,originalH))return false;

        stage=TEXT("native_edge_chart");
        const auto& a=lastPose[corner];const auto& b=lastPose[opposite];
        if(!graph.InOriginalPatch(a)||!graph.InOriginalPatch(b))return false;
        FIntervalPoint nearA;
        for(int32 d=0;d<3;++d)nearA.Axis[d]=A::Add(a.Axis[d],
            A::Multiply(Interval::Exact(0x1p-32),A::Subtract(b.Axis[d],a.Axis[d])));
        Interval nearGap;
        if(!graph.InOriginalPatch(nearA)||!gap(nearA,nearGap)||nearGap.Lower<=0.)return false;
        const auto edge=TimeDetail::Hull(a,nearA);
        FGraph::FPositionMemo memo(graph,reuse);
        Interval u,v;
        if(!graph.BoundParameter(edge.Axis[1],work,memo,v)||!work.Pay()||
            !AffineReferenceDetail::FreeParameter(product,edge,u)||!(0.<u.Lower&&u.Upper<1.))return false;
        FYZ lower,upper;
        if(!memo.At(v.Lower,work,lower)||!memo.At(v.Upper,work,upper)||!work.Pay())return false;
        const auto tangentY=G::Evaluate(dy,v),tangentZ=G::Evaluate(dz,v);
        if(!A::Valid(tangentY)||!A::Valid(tangentZ)||tangentY.Lower<=0.)return false;
        const FIntervalPoint surface{{edge.Axis[0],{lower.Y.Lower,upper.Y.Upper,true},{lower.Z.Lower,upper.Z.Upper,true}}};
        const FIntervalPoint normal{{Interval::Exact(0.),A::Negate(tangentZ),tangentY}};
        if(!contains(clearCenter)||!Detail::ValidPoint(surface)||!Detail::ValidPoint(normal)||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)return false;

        TSharedPtr<FProof,ESPMode::ThreadSafe> proof(new FProof);
        proof->Input=input;proof->Graph=graph;proof->Numeric=numeric;proof->ReuseExactEvaluations=reuse;
        proof->ClearOrdinal=nativeClear;proof->InsideOrdinal=nativeInside;proof->SearchOrdinal=nativeInside;proof->Corner=corner;
        proof->U=u;proof->V=v;proof->SurfacePoint=surface;proof->OutwardRawNormal=normal;proof->Envelope=envelope;
        proof->ClearNativeCenter=clearCenter;proof->Prefix=MoveTemp(prefix);proof->Serial3=MoveTemp(evidence);
        out=MoveTemp(proof);stage=TEXT("complete");return true;
    }
private:

    static bool TrySplit202(const FGeneralBoxMotionInput& input,const FProduct& product,const FGraph& graph,
        uint32 common,uint32 end,const FIntervalPoint& envelope,FAccount& work,FGraph::FPositionMemo& memo,
        uint64 numeric,bool reuse,const TArray<FCell,TInlineAllocator<32>>& prefix,
        TSharedPtr<const FProof,ESPMode::ThreadSafe>& out,const TCHAR*& stage)
    {
        namespace R=Case202SplitReference;
        namespace G=FactoredLowerCapGraphGap;
        constexpr int32 contactCorner=5,edgeOther=contactCorner^4;
        stage=TEXT("capture");
        const auto ordinalForBits=[](uint32 bits,uint32& ordinal)
        {return FloorNativeSweepDetail::Ordinal(KinematicCenterDetail::FromBits(bits),ordinal);};
        uint32 expectedCommon=0,expectedEnd=0,rawClear=0,rawInside=0,nativeClear=0,nativeInside=0;
        // Captures are printed as IEEE bits. Traversal ordinals omit positive
        // subnormals and must always use the engine's explicit conversion.
        if(!ordinalForBits(0x3b1e2085u,expectedCommon)||!ordinalForBits(0x3b5a740eu,expectedEnd)||
            !ordinalForBits(0x3b1e21dbu,rawClear)||!ordinalForBits(0x3b1e21dcu,rawInside)||
            !ordinalForBits(0x3b1e21ddu,nativeClear)||!ordinalForBits(0x3b1e21deu,nativeInside)||
            common!=expectedCommon||end!=expectedEnd||!PreparedExecution::Requested()||
            input.LocalRotation.X!=0.||input.LocalRotation.Y!=0.||input.LocalRotation.Z!=0.||input.LocalRotation.W!=1.||
            prefix.IsEmpty()||!Detail::ValidPoint(envelope))return false;
        uint32 covered=0;
        for(const auto& cell:prefix)
        {
            if(!work.Pay()||cell.First!=covered||cell.Last<=covered)return false;
            covered=cell.Last;
        }
        if(covered!=common)return false;
        const auto xyz=[](const FVector& v,double (&out)[3]){out[0]=v.X;out[1]=v.Y;out[2]=v.Z;};
        R::FInput reference{};
        xyz(input.Owner.Location,reference.Position);xyz(input.Owner.Velocity,reference.Velocity);
        xyz(input.Owner.Acceleration,reference.Acceleration);xyz(input.Owner.AngularVelocity,reference.AngularVelocity);
        xyz(input.Owner.AngularAcceleration,reference.AngularAcceleration);
        xyz(input.LocalCOM,reference.LocalCOM);xyz(input.LocalOffset,reference.LocalOffset);
        xyz(input.PhysicalHalfExtent,reference.HalfExtent);
        reference.Rotation[0]=input.Owner.Rotation.X;reference.Rotation[1]=input.Owner.Rotation.Y;
        reference.Rotation[2]=input.Owner.Rotation.Z;reference.Rotation[3]=input.Owner.Rotation.W;
        G::FControl controls[6];
        for(int32 i=0;i<6;++i)controls[i]={graph.P[i].Y,graph.P[i].Z};
        G::FProfile profile;
        const auto pay=[&](){return work.Pay();};
        if(!G::FProfile::TryBuild(controls,pay,profile))return false;
        const auto sign=[&](const FIntervalPoint& point,bool above)
        {return Split202GraphSign(graph,profile,point,above,work);};
        const auto referencePoint=[&](uint32 first,uint32 last,int32 k,bool unit,FIntervalPoint& point)
        {
            const Interval times{double(FloorNativeSweepDetail::Seconds(first)),double(FloorNativeSweepDetail::Seconds(last)),true};
            R::FBounds bounds;
            if(!work.Pay()||!R::TryBound(reference,times,k,bounds))return false;
            const auto& p=unit?bounds.Normalized:bounds.Raw;
            for(int32 d=0;d<3;++d)point.Axis[d]=p[d];
            return true;
        };
        const FCell referenceBrackets[]={{rawClear,rawInside},{nativeClear,nativeInside}};
        for(int32 family=0;family<2;++family)
        {
            stage=family?TEXT("normalized_reference"):TEXT("raw_reference");
            const auto bracket=referenceBrackets[family];
            for(int32 k=0;k<8;++k)
            {
                FIntervalPoint p;
                const uint32 last=k==contactCorner?bracket.First:bracket.Last;
                if(!referencePoint(common,last,k,family==1,p)||!sign(p,true))return false;
            }
            for(int32 k:{contactCorner,edgeOther})
            {
                FIntervalPoint p;
                if(!referencePoint(bracket.Last,bracket.Last,k,family==1,p)||
                    !graph.InOriginalPatch(p)||!sign(p,k!=contactCorner))return false;
            }
        }
        // Enumerate the exact prepared binary32 instants only over the unresolved
        // suffix. The existing proof still covers the complete common prefix.
        stage=TEXT("native_samples");
        FIntervalPoint nativePrefix[8],lastPose[8],clearCenter;
        for(uint32 ordinal=common;ordinal<=nativeInside;++ordinal)
        {
            if(!work.Pay())return false;
            auto pose=input.Owner.Integrate(FloorNativeSweepDetail::Seconds(ordinal));
            pose.Location-=pose.Rotation.RotateVector(input.LocalCOM);
            pose.Location+=pose.Rotation.RotateVector(input.LocalOffset);
            pose.Rotation=pose.Rotation*input.LocalRotation;
            if(pose.Location.ContainsNaN()||pose.Rotation.ContainsNaN()||
                RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)return false;
            if(ordinal==nativeClear)clearCenter=Detail::Point(pose.Location);
            for(int32 k=0;k<8;++k)
            {
                if(!work.Pay())return false;
                FVector corner;
                for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?input.PhysicalHalfExtent[d]:-input.PhysicalHalfExtent[d];
                const auto point=Detail::Point(pose.Location+pose.Rotation.RotateVector(corner));
                if(!Detail::ValidPoint(point))return false;
                for(int32 d=0;d<3;++d)
                    if(point.Axis[d].Lower<envelope.Axis[d].Lower||point.Axis[d].Upper>envelope.Axis[d].Upper)return false;
                if(ordinal==nativeInside)lastPose[k]=point;
                else nativePrefix[k]=ordinal==common?point:TimeDetail::Hull(nativePrefix[k],point);
            }
        }
        stage=TEXT("native_edge");
        for(int32 k=0;k<8;++k)
            if(!sign(nativePrefix[k],true)||(k!=contactCorner&&!sign(lastPose[k],true)))return false;
        const auto& a=lastPose[contactCorner];const auto& b=lastPose[edgeOther];
        if(!graph.InOriginalPatch(a)||!graph.InOriginalPatch(b)||!sign(a,false))return false;
        // Locate a short sub-edge containing the native event as well as the
        // normalized continuous bracket used for the response chart.
        FIntervalPoint closeToA,referenceCorner;
        for(int32 d=0;d<3;++d)closeToA.Axis[d]=A::Add(a.Axis[d],
            A::Multiply(Interval::Exact(0x1p-32),A::Subtract(b.Axis[d],a.Axis[d])));
        if(!graph.InOriginalPatch(closeToA)||!sign(closeToA,true)||
            !referencePoint(nativeClear,nativeInside,contactCorner,true,referenceCorner))return false;
        const auto corner=TimeDetail::Hull(referenceCorner,TimeDetail::Hull(a,closeToA));
        stage=TEXT("response_chart");
        Interval u,v;
        if(!graph.InOriginalPatch(corner)||!graph.BoundParameter(corner.Axis[1],work,memo,v)||!work.Pay()||
            !AffineReferenceDetail::FreeParameter(product,corner,u)||!(0.<u.Lower&&u.Upper<1.))return false;
        FYZ lower,upper;
        if(!memo.At(v.Lower,work,lower)||!memo.At(v.Upper,work,upper)||!work.Pay())return false;
        Interval dy[5],dz[5];
        for(int32 i=0;i<5;++i){dy[i]=graph.D[i].Y;dz[i]=graph.D[i].Z;}
        const auto derivativeY=G::Evaluate(dy,v),derivativeZ=G::Evaluate(dz,v);
        if(!A::Valid(derivativeY)||!A::Valid(derivativeZ)||!(derivativeY.Lower>0.))return false;
        const FIntervalPoint point{{corner.Axis[0],{lower.Y.Lower,upper.Y.Upper,true},{lower.Z.Lower,upper.Z.Upper,true}}};
        const FIntervalPoint normal{{A::Interval::Exact(0.),A::Negate(derivativeZ),derivativeY}};
        if(!Detail::ValidPoint(point)||!Detail::ValidPoint(normal)||!Detail::ValidPoint(clearCenter)||!work.Pay()||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)return false;
        TSharedPtr<FProof,ESPMode::ThreadSafe> proof(new FProof);
        proof->Input=input;proof->Graph=graph;proof->Numeric=numeric;proof->ReuseExactEvaluations=reuse;
        proof->ClearOrdinal=nativeClear;proof->InsideOrdinal=nativeInside;proof->SearchOrdinal=end;proof->Corner=contactCorner;
        proof->U=u;proof->V=v;proof->SurfacePoint=point;proof->OutwardRawNormal=normal;proof->Envelope=envelope;
        proof->ClearNativeCenter=clearCenter;proof->Prefix=prefix;
        proof->SplitReference=FSplitReferenceProof{referenceBrackets[0],referenceBrackets[1],common};
        out=MoveTemp(proof);stage=TEXT("complete");return true;
    }

    static bool Bounds(const FGeneralBoxMotionInput& input,uint32 a,uint32 b,FAccount& work,FGeneralBoxMotionBounds& out)
    {
        if(!work.Pay())return false;
        if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,FloorNativeSweepDetail::Seconds(a),
            FloorNativeSweepDetail::Seconds(b),out)||out.Corners.Num()!=32)
        {work.Invalid=true;return false;}
        return true;
    }
    static uint16 ClearMask(const FGraph& graph,const FGeneralBoxMotionBounds& bounds,uint16 proven,
        FAccount& work,FGraph::FPositionMemo& memo,int32 witnessCorner=INDEX_NONE,FGraph::FAboveWitness* witness=nullptr)
    {
        for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
        {
            const auto bit=uint16(1u<<(family*8+k));
            if(!(proven&bit)&&graph.Sign(bounds.Corners[family*16+k],true,work,memo,
                family==1&&k==witnessCorner?witness:nullptr))proven|=bit;
            if(work.Stopped||work.Invalid)return proven;
        }
        return proven;
    }
    static bool ReferenceAdjacentChord(const FGeneralBoxMotionInput& input,const FGraph& graph,
        uint32 first,uint32 last,int32 corner,const FGeneralBoxMotionBounds& atFirst,const FGeneralBoxMotionBounds& atLast,
        const FGraph::FAboveWitness& firstWitness,const FGraph::FAboveWitness& lastWitness,FAccount& work)
    {
        if(last!=first+1||corner<0||corner>=8||atFirst.Corners.Num()!=32||atLast.Corners.Num()!=32||
            !firstWitness.Known||!lastWitness.Known||!work.Pay())return false;
        const double lo=FloorNativeSweepDetail::Seconds(first),hi=FloorNativeSweepDetail::Seconds(last);
        const Interval time{lo,hi,true};
        if(!A::Valid(time)||!(0.<lo&&lo<hi&&hi<=input.Horizon))return false;
        const auto square=A::Square(time);
        const auto& owner=input.Owner;
        bool nonzeroTheta=false;
        for(int32 d=0;d<3;++d)
        {
            const auto theta=A::Add(A::Multiply(Interval::Exact(owner.AngularVelocity[d]),time),
                A::Multiply(A::Multiply(Interval::Exact(.5),Interval::Exact(owner.AngularAcceleration[d])),square));
            if(!A::Valid(theta))return false;
            nonzeroTheta|=theta.Lower>0.||theta.Upper<0.;
        }
        const auto norm2=[](const FQuat& q)
        {
            auto value=Interval::Exact(0.);
            for(double axis:{q.X,q.Y,q.Z,q.W})value=A::Add(value,A::Square(Interval::Exact(axis)));
            return A::Valid(value)&&value.Lower>.5&&value.Upper<2.;
        };
        if(!nonzeroTheta||!norm2(owner.Rotation)||!norm2(input.LocalRotation))return false;
        const auto oneNorm=[](const FVector& v)
        {
            return A::Add(A::Add(Interval::Exact(FMath::Abs(v.X)),Interval::Exact(FMath::Abs(v.Y))),
                Interval::Exact(FMath::Abs(v.Z)));
        };
        // For R=exp([theta]_x), differentiate the matrix exponential on
        // its integral simplex. All exp(s[theta]_x) have operator norm 1,
        // so ||R''|| <= ||theta''|| + ||theta'||^2. In the raw family a
        // quaternion of constant squared norm s rotates by (1-s)I+sR.
        // The normalized family has s=1. The two fixed quaternion bounds
        // above give factors <=2 for the lever and <=4 for the body vertex.
        const auto alpha=oneNorm(owner.AngularAcceleration);
        const auto omega=A::Add(oneNorm(owner.AngularVelocity),A::Multiply(Interval::Exact(hi),alpha));
        const auto rotationSecond=A::Add(alpha,A::Square(omega));
        const auto lever=A::Multiply(Interval::Exact(2.),A::Add(oneNorm(input.LocalCOM),oneNorm(input.LocalOffset)));
        const auto vertex=A::Multiply(Interval::Exact(4.),oneNorm(input.PhysicalHalfExtent));
        const auto second=A::Add(oneNorm(owner.Acceleration),A::Multiply(A::Add(lever,vertex),rotationSecond));
        const auto dt=A::Subtract(Interval::Exact(hi),Interval::Exact(lo));
        const auto error=A::Multiply(A::Multiply(second,A::Square(dt)),Interval::Exact(.125));
        if(!A::Valid(error)||error.Upper<0.)return false;
        // Each exact reference family is within M*dt^2/8 of its own
        // endpoint chord. Their two endpoint boxes already cover both raw
        // and normalized values. Move each endpoint to the worst Y/Z by
        // that bound. Its chord lies in the same strict convex epigraph,
        // and the actual reference trajectory is further left/above it.
        const auto inWitness=[&](FIntervalPoint point,const FGraph::FAboveWitness& witness)
        {
            for(auto& axis:point.Axis)axis=A::Add(axis,Interval{-error.Upper,error.Upper,true});
            return Detail::ValidPoint(point)&&graph.InOriginalPatch(point)&&
                point.Axis[1].Upper<witness.YLimit&&point.Axis[2].Lower>witness.ZFloor;
        };
        return inWitness(atFirst.Corners[16+corner],firstWitness)&&inWitness(atLast.Corners[16+corner],lastWitness);
    }

    static bool ReferenceCellClear(const FGeneralBoxMotionInput& input,const FGraph& graph,
        Interval time,uint16 alreadyClear,FAccount& work,FGraph::FPositionMemo& memo)
    {
        using namespace GeneralBoxDetail;
        using namespace YawFloorBoxDetail;
        if(!work.Pay())return false;
        if(!A::Valid(time)||time.Lower<0.||time.Upper>input.Horizon)
        {work.Invalid=true;return false;}
        // Match the reference side of RotationFamilyOnGlobalFloatSeconds.
        // This cell restricts the real time variable. The captured state and
        // time origin stay unchanged. Native poses are handled separately.
        const auto& owner=input.Owner;
        const auto squared=A::Square(time);
        if(!A::Valid(squared)){work.Invalid=true;return false;}
        FNode theta[3];
        bool includeInitial=true;
        for(int32 d=0;d<3;++d)
        {
            const auto node=Sum(Product(Leaf(owner.AngularVelocity[d]),FNode{time,0}),
                Product(Product(Leaf(.5),Leaf(owner.AngularAcceleration[d])),FNode{squared,0}));
            const auto zeroRange=Expanded(node);
            if(!A::Valid(node.Range)||!A::Valid(zeroRange)){work.Invalid=true;return false;}
            theta[d]={node.Range,0};
            includeInitial&=zeroRange.Lower<=0.&&0.<=zeroRange.Upper;
        }
        auto angleSquared=A::Add(A::Add(A::Square(theta[0].Range),A::Square(theta[1].Range)),A::Square(theta[2].Range));
        if(!A::Valid(angleSquared)){work.Invalid=true;return false;}
        angleSquared.Lower=FMath::Max(0.,angleSquared.Lower);
        Interval scale,cosine;
        if(!TrigPolynomialBounds(angleSquared,scale,cosine))return false;
        FQuaternion delta,ownerRotation;
        for(int32 d=0;d<3;++d)delta.Axis[d]=Product(theta[d],FNode{scale,0});
        delta.Axis[3]={cosine,0};
        if(!GeneralBoxGlobalTimeDetail::NormalizedFamilyOnGlobalRange(
            Hamilton(delta,Quaternion(owner.Rotation)),owner.Rotation,includeInitial,ownerRotation))return false;
        const auto com=PointRanges(Rotate(ownerRotation,input.LocalCOM));
        const auto offset=PointRanges(Rotate(ownerRotation,input.LocalOffset));
        if(!Detail::ValidPoint(com)||!Detail::ValidPoint(offset)){work.Invalid=true;return false;}
        FIntervalPoint center;
        for(int32 d=0;d<3;++d)
        {
            const auto ownerCenter=A::Add(A::Add(A::Interval::Exact(owner.Location[d]),
                A::Multiply(A::Interval::Exact(owner.Velocity[d]),time)),
                A::Multiply(A::Multiply(A::Interval::Exact(.5),A::Interval::Exact(owner.Acceleration[d])),squared));
            center.Axis[d]=A::Add(A::Subtract(ownerCenter,com.Axis[d]),offset.Axis[d]);
        }
        if(!Detail::ValidPoint(center)){work.Invalid=true;return false;}
        const auto bodyRotation=Hamilton(ownerRotation,Quaternion(input.LocalRotation));
        for(int32 corner=0;corner<8;++corner)
        {
            if(alreadyClear&uint16(1u<<(8+corner)))continue;
            FVector local;
            for(int32 d=0;d<3;++d)
                local[d]=(corner&(1<<d))?input.PhysicalHalfExtent[d]:-input.PhysicalHalfExtent[d];
            const auto rotated=PointRanges(Rotate(bodyRotation,local));
            if(!Detail::ValidPoint(rotated)){work.Invalid=true;return false;}
            FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],rotated.Axis[d]);
            if(!Detail::ValidPoint(point)){work.Invalid=true;return false;}
            if(!graph.Sign(point,true,work,memo))return false;
        }
        return true;
    }

    static bool ReferenceAdjacentClear(const FGeneralBoxMotionInput& input,const FGraph& graph,
        uint32 first,uint32 last,uint16 alreadyClear,FAccount& work,FGraph::FPositionMemo& memo,int32& visited,
        int32 maxTimeCells,bool extendOrdinary,bool traceOrdinary)
    {
        if(last!=first+1){work.Invalid=true;return false;}
        constexpr int32 DefaultMaxSubcells=8;
        const int32 maxSubcells=extendOrdinary?16:DefaultMaxSubcells;
        const double lo=FloorNativeSweepDetail::Seconds(first),hi=FloorNativeSweepDetail::Seconds(last);
        struct FRealCell {double First,Last;};
        TArray<FRealCell,TInlineAllocator<DefaultMaxSubcells>> pending;
        pending.Add({lo,hi});
        double covered=lo;
        int32 coveredCells=0;
        while(!pending.IsEmpty())
        {
            if(visited+1>=maxTimeCells)return false;
            ++visited;
            const auto cell=pending.Pop(EAllowShrinking::No);
            if(cell.First!=covered||!(cell.First<cell.Last&&cell.Last<=hi)){work.Invalid=true;return false;}
            if(ReferenceCellClear(input,graph,{cell.First,cell.Last,true},alreadyClear,work,memo))
            {
                covered=cell.Last;
                ++coveredCells;
                continue;
            }
            // Reallocate the bounded subcells to the uncertain part.
            // Both children use the original real time and remain closed.
            // Every attempt still charges the same global work and cell limits.
            const double middle=.5*cell.First+.5*cell.Last;
            const bool localLimit=coveredCells+pending.Num()+2>maxSubcells;
            if(work.Stopped||work.Invalid||localLimit||
                !(cell.First<middle&&middle<cell.Last)||!work.Pay())
            {
                if(traceOrdinary&&localLimit)
                    UE_LOG(LogTemp,Display,TEXT("[CurvedBoxOrdinaryReferenceSubcells] Cap=%d Covered=%d Pending=%d Visited=%d"),
                        maxSubcells,coveredCells,pending.Num(),visited);
                return false;
            }
            pending.Add({middle,cell.Last});
            pending.Add({cell.First,middle});
        }
        return covered==hi;
    }

    static bool InsideEdge(const FGraph& graph,const FGeneralBoxMotionBounds& bounds,int32 corner,
        FAccount& work,FGraph::FPositionMemo& memo,const TCHAR* traceAdjacent=nullptr)
    {
        for(int32 family=0;family<2;++family)
        {
            const auto& a=bounds.Corners[family*16+corner];
            const auto& b=bounds.Corners[family*16+(corner^4)];
            const auto refused=[&](const TCHAR* stage)
            {
                if(traceAdjacent)
                    UE_LOG(LogTemp,Display,TEXT("%s Stage=%s Family=%s Corner=%d"),
                        traceAdjacent,stage,family?TEXT("Reference"):TEXT("Consumed"),corner);
                return false;
            };
            if(!graph.InOriginalPatch(a))return refused(TEXT("CornerOutsideOriginalPatch"));
            auto opposite=b;
            if(!graph.InOriginalPatch(b)&&graph.AdjacentBandSeam)
            {
                if(!work.Pay())return false;
                // A fixed interior point on the SAME rigid edge is enough.
                // Both endpoints of this sub-edge must lie in the original
                // finite chart and prove opposite signs independently.
                for(int32 axis=0;axis<3;++axis)
                    opposite.Axis[axis]=A::Add(a.Axis[axis],A::Multiply(
                        Interval::Exact(1./64.),A::Subtract(b.Axis[axis],a.Axis[axis])));
                if(!Detail::ValidPoint(opposite)){work.Invalid=true;return false;}
            }
            if(!graph.InOriginalPatch(opposite))return refused(TEXT("OppositeOutsideOriginalPatch"));
            if(!graph.Sign(a,false,work,memo))return refused(TEXT("CornerNotProvedInside"));
            if(!graph.Sign(opposite,true,work,memo))return refused(TEXT("OppositeNotProvedClear"));
        }
        // On each actual rigid body the whole same edge lies in the true X/Y
        // domain. Its signed gap is continuous from opposite signs, so it
        // intersects the original patch. A negative vertex alone is not proof.
        return true;
    }

    static uint32 MidpointOrdinal(uint32 first,uint32 last)
    {
        const float midpoint=static_cast<float>(.5*double(FloorNativeSweepDetail::Seconds(first))+
            .5*double(FloorNativeSweepDetail::Seconds(last)));
        uint32 middle=0;
        if(!FloorNativeSweepDetail::Ordinal(midpoint,middle)||middle<=first||middle>=last)
            middle=first+(last-first)/2;
        return middle;
    }

public:
    using FPtr=TSharedPtr<const FProof,ESPMode::ThreadSafe>;
    static EResult Build(const FGeneralBoxMotionInput& input,const FProduct& product,
        FDistanceBudget& budget,int32 maximumWork,FPtr& out,bool reuseExactEvaluations=false,bool mixedConvexity=false,
        float searchCap=0.f,bool adjacentBandSeam=false,bool allowNativeSplit202=false,
        bool nominalInverseProposal=false,const Prepared::FReferenceTransport* reference205=nullptr,
        ETraversalProposal traversalProposal=ETraversalProposal::Default)
    {
        out.Reset();
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        uint32 end=0;
        if(maximumWork<=0||!FloorNativeSweepDetail::Ordinal(input.Horizon,end)||end<=1)return EResult::NotApplicable;
        if(searchCap!=0.f&&(mixedConvexity||!FloorNativeSweepDetail::Ordinal(searchCap,end)||end<=1||searchCap>input.Horizon))
            return EResult::NotApplicable;
        FAccount work{budget,budget.Used,maximumWork};
        const bool traceOrdinary=!mixedConvexity&&CVarOrdinaryCurvedBoxTrace.GetValueOnAnyThread()==1;
        const bool trace=traceOrdinary||(mixedConvexity&&CVarMixedFloorCurveTrace.GetValueOnAnyThread()==1);
        const bool extendOrdinary=!mixedConvexity&&CVarOrdinaryCurvedBoxReferenceSubcells.GetValueOnAnyThread()==1;
        int32 visited=0;
        const auto reportCase585Frontier=[&](const TCHAR* stage,uint32 first,uint32 last)
        {
            if(traversalProposal==ETraversalProposal::Owner88First)
            {
                UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCase585Owner88FocusedTraversal] Stage=%s Proposal=3/32 Cells=%d Work=%llu FirstBits=%08x LastBits=%08x"),
                    stage,visited,budget.Used-work.Began,
                    KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(first)),
                    KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(last)));
            }
            if(traversalProposal==ETraversalProposal::Owner88Remainder)
            {
                UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCase585Owner88RemainderTraversal] Stage=%s Proposal=1/8 Cells=%d Work=%llu Remaining=%d Floor=%d FirstBits=%08x LastBits=%08x"),
                    stage,visited,budget.Used-work.Began,budget.Remaining,budget.ConsumptionFloor,
                    KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(first)),
                    KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(last)));
            }
        };
        const auto refuse=[&](const TCHAR* stage,EResult result,uint32 first=0,uint32 last=0,
            uint16 mask=0,uint16 clearMask=0,uint16 endMask=0,const FGeneralBoxMotionBounds* atClear=nullptr,
            const FGeneralBoxMotionBounds* atInside=nullptr)
        {
            reportCase585Frontier(stage,first,last);
            if(!trace)return result;
            const auto firstBits=KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(first));
            const auto lastBits=KinematicCenterDetail::Bits(FloorNativeSweepDetail::Seconds(last));
            UE_LOG(LogTemp,Display,TEXT("%s Stage=%s Status=%u FirstBits=%08x LastBits=%08x RangeClear=%04x FirstClear=%04x LastClear=%04x Work=%llu"),
                traceOrdinary?TEXT("[CurvedBoxOrdinaryRefusal]"):TEXT("[CurvedBoxMixedFloorRefusal]"),
                stage,uint32(result),firstBits,lastBits,uint32(mask),uint32(clearMask),uint32(endMask),budget.Used-work.Began);
            const auto dump=[&](const TCHAR* endpoint,const FGeneralBoxMotionBounds* bounds)
            {
                if(!bounds||bounds->Corners.Num()!=32)return;
                for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
                {
                    // Each family's stored box already covers its raw and
                    // normalized quaternion alternatives. Do not recalculate
                    // proof signs, consume work, or refine those bounds here.
                    const auto& c=bounds->Corners[16*family+k];
                    UE_LOG(LogTemp,Display,TEXT("%s At=%s Family=%s Corner=%d X=[%.17g,%.17g] Y=[%.17g,%.17g] Z=[%.17g,%.17g]"),
                        traceOrdinary?TEXT("[CurvedBoxOrdinaryBound]"):TEXT("[CurvedBoxMixedFloorBound]"),
                        endpoint,family?TEXT("Reference"):TEXT("Consumed"),k,c.Axis[0].Lower,c.Axis[0].Upper,
                        c.Axis[1].Lower,c.Axis[1].Upper,c.Axis[2].Lower,c.Axis[2].Upper);
                }
            };
            dump(TEXT("First"),atClear);
            dump(TEXT("Last"),atInside);
            return result;
        };
        FGraph graph;
        if((adjacentBandSeam&&(mixedConvexity||searchCap==0.f))||
            !FGraph::Make(product,work,graph,mixedConvexity,adjacentBandSeam))
            return refuse(TEXT("Graph"),work.Stopped?EResult::Work:EResult::NotApplicable);
        graph.NominalInverseProposal=nominalInverseProposal&&adjacentBandSeam&&!mixedConvexity;
        FGraph::FPositionMemo memo(graph,reuseExactEvaluations);
        FGeneralBoxMotionBounds full,terminal;
        if(!Bounds(input,0,end,work,full)||!Bounds(input,end,end,work,terminal))return refuse(TEXT("TerminalBounds"),work.Failure(),0,end);
        // Narrow v1 supports the original lower cap and only the LEFT extension.
        if(!(full.Hull.Axis[1].Upper<graph.P[5].Y.Lower))return refuse(TEXT("RightChartLimit"),EResult::NotApplicable,0,end);
        int32 insideCorner=INDEX_NONE;
        for(int32 k=0;k<8&&insideCorner==INDEX_NONE;++k)
            if(InsideEdge(graph,terminal,k,work,memo))insideCorner=k;
        if(insideCorner==INDEX_NONE)return refuse(TEXT("NoTerminalInsideEdge"),work.Failure(),end,end,0,0,0,nullptr,&terminal);
        FGeneralBoxMotionBounds initial;
        if(!Bounds(input,0,0,work,initial))return refuse(TEXT("InitialBounds"),work.Failure());
        const auto initialMask=ClearMask(graph,initial,0,work,memo);
        if(initialMask!=AllCorners)return refuse(TEXT("InitialNotClear"),work.Failure(),0,0,0,initialMask,0,&initial);
        struct FPending {FCell Cell;uint16 Inherited;};
        TArray<FPending,TInlineAllocator<32>> pending;
        TArray<FCell,TInlineAllocator<32>> clear;
        uint32 lastInside=end;
        int32 hintSamples=0;
        if(reuseExactEvaluations)
        {
            uint32 firstCandidate=0;
            const uint64 began=budget.Used;
            constexpr uint64 pointWork=1+4*(1+MaxInverseDepth);
            const uint64 hintAllowance=FMath::Min<uint64>(2048,uint64(maximumWork)/4);
            for(;hintSamples<28&&lastInside>firstCandidate+1&&
                budget.Used-began+pointWork<=hintAllowance;++hintSamples)
            {
                const uint32 middle=MidpointOrdinal(firstCandidate,lastInside);
                if(middle<=firstCandidate||middle>=lastInside)return EResult::Arithmetic;
                FGeneralBoxMotionBounds point;
                if(!Bounds(input,middle,middle,work,point))return work.Failure();
                // An exact inside witness is a safe upper time bound. A
                // rejected point only changes where the next hint is sought.
                if(InsideEdge(graph,point,insideCorner,work,memo))lastInside=middle;
                else firstCandidate=middle;
                if(work.Stopped||work.Invalid)return work.Failure();
            }
        }
        const bool focused=lastInside<end;
        const int32 maxTimeCells=focused&&!mixedConvexity&&CVarOrdinaryCurvedBoxFocusedTimeCells.GetValueOnAnyThread()==1?
            MaxFocusedOrdinaryTimeCells:MaxTimeCells;
        uint16 prefixMask=0;
        if(focused)
        {
            FGeneralBoxMotionBounds prefix;
            if(!Bounds(input,0,lastInside,work,prefix))return work.Failure();
            prefixMask=ClearMask(graph,prefix,0,work,memo);
            if(work.Stopped||work.Invalid)return work.Failure();
        }
        else pending.Add({{0,end},0});

        uint32 covered=0;
        for(visited=hintSamples;visited<maxTimeCells&&(!pending.IsEmpty()||(focused&&covered<lastInside));++visited)
        {
            if(pending.IsEmpty())
            {
                // Spend larger cells far from the witnessed event and finer
                // cells near it. Each is still closed below before advancing.
                const double firstSeconds=FloorNativeSweepDetail::Seconds(covered);
                const double insideSeconds=FloorNativeSweepDetail::Seconds(lastInside);
                // Only a different cell proposal. The existing full-family
                // bounds and strict traversal prove every accepted cell.
                const float proposed=static_cast<float>(traversalProposal==ETraversalProposal::Owner88Remainder?
                    .875*firstSeconds+.125*insideSeconds:
                    traversalProposal==ETraversalProposal::Owner88First?(29./32.)*firstSeconds+(3./32.)*insideSeconds:
                    .75*firstSeconds+.25*insideSeconds);
                uint32 next=0;
                if(!FloorNativeSweepDetail::Ordinal(proposed,next)||next<=covered||next>lastInside)
                    next=covered+1;
                if(!work.Pay())return work.Failure();
                pending.Add({{covered,next},prefixMask});
            }
            const auto node=pending.Pop(EAllowShrinking::No);
            const auto [first,last]=node.Cell;
            if(first!=covered||first>=last)return EResult::Arithmetic;
            FGeneralBoxMotionBounds restricted;
            const bool isFull=reuseExactEvaluations&&first==0&&last==end;
            if(isFull)
            {
                // The exact same original root interval was already enclosed.
                if(!work.Pay())return work.Failure();
            }
            else if(!Bounds(input,first,last,work,restricted))return work.Failure();
            const auto& current=isFull?full:restricted;
            const uint16 mask=ClearMask(graph,current,node.Inherited,work,memo);
            if(work.Stopped||work.Invalid)
                return nominalInverseProposal||traversalProposal==ETraversalProposal::Owner88Remainder?
                    refuse(TEXT("PrefixCellSign"),work.Failure(),first,last,mask):work.Failure();
            if(mask==AllCorners)
            {
                if(!work.Pay())return work.Failure();
                clear.Add(node.Cell);covered=last;continue;
            }
            if(last==first+1)
            {
                if(!first||clear.IsEmpty()||clear.Last().Last!=first)return refuse(TEXT("AdjacentWithoutPrefix"),EResult::Unproved,first,last,mask);
                FGeneralBoxMotionBounds atClear,atInside;
                FGraph::FAboveWitness firstWitness,lastWitness;
                int32 witnessCorner=INDEX_NONE;
                const uint32 referenceMissing=(uint32(~mask)>>8)&0xffu;
                if(!mixedConvexity&&CVarOrdinaryCurvedBoxAdjacentReferenceChord.GetValueOnAnyThread()==1&&
                    referenceMissing&&!(referenceMissing&(referenceMissing-1u)))
                    witnessCorner=FMath::CountTrailingZeros(referenceMissing);
                if(!Bounds(input,first,first,work,atClear))return refuse(TEXT("AdjacentFirstBounds"),work.Failure(),first,last,mask);
                const uint16 firstMask=ClearMask(graph,atClear,mask,work,memo,witnessCorner,&firstWitness);
                if(firstMask!=AllCorners)return refuse(TEXT("AdjacentFirstNotClear"),work.Failure(),first,last,mask,firstMask,0,&atClear);
                if(!Bounds(input,last,last,work,atInside))return refuse(TEXT("AdjacentLastBounds"),work.Failure(),first,last,mask,firstMask,0,&atClear);
                const uint16 endMask=ClearMask(graph,atInside,mask,work,memo,witnessCorner,&lastWitness);
                if(work.Stopped||work.Invalid)return work.Failure();
                if(endMask==AllCorners)
                {
                    // Adjacent native times admit only the two proved poses.
                    // For continuous clearance, close every real subcell as
                    // well, using the same original reference motion.
                    const bool chord=witnessCorner!=INDEX_NONE&&ReferenceAdjacentChord(
                        input,graph,first,last,witnessCorner,atClear,atInside,firstWitness,lastWitness,work);
                    if((!chord&&(work.Stopped||work.Invalid||
                        !ReferenceAdjacentClear(input,graph,first,last,mask,work,memo,visited,maxTimeCells,extendOrdinary,traceOrdinary)))||
                        !work.Pay())
                        return refuse(TEXT("AdjacentReferenceNotClear"),work.Failure(),first,last,mask,firstMask,endMask,&atClear,&atInside);
                    if(chord&&traceOrdinary)
                        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxOrdinaryReferenceChord] First=%.17g Last=%.17g Corner=%d Work=%llu"),
                            double(FloorNativeSweepDetail::Seconds(first)),double(FloorNativeSweepDetail::Seconds(last)),
                            witnessCorner,budget.Used-work.Began);
                    clear.Add(node.Cell);covered=last;continue;
                }
                int32 selected=INDEX_NONE;
                for(int32 k=0;k<8;++k)
                {
                    const auto bit=uint16((1u<<k)|(1u<<(k+8)));
                    // Require every OTHER corner to stay clear throughout the
                    // final interval. The unique event feature is this vertex.
                    const TCHAR* traceEdge=trace?(traceOrdinary?TEXT("[CurvedBoxOrdinaryInsideEdge]"):
                        TEXT("[CurvedBoxMixedFloorInsideEdge]")):nullptr;
                    if(uint16(mask|bit)!=AllCorners)continue;
                    bool inside=InsideEdge(graph,atInside,k,work,memo,traceEdge);
                    if(!inside&&!mixedConvexity&&!work.Stopped&&!work.Invalid&&
                        CVarOrdinaryCurvedBoxAdjacentConsumedSingleton.GetValueOnAnyThread()==1&&work.Pay())
                    {
                        const float seconds=FloorNativeSweepDetail::Seconds(last);
                        FGeneralBoxMotionBounds exactConsumed;
                        if(BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,seconds,seconds,exactConsumed,false,true)&&
                            exactConsumed.Corners.Num()==32)
                        {
                            // Retain the original continuous-reference endpoint; the retry
                            // narrows only the two consumed corners on the same rigid edge.
                            exactConsumed.Corners[16+k]=atInside.Corners[16+k];
                            exactConsumed.Corners[16+(k^4)]=atInside.Corners[16+(k^4)];
                            inside=InsideEdge(graph,exactConsumed,k,work,memo,traceEdge);
                        }
                        if(traceOrdinary)
                            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxOrdinaryAdjacentConsumedSingleton] InsideBits=%08x Corner=%d Proved=%d Work=%llu"),
                                KinematicCenterDetail::Bits(seconds),k,int32(inside),budget.Used-work.Began);
                    }
                    if(inside)
                    {
                        if(selected!=INDEX_NONE)return refuse(TEXT("MultipleArrivalCorners"),EResult::Unproved,first,last,mask,firstMask,endMask,&atClear,&atInside);
                        selected=k;
                    }
                }
                if(selected==INDEX_NONE)
                {
                    if(allowNativeSplit202&&mixedConvexity&&!work.Stopped&&!work.Invalid)
                    {
                        const TCHAR* splitStage=TEXT("none");
                        const bool proved=TrySplit202(input,product,graph,first,end,full.Hull,work,memo,
                            numeric.ControlFingerprint(),reuseExactEvaluations,clear,out,splitStage);
                        UE_LOG(LogTemp,Display,TEXT("[NativeBoxSplitArrival202] Stage=%s Proved=%d Work=%llu"),
                            splitStage,int32(proved),budget.Used-work.Began);
                        if(proved)return EResult::NativeFirstSeparateReferences;
                    }
#if !UE_BUILD_SHIPPING
                    if(reference205&&!mixedConvexity&&!adjacentBandSeam&&!work.Stopped&&!work.Invalid)
                    {
                        const TCHAR* stage=TEXT("capture");
                        const bool proved=TryBuildReference205(input,product,*reference205,graph,full.Hull,first,end,
                            work,memo,numeric.ControlFingerprint(),reuseExactEvaluations,out,stage);
                        UE_LOG(LogTemp,Display,TEXT("[NativeBox205ReferenceTiming] Stage=%s Proved=%d ReferenceClear=2f343419 NativeInside=2f343425 Work=%llu"),
                            stage,int32(proved),budget.Used-work.Began);
                        if(proved)return EResult::ReferenceFirst205;
                    }
                    if(!mixedConvexity)
                        Case205RemainderObservation::Capture(input,product,
                            FloorNativeSweepDetail::Seconds(first),FloorNativeSweepDetail::Seconds(last));
#endif
                    return refuse(TEXT("NoUniqueAdjacentInsideEdge"),work.Failure(),first,last,mask,firstMask,endMask,&atClear,&atInside);
                }
                const auto corner=TimeDetail::Hull(current.Corners[selected],current.Corners[16+selected]);
                Interval u,v;
                if(!graph.InOriginalPatch(corner)||!graph.BoundParameter(corner.Axis[1],work,memo,v)||!work.Pay()||
                    !AffineReferenceDetail::FreeParameter(product,corner,u)||!(0.<u.Lower&&u.Upper<1.))
                    return refuse(TEXT("AdjacentResponseChart"),work.Failure(),first,last,mask,firstMask,endMask,&atClear,&atInside);
                FYZ lower,upper,dlo,dhi;
                if(!memo.At(v.Lower,work,lower)||!memo.At(v.Upper,work,upper)||
                    !FGraph::Evaluate(graph.D,v.Lower,work,dlo)||!FGraph::Evaluate(graph.D,v.Upper,work,dhi))
                    return refuse(TEXT("AdjacentResponsePointOrDerivative"),work.Failure(),first,last,mask,firstMask,endMask,&atClear,&atInside);
                // Enclose the continuous-reference event at the unique
                // vertex. The discrete native inside edge is a separate
                // witness of existence at last, not this response point.
                const FIntervalPoint point{{corner.Axis[0],{lower.Y.Lower,upper.Y.Upper,true},{lower.Z.Lower,upper.Z.Upper,true}}};
                const FIntervalPoint normal{{A::Interval::Exact(0.),Interval{-dhi.Z.Upper,-dlo.Z.Lower,true},Interval{dhi.Y.Lower,dlo.Y.Upper,true}}};
                if(!Detail::ValidPoint(point)||!Detail::ValidPoint(normal)||!work.Pay()||
                    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric.ControlFingerprint())
                    return work.Stopped?EResult::Work:EResult::Arithmetic;
                TSharedPtr<FProof,ESPMode::ThreadSafe> proof(new FProof);
                proof->Input=input;proof->Graph=graph;proof->Numeric=numeric.ControlFingerprint();
                proof->Graph.NominalInverseProposal=false;
                proof->ReuseExactEvaluations=reuseExactEvaluations;
                proof->ClearNativeCenter=atClear.NativeCenter;
                proof->ClearOrdinal=first;proof->InsideOrdinal=last;proof->SearchOrdinal=end;proof->Corner=selected;
                proof->U=u;proof->V=v;proof->SurfacePoint=point;proof->OutwardRawNormal=normal;proof->Envelope=full.Hull;
                proof->Prefix=MoveTemp(clear);out=MoveTemp(proof);
                reportCase585Frontier(TEXT("closed"),first,last);
                return EResult::FirstAdjacent;
            }
            // Both children stay closed, including an uncertain cell
            // proposed by the focused path.
            const uint32 middle=MidpointOrdinal(first,last);
            if(middle<=first||middle>=last||pending.Num()>30)return EResult::Arithmetic;
            if(!work.Pay())return work.Failure();
            pending.Add({{middle,last},mask});pending.Add({{first,middle},mask});
        }
        return refuse(TEXT("PrefixTraversalEnded"),EResult::Unproved,covered,lastInside);
    }

    bool MatchesPrefix() const
    {
#if !UE_BUILD_SHIPPING
        if(Reference205.IsSet())return MatchesReference205();
#endif
        return !Prefix.IsEmpty()&&Prefix[0].First==0&&
            (SplitReference.IsSet()?(SplitReference->Matches(ClearOrdinal,InsideOrdinal)&&
                Prefix.Last().Last==SplitReference->CommonClear):Prefix.Last().Last==ClearOrdinal);
    }

    bool Matches(const FGeneralBoxMotionInput& input,const FProduct& product) const
    {
        uint32 horizon=0;
        return FloorNativeSweepDetail::Ordinal(input.Horizon,horizon)&&InsideOrdinal<=SearchOrdinal&&SearchOrdinal<=horizon&&
            GeneralPhysicalFloorClearDetail::SameInput(Input,input)&&Graph.SameProduct(product)&&
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==Numeric&&
            ClearOrdinal>0&&InsideOrdinal==ClearOrdinal+1&&Corner>=0&&Corner<8&&MatchesPrefix()&&
            (!Serial3||(!SplitReference.IsSet()&&Serial3->Native.Clear==ClearOrdinal&&Serial3->Native.Inside==InsideOrdinal&&
                Serial3->Chart.ImmutableProduct==&product&&Case202Serial3::VerifyAttestedGraphFacts(
                    Serial3->Seal.Get(),Serial3->Live,Serial3->Chart,Serial3->Raw,Serial3->Normalized,Serial3->Native,horizon)));
    }
    // The actual primitive may be shorter because another owner won the
    // selector. No acceleration, initial pose, geometry, local configuration
    // or float operation changes when restricting this same global prefix.
    bool CoversOriginalPrefix(const FGeneralBoxMotionInput& actual,const FProduct& product) const
    {
        uint32 end=0;
        if(!FloorNativeSweepDetail::Ordinal(actual.Horizon,end)||end>ClearOrdinal)return false;
        auto rejoin=actual;rejoin.Horizon=Input.Horizon;
        return Matches(rejoin,product);
    }
    // Pure independent endpoint check. The consumer must ALSO verify the
    // actual epoch/owner/shape/pose and that the full proxy is covered by its
    // competing-provider and external-world envelope.
    bool ProvePhysicalEndpoint(const Analytic::FWorldQuery& query,const FVector& extent,
        FDistanceBudget& budget,int32 maximumWork,const FProduct* case205Product,
        const Analytic::Prepared::FReferenceTransport* case205Reference) const
    {
        const auto& center=query.Start;
        const auto& rotation=query.Rotation;
        if(maximumWork<=0||budget.ConsumptionFloor<0||budget.Remaining<=budget.ConsumptionFloor||
            !FloorNativeSweepDetail::SameVector(extent,Input.PhysicalHalfExtent)||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=Numeric)
            return false;
        const int32 allowed=FMath::Min(maximumWork,budget.Remaining-budget.ConsumptionFloor);
        FScopedDistanceBudgetFloor limit(budget,budget.Remaining-allowed);
        if(!limit.IsValid())return false;
        FAccount work{budget,budget.Used,maximumWork};
        FPhysicalBodyBoxPoseInput pose;pose.BeforeCenter=pose.AfterCenter=center;
        pose.BeforeRotation=pose.AfterRotation=rotation;pose.PhysicalHalfExtent=extent;
        FPhysicalBodyBoxPoseBounds bounds;
        if(!BuildPhysicalBodyBoxPoseBounds(pose,budget,bounds)||!bounds.Known)return false;
        if(SplitReference.IsSet()||Serial3)
        {
            // Rejoin the actual setter pose independently. Keep both original
            // predicates: compiled body Rotate/Sum and ideal unit body pose.
            namespace R=Case202SplitReference;
            namespace G=FactoredLowerCapGraphGap;
            G::FControl controls[6];
            for(int32 i=0;i<6;++i)controls[i]={Graph.P[i].Y,Graph.P[i].Z};
            G::FProfile profile;
            const auto pay=[&](){return work.Pay();};
            if(!G::FProfile::TryBuild(controls,pay,profile))return false;
            R::FInput body{};
            for(int32 d=0;d<3;++d){body.Position[d]=center[d];body.HalfExtent[d]=extent[d];}
            body.Rotation[0]=rotation.X;body.Rotation[1]=rotation.Y;body.Rotation[2]=rotation.Z;body.Rotation[3]=rotation.W;
            for(int32 k=0;k<8;++k)
            {
                FVector local;
                for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?extent[d]:-extent[d];
                const auto native=Detail::Point(center+rotation.RotateVector(local));
                R::FBounds tight;
                if(!work.Pay()||!R::TryBound(body,Interval::Exact(0.),k,tight))return false;
                FIntervalPoint unit;
                for(int32 d=0;d<3;++d)unit.Axis[d]=tight.Normalized[d];
                if(!Split202GraphSign(Graph,profile,native,true,work)||!Split202GraphSign(Graph,profile,unit,true,work))return false;
            }
            return RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==Numeric;
        }
#if !UE_BUILD_SHIPPING
        bool exact205=false;
        constexpr int32 active205=Case205ReferenceTiming::ActiveCorner;
        if(case205Product&&case205Reference&&MatchesReference205())
        {
            auto physicalQuery=query;
            physicalQuery.HalfExtent=extent;
            FBoxVolumeGroup fixed;
            exact205=work.Pay()&&NativeGroupedBoxDetail::FixedQueryGroup(physicalQuery,fixed)&&fixed.Corners.Num()==8&&
                ProveCase205ActiveProjectionCorner(physicalQuery,*case205Reference,*case205Product,
                    fixed.Corners[active205],bounds.NativeCorners[active205],bounds.UnitReferenceCorners[active205],work);
        }
#endif
        FGraph::FPositionMemo memo(Graph,ReuseExactEvaluations);
        for(int32 k=0;k<8;++k)
        {
#if !UE_BUILD_SHIPPING
            if(exact205&&k==active205)continue;
#endif
            if(!Graph.Sign(bounds.NativeCorners[k],true,work,memo)||
                !Graph.Sign(bounds.UnitReferenceCorners[k],true,work,memo))return false;
        }
        if(RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=Numeric)
            return false;
#if !UE_BUILD_SHIPPING
        if(exact205)
        {
            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCase205Q112ActualEndpoint] Provider=1407 Native=1 Raw=1 Unit=1 CachedAxes=1 Work=%llu"),
                budget.Used-work.Began);
        }
#endif
        return true;
    }
    bool ProvePhysicalCorrectionCorridor(FBoxPoseCorridorInput input,FDistanceBudget& budget,bool pureRotation,bool ordinary,
        const FIntervalPoint* fixtureRotationEnvelope=nullptr) const
    {
        using namespace FloorNativeSweepDetail;
        const bool allowedPose=pureRotation?SameVector(input.BeforeCOM,input.AfterCOM):
            SameQuat(input.BeforeRotation,input.AfterRotation);
        if(!allowedPose||!SameQuat(input.LocalRotation,Input.LocalRotation)||
            !SameVector(input.LocalCOM,Input.LocalCOM)||!SameVector(input.LocalOffset,Input.LocalOffset)||
            !SameVector(input.PhysicalHalfExtent,Input.PhysicalHalfExtent)||!SameVector(input.QueryHalfExtent,Input.QueryHalfExtent)||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=Numeric)
            return false;

        // The original shell envelope is checked independently for the other
        // providers. These are the actual physical corners and original DAG.
        input.DomainShell=0.;
        input.OverlapShell=0.;
        FAccount work{budget,budget.Used,768};
        FGraph::FPositionMemo memo(Graph,ReuseExactEvaluations);
        const auto prove=[&](const FBoxPoseCorridorInput& cell)
        {
            FIntervalPoint hull;
            TArray<FIntervalPoint,TInlineAllocator<16>> corners;
            if(!work.Pay()||!BuildBoxPoseCorridorBounds(cell,hull,&corners)||corners.Num()!=16||
                (fixtureRotationEnvelope&&!BoxPoseCorridorWithin(*fixtureRotationEnvelope,hull)))return false;
            for(int32 corner=0;corner<8;++corner)
                if(!Graph.Sign(corners[corner],true,work,memo))return false;
            return true;
        };
        if(!prove(input))
        {
            if(!pureRotation||work.Stopped||work.Invalid||
                (!fixtureRotationEnvelope&&(!ordinary||
                    CVarOrdinaryCurvedBoxRotationCorridorHalves.GetValueOnAnyThread()!=1)))return false;
            const auto& a=input.BeforeRotation;const auto& b=input.AfterRotation;
            const double first[]={a.X,a.Y,a.Z,a.W},last[]={b.X,b.Y,b.Z,b.W};
            double leftEnd[4],rightStart[4];
            if(!work.Pay())return false;
            for(int32 d=0;d<4;++d)
            {
                const auto start=A::Interval::Exact(first[d]),finish=A::Interval::Exact(last[d]);
                if(!A::Valid(start)||!A::Valid(finish)||A::Bits(start.Lower)!=A::Bits(start.Upper)||
                    A::Bits(finish.Lower)!=A::Bits(finish.Upper))return false;
                const auto middle=A::Multiply(A::Interval::Exact(.5),A::Add(start,finish));
                if(!A::Valid(middle))return false;
                // Both interval boxes must contain the exact raw midpoint.
                // Intersect its enclosure with the known original axis span.
                const double lower=FMath::Max(FMath::Min(first[d],last[d]),middle.Lower);
                const double upper=FMath::Min(FMath::Max(first[d],last[d]),middle.Upper);
                if(lower>upper)return false;
                leftEnd[d]=first[d]<=last[d]?upper:lower;
                rightStart[d]=first[d]<=last[d]?lower:upper;
            }
            auto left=input,right=input;
            // These componentwise hull limits are values-only. Do not
            // normalize them or substitute them for either real pose.
            left.AfterRotation=FQuat(leftEnd[0],leftEnd[1],leftEnd[2],leftEnd[3]);
            right.BeforeRotation=FQuat(rightStart[0],rightStart[1],rightStart[2],rightStart[3]);
            const bool leftClear=prove(left);
            const bool rightClear=leftClear&&prove(right);
            if(CVarOrdinaryCurvedBoxTrace.GetValueOnAnyThread()==1)
                UE_LOG(LogTemp,Display,TEXT("[CurvedBoxOrdinaryRotationCorridorHalves] Left=%d Right=%d Work=%llu"),
                    int32(leftClear),int32(rightClear),budget.Used-work.Began);
            if(!rightClear)return false;
        }
        return RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==Numeric;
    }
    bool ProvePhysicalObstructedEndpoint(FBoxPoseCorridorInput input,FDistanceBudget& budget) const
    {
        if(Corner<0||Corner>=8||
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=Numeric)
            return false;
        // This rejects a proposed physical pose. No query shell is a contact witness.
        input.DomainShell=0.;input.OverlapShell=0.;
        FAccount work{budget,budget.Used,128};
        FIntervalPoint hull;
        TArray<FIntervalPoint,TInlineAllocator<16>> corners;
        if(!work.Pay()||!BuildBoxPoseCorridorBounds(input,hull,&corners)||corners.Num()!=16)return false;
        const auto& a=corners[Corner];const auto& b=corners[Corner^4];
        FGraph::FPositionMemo memo(Graph,ReuseExactEvaluations);
        return Graph.InOriginalPatch(a)&&Graph.InOriginalPatch(b)&&
            Graph.Sign(a,false,work,memo)&&Graph.Sign(b,true,work,memo)&&
            RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==Numeric;
    }
    bool ProveNegativeXCompanionAdjacent(FDistanceBudget& budget) const
    {
        FAccount work{budget,budget.Used,512};
        FGeneralBoxMotionBounds bounds;
        if(!Bounds(Input,ClearOrdinal,InsideOrdinal,work,bounds))return false;
        FGraph::FPositionMemo memo(Graph,ReuseExactEvaluations);
        for(int32 family=0;family<2;++family)
        {
            const auto* corners=bounds.Corners.GetData()+16*family;
            bool negative[8];int32 negativeCount=0;
            for(int32 k=0;k<8;++k)
            {
                const auto x=corners[k].Axis[0];
                negative[k]=x.Upper<0.;
                if(!negative[k]&&!(x.Lower>0.))return false;
                if(negative[k])
                {
                    ++negativeCount;
                    if(!Graph.Sign(corners[k],true,work,memo))return false;
                }
            }
            if(negativeCount==0||negativeCount==8)return false;
            // Clipping a rigid Box by X<=0 leaves its negative vertices and
            // intersections of the original twelve edges with X=0.
            for(int32 k=0;k<8;++k)for(int32 axis=0;axis<3;++axis)
            {
                const int32 other=k^(1<<axis);
                if(k>=other||negative[k]==negative[other])continue;
                const auto& a=corners[negative[k]?k:other];
                const auto& b=corners[negative[k]?other:k];
                Interval weight;
                if(!work.Pay()||!ContactDetail::ProjectionDividePositive(A::Negate(a.Axis[0]),
                    A::Subtract(b.Axis[0],a.Axis[0]),weight)||!(weight.Lower>0.&&weight.Upper<1.))return false;
                FIntervalPoint seam;
                seam.Axis[0]=Interval::Exact(0.);
                for(int32 d=1;d<3;++d)
                    seam.Axis[d]=A::Add(a.Axis[d],A::Multiply(weight,A::Subtract(b.Axis[d],a.Axis[d])));
                if(!Detail::ValidPoint(seam)||!Graph.Sign(seam,true,work,memo))return false;
            }
        }
        return RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==Numeric;
    }
    bool ProveMirrorLiteralProxy(const FWorldQuery& query,const FProduct& companion,
        FDistanceBudget& budget,Interval (&witness)[2],int32 (&witnessCorners)[2]) const
    {
        // Search brackets only. No times or decisions are used without both
        // clearance and a real edge crossing on the literal fixed-Q query.
        constexpr double clear=35./64.,inside[]={71./128.,9./16.};
        FAccount work{budget,budget.Used,640};
        auto fixed=query;fixed.Start=fixed.End=FVector::ZeroVector;
        FBoxVolumeGroup footprint;
        if(!work.Pay()||!NativeGroupedBoxDetail::FixedQueryGroup(fixed,footprint)||footprint.Corners.Num()!=8)return false;
        FGraph::FPositionMemo memo(Graph,ReuseExactEvaluations);
        const auto point=[&](double u,int32 k)
        {
            FIntervalPoint out;
            for(int32 d=0;d<3;++d)
            {
                const auto a=Interval::Exact(query.Start[d]),b=Interval::Exact(query.End[d]);
                const auto center=A::Add(a,A::Multiply(Interval::Exact(u),A::Subtract(b,a)));
                out.Axis[d]=A::Add(center,footprint.Corners[k].Axis[d]);
            }
            return out;
        };
        const auto shell=A::Add(Interval::Exact(query.DomainTolerance),Interval::Exact(query.InitialOverlapTolerance));
        if(!A::Valid(shell)||query.DomainTolerance<0.||query.InitialOverlapTolerance<0.)return false;
        for(int32 k=0;k<8;++k)
        {
            if(!work.Pay())return false;
            auto initial=point(0.,k);
            for(int32 d=0;d<3;++d)initial.Axis[d]=A::Add(initial.Axis[d],Interval{-shell.Upper,shell.Upper,true});
            const auto lastClear=point(clear,k);
            if(!Detail::ValidPoint(initial)||!Detail::ValidPoint(lastClear)||
                !Graph.Sign(initial,true,work,memo)||!Graph.Sign(lastClear,true,work,memo))return false;
        }
        // Each literal corner is a line. The allowed epigraph of the graph
        // is convex, so the two clear endpoints cover the whole literal prefix.
        for(int32 side=0;side<2;++side)
        {
            const auto inChart=[&](const FIntervalPoint& p)
            {
                if(!Detail::ValidPoint(p))return false;
                if(!side)return Graph.InOriginalPatch(p);
                const auto lo=companion.Start.Lower<companion.End.Lower?companion.Start:companion.End;
                const auto hi=companion.Start.Lower<companion.End.Lower?companion.End:companion.Start;
                return lo.Upper<p.Axis[0].Lower&&p.Axis[0].Upper<hi.Lower&&
                    Graph.P[0].Y.Upper<p.Axis[1].Lower&&p.Axis[1].Upper<Graph.P[5].Y.Lower;
            };
            for(int32 k=0;k<8;++k)
            {
                if(!work.Pay())return false;
                const auto a=point(inside[side],k),b=point(inside[side],k^4);
                if(!inChart(a)||!inChart(b)||!Graph.Sign(a,false,work,memo)||!Graph.Sign(b,true,work,memo))continue;
                witness[side]={clear,inside[side],true};witnessCorners[side]=k;break;
            }
            if(!A::Valid(witness[side]))return false;
        }
        return RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==Numeric;
    }
    bool ProveNegativeXProxyShellAtClear(const FWorldQuery& query,const FProduct& companion,
        FDistanceBudget& budget,int32& witnessCorner) const
    {
        FAccount work{budget,budget.Used,256};
        const double shell=query.InitialOverlapTolerance;
        if(!A::Finite(shell)||!(shell>0.))return false;
        FGeneralBoxMotionBounds bounds;
        if(!Bounds(Input,ClearOrdinal,ClearOrdinal,work,bounds))return false;
        FGraph::FPositionMemo memo(Graph,ReuseExactEvaluations);
        const auto lo=companion.Start.Lower<companion.End.Lower?companion.Start:companion.End;
        const auto hi=companion.Start.Lower<companion.End.Lower?companion.End:companion.Start;
        const auto shift=[&](int32 k)
        {
            // Consumed QUERY-extent corner plus one point of the original
            // world-axis overlap shell. Never a physical vertex witness.
            auto p=bounds.Corners[8+k];
            p.Axis[1]=A::Add(p.Axis[1],Interval::Exact(shell));
            p.Axis[2]=A::Subtract(p.Axis[2],Interval::Exact(shell));
            return p;
        };
        const auto inChart=[&](const FIntervalPoint& p)
        {
            return Detail::ValidPoint(p)&&lo.Upper<p.Axis[0].Lower&&p.Axis[0].Upper<hi.Lower&&
                Graph.P[0].Y.Upper<p.Axis[1].Lower&&p.Axis[1].Upper<Graph.P[5].Y.Lower;
        };
        for(int32 k=0;k<8;++k)
        {
            if(!work.Pay())return false;
            const auto a=shift(k),b=shift(k^4);
            if(inChart(a)&&inChart(b)&&Graph.Sign(a,false,work,memo)&&Graph.Sign(b,true,work,memo))
            {witnessCorner=k;return true;}
        }
        return false;
    }
    float ClearSeconds() const {return FloorNativeSweepDetail::Seconds(ClearOrdinal);}
    const FIntervalPoint& ClearNativeCenterForInspection() const {return ClearNativeCenter;}
    float InsideSeconds() const {return FloorNativeSweepDetail::Seconds(InsideOrdinal);}
    float SearchSecondsForInspection() const {return FloorNativeSweepDetail::Seconds(SearchOrdinal);}
    bool HasSeparateNativeReferences() const {return SplitReference.IsSet();}
#if !UE_BUILD_SHIPPING
    bool HasCase205ReferenceTiming() const {return MatchesReference205();}
#endif
    bool TryProveRebasedRoof202(const FGeneralBoxMotionInput& input,const FIntervalPoint& envelope,
        const FGeneralBoxPhysicalFloorClearBounds& floor,FDistanceBudget& budget,
        PreparedExecution::FNativeBoxRebasedRoof202& evidence) const
    {
        namespace R=Case202SplitReference;
        namespace G=FactoredLowerCapGraphGap;
        evidence={};
        constexpr uint32 bulkBits=0x3a52ca00u,rawClearBits=0x3a52d7cbu,rawInsideBits=0x3a52d7ccu;
        constexpr uint32 localBits=0x3a52d7d1u;
        constexpr int32 sensitive=5,partner=sensitive^4,maximumWork=8192;
        if(!SplitReference.IsSet()||!PreparedExecution::Requested()||KinematicCenterDetail::Bits(input.Horizon)!=localBits||
            input.LocalRotation.X!=0.||input.LocalRotation.Y!=0.||input.LocalRotation.Z!=0.||input.LocalRotation.W!=1.||
            !floor.Matches(input,FVector::ZeroVector,FVector::UpVector)||budget.Remaining-budget.ConsumptionFloor<maximumWork)
            return false;
        FScopedDistanceBudgetFloor limit(budget,budget.Remaining-maximumWork);
        if(!limit.IsValid())return false;
        FAccount work{budget,budget.Used,maximumWork};
        const auto pay=[&](){return work.Pay();};
        const auto numeric=[&]()
        {return RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==Numeric;};
        if(!numeric())return false;
        uint32 cutoff=0,last=0;
        if(!FloorNativeSweepDetail::Ordinal(KinematicCenterDetail::FromBits(bulkBits),cutoff)||
            !FloorNativeSweepDetail::Ordinal(input.Horizon,last)||cutoff>=last||last-cutoff>4095)return false;
        G::FControl controls[6];
        for(int32 i=0;i<6;++i)controls[i]={Graph.P[i].Y,Graph.P[i].Z};
        G::FProfile profile;
        if(!G::FProfile::TryBuild(controls,pay,profile))return false;
        const auto clear=[&](FIntervalPoint point)
        {
            if(!Detail::ValidPoint(point))return false;
            // The convex left extension is nondecreasing; its height at the
            // interval's upper Y bounds every earlier Y in this enclosure.
            point.Axis[1]=Interval::Exact(point.Axis[1].Upper);
            return Split202GraphSign(Graph,profile,point,true,work);
        };
        R::FInput reference{};
        for(int32 d=0;d<3;++d)
        {
            reference.Position[d]=input.Owner.Location[d];reference.Velocity[d]=input.Owner.Velocity[d];
            reference.Acceleration[d]=input.Owner.Acceleration[d];reference.AngularVelocity[d]=input.Owner.AngularVelocity[d];
            reference.AngularAcceleration[d]=input.Owner.AngularAcceleration[d];reference.LocalCOM[d]=input.LocalCOM[d];
            reference.LocalOffset[d]=input.LocalOffset[d];reference.HalfExtent[d]=input.PhysicalHalfExtent[d];
        }
        reference.Rotation[0]=input.Owner.Rotation.X;reference.Rotation[1]=input.Owner.Rotation.Y;
        reference.Rotation[2]=input.Owner.Rotation.Z;reference.Rotation[3]=input.Owner.Rotation.W;
        const auto referencePoint=[&](Interval time,int32 k,bool unit,FIntervalPoint& point)
        {
            R::FBounds bounds;
            if(!work.Pay()||!R::TryBound(reference,time,k,bounds))return false;
            const auto& p=unit?bounds.Normalized:bounds.Raw;
            for(int32 d=0;d<3;++d)point.Axis[d]=p[d];
            return true;
        };
        const double rawClear=KinematicCenterDetail::FromBits(rawClearBits);
        const auto rawInside=Interval::Exact(KinematicCenterDetail::FromBits(rawInsideBits));
        FIntervalPoint point;
        for(int32 k=0;k<8;++k)
        {
            const Interval rawPrefix{0.,k==sensitive?rawClear:double(input.Horizon),true};
            const Interval unitPrefix{0.,double(input.Horizon),true};
            if(!referencePoint(rawPrefix,k,false,point)||!clear(point)||
                !referencePoint(unitPrefix,k,true,point)||!clear(point))return false;
        }
        if(!referencePoint(rawInside,sensitive,false,point)||!Graph.InOriginalPatch(point)||
            !Split202GraphSign(Graph,profile,point,false,work)||
            !referencePoint(rawInside,partner,false,point)||!Graph.InOriginalPatch(point)||!clear(point))return false;

        FGeneralBoxMotionBounds whole,bulk;
        if(!Bounds(input,0,last,work,whole)||!Bounds(input,0,cutoff,work,bulk))return false;
        for(int32 k=0;k<8;++k)
        {
            auto native=(k==sensitive?bulk:whole).Corners[k];
            const auto gap=floor.GetMinimumGapForInspection(k);
            if(!A::Valid(gap))return false;
            native.Axis[2].Lower=FMath::Max(native.Axis[2].Lower,gap.Lower);
            if(!clear(native))return false;
        }

        FVector local;
        for(int32 d=0;d<3;++d)local[d]=(sensitive&(1<<d))?input.PhysicalHalfExtent[d]:-input.PhysicalHalfExtent[d];
        FIntervalPoint suffix;
        for(uint32 ordinal=cutoff;ordinal<=last;++ordinal)
        {
            if(!work.Pay())return false;
            auto pose=input.Owner.Integrate(FloorNativeSweepDetail::Seconds(ordinal));
            pose.Location-=pose.Rotation.RotateVector(input.LocalCOM);
            pose.Location+=pose.Rotation.RotateVector(input.LocalOffset);
            pose.Rotation=pose.Rotation*input.LocalRotation;
            if(!numeric()||!work.Pay())return false;
            const auto corner=Detail::Point(pose.Location+pose.Rotation.RotateVector(local));
            if(!Detail::ValidPoint(corner))return false;
            for(int32 d=0;d<3;++d)
                if(corner.Axis[d].Lower<envelope.Axis[d].Lower||corner.Axis[d].Upper>envelope.Axis[d].Upper)return false;
            suffix=ordinal==cutoff?corner:TimeDetail::Hull(suffix,corner);
        }
        if(!clear(suffix)||!numeric())return false;
        evidence={bulkBits,rawClearBits,rawInsideBits,localBits,localBits};
        return true;
    }
    int32 PhysicalCornerIndex() const {return Corner;}
    const auto& UForInspection() const {return U;}
    const auto& VForInspection() const {return V;}
    const auto& SurfacePointForInspection() const {return SurfacePoint;}
    const auto& OutwardRawNormalForInspection() const {return OutwardRawNormal;}
    const auto& EnvelopeForInspection() const {return Envelope;}
    const auto& PrefixForInspection() const {return Prefix;}
};

// Fixture: captured owner from GutterMotionClosure18Body585ONProfile1, last
// StadiumGeneralBoxCapturedState, plus authenticated provider726 FProduct.
// Require Matches() and exact adjacent bits 3B4AD7A7 / 3B4AD7A8, corner 1.
// A different numerical result is an audit failure, not a widened tolerance.
// Costs: max 128 time cells, 42 fixed Bernstein evaluations per Y inversion,
// at most 16 signs per cell with inherited clear bits, all paid on the caller
// account with a separate non-refunding maximumWork. Inside preflight adds
// at most 16 signs, witness bounds four fixed net evaluations and two inverses.
inline FGeneralBoxMotionInput MakeCase585FixtureInput()
{
    FGeneralBoxMotionInput input;
    input.Owner.Location={1829.4048487886662,3897.4873080048264,33.995433443121954};
    input.Owner.Rotation={0.041964308065689307,-0.042655155106700308,0.71002785316361205,0.70162666876570734};
    input.Owner.Velocity={-34.943684384731782,2226.5610140322829,271.10338961945814};
    input.Owner.Acceleration={-995.73133227820188,-1794.2762771902273,4878.3615264025539};
    input.Owner.AngularVelocity={5.4843399801029902,-0.022298187836458545,0.41414704328673391};
    input.Owner.AngularAcceleration={-0.60098596048397657,-2.1794787580767099,7.5888724185491165};
    input.LocalCOM={-14.008570000000001,0,17.415800000000001};
    input.LocalOffset={-5,0,29.510000000000002};
    input.LocalRotation=FQuat::Identity;
    input.PhysicalHalfExtent={65.659999999999997,43.5852,15.9472};
    input.QueryHalfExtent={65.660003662109375,43.585201263427734,15.947199821472168};
    input.Horizon=KinematicCenterDetail::FromBits(0x3B5A740E);
    return input;
}
inline bool VerifyCase585Fixture(const FProduct& authenticated726,FDistanceBudget& budget,int32 maximumWork)
{
    const auto input=MakeCase585FixtureInput();
    FProof::FPtr result;
    return FProof::Build(input,authenticated726,budget,maximumWork,result)==EResult::FirstAdjacent&&result&&
        result->Matches(input,authenticated726)&&result->PhysicalCornerIndex()==1&&
        KinematicCenterDetail::Bits(result->ClearSeconds())==0x3B4AD7A7&&
        KinematicCenterDetail::Bits(result->InsideSeconds())==0x3B4AD7A8;
}
}

// Private values-only producer. Include outside namespaces in its eventual TU.
#include "PreparedNativeSolidRequest.h"
#include "IAmSpeed/World/Analytic/CanonicalAxialPlaneBounds.h"
#include "IAmSpeed/World/Analytic/ScalarSphereContactBounds.h"

namespace Speed::PreparedExecution
{
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandIgnoredTailClear(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandIgnoredTailClear"),0,
    TEXT("Default OFF. Prove the complete unfiltered 607/726 residual motion of the authenticated ignored case 976 Box."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandStep51Remainder(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandStep51Remainder"),0,
    TEXT("Default OFF. Prove native timing on the fresh 976 step-51 remainder without extending pose admission."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandStep51Inverse(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandStep51Inverse"),0,
    TEXT("Default OFF. Try paid nominal Y proposals, with directed verification, on the existing case 976 step-51 graph budget."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandStep51(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandStep51"),0,
    TEXT("Default OFF. Prove the full-horizon case 976 step-51 timing with a verified H/4 search proposal."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandContinuation(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandContinuation"),0,
    TEXT("Default OFF. Retry the fully proved native 607/726 continuation in case 976 frame 50."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandFreshRemainder(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandFreshRemainder"),0,
    TEXT("Default OFF. Qualify the exact case 976 fresh remainder after the native seam correction."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandTiming(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandTiming"),0,
    TEXT("Default OFF. Case 976 native Box timing with separately retained original reference intersection witnesses."));
static TAutoConsoleVariable<int32> CVarCurvedBoxMirrorArcTimingTrace(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxMirrorArcTimingTrace"),0,
    TEXT("Trace physical timing and literal witnesses without changing the mirror-arc shell veto. Default off."));
static TAutoConsoleVariable<int32> CVarCurvedBoxTwoBandPrefix(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxTwoBandPrefix"),0,
    TEXT("Allow an authenticated ordinary two-band curved arrival after the complete sweep refuses. Default OFF."));
#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarCurvedBoxCase585Owner88FocusedTraversal(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxCase585Owner88FocusedTraversal"),0,
    TEXT("Default OFF. Propose 3/32 focused cells for the authenticated case-585 owner88 726/607 prefix only."));
static TAutoConsoleVariable<int32> CVarCurvedBoxCase585Owner88RemainderTraversal(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxCase585Owner88RemainderTraversal"),0,
    TEXT("Default OFF. Propose 1/8 focused cells for the authenticated case-585 owner88 noninitial 726/607 requests."));
#endif
static TAutoConsoleVariable<int32> CVarCurvedBoxFirstArrival(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxFirstArrival"),0,
    TEXT("Exact convex retained Box first-arrival experiment. Default off."));
static TAutoConsoleVariable<int32> CVarCurvedBoxPureRotationCorridor(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxPureRotationCorridor"),0,
    TEXT("Prove actual pure-rotation corrections against the authenticated curved Box graph and complete world. Default off."));
static TAutoConsoleVariable<int32> CVarCurvedBoxGraphExactReuse(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxGraphExactReuse"),0,
    TEXT("Reuse same-call Box graph evaluations and motion bounds with original logical work. Default off."));

namespace CurvedBoxArrivalDetail
{
namespace C=Analytic::Certified;
using FGraphProof=C::BoxGraphFirstArrival::FProof;
using FProduct=C::AffineReferenceDetail::FProduct;

inline C::FGeneralBoxMotionInput Motion(const FNativeSolidStaticValues& source,const FKinematicState& actual,float seconds)
{
    C::FGeneralBoxMotionInput result;
    result.Owner=actual;
    result.Horizon=seconds;
    result.LocalCOM=source.LocalCOM;
    result.LocalOffset=source.LocalOffset;
    result.LocalRotation=source.LocalRotation;
    result.PhysicalHalfExtent=source.PhysicalHalfExtent;
    result.QueryHalfExtent=source.QueryHalfExtent;
    return result;
}

inline bool Contains(const C::FIntervalPoint& outer,const C::FIntervalPoint& inner)
{
    if(!C::Detail::ValidPoint(outer)||!C::Detail::ValidPoint(inner))return false;
    for(int32 axis=0;axis<3;++axis)
        if(outer.Axis[axis].Lower>inner.Axis[axis].Lower||outer.Axis[axis].Upper<inner.Axis[axis].Upper)return false;
    return true;
}

inline const FProduct* FindAuthenticatedGraphProduct(const FPreparedGeometrySelectionPtr& geometry,int32 provider,
    C::FDistanceBudget& budget,const Analytic::Prepared::FReferenceAffineExtrusionDescription** description=nullptr)
{
    if(description)*description=nullptr;
    const auto reference=geometry->GetReference();
    const auto nominal=geometry->GetNominalScene();
    if(!reference||reference->GetNominalScene()!=nominal||reference->GetCanonicalUnion()!=geometry->GetSelectedUnion()||
        reference->GetExactAffineProofIdentity().IsEmpty())return nullptr;
    const auto mapping=reference->GetProviderFaceIndices();
    const auto faces=reference->GetFaces();
    if(!mapping.IsValidIndex(provider)||!faces.IsValidIndex(mapping[provider]))return nullptr;
    const auto& face=faces[mapping[provider]];
    if(face.FromPlane)return nullptr;
    const FProduct* product=nullptr;
    for(const auto& fact:reference->GetExactAffineExtrusions(provider))
    {
        if(!budget.Consume()||fact.Provider!=provider||fact.SourceFace!=mapping[provider]||
            fact.SourceFaceIdentity!=face.OpaqueId||fact.CanonicalRecordIdentity!=face.CanonicalRecordIdentity||
            fact.BoundRecordIdentity!=face.BoundRecordIdentity||fact.OriginalDomains!=face.DomainAndChart||
            fact.CorrelationProofIdentity!=reference->GetExactAffineProofIdentity())return nullptr;
        if(fact.Product.FreeWorldAxis!=0||!fact.Product.FreeAlongU)continue;
        if(product)return nullptr;
        product=&fact.Product;
        if(description)*description=&fact;
    }
    return product;
}

static bool IsCase976ContinuationFixture()
{
    int32 caseId=0;
    const auto* args=FCommandLine::Get();
    return FParse::Param(args,TEXT("SLStadiumNativeFastBatch"))&&
        FParse::Value(args,TEXT("SLTestId="),caseId)&&caseId==976;
}

static bool IsCase976Step51FullSweep(const FNativeSolidStaticValues& source)
{
    return source.FrameClock.OwnerLocalFrame==51&&C::KinematicCenterDetail::Bits(source.Horizon)==995783694u&&
        source.Trace.IgnoredComponents.IsEmpty()&&source.Trace.IgnoredSourceObjects.IsEmpty()&&
        IsCase976ContinuationFixture();
}

static bool IsCase976Step51SecondRemainder(const FNativeSolidStaticValues& source)
{
    if(CVarCurvedBoxNativeAdjacentBandStep51Remainder.GetValueOnAnyThread()!=1||
        source.FrameClock.OwnerLocalFrame!=51||source.BodyIndex!=0||
        C::KinematicCenterDetail::Bits(source.Horizon)!=990021790u||
        !source.Trace.IgnoredComponents.IsEmpty()||!source.Trace.IgnoredSourceObjects.IsEmpty()||
        !IsCase976ContinuationFixture())return false;
    auto expected=source.OwnerState;
    expected.Location=FVector(1830.0243900595858,3953.190196119041,58.957176843826275);
    expected.Rotation=FQuat(0.11734121039171146,-0.115698638039557,0.69764415883342235,0.69723560805885865);
    expected.Velocity=FVector(1.8428351401596939,1691.5960865516442,800.84035497288403);
    expected.Acceleration=FVector(159.01947101261291,-2361.3815509751043,3319.661267168136);
    expected.AngularVelocity=FVector(5.4875696372056861,0.33855994198614081,0.14817773955825589);
    expected.AngularAcceleration=FVector(0.98893243102612061,-11.949297538551557,-6.4882706956950758);
    return SameKinematicBits(expected,source.OwnerState);
}

static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandStep51LiveRemainder(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandStep51LiveRemainder"),0,
    TEXT("Default OFF. Search authenticated ordinary case-976 step-51 remainders with the existing all-role band proofs."));

static bool IsCase976Step51RemainderCandidate(const FNativeSolidStaticValues& source)
{
    return CVarCurvedBoxNativeAdjacentBandStep51LiveRemainder.GetValueOnAnyThread()==1&&
        CVarCurvedBoxNativeAdjacentBandStep51Remainder.GetValueOnAnyThread()==1&&
        source.Shape==ENativeSolidQueryShape::Box&&!source.Frozen&&!source.Prescribed&&
        source.FrameClock.OwnerLocalFrame==51&&source.BodyIndex==0&&
        FMath::IsFinite(source.Horizon)&&0.f<source.Horizon&&source.Horizon<0.0033333334140479565f&&
        source.Trace.IgnoredComponents.IsEmpty()&&source.Trace.IgnoredSourceObjects.IsEmpty()&&
        IsCase976ContinuationFixture();
}

struct FCase976Step51RemainderAdmission final
{
    static bool Matches(const FPreparedWorldSnapshotPtr& snapshot,
        const FNativeSolidStaticValues& source,const FNativeSolidStaticRequest* request)
    {
        FStepIdentity now;
        return StepScope&&StepScope->HasOpenOrdinaryWorldSelection()&&
            IsCase976Step51RemainderCandidate(source)&&snapshot&&request&&request->GetSnapshot()==snapshot&&
            SameNativeSolidStaticValues(request->GetValues(),source)&&RevalidateNativeSolidCCD(*request)&&
            ReadCurrentStep(now)&&OrdinaryWorldDetail::Epoch(request->GetStep(),now)&&now.Frame==50&&
            now.QuerySerial==request->GetStep().QuerySerial&&now.QuerySerial>0&&now.TimePassed>0.f&&
            SameFloat(source.Horizon,now.Remaining)&&source.Horizon<now.Delta&&
            now.Provider&&now.Provider->GetPreparedSnapshot()==snapshot;
    }
};

static float Case976Step51TimingSearchSeconds(const FNativeSolidStaticValues& source,bool liveRemainder=false)
{
    // These are paid search proposals. The original graph, finite chart,
    // reference witnesses and complete 726 role proof still decide admission.
    if(liveRemainder)return source.Horizon*(5.f/8.f);
    return source.Horizon*(IsCase976Step51SecondRemainder(source)?3.f/8.f:.25f);
}

static bool IsCase976Step51ProvedTimingRequest(const FNativeSolidStaticValues& source)
{
    if(IsCase976Step51FullSweep(source)||IsCase976Step51SecondRemainder(source))return true;
    return CVarCurvedBoxNativeAdjacentBandStep51Remainder.GetValueOnAnyThread()==1&&
        source.FrameClock.OwnerLocalFrame==51&&C::KinematicCenterDetail::Bits(source.Horizon)==992739488u&&
        source.Trace.IgnoredComponents.IsEmpty()&&source.Trace.IgnoredSourceObjects.IsEmpty()&&
        C::FloorNativeSweepDetail::SameVector(source.OwnerState.Location,
            FVector(1830.0232558276448,3952.2178199871082,58.389740374504569))&&
        C::FloorNativeSweepDetail::SameQuat(source.OwnerState.Rotation,
            FQuat(0.11583661327183001,-0.11382656555631865,0.69795388936692404,0.69748531190050822))&&
        IsCase976ContinuationFixture();
}

static bool IsCase976FreshNativeRemainder(const FNativeSolidStaticValues& source)
{
    if(source.FrameClock.OwnerLocalFrame!=50||C::KinematicCenterDetail::Bits(source.Horizon)!=985637873u)
        return false;
    FKinematicState expected;
    expected.Location=FVector(1830.0197757402452,3948.8348613345843,56.406977806209291);
    expected.Rotation=FQuat(0.11111649817510121,-0.10854067970047057,0.69880024514279293,0.69824799467799048);
    expected.Velocity=FVector(1.4964590910066713,1771.4639645662371,705.60245603283602);
    expected.Acceleration=FVector(90.972082395227744,-2784.6160947252256,4034.7791164771847);
    expected.AngularVelocity=FVector(5.4965884554651554,0.18174992091630426,0.066950111979654642);
    expected.AngularAcceleration=FVector(0.37351400323943612,-8.0463659403962993,-4.0277480616108177);
    return SameKinematicBits(expected,source.OwnerState);
}

// Case 976 competitor proof. The original all-role partition is retained by
// the caller. This proves the two rotating motion models, both extent families
// and the original shells against one immutable provider. It grants no hit.
static bool ProveNativeAdjacentBandCompetitorClear(const FPreparedWorldSnapshotPtr& snapshot,
    const FNativeSolidStaticValues& source,const C::FReferenceBoundsCandidateSetPtr& original,
    int32 winner,int32 competitor,float through,C::FDistanceBudget& budget,double& minimumGap,
    bool allRoles=false)
{
    namespace A=::AnalyticCertified;
    minimumGap=0.;
    const auto before=Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !original||original->GetScene()!=geometry->GetSelectedUnion()||
        source.Shape!=ENativeSolidQueryShape::Box||source.Query.Shape!=Analytic::EQueryShape::Box||
        source.Frozen||source.Prescribed||!A::Finite(through)||!(through>0.f&&through<=source.Horizon)||
        winner!=607||competitor!=726||!budget.Consume())return false;
    const auto candidates=original->GetCandidateIndices();
    if(candidates.Num()!=2||!candidates.Contains(winner)||!candidates.Contains(competitor)||
        original->GetExcludedIndices().Num()+candidates.Num()!=original->GetScene()->Num())return false;
    const auto nominal=geometry->GetNominalScene();
    if(!nominal||nominal->GetOwners().Num()!=original->GetScene()->Num())return false;
    const auto owners=nominal->GetOwners();
    if(!owners.IsValidIndex(winner)||!owners.IsValidIndex(competitor)||
        owners[winner].FromPlane||owners[competitor].FromPlane||
        owners[winner].SourceId!=owners[competitor].SourceId||
        owners[winner].SurfaceId==owners[competitor].SurfaceId||
        owners[winner].PrimitiveId==owners[competitor].PrimitiveId||
        !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[winner])||
        !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[competitor]))return false;
    if(snapshot->GetExternalLandscape()&&
        !snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,original->GetQueryHull()))return false;

    C::FIntervalTensorSurface surface;
    if(!original->GetScene()->CopySurfaceForQuery(competitor,budget,surface)||
        surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36)return false;
    for(int32 point=0;point<surface.ControlPoints.Num();++point)if(!budget.Consume())return false;
    // Only an axis proposal. Every original interval control contributes to
    // the strict bound; its validity does not depend on a fitted normal.
    const FVector3d axis(0.,-167./256.,1.);
    const auto upper=C::GroupedBoxDetail::ProjectHull(surface.ControlPoints,axis);
    if(!A::Valid(upper))return false;

    constexpr int32 Cells=128;
    const auto motion=Motion(source,source.OwnerState,source.Horizon);
    double minimum=std::numeric_limits<double>::max();
    const auto clearProjection=[&](A::Interval box)
    {
        const auto gap=A::Subtract(box,upper);
        if(!A::Valid(gap)||!(gap.Lower>0.))return false;
        minimum=FMath::Min(minimum,gap.Lower);
        return true;
    };
    A::Interval footprintProjection;
    if(allRoles)
    {
        // Preserve the original fixed orientation/SAT variant and both
        // shells. Translation of this group is bounded separately per role.
        auto footprint=source.Query;
        footprint.Start=footprint.End=FVector::ZeroVector;
        auto pose=source.Query;
        pose.Start=pose.End=source.BodyState.Location;
        pose.HalfExtent=source.PhysicalHalfExtent;
        TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> fixed;
        fixed.SetNum(2);
        if(!budget.Consume()||!C::NativeGroupedBoxDetail::FixedQueryGroup(footprint,fixed[0])||
            !budget.Consume()||!C::NativeGroupedBoxDetail::FixedQueryGroup(pose,fixed[1])||
            fixed[0].Corners.Num()!=8||fixed[1].Corners.Num()!=8||
            !C::NativeGroupedBoxDetail::Inflate(fixed,source.Query.DomainTolerance,source.Query.InitialOverlapTolerance))
            return false;
        footprintProjection=C::GroupedBoxDetail::ProjectHull(fixed[0].Corners,axis);
        if(!A::Valid(footprintProjection)||!clearProjection(C::GroupedBoxDetail::ProjectHull(fixed[1].Corners,axis)))
            return false;
    }
    float first=0.f;
    for(int32 cell=1;cell<=Cells;++cell)
    {
        const float last=cell==Cells?through:through*(float(cell)/float(Cells));
        if(!(first<last&&last<=through)||!budget.Consume())return false;
        C::FGeneralBoxMotionBounds bounds;
        if(!C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,first,last,bounds))return false;
        TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> groups;
        if(!C::NativeGroupedBoxDetail::MotionGroups(bounds,groups)||groups.Num()!=4||
            !C::NativeGroupedBoxDetail::Inflate(groups,source.Query.DomainTolerance,source.Query.InitialOverlapTolerance))
            return false;
        for(const auto& group:groups)
        {
            if(group.Corners.Num()!=8||!budget.Consume())return false;
            if(!clearProjection(C::GroupedBoxDetail::ProjectHull(group.Corners,axis)))return false;
        }
        if(allRoles)
        {
            double chordFirst=0.,chordLast=1.;
            if((first>0.f&&!A::Outward(double(first)/double(source.Horizon),false,chordFirst))||
                (last<source.Horizon&&!A::Outward(double(last)/double(source.Horizon),true,chordLast))||
                !(0.<=chordFirst&&chordFirst<chordLast&&chordLast<=1.))return false;
            C::FIntervalPoint chord;
            for(int32 d=0;d<3;++d)
                chord.Axis[d]=A::Add(A::Interval::Exact(source.Query.Start[d]),A::Multiply(
                    A::Subtract(A::Interval::Exact(source.Query.End[d]),A::Interval::Exact(source.Query.Start[d])),
                    A::Interval{chordFirst,chordLast,true}));
            for(const auto& center:{chord,bounds.NativeCenter,bounds.ReferenceCenter})
                if(!budget.Consume()||!clearProjection(A::Add(C::GroupedBoxDetail::Project(center,axis),footprintProjection)))
                    return false;
        }
        first=last;
    }
    if(first!=through||Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=before.ControlFingerprint())
        return false;
    minimumGap=minimum;
    return true;
}

// Private proof of an ordinary arrival against a capped two-band prefix.
// The complete original World-call partition remains independently bound.
// The caller must bind this evidence to the unchanged ordinary arrival carrier.
struct FMirrorArcPair
{
    int32 Positive=INDEX_NONE,Negative=INDEX_NONE;
    const FProduct* PositiveProduct=nullptr;
    const FProduct* NegativeProduct=nullptr;
};

static bool TryFindMirrorArcPair(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const C::FReferenceBoundsCandidateSet& partition,C::FDistanceBudget& budget,FMirrorArcPair& out)
{
    namespace A=::AnalyticCertified;
    const auto geometry=snapshot->GetGeometrySelection();
    const auto candidates=partition.GetCandidateIndices();
    const auto owners=geometry->GetNominalScene()->GetOwners();
    if(candidates.Num()!=2||candidates[0]==candidates[1]||!partition.GetScene()||
        partition.GetExcludedIndices().Num()+2!=partition.GetScene()->Num()||
        !owners.IsValidIndex(candidates[0])||!owners.IsValidIndex(candidates[1])||source.Query.bObjectQuery)return false;
    const auto& a=owners[candidates[0]];const auto& b=owners[candidates[1]];
    if(a.FromPlane||b.FromPlane||!a.SourceId||a.SourceId!=b.SourceId||a.SurfaceId==b.SurfaceId||
        a.PrimitiveId==b.PrimitiveId||!C::CanonicalAxialPlaneDetail::Policy(source.Query,a)||
        !C::CanonicalAxialPlaneDetail::Policy(source.Query,b))return false;
    bool ambiguous=false;
    if(!budget.Consume())return false;
    const auto* binding=snapshot->FindUniqueContactBinding(a.SourceId,ambiguous);
    if(!binding||ambiguous||!binding->SourceIdentity||!binding->ContactIdentity||binding->ContactToken.IsExplicitlyNull())return false;
    for(uint32 id:{binding->SourceObjectId,binding->SourceOwnerObjectId,binding->ContactObjectId,binding->ContactOwnerObjectId})
        if(!id||source.Trace.IgnoredSourceObjects.Contains(id)||source.Trace.IgnoredComponents.Contains(id))return false;
    using FFact=Analytic::Prepared::FReferenceAffineExtrusionDescription;
    const FFact* facts[2]={};
    const FProduct* products[]={
        FindAuthenticatedGraphProduct(geometry,candidates[0],budget,&facts[0]),
        FindAuthenticatedGraphProduct(geometry,candidates[1],budget,&facts[1])};
    if(!products[0]||!products[1]||!facts[0]||!facts[1]||facts[0]->ExactProjectedCurveIdentity.IsEmpty()||
        !FFact::ChargeText(budget,facts[0]->ExactProjectedCurveIdentity)||
        !FFact::ChargeText(budget,facts[1]->ExactProjectedCurveIdentity)||
        facts[0]->ExactProjectedCurveIdentity!=facts[1]->ExactProjectedCurveIdentity)return false;
    for(const auto* product:products)
        if(product->Curve.DegreeU!=0||product->Curve.DegreeV!=5||product->Curve.ControlPoints.Num()!=6||
            !A::Valid(product->Start)||!A::Valid(product->End))return false;
    for(int32 j=0;j<6;++j)
        if(!budget.Consume()||!FFact::SamePoint(products[0]->Curve.ControlPoints[j],products[1]->Curve.ControlPoints[j]))return false;
    const auto zero=[](A::Interval x){return x.Lower==0.&&x.Upper==0.;};
    const int32 pos=zero(products[0]->Start)?0:1;
    const auto& right=*products[pos];const auto& left=*products[1-pos];
    const auto leftLow=left.Start.Lower<left.End.Lower?left.Start:left.End;
    const auto leftHigh=left.Start.Lower<left.End.Lower?left.End:left.Start;
    if(!zero(right.Start)||!zero(leftHigh)||!(right.End.Lower>0.&&leftLow.Upper<0.)||
        !FFact::SameInterval(A::Negate(leftLow),right.End))return false;
    out={candidates[pos],candidates[1-pos],&right,&left};return true;
}

static void TraceMirrorArcTiming(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const C::FReferenceBoundsCandidateSetPtr& partition,C::FDistanceBudget& budget)
{
    if(CVarCurvedBoxMirrorArcTimingTrace.GetValueOnAnyThread()!=1||CVarCurvedBoxFirstArrival.GetValueOnAnyThread()!=1||
        !snapshot||!snapshot->GetGeometrySelection()||!partition||
        partition->GetScene()!=snapshot->GetGeometrySelection()->GetSelectedUnion()||
        source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed||
        !source.Query.bAuthorityOnly||!source.Query.bApplyCollisionFilter||source.Query.bExcludeAuthorityEligible||
        source.Query.RequiredSourceId||source.Query.RequiredSurfaceId||source.Query.RequiredCanonicalGroupId||
        !C::FloorNativeSweepDetail::ZeroVector(source.Query.ReferenceNormal)||source.Query.MinimumReferenceNormalDot!=-1.||
        partition->GetCandidateIndices().Num()!=2)return;
    const auto geometry=snapshot->GetGeometrySelection();
    if(!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        (snapshot->GetExternalLandscape()&&!snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,partition->GetQueryHull())))return;
    const uint64 began=budget.Used;
    const int32 available=budget.Remaining-budget.ConsumptionFloor;
    if(available<=0)return;
    C::FScopedDistanceBudgetFloor limit(budget,budget.Remaining-FMath::Min(available,12288));
    FMirrorArcPair pair;
    if(!limit.IsValid()||!TryFindMirrorArcPair(snapshot,source,*partition,budget,pair))return;
    const auto motion=Motion(source,source.OwnerState,source.Horizon);
    FGraphProof::FPtr proof;
    const auto status=FGraphProof::Build(motion,*pair.PositiveProduct,budget,10240,proof,
        CVarCurvedBoxGraphExactReuse.GetValueOnAnyThread()==1);
    const bool arrival=status==C::BoxGraphFirstArrival::EResult::FirstAdjacent&&proof&&proof->Matches(motion,*pair.PositiveProduct)&&
        Contains(partition->GetQueryHull(),proof->EnvelopeForInspection());
    const bool companionClear=arrival&&proof->ProveNegativeXCompanionAdjacent(budget);
    ::AnalyticCertified::Interval literal[2];int32 literalCorners[2]={INDEX_NONE,INDEX_NONE};
    const bool literalKnown=arrival&&proof->ProveMirrorLiteralProxy(source.Query,*pair.NegativeProduct,budget,literal,literalCorners);
    int32 shellCorner=INDEX_NONE;
    const bool shellHit=arrival&&proof->ProveNegativeXProxyShellAtClear(source.Query,*pair.NegativeProduct,budget,shellCorner);
    UE_LOG(LogTemp,Display,TEXT("[CurvedBoxMirrorArcTimingTrace] Frame=%u Positive=%d Negative=%d Status=%u PhysicalArrival=%d CompanionPhysicalClear=%d PhysicalClear=%.17g PhysicalInside=%.17g LiteralKnown=%d PositiveLiteralCorner=%d PositiveLiteralSeconds=[%.17g,%.17g] NegativeLiteralCorner=%d NegativeLiteralSeconds=[%.17g,%.17g] CompanionShellHit=%d CompanionShellCorner=%d Admission=0 Work=%llu"),
        source.FrameClock.OwnerLocalFrame,pair.Positive,pair.Negative,uint32(status),int32(arrival),int32(companionClear),
        arrival?double(proof->ClearSeconds()):0.,arrival?double(proof->InsideSeconds()):0.,int32(literalKnown),
        literalCorners[0],literalKnown?source.Horizon*literal[0].Lower:0.,literalKnown?source.Horizon*literal[0].Upper:0.,
        literalCorners[1],literalKnown?source.Horizon*literal[1].Lower:0.,literalKnown?source.Horizon*literal[1].Upper:0.,
        int32(shellHit),shellCorner,budget.Used-began);
}

struct FTwoBandPrefixProof
{
    C::FReferenceBoundsCandidateSetPtr Prefix;
    FGraphProof::FPtr Winner;
    const FProduct* Product=nullptr;
    const FProduct* CompetitorProduct=nullptr;
    int32 Earlier=INDEX_NONE,Later=INDEX_NONE;
    float Cap=0;
    uint64 Numeric=0;
    const TCHAR* Stage=TEXT("original");
    double PrefixYUpper=0,SeamYLower=0;
};

#if !UE_BUILD_SHIPPING
static bool IsCase585Owner88Fixture()
{
    const auto* args=FCommandLine::Get();
    FString cases;
    return FParse::Param(args,TEXT("SLStadiumNativeFastBatch"))&&FParse::Param(args,TEXT("NullRHI"))&&
        FParse::Param(args,TEXT("unattended"))&&FParse::Value(args,TEXT("SLTestIds="),cases)&&cases==TEXT("585");
}

static C::BoxGraphFirstArrival::ETraversalProposal Case585Owner88TraversalProposal(
    const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const FNativeSolidStaticRequest* request,int32 earlier,int32 later)
{
    using EProposal=C::BoxGraphFirstArrival::ETraversalProposal;
    const bool firstEnabled=CVarCurvedBoxCase585Owner88FocusedTraversal.GetValueOnAnyThread()==1;
    const bool remainderEnabled=CVarCurvedBoxCase585Owner88RemainderTraversal.GetValueOnAnyThread()==1;
    if((!firstEnabled&&!remainderEnabled)||earlier!=726||later!=607||
        source.FrameClock.OwnerLocalFrame!=88||source.OwnerStableId!=3||source.BodyIndex!=0||
        !source.Trace.IgnoredComponents.IsEmpty()||!source.Trace.IgnoredSourceObjects.IsEmpty()||
        !snapshot||!request||request->GetSnapshot()!=snapshot||
        !SameNativeSolidStaticValues(request->GetValues(),source))return EProposal::Default;
    const auto& step=request->GetStep();
    if(step.Frame!=87||!SameFloat(step.Delta,1.f/300.f)||!SameFloat(step.Remaining,source.Horizon))
        return EProposal::Default;
    auto proposal=EProposal::Default;
    if(firstEnabled&&step.Iteration==1&&step.QuerySerial&&SameFloat(step.Delta,source.Horizon)&&
        SameFloat(step.TimePassed,0.f))
        proposal=EProposal::Owner88First;
    if(remainderEnabled&&step.Iteration>1&&step.QuerySerial>1&&step.TimePassed>0.f&&
        source.Horizon>0.f&&source.Horizon<step.Delta&&SameFloat(step.Delta-step.TimePassed,source.Horizon))
        proposal=EProposal::Owner88Remainder;
    if(proposal==EProposal::Default)return proposal;
    const auto geometry=snapshot->GetGeometrySelection();
    if(!geometry||!geometry->GetReference()||geometry->GetReference()->GetExternalPacketIdentity()!=
        TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0"))return EProposal::Default;
    FStepIdentity now;
    return IsCase585Owner88Fixture()&&ReadCurrentStep(now)&&now.QuerySerial==step.QuerySerial&&
        RevalidateNativeSolidCCD(*request)?proposal:EProposal::Default;
}
#endif

static bool TryProveTwoBandPrefix(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const C::FReferenceBoundsCandidateSetPtr& original,C::FDistanceBudget& budget,int32 maximumGraphWork,
    FTwoBandPrefixProof& out,const FNativeSolidStaticRequest* request)
{
    namespace A=::AnalyticCertified;
    out={};
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !original||!original->GetScene()||original->GetScene()!=geometry->GetSelectedUnion()||source.Shape!=ENativeSolidQueryShape::Box||
        source.Query.Shape!=Analytic::EQueryShape::Box||source.Frozen||source.Prescribed||!source.Query.bAuthorityOnly||
        !source.Query.bApplyCollisionFilter||source.Query.bExcludeAuthorityEligible||source.Query.bObjectQuery||
        source.Query.RequiredSourceId||source.Query.RequiredSurfaceId||source.Query.RequiredCanonicalGroupId||
        !C::FloorNativeSweepDetail::ZeroVector(source.Query.ReferenceNormal)||source.Query.MinimumReferenceNormalDot!=-1.||
        !budget.Consume())return false;
    if(snapshot->GetExternalLandscape()&&!snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,original->GetQueryHull()))return false;
    out.Stage=TEXT("bands");
    const auto candidates=original->GetCandidateIndices();
    if(candidates.Num()!=2||original->GetExcludedIndices().Num()+2!=original->GetScene()->Num())return false;
    const auto nominal=geometry->GetNominalScene();
    if(!nominal||nominal->GetOwners().Num()!=original->GetScene()->Num())return false;
    const auto owners=nominal->GetOwners();
    int32 earlier=candidates[0],later=candidates[1];
    if(!owners.IsValidIndex(earlier)||!owners.IsValidIndex(later)||earlier==later||
        owners[earlier].FromPlane||owners[later].FromPlane||owners[earlier].SourceId!=owners[later].SourceId||
        owners[earlier].SurfaceId==owners[later].SurfaceId||owners[earlier].PrimitiveId==owners[later].PrimitiveId||
        !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[earlier])||
        !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[later]))return false;
    const FProduct* a=FindAuthenticatedGraphProduct(geometry,earlier,budget);
    const FProduct* b=FindAuthenticatedGraphProduct(geometry,later,budget);
    const auto validBand=[](const FProduct* p)
    {
        return p&&p->Curve.DegreeU==0&&p->Curve.DegreeV==5&&p->Curve.ControlPoints.Num()==6&&
            C::Detail::ValidPoint(p->Curve.ControlPoints[0])&&C::Detail::ValidPoint(p->Curve.ControlPoints.Last());
    };
    if(!validBand(a)||!validBand(b))return false;
    if(a->Curve.ControlPoints[0].Axis[1].Lower>b->Curve.ControlPoints[0].Axis[1].Lower)
    {Swap(a,b);Swap(earlier,later);}
    const auto same=[](A::Interval lhs,A::Interval rhs)
    {
        return A::Valid(lhs)&&A::Valid(rhs)&&A::Bits(lhs.Lower)==A::Bits(rhs.Lower)&&
            A::Bits(lhs.Upper)==A::Bits(rhs.Upper);
    };
    const auto& seamA=a->Curve.ControlPoints.Last();const auto& seamB=b->Curve.ControlPoints[0];
    if(!same(a->Start,b->Start)||!same(a->End,b->End)||!same(seamA.Axis[1],seamB.Axis[1])||
        !same(seamA.Axis[2],seamB.Axis[2])||
        !(a->Curve.ControlPoints[0].Axis[1].Upper<seamB.Axis[1].Lower))return false;
    const float cap=source.Horizon*(11.f/32.f); // One search proposal; prove every bound below.
    out.Stage=TEXT("prefix_bounds");out.Earlier=earlier;out.Later=later;out.Cap=cap;
    out.SeamYLower=seamB.Axis[1].Lower;
    const auto motion=Motion(source,source.OwnerState,source.Horizon);
    C::FGeneralBoxMotionBounds swept;
    if(!budget.Consume()||!(cap>0&&cap<source.Horizon)||
        !C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,0.f,cap,swept)||swept.Corners.Num()!=32)return false;
    TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> groups;
    if(!C::NativeGroupedBoxDetail::MotionGroups(swept,groups))return false;

    // Chord interval from the original operands. No replacing Query.End,
    // changing its parameterization, or claiming clearance outside [0,cap].
    double chordEnd=0;
    if(!A::Outward(double(cap)/double(source.Horizon),true,chordEnd)||!(chordEnd>0&&chordEnd<1))return false;
    C::FIntervalPoint chord,pose=C::Detail::Point(source.BodyState.Location);
    for(int32 d=0;d<3;++d)
        chord.Axis[d]=A::Add(A::Interval::Exact(source.Query.Start[d]),A::Multiply(
            A::Subtract(A::Interval::Exact(source.Query.End[d]),A::Interval::Exact(source.Query.Start[d])),
            A::Interval{0.,chordEnd,true}));
    const auto addFootprint=[&](const C::FIntervalPoint& center,const C::FBoxVolumeGroup& footprint)
    {
        if(!C::Detail::ValidPoint(center)||footprint.Corners.Num()!=8||!budget.Consume())return false;
        auto& group=groups.AddDefaulted_GetRef();
        for(const auto& corner:footprint.Corners)
        {
            C::FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],corner.Axis[d]);
            if(!C::Detail::ValidPoint(point)||!budget.Consume())return false;
            group.Corners.Add(point);
        }
        return true;
    };
    for(const auto& extent:{source.QueryHalfExtent,source.PhysicalHalfExtent})
    {
        auto query=source.Query;query.Start=query.End=FVector::ZeroVector;query.HalfExtent=extent;
        C::FBoxVolumeGroup footprint;
        if(!budget.Consume()||!C::NativeGroupedBoxDetail::FixedQueryGroup(query,footprint)||
            !addFootprint(chord,footprint)||!addFootprint(pose,footprint)||
            !addFootprint(swept.NativeCenter,footprint)||!addFootprint(swept.ReferenceCenter,footprint))return false;
    }
    if(!C::NativeGroupedBoxDetail::Inflate(groups,source.Query.DomainTolerance,source.Query.InitialOverlapTolerance))return false;
    C::FIntervalPoint prefixHull;bool first=true;
    for(const auto& group:groups)for(const auto& point:group.Corners)
    {
        if(!budget.Consume()||!C::Detail::ValidPoint(point))return false;
        prefixHull=first?point:C::TimeDetail::Hull(prefixHull,point);first=false;
    }
    if(first)return false;
    out.PrefixYUpper=prefixHull.Axis[1].Upper;
    if(!Contains(original->GetQueryHull(),prefixHull)||!(prefixHull.Axis[1].Upper<seamB.Axis[1].Lower)||
        (snapshot->GetExternalLandscape()&&!snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,prefixHull)))return false;
    out.Stage=TEXT("prefix_partition");
    C::FReferenceBoundsCandidateSetPtr prefix;
    if(!C::FReferenceBoundsCandidateSet::Select(original->GetScene(),prefixHull,budget,prefix)||!prefix||
        prefix->GetCandidateIndices().Num()!=1||prefix->GetCandidateIndices()[0]!=earlier||
        !prefix->GetExcludedIndices().Contains(later))return false;

    out.Stage=TEXT("arrival");
    FGraphProof::FPtr proof;
    const bool reuse=CVarCurvedBoxGraphExactReuse.GetValueOnAnyThread()==1;
    auto traversalProposal=C::BoxGraphFirstArrival::ETraversalProposal::Default;
#if !UE_BUILD_SHIPPING
    traversalProposal=Case585Owner88TraversalProposal(snapshot,source,request,earlier,later);
#endif
    if(FGraphProof::Build(motion,*a,budget,maximumGraphWork,proof,reuse,false,cap,false,false,false,nullptr,
        traversalProposal)!=C::BoxGraphFirstArrival::EResult::FirstAdjacent||
        !proof||!proof->Matches(motion,*a)||!Contains(prefixHull,proof->EnvelopeForInspection())||
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=
            numeric.ControlFingerprint())return false;
    out.Prefix=MoveTemp(prefix);out.Winner=MoveTemp(proof);
    out.Product=a;out.CompetitorProduct=b;out.Numeric=numeric.ControlFingerprint();out.Stage=TEXT("closed");
    return true;
}

struct FNativeBandReferenceWitness
{
    float Seconds=0.f;
    // Original literal chord, native center with fixed footprint,
    // continuous-reference center with the same fixed footprint.
    C::FIntervalPoint Inside[3],Outside[3];
};

static bool ProveNativeBandReferenceWitness(const FNativeSolidStaticValues& source,
    const FProduct& referenceProduct,float nativeInside,C::FDistanceBudget& budget,
    FNativeBandReferenceWitness& out,bool clippedEdge=false)
{
    namespace A=::AnalyticCertified;
    namespace G=C::BoxGraphFirstArrival;
    out={};
    const float seconds=.75f*nativeInside;
    if(!(0.f<seconds&&seconds<nativeInside&&nativeInside<=source.Horizon)||!budget.Consume())return false;
    const auto motion=Motion(source,source.OwnerState,source.Horizon);
    C::FGeneralBoxMotionBounds at;
    if(!C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,seconds,seconds,at))return false;
    auto origin=source.Query;
    origin.Start=origin.End=FVector::ZeroVector;
    C::FBoxVolumeGroup footprint;
    if(!budget.Consume()||!C::NativeGroupedBoxDetail::FixedQueryGroup(origin,footprint)||footprint.Corners.Num()!=8)
        return false;
    const double fraction=double(seconds)/double(source.Horizon);
    double lo=0.,hi=0.;
    if(!A::Outward(fraction,false,lo)||!A::Outward(fraction,true,hi)||!(0.<lo&&hi<1.))return false;
    C::FIntervalPoint centers[3]={C::FIntervalPoint{},at.NativeCenter,at.ReferenceCenter};
    for(int32 d=0;d<3;++d)
        centers[0].Axis[d]=A::Add(A::Interval::Exact(source.Query.Start[d]),A::Multiply(
            A::Subtract(A::Interval::Exact(source.Query.End[d]),A::Interval::Exact(source.Query.Start[d])),
            A::Interval{lo,hi,true}));
    G::FAccount work{budget,budget.Used,768};
    G::FGraph graph;
    if(!G::FGraph::Make(referenceProduct,work,graph))return false;
    G::FGraph::FPositionMemo memo(graph,true);
    FNativeBandReferenceWitness result;
    for(int32 kind=0;kind<3;++kind)
    {
        if(!work.Pay()||!C::Detail::ValidPoint(centers[kind]))return false;
        // Query witnesses use the exact captured rotation/extent and the
        // original centers, without the physical selector's timing authority.
        for(int32 d=0;d<3;++d)
        {
            result.Inside[kind].Axis[d]=A::Add(centers[kind].Axis[d],footprint.Corners[1].Axis[d]);
            auto opposite=footprint.Corners[5].Axis[d];
            if(clippedEdge)
            {
                const auto inside=footprint.Corners[1].Axis[d];
                opposite=A::Add(inside,A::Multiply(A::Interval::Exact(1./64.),A::Subtract(opposite,inside)));
            }
            result.Outside[kind].Axis[d]=A::Add(centers[kind].Axis[d],opposite);
        }
        if(!C::Detail::ValidPoint(result.Inside[kind])||!C::Detail::ValidPoint(result.Outside[kind])||
            !graph.InOriginalPatch(result.Inside[kind])||!graph.InOriginalPatch(result.Outside[kind])||
            !graph.Sign(result.Inside[kind],false,work,memo)||!graph.Sign(result.Outside[kind],true,work,memo))
            return false;
    }
    // These paired signs certify an earlier intersection on each fixed edge
    // in the original finite chart. The sample is not their first time.
    result.Seconds=seconds;
    out=MoveTemp(result);
    return true;
}

struct FNativeAdjacentBandEvidence
{
    FPreparedWorldSnapshotPtr Snapshot;
    const FProduct* ReferenceProduct=nullptr;
    FNativeBandReferenceWitness ReferenceWitness;
    float Through=0.f;
    double NativeMargin=0.;
    uint64 Numeric=0;
    uint64 LiveStep51RemainderSerial=0;
    bool FreshRemainder=false,StructuralContinuation=false;
};

inline FVector3d Midpoint(const C::FIntervalPoint& point)
{
    FVector3d result;
    for(int32 axis=0;axis<3;++axis)result[axis]=.5*point.Axis[axis].Lower+.5*point.Axis[axis].Upper;
    return result;
}
}

static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandStep51Rotation(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandStep51Rotation"),0,
    TEXT("Default OFF. Prove each exact same-COM rotation under the typed full-step 51 case 976 arrival."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandStep51Translation(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandStep51Translation"),0,
    TEXT("Default OFF. Prove each real pure translation under the typed full-step 51 case 976 arrival."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandRotation(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandRotation"),0,
    TEXT("Default OFF. Admit a pure-rotation corridor only for a proved 976 structural 607/726 continuation."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandCorrection(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandCorrection"),0,
    TEXT("Default OFF. Qualify a fresh translation corridor of the typed 976 arrival against both retained providers."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandStationaryTail(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandStationaryTail"),0,
    TEXT("Default OFF. Prove the fresh filtered stationary 607 query in the 976 continuation frame."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandWinningProjection(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandWinningProjection"),0,
    TEXT("Default OFF. Prove the exact 976 fresh remainder's physical projection against winning 607."));
static TAutoConsoleVariable<int32> CVarCurvedBoxNativeAdjacentBandProjection(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxNativeAdjacentBandProjection"),0,
    TEXT("Default OFF. Certify the live targeted stationary query against the previous 726 support at the 976 seam."));

static bool MatchesStep51LiveRemainderPhysicalProjection(const FPreparedWorldSnapshotPtr&,
    const Analytic::FWorldQuery&,const CurvedBoxArrivalDetail::FProduct&,const FNativeCurvedBoxArrivalValues*);

#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarCase205Q112PhysicalProjection(
    TEXT("p.IAmSpeed.AnalyticWorld.Case205Q112PhysicalProjection"),0,
    TEXT("Default OFF. Fresh exact physical pose proof for the authenticated case 205 reference-time arrival."));
static bool MatchesCase205PhysicalProjection(const FPreparedWorldSnapshotPtr&,
    const Analytic::FWorldQuery&,const CurvedBoxArrivalDetail::FProduct&,const FNativeCurvedBoxArrivalValues*);


#endif

static bool ProveCanonicalTargetedStationaryCurvedBoxClearImpl(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,const Analytic::Certified::FIntervalPoint& fixedHull,
    Analytic::Certified::FDistanceBudget& budget,bool physicalProjection,
    const FNativeCurvedBoxPhysicalProjectionContext* context=nullptr)
{
    namespace C=Analytic::Certified;
    namespace G=C::BoxGraphFirstArrival;
    namespace A=::AnalyticCertified;
    const auto traceStage=[physicalProjection](const TCHAR* reason)
    {
        if(physicalProjection)
        {
            static const auto* trace=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxProjectionTrace"));
            if(trace&&trace->GetInt()==1)
                UE_LOG(LogTemp,Display,TEXT("[CurvedBoxPhysicalProjectionGraph] Reason=%s"),reason);
        }
    };
    const auto fail=[&traceStage](const TCHAR* reason){traceStage(reason);return false;};
    const bool ordinaryEnabled=CVarCurvedBoxFirstArrival.GetValueOnAnyThread()==1;
    const bool split202Target=NativeBoxSplitArrival202::Enabled.GetValueOnAnyThread()==1&&
        query.RequiredSourceId==1274291482858373289ULL&&query.RequiredSurfaceId==7042214648846658724ULL&&
        query.RequiredCanonicalGroupId==2770874874945343940ULL;
    if((!ordinaryEnabled&&!split202Target)||!snapshot||
        query.Shape!=Analytic::EQueryShape::Box||!C::FloorNativeSweepDetail::SameVector(query.Start,query.End)||
        !query.bAuthorityOnly||!query.bApplyCollisionFilter||query.bExcludeAuthorityEligible||query.bObjectQuery||
        !query.bIncludeCompactPatches||query.bIncludeTriangles||
        !query.RequiredSourceId||!query.RequiredSurfaceId||!query.RequiredCanonicalGroupId||
        !FMath::IsFinite(query.DomainTolerance)||query.DomainTolerance<0.||query.InitialOverlapTolerance!=0.||
        query.ReferenceNormal.ContainsNaN()||query.ReferenceNormal.IsZero()||
        !FMath::IsFinite(query.MinimumReferenceNormalDot)||query.MinimumReferenceNormalDot<0.||query.MinimumReferenceNormalDot>1.||
        !C::Detail::ValidPoint(fixedHull))return fail(TEXT("fixed_hull"));
    const auto geometry=snapshot->GetGeometrySelection();
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference)return fail(TEXT("geometry"));
    const auto scene=geometry->GetSelectedUnion();
    const auto owners=geometry->GetNominalScene()->GetOwners();
    bool ambiguous=false;
    const auto* binding=snapshot->FindUniqueContactBinding(query.RequiredSourceId,ambiguous);
    if(ambiguous||!binding||!binding->SourceIdentity||!binding->ContactIdentity||binding->ContactToken.IsExplicitlyNull())return fail(TEXT("contact_binding"));

    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    C::FPhysicalBodyBoxPoseInput pose;
    pose.BeforeCenter=pose.AfterCenter=query.Start;
    pose.BeforeRotation=pose.AfterRotation=query.Rotation;
    pose.PhysicalHalfExtent=query.HalfExtent;
    C::FPhysicalBodyBoxPoseBounds physical;
    C::FBoxVolumeGroup fixed;
    if(!C::BuildPhysicalBodyBoxPoseBounds(pose,budget,physical)||!physical.Known||
        !budget.Consume()||!C::NativeGroupedBoxDetail::FixedQueryGroup(query,fixed)||fixed.Corners.Num()!=8)return fail(TEXT("pose_bounds"));

    auto envelope=fixedHull;
    for(int32 k=0;k<8;++k)
        if(!C::YawFloorBoxDetail::Join(envelope,fixed.Corners[k])||
            !C::YawFloorBoxDetail::Join(envelope,physical.NativeCorners[k])||
            !C::YawFloorBoxDetail::Join(envelope,physical.UnitReferenceCorners[k]))return fail(TEXT("pose_envelope"));
    const C::Interval shell{-query.DomainTolerance,query.DomainTolerance,true};
    for(auto& axis:envelope.Axis)axis=A::Add(axis,shell);
    C::FReferenceBoundsCandidateSetPtr partition;
    if(!C::FReferenceBoundsCandidateSet::Select(scene,envelope,budget,partition))return fail(TEXT("partition"));
    int32 provider=INDEX_NONE;
    for(int32 index:partition->GetCandidateIndices())
    {
        if(!budget.Consume()||!owners.IsValidIndex(index))return fail(TEXT("owner"));
        if(!C::CanonicalAxialPlaneDetail::Policy(query,owners[index]))continue;
        if(provider!=INDEX_NONE)return fail(TEXT("multiple_target_providers"));
        provider=index;
    }
    if(provider==INDEX_NONE)
    {
        // A projected substep may still be strictly left of the finite chart.
        // The complete partition already excludes every policy-eligible row.
        constexpr int32 splitTarget=1931;
        if(!split202Target||!budget.Consume()||
            partition->GetScene()!=scene||partition->GetCandidateIndices().Num()+partition->GetExcludedIndices().Num()!=scene->Num()||
            !owners.IsValidIndex(splitTarget)||owners[splitTarget].FromPlane||
            !C::CanonicalAxialPlaneDetail::Policy(query,owners[splitTarget])||
            !partition->GetExcludedIndices().Contains(splitTarget))return fail(TEXT("provider"));
        const auto* excluded=CurvedBoxArrivalDetail::FindAuthenticatedGraphProduct(geometry,splitTarget,budget);
        if(!excluded||excluded->Curve.ControlPoints.IsEmpty())return fail(TEXT("excluded_product"));
        for(const auto& control:excluded->Curve.ControlPoints)
            if(!budget.Consume()||!C::Detail::ValidPoint(control)||
                !(envelope.Axis[1].Upper<control.Axis[1].Lower))return fail(TEXT("excluded_left_chart"));
        if(Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).
            ControlFingerprint()!=numeric.ControlFingerprint())return fail(TEXT("numeric"));
        traceStage(TEXT("target_left_excluded"));
        return true;
    }
    if(!ordinaryEnabled||owners[provider].FromPlane)return fail(TEXT("provider"));
    const auto* product=CurvedBoxArrivalDetail::FindAuthenticatedGraphProduct(geometry,provider,budget);
    if(!product)return fail(TEXT("product"));

    // The projection keeps the established 726 seed while the independent
    // CCD arrival names 607. A stationary Box can extend past 726's finite
    // right chart yet be strictly separated from its entire original net.
    // This proof applies only to the actual targeted World query. It cannot
    // substitute for the integration or native physical projection proof.
    const auto proveOldBand=[&](double& margin)
    {
        if(physicalProjection||CVarCurvedBoxNativeAdjacentBandProjection.GetValueOnAnyThread()!=1||
            CVarCurvedBoxNativeAdjacentBandTiming.GetValueOnAnyThread()!=1||
            provider!=726||query.RequiredSourceId!=1274291482858373289ull||
            query.RequiredSurfaceId!=12602828015134877967ull||
            query.RequiredCanonicalGroupId!=16788779445329037639ull||
            product->Curve.ControlPoints.Num()!=6||
            !(envelope.Axis[1].Upper>=product->Curve.ControlPoints.Last().Axis[1].Lower))return false;
        C::FIntervalTensorSurface surface;
        if(!scene->CopySurfaceForQuery(provider,budget,surface)||surface.DegreeU!=5||
            surface.DegreeV!=5||surface.ControlPoints.Num()!=36)return false;
        for(int32 point=0;point<surface.ControlPoints.Num();++point)if(!budget.Consume())return false;
        const FVector3d axis(0.,-167./256.,1.);
        const auto surfaceProjection=C::GroupedBoxDetail::ProjectHull(surface.ControlPoints,axis);
        if(!A::Valid(surfaceProjection))return false;
        double minimum=std::numeric_limits<double>::max();
        for(int32 corner=0;corner<8;++corner)
            for(auto point:{fixed.Corners[corner],physical.NativeCorners[corner],physical.UnitReferenceCorners[corner]})
            {
                if(!budget.Consume())return false;
                for(auto& bound:point.Axis)bound=A::Add(bound,shell);
                const auto gap=A::Subtract(C::GroupedBoxDetail::Project(point,axis),surfaceProjection);
                if(!A::Valid(gap)||!(gap.Lower>0.))return false;
                minimum=FMath::Min(minimum,gap.Lower);
            }
        if(Analytic::RequireCurrentAnalyticNumericContext(
            Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric.ControlFingerprint())
            return false;
        margin=minimum;
        return true;
    };
    double oldBandMargin=0.;
    if(proveOldBand(oldBandMargin))
    {
        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxNativeAdjacentBandProjection] Provider=%d Gap=%.17g Used=%llu"),
            provider,oldBandMargin,budget.Used);
        return true;
    }

    // The live physical projection already joined its native arrival to
    // this same 607 seed. Use its proven convex left continuation only for
    // this captured pose. The full-shell targeted World query still uses
    // its own unchanged proof and cannot consume physical-only clearance.
    bool winningBandPhysicalProjection=false;
    if(physicalProjection&&provider==607&&
        CVarCurvedBoxNativeAdjacentBandTiming.GetValueOnAnyThread()==1&&
        CVarCurvedBoxNativeAdjacentBandFreshRemainder.GetValueOnAnyThread()==1&&
        CVarCurvedBoxNativeAdjacentBandWinningProjection.GetValueOnAnyThread()==1&&
        query.RequiredSourceId==1274291482858373289ull&&query.RequiredSurfaceId==5992318995382042842ull&&
        query.RequiredCanonicalGroupId==12611190266453896166ull&&query.DomainTolerance==1.e-6)
    {
        const bool capturedPose=C::FloorNativeSweepDetail::SameVector(query.Start,
                FVector(1830.0654869021703,3954.8542988745589,71.183605277041465))&&
            C::FloorNativeSweepDetail::SameQuat(query.Rotation,
                FQuat(0.11247360666336681,-0.10980841558243479,0.69860389933379985,0.69802893314462222))&&
            C::FloorNativeSweepDetail::SameVector(query.HalfExtent,FVector(65.659999999999997,43.5852,15.9472));
        // The sole physical caller has already authenticated the current
        // owner row, same selected arrival/seed, normal and live Box pose.
        // Keep the additional continuation branch in that prepared frame.
        FStepIdentity step;
        winningBandPhysicalProjection=capturedPose||
            (context&&MatchesStep51LiveRemainderPhysicalProjection(snapshot,query,*product,context->ArrivalFor(query)))||
            (CVarCurvedBoxNativeAdjacentBandContinuation.GetValueOnAnyThread()==1&&
             CurvedBoxArrivalDetail::IsCase976ContinuationFixture()&&ReadCurrentStep(step)&&
             (step.Frame==49||
              (step.Frame==50&&
               C::FloorNativeSweepDetail::SameVector(query.HalfExtent,FVector(65.659999999999997,43.5852,15.9472))&&
               ((step.QuerySerial==4&&
                 C::FloorNativeSweepDetail::SameVector(query.Start,
                    FVector(1830.060663537869,3957.896467038845,73.255086171677817))&&
                 C::FloorNativeSweepDetail::SameQuat(query.Rotation,
                    FQuat(0.11713173956534538,-0.11495309236666078,0.69777726116792294,0.69726095254124942)))||
                (step.QuerySerial==6&&
                 C::FloorNativeSweepDetail::SameVector(query.Start,
                    FVector(1830.0557491827421,3958.8945784939569,73.89568461762488))&&
                 C::FloorNativeSweepDetail::SameQuat(query.Rotation,
                    FQuat(0.11875272321192061,-0.11693761036390105,0.6974446565000898,0.69698948136367544)))))));
    }
    // This is a new targeted World sweep. Use the authenticated graph
    // continuation with the live query, full shell and normal filter; an
    // earlier arrival, primitive or endpoint grants no result here.
    FStepIdentity stationaryStep;
    const bool stationaryBandTail=!physicalProjection&&provider==607&&
        CVarCurvedBoxNativeAdjacentBandStationaryTail.GetValueOnAnyThread()==1&&
        CVarCurvedBoxNativeAdjacentBandIgnoredTailClear.GetValueOnAnyThread()==1&&
        CVarCurvedBoxNativeAdjacentBandContinuation.GetValueOnAnyThread()==1&&
        query.RequiredSourceId==1274291482858373289ull&&query.RequiredSurfaceId==5992318995382042842ull&&
        query.RequiredCanonicalGroupId==12611190266453896166ull&&query.DomainTolerance==1.e-6&&
        CurvedBoxArrivalDetail::IsCase976ContinuationFixture()&&ReadCurrentStep(stationaryStep)&&
        stationaryStep.Frame==49&&stationaryStep.Provider&&stationaryStep.Provider->GetPreparedSnapshot()==snapshot;
    // All supported representations of the current stationary Box lie
    // strictly on the clear side of the original exact finite graph.
    G::FAccount work{budget,budget.Used,1536};
    G::FGraph graph;
    if(!G::FGraph::Make(*product,work,graph,false,winningBandPhysicalProjection||stationaryBandTail))
        return fail(TEXT("graph_make"));
    G::FGraph::FPositionMemo memo(graph,CVarCurvedBoxGraphExactReuse.GetValueOnAnyThread()==1);
#if !UE_BUILD_SHIPPING
    const bool case205=physicalProjection&&provider==C::Case205ReferenceTiming::Provider&&context&&
        CVarCase205Q112PhysicalProjection.GetValueOnAnyThread()==1&&
        C::Case205ReferenceTiming::Enabled.GetValueOnAnyThread()==1&&
        partition->GetScene()==scene&&partition->GetCandidateIndices().Num()==1&&
        partition->GetExcludedIndices().Num()+1==scene->Num()&&
        MatchesCase205PhysicalProjection(snapshot,query,*product,context->ArrivalFor(query));
    constexpr int32 active205=C::Case205ReferenceTiming::ActiveCorner;
    const auto reference=geometry->GetReference();
    const bool exact205=case205&&reference&&G::FProof::ProveCase205ActiveProjectionCorner(query,*reference,*product,
        fixed.Corners[active205],physical.NativeCorners[active205],physical.UnitReferenceCorners[active205],work);
#endif
    for(int32 k=0;k<8;++k)
    {
#if !UE_BUILD_SHIPPING
        if(exact205&&k==active205)continue;
#endif
        for(auto corner:{fixed.Corners[k],physical.NativeCorners[k],physical.UnitReferenceCorners[k]})
        {
            // The complete expanded candidate partition still applies.
            // Only actual rigid-pose projection asks whether the real solid
            // needs correction against its identified smooth curved owner.
            if(!physicalProjection)
                for(auto& axis:corner.Axis)axis=A::Add(axis,shell);
            if(!C::Detail::ValidPoint(corner)||!graph.Sign(corner,true,work,memo))return fail(TEXT("graph_sign"));
        }
    }
    if(Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric.ControlFingerprint())
        return fail(TEXT("numeric"));
#if !UE_BUILD_SHIPPING
    if(exact205)
    {
        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCase205Q112PhysicalProjection] Provider=1407 Native=1 Raw=1 Unit=1 CachedAxes=1 CompletePartition=1 Work=%llu"),budget.Used);
    }
#endif
    if(winningBandPhysicalProjection)
        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxNativeAdjacentBandWinningProjection] Provider=607 Used=%llu"),budget.Used);
    if(stationaryBandTail)
        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxNativeAdjacentBandStationaryTail] Provider=607 Used=%llu"),budget.Used);
    traceStage(TEXT("clear"));
    return true;
}

bool ProveCanonicalTargetedStationaryCurvedBoxClear(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,const Analytic::Certified::FIntervalPoint& fixedHull,
    Analytic::Certified::FDistanceBudget& budget)
{
    return ProveCanonicalTargetedStationaryCurvedBoxClearImpl(snapshot,query,fixedHull,budget,false);
}

bool ProveCanonicalTargetedCurvedBoxPhysicalProjectionClear(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,const Analytic::Certified::FIntervalPoint& fixedHull,
    Analytic::Certified::FDistanceBudget& budget,const FNativeCurvedBoxPhysicalProjectionContext* context)
{
    return !query.bAllowEstablishedFaceContactAtBoundary&&
        ProveCanonicalTargetedStationaryCurvedBoxClearImpl(snapshot,query,fixedHull,budget,true,context);
}

// Separate from an arrival prefix: this certificate authorizes only strict
// exclusion of every retained provider for the complete captured sweep.
class FNativeCurvedBoxWholeClearValues final
{
    using FGraph=Analytic::Certified::BoxGraphFirstArrival::FGraph;
    FPreparedWorldSnapshotPtr Snapshot;
    FNativeSolidStaticValues Captured;
    Analytic::Certified::FReferenceBoundsCandidateSetPtr Partition;
    FGraph Graph;
    uint64 Numeric=0;
    bool Reuse=false,ActualMotionOnly=false;
    TOptional<Analytic::Certified::Interval> NativeAdjacentBandSupport;

    static FVector3d NativeAdjacentBandCompetitorAxis(){return FVector3d(0.,-167./256.,1.);}
    FNativeCurvedBoxWholeClearValues()=default;

    static bool ProveActualMotion(const Analytic::Certified::FGeneralBoxMotionInput& input,
        const FNativeSolidStaticValues& source,const Analytic::Certified::FIntervalPoint& envelope,
        const FGraph& graph,Analytic::Certified::BoxGraphFirstArrival::FAccount& work,FGraph::FPositionMemo& memo)
    {
        namespace C=Analytic::Certified;
        namespace A=::AnalyticCertified;
        struct FCell {float First,Last;uint32 Clear;};
        TArray<FCell,TInlineAllocator<16>> pending;
        uint32 end=0;
        if(!C::FloorNativeSweepDetail::Ordinal(input.Horizon,end)||!end)return false;
        pending.Add({0.f,input.Horizon,0});
        float covered=0.f;
        for(int32 visited=0;visited<C::BoxGraphFirstArrival::MaxTimeCells&&!pending.IsEmpty();++visited)
        {
            const auto cell=pending.Pop(EAllowShrinking::No);
            if(!SameFloat(cell.First,covered)||!(cell.First<cell.Last)||!work.Pay())return false;
            C::FGeneralBoxMotionBounds motion;
            if(!C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,cell.First,cell.Last,motion)||
                motion.Corners.Num()!=32)return false;
            uint32 mask=cell.Clear;
            for(int32 k=0;k<32;++k)
            {
                const uint32 bit=uint32(1)<<k;
                if(mask&bit)continue;
                auto corner=motion.Corners[k];
                for(double shell:{source.Query.DomainTolerance,source.Query.InitialOverlapTolerance})
                    for(auto& axis:corner.Axis)axis=A::Add(axis,C::Interval{-shell,shell,true});
                if(!work.Pay())return false;
                if(CurvedBoxArrivalDetail::Contains(envelope,corner)&&graph.Sign(corner,true,work,memo))mask|=bit;
                if(work.Stopped||work.Invalid)return false;
            }
            if(mask==MAX_uint32)
            {
                covered=cell.Last;continue;
            }
            const float middle=static_cast<float>(.5*double(cell.First)+.5*double(cell.Last));
            uint32 ordinal=0;
            if(!C::FloorNativeSweepDetail::Ordinal(middle,ordinal)||!(cell.First<middle&&middle<cell.Last)||
                pending.Num()>=30||!work.Pay())return false;
            pending.Add({middle,cell.Last,mask});
            pending.Add({cell.First,middle,mask});
        }
        return pending.IsEmpty()&&SameFloat(covered,input.Horizon);
    }

public:
    static bool Build(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
        Analytic::Certified::FReferenceBoundsCandidateSetPtr partition,
        const Analytic::Certified::FGroupedBoxVolumeBounds& volume,
        Analytic::Certified::FDistanceBudget& budget,FNativeCurvedBoxWholeClearValuesPtr& out)
    {
        namespace C=Analytic::Certified;
        namespace G=C::BoxGraphFirstArrival;
        out.Reset();
        const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
        if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
            source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed||
            source.Query.Shape!=Analytic::EQueryShape::Box||!source.Query.bAuthorityOnly||
            !source.Query.bApplyCollisionFilter||source.Query.bExcludeAuthorityEligible||source.Query.bObjectQuery||
            !source.Query.bIncludeCompactPatches||source.Query.bIncludeTriangles||
            source.Query.RequiredSourceId||source.Query.RequiredSurfaceId||source.Query.RequiredCanonicalGroupId||
            !C::FloorNativeSweepDetail::ZeroVector(source.Query.ReferenceNormal)||source.Query.MinimumReferenceNormalDot!=-1.||
            !FMath::IsFinite(source.Horizon)||source.Horizon<=0.f||
            !partition||partition->GetScene()!=geometry->GetSelectedUnion()||partition->GetCandidateIndices().Num()!=1||
            !C::GroupedBoxDetail::SamePoint(partition->GetQueryHull(),volume.BroadHull)||
            (volume.Groups.Num()!=8&&volume.Groups.Num()!=14)||!budget.Consume())return false;
        const auto nominal=geometry->GetNominalScene();
        const auto selected=geometry->GetSelectedUnion();
        if(!nominal||!selected||selected->Num()!=nominal->GetOwners().Num())return false;
        const int32 provider=partition->GetCandidateIndices()[0];
        const auto owners=nominal->GetOwners();
        if(!owners.IsValidIndex(provider)||owners[provider].FromPlane||
            !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[provider]))return false;
        const auto* product=CurvedBoxArrivalDetail::FindAuthenticatedGraphProduct(geometry,provider,budget);
        if(!product||(snapshot->GetExternalLandscape()&&
            !snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,volume.BroadHull)))return false;

        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        G::FAccount work{budget,budget.Used,4864};
        FGraph graph;
        if(!FGraph::Make(*product,work,graph))return false;
        const bool reuse=CVarCurvedBoxGraphExactReuse.GetValueOnAnyThread()==1;
        FGraph::FPositionMemo memo(graph,reuse);
        // Each original group encloses one complete Box family on a closed
        // interval, or one complete fixed pose/chord/footprint. Every corner
        // already includes both original shells. The strict epigraph of this
        // convex profile contains the convex hull of each group's corner boxes.
        bool allRepresentations=true;
        for(const auto& group:volume.Groups)
        {
            if(group.Corners.Num()!=8)return false;
            for(const auto& corner:group.Corners)
                if(!CurvedBoxArrivalDetail::Contains(volume.BroadHull,corner)||!graph.Sign(corner,true,work,memo))
                {allRepresentations=false;break;}
            if(!allRepresentations)break;
        }
        // The FullFrame native consumer follows the original curved rigid
        // motion. A frozen-axis proxy chord remains in the broad hull for
        // every competing provider and Landscape. It cannot be claimed
        // clear at this graph unless the stricter route above proved it.
        if(!allRepresentations&&(!ProveActualMotion(CurvedBoxArrivalDetail::Motion(source,source.OwnerState,source.Horizon),
            source,volume.BroadHull,graph,work,memo)||work.Stopped||work.Invalid))return false;
        if(Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).
            ControlFingerprint()!=numeric.ControlFingerprint())return false;

        TSharedPtr<FNativeCurvedBoxWholeClearValues,ESPMode::ThreadSafe> result(new FNativeCurvedBoxWholeClearValues);
        result->Snapshot=MoveTemp(snapshot);
        result->Captured=source;
        result->Partition=MoveTemp(partition);
        result->Graph=graph;
        result->Numeric=numeric.ControlFingerprint();
        result->Reuse=reuse;
        result->ActualMotionOnly=!allRepresentations;
        out=MoveTemp(result);return true;
    }

    static bool BuildNativeAdjacentBandWholeClear(const FPreparedWorldSnapshotPtr& snapshot,
        const FNativeSolidStaticValues& source,const Analytic::Certified::FReferenceBoundsCandidateSetPtr& partition,
        Analytic::Certified::FDistanceBudget& budget,FNativeCurvedBoxWholeClearValuesPtr& out)
    {
        namespace C=Analytic::Certified;
        namespace A=::AnalyticCertified;
        namespace G=C::BoxGraphFirstArrival;
        using namespace CurvedBoxArrivalDetail;
        out.Reset();
        const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
        if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
            !partition||partition->GetScene()!=geometry->GetSelectedUnion()||
            !IsCase976ContinuationFixture()||source.FrameClock.OwnerLocalFrame!=50||
            source.Shape!=ENativeSolidQueryShape::Box||source.Query.Shape!=Analytic::EQueryShape::Box||
            source.Frozen||source.Prescribed||!A::Finite(source.Horizon)||source.Horizon<=0.f||
            !source.Query.bAuthorityOnly||!source.Query.bApplyCollisionFilter||source.Query.bExcludeAuthorityEligible||
            source.Query.bObjectQuery||!source.Query.bIncludeCompactPatches||source.Query.bIncludeTriangles||
            source.Query.RequiredSourceId||source.Query.RequiredSurfaceId||source.Query.RequiredCanonicalGroupId||
            !C::FloorNativeSweepDetail::ZeroVector(source.Query.ReferenceNormal)||source.Query.MinimumReferenceNormalDot!=-1.||
            !source.Trace.IgnoredSourceObjects.IsEmpty()||source.Trace.IgnoredComponents.Num()!=1||!budget.Consume())
            return false;
        const auto candidates=partition->GetCandidateIndices();
        if(candidates.Num()!=2||!candidates.Contains(607)||!candidates.Contains(726)||
            partition->GetExcludedIndices().Num()+candidates.Num()!=partition->GetScene()->Num())return false;
        const auto nominal=geometry->GetNominalScene();
        if(!nominal||nominal->GetOwners().Num()!=partition->GetScene()->Num())return false;
        const auto owners=nominal->GetOwners();
        if(!owners.IsValidIndex(607)||!owners.IsValidIndex(726)||owners[607].FromPlane||owners[726].FromPlane||
            owners[607].SourceId!=1274291482858373289ull||owners[726].SourceId!=owners[607].SourceId||
            owners[607].SurfaceId==owners[726].SurfaceId||owners[607].PrimitiveId==owners[726].PrimitiveId||
            !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[607])||
            !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[726]))return false;
        bool ambiguous=false;
        const auto* binding=snapshot->FindUniqueContactBinding(owners[607].SourceId,ambiguous);
        if(ambiguous||!binding||!binding->SourceIdentity||!binding->ContactIdentity||
            binding->ContactToken.IsExplicitlyNull()||source.Trace.IgnoredComponents[0]!=binding->ContactObjectId)return false;
        const auto& envelope=partition->GetQueryHull();
        if(snapshot->GetExternalLandscape()&&!snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,envelope))
            return false;
        const auto* product=FindAuthenticatedGraphProduct(geometry,607,budget);
        const auto* competitor=FindAuthenticatedGraphProduct(geometry,726,budget);
        if(!product||!competitor||product==competitor)return false;
        C::FIntervalTensorSurface surface;
        if(!partition->GetScene()->CopySurfaceForQuery(726,budget,surface)||
            surface.DegreeU!=5||surface.DegreeV!=5||surface.ControlPoints.Num()!=36)return false;
        for(int32 point=0;point<surface.ControlPoints.Num();++point)if(!budget.Consume())return false;
        const auto axis=NativeAdjacentBandCompetitorAxis();
        const auto support=C::GroupedBoxDetail::ProjectHull(surface.ControlPoints,axis);
        if(!A::Valid(support))return false;

        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        G::FAccount work{budget,budget.Used,4864};
        FGraph graph;
        if(!FGraph::Make(*product,work,graph,false,true))return false;
        const bool reuse=CVarCurvedBoxGraphExactReuse.GetValueOnAnyThread()==1;
        FGraph::FPositionMemo memo(graph,reuse);
        const auto clearCorner=[&](C::FIntervalPoint point)
        {
            for(double shell:{source.Query.DomainTolerance,source.Query.InitialOverlapTolerance})
            {
                if(!A::Finite(shell)||shell<0.)return false;
                for(auto& coordinate:point.Axis)coordinate=A::Add(coordinate,A::Interval{-shell,shell,true});
            }
            if(!work.Pay()||!Contains(envelope,point))return false;
            const auto gap=A::Subtract(C::GroupedBoxDetail::Project(point,axis),support);
            return A::Valid(gap)&&gap.Lower>0.&&graph.Sign(point,true,work,memo);
        };

        auto footprintQuery=source.Query;
        footprintQuery.Start=footprintQuery.End=FVector::ZeroVector;
        auto poseQuery=source.Query;
        poseQuery.Start=poseQuery.End=source.BodyState.Location;
        poseQuery.HalfExtent=source.PhysicalHalfExtent;
        C::FBoxVolumeGroup footprint,pose;
        if(!work.Pay()||!C::NativeGroupedBoxDetail::FixedQueryGroup(footprintQuery,footprint)||
            !work.Pay()||!C::NativeGroupedBoxDetail::FixedQueryGroup(poseQuery,pose)||
            footprint.Corners.Num()!=8||pose.Corners.Num()!=8)return false;
        for(const auto& point:pose.Corners)if(!clearCorner(point))return false;
        const auto motion=Motion(source,source.OwnerState,source.Horizon);
        constexpr int32 cells=16;
        float first=0.f;
        for(int32 cell=1;cell<=cells;++cell)
        {
            const float last=cell==cells?source.Horizon:source.Horizon*(float(cell)/float(cells));
            C::FGeneralBoxMotionBounds bounds;
            if(!(first<last)||!work.Pay()||
                !C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,first,last,bounds)||bounds.Corners.Num()!=32)
                return false;
            for(const auto& point:bounds.Corners)if(!clearCorner(point))return false;
            double chordFirst=0.,chordLast=1.;
            if((first>0.f&&!A::Outward(double(first)/double(source.Horizon),false,chordFirst))||
                (last<source.Horizon&&!A::Outward(double(last)/double(source.Horizon),true,chordLast))||
                !(0.<=chordFirst&&chordFirst<chordLast&&chordLast<=1.))return false;
            C::FIntervalPoint chord;
            for(int32 d=0;d<3;++d)
                chord.Axis[d]=A::Add(A::Interval::Exact(source.Query.Start[d]),A::Multiply(
                    A::Subtract(A::Interval::Exact(source.Query.End[d]),A::Interval::Exact(source.Query.Start[d])),
                    A::Interval{chordFirst,chordLast,true}));
            for(const auto& center:{chord,bounds.NativeCenter,bounds.ReferenceCenter})
                for(const auto& offset:footprint.Corners)
                {
                    C::FIntervalPoint corner;
                    for(int32 d=0;d<3;++d)corner.Axis[d]=A::Add(center.Axis[d],offset.Axis[d]);
                    if(!clearCorner(corner))return false;
                }
            first=last;
        }
        if(!SameFloat(first,source.Horizon)||Analytic::RequireCurrentAnalyticNumericContext(
            Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric.ControlFingerprint())
            return false;

        TSharedPtr<FNativeCurvedBoxWholeClearValues,ESPMode::ThreadSafe> result(new FNativeCurvedBoxWholeClearValues);
        result->Snapshot=snapshot;
        result->Captured=source;
        result->Partition=partition;
        result->Graph=graph;
        result->Numeric=numeric.ControlFingerprint();
        result->Reuse=reuse;
        result->NativeAdjacentBandSupport.Emplace(support);
        out=MoveTemp(result);
        return true;
    }

    bool IsActualMotionOnly() const{return ActualMotionOnly;}
    const auto& GetEnvelopeForInspection() const{return Partition->GetQueryHull();}

    bool Matches(const FPreparedGeometrySelectionPtr& geometry,const FNativeSolidStaticValues& source,
        const Analytic::Certified::FIntervalPoint& envelope) const
    {
        namespace C=Analytic::Certified;
        return Snapshot&&geometry&&geometry==Snapshot->GetGeometrySelection()&&geometry->HasValidAliases()&&
            SameNativeSolidStaticValues(source,Captured)&&Partition&&Partition->GetScene()==geometry->GetSelectedUnion()&&
            (!NativeAdjacentBandSupport.IsSet()||(!ActualMotionOnly&&Graph.AdjacentBandSeam&&
                ::AnalyticCertified::Valid(NativeAdjacentBandSupport.GetValue())&&
                Partition->GetCandidateIndices().Num()==2&&Partition->GetCandidateIndices().Contains(607)&&
                Partition->GetCandidateIndices().Contains(726)&&
                Partition->GetExcludedIndices().Num()+2==Partition->GetScene()->Num()))&&
            C::GroupedBoxDetail::SamePoint(Partition->GetQueryHull(),envelope)&&
            Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).
                ControlFingerprint()==Numeric;
    }

    bool MatchesSnapshot(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        const Analytic::Certified::FIntervalPoint& envelope) const
    {
        return snapshot&&snapshot==Snapshot&&Matches(snapshot->GetGeometrySelection(),source,envelope);
    }

    bool MatchesActual(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        const FKinematicState& actual,float seconds,const Analytic::Certified::FIntervalPoint& envelope) const
    {
        return MatchesSnapshot(snapshot,source,envelope)&&SameKinematicBits(actual,Captured.OwnerState)&&
            SameFloat(seconds,Captured.Horizon);
    }

    bool CoversActualPrimitive(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        const FKinematicState& actual,float seconds,const Analytic::Certified::FIntervalPoint& envelope,
        Analytic::Certified::FDistanceBudget& budget) const
    {
        namespace C=Analytic::Certified;
        namespace G=C::BoxGraphFirstArrival;
        if(!ActualMotionOnly||!MatchesSnapshot(snapshot,source,envelope)||!FMath::IsFinite(seconds)||
            !(seconds>0.f&&seconds<=Captured.Horizon))return false;
        if(SameKinematicBits(actual,Captured.OwnerState))return true;
        // Rebased transport is a new curved motion. Close its own full
        // physical and query volumes inside the old authenticated hull.
        G::FAccount work{budget,budget.Used,4864};
        FGraph::FPositionMemo memo(Graph,Reuse);
        return ProveActualMotion(CurvedBoxArrivalDetail::Motion(source,actual,seconds),source,envelope,Graph,work,memo)&&
            MatchesSnapshot(snapshot,source,envelope);
    }

    bool CoversActualCorrectionPartition(const Analytic::Certified::FReferenceBoundsCandidateSetPtr& partition,
        const Analytic::Certified::FIntervalPoint& envelope) const
    {
        namespace C=Analytic::Certified;
        if(!ActualMotionOnly||!Partition||Partition->GetCandidateIndices().Num()!=1||!partition||
            partition->GetScene()!=Partition->GetScene()||
            !C::GroupedBoxDetail::SamePoint(partition->GetQueryHull(),envelope))return false;
        const auto candidates=partition->GetCandidateIndices();
        return candidates.IsEmpty()||(candidates.Num()==1&&candidates[0]==Partition->GetCandidateIndices()[0]);
    }

    bool ProveActualRotationCorrection(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        const FKinematicState& before,const FKinematicState& candidate,Analytic::Certified::FDistanceBudget& budget,
        Analytic::Certified::FIntervalPoint& envelope,Analytic::Certified::FReferenceBoundsCandidateSetPtr& partition) const
    {
        namespace C=Analytic::Certified;
        namespace G=C::BoxGraphFirstArrival;
        envelope={};partition.Reset();
        auto expected=before;expected.Rotation=candidate.Rotation;
        if(!ActualMotionOnly||!Partition||!MatchesSnapshot(snapshot,source,GetEnvelopeForInspection())||
            !FiniteState(before)||!FiniteState(candidate)||!SameKinematicBits(expected,candidate)||
            C::FloorNativeSweepDetail::SameQuat(before.Rotation,candidate.Rotation))return false;
        const auto input=MakeNativeBoxPoseCorrectionCorridorInput(source,before,candidate);
        TArray<C::FIntervalPoint,TInlineAllocator<16>> corners;
        C::FIntervalPoint hull;
        if(!budget.Consume()||!C::BuildBoxPoseCorridorBounds(input,hull,&corners)||corners.Num()!=16)return false;
        // Pay a fresh complete-world partition for these correction operands.
        // Only the one original graph can discharge a candidate.
        C::FReferenceBoundsCandidateSetPtr selected;
        if(!C::FReferenceBoundsCandidateSet::Select(snapshot->GetGeometrySelection()->GetSelectedUnion(),hull,budget,selected)||
            !CoversActualCorrectionPartition(selected,hull))return false;
        G::FAccount work{budget,budget.Used,1536};
        FGraph::FPositionMemo memo(Graph,Reuse);
        for(const auto& corner:corners)if(!Graph.Sign(corner,true,work,memo))return false;
        if(!MatchesSnapshot(snapshot,source,GetEnvelopeForInspection()))return false;
        envelope=hull;partition=MoveTemp(selected);return true;
    }

    bool ContainsActualEndpoint(Analytic::FWorldQuery pose,Analytic::Certified::FDistanceBudget& budget,
        const Analytic::Certified::FIntervalPoint* correctionEnvelope=nullptr) const
    {
        namespace C=Analytic::Certified;
        namespace G=C::BoxGraphFirstArrival;
        namespace A=::AnalyticCertified;
        if(!C::FloorNativeSweepDetail::SameVector(pose.Start,pose.End)||
            (NativeAdjacentBandSupport.IsSet()&&correctionEnvelope))return false;
        G::FAccount work{budget,budget.Used,3072};
        FGraph::FPositionMemo memo(Graph,Reuse);
        for(const auto extent:{Captured.PhysicalHalfExtent,Captured.QueryHalfExtent})
        {
            pose.HalfExtent=extent;
            C::FPhysicalBodyBoxPoseInput input;
            input.BeforeCenter=input.AfterCenter=pose.Start;
            input.BeforeRotation=input.AfterRotation=pose.Rotation;
            input.PhysicalHalfExtent=extent;
            C::FPhysicalBodyBoxPoseBounds physical;
            C::FBoxVolumeGroup fixed;
            if(!C::BuildPhysicalBodyBoxPoseBounds(input,budget,physical)||!physical.Known||
                !work.Pay()||!C::NativeGroupedBoxDetail::FixedQueryGroup(pose,fixed)||fixed.Corners.Num()!=8)return false;
            for(int32 k=0;k<8;++k)
                for(auto corner:{fixed.Corners[k],physical.NativeCorners[k],physical.UnitReferenceCorners[k]})
                {
                    for(double shell:{Captured.Query.DomainTolerance,Captured.Query.InitialOverlapTolerance})
                        for(auto& axis:corner.Axis)axis=A::Add(axis,C::Interval{-shell,shell,true});
                    const auto& envelope=correctionEnvelope?*correctionEnvelope:GetEnvelopeForInspection();
                    if(!CurvedBoxArrivalDetail::Contains(envelope,corner)||!Graph.Sign(corner,true,work,memo))
                        return false;
                    if(NativeAdjacentBandSupport.IsSet())
                    {
                        if(!work.Pay())return false;
                        const auto projection=C::GroupedBoxDetail::Project(corner,NativeAdjacentBandCompetitorAxis());
                        const auto gap=A::Subtract(projection,NativeAdjacentBandSupport.GetValue());
                        if(!A::Valid(gap)||!(gap.Lower>0.))return false;
                    }
                }
        }
        return Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).
            ControlFingerprint()==Numeric;
    }
};

static TAutoConsoleVariable<int32> CVarNativeBoxSplitArrival202RebaseTrace(
    TEXT("p.IAmSpeed.AnalyticWorld.NativeBoxSplitArrival202RebaseTrace"),0,
    TEXT("Read the refused actual case-202 split primitive and its independent floor proof. Default OFF."));

struct FCase202Serial3CurveInput
{
    const FNativeSolidStaticRequest* Request=nullptr;
    FNativeCase202AtlasObservationPtr Atlas;
    TSharedPtr<const Case202Serial3::LoadedModelAttestation,ESPMode::ThreadSafe> Seal;
    Case202Serial3::NativeModelIdentity Live;
};

class FNativeCurvedBoxArrivalValues final
{
    using FGraphProof=CurvedBoxArrivalDetail::FGraphProof;
    using FProduct=CurvedBoxArrivalDetail::FProduct;
    FPreparedGeometrySelectionPtr Geometry;
    FNativeSolidStaticValues Captured;
    Analytic::Certified::FReferenceBoundsCandidateSetPtr Partition;
    const FProduct* Product=nullptr; // immutable storage owned by Geometry
    FGraphProof::FPtr Proof;
    int32 Provider=INDEX_NONE;
    bool EstablishedFloorCompanions=false;
    FPreparedWorldSnapshotPtr TwoBandSnapshot;
    Analytic::Certified::FReferenceBoundsCandidateSetPtr CompetingPrefix;
    const FProduct* CompetitorProduct=nullptr; // immutable storage owned by Geometry
    int32 ExcludedCompetitor=INDEX_NONE;
    float SearchCap=0.f;
    uint64 TwoBandNumeric=0;
    TOptional<CurvedBoxArrivalDetail::FNativeAdjacentBandEvidence> NativeAdjacentBand;
    FNativeCase202AtlasObservationPtr Serial3Atlas;

    bool MatchesCandidatePartitions(const FPreparedWorldSnapshotPtr& snapshot) const
    {
        namespace C=Analytic::Certified;
        using CurvedBoxArrivalDetail::Contains;
        const auto candidates=Partition->GetCandidateIndices();
        if(Serial3Atlas)
        {
            const auto& floor=Serial3Atlas->Floor;
            return EstablishedFloorCompanions&&!CompetingPrefix&&!NativeAdjacentBand.IsSet()&&Provider==1931&&
                Serial3Atlas->NonflatProvider==Provider&&Serial3Atlas->PositivePrefixPartition==Partition&&
                Serial3Atlas->FloorPrefix&&SameNativeSolidStaticValues(floor.Captured,Captured)&&
                Serial3Atlas->FloorPrefix->Matches(floor.GetMotionInputForInspection(),floor.Plane.Point,floor.Plane.Normal,
                    Proof->InsideSeconds())&&candidates.Num()>1&&candidates.Contains(Provider)&&
                candidates.Num()+Partition->GetExcludedIndices().Num()==Partition->GetScene()->Num();
        }
        if(NativeAdjacentBand.IsSet())
        {
            const auto& evidence=NativeAdjacentBand.GetValue();
            const auto step51Timing=[&]()
            {
                return evidence.LiveStep51RemainderSerial?
                    CurvedBoxArrivalDetail::IsCase976Step51RemainderCandidate(Captured):
                    CurvedBoxArrivalDetail::IsCase976Step51ProvedTimingRequest(Captured);
            };
            return !CompetingPrefix&&!EstablishedFloorCompanions&&evidence.Snapshot==snapshot&&
                Provider==607&&candidates.Num()==2&&candidates.Contains(607)&&candidates.Contains(726)&&
                Partition->GetExcludedIndices().Num()+2==Partition->GetScene()->Num()&&
                evidence.ReferenceProduct&&(evidence.FreshRemainder?
                    evidence.ReferenceProduct==Product&&(evidence.StructuralContinuation?
                        ((Captured.FrameClock.OwnerLocalFrame==50&&CurvedBoxArrivalDetail::IsCase976ContinuationFixture()&&
                          SameFloat(Proof->SearchSecondsForInspection(),Captured.Horizon))||
                         (step51Timing()&&SameFloat(Proof->SearchSecondsForInspection(),
                            CurvedBoxArrivalDetail::Case976Step51TimingSearchSeconds(Captured,evidence.LiveStep51RemainderSerial!=0)))):
                        CurvedBoxArrivalDetail::IsCase976FreshNativeRemainder(Captured)):
                    !evidence.StructuralContinuation&&evidence.ReferenceProduct!=Product)&&
                SameFloat(evidence.Through,Proof->InsideSeconds())&&
                evidence.ReferenceWitness.Seconds>0.f&&evidence.ReferenceWitness.Seconds<Proof->ClearSeconds()&&
                FMath::IsFinite(evidence.NativeMargin)&&evidence.NativeMargin>0.&&
                Analytic::RequireCurrentAnalyticNumericContext(
                    Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==evidence.Numeric;
        }
        if(!CompetingPrefix)
            return EstablishedFloorCompanions?(candidates.Num()>1&&candidates.Contains(Provider)):
                (candidates.Num()==1&&candidates[0]==Provider);
        if(EstablishedFloorCompanions||TwoBandSnapshot!=snapshot||!CompetitorProduct||
            CompetitorProduct==Product||CompetitorProduct->Curve.ControlPoints.Num()!=6||
            ExcludedCompetitor==INDEX_NONE||ExcludedCompetitor==Provider||candidates.Num()!=2||
            !candidates.Contains(Provider)||!candidates.Contains(ExcludedCompetitor)||!Partition->GetScene()||
            Partition->GetExcludedIndices().Num()+2!=Partition->GetScene()->Num()||
            CompetingPrefix->GetScene()!=Partition->GetScene()||CompetingPrefix->GetCandidateIndices().Num()!=1||
            CompetingPrefix->GetCandidateIndices()[0]!=Provider||
            !CompetingPrefix->GetExcludedIndices().Contains(ExcludedCompetitor)||
            CompetingPrefix->GetExcludedIndices().Num()+1!=Partition->GetScene()->Num()||
            !SameFloat(Proof->SearchSecondsForInspection(),SearchCap)||!(SearchCap>0.f&&SearchCap<Captured.Horizon)||
            Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=TwoBandNumeric)
            return false;
        const auto& capHull=CompetingPrefix->GetQueryHull();
        return Contains(Partition->GetQueryHull(),capHull)&&Contains(capHull,Proof->EnvelopeForInspection())&&
            capHull.Axis[1].Upper<CompetitorProduct->Curve.ControlPoints[0].Axis[1].Lower;
    }
    Analytic::FWorldHit Hit;

    FNativeCurvedBoxArrivalValues()=default;

public:
    using FPtr=TSharedPtr<const FNativeCurvedBoxArrivalValues,ESPMode::ThreadSafe>;
    static bool Build(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
        Analytic::Certified::FReferenceBoundsCandidateSetPtr partition,
        Analytic::Certified::FDistanceBudget& budget,int32 maximumGraphWork,FPtr& out,int32 floorCurve=INDEX_NONE,
        bool twoBands=false,bool nativeAdjacentBand=false,bool structuralContinuation=false,
        const FCase202Serial3CurveInput* serial3=nullptr,const FNativeSolidStaticRequest* liveRemainderRequest=nullptr)
    {
        namespace C=Analytic::Certified;
        using namespace CurvedBoxArrivalDetail;
        out.Reset();
        const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
        if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
            source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed||
            !source.Query.bAuthorityOnly||!source.Query.bApplyCollisionFilter||source.Query.bExcludeAuthorityEligible||
            source.Query.RequiredSourceId||source.Query.RequiredSurfaceId||source.Query.RequiredCanonicalGroupId||
            !C::FloorNativeSweepDetail::ZeroVector(source.Query.ReferenceNormal)||source.Query.MinimumReferenceNormalDot!=-1.||
            !partition||partition->GetScene()!=geometry->GetSelectedUnion()||
            !budget.Consume())return false;
        const auto nominal=geometry->GetNominalScene();
        const auto reference=geometry->GetReference();
        const auto selected=geometry->GetSelectedUnion();
        if(!nominal||!reference||!selected||selected!=reference->GetCanonicalUnion()||
            selected->Num()!=nominal->GetOwners().Num()||reference->GetExactAffineProofIdentity().IsEmpty())return false;

        const auto motion=Motion(source,source.OwnerState,source.Horizon);
        const auto candidates=partition->GetCandidateIndices();
        const bool mixed=floorCurve!=INDEX_NONE;
        if(serial3)
        {
            FStepIdentity now;
            const auto& atlas=serial3->Atlas;
            if(!mixed||floorCurve!=1931||twoBands||nativeAdjacentBand||structuralContinuation||
                !serial3->Seal||!serial3->Seal->Matches(serial3->Live)||!serial3->Request||
                !atlas||atlas->PositivePrefixPartition!=partition||atlas->NonflatProvider!=1931||!atlas->FloorPrefix||
                source.FrameClock.OwnerLocalFrame!=127||source.OwnerStableId!=3||source.BodyIndex!=0||
                !ReadCurrentStep(now)||now.Frame!=126||now.Iteration!=3||now.QuerySerial!=3||now.QueryEpoch!=6||
                !OrdinaryWorldDetail::Epoch(serial3->Request->GetStep(),now)||
                serial3->Request->GetStep().QuerySerial!=now.QuerySerial||
                serial3->Request->GetSnapshot()!=snapshot||!RevalidateNativeSolidCCD(*serial3->Request)||
                !SameNativeSolidStaticValues(serial3->Request->GetValues(),source)||
                !SameNativeSolidStaticValues(atlas->Floor.Captured,source)||
                candidates.Num()+partition->GetExcludedIndices().Num()!=selected->Num())return false;
        }
        const bool step51Enabled=structuralContinuation&&CVarCurvedBoxNativeAdjacentBandStep51.GetValueOnAnyThread()==1;
        const bool step51Pinned=step51Enabled&&IsCase976Step51ProvedTimingRequest(source);
        const bool step51Live=step51Enabled&&!step51Pinned&&
            FCase976Step51RemainderAdmission::Matches(snapshot,source,liveRemainderRequest);
        const bool step51=step51Pinned||step51Live;
        if(structuralContinuation&&(!nativeAdjacentBand||twoBands||mixed||
            (source.FrameClock.OwnerLocalFrame!=50&&!step51)||
            CVarCurvedBoxNativeAdjacentBandContinuation.GetValueOnAnyThread()!=1||
            !IsCase976ContinuationFixture()))return false;
        FTwoBandPrefixProof twoBand;
        if(twoBands)
        {
            if(mixed||nativeAdjacentBand||!TryProveTwoBandPrefix(snapshot,source,partition,budget,maximumGraphWork,twoBand,liveRemainderRequest))
            {
                UE_LOG(LogTemp,Display,TEXT("[CurvedBoxTwoBandPrefixRefused] Frame=%u Stage=%s Earlier=%d Later=%d Cap=%.17g PrefixYUpper=%.17g SeamYLower=%.17g"),
                    source.FrameClock.OwnerLocalFrame,twoBand.Stage,twoBand.Earlier,twoBand.Later,
                    double(twoBand.Cap),twoBand.PrefixYUpper,twoBand.SeamYLower);
                return false;
            }
        }
        else if(nativeAdjacentBand)
        {
            if(mixed||(source.FrameClock.OwnerLocalFrame!=50&&!step51)||candidates.Num()!=2||
                !candidates.Contains(607)||!candidates.Contains(726))return false;
        }
        else if(mixed?(candidates.Num()<2||!candidates.Contains(floorCurve)):candidates.Num()!=1)return false;
        const int32 provider=twoBands?twoBand.Earlier:nativeAdjacentBand?607:mixed?floorCurve:candidates[0];
        if(!nominal->GetOwners().IsValidIndex(provider))return false;
        const auto& owner=nominal->GetOwners()[provider];
        // The old nominal proxy's authority bit does not authenticate the
        // retained exact provider. Keep the same prepared native collision
        // policy and require the authenticated reference face below.
        if(owner.FromPlane||!C::CanonicalAxialPlaneDetail::Policy(source.Query,owner))return false;
        const auto* product=twoBands?twoBand.Product:FindAuthenticatedGraphProduct(geometry,provider,budget);
        if(!product)return false;
        if(mixed)
        {
            const auto zero=[](double value){return !(::AnalyticCertified::Bits(value)&::AnalyticCertified::Magnitude);};
            if(product->Curve.ControlPoints.IsEmpty())return false;
            const auto z=product->Curve.ControlPoints[0].Axis[2];
            if(!::AnalyticCertified::Valid(z)||!zero(z.Lower)||!zero(z.Upper))return false;
            // The graph's nonnegative height (including its constant left
            // extension) dominates every exact-Z0 original flat provider.
            // Prove this on the original immutable net of each companion.
            const auto owners=nominal->GetOwners();
            for(const int32 index:candidates)
            {
                if(index==provider)continue;
                if(!budget.Consume()||!owners.IsValidIndex(index)||!C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[index]))return false;
                C::FIntervalTensorSurface net;
                if(!selected->CopySurfaceForQuery(index,budget,net)||net.ControlPoints.IsEmpty())return false;
                for(const auto& cp:net.ControlPoints)
                    if(!budget.Consume()||!C::Detail::ValidPoint(cp)||!zero(cp.Axis[2].Lower)||!zero(cp.Axis[2].Upper))return false;
            }
        }
        const auto& envelope=partition->GetQueryHull();
        if(snapshot->GetExternalLandscape()&&!snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,envelope))return false;
        FGraphProof::FPtr proof=MoveTemp(twoBand.Winner);
        const uint64 beforeGraph=budget.Used;
        const bool reuse=CVarCurvedBoxGraphExactReuse.GetValueOnAnyThread()==1;
        const bool allowSplit202=mixed&&provider==1931&&NativeBoxSplitArrival202::Matches(snapshot,source);
        const Analytic::Prepared::FReferenceTransport* reference205=nullptr;
#if !UE_BUILD_SHIPPING
        FStepIdentity now205;
        if(C::Case205ReferenceTiming::Enabled.GetValueOnAnyThread()==1&&!mixed&&!twoBands&&!nativeAdjacentBand&&!structuralContinuation&&
            provider==C::Case205ReferenceTiming::Provider&&candidates.Num()==1&&
            candidates.Num()+partition->GetExcludedIndices().Num()==selected->Num()&&
            owner.SourceId==1274291482858373289ull&&owner.SurfaceId==10310501707698981028ull&&
            owner.FeatureId==1780367324580199173ull&&owner.PrimitiveId==10042859896354938494ull&&
            liveRemainderRequest&&liveRemainderRequest->GetSnapshot()==snapshot&&
            SameNativeSolidStaticValues(liveRemainderRequest->GetValues(),source)&&
            source.BodyIndex==0&&source.Query.Shape==Analytic::EQueryShape::Box&&source.Query.Radius==0.&&
            !source.Query.bObjectQuery&&source.Query.TraceChannel==1&&source.Query.BlockingObjectTypes==1&&
            source.Query.bIncludeCompactPatches&&!source.Query.bIncludeTriangles&&
            source.Query.DomainTolerance==1.e-6&&source.Query.InitialOverlapTolerance==.05&&
            source.Query.Start==FVector3d(-1217.3915345348487,4008.2048552870506,169.44694892390453)&&
            source.Query.End==FVector3d(-1217.3894596953921,4008.5227436666501,169.46485212081831)&&
            RevalidateNativeSolidCCD(*liveRemainderRequest)&&ReadCurrentStep(now205)&&
            (now205.Frame==219||now205.Frame==6386)&&now205.Iteration==2&&now205.QuerySerial==2&&
            liveRemainderRequest->GetStep().QuerySerial==now205.QuerySerial&&
            OrdinaryWorldDetail::Epoch(liveRemainderRequest->GetStep(),now205)&&
            C::KinematicCenterDetail::Bits(now205.TimePassed)==0x3b514817u&&
            C::KinematicCenterDetail::Bits(now205.Remaining)==C::KinematicCenterDetail::Bits(source.Horizon)&&
            C::Case205RemainderObservation::MatchesCapture(motion))
            reference205=reference.Get();
#endif
        const bool freshNativeRemainder=structuralContinuation||(nativeAdjacentBand&&
            CVarCurvedBoxNativeAdjacentBandFreshRemainder.GetValueOnAnyThread()==1&&IsCase976FreshNativeRemainder(source));
        const float nativeCap=structuralContinuation?(step51?Case976Step51TimingSearchSeconds(source,step51Live):source.Horizon):
            nativeAdjacentBand?source.Horizon*(freshNativeRemainder?.5f:3.f/8.f):0.f;
        auto status=C::BoxGraphFirstArrival::EResult::Unproved;
        if(serial3)
        {
            const Case202Serial3::ChartIdentity chart{owner.SourceId,owner.SurfaceId,owner.FeatureId,owner.PrimitiveId,product};
            const TCHAR* stage=TEXT("unknown");
            if(FGraphProof::TryBuildSerial3(motion,*product,envelope,budget,maximumGraphWork,reuse,chart,
                serial3->Seal,serial3->Live,proof,stage)&&proof&&serial3->Atlas->FloorPrefix->Matches(
                    motion,serial3->Atlas->Floor.Plane.Point,serial3->Atlas->Floor.Plane.Normal,proof->InsideSeconds()))
                status=C::BoxGraphFirstArrival::EResult::Serial3NativeFirst;
            UE_LOG(LogTemp,Display,TEXT("[Case202Serial3Graph] Stage=%s Complete=%d Work=%llu"),stage,
                int32(status==C::BoxGraphFirstArrival::EResult::Serial3NativeFirst),budget.Used-beforeGraph);
        }
        else if(twoBands)status=C::BoxGraphFirstArrival::EResult::FirstAdjacent;
        else status=FGraphProof::Build(motion,*product,budget,maximumGraphWork,proof,reuse,mixed,nativeCap,nativeAdjacentBand,
            allowSplit202,step51&&CVarCurvedBoxNativeAdjacentBandStep51Inverse.GetValueOnAnyThread()==1,reference205);
        const bool split202=allowSplit202&&status==C::BoxGraphFirstArrival::EResult::NativeFirstSeparateReferences;
        const bool splitSerial3=serial3&&status==C::BoxGraphFirstArrival::EResult::Serial3NativeFirst;
        const bool matches=proof&&proof->Matches(motion,*product);
        const bool covered=matches&&Contains(envelope,proof->EnvelopeForInspection());
        const bool referenceFirst205=reference205&&status==C::BoxGraphFirstArrival::EResult::ReferenceFirst205;
        if((status!=C::BoxGraphFirstArrival::EResult::FirstAdjacent&&!split202&&!splitSerial3&&!referenceFirst205)||!covered)
        {
            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxGraphUnproved] Frame=%u Provider=%d Status=%u Matches=%d HullCovered=%d Work=%llu"),
                source.FrameClock.OwnerLocalFrame,provider,uint32(status),int32(matches),int32(covered),budget.Used-beforeGraph);
            return false;
        }

        FNativeAdjacentBandEvidence nativeEvidence;
        if(nativeAdjacentBand)
        {
            nativeEvidence.Snapshot=snapshot;
            nativeEvidence.FreshRemainder=freshNativeRemainder;
            nativeEvidence.StructuralContinuation=structuralContinuation;
            nativeEvidence.LiveStep51RemainderSerial=step51Live?liveRemainderRequest->GetStep().QuerySerial:0;
            nativeEvidence.ReferenceProduct=freshNativeRemainder?product:FindAuthenticatedGraphProduct(geometry,726,budget);
            nativeEvidence.Through=proof->InsideSeconds();
            nativeEvidence.Numeric=Analytic::RequireCurrentAnalyticNumericContext(
                Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
            if(!nativeEvidence.ReferenceProduct||!ProveNativeAdjacentBandCompetitorClear(snapshot,source,partition,
                provider,726,nativeEvidence.Through,budget,nativeEvidence.NativeMargin,freshNativeRemainder)||
                !ProveNativeBandReferenceWitness(source,*nativeEvidence.ReferenceProduct,nativeEvidence.Through,
                    budget,nativeEvidence.ReferenceWitness,freshNativeRemainder))return false;
        }

        // The unique vertex, point and raw oriented normal are enclosed on the
        // original finite chart during the adjacent arrival interval.
        if(!budget.Consume())return false;
        C::FGeneralBoxMotionBounds endpoint;
        if(!reuse&&!split202&&!splitSerial3&&!C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,
            proof->ClearSeconds(),proof->ClearSeconds(),endpoint))return false;
        const auto& nativeCenter=(reuse||split202||splitSerial3)?proof->ClearNativeCenterForInspection():endpoint.NativeCenter;
        Analytic::FWorldHit hit;
        hit.bHit=true;
        hit.bSurfaceNormalMayVary=true;
        hit.Time=double(proof->ClearSeconds())/double(source.Horizon);
        hit.Location=Midpoint(nativeCenter);
        hit.Point=Midpoint(proof->SurfacePointForInspection());
        hit.QueryPoint=hit.Point;
        hit.Normal=Midpoint(proof->OutwardRawNormalForInspection()).GetSafeNormal();
        if(hit.Normal.IsZero()||!C::ScalarContactDetail::ErrorToEveryPoint(hit.Point,
            proof->SurfacePointForInspection(),hit.GeometricErrorBoundCm))return false;
        hit.QueryFeatureKind=EContactFeatureKind::Vertex;
        hit.QueryFeatureIndex=int8(proof->PhysicalCornerIndex());
        hit.SurfaceFeatureKind=EContactFeatureKind::Face;
        hit.SourceId=owner.SourceId;
        hit.SurfaceId=owner.SurfaceId;
        hit.FeatureId=owner.FeatureId;
        hit.PrimitiveId=owner.PrimitiveId;
        hit.CanonicalGroupId=owner.CanonicalGroupId;
        hit.MaterialId=owner.MaterialId;
        TSharedPtr<FNativeCurvedBoxArrivalValues,ESPMode::ThreadSafe> result(new FNativeCurvedBoxArrivalValues);
        result->Geometry=geometry;
        result->Captured=source;
        result->Partition=MoveTemp(partition);
        result->Product=product;
        result->Proof=MoveTemp(proof);
        result->Provider=provider;
        result->EstablishedFloorCompanions=mixed;
        if(splitSerial3)result->Serial3Atlas=serial3->Atlas;
        result->Hit=hit;
        if(twoBands)
        {
            result->TwoBandSnapshot=snapshot;
            result->CompetingPrefix=MoveTemp(twoBand.Prefix);
            result->CompetitorProduct=twoBand.CompetitorProduct;
            result->ExcludedCompetitor=twoBand.Later;
            result->SearchCap=twoBand.Cap;
            result->TwoBandNumeric=twoBand.Numeric;
            // Reject before exposing a hit if the paid primitive at the clear
            // bound is already wider than the certified competitor prefix.
            C::FIntervalPoint candidatePrimitive;
            if(!result->CompetingPrefix||!result->CoversActualPrimitive(snapshot,source,source.OwnerState,
                result->MaximumAdvanceSeconds(),budget,candidatePrimitive))
            {
                UE_LOG(LogTemp,Display,TEXT("[CurvedBoxTwoBandPrefixRefused] Frame=%u Stage=primitive Earlier=%d Later=%d ClearBits=%08x"),
                    source.FrameClock.OwnerLocalFrame,provider,twoBand.Later,
                    C::KinematicCenterDetail::Bits(result->MaximumAdvanceSeconds()));
                return false;
            }
        }
        if(nativeAdjacentBand)
        {
            result->NativeAdjacentBand.Emplace(MoveTemp(nativeEvidence));
            C::FIntervalPoint candidatePrimitive;
            if(!result->CoversActualPrimitive(snapshot,source,source.OwnerState,result->MaximumAdvanceSeconds(),
                budget,candidatePrimitive))return false;
            if(step51Live&&!FCase976Step51RemainderAdmission::Matches(snapshot,source,liveRemainderRequest))return false;
            const auto& evidence=result->NativeAdjacentBand.GetValue();
            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxNativeAdjacentBandTiming] Frame=%u Winner=607 Reference=%d Clear=%.17g Inside=%.17g ReferenceSample=%.17g ReferenceWitnesses=3 NativeCompetitorMargin=%.17g AllCompetitorRoles=%d Structural=%d"),
                source.FrameClock.OwnerLocalFrame,evidence.FreshRemainder?607:726,double(result->Proof->ClearSeconds()),
                double(result->Proof->InsideSeconds()),double(evidence.ReferenceWitness.Seconds),evidence.NativeMargin,
                int32(evidence.FreshRemainder),int32(evidence.StructuralContinuation));
        }
        out=MoveTemp(result);
        return true;
    }

    bool Matches(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source) const
    {
        return snapshot&&Geometry==snapshot->GetGeometrySelection()&&Geometry->HasValidAliases()&&
            SameNativeSolidStaticValues(source,Captured)&&Partition&&Partition->GetScene()==Geometry->GetSelectedUnion()&&
            Product&&Proof&&MatchesCandidatePartitions(snapshot)&&
            Proof->Matches(CurvedBoxArrivalDetail::Motion(source,source.OwnerState,source.Horizon),*Product)&&
            CurvedBoxArrivalDetail::Contains(Partition->GetQueryHull(),Proof->EnvelopeForInspection());
    }
#if !UE_BUILD_SHIPPING
    bool HasLiveCase205Reference(const FPreparedWorldSnapshotPtr& snapshot) const
    {
        using namespace Analytic::Certified;
        FStepIdentity now;
        return Proof&&Proof->HasCase205ReferenceTiming()&&Provider==Case205ReferenceTiming::Provider&&
            Captured.BodyIndex==0&&Captured.FrameClock.OwnerLocalFrame==220&&
            ReadCurrentStep(now)&&(now.Frame==219||now.Frame==6386)&&now.Iteration==2&&now.QuerySerial==2&&
            now.Provider&&now.Provider->GetPreparedSnapshot()==snapshot;
    }
    bool MatchesCase205PhysicalProjection(const FPreparedWorldSnapshotPtr& snapshot,
        const Analytic::FWorldQuery& query,const FProduct& product) const
    {
        using namespace Analytic::Certified;
        return HasLiveCase205Reference(snapshot)&&Product==&product&&
            query.RequiredSourceId==Hit.SourceId&&query.RequiredSurfaceId==Hit.SurfaceId&&
            query.RequiredCanonicalGroupId==Hit.CanonicalGroupId&&
            FloorNativeSweepDetail::SameVector(query.HalfExtent,Captured.PhysicalHalfExtent)&&
            query.TraceChannel==Captured.Query.TraceChannel&&query.BlockingObjectTypes==Captured.Query.BlockingObjectTypes&&
            query.DomainTolerance==Captured.Query.DomainTolerance&&query.Radius==0.&&Matches(snapshot,Captured);
    }
#endif
    bool MatchesLiveRemainderPhysicalProjection(const FPreparedWorldSnapshotPtr& snapshot,
        const Analytic::FWorldQuery& query,const FProduct& product) const
    {
        if(!NativeAdjacentBand.IsSet())return false;
        const auto& evidence=NativeAdjacentBand.GetValue();
        FStepIdentity step;
        return evidence.LiveStep51RemainderSerial&&
            evidence.StructuralContinuation&&Provider==607&&Product==&product&&
            query.RequiredSourceId==Hit.SourceId&&query.RequiredSurfaceId==Hit.SurfaceId&&
            query.RequiredCanonicalGroupId==Hit.CanonicalGroupId&&
            Analytic::Certified::FloorNativeSweepDetail::SameVector(query.HalfExtent,Captured.PhysicalHalfExtent)&&
            query.TraceChannel==Captured.Query.TraceChannel&&query.BlockingObjectTypes==Captured.Query.BlockingObjectTypes&&
            ReadCurrentStep(step)&&step.Frame==50&&step.QuerySerial==evidence.LiveStep51RemainderSerial&&
            step.Provider&&step.Provider->GetPreparedSnapshot()==snapshot&&Matches(snapshot,Captured);
    }
    bool CoversActualPrimitive(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        const FKinematicState& actual,float seconds,Analytic::Certified::FDistanceBudget& budget,
        Analytic::Certified::FIntervalPoint& output,
        Analytic::Certified::FGeneralBoxPhysicalFloorClearBoundsPtr* rebasedFloor=nullptr,
        TOptional<FNativeBoxRebasedRoof202>* rebasedRoof=nullptr) const
    {
        namespace C=Analytic::Certified;
        using namespace CurvedBoxArrivalDetail;
        output={};
        if(rebasedFloor)rebasedFloor->Reset();
        if(rebasedRoof)rebasedRoof->Reset();
        if(!Matches(snapshot,source)||!FMath::IsFinite(seconds)||seconds<=0.f||
            (NativeAdjacentBand.IsSet()&&seconds>NativeAdjacentBand.GetValue().Through))return false;
        // The original global proof does not imply any restarted local DAG.
        const auto input=Motion(source,actual,seconds);
        const bool original=Proof->CoversOriginalPrefix(input,*Product);
        if(!original&&(!rebasedFloor||!EstablishedFloorCompanions||!Proof->HasSeparateNativeReferences()||
            seconds>MaximumAdvanceSeconds()))return false;
        // The partition was selected on a complete original hull. Recheck
        // the actual primitive envelope before carrying that partition over.
        C::FGroupedBoxVolumeBounds volume;
        const auto diagnose=[&](const TCHAR* reason,bool built,bool contained,bool competing,bool landscape)
        {
            if(original||source.FrameClock.OwnerLocalFrame!=127||!Proof->HasSeparateNativeReferences()||
                CVarNativeBoxSplitArrival202RebaseTrace.GetValueOnAnyThread()!=1)return;
            const auto before=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
            const uint64 began=budget.Used;
            int32 floor=-1;
            constexpr int32 floorTraceWork=2048;
            // Run only after the real branch already knows it will refuse.
            // This interval-only proof does not call Integrate/poison.
            if(landscape&&budget.Remaining-budget.ConsumptionFloor>=floorTraceWork&&
                before.Status==Analytic::EAnalyticNumericStatus::Complete)
            {
                C::FScopedDistanceBudgetFloor limit(budget,budget.Remaining-floorTraceWork);
                if(limit.IsValid())
                {
                    try
                    {
                        C::FGeneralBoxPhysicalFloorClearBoundsPtr observedFloor;
                        floor=int32(C::TryProveGeneralBoxPhysicalFloorClear(input,FVector::ZeroVector,FVector::UpVector,
                            budget,observedFloor)&&observedFloor);
                    }
                    catch(...){floor=-2;}
                }
            }
            const auto& p=actual.Location;const auto& q=actual.Rotation;const auto& v=actual.Velocity;
            const auto& a=actual.Acceleration;const auto& w=actual.AngularVelocity;const auto& alpha=actual.AngularAcceleration;
            UE_LOG(LogTemp,Display,TEXT("[NativeBoxSplitArrival202RebaseTrace] Frame=%u Body=%d Reason=%s LocalBits=%08x ClearBits=%08x Numeric=%016llx P=(%.17g,%.17g,%.17g) Q=(%.17g,%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) W=(%.17g,%.17g,%.17g) Alpha=(%.17g,%.17g,%.17g)"),
                source.FrameClock.OwnerLocalFrame,source.BodyIndex,reason,C::KinematicCenterDetail::Bits(seconds),
                C::KinematicCenterDetail::Bits(MaximumAdvanceSeconds()),before.ControlFingerprint(),
                p.X,p.Y,p.Z,q.X,q.Y,q.Z,q.W,v.X,v.Y,v.Z,a.X,a.Y,a.Z,w.X,w.Y,w.Z,alpha.X,alpha.Y,alpha.Z);
            const auto& hull=volume.BroadHull;const auto& originalHull=Partition->GetQueryHull();
            UE_LOG(LogTemp,Display,TEXT("[NativeBoxSplitArrival202RebaseEnvelope] Built=%d OriginalContains=%d CompetingContains=%d LandscapeClear=%d Floor=%d FloorWork=%llu Groups=%d Axes=%d Candidates=%d Excluded=%d Scene=%d Domain=%.17g Overlap=%.17g HullX=[%.17g,%.17g] HullY=[%.17g,%.17g] HullZ=[%.17g,%.17g] OriginalX=[%.17g,%.17g] OriginalY=[%.17g,%.17g] OriginalZ=[%.17g,%.17g]"),
                int32(built),int32(contained),int32(competing),int32(landscape),floor,budget.Used-began,
                volume.Groups.Num(),volume.Axes.Num(),Partition->GetCandidateIndices().Num(),Partition->GetExcludedIndices().Num(),
                Partition->GetScene()->Num(),source.Query.DomainTolerance,source.Query.InitialOverlapTolerance,
                hull.Axis[0].Lower,hull.Axis[0].Upper,hull.Axis[1].Lower,hull.Axis[1].Upper,hull.Axis[2].Lower,hull.Axis[2].Upper,
                originalHull.Axis[0].Lower,originalHull.Axis[0].Upper,originalHull.Axis[1].Lower,originalHull.Axis[1].Upper,
                originalHull.Axis[2].Lower,originalHull.Axis[2].Upper);
            for(int32 candidate:Partition->GetCandidateIndices())
                UE_LOG(LogTemp,Display,TEXT("[NativeBoxSplitArrival202RebaseCandidate] Provider=%d Winner=%d"),
                    candidate,int32(candidate==Provider));
        };
        const bool built=C::BuildGeneralBoxPrimitiveGroupedVolume(input,actual.Rotation*source.LocalRotation,
            source.Query.DomainTolerance,source.Query.InitialOverlapTolerance,budget,volume);
        const bool contained=built&&Contains(Partition->GetQueryHull(),volume.BroadHull);
        const bool competing=contained&&CoversCompetingPrefixEnvelope(volume.BroadHull);
        const bool landscape=competing&&(!snapshot->GetExternalLandscape()||
            snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,volume.BroadHull));
        if(!landscape)
        {
            diagnose(TEXT("envelope"),built,contained,competing,landscape);
            return false;
        }
        if(!original)
        {
            // Every other original retained companion was authenticated as
            // exact Z=0 by the mixed producer. Reprove the physical floor
            // on this actual local interval, and exclude the original curve
            // with the full shell-bearing physical/query primitive envelope.
            if(Product->Curve.ControlPoints.IsEmpty())return false;
            bool left=true;
            for(const auto& control:Product->Curve.ControlPoints)
            {
                if(!budget.Consume()||!C::Detail::ValidPoint(control))return false;
                if(!(volume.BroadHull.Axis[1].Upper<control.Axis[1].Lower)){left=false;break;}
            }
            if(!C::TryProveGeneralBoxPhysicalFloorClear(input,FVector::ZeroVector,FVector::UpVector,
                budget,*rebasedFloor)||!*rebasedFloor)return false;
            if(!left)
            {
                // The winning curve remains in the complete partition.
                // The shell still bounds policy, all other providers and exterior geometry.
                FNativeBoxRebasedRoof202 roof;
                if(!rebasedRoof||!Proof->TryProveRebasedRoof202(input,volume.BroadHull,**rebasedFloor,budget,roof))
                {
                    diagnose(TEXT("rebased_curve"),built,contained,competing,landscape);
                    return false;
                }
                *rebasedRoof=roof;
                UE_LOG(LogTemp,Display,TEXT("[NativeBoxSplitArrival202RebasedRoof] Frame=%u Bulk=%08x RawClear=%08x RawInside=%08x UnitClear=%08x NativeClear=%08x"),
                    source.FrameClock.OwnerLocalFrame,roof.NativeBulkBits,roof.RawClearBits,roof.RawInsideBits,
                    roof.NormalizedClearBits,roof.NativeClearBits);
            }
            else
                UE_LOG(LogTemp,Display,TEXT("[NativeBoxSplitArrival202RebasedLeft] Frame=%u Seconds=%.17g UpperY=%.17g"),
                    source.FrameClock.OwnerLocalFrame,double(seconds),volume.BroadHull.Axis[1].Upper);
        }
        output=volume.BroadHull;
        return true;
    }
    bool MatchesRebasedSplitFloor(const FNativeSolidStaticValues& source,const FKinematicState& actual,
        float seconds,const Analytic::Certified::FIntervalPoint& envelope,
        const Analytic::Certified::FGeneralBoxPhysicalFloorClearBoundsPtr& floor) const
    {
        using namespace CurvedBoxArrivalDetail;
        return EstablishedFloorCompanions&&Proof&&Proof->HasSeparateNativeReferences()&&floor&&
            Contains(Partition->GetQueryHull(),envelope)&&floor->Matches(Motion(source,actual,seconds),
                FVector::ZeroVector,FVector::UpVector);
    }
    bool ContainsActualEndpoint(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        const Analytic::FWorldQuery& pose,const Analytic::Certified::FIntervalPoint& actualEnvelope,
        Analytic::Certified::FDistanceBudget& budget,int32 maximumWork) const
    {
        namespace C=Analytic::Certified;
        const FProduct* case205Product=nullptr;
        const Analytic::Prepared::FReferenceTransport* case205Reference=nullptr;
#if !UE_BUILD_SHIPPING
        const bool trace205=CVarCase205Q112PhysicalProjection.GetValueOnAnyThread()==1&&
            C::Case205ReferenceTiming::Enabled.GetValueOnAnyThread()==1&&HasLiveCase205Reference(snapshot);
#endif
        const auto refuseEndpoint=[&](const TCHAR* stage)
        {
#if !UE_BUILD_SHIPPING
            if(trace205)
            {
                UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCase205Q112EndpointRefused] Frame=220 Serial=2 Stage=%s Work=%llu"),
                    stage,budget.Used);
            }
#endif
            return false;
        };
        if(!Matches(snapshot,source))return refuseEndpoint(TEXT("source"));
        if(!CoversCompetingPrefixEnvelope(actualEnvelope))return refuseEndpoint(TEXT("competitor_envelope"));
        if(!C::FloorNativeSweepDetail::SameVector(pose.Start,pose.End))return refuseEndpoint(TEXT("pose"));
        for(int32 representation=0;representation<2;++representation)
        {
            auto query=pose;
            query.HalfExtent=representation==0?source.PhysicalHalfExtent:source.QueryHalfExtent;
            C::FIntervalPoint volume;
            if(!C::BuildFixedWorldQueryVolumeBounds(query,volume)||
                !CurvedBoxArrivalDetail::Contains(actualEnvelope,volume))
                return refuseEndpoint(representation==0?TEXT("physical_volume"):TEXT("query_volume"));
        }
#if !UE_BUILD_SHIPPING
        if(trace205&&
            Partition->GetCandidateIndices().Num()==1&&Partition->GetCandidateIndices()[0]==Provider&&
            Partition->GetExcludedIndices().Num()+1==Partition->GetScene()->Num())
        {
            case205Product=Product;
            case205Reference=Geometry->GetReference().Get();
        }
#endif
        return Proof->ProvePhysicalEndpoint(pose,source.PhysicalHalfExtent,budget,maximumWork,case205Product,case205Reference)||
            refuseEndpoint(TEXT("physical_sign"));
    }
    bool CoversCorrectionPartition(const Analytic::Certified::FReferenceBoundsCandidateSetPtr& partition,
        const Analytic::Certified::FIntervalPoint& envelope) const
    {
        if(!partition||partition->GetScene()!=Geometry->GetSelectedUnion()||
            !CurvedBoxArrivalDetail::Contains(partition->GetQueryHull(),envelope))return false;
        const auto candidates=partition->GetCandidateIndices();
        if(EstablishedFloorCompanions)
        {
            for(const int32 candidate:candidates)
                if(!Partition->GetCandidateIndices().Contains(candidate))return false;
            return true;
        }
        return candidates.IsEmpty()||(candidates.Num()==1&&candidates[0]==Provider);
    }
    uint64 Step51RemainderCorrectionSerial(const FKinematicState& before,const FKinematicState& candidate) const
    {
        if(!NativeAdjacentBand.IsSet()||!NativeAdjacentBand.GetValue().StructuralContinuation||Provider!=607||
            CurvedBoxArrivalDetail::IsCase976Step51FullSweep(Captured)||
            CVarCurvedBoxNativeAdjacentBandStep51Translation.GetValueOnAnyThread()!=1||
            !FiniteState(before)||!FiniteState(candidate))return 0;
        const uint64 liveSerial=NativeAdjacentBand.GetValue().LiveStep51RemainderSerial;
        if(liveSerial)
        {
            // Only the fourth arrival, with its original live admission.
            if(liveSerial!=8||!CurvedBoxArrivalDetail::IsCase976Step51RemainderCandidate(Captured))return 0;
        }
        else if(!CurvedBoxArrivalDetail::IsCase976Step51ProvedTimingRequest(Captured))return 0;

        // This proposal retreats from the increasing Y/Z chart at fixed X.
        // The proof below still checks the whole physical corridor and the
        // fresh complete original-shell partition against its companion.
        auto expected=before;
        expected.Location.Y=candidate.Location.Y;
        expected.Location.Z=candidate.Location.Z;
        if(!(candidate.Location.Y<before.Location.Y&&candidate.Location.Z>before.Location.Z&&
             SameKinematicBits(expected,candidate)))return 0;
        if(liveSerial)return liveSerial;
        return CurvedBoxArrivalDetail::IsCase976Step51SecondRemainder(Captured)?6u:4u;
    }
    uint64 Step51RemainderRotationSerial(const FKinematicState& before,const FKinematicState& candidate) const
    {
        if(!NativeAdjacentBand.IsSet()||!NativeAdjacentBand.GetValue().StructuralContinuation||Provider!=607||
            CurvedBoxArrivalDetail::IsCase976Step51FullSweep(Captured)||
            CVarCurvedBoxNativeAdjacentBandStep51Rotation.GetValueOnAnyThread()!=1||
            !FiniteState(before)||!FiniteState(candidate)||
            Analytic::Certified::FloorNativeSweepDetail::SameQuat(before.Rotation,candidate.Rotation))return 0;
        const uint64 liveSerial=NativeAdjacentBand.GetValue().LiveStep51RemainderSerial;
        if(liveSerial)
        {
            if(liveSerial!=8||!CurvedBoxArrivalDetail::IsCase976Step51RemainderCandidate(Captured))return 0;
        }
        else if(!CurvedBoxArrivalDetail::IsCase976Step51ProvedTimingRequest(Captured))return 0;

        // The caller still seals this exact same-COM chord and original
        // partition. Every new rotation must prove its own physical corridor.
        auto expected=before;
        expected.Rotation=candidate.Rotation;
        if(!SameKinematicBits(expected,candidate))return 0;
        if(liveSerial)return liveSerial;
        return CurvedBoxArrivalDetail::IsCase976Step51SecondRemainder(Captured)?6u:4u;
    }
    // Only a sealed correction value built by ProveActualCorrection may
    // interpret this exact two-provider partition as a proved corridor.
    // This check alone grants no correction, query or integration authority.
    bool MatchesNativeAdjacentBandCorrectionPartition(
        const Analytic::Certified::FReferenceBoundsCandidateSetPtr& partition,
        const Analytic::Certified::FIntervalPoint& envelope,
        const FKinematicState& before,const FKinematicState& candidate) const
    {
        namespace C=Analytic::Certified;
        if(!NativeAdjacentBand.IsSet()||
            (Captured.FrameClock.OwnerLocalFrame!=50&&
             (!NativeAdjacentBand.GetValue().StructuralContinuation||
              (!CurvedBoxArrivalDetail::IsCase976Step51FullSweep(Captured)&&
               !Step51RemainderCorrectionSerial(before,candidate)&&
               !Step51RemainderRotationSerial(before,candidate))))||
            Provider!=607||!partition||partition==Partition||
            partition->GetScene()!=Geometry->GetSelectedUnion()||
            !C::GroupedBoxDetail::SamePoint(partition->GetQueryHull(),envelope))return false;
        const auto candidates=partition->GetCandidateIndices();
        return candidates.Num()==2&&candidates.Contains(607)&&candidates.Contains(726)&&
            partition->GetExcludedIndices().Num()+candidates.Num()==partition->GetScene()->Num();
    }

    bool ProveActualCorrection(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
        const FKinematicState& before,const FKinematicState& candidate,
        Analytic::Certified::FDistanceBudget& budget,Analytic::Certified::FIntervalPoint& envelope,
        Analytic::Certified::FReferenceBoundsCandidateSetPtr& correctionPartition) const
    {
        namespace C=Analytic::Certified;
        envelope={};
        correctionPartition.Reset();
        const bool rotationChanged=!C::FloorNativeSweepDetail::SameQuat(before.Rotation,candidate.Rotation);
        const bool nativeAdjacentCorrection=NativeAdjacentBand.IsSet()&&source.FrameClock.OwnerLocalFrame==50&&
            CVarCurvedBoxNativeAdjacentBandCorrection.GetValueOnAnyThread()==1;
        const bool nativeAdjacentRotation=rotationChanged&&nativeAdjacentCorrection&&
            NativeAdjacentBand.GetValue().StructuralContinuation&&
            CVarCurvedBoxNativeAdjacentBandRotation.GetValueOnAnyThread()==1;
        const bool nativeStep51=NativeAdjacentBand.IsSet()&&NativeAdjacentBand.GetValue().StructuralContinuation&&
            CurvedBoxArrivalDetail::IsCase976Step51FullSweep(source);
        FStepIdentity rotationStep;
        const uint64 rotationSerial=rotationChanged?Step51RemainderRotationSerial(before,candidate):0;
        const bool nativeStep51RemainderRotation=rotationSerial&&
            ReadCurrentStep(rotationStep)&&rotationStep.Frame==50&&
            rotationStep.QuerySerial==rotationSerial&&
            rotationStep.Provider&&rotationStep.Provider->GetPreparedSnapshot()==snapshot;
        const bool nativeStep51Rotation=rotationChanged&&(nativeStep51||nativeStep51RemainderRotation)&&
            CVarCurvedBoxNativeAdjacentBandStep51Rotation.GetValueOnAnyThread()==1;
        const bool pureRotation=rotationChanged&&(nativeAdjacentRotation||nativeStep51Rotation||
            CVarCurvedBoxPureRotationCorridor.GetValueOnAnyThread()==1);
        auto expected=before;
        if(pureRotation)expected.Rotation=candidate.Rotation;
        else expected.Location=candidate.Location;
        if(!Matches(snapshot,source)||!FiniteState(before)||!FiniteState(candidate)||
            !SameKinematicBits(expected,candidate))return false;

        const auto input=MakeNativeBoxPoseCorrectionCorridorInput(source,before,candidate);
        // The actual setter and its current before pose are joined by the
        // owner transaction. Repartition and prove this new pure translation
        // below on the typed step 51 carrier and original shells.
        FStepIdentity remainderStep;
        const uint64 correctionSerial=Step51RemainderCorrectionSerial(before,candidate);
        const bool nativeStep51Remainder=correctionSerial&&
            ReadCurrentStep(remainderStep)&&remainderStep.Frame==50&&
            remainderStep.QuerySerial==correctionSerial&&
            remainderStep.Provider&&remainderStep.Provider->GetPreparedSnapshot()==snapshot;
        const bool nativeStep51Translation=!rotationChanged&&(nativeStep51||nativeStep51Remainder)&&
            CVarCurvedBoxNativeAdjacentBandStep51Translation.GetValueOnAnyThread()==1;
        const bool tryNativeAdjacentBand=(nativeAdjacentCorrection&&(!pureRotation||nativeAdjacentRotation))||
            nativeStep51Translation||nativeStep51Rotation;
        C::FIntervalPoint hull;
        TArray<C::FIntervalPoint,TInlineAllocator<16>> corners;
        if(!budget.Consume()||!C::BuildBoxPoseCorridorBounds(input,hull,tryNativeAdjacentBand?&corners:nullptr))
            return false;
        // Every other retained provider must be excluded by the original
        // complete partition, or by a fresh partition of this exact hull.
        auto selected=Partition;
        if(!CoversCorrectionPartition(selected,hull)&&
            !C::FReferenceBoundsCandidateSet::Select(Geometry->GetSelectedUnion(),hull,budget,selected))return false;
        double competitorMargin=0.;
        if(!CoversCorrectionPartition(selected,hull))
        {
            namespace A=::AnalyticCertified;
            if(!tryNativeAdjacentBand||!MatchesNativeAdjacentBandCorrectionPartition(selected,hull,before,candidate)||corners.Num()!=16)
                return false;
            C::FIntervalTensorSurface competitor;
            if(!selected->GetScene()->CopySurfaceForQuery(726,budget,competitor)||
                competitor.DegreeU!=5||competitor.DegreeV!=5||competitor.ControlPoints.Num()!=36)return false;
            for(int32 point=0;point<competitor.ControlPoints.Num();++point)if(!budget.Consume())return false;
            for(int32 point=0;point<corners.Num();++point)if(!budget.Consume())return false;
            // Retained corners already cover BOTH extents, raw/normalized
            // variants and both original shells for this exact setter corridor.
            const FVector3d axis(0.,-167./256.,1.);
            const auto gap=A::Subtract(C::GroupedBoxDetail::ProjectHull(corners,axis),
                C::GroupedBoxDetail::ProjectHull(competitor.ControlPoints,axis));
            if(!A::Valid(gap)||!(gap.Lower>0.))return false;
            competitorMargin=gap.Lower;
        }
        if(!Proof->ProvePhysicalCorrectionCorridor(input,budget,pureRotation,
            !EstablishedFloorCompanions&&!CompetingPrefix&&!NativeAdjacentBand.IsSet(),
            nativeStep51Rotation?&hull:nullptr))return false;
        if(competitorMargin>0.)
            UE_LOG(LogTemp,Display,TEXT("[CurvedBoxNativeAdjacentBandCorrection] Frame=%u Winner=607 Competitor=726 Gap=%.17g Used=%llu PureRotation=%d"),
                source.FrameClock.OwnerLocalFrame,competitorMargin,budget.Used,int32(pureRotation));
        envelope=hull;
        correctionPartition=MoveTemp(selected);
        return true;
    }
    bool ProveObstructedWheelSupportTranslation(const FPreparedWorldSnapshotPtr& snapshot,
        const FNativeSolidStaticValues& source,const FKinematicState& before,const FKinematicState& candidate,
        Analytic::Certified::FDistanceBudget& budget,bool case585Owner88,bool& provedTwoBand) const
    {
        namespace C=Analytic::Certified;
        provedTwoBand=false;
        if(EstablishedFloorCompanions||(CompetingPrefix&&!case585Owner88)||NativeAdjacentBand.IsSet()||
            !Matches(snapshot,source))return false;
        if(CompetingPrefix)
        {
            // This is the same complete original typed 726/607 partition.
            // Check the entire actual correction, including both original
            // shells and both extents, within its already sealed prefix.
            if(Provider!=726||ExcludedCompetitor!=607||!budget.Consume())return false;
            const auto owners=Geometry->GetNominalScene()->GetOwners();
            for(const int32 provider:Partition->GetCandidateIndices())
                if(!budget.Consume()||!owners.IsValidIndex(provider)||
                    !C::CanonicalAxialPlaneDetail::Policy(source.Query,owners[provider]))return false;
            C::FIntervalPoint corridor;
            if(!C::BuildBoxPoseCorridorBounds(MakeNativeBoxPoseCorrectionCorridorInput(source,before,candidate),corridor)||
                !CoversCompetingPrefixEnvelope(corridor)||
                (snapshot->GetExternalLandscape()&&!snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,corridor)))
                return false;
        }
        // The original positive physical edge test alone proves obstruction.
        if(!Proof->ProvePhysicalObstructedEndpoint(MakeNativeBoxPoseCorrectionCorridorInput(source,candidate,candidate),budget))
            return false;
        provedTwoBand=CompetingPrefix.IsValid();
        return true;
    }
    bool CoversCompetingPrefixEnvelope(const Analytic::Certified::FIntervalPoint& envelope) const
    {return !CompetingPrefix||CurvedBoxArrivalDetail::Contains(CompetingPrefix->GetQueryHull(),envelope);}
    float MaximumAdvanceSeconds() const {return Proof->ClearSeconds();}
    const auto& GetEnvelopeForInspection() const {return Partition->GetQueryHull();}
    const auto& GetHitForInspection() const {return Hit;}
};
}

namespace Speed::Analytic::Certified
{
namespace YawFloorBoxDetail::NodeErrorUpper
{
std::atomic<bool> Enabled{false};
static void Changed(IConsoleVariable* variable)
{Enabled.store(variable&&variable->GetInt()==1,std::memory_order_relaxed);}
static TAutoConsoleVariable<int32> CVar(TEXT("p.IAmSpeed.AnalyticWorld.NodeErrorUpper"),0,
    TEXT("Default OFF. Same FNode Range/Error upper bounds and refusals with upper-only nonnegative error propagation. Masked sticky history may differ; controls and geometry do not."),
    FConsoleVariableDelegate::CreateStatic(&Changed),ECVF_Default);
struct FRegister
{
    FRegister(){Changed(CVar.AsVariable());}
};
static FRegister Register;
}

namespace GeneralBoxMotionMemo
{
namespace A=::AnalyticCertified;
TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxMotionValueMemo"),0,
    TEXT("Default OFF. Exact numeric-value memo for complete general Box motion models. No query authority or separation cache."));
struct FKey
{
    uint64 Words[37]{};
    bool operator==(const FKey& b)const
    {for(int32 i=0;i<37;++i)if(Words[i]!=b.Words[i])return false;return true;}
};
FKey Key(const FGeneralBoxMotionInput& v,uint64 numeric)
{
    const auto& o=v.Owner;
    const double values[]={o.Location.X,o.Location.Y,o.Location.Z,
        o.Velocity.X,o.Velocity.Y,o.Velocity.Z,o.Acceleration.X,o.Acceleration.Y,o.Acceleration.Z,
        o.Rotation.X,o.Rotation.Y,o.Rotation.Z,o.Rotation.W,
        o.AngularVelocity.X,o.AngularVelocity.Y,o.AngularVelocity.Z,
        o.AngularAcceleration.X,o.AngularAcceleration.Y,o.AngularAcceleration.Z,
        v.LocalCOM.X,v.LocalCOM.Y,v.LocalCOM.Z,v.LocalOffset.X,v.LocalOffset.Y,v.LocalOffset.Z,
        v.LocalRotation.X,v.LocalRotation.Y,v.LocalRotation.Z,v.LocalRotation.W,
        v.PhysicalHalfExtent.X,v.PhysicalHalfExtent.Y,v.PhysicalHalfExtent.Z,
        v.QueryHalfExtent.X,v.QueryHalfExtent.Y,v.QueryHalfExtent.Z};
    static_assert(UE_ARRAY_COUNT(values)==35);FKey key;
    for(int32 i=0;i<35;++i)key.Words[i]=A::Bits(values[i]);
    key.Words[35]=KinematicCenterDetail::Bits(v.Horizon);key.Words[36]=numeric;return key;
}
uint32 Slot(const FKey& key)
{
    uint64 h=1469598103934665603ULL;for(uint64 w:key.Words){h^=w;h*=1099511628211ULL;}
    // Mix high exponent/mantissa bits before the bounded index; structured
    // coordinates often have zero low bytes. Equality still checks every word.
    h^=h>>33;h*=0xff51afd7ed558ccdULL;h^=h>>33;h*=0xc4ceb9fe1a85ec53ULL;h^=h>>33;return uint32(h&15);
}
struct FEntry{FKey Key;FGeneralBoxMotionBounds Value;uint8 Route=2;bool Ready=false;};
struct FCache{FEntry Entries[16];uint64 Hits=0,Misses=0;};
FCache& Cache(){static thread_local FCache cache;return cache;}
struct FRow{std::atomic<uint64> Calls{0},Finished{0},Hits{0},Misses{0},CrossRouteHits{0},Stores{0},False{0},Ticks{0},BuildTicks{0};};
FRow Rows[3];
bool Observe()
{static const IConsoleVariable* flag=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));return flag&&flag->GetInt()!=0;}
struct FScope
{
    FRow& Row;bool Active;uint64 Start=0;bool Result=false;
    explicit FScope(uint8 route):Row(Rows[route<3?route:2]),Active(Observe())
    {if(Active){++Row.Calls;Start=FPlatformTime::Cycles64();}}
    ~FScope(){if(Active){Row.Ticks.fetch_add(FPlatformTime::Cycles64()-Start);if(!Result)++Row.False;++Row.Finished;}}
};
void Publish()
{
    static constexpr const TCHAR* names[]={TEXT("Solid"),TEXT("Primitive"),TEXT("Other")};
    for(int32 i=0;i<3;++i){const auto& x=Rows[i];if(!x.Calls.load())continue;
        UE_LOG(LogTemp,Display,TEXT("[GeneralBoxMotionValueMemo] Scope=ProgramPrefixAtPreExit RequiresDrain=1 NoPASS=1 Route=%s CapacityPerThread=16 EnabledNow=%d Calls=%llu Finished=%llu Hits=%llu Misses=%llu CrossRouteHits=%llu Stores=%llu False=%llu Ticks=%llu MissBuildTicks=%llu SecondsPerCycle=%.17g"),
            names[i],Enabled.GetValueOnAnyThread(),x.Calls.load(),x.Finished.load(),x.Hits.load(),x.Misses.load(),x.CrossRouteHits.load(),x.Stores.load(),x.False.load(),x.Ticks.load(),x.BuildTicks.load(),FPlatformTime::GetSecondsPerCycle64());}
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};FRegister Register;
}
namespace NativeBoxAntipodal
{
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.NativeBoxAntipodal"),0,
 TEXT("Default OFF. Fine14 certified antipodal LOCAL corner offsets; original center arithmetic and consumers remain fresh."));
struct FCounts {std::atomic<uint64> Calls{0},Known{0},False{0},Rotations{0},Reflections{0};};
static FCounts Counts;
void ObserveRange(uint32 rotations,uint32 reflections,bool known)
{
 if(!GeneralBoxMotionMemo::Observe())return;
 ++Counts.Calls;Counts.Rotations.fetch_add(rotations);Counts.Reflections.fetch_add(reflections);
 if(known)++Counts.Known;else ++Counts.False;
}
static void Publish()
{
 if(!Counts.Calls.load())return;
 UE_LOG(LogTemp,Display,TEXT("[NativeBoxAntipodal] Scope=ProgramPrefixAtPreExit NoPASS=1 FineOnly=1 Calls=%llu Known=%llu False=%llu Rotations=%llu Reflections=%llu"),
  Counts.Calls.load(),Counts.Known.load(),Counts.False.load(),Counts.Rotations.load(),Counts.Reflections.load());
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}
namespace GeneralBoxPrologueReuse
{
// Pure numerical values only. One current input, two arithmetic families.
// No scene, query, separation, owner pointer or permission is retained.
TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxPrologueReuse"),0,
    TEXT("Default OFF. Reuse exact quaternion/center prologues from loose to full Box motion, with fresh input and numeric guards."));
struct FFamily {FIntervalPoint Center;YawFloorBoxDetail::FQuaternion Rotation;};
struct FEntry {FFamily Value;uint32 RequiredSticky=0;uint8 Route=2;bool Ready=false;};
struct FCache
{
    GeneralBoxMotionMemo::FKey Key;FEntry Family[2];bool KeyReady=false,Busy=false;
    uint64 Hits=0,Misses=0,Stores=0,StickyMisses=0,CrossRouteHits=0;
};
static FCache& Cache(){static thread_local FCache cache;return cache;}
struct FRow {std::atomic<uint64> Calls{0},Finished{0},Hits{0},Misses{0},Stores{0},StickyMisses{0},CrossRouteHits{0},False{0},Ticks{0},BuildTicks{0};};
static FRow Rows[2];
struct FScope
{
    FRow& Row;bool Active=false,Result=false;uint64 Start=0;
    explicit FScope(uint8 route):Row(Rows[route==1?1:0]),Active(GeneralBoxMotionMemo::Observe())
    {if(Active){++Row.Calls;Start=FPlatformTime::Cycles64();}}
    ~FScope(){if(Active){Row.Ticks.fetch_add(FPlatformTime::Cycles64()-Start);if(!Result)++Row.False;++Row.Finished;}}
};
static void Publish()
{
    static constexpr const TCHAR* names[]={TEXT("General"),TEXT("Loose")};
    for(int32 i=0;i<2;++i){const auto& x=Rows[i];if(!x.Calls.load())continue;
        UE_LOG(LogTemp,Display,TEXT("[GeneralBoxPrologueReuse] Scope=ProgramPrefixAtPreExit RequiresDrain=1 NoPASS=1 Route=%s InputCapacityPerThread=1 FamiliesPerInput=2 EnabledNow=%d Calls=%llu Finished=%llu Hits=%llu Misses=%llu Stores=%llu StickyMisses=%llu CrossRouteHits=%llu False=%llu Ticks=%llu BuildTicks=%llu SecondsPerCycle=%.17g"),
            names[i],Enabled.GetValueOnAnyThread(),x.Calls.load(),x.Finished.load(),x.Hits.load(),x.Misses.load(),x.Stores.load(),x.StickyMisses.load(),x.CrossRouteHits.load(),x.False.load(),x.Ticks.load(),x.BuildTicks.load(),FPlatformTime::GetSecondsPerCycle64());}
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
static bool BuildFamilyOriginal(const FGeneralBoxMotionInput& v,const FZeroOffsetKinematicCenterReference& center,
    bool consumed,FFamily& output)
{
    output={};using namespace GeneralBoxDetail;using namespace YawFloorBoxDetail;
    const auto& owner=v.Owner;
    FQuaternion ownerRotation;if(!RotationFamily(owner,v.Horizon,consumed,ownerRotation))return false;
    FIntervalPoint ownerCenter;
    if(consumed)ownerCenter=BoundConsumedCenterBetweenFloatSeconds(center,0.f,v.Horizon);
    else
    {
        const Interval time{0.,double(v.Horizon),true};const auto squared=Square(time);
        for(int32 d=0;d<3;++d)ownerCenter.Axis[d]=Add(Add(Interval::Exact(owner.Location[d]),
            Multiply(Interval::Exact(owner.Velocity[d]),time)),
            Multiply(Multiply(Interval::Exact(.5),Interval::Exact(owner.Acceleration[d])),squared));
    }
    if(!Detail::ValidPoint(ownerCenter))return false;
    const auto com=PointRanges(Rotate(ownerRotation,v.LocalCOM)),offset=PointRanges(Rotate(ownerRotation,v.LocalOffset));
    if(!Detail::ValidPoint(com)||!Detail::ValidPoint(offset))return false;
    FIntervalPoint bodyCenter;
    for(int32 d=0;d<3;++d)bodyCenter.Axis[d]=Add(Subtract(ownerCenter.Axis[d],com.Axis[d]),offset.Axis[d]);
    if(!Detail::ValidPoint(bodyCenter))return false;
    const auto bodyRotation=Hamilton(ownerRotation,Quaternion(v.LocalRotation));
    output.Center=bodyCenter;output.Rotation=bodyRotation;return true;
}
static bool BuildFamily(const FGeneralBoxMotionInput& v,const FZeroOffsetKinematicCenterReference& center,
    bool consumed,FFamily& output,uint8 route,bool allowReuse=true)
{
    if(!allowReuse||Enabled.GetValueOnAnyThread()!=1)return BuildFamilyOriginal(v,center,consumed,output);
    output={};const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FScope observed(route);auto& cache=Cache();
    // A nested call never replaces storage being used by its caller. Values
    // escape only by copy, so no entry reference can outlive this operation.
    if(cache.Busy){observed.Result=BuildFamilyOriginal(v,center,consumed,output);return observed.Result;}
    struct FBusy {bool& Value;explicit FBusy(bool& value):Value(value){Value=true;}~FBusy(){Value=false;}} busy(cache.Busy);
    const auto key=GeneralBoxMotionMemo::Key(v,before.ControlFingerprint());
    if(!cache.KeyReady||!(cache.Key==key))
    {cache.Family[0].Ready=false;cache.Family[1].Ready=false;cache.Key=key;cache.KeyReady=true;}
    auto& entry=cache.Family[consumed?0:1];
    if(entry.Ready&&(before.VectorControl&entry.RequiredSticky)==entry.RequiredSticky)
    {
        ++cache.Hits;if(observed.Active)++observed.Row.Hits;
        if(entry.Route!=route){++cache.CrossRouteHits;if(observed.Active)++observed.Row.CrossRouteHits;}
        output=entry.Value;observed.Result=true;return true;
    }
    if(entry.Ready){++cache.StickyMisses;if(observed.Active)++observed.Row.StickyMisses;}
    ++cache.Misses;if(observed.Active)++observed.Row.Misses;
    const uint64 begin=observed.Active?FPlatformTime::Cycles64():0;FFamily built;
    const bool known=BuildFamilyOriginal(v,center,consumed,built);
    if(observed.Active)observed.Row.BuildTicks.fetch_add(FPlatformTime::Cycles64()-begin);
    if(!known)return false;
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.ControlFingerprint()!=after.ControlFingerprint())return false;
    entry.Ready=false;entry.Value=built;entry.RequiredSticky=after.VectorControl&uint32(0x3f);entry.Route=route;entry.Ready=true;
    ++cache.Stores;if(observed.Active)++observed.Row.Stores;output=built;observed.Result=true;return true;
}
static bool BuildGeneral(const FGeneralBoxMotionInput& v,FGeneralBoxMotionBounds& output)
{
    if(Enabled.GetValueOnAnyThread()!=1)return BuildGeneralBoxMotionBounds(v,output);
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace GeneralBoxDetail;using namespace YawFloorBoxDetail;
    const auto& owner=v.Owner;
    if(owner.Location.ContainsNaN()||owner.Velocity.ContainsNaN()||owner.Acceleration.ContainsNaN()||
        owner.AngularVelocity.ContainsNaN()||owner.AngularAcceleration.ContainsNaN()||
        owner.Rotation.ContainsNaN()||!owner.Rotation.IsNormalized()||v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||v.PhysicalHalfExtent.ContainsNaN()||v.QueryHalfExtent.ContainsNaN()||
        v.PhysicalHalfExtent.GetMin()<=0||v.QueryHalfExtent.GetMin()<=0)return false;
    FZeroOffsetKinematicCenterReference center;
    if(!BuildZeroOffsetKinematicCenterReference(owner.Location,owner.Velocity,owner.Acceleration,v.Horizon,
        FVector::ZeroVector,FVector::ZeroVector,center))return false;
    FGeneralBoxMotionBounds result;
    for(bool consumed:{true,false})
    {
        FFamily family;if(!BuildFamily(v,center,consumed,family,0))return false;
        const auto& bodyCenter=family.Center;const auto& bodyRotation=family.Rotation;
        if(consumed)result.NativeCenter=bodyCenter;else result.ReferenceCenter=bodyCenter;
        for(const auto& extent:{v.PhysicalHalfExtent,v.QueryHalfExtent})for(int32 corner=0;corner<8;++corner)
        {
            FVector local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
            const auto rotated=PointRanges(Rotate(bodyRotation,local));if(!Detail::ValidPoint(rotated))return false;
            FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=Add(bodyCenter.Axis[d],rotated.Axis[d]);
            if(!YawFloorBoxDetail::Join(result.Hull,point))return false;
            result.Corners.Add(point);
        }
    }
    if(!Detail::ValidPoint(result.Hull))return false;
    output=MoveTemp(result);return true;
}
}
IAMSPEED_API bool BuildGeneralBoxMotionBoundsMemo(const FGeneralBoxMotionInput& input,FGeneralBoxMotionBounds& output,uint8 observerRoute)
{
    using namespace GeneralBoxMotionMemo;
    if(Enabled.GetValueOnAnyThread()!=1)return GeneralBoxPrologueReuse::BuildGeneral(input,output);
    output={};const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    FScope observed(observerRoute);const auto key=Key(input,numeric.ControlFingerprint());
    auto& cache=Cache();auto& entry=cache.Entries[Slot(key)];
    if(entry.Ready&&entry.Key==key)
    {
        ++cache.Hits;if(observed.Active){++observed.Row.Hits;if(entry.Route!=observerRoute)++observed.Row.CrossRouteHits;}
        output=entry.Value;observed.Result=true;return true;
    }
    ++cache.Misses;if(observed.Active)++observed.Row.Misses;
    FGeneralBoxMotionBounds built;const uint64 start=observed.Active?FPlatformTime::Cycles64():0;
    const bool known=GeneralBoxPrologueReuse::BuildGeneral(input,built);
    if(observed.Active)observed.Row.BuildTicks.fetch_add(FPlatformTime::Cycles64()-start);
    if(!known)return false; // Failure is not published or memoized.
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.ControlFingerprint()!=numeric.ControlFingerprint())return false;
    entry.Ready=false;entry.Key=key;entry.Value=built;entry.Route=observerRoute;entry.Ready=true;
    if(observed.Active)++observed.Row.Stores;output=MoveTemp(built);observed.Result=true;return true;
}
using FGeneralBoxMotionMemoApi=bool(*)(const FGeneralBoxMotionInput&,FGeneralBoxMotionBounds&,uint8);
static_assert(std::is_same_v<decltype(&BuildGeneralBoxMotionBoundsMemo),FGeneralBoxMotionMemoApi>);
IAMSPEED_API bool BuildGeneralNativeBoxSweepVolumeMemo(const PreparedExecution::FNativeSolidStaticValues& v,FIntervalPoint& output)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace FloorNativeSweepDetail;
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.Frozen||v.Prescribed||
        v.Query.Shape!=EQueryShape::Box||v.Query.HalfExtent!=v.QueryHalfExtent||
        !SameQuat(v.BodyState.Rotation,v.OwnerState.Rotation*v.LocalRotation)||!SameQuat(v.Query.Rotation,v.BodyState.Rotation)||
        !SameVector(v.Query.Start,v.BodyState.Location))return false;
    const FVector bodyOrigin=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+
        v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    const FVector chordEnd=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    if(!SameVector(bodyOrigin,v.BodyState.Location)||!SameVector(chordEnd,v.Query.End))return false;
    FGeneralBoxMotionInput motion;motion.Owner=v.OwnerState;motion.LocalCOM=v.LocalCOM;motion.LocalOffset=v.LocalOffset;
    motion.LocalRotation=v.LocalRotation;motion.PhysicalHalfExtent=v.PhysicalHalfExtent;motion.QueryHalfExtent=v.QueryHalfExtent;
    motion.Horizon=v.Horizon;FGeneralBoxMotionBounds bounds;
    if(!BuildGeneralBoxMotionBoundsMemo(motion,bounds,0))return false;
    auto hull=bounds.Hull;
    FIntervalPoint chord;if(!BuildFixedWorldQueryVolumeBounds(v.Query,chord)||!YawFloorBoxDetail::Join(hull,chord))return false;
    auto captured=v.Query;captured.Start=captured.End=v.BodyState.Location;captured.HalfExtent=v.PhysicalHalfExtent;
    FIntervalPoint pose;if(!BuildFixedWorldQueryVolumeBounds(captured,pose)||!YawFloorBoxDetail::Join(hull,pose))return false;
    auto footprintQuery=v.Query;footprintQuery.Start=footprintQuery.End=FVector3d::ZeroVector;
    FIntervalPoint footprint;if(!BuildFixedWorldQueryVolumeBounds(footprintQuery,footprint))return false;
    for(const auto& center:{bounds.NativeCenter,bounds.ReferenceCenter})
    {
        FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(center.Axis[d],footprint.Axis[d]);
        if(!YawFloorBoxDetail::Join(hull,point))return false;
    }
    for(double shell:{v.Query.DomainTolerance,v.Query.InitialOverlapTolerance})
    {
        const auto bits=AnalyticCertified::Bits(shell);
        if((bits&AnalyticCertified::Magnitude)>=AnalyticCertified::Infinity||
            ((bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude)))return false;
        for(int32 d=0;d<3;++d)hull.Axis[d]=AnalyticCertified::Add(hull.Axis[d],Interval{-shell,shell,true});
    }
    if(!Detail::ValidPoint(hull))return false;output=hull;return true;
}
using FGeneralNativeBoxMemoApi=bool(*)(const PreparedExecution::FNativeSolidStaticValues&,FIntervalPoint&);
static_assert(std::is_same_v<decltype(&BuildGeneralNativeBoxSweepVolumeMemo),FGeneralNativeBoxMemoApi>);
// R-only draft. Pure polynomial geometry, not a complete native clearance.
// The caller must separately include the actual Hamilton/Normalize/Rotate
// floating-point errors, both center families and every World-query family.
namespace InstantMotionClearanceDetail
{
namespace A=::AnalyticCertified;
using I=A::Interval;
struct FQuaternionDelta { I Axis[4]; };
struct FRotationDeltaMatrix { I Element[3][3]; };

// Exact identity for Unreal's polynomial Rotate, including non-unit q:
// M(q)p = p + 2 qw (qv cross p) + 2 qv cross (qv cross p).
// q=q0+d. Coefficient intervals contain every independently represented d.
static bool RotationDeltaMatrix(const FQuat4d& q0,const FQuaternionDelta& delta,
    FRotationDeltaMatrix& output)
{
    output={};const double raw[4]={q0.X,q0.Y,q0.Z,q0.W};I q[4];
    for(int32 k=0;k<4;++k)
    {q[k]=I::Exact(raw[k]);if(!A::Valid(q[k])||!A::Valid(delta.Axis[k]))return false;}
    const auto two=I::Exact(2.);
    const auto pair=[&](int32 a,int32 b)
    {
        return A::Add(A::Add(A::Multiply(q[a],delta.Axis[b]),
            A::Multiply(q[b],delta.Axis[a])),A::Multiply(delta.Axis[a],delta.Axis[b]));
    };
    FRotationDeltaMatrix result;
    for(int32 i=0;i<3;++i)
    {
        const int32 j=(i+1)%3,k=(i+2)%3;
        const auto sj=A::Multiply(delta.Axis[j],A::Add(A::Multiply(two,q[j]),delta.Axis[j]));
        const auto sk=A::Multiply(delta.Axis[k],A::Add(A::Multiply(two,q[k]),delta.Axis[k]));
        result.Element[i][i]=A::Negate(A::Multiply(two,A::Add(sj,sk)));
        result.Element[i][j]=A::Multiply(two,A::Subtract(pair(i,j),pair(3,k)));
        result.Element[i][k]=A::Multiply(two,A::Add(pair(i,k),pair(3,j)));
        for(int32 d=0;d<3;++d)if(!A::Valid(result.Element[i][d]))return false;
    }
    output=MoveTemp(result);return true;
}

// For every p in the closed local box, this contains M(q)p-M(q0)p.
// No unit-quaternion, zero-yaw, plane or native roundoff assumption is made.
static bool RotationDeltaSupport(const FRotationDeltaMatrix& matrix,
    const FVector3d& halfExtent,FIntervalPoint& output)
{
    output={};I extent[3];
    for(int32 d=0;d<3;++d)
    {extent[d]=I::Exact(halfExtent[d]);if(!A::Valid(extent[d])||halfExtent[d]<0)return false;}
    FIntervalPoint result;
    for(int32 i=0;i<3;++i)
    {
        I radius=I::Exact(0.);
        for(int32 j=0;j<3;++j)
        {
            const auto coefficient=matrix.Element[i][j];if(!A::Valid(coefficient))return false;
            const double magnitude=FMath::Max(FMath::Abs(coefficient.Lower),FMath::Abs(coefficient.Upper));
            radius=A::Add(radius,A::Multiply(I::Exact(magnitude),extent[j]));
        }
        if(!A::Valid(radius)||radius.Upper<0)return false;
        result.Axis[i]={-radius.Upper,radius.Upper,true};
    }
    output=MoveTemp(result);return true;
}
}

// Append within namespace Speed::Analytic::Certified, after the polynomial
// fragment. R-only: this component still requires the complete volume caller.
namespace InstantMotionClearanceDetail
{
namespace Y=YawFloorBoxDetail;
namespace G=GeneralBoxDetail;
struct FStaticNumericalBounds
{
    uint64 Numeric=0;uint32 RequiredSticky=0;bool Ready=false;
    I Scale[6],Cosine[6];
    double Hamilton2Error=0,Dot8Error=0,Normalize8x2Error=0,Rotate2Error[9]{};
};
static constexpr double ThetaSquareBucket[6]={0x1p-20,0x1p-16,0x1p-12,0x1p-8,0x1p-4,1.};
static constexpr double LocalExtentBucket[9]={1.,4.,16.,64.,256.,1024.,4096.,16384.,65536.};
static bool AddUniformError(I value,double error,I& output)
{
    output={};const auto e=I::Exact(error);
    if(!A::Valid(value)||!A::Valid(e)||error<0)return false;
    output=A::Add(value,I{-error,error,true});return A::Valid(output);
}
static bool BoundedMagnitude(I value,double limit)
{return A::Valid(value)&&value.Lower>=-limit&&value.Upper<=limit;}
static bool BuildStaticNumericalBounds(uint64 numeric,FStaticNumericalBounds& output)
{
    output={};FStaticNumericalBounds result;result.Numeric=numeric;
    for(int32 k=0;k<6;++k)
        if(!G::TrigPolynomialBounds(I{0.,ThetaSquareBucket[k],true},result.Scale[k],result.Cosine[k]))return false;
    // These are error-only computations on complete technical boxes. Every
    // future use validates that its ACTUAL operands lie in the same boxes.
    Y::FQuaternion a,b;
    for(int32 k=0;k<4;++k)a.Axis[k]=b.Axis[k]={I{-2.,2.,true},0.};
    const auto h=Y::Hamilton(a,b);if(!Y::ValidQuaternion(h))return false;
    for(int32 k=0;k<4;++k)result.Hamilton2Error=FMath::Max(result.Hamilton2Error,h.Axis[k].Error);
    const Y::FNode raw{I{-8.,8.,true},0.},inverse{I{0.,2.,true},0.};
    Y::FNode squares[4];for(int32 k=0;k<4;++k)squares[k]=Y::Product(raw,raw);
    const auto avx=Y::Sum(Y::Sum(squares[0],squares[1]),Y::Sum(squares[2],squares[3]));
    const auto sse=Y::Sum(Y::Sum(squares[0],squares[2]),Y::Sum(squares[1],squares[3]));
    const auto product=Y::Product(raw,inverse);
    if(!Y::ValidNode(avx)||!Y::ValidNode(sse)||!Y::ValidNode(product))return false;
    result.Dot8Error=FMath::Max(avx.Error,sse.Error);result.Normalize8x2Error=product.Error;
    for(int32 bucket=0;bucket<9;++bucket)
    {
        // Literal Rotate DAG with every coordinate of p independently in
        // [-L,L]. Retain only its uniform rounding/DAZ error, not its value.
        Y::FPoint point,vector;for(int32 d=0;d<3;++d)
        {point.Axis[d]={I{-LocalExtentBucket[bucket],LocalExtentBucket[bucket],true},0.};vector.Axis[d]=a.Axis[d];}
        auto twiceCross=Y::Cross(vector,point);
        for(int32 d=0;d<3;++d)twiceCross.Axis[d]=Y::Product(Y::Leaf(2.),twiceCross.Axis[d]);
        const auto secondCross=Y::Cross(vector,twiceCross);
        for(int32 d=0;d<3;++d)
        {
            const auto value=Y::Sum(Y::Sum(point.Axis[d],Y::Product(a.Axis[3],twiceCross.Axis[d])),secondCross.Axis[d]);
            if(!Y::ValidNode(value))return false;
            result.Rotate2Error[bucket]=FMath::Max(result.Rotate2Error[bucket],value.Error);
        }
    }
    const auto current=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(current.Status!=EAnalyticNumericStatus::Complete||current.ControlFingerprint()!=numeric)return false;
    result.RequiredSticky=current.VectorControl&uint32(0x3f);result.Ready=true;output=MoveTemp(result);return true;
}
static bool GetStaticNumericalBounds(FStaticNumericalBounds& output)
{
    output={};const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.Status!=EAnalyticNumericStatus::Complete)return false;
    static thread_local FStaticNumericalBounds memo;
    if(memo.Ready&&memo.Numeric==before.ControlFingerprint()&&
        (before.VectorControl&memo.RequiredSticky)==memo.RequiredSticky)
    {output=memo;return true;}
    FStaticNumericalBounds built;if(!BuildStaticNumericalBounds(before.ControlFingerprint(),built))return false;
    memo=MoveTemp(built);output=memo;return true;
}
static bool ExactHamilton(const FQuaternionDelta& a,const FQuaternionDelta& b,FQuaternionDelta& output)
{
    output={};for(int32 k=0;k<4;++k)if(!A::Valid(a.Axis[k])||!A::Valid(b.Axis[k]))return false;
    const auto p=[&](int32 x,int32 y){return A::Multiply(a.Axis[x],b.Axis[y]);};
    FQuaternionDelta result;
    result.Axis[0]=A::Subtract(A::Add(A::Add(p(3,0),p(0,3)),p(1,2)),p(2,1));
    result.Axis[1]=A::Add(A::Add(A::Subtract(p(3,1),p(0,2)),p(1,3)),p(2,0));
    result.Axis[2]=A::Add(A::Subtract(A::Add(p(3,2),p(0,1)),p(1,0)),p(2,3));
    result.Axis[3]=A::Subtract(A::Subtract(A::Subtract(p(3,3),p(0,0)),p(1,1)),p(2,2));
    for(int32 k=0;k<4;++k)if(!A::Valid(result.Axis[k]))return false;
    output=MoveTemp(result);return true;
}
static I QuaternionNormSquared(const FQuaternionDelta& q)
{
    I result=I::Exact(0.);for(int32 k=0;k<4;++k)result=A::Add(result,A::Square(q.Axis[k]));
    if(!A::Valid(result))return {};result.Lower=FMath::Max(0.,result.Lower);return result;
}

// Every branch returned by the current RotationFamily lies in q0+output:
// consumed native normalized, unnormalized/reference, and Theta.IsZero.
// No unit-length identity for q0 is assumed. Technical failure is only NA.
static bool BoundOwnerQuaternionDelta(const FKinematicState& state,float horizon,
    const FStaticNumericalBounds& constants,FQuaternionDelta& output)
{
    output={};
#if !defined(_MSC_VER) || !defined(_M_X64) || defined(_M_ARM64EC) || !PLATFORM_ENABLE_VECTORINTRINSICS
    return false;
#else
    const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!constants.Ready||before.Status!=EAnalyticNumericStatus::Complete||constants.Numeric!=before.ControlFingerprint()||
        !A::Valid(I::Exact(double(horizon)))||horizon<0||state.Rotation.ContainsNaN()||
        state.AngularVelocity.ContainsNaN()||state.AngularAcceleration.ContainsNaN())return false;
    FQuaternionDelta q0;const double captured[4]={state.Rotation.X,state.Rotation.Y,state.Rotation.Z,state.Rotation.W};
    for(int32 k=0;k<4;++k)
    {q0.Axis[k]=I::Exact(captured[k]);if(!BoundedMagnitude(q0.Axis[k],2.))return false;}
    const I time{0.,double(horizon),true};const auto referenceSquared=A::Square(time);
    const auto consumedSquared=KinematicCenterDetail::RoundedBinary32(referenceSquared);
    const auto timeSquared=G::Hull(referenceSquared,consumedSquared);if(!A::Valid(timeSquared))return false;
    Y::FNode theta[3],squares[3];
    for(int32 d=0;d<3;++d)
    {
        const auto value=Y::Sum(Y::Product(Y::Leaf(state.AngularVelocity[d]),Y::FNode{time,0.}),
            Y::Product(Y::Product(Y::Leaf(.5),Y::Leaf(state.AngularAcceleration[d])),Y::FNode{timeSquared,0.}));
        const auto expanded=G::Expanded(value);if(!A::Valid(expanded))return false;
        theta[d]={expanded,0.};squares[d]=Y::Product(theta[d],theta[d]);
    }
    I angleSquared;
    for(int32 first=0;first<3;++first)
    {
        const auto total=G::Expanded(Y::Sum(Y::Sum(squares[first],squares[(first+1)%3]),squares[(first+2)%3]));
        if(!A::Valid(total))return false;angleSquared=A::Valid(angleSquared)?G::Hull(angleSquared,total):total;
    }
    if(!A::Valid(angleSquared))return false;angleSquared.Lower=FMath::Max(0.,angleSquared.Lower);
    int32 bucket=0;while(bucket<6&&angleSquared.Upper>ThetaSquareBucket[bucket])++bucket;
    if(bucket==6)return false;
    FQuaternionDelta delta,deltaMinusIdentity;
    for(int32 d=0;d<3;++d)delta.Axis[d]=G::Expanded(Y::Product(theta[d],Y::FNode{constants.Scale[bucket],0.}));
    delta.Axis[3]=constants.Cosine[bucket];
    for(int32 k=0;k<4;++k)
    {
        if(!BoundedMagnitude(delta.Axis[k],2.))return false;
        deltaMinusIdentity.Axis[k]=k==3?A::Subtract(delta.Axis[k],I::Exact(1.)):delta.Axis[k];
    }
    FQuaternionDelta exactRawDifference,rawDifference;
    if(!ExactHamilton(deltaMinusIdentity,q0,exactRawDifference))return false;
    for(int32 k=0;k<4;++k)
    {
        if(!AddUniformError(exactRawDifference.Axis[k],constants.Hamilton2Error,rawDifference.Axis[k])||
            !BoundedMagnitude(A::Add(q0.Axis[k],rawDifference.Axis[k]),8.))return false;
    }
    // Norm multiplication preserves the delta/q0 correlation that would be
    // lost by a direct sum of squares of a four-dimensional raw-value box.
    const auto idealSquared=A::Multiply(QuaternionNormSquared(delta),QuaternionNormSquared(q0));
    if(!A::Valid(idealSquared)||idealSquared.Upper>16.)return false;
    const auto errorNorm=A::Multiply(I::Exact(2.),I::Exact(constants.Hamilton2Error));
    const auto normSquaredError=A::Add(A::Multiply(I::Exact(8.),errorNorm),A::Square(errorNorm));
    const auto allSquaredError=A::Add(normSquaredError,I::Exact(constants.Dot8Error));
    if(!A::Valid(errorNorm)||errorNorm.Upper>4.||!A::Valid(allSquaredError))return false;
    I computedSquared;if(!AddUniformError(idealSquared,allSquaredError.Upper,computedSquared)||
        computedSquared.Lower<=double(UE_SMALL_NUMBER))return false;
    const auto root=[](double value){return _mm_cvtsd_f64(_mm_sqrt_sd(_mm_setzero_pd(),_mm_set_sd(value)));};
    double rootLower=0,rootUpper=0;
    if(!A::Outward(root(computedSquared.Lower),false,rootLower)||
        !A::Outward(root(computedSquared.Upper),true,rootUpper)||rootLower<=0)return false;
    volatile double inverseLower=1./rootUpper,inverseUpper=1./rootLower;
    I inverse;inverse.Known=A::Outward(inverseLower,false,inverse.Lower)&&A::Outward(inverseUpper,true,inverse.Upper);
    if(!A::Valid(inverse)||inverse.Lower<0||inverse.Upper>2.)return false;
    const auto inverseDifference=A::Subtract(inverse,I::Exact(1.));FQuaternionDelta result;
    for(int32 k=0;k<4;++k)
    {
        // E(raw component) was already included in rawDifference. The cached
        // multiply error applies to ACTUAL raw (norm<=8) times native inverse.
        const auto exactNormalizedDifference=A::Add(A::Multiply(rawDifference.Axis[k],inverse),
            A::Multiply(q0.Axis[k],inverseDifference));
        I normalizedDifference;
        if(!AddUniformError(exactNormalizedDifference,constants.Normalize8x2Error,normalizedDifference))return false;
        result.Axis[k]=G::Hull(G::Hull(rawDifference.Axis[k],normalizedDifference),I::Exact(0.));
        if(!BoundedMagnitude(A::Add(q0.Axis[k],result.Axis[k]),2.))return false;
    }
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return false;
    output=MoveTemp(result);return true;
#endif
}
}

// Pure geometry helper, intentionally NOT routed into any physical producer.
// Append to NativeCanonicalBoxSweepClearanceValues.inl in a future coherent
// candidate. No public header, source admission or collision authority here.
namespace InstantDirectionalSupportDetail
{
namespace A=::AnalyticCertified;
namespace Y=YawFloorBoxDetail;
using I=::AnalyticCertified::Interval;
using FMatrix=InstantMotionClearanceDetail::FRotationDeltaMatrix;

// Describes one family p = native_add(c, native_rotate(M,x)), |x_j|<=e_j.
// The caller supplies a certified enclosure M of the EXACT rotation polynomial
// and an absolute per-coordinate error of its real native Rotate DAG. It must
// not supply a nominal quaternion or an empirical epsilon. Center contains the
// actual center values, including their own already-paid native errors.
//
// This value certifies only that mathematical/native family. It is not a scene
// proof, an authenticated current-motion value, or an FGroupedBoxVolumeBounds.
// All fields are deeply owned. A successful factory has no mutable accessor.
class FPreparedFamily final
{
    FMatrix Matrix;
    FIntervalPoint Center;
    I Extent[3];
    double TotalCoordinateError[3]={};
    uint64 Numeric=0;
    bool Ready=false;
public:
    FPreparedFamily()=default;
    static constexpr int32 PrepareWork=5; // entry, three coordinates, publish
    static constexpr int32 ProjectWork=8; // entry, three columns, three rows, publish
    bool IsPrepared() const {return Ready;}
    static bool TryPrepare(const FMatrix& matrix,const FVector3d& halfExtent,
        const FIntervalPoint& center,const FVector3d& rotateError,
        FDistanceBudget& budget,FPreparedFamily& output)
    {
        output={};
        const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(before.Status!=EAnalyticNumericStatus::Complete||!budget.Consume())return false;
        FPreparedFamily result;result.Matrix=matrix;result.Center=center;result.Numeric=before.ControlFingerprint();
        for(int32 d=0;d<3;++d)
        {
            result.Extent[d]=I::Exact(halfExtent[d]);
            const uint64 extentBits=A::Bits(halfExtent[d]);
            if(!A::Valid(result.Extent[d])||((extentBits&A::Sign)&&(extentBits&A::Magnitude))||!A::Valid(center.Axis[d])||
                !A::Valid(I::Exact(rotateError[d]))||(A::Bits(rotateError[d])&A::Sign))return false;
            for(int32 j=0;j<3;++j)if(!A::Valid(matrix.Element[d][j]))return false;
        }
        for(int32 d=0;d<3;++d)
        {
            if(!budget.Consume())return false;
            // This XYZ range is used only to bound the actual final Add's
            // rounding/DAZ error. Directional support below never projects it.
            I radius=I::Exact(0.);
            for(int32 j=0;j<3;++j)
            {
                const auto coefficient=matrix.Element[d][j];
                const auto magnitude=I::Exact(FMath::Max(FMath::Abs(coefficient.Lower),FMath::Abs(coefficient.Upper)));
                radius=A::Add(radius,A::Multiply(magnitude,result.Extent[j]));
            }
            if(!A::Valid(radius)||radius.Upper<0)return false;
            const double errorMagnitude=I::Exact(rotateError[d]).Upper;
            const auto actualRotated=A::Add(I{-radius.Upper,radius.Upper,true},I{-errorMagnitude,errorMagnitude,true});
            if(!A::Valid(actualRotated))return false;
            const auto nativeAdd=Y::Sum(Y::FNode{center.Axis[d],0.},Y::FNode{actualRotated,0.});
            if(!Y::ValidNode(nativeAdd))return false;
            const auto error=A::Add(I::Exact(rotateError[d]),I::Exact(nativeAdd.Error));
            if(!A::Valid(error)||error.Upper<0)return false;
            result.TotalCoordinateError[d]=error.Upper;
        }
        if(!budget.Consume())return false;
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=result.Numeric)return false;
        result.Ready=true;output=MoveTemp(result);return true;
    }
    bool TryProject(const FVector3d& direction,FDistanceBudget& budget,I& output) const
    {
        output={};
        const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Ready||before.Status!=EAnalyticNumericStatus::Complete||before.ControlFingerprint()!=Numeric||!budget.Consume())return false;
        I axis[3];for(int32 d=0;d<3;++d)
        {axis[d]=I::Exact(direction[d]);if(!A::Valid(axis[d]))return false;}
        I radius=I::Exact(0.);
        for(int32 j=0;j<3;++j)
        {
            if(!budget.Consume())return false;
            I coefficient=I::Exact(0.);
            for(int32 d=0;d<3;++d)coefficient=A::Add(coefficient,A::Multiply(axis[d],Matrix.Element[d][j]));
            if(!A::Valid(coefficient))return false;
            const auto magnitude=I::Exact(FMath::Max(FMath::Abs(coefficient.Lower),FMath::Abs(coefficient.Upper)));
            radius=A::Add(radius,A::Multiply(magnitude,Extent[j]));
        }
        I projectedCenter=I::Exact(0.),error=I::Exact(0.);
        for(int32 d=0;d<3;++d)
        {
            if(!budget.Consume())return false;
            projectedCenter=A::Add(projectedCenter,A::Multiply(axis[d],Center.Axis[d]));
            error=A::Add(error,A::Multiply(I::Exact(FMath::Abs(direction[d])),I::Exact(TotalCoordinateError[d])));
        }
        if(!A::Valid(radius)||radius.Upper<0||!A::Valid(projectedCenter)||!A::Valid(error)||error.Upper<0)return false;
        const auto result=A::Add(A::Add(projectedCenter,I{-radius.Upper,radius.Upper,true}),I{-error.Upper,error.Upper,true});
        if(!A::Valid(result)||!budget.Consume())return false;
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=Numeric)return false;
        output=result;return true;
    }
};
}

namespace InstantDirectionalSupportDetail
{
// Only the successful current Instant builder can supply these seven complete
// families. This private owned value is consumed synchronously by the coarse
// proof; it is never a World query, token, endpoint or authority cache.
class FCompletePacket final
{
    FPreparedFamily Families[7];double Shells[2]{};uint64 Numeric=0;bool Ready=false;
public:
    static bool TryBuild(const FMatrix& fixed,const FMatrix& difference,const FIntervalPoint& center,
        const Private::FBoxSweepContext& sat,const FVector3d& physicalExtent,const FVector3d& queryExtent,
        double rotateError,const FIntervalPoint& chordCenter,const FIntervalPoint& poseCenter,
        double domainShell,double overlapShell,FDistanceBudget& budget,FCompletePacket& output)
    {
        output={};const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(before.Status!=EAnalyticNumericStatus::Complete||!budget.Consume())return false;
        FMatrix motion,satMatrix;FVector3d maxExtent,satError;
        for(int32 d=0;d<3;++d)
        {
            maxExtent[d]=FMath::Max(physicalExtent[d],queryExtent[d]);
            for(int32 j=0;j<3;++j)
            {motion.Element[d][j]=A::Add(fixed.Element[d][j],difference.Element[d][j]);satMatrix.Element[d][j]=I::Exact(sat.Axis[j][d]);
             if(!A::Valid(motion.Element[d][j])||!A::Valid(satMatrix.Element[d][j]))return false;}
        }
        // The fixed SAT representation is a distinct original obligation.
        // Bound its literal zero-start Multiply/Add DAG uniformly on the box.
        for(int32 d=0;d<3;++d)
        {
            if(!budget.Consume())return false;Y::FNode value=Y::Leaf(0.);
            for(int32 j=0;j<3;++j)value=Y::Sum(value,Y::Product(Y::FNode{satMatrix.Element[d][j],0.},Y::FNode{I{-maxExtent[j],maxExtent[j],true},0.}));
            if(!Y::ValidNode(value))return false;satError[d]=value.Error;
        }
        FCompletePacket result;const FVector3d rotate(rotateError,rotateError,rotateError);
        if(!FPreparedFamily::TryPrepare(motion,maxExtent,center,rotate,budget,result.Families[0])||
            !FPreparedFamily::TryPrepare(fixed,queryExtent,center,rotate,budget,result.Families[1])||
            !FPreparedFamily::TryPrepare(satMatrix,queryExtent,center,satError,budget,result.Families[2])||
            !FPreparedFamily::TryPrepare(fixed,queryExtent,chordCenter,rotate,budget,result.Families[3])||
            !FPreparedFamily::TryPrepare(satMatrix,queryExtent,chordCenter,satError,budget,result.Families[4])||
            !FPreparedFamily::TryPrepare(fixed,physicalExtent,poseCenter,rotate,budget,result.Families[5])||
            !FPreparedFamily::TryPrepare(satMatrix,physicalExtent,poseCenter,satError,budget,result.Families[6]))return false;
        if(!budget.Consume())return false;result.Shells[0]=domainShell;result.Shells[1]=overlapShell;
        for(double shell:result.Shells)
        {const uint64 b=A::Bits(shell);if((b&A::Magnitude)>=A::Infinity||((b&A::Sign)&&(b&A::Magnitude)))return false;}
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return false;
        result.Numeric=after.ControlFingerprint();result.Ready=true;output=MoveTemp(result);return true;
    }
    bool TryProject(const FVector3d& axis,FDistanceBudget& budget,I& output)const
    {
        output={};const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!Ready||before.Status!=EAnalyticNumericStatus::Complete||before.ControlFingerprint()!=Numeric)return false;
        I bound;
        for(const auto& family:Families)
        {
            I next;if(!family.TryProject(axis,budget,next))return false;
            if(!bound.Known)bound=next;else{bound.Lower=FMath::Min(bound.Lower,next.Lower);bound.Upper=FMath::Max(bound.Upper,next.Upper);}
        }
        if(!budget.Consume())return false;I l1=I::Exact(0.);
        for(int32 d=0;d<3;++d)l1=A::Add(l1,I::Exact(FMath::Abs(axis[d])));
        if(!A::Valid(l1))return false;
        for(double shell:Shells)
        {
            if(!budget.Consume())return false;const auto radius=A::Multiply(I::Exact(shell),l1);
            if(!A::Valid(radius)||radius.Upper<0)return false;
            bound=A::Add(bound,I{-radius.Upper,radius.Upper,true});
        }
        if(!A::Valid(bound)||!budget.Consume())return false;
        const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=Numeric)return false;
        output=bound;return true;
    }
};
static_assert(sizeof(FCompletePacket)<=4096,"Bounded operation-local complete directional packet");
}

// Append after the two previous fragments, inside Certified namespace.
// No contact, support, TOI or frame authority is produced by this AABB builder.
namespace InstantMotionClearanceDetail
{
static bool LocalError(const FStaticNumericalBounds& constants,const FVector3d& absolute,
    double& error)
{
    error=0.;double largest=0.;
    for(int32 d=0;d<3;++d)
    {if(!A::Valid(I::Exact(absolute[d]))||absolute[d]<0)return false;largest=FMath::Max(largest,absolute[d]);}
    int32 bucket=0;while(bucket<9&&largest>LocalExtentBucket[bucket])++bucket;
    if(bucket==9)return false;error=constants.Rotate2Error[bucket];return A::Valid(I::Exact(error))&&error>=0;
}
static bool AddError(FIntervalPoint& point,double error)
{
    for(int32 d=0;d<3;++d)
    {I value;if(!AddUniformError(point.Axis[d],error,value))return false;point.Axis[d]=value;}
    return true;
}
static bool FixedRotationMatrix(const FQuat4d& q,FRotationDeltaMatrix& output)
{
    output={};const double raw[4]={q.X,q.Y,q.Z,q.W};I v[4];
    for(int32 k=0;k<4;++k){v[k]=I::Exact(raw[k]);if(!BoundedMagnitude(v[k],2.))return false;}
    FRotationDeltaMatrix result;const auto two=I::Exact(2.);
    for(int32 i=0;i<3;++i)
    {
        const int32 j=(i+1)%3,k=(i+2)%3;
        result.Element[i][i]=A::Subtract(I::Exact(1.),A::Multiply(two,A::Add(A::Square(v[j]),A::Square(v[k]))));
        result.Element[i][j]=A::Multiply(two,A::Subtract(A::Multiply(v[i],v[j]),A::Multiply(v[3],v[k])));
        result.Element[i][k]=A::Multiply(two,A::Add(A::Multiply(v[i],v[k]),A::Multiply(v[3],v[j])));
        for(int32 d=0;d<3;++d)if(!A::Valid(result.Element[i][d]))return false;
    }
    output=MoveTemp(result);return true;
}
static bool FixedFootprint(const FRotationDeltaMatrix& rotation,const Private::FBoxSweepContext& sat,
    const FVector3d& extent,double rotateError,FIntervalPoint& output)
{
    output={};FIntervalPoint rotated;
    if(!RotationDeltaSupport(rotation,extent,rotated)||!AddError(rotated,rotateError))return false;
    FIntervalPoint axes;
    for(int32 d=0;d<3;++d)
    {
        I radius=I::Exact(0.);
        for(int32 k=0;k<3;++k)
        {
            const auto axis=I::Exact(sat.Axis[k][d]);if(!A::Valid(axis))return false;
            radius=A::Add(radius,A::Multiply(I::Exact(FMath::Abs(sat.Axis[k][d])),I::Exact(extent[k])));
        }
        if(!A::Valid(radius)||radius.Upper<0)return false;axes.Axis[d]={-radius.Upper,radius.Upper,true};
    }
    output=TimeDetail::Hull(rotated,axes);return Detail::ValidPoint(output);
}
static bool TranslateFootprint(const FIntervalPoint& center,const FIntervalPoint& footprint,FIntervalPoint& output)
{
    output={};for(int32 d=0;d<3;++d)output.Axis[d]=A::Add(center.Axis[d],footprint.Axis[d]);
    return Detail::ValidPoint(output);
}
static bool BuildCompleteVolume(const PreparedExecution::FNativeSolidStaticValues& values,FIntervalPoint& output,
    InstantDirectionalSupportDetail::FCompletePacket* directional=nullptr,FDistanceBudget* supportBudget=nullptr)
{
    output={};if(directional)*directional={};if(directional&&!supportBudget)return false;const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(before.Status!=EAnalyticNumericStatus::Complete)return false;
    using namespace FloorNativeSweepDetail;const auto& v=values;
    // These are the original fresh capture/shape guards. An old body pose or
    // a different World chord cannot be made admissible by the new bounds.
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.Frozen||v.Prescribed||
        v.Query.Shape!=EQueryShape::Box||v.Query.HalfExtent!=v.QueryHalfExtent||
        v.OwnerState.Location.ContainsNaN()||v.OwnerState.Velocity.ContainsNaN()||v.OwnerState.Acceleration.ContainsNaN()||
        v.OwnerState.Rotation.ContainsNaN()||!v.OwnerState.Rotation.IsNormalized()||
        v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.BodyState.Rotation.ContainsNaN()||!v.BodyState.Rotation.IsNormalized()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||
        v.PhysicalHalfExtent.ContainsNaN()||v.PhysicalHalfExtent.GetMin()<=0||
        v.QueryHalfExtent.ContainsNaN()||v.QueryHalfExtent.GetMin()<=0||
        !SameQuat(v.BodyState.Rotation,v.OwnerState.Rotation*v.LocalRotation)||
        !SameQuat(v.Query.Rotation,v.BodyState.Rotation)||!SameVector(v.Query.Start,v.BodyState.Location))return false;
    const FVector bodyOrigin=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+
        v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    const FVector chordEnd=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    if(bodyOrigin.ContainsNaN()||chordEnd.ContainsNaN()||
        !SameVector(bodyOrigin,v.BodyState.Location)||!SameVector(chordEnd,v.Query.End))return false;
    FZeroOffsetKinematicCenterReference ownerReference;
    if(!BuildZeroOffsetKinematicCenterReference(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,
        v.Horizon,FVector::ZeroVector,FVector::ZeroVector,ownerReference))return false;
    FStaticNumericalBounds constants;if(!GetStaticNumericalBounds(constants))return false;
    FQuaternionDelta ownerDelta;if(!BoundOwnerQuaternionDelta(v.OwnerState,v.Horizon,constants,ownerDelta))return false;
    FRotationDeltaMatrix ownerMatrix;if(!RotationDeltaMatrix(v.OwnerState.Rotation,ownerDelta,ownerMatrix))return false;
    FQuaternionDelta local,bodyDelta;const double localRaw[4]={v.LocalRotation.X,v.LocalRotation.Y,v.LocalRotation.Z,v.LocalRotation.W};
    const double bodyRaw[4]={v.BodyState.Rotation.X,v.BodyState.Rotation.Y,v.BodyState.Rotation.Z,v.BodyState.Rotation.W};
    for(int32 k=0;k<4;++k){local.Axis[k]=I::Exact(localRaw[k]);if(!BoundedMagnitude(local.Axis[k],2.))return false;}
    if(!ExactHamilton(ownerDelta,local,bodyDelta))return false;
    const auto bodyHamiltonError=A::Multiply(I::Exact(2.),I::Exact(constants.Hamilton2Error));
    if(!A::Valid(bodyHamiltonError))return false;
    for(int32 k=0;k<4;++k)
    {
        I expanded;if(!AddUniformError(bodyDelta.Axis[k],bodyHamiltonError.Upper,expanded))return false;
        bodyDelta.Axis[k]=expanded;
        if(!BoundedMagnitude(A::Add(I::Exact(bodyRaw[k]),bodyDelta.Axis[k]),2.))return false;
    }
    FRotationDeltaMatrix bodyMatrix,fixedMatrix;
    if(!RotationDeltaMatrix(v.BodyState.Rotation,bodyDelta,bodyMatrix)||
        !FixedRotationMatrix(v.BodyState.Rotation,fixedMatrix))return false;
    FVector3d extent,absoluteCOM,absoluteOffset;
    for(int32 d=0;d<3;++d)
    {
        extent[d]=FMath::Max(v.PhysicalHalfExtent[d],v.QueryHalfExtent[d]);
        absoluteCOM[d]=FMath::Abs(v.LocalCOM[d]);absoluteOffset[d]=FMath::Abs(v.LocalOffset[d]);
    }
    double cornerError=0,comError=0,offsetError=0;
    if(!LocalError(constants,extent,cornerError)||!LocalError(constants,absoluteCOM,comError)||
        !LocalError(constants,absoluteOffset,offsetError))return false;
    FIntervalPoint comDisplacement,offsetDisplacement,cornerDisplacement;
    if(!RotationDeltaSupport(ownerMatrix,absoluteCOM,comDisplacement)||
        !RotationDeltaSupport(ownerMatrix,absoluteOffset,offsetDisplacement)||
        !RotationDeltaSupport(bodyMatrix,extent,cornerDisplacement))return false;
    if(!AddError(comDisplacement,comError)||!AddError(offsetDisplacement,offsetError)||
        !AddError(cornerDisplacement,cornerError))return false;
    const auto com0=FixedQueryBoundsDetail::Rotate(v.OwnerState.Rotation,v.LocalCOM);
    const auto offset0=FixedQueryBoundsDetail::Rotate(v.OwnerState.Rotation,v.LocalOffset);
    if(!Detail::ValidPoint(com0)||!Detail::ValidPoint(offset0))return false;
    const auto nativeOwner=BoundConsumedCenterBetweenFloatSeconds(ownerReference,0.f,v.Horizon);
    FIntervalPoint referenceOwner;const I time{0.,double(v.Horizon),true};const auto squared=A::Square(time);
    for(int32 d=0;d<3;++d)referenceOwner.Axis[d]=A::Add(A::Add(I::Exact(v.OwnerState.Location[d]),
        A::Multiply(I::Exact(v.OwnerState.Velocity[d]),time)),
        A::Multiply(A::Multiply(I::Exact(.5),I::Exact(v.OwnerState.Acceleration[d])),squared));
    if(!Detail::ValidPoint(nativeOwner)||!Detail::ValidPoint(referenceOwner))return false;
    const auto ownerCenter=TimeDetail::Hull(nativeOwner,referenceOwner);FIntervalPoint bodyCenter;
    for(int32 d=0;d<3;++d)
    {
        const auto futureCOM=A::Add(com0.Axis[d],comDisplacement.Axis[d]);
        const auto futureOffset=A::Add(offset0.Axis[d],offsetDisplacement.Axis[d]);
        // Preserve the literal native association. A bound on an exact
        // reassociated sum alone would not bound its differently rounded DAG.
        bodyCenter.Axis[d]=A::Add(A::Subtract(ownerCenter.Axis[d],futureCOM),futureOffset);
    }
    if(!Detail::ValidPoint(bodyCenter))return false;
    Private::FBoxSweepContext sat;sat.Initialize(v.Query);
    FIntervalPoint wholeFootprint,queryFootprint,physicalFootprint;
    if(!FixedFootprint(fixedMatrix,sat,extent,cornerError,wholeFootprint)||
        !FixedFootprint(fixedMatrix,sat,v.QueryHalfExtent,cornerError,queryFootprint)||
        !FixedFootprint(fixedMatrix,sat,v.PhysicalHalfExtent,cornerError,physicalFootprint))return false;
    FIntervalPoint hull;
    for(int32 d=0;d<3;++d)
    {
        const auto futureRotated=A::Add(wholeFootprint.Axis[d],cornerDisplacement.Axis[d]);
        hull.Axis[d]=A::Add(bodyCenter.Axis[d],futureRotated);
    }
    if(!Detail::ValidPoint(hull))return false;
    // Literal World chord, captured physical pose and the fixed SAT/Rotate
    // footprint translated by BOTH center families remain in the union.
    FIntervalPoint chordCenter,poseCenter=Detail::Point(v.BodyState.Location),chord,pose,footprintMotion;
    for(int32 d=0;d<3;++d)chordCenter.Axis[d]=A::Add(I::Exact(v.Query.Start[d]),
        A::Multiply(A::Subtract(I::Exact(v.Query.End[d]),I::Exact(v.Query.Start[d])),I{0.,1.,true}));
    if(!TranslateFootprint(chordCenter,queryFootprint,chord)||!TranslateFootprint(poseCenter,physicalFootprint,pose)||
        !TranslateFootprint(bodyCenter,queryFootprint,footprintMotion)||
        !YawFloorBoxDetail::Join(hull,chord)||!YawFloorBoxDetail::Join(hull,pose)||!YawFloorBoxDetail::Join(hull,footprintMotion))return false;
    for(double shell:{v.Query.DomainTolerance,v.Query.InitialOverlapTolerance})
    {
        const uint64 bits=A::Bits(shell);
        if((bits&A::Magnitude)>=A::Infinity||((bits&A::Sign)&&(bits&A::Magnitude)))return false;
        for(int32 d=0;d<3;++d)hull.Axis[d]=A::Add(hull.Axis[d],I{-shell,shell,true});
    }
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!Detail::ValidPoint(hull)||after.Status!=EAnalyticNumericStatus::Complete||
        after.ControlFingerprint()!=before.ControlFingerprint())return false;
    if(directional&&!InstantDirectionalSupportDetail::FCompletePacket::TryBuild(fixedMatrix,bodyMatrix,bodyCenter,sat,
        v.PhysicalHalfExtent,v.QueryHalfExtent,cornerError,chordCenter,poseCenter,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,
        *supportBudget,*directional))return false;
    output=MoveTemp(hull);return true;
}
}

namespace InstantMotionClearanceDetail
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.InstantMotionClearance"),0,
    TEXT("Default OFF. Bound complete short native Box motion from certified quaternion differences and exact directional support before the full prologue. Clear-only callers remain authoritative."));
struct FCounters {std::atomic<uint64> Calls{0},Admitted{0},NotApplicable{0},Ticks{0};};
static FCounters Counts;
static bool TryBuild(const PreparedExecution::FNativeSolidStaticValues& values,FIntervalPoint& output)
{
    if(Enabled.GetValueOnAnyThread()!=1)return false;
    const bool observe=GeneralBoxMotionMemo::Observe();const uint64 start=observe?FPlatformTime::Cycles64():0;
    if(observe)++Counts.Calls;const bool known=BuildCompleteVolume(values,output);
    if(observe){Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-start);if(known)++Counts.Admitted;else ++Counts.NotApplicable;}
    return known;
}
static void Publish()
{
    if(!Counts.Calls.load())return;
    UE_LOG(LogTemp,Display,TEXT("[InstantMotionClearance] Scope=ProgramPrefixAtPreExit NoPASS=1 DefaultOFF=1 Calls=%llu Admitted=%llu NotApplicable=%llu Ticks=%llu SecondsPerCycle=%.17g"),
        Counts.Calls.load(),Counts.Admitted.load(),Counts.NotApplicable.load(),Counts.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
}
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}

namespace NativeLooseWholeDetail
{
struct FLooseMotionBounds {FIntervalPoint Hull,NativeCenter,ReferenceCenter;};
static YawFloorBoxDetail::FPoint RotateFamily(const YawFloorBoxDetail::FQuaternion& q,
    const YawFloorBoxDetail::FPoint& p)
{
    using namespace YawFloorBoxDetail;
    FPoint v;for(int32 d=0;d<3;++d)v.Axis[d]=q.Axis[d];
    auto t=Cross(v,p);for(int32 d=0;d<3;++d)t.Axis[d]=Product(Leaf(2),t.Axis[d]);
    const auto c=Cross(v,t);FPoint out;
    for(int32 d=0;d<3;++d)out.Axis[d]=Sum(Sum(p.Axis[d],Product(q.Axis[3],t.Axis[d])),c.Axis[d]);return out;
}
static bool BuildMotion(const FGeneralBoxMotionInput& v,FLooseMotionBounds& output,bool allowPrologueReuse=true)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace GeneralBoxDetail;using namespace YawFloorBoxDetail;
    const auto& owner=v.Owner;
    if(owner.Location.ContainsNaN()||owner.Velocity.ContainsNaN()||owner.Acceleration.ContainsNaN()||
        owner.AngularVelocity.ContainsNaN()||owner.AngularAcceleration.ContainsNaN()||
        owner.Rotation.ContainsNaN()||!owner.Rotation.IsNormalized()||v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||v.PhysicalHalfExtent.ContainsNaN()||v.QueryHalfExtent.ContainsNaN()||
        v.PhysicalHalfExtent.GetMin()<=0||v.QueryHalfExtent.GetMin()<=0)return false;
    FZeroOffsetKinematicCenterReference center;
    if(!BuildZeroOffsetKinematicCenterReference(owner.Location,owner.Velocity,owner.Acceleration,v.Horizon,
        FVector::ZeroVector,FVector::ZeroVector,center))return false;
    FLooseMotionBounds result;
    for(bool consumed:{true,false})
    {
        GeneralBoxPrologueReuse::FFamily family;
        if(!GeneralBoxPrologueReuse::BuildFamily(v,center,consumed,family,1,allowPrologueReuse))return false;
        const auto& bodyCenter=family.Center;const auto& bodyRotation=family.Rotation;
        if(consumed)result.NativeCenter=bodyCenter;else result.ReferenceCenter=bodyCenter;
        // One interval local box contains both eight-corner input families.
        // This executes the SAME rounded Rotate DAG for every fixed local point,
        // preserving its per-operand DAZ and per-node error allowances.
        FPoint localBox;
        for(int32 d=0;d<3;++d)
        {
            const double extent=FMath::Max(v.PhysicalHalfExtent[d],v.QueryHalfExtent[d]);
            localBox.Axis[d]={Interval{-extent,extent,true},0};
        }
        const auto rotated=PointRanges(RotateFamily(bodyRotation,localBox));
        if(!Detail::ValidPoint(rotated))return false;
        FIntervalPoint point;
        for(int32 d=0;d<3;++d)point.Axis[d]=Add(bodyCenter.Axis[d],rotated.Axis[d]);
        if(!YawFloorBoxDetail::Join(result.Hull,point))return false;
    }
    if(!Detail::ValidPoint(result.Hull))return false;
    output=MoveTemp(result);return true;
}
}
IAMSPEED_API bool BuildGeneralNativeBoxLooseSweepVolume(const PreparedExecution::FNativeSolidStaticValues& v,FIntervalPoint& output)
{
    if(InstantMotionClearanceDetail::TryBuild(v,output))return true;
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace FloorNativeSweepDetail;
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.Frozen||v.Prescribed||
        v.Query.Shape!=EQueryShape::Box||v.Query.HalfExtent!=v.QueryHalfExtent||
        !SameQuat(v.BodyState.Rotation,v.OwnerState.Rotation*v.LocalRotation)||!SameQuat(v.Query.Rotation,v.BodyState.Rotation)||
        !SameVector(v.Query.Start,v.BodyState.Location))return false;
    const FVector bodyOrigin=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+
        v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    const FVector chordEnd=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    if(!SameVector(bodyOrigin,v.BodyState.Location)||!SameVector(chordEnd,v.Query.End))return false;
    FGeneralBoxMotionInput motion;motion.Owner=v.OwnerState;motion.LocalCOM=v.LocalCOM;motion.LocalOffset=v.LocalOffset;
    motion.LocalRotation=v.LocalRotation;motion.PhysicalHalfExtent=v.PhysicalHalfExtent;motion.QueryHalfExtent=v.QueryHalfExtent;
    motion.Horizon=v.Horizon;NativeLooseWholeDetail::FLooseMotionBounds bounds;
    if(!NativeLooseWholeDetail::BuildMotion(motion,bounds))return false;
    auto hull=bounds.Hull;
    FIntervalPoint chord;if(!BuildFixedWorldQueryVolumeBounds(v.Query,chord)||!YawFloorBoxDetail::Join(hull,chord))return false;
    auto captured=v.Query;captured.Start=captured.End=v.BodyState.Location;captured.HalfExtent=v.PhysicalHalfExtent;
    FIntervalPoint pose;if(!BuildFixedWorldQueryVolumeBounds(captured,pose)||!YawFloorBoxDetail::Join(hull,pose))return false;
    auto footprintQuery=v.Query;footprintQuery.Start=footprintQuery.End=FVector3d::ZeroVector;
    FIntervalPoint footprint;if(!BuildFixedWorldQueryVolumeBounds(footprintQuery,footprint))return false;
    for(const auto& center:{bounds.NativeCenter,bounds.ReferenceCenter})
    {
        FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]=AnalyticCertified::Add(center.Axis[d],footprint.Axis[d]);
        if(!YawFloorBoxDetail::Join(hull,point))return false;
    }
    for(double shell:{v.Query.DomainTolerance,v.Query.InitialOverlapTolerance})
    {
        const auto bits=AnalyticCertified::Bits(shell);
        if((bits&AnalyticCertified::Magnitude)>=AnalyticCertified::Infinity||
            ((bits&AnalyticCertified::Sign)&&(bits&AnalyticCertified::Magnitude)))return false;
        for(int32 d=0;d<3;++d)hull.Axis[d]=AnalyticCertified::Add(hull.Axis[d],Interval{-shell,shell,true});
    }
    if(!Detail::ValidPoint(hull))return false;output=hull;return true;
}

static_assert(std::is_same_v<decltype(&BuildGeneralNativeBoxLooseSweepVolume),FGeneralNativeBoxMemoApi>);
static bool BuildGeneralBoxPrimitiveGroupedVolumeMemo(const FGeneralBoxMotionInput& input,const FQuat4d& axesRotation,
    double domainShell,double overlapShell,FDistanceBudget& budget,FGroupedBoxVolumeBounds& output)
{
    output={};FGeneralBoxMotionBounds motion;
    if(!BuildGeneralBoxMotionBoundsMemo(input,motion,1))return false;
    return NativeGroupedBoxDetail::RefinedPrimitiveFromWhole(input,motion,axesRotation,domainShell,overlapShell,budget,output);
}
}

// Same reviewed quaternion-difference theorem on ORIGINAL seconds.
// No rebasing; the exact existing trig bounds contract with this cell.
namespace Speed::Analytic::Certified::InstantMotionClearanceDetail
{
static bool BoundOwnerQuaternionDeltaRange(const FKinematicState& state,float first,float last,
    const FStaticNumericalBounds& constants,FQuaternionDelta& output)
{
    output={};
#if !defined(_MSC_VER) || !defined(_M_X64) || defined(_M_ARM64EC) || !PLATFORM_ENABLE_VECTORINTRINSICS
    return false;
#else
    const auto before=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!constants.Ready||before.Status!=EAnalyticNumericStatus::Complete||constants.Numeric!=before.ControlFingerprint()||
        !GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(first,last)||state.Rotation.ContainsNaN()||
        state.AngularVelocity.ContainsNaN()||state.AngularAcceleration.ContainsNaN())return false;
    FQuaternionDelta q0;const double captured[4]={state.Rotation.X,state.Rotation.Y,state.Rotation.Z,state.Rotation.W};
    for(int32 k=0;k<4;++k)
    {q0.Axis[k]=I::Exact(captured[k]);if(!BoundedMagnitude(q0.Axis[k],2.))return false;}
    const I time{double(first),double(last),true};const auto referenceSquared=A::Square(time);
    const auto consumedSquared=KinematicCenterDetail::RoundedBinary32(referenceSquared);
    const auto timeSquared=G::Hull(referenceSquared,consumedSquared);if(!A::Valid(timeSquared))return false;
    Y::FNode theta[3],squares[3];bool initialPossible=true;
    for(int32 d=0;d<3;++d)
    {
        const auto value=Y::Sum(Y::Product(Y::Leaf(state.AngularVelocity[d]),Y::FNode{time,0.}),
            Y::Product(Y::Product(Y::Leaf(.5),Y::Leaf(state.AngularAcceleration[d])),Y::FNode{timeSquared,0.}));
        const auto expanded=G::Expanded(value);if(!A::Valid(expanded))return false;
        theta[d]={expanded,0.};squares[d]=Y::Product(theta[d],theta[d]);
        initialPossible&=expanded.Lower<=0.&&expanded.Upper>=0.;
    }
    I angleSquared;
    for(int32 axisOrder=0;axisOrder<3;++axisOrder)
    {
        const auto total=G::Expanded(Y::Sum(Y::Sum(squares[axisOrder],squares[(axisOrder+1)%3]),squares[(axisOrder+2)%3]));
        if(!A::Valid(total))return false;angleSquared=A::Valid(angleSquared)?G::Hull(angleSquared,total):total;
    }
    if(!A::Valid(angleSquared))return false;angleSquared.Lower=FMath::Max(0.,angleSquared.Lower);
    I scale,cosine;if(!G::TrigPolynomialBounds(angleSquared,scale,cosine))return false;
    FQuaternionDelta delta,deltaMinusIdentity;
    for(int32 d=0;d<3;++d)delta.Axis[d]=G::Expanded(Y::Product(theta[d],Y::FNode{scale,0.}));
    delta.Axis[3]=cosine;
    for(int32 k=0;k<4;++k)
    {
        if(!BoundedMagnitude(delta.Axis[k],2.))return false;
        deltaMinusIdentity.Axis[k]=k==3?A::Subtract(delta.Axis[k],I::Exact(1.)):delta.Axis[k];
    }
    FQuaternionDelta exactRawDifference,rawDifference;
    if(!ExactHamilton(deltaMinusIdentity,q0,exactRawDifference))return false;
    for(int32 k=0;k<4;++k)
    {
        if(!AddUniformError(exactRawDifference.Axis[k],constants.Hamilton2Error,rawDifference.Axis[k])||
            !BoundedMagnitude(A::Add(q0.Axis[k],rawDifference.Axis[k]),8.))return false;
    }
    // Norm multiplication preserves the delta/q0 correlation that would be
    // lost by a direct sum of squares of a four-dimensional raw-value box.
    const auto idealSquared=A::Multiply(QuaternionNormSquared(delta),QuaternionNormSquared(q0));
    if(!A::Valid(idealSquared)||idealSquared.Upper>16.)return false;
    const auto errorNorm=A::Multiply(I::Exact(2.),I::Exact(constants.Hamilton2Error));
    const auto normSquaredError=A::Add(A::Multiply(I::Exact(8.),errorNorm),A::Square(errorNorm));
    const auto allSquaredError=A::Add(normSquaredError,I::Exact(constants.Dot8Error));
    if(!A::Valid(errorNorm)||errorNorm.Upper>4.||!A::Valid(allSquaredError))return false;
    I computedSquared;if(!AddUniformError(idealSquared,allSquaredError.Upper,computedSquared)||
        computedSquared.Lower<=double(UE_SMALL_NUMBER))return false;
    const auto root=[](double value){return _mm_cvtsd_f64(_mm_sqrt_sd(_mm_setzero_pd(),_mm_set_sd(value)));};
    double rootLower=0,rootUpper=0;
    if(!A::Outward(root(computedSquared.Lower),false,rootLower)||
        !A::Outward(root(computedSquared.Upper),true,rootUpper)||rootLower<=0)return false;
    volatile double inverseLower=1./rootUpper,inverseUpper=1./rootLower;
    I inverse;inverse.Known=A::Outward(inverseLower,false,inverse.Lower)&&A::Outward(inverseUpper,true,inverse.Upper);
    if(!A::Valid(inverse)||inverse.Lower<0||inverse.Upper>2.)return false;
    const auto inverseDifference=A::Subtract(inverse,I::Exact(1.));FQuaternionDelta result;
    for(int32 k=0;k<4;++k)
    {
        // E(raw component) was already included in rawDifference. The cached
        // multiply error applies to ACTUAL raw (norm<=8) times native inverse.
        const auto exactNormalizedDifference=A::Add(A::Multiply(rawDifference.Axis[k],inverse),
            A::Multiply(q0.Axis[k],inverseDifference));
        I normalizedDifference;
        if(!AddUniformError(exactNormalizedDifference,constants.Normalize8x2Error,normalizedDifference))return false;
        result.Axis[k]=G::Hull(rawDifference.Axis[k],normalizedDifference);
        if(initialPossible)result.Axis[k]=G::Hull(result.Axis[k],I::Exact(0.));
        if(!BoundedMagnitude(A::Add(q0.Axis[k],result.Axis[k]),2.))return false;
    }
    const auto after=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=before.ControlFingerprint())return false;
    output=MoveTemp(result);return true;
#endif
}
}

namespace Speed::Analytic::Certified::CorrelatedNativeBoxGroups
{
namespace A=::AnalyticCertified;namespace X=InstantMotionClearanceDetail;namespace Y=YawFloorBoxDetail;
using I=A::Interval;using Matrix=X::FRotationDeltaMatrix;
struct FFrame {FVector3d Origin;FVector3d Axis[3];Matrix Inverse;};
static bool Divide(I numerator,I denominator,I& output)
{
    output={};if(!A::Valid(numerator)||!A::Valid(denominator)||!(denominator.Lower>0||denominator.Upper<0))return false;
    volatile double lo=1./denominator.Upper,hi=1./denominator.Lower;I inverse;
    inverse.Known=A::Outward(lo,false,inverse.Lower)&&A::Outward(hi,true,inverse.Upper);
    if(!A::Valid(inverse))return false;output=A::Multiply(numerator,inverse);return A::Valid(output);
}
static bool PrepareFrame(const FVector3d& origin,const Private::FBoxSweepContext& sat,FFrame& output)
{
    output={};if(!Detail::ValidPoint(Detail::Point(origin)))return false;I n[3][3],cofactor[3][3];
    for(int32 i=0;i<3;++i)for(int32 j=0;j<3;++j)
    {n[i][j]=I::Exact(sat.Axis[i][j]);if(!A::Valid(n[i][j]))return false;}
    for(int32 i=0;i<3;++i)for(int32 j=0;j<3;++j)
    {
        const int32 r1=(i+1)%3,r2=(i+2)%3,c1=(j+1)%3,c2=(j+2)%3;
        cofactor[i][j]=A::Subtract(A::Multiply(n[r1][c1],n[r2][c2]),A::Multiply(n[r1][c2],n[r2][c1]));
    }
    I determinant=I::Exact(0.);for(int32 j=0;j<3;++j)determinant=A::Add(determinant,A::Multiply(n[0][j],cofactor[0][j]));
    if(!A::Valid(determinant)||!(determinant.Lower>0||determinant.Upper<0))return false;
    FFrame result;result.Origin=origin;
    for(int32 i=0;i<3;++i)
    {
        result.Axis[i]=sat.Axis[i];
        for(int32 j=0;j<3;++j)if(!Divide(cofactor[j][i],determinant,result.Inverse.Element[i][j])||
            !X::BoundedMagnitude(result.Inverse.Element[i][j],4.))return false;
    }
    output=MoveTemp(result);return true;
}
static bool SlabsToGroup(const FFrame& frame,const I slabs[3],FBoxVolumeGroup& output)
{
    output={};for(int32 d=0;d<3;++d)if(!A::Valid(slabs[d]))return false;FBoxVolumeGroup result;
    for(int32 corner=0;corner<8;++corner)
    {
        FIntervalPoint vertex;
        for(int32 d=0;d<3;++d)
        {
            I offset=I::Exact(0.);
            for(int32 j=0;j<3;++j)offset=A::Add(offset,A::Multiply(frame.Inverse.Element[d][j],
                I::Exact((corner&(1<<j))?slabs[j].Upper:slabs[j].Lower)));
            vertex.Axis[d]=A::Add(I::Exact(frame.Origin[d]),offset);
        }
        if(!Detail::ValidPoint(vertex))return false;result.Corners.Add(vertex);
    }
    output=MoveTemp(result);return true;
}
struct FPrepared
{
    PreparedExecution::FNativeSolidStaticValues Captured;
    FZeroOffsetKinematicCenterReference CenterReference;
    X::FStaticNumericalBounds Constants;
    X::FQuaternionDelta LocalQuaternion;
    Matrix FixedBody,SatBody;
    FIntervalPoint InitialCOM,InitialOffset;
    FVector3d AbsCOM,AbsOffset;
    double COMError=0,OffsetError=0,CornerError=0;
    uint64 Numeric=0;
};
static bool Prepare(const PreparedExecution::FNativeSolidStaticValues& v,FPrepared& output)
{
    output={};const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    using namespace FloorNativeSweepDetail;
    if(v.Shape!=PreparedExecution::ENativeSolidQueryShape::Box||v.Frozen||v.Prescribed||
        v.Query.Shape!=EQueryShape::Box||v.Query.HalfExtent!=v.QueryHalfExtent||
        v.OwnerState.Location.ContainsNaN()||v.OwnerState.Velocity.ContainsNaN()||v.OwnerState.Acceleration.ContainsNaN()||
        v.OwnerState.Rotation.ContainsNaN()||!v.OwnerState.Rotation.IsNormalized()||
        v.LocalRotation.ContainsNaN()||!v.LocalRotation.IsNormalized()||
        v.BodyState.Rotation.ContainsNaN()||!v.BodyState.Rotation.IsNormalized()||
        v.LocalCOM.ContainsNaN()||v.LocalOffset.ContainsNaN()||
        v.PhysicalHalfExtent.ContainsNaN()||v.PhysicalHalfExtent.GetMin()<=0||
        v.QueryHalfExtent.ContainsNaN()||v.QueryHalfExtent.GetMin()<=0||
        !SameQuat(v.BodyState.Rotation,v.OwnerState.Rotation*v.LocalRotation)||
        !SameQuat(v.Query.Rotation,v.BodyState.Rotation)||!SameVector(v.Query.Start,v.BodyState.Location))return false;
    const FVector origin=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    const FVector end=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    if(origin.ContainsNaN()||end.ContainsNaN()||!SameVector(origin,v.BodyState.Location)||!SameVector(end,v.Query.End))return false;
    FPrepared result;result.Captured=v;result.Numeric=before.ControlFingerprint();
    if(!BuildZeroOffsetKinematicCenterReference(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,
        v.Horizon,FVector::ZeroVector,FVector::ZeroVector,result.CenterReference)||!X::GetStaticNumericalBounds(result.Constants)||
        !X::FixedRotationMatrix(v.BodyState.Rotation,result.FixedBody))return false;
    Private::FBoxSweepContext sat;sat.Initialize(v.Query);
    const double q[4]={v.LocalRotation.X,v.LocalRotation.Y,v.LocalRotation.Z,v.LocalRotation.W};
    for(int32 k=0;k<4;++k){result.LocalQuaternion.Axis[k]=I::Exact(q[k]);if(!X::BoundedMagnitude(result.LocalQuaternion.Axis[k],2.))return false;}
    FVector3d extent;
    for(int32 d=0;d<3;++d)
    {
        result.AbsCOM[d]=FMath::Abs(v.LocalCOM[d]);result.AbsOffset[d]=FMath::Abs(v.LocalOffset[d]);
        extent[d]=FMath::Max(v.PhysicalHalfExtent[d],v.QueryHalfExtent[d]);
        for(int32 j=0;j<3;++j)result.SatBody.Element[d][j]=I::Exact(sat.Axis[j][d]);
    }
    if(!X::LocalError(result.Constants,result.AbsCOM,result.COMError)||!X::LocalError(result.Constants,result.AbsOffset,result.OffsetError)||
        !X::LocalError(result.Constants,extent,result.CornerError))return false;
    result.InitialCOM=FixedQueryBoundsDetail::Rotate(v.OwnerState.Rotation,v.LocalCOM);
    result.InitialOffset=FixedQueryBoundsDetail::Rotate(v.OwnerState.Rotation,v.LocalOffset);
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!Detail::ValidPoint(result.InitialCOM)||!Detail::ValidPoint(result.InitialOffset)||after.ControlFingerprint()!=result.Numeric)return false;
    output=MoveTemp(result);return true;
}
}

namespace Speed::Analytic::Certified::CorrelatedNativeBoxGroups
{
// Every output has the SAME fixed local-corner role as the original model.
// Matrix is the certified exact Rotate polynomial over the original time
// range; Error bounds the actual native Rotate DAG on all admitted operands.
static bool MatrixCorner(const Matrix& matrix,const FVector3d& local,double error,FIntervalPoint& output)
{
    output={};FIntervalPoint result;
    for(int32 d=0;d<3;++d)
    {
        I value=I::Exact(0.);
        for(int32 j=0;j<3;++j)value=A::Add(value,A::Multiply(matrix.Element[d][j],I::Exact(local[j])));
        if(!X::AddUniformError(value,error,result.Axis[d]))return false;
    }
    if(!Detail::ValidPoint(result))return false;output=result;return true;
}

static bool MotionRange(const FPrepared& prepared,float first,float last,FGeneralBoxMotionBounds& output)
{
    output={};const auto& v=prepared.Captured;
    const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(numeric.ControlFingerprint()!=prepared.Numeric||!GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(first,last)||last>v.Horizon)return false;
    X::FQuaternionDelta ownerDelta,bodyDelta;
    if(!X::BoundOwnerQuaternionDeltaRange(v.OwnerState,first,last,prepared.Constants,ownerDelta)||
        !X::ExactHamilton(ownerDelta,prepared.LocalQuaternion,bodyDelta))return false;
    const auto he=A::Multiply(I::Exact(2.),I::Exact(prepared.Constants.Hamilton2Error));if(!A::Valid(he))return false;
    const double bodyRaw[4]={v.BodyState.Rotation.X,v.BodyState.Rotation.Y,v.BodyState.Rotation.Z,v.BodyState.Rotation.W};
    for(int32 k=0;k<4;++k)
    {I value;if(!X::AddUniformError(bodyDelta.Axis[k],he.Upper,value))return false;bodyDelta.Axis[k]=value;
        if(!X::BoundedMagnitude(A::Add(I::Exact(bodyRaw[k]),value),2.))return false;}
    Matrix ownerDifference,bodyDifference,matrix;
    if(!X::RotationDeltaMatrix(v.OwnerState.Rotation,ownerDelta,ownerDifference)||
        !X::RotationDeltaMatrix(v.BodyState.Rotation,bodyDelta,bodyDifference))return false;
    for(int32 d=0;d<3;++d)for(int32 j=0;j<3;++j)
    {matrix.Element[d][j]=A::Add(prepared.FixedBody.Element[d][j],bodyDifference.Element[d][j]);if(!A::Valid(matrix.Element[d][j]))return false;}
    FIntervalPoint comDelta,offsetDelta;
    if(!X::RotationDeltaSupport(ownerDifference,prepared.AbsCOM,comDelta)||
        !X::RotationDeltaSupport(ownerDifference,prepared.AbsOffset,offsetDelta)||
        !X::AddError(comDelta,prepared.COMError)||!X::AddError(offsetDelta,prepared.OffsetError))return false;
    FIntervalPoint futureCOM,futureOffset;
    for(int32 d=0;d<3;++d)
    {futureCOM.Axis[d]=A::Add(prepared.InitialCOM.Axis[d],comDelta.Axis[d]);futureOffset.Axis[d]=A::Add(prepared.InitialOffset.Axis[d],offsetDelta.Axis[d]);}
    FGeneralBoxMotionBounds result;
    for(bool consumed:{true,false})
    {
        FIntervalPoint ownerCenter;
        if(consumed)ownerCenter=BoundConsumedCenterBetweenFloatSeconds(prepared.CenterReference,first,last);
        else
        {
            const I time{double(first),double(last),true};const auto squared=A::Square(time);
            for(int32 d=0;d<3;++d)ownerCenter.Axis[d]=A::Add(A::Add(I::Exact(v.OwnerState.Location[d]),
                A::Multiply(I::Exact(v.OwnerState.Velocity[d]),time)),
                A::Multiply(A::Multiply(I::Exact(.5),I::Exact(v.OwnerState.Acceleration[d])),squared));
        }
        FIntervalPoint center;
        for(int32 d=0;d<3;++d)center.Axis[d]=A::Add(A::Subtract(ownerCenter.Axis[d],futureCOM.Axis[d]),futureOffset.Axis[d]);
        if(!Detail::ValidPoint(center))return false;
        if(consumed)result.NativeCenter=center;else result.ReferenceCenter=center;
        for(const auto& extent:{v.PhysicalHalfExtent,v.QueryHalfExtent})
        {
            for(int32 corner=0;corner<8;++corner)
            {
                FVector3d local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
                FIntervalPoint rotated,point;if(!MatrixCorner(matrix,local,prepared.CornerError,rotated))return false;
                // Same native association: actual body center + actual
                // rotated local vertex. Outward Add also covers its rounding.
                for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],rotated.Axis[d]);
                if(!YawFloorBoxDetail::Join(result.Hull,point))return false;result.Corners.Add(point);
            }
        }
    }
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.ControlFingerprint()!=prepared.Numeric||result.Corners.Num()!=32||!Detail::ValidPoint(result.Hull))return false;
    output=MoveTemp(result);return true;
}
static bool ContractedPart(const FPrepared& prepared,float first,float last,FDistanceBudget& budget,FGeneralBoxMotionBounds& output)
{
    output={};struct FCell{float First,Last;int32 Depth;};TArray<FCell,TInlineAllocator<4>> stack;
    stack.Add(FCell{first,last,0});FGeneralBoxMotionBounds result;bool have=false;
    while(!stack.IsEmpty())
    {
        if(!budget.Consume())return false;const auto cell=stack.Pop(EAllowShrinking::No);
        const float middle=cell.First+.5f*(cell.Last-cell.First);
        if(cell.Depth<2&&middle>cell.First&&middle<cell.Last&&
            GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(cell.First,middle)&&
            GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(middle,cell.Last))
        {stack.Add(FCell{middle,cell.Last,cell.Depth+1});stack.Add(FCell{cell.First,middle,cell.Depth+1});continue;}
        FGeneralBoxMotionBounds part;if(!MotionRange(prepared,cell.First,cell.Last,part))return false;
        if(!have){result=MoveTemp(part);have=true;continue;}
        if(!budget.Consume())return false;result.Hull=TimeDetail::Hull(result.Hull,part.Hull);
        if(!budget.Consume())return false;result.NativeCenter=TimeDetail::Hull(result.NativeCenter,part.NativeCenter);
        if(!budget.Consume())return false;result.ReferenceCenter=TimeDetail::Hull(result.ReferenceCenter,part.ReferenceCenter);
        for(int32 k=0;k<32;++k)
        {if(!budget.Consume())return false;result.Corners[k]=TimeDetail::Hull(result.Corners[k],part.Corners[k]);}
    }
    if(!have)return false;output=MoveTemp(result);return true;
}
static bool GlobalParts(const FPrepared& prepared,FDistanceBudget& budget,TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>>& output)
{
    output.Reset();const float horizon=prepared.Captured.Horizon,middle=.5f*horizon;
    const bool split=middle>0&&middle<horizon&&GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(0.f,middle)&&
        GeneralBoxGlobalTimeDetail::ValidFloatSecondsRange(middle,horizon);
    TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>> result;
    if(split)
    {
        for(const auto& range:{TPair<float,float>(0.f,middle),TPair<float,float>(middle,horizon)})
        {FGeneralBoxMotionBounds part;if(!ContractedPart(prepared,range.Key,range.Value,budget,part))return false;result.Add(MoveTemp(part));}
    }
    else
    {FGeneralBoxMotionBounds whole;if(!budget.Consume()||!MotionRange(prepared,0.f,horizon,whole))return false;result.Add(MoveTemp(whole));}
    output=MoveTemp(result);return true;
}
static bool FixedGroup(const FPrepared& prepared,const FIntervalPoint& center,const FVector3d& extent,FBoxVolumeGroup& output)
{
    output={};FBoxVolumeGroup result;
    for(int32 corner=0;corner<8;++corner)
    {
        FVector3d local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
        FIntervalPoint polynomial,sat;
        if(!MatrixCorner(prepared.FixedBody,local,prepared.CornerError,polynomial))return false;
        // Literal original fixed SAT multiply/add association, per SAME
        // signed local vertex. Its rounded values are enclosed directly.
        for(int32 d=0;d<3;++d)
        {
            sat.Axis[d]=I::Exact(0.);
            for(int32 j=0;j<3;++j)sat.Axis[d]=A::Add(sat.Axis[d],A::Multiply(prepared.SatBody.Element[d][j],I::Exact(local[j])));
        }
        if(!Detail::ValidPoint(sat))return false;
        const auto rotated=TimeDetail::Hull(polynomial,sat);FIntervalPoint point;
        for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],rotated.Axis[d]);
        if(!Detail::ValidPoint(point))return false;result.Corners.Add(point);
    }
    output=MoveTemp(result);return true;
}
static bool Build(const PreparedExecution::FNativeSolidStaticValues& v,bool fine,FDistanceBudget& budget,
    FGroupedBoxVolumeBounds& output,const TCHAR** stage=nullptr,FClosedGroupedBoxVolumePtr* closedOutput=nullptr)
{
    output={};if(closedOutput)closedOutput->Reset();const auto note=[&](const TCHAR* text){if(stage)*stage=text;};note(TEXT("correlated_native_inputs"));
    // Same bounded-model charges as the old coarse producer; the old fine
    // preflight had no separate logical charge. Native runtime stays charged.
    if(!fine&&!budget.Consume())return false;FPrepared prepared;if(!Prepare(v,prepared))return false;
    TArray<FGeneralBoxMotionBounds,TInlineAllocator<2>> pieces;
    if(fine){note(TEXT("correlated_global_parts"));if(!GlobalParts(prepared,budget,pieces))return false;}
    else
    {note(TEXT("correlated_whole_motion"));FGeneralBoxMotionBounds whole;if(!budget.Consume()||!MotionRange(prepared,0.f,v.Horizon,whole))return false;pieces.Add(MoveTemp(whole));}
    TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;
    for(const auto& part:pieces)if(!NativeGroupedBoxDetail::MotionGroups(part,groups))return false;
    FIntervalPoint chordCenter;
    for(int32 d=0;d<3;++d)chordCenter.Axis[d]=A::Add(I::Exact(v.Query.Start[d]),
        A::Multiply(A::Subtract(I::Exact(v.Query.End[d]),I::Exact(v.Query.Start[d])),I{0.,1.,true}));
    FBoxVolumeGroup chord,pose;note(TEXT("correlated_fixed_groups"));
    if(!FixedGroup(prepared,chordCenter,v.QueryHalfExtent,chord)||
        !FixedGroup(prepared,Detail::Point(v.BodyState.Location),v.PhysicalHalfExtent,pose))return false;
    groups.Add(MoveTemp(chord));groups.Add(MoveTemp(pose));
    for(const auto& part:pieces)for(const auto& center:{part.NativeCenter,part.ReferenceCenter})
    {FBoxVolumeGroup footprint;if(!FixedGroup(prepared,center,v.QueryHalfExtent,footprint))return false;groups.Add(MoveTemp(footprint));}
    if(pieces.IsEmpty()||pieces.Num()>2||groups.Num()!=pieces.Num()*6+2)return false;
    FIntervalPoint broad;
    for(const auto& group:groups)for(const auto& vertex:group.Corners)if(!YawFloorBoxDetail::Join(broad,vertex))return false;
    note(TEXT("correlated_close_grouped_volume"));
    if(!NativeGroupedBoxDetail::Close(broad,groups,v.Query.Rotation,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,budget,output,closedOutput))return false;
    const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.ControlFingerprint()!=prepared.Numeric){output={};if(closedOutput)closedOutput->Reset();return false;}return true;
}
}

namespace Speed::Analytic::Certified::CorrelatedNativeBoxGroups
{
TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.CorrelatedNativeBoxGroups"),0,
    TEXT("Default OFF. Original local-corner families from a certified global-time Rotate matrix; optional coarse only, protected original fine and endpoint checks remain authoritative."));
struct FCounters{std::atomic<uint64> Calls{0},Admitted{0},Fallback{0},Used{0},Ticks{0};};
static FCounters Counts[2];
static bool TryBuild(const PreparedExecution::FNativeSolidStaticValues& v,bool fine,FDistanceBudget& budget,
    FGroupedBoxVolumeBounds& output,const TCHAR** stage=nullptr,FClosedGroupedBoxVolumePtr* closedOutput=nullptr)
{
    if(closedOutput)closedOutput->Reset();
    if(Enabled.GetValueOnAnyThread()!=1)return false;
    const bool observe=GeneralBoxMotionMemo::Observe();const uint64 tick=observe?FPlatformTime::Cycles64():0,before=budget.Used;
    auto& row=Counts[fine?1:0];if(observe)++row.Calls;
    const bool result=Build(v,fine,budget,output,stage,closedOutput);
    if(observe){row.Ticks.fetch_add(FPlatformTime::Cycles64()-tick);row.Used.fetch_add(budget.Used-before);if(result)++row.Admitted;else ++row.Fallback;}
    return result;
}
static bool BuildFineOrOriginal(const PreparedExecution::FNativeSolidStaticValues& v,FDistanceBudget& budget,
    FGroupedBoxVolumeBounds& output,const TCHAR** stage=nullptr,FClosedGroupedBoxVolumePtr* closedOutput=nullptr)
{
    // Correlated geometry is an OPTIONAL COARSE proof only. Attempt has
    // already protected the original fine allowance in this same account.
    // A failed coarse proof therefore reaches the literal original producer,
    // including all spent coarse work, without retry/reset or a wider cap.
    return BuildGeneralNativeBoxGroupedVolume(v,budget,output,stage,NativeBoxAntipodal::Enabled.GetValueOnAnyThread()==1,closedOutput);
}
static void Publish()
{
    for(int32 kind=0;kind<2;++kind)
    {
        const auto& row=Counts[kind];if(!row.Calls.load())continue;
        UE_LOG(LogTemp,Display,TEXT("[CorrelatedNativeBoxGroups] Scope=ProgramPrefixAtPreExit NoPASS=1 DefaultOFF=1 Groups=%d Calls=%llu Admitted=%llu Fallback=%llu Used=%llu Ticks=%llu SecondsPerCycle=%.17g"),
            kind?14:8,row.Calls.load(),row.Admitted.load(),row.Fallback.load(),row.Used.load(),row.Ticks.load(),FPlatformTime::GetSecondsPerCycle64());
    }
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
}

// Existing PreparedPhysicalExecution TU supplies atomic, ConsoleManager,
// PlatformTime and CoreDelegates. Passive counters never authorize geometry.
namespace NativePrimitiveCoarseProfile
{
enum ERow:uint8 {WholeModel,CoarseVolume,CoarseSelect,CoarseProve,FineVolume,FineSelect,FineProve,PredictionVolume,PredictionSelect,PredictionProve,PredictionSimpleProve,PredictionCoarseVolume,PredictionCoarseSelect,PredictionCoarseProve,PredictionExtraAxes,PredictionActualFirst,Count};
struct FRow {std::atomic<uint64> Calls{0},Finished{0},Ticks{0},True{0},False{0},Used{0},Cells{0},Candidates{0},Reused{0};};
FRow Rows[Count];
struct FScope
{
    FRow& Row;Speed::Analytic::Certified::FDistanceBudget& Budget;
    bool Active=false;uint64 Start=0,Before=0;
    FScope(ERow id,Speed::Analytic::Certified::FDistanceBudget& budget):Row(Rows[id]),Budget(budget)
    {
        static const IConsoleVariable* flag=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
        Active=flag&&flag->GetInt()!=0;
        if(Active){++Row.Calls;Before=budget.Used;Start=FPlatformTime::Cycles64();}
    }
    void Result(bool result){if(Active)++(result?Row.True:Row.False);}
    void Work(uint64 cells,int32 candidates,bool reused=false)
    {if(Active){Row.Cells.fetch_add(cells);Row.Candidates.fetch_add(uint64(FMath::Max(0,candidates)));Row.Reused.fetch_add(reused?1:0);}}
    ~FScope(){if(Active){Row.Ticks.fetch_add(FPlatformTime::Cycles64()-Start);Row.Used.fetch_add(Budget.Used-Before);++Row.Finished;}}
};
void PublishSnapshot(const TCHAR* scope)
{
    static constexpr const TCHAR* names[]={TEXT("WholeModel"),TEXT("CoarseVolume"),TEXT("CoarseSelect"),TEXT("CoarseProve"),TEXT("FineVolume"),TEXT("FineSelect"),TEXT("FineProve"),TEXT("PredictionVolume"),TEXT("PredictionSelect"),TEXT("PredictionProve"),TEXT("PredictionSimpleProve"),TEXT("PredictionCoarseVolume"),TEXT("PredictionCoarseSelect"),TEXT("PredictionCoarseProve"),TEXT("PredictionExtraAxes"),TEXT("PredictionActualFirst")};
    FString text=FString::Printf(TEXT("[NativePrimitiveCoarseProfile] Scope=%s CoarseSuccess=CoarseProveTrue NoGainClaim=1 InclusiveRowsNotAdditive=1 SecondsPerCycle=%.17g"),scope,FPlatformTime::GetSecondsPerCycle64());
    for(uint8 i=0;i<Count;++i)
    {
        const auto& r=Rows[i];const uint64 calls=r.Calls.load(),finished=r.Finished.load();
        text+=FString::Printf(TEXT(" | Row=%s Coherent=%d Calls=%llu Finished=%llu Ticks=%llu True=%llu False=%llu BudgetUsed=%llu Cells=%llu Candidates=%llu ReusedPartition=%llu"),
            names[i],int32(calls==finished),calls,finished,r.Ticks.load(),r.True.load(),r.False.load(),r.Used.load(),r.Cells.load(),r.Candidates.load(),r.Reused.load());
    }
    UE_LOG(LogTemp,Display,TEXT("%s"),*text);
}
void Publish(){PublishSnapshot(TEXT("ProgramPrefixAtPreExit"));}
// Existing same-DLL registration pattern; no worker modifies the delegate.
struct FRegister {FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};
FRegister Register;
}

// Existing IAS TU: immutable values only, no live lookup or kernel fallback.
namespace Speed::PreparedExecution
{
namespace NativePredictionObliqueAxes
{
namespace A=::AnalyticCertified;
static TAutoConsoleVariable<int32> Enabled(TEXT("p.IAmSpeed.AnalyticWorld.NativePredictionObliqueAxes"),0,
    TEXT("Default OFF. Add at most six proposed original-candidate axes to the complete coarse volume; every added slab remains in its newly sealed proof. Existing maximum12 axes and fine reserve unchanged."),ECVF_Default);
static TAutoConsoleVariable<int32> ActualFirst(TEXT("p.IAmSpeed.AnalyticWorld.NativePredictionActualFirst"),0,
    TEXT("Default OFF. Try a bounded certificate over the original rotating Box before the general group search."),ECVF_Default);
static TAutoConsoleVariable<int32> RefinedWholeGroups(TEXT("p.IAmSpeed.AnalyticWorld.NativePredictionRefinedWholeGroups"),0,
    TEXT("Default OFF. Try six closed global-time cells for a whole Box proof on the same complete candidate roster and paid account."));
static TAutoConsoleVariable<int32> RefinedActualTail(TEXT("p.IAmSpeed.AnalyticWorld.NativePredictionRefinedActualTail"),0,
    TEXT("Default OFF. Retry actual-motion clearance on the observed stadium rosters using fifteen tail-weighted closed time cells."),ECVF_Default);
static bool Finite(const FVector3d& p){return A::Finite(p.X)&&A::Finite(p.Y)&&A::Finite(p.Z);}
static bool Midpoint(const C::FIntervalPoint& p,FVector3d& out)
{if(!C::Detail::ValidPoint(p))return false;for(int32 d=0;d<3;++d)out[d]=.5*p.Axis[d].Lower+.5*p.Axis[d].Upper;return Finite(out);}
// Scale/sign are proposals only. Close recomputes all original interval support
// values; no unit-normal or derivative theorem is inferred from this vector.
static bool CanonicalAxis(FVector3d& axis)
{
    if(!Finite(axis))return false;int32 largest=0;
    for(int32 d=1;d<3;++d)if(FMath::Abs(axis[d])>FMath::Abs(axis[largest]))largest=d;
    const double scale=FMath::Abs(axis[largest]);if(!(scale>0)||!A::Finite(scale))return false;
    axis/=scale;if(axis[largest]<0)axis=-axis;for(int32 d=0;d<3;++d)if(axis[d]==0)axis[d]=0.;
    return C::GroupedBoxDetail::ValidAxis(axis);
}
static bool SameDirection(FVector3d a,FVector3d b)
{
    if(!CanonicalAxis(a)||!CanonicalAxis(b))return false;
    for(int32 d=0;d<3;++d)if(A::Bits(a[d])!=A::Bits(b[d]))return false;return true;
}
static bool Propose(const C::FIntervalTensorSurface& surface,const FVector3d& target,C::FDistanceBudget& budget,FVector3d& out,bool& found)
{
    found=false;out=FVector3d::ZeroVector;
    if(surface.DegreeU<1||surface.DegreeV<1||surface.DegreeU>16||surface.DegreeV>16||
        surface.ControlPoints.Num()!=(surface.DegreeU+1)*(surface.DegreeV+1)||surface.ControlPoints.Num()>64)return true;
    double best=std::numeric_limits<double>::max();const int32 stride=surface.DegreeV+1;
    for(int32 u=0;u<surface.DegreeU;++u)for(int32 v=0;v<surface.DegreeV;++v)
    {
        if(!budget.Consume())return false;
        FVector3d p00,p01,p10,p11;
        if(!Midpoint(surface.ControlPoints[u*stride+v],p00)||!Midpoint(surface.ControlPoints[u*stride+v+1],p01)||
            !Midpoint(surface.ControlPoints[(u+1)*stride+v],p10)||!Midpoint(surface.ControlPoints[(u+1)*stride+v+1],p11))continue;
        const auto center=.25*p00+.25*p01+.25*p10+.25*p11;
        const auto du=.5*(p10-p00)+.5*(p11-p01),dv=.5*(p01-p00)+.5*(p11-p10);
        auto axis=FVector3d::CrossProduct(du,dv);const double score=(center-target).SquaredLength();
        if(!Finite(center)||!A::Finite(score)||score>=best||!CanonicalAxis(axis))continue;
        out=axis;best=score;found=true;
    }
    return true;
}
// The failed original cell only suggests an extra direction. The proof
// still projects and checks the complete original regions and providers.
static bool ProposeAt(const C::FIntervalTensorSurface& surface,FVector2d uv,
    C::FDistanceBudget& budget,FVector3d& axis)
{
    if(surface.DegreeU<1||surface.DegreeV<1||surface.ControlPoints.Num()>64||!budget.Consume())return false;
    C::Detail::FNode original;original.DegreeU=surface.DegreeU;original.DegreeV=surface.DegreeV;
    original.Net.Append(surface.ControlPoints.GetData(),surface.ControlPoints.Num());
    FVector3d tangents[2];
    for(int32 d=0;d<2;++d)
    {
        if(!budget.Consume())return false;
        C::Detail::FNode derivative;
        if(!C::ContactDetail::OriginalDerivativeNet(original,d==0,derivative))return false;
        C::FIntervalTensorSurface net;net.DegreeU=derivative.DegreeU;net.DegreeV=derivative.DegreeV;
        net.ControlPoints.Append(derivative.Net.GetData(),derivative.Net.Num());
        C::FIntervalPoint tangent;
        if(!C::ProjectedDistanceDetail::EvaluateFixedUV(net,uv,tangent)||!Midpoint(tangent,tangents[d]))return false;
    }
    axis=FVector3d::CrossProduct(tangents[0],tangents[1]);return CanonicalAxis(axis);
}
// Pure optional volume construction. Partition is still bound to the SAME
// broad hull. Already-inflated groups are not inflated a second time.
static bool Augment(const C::FReferenceBoundsCandidateSetPtr& partition,const FVector3d& target,
    C::FDistanceBudget& budget,C::FGroupedBoxVolumeBounds& volume,bool& changed,C::FClosedGroupedBoxVolumePtr* closedOutput=nullptr,
    const C::FGroupedBoxSeparationObservation* failedCell=nullptr)
{
    changed=false;
    if(!partition||!partition->GetScene()||!C::GroupedBoxDetail::SamePoint(partition->GetQueryHull(),volume.BroadHull)||
        !Finite(target)||volume.Axes.IsEmpty()||volume.Axes.Num()>12||budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return false;
    if(volume.Axes.Num()==12||partition->GetCandidateIndices().IsEmpty())return true;
    constexpr int32 allowance=768;
    if(int64(budget.Remaining)-budget.ConsumptionFloor<allowance)return true;
    if(budget.Used>MAX_uint64-uint64(allowance))return false;
    C::FScopedDistanceBudgetFloor limit(budget,budget.Remaining-allowance);if(!limit.IsValid())return false;
    TArray<FVector3d,TInlineAllocator<8>> axes=volume.Axes;
    int32 read=0;
    for(int32 index:partition->GetCandidateIndices())
    {
        if(read++>=6||axes.Num()>=12)break;
        C::FIntervalTensorSurface surface;
        if(!partition->GetScene()->CopySurfaceForQuery(index,budget,surface))return false;
        FVector3d axis;bool found=false;if(!Propose(surface,target,budget,axis,found))return false;
        const auto add=[&](const FVector3d& direction)
        {
            if(axes.Num()>=12)return;
            for(const auto& old:axes)if(SameDirection(direction,old))return;
            axes.Add(direction);
        };
        if(found)add(axis);
        if(failedCell&&index==failedCell->Provider&&axes.Num()<12)
        {
            const FVector2d uv(.5*failedCell->MinU+.5*failedCell->MaxU,.5*failedCell->MinV+.5*failedCell->MaxV);
            if(ProposeAt(surface,uv,budget,axis))add(axis);
        }
    }
    if(axes.Num()==volume.Axes.Num())return true;
    C::FGroupedBoxVolumeBounds complete;
    C::FClosedGroupedBoxVolumePtr replacement;
    if(closedOutput)
    {
        if(!C::FClosedGroupedBoxVolume::Build(volume.BroadHull,volume.Groups,axes,budget,replacement))return false;
        complete=replacement->GetVolumeForInspection();
    }
    else if(!C::CloseGroupedBoxVolumeBounds(volume.BroadHull,volume.Groups,axes,budget,complete))return false;
    if(!C::GroupedBoxDetail::SamePoint(complete.BroadHull,volume.BroadHull))return false;
    // Publish the full new volume only; the old proof, if any, cannot match it.
    // The following TryProve must seal these exact additional slabs.
    volume=MoveTemp(complete);if(closedOutput)*closedOutput=MoveTemp(replacement);changed=true;return true;
}

// Keep the SAME fixed-query chord as two endpoint OBBs in one convex
// hull. The original componentwise center box also admitted high-Y /
// low-Z combinations outside the chord. Retain every other motion
// group and the original broad envelope and add the same query shells.
static bool BuildTightFixedChord(const Analytic::FWorldQuery& q,C::FDistanceBudget& budget,
    C::FBoxVolumeGroup& chord)
{
    chord={};
    if(q.Start==q.End||!budget.Consume())return false;
    for(const auto& center:{q.Start,q.End})
    {
        auto end=q;end.Start=end.End=center;
        C::FBoxVolumeGroup endpoint;
        if(!C::NativeGroupedBoxDetail::FixedQueryGroup(end,endpoint)||!budget.Consume())return false;
        chord.Corners.Append(endpoint.Corners);
    }
    if(chord.Corners.Num()!=16)return false;
    // Bound rounding of the literal binary64 subtract, multiply and add
    // used by the chord center, plus its final center+corner addition.
    // 64 ulps of this absolute operand bound also dominates any FTZ loss.
    for(int32 d=0;d<3;++d)
    {
        auto scale=A::Interval::Exact(1.);
        for(double term:{FMath::Abs(q.Start[d]),FMath::Abs(q.End[d]),q.HalfExtent.X,q.HalfExtent.Y,q.HalfExtent.Z})
            scale=A::Add(scale,A::Interval::Exact(term));
        auto error=A::Multiply(scale,A::Interval::Exact(64.*std::numeric_limits<double>::epsilon()));
        // The literal fixed chord uses the geometric domain shell.
        // Initial overlap tolerance only suppresses an existing contact.
        // The enclosing query hull and the other motion groups keep it.
        error=A::Add(error,A::Interval::Exact(q.DomainTolerance));
        if(!A::Valid(error)||error.Lower<0)return false;
        for(auto& corner:chord.Corners)
        {
            if(!budget.Consume())return false;
            corner.Axis[d]=A::Add(corner.Axis[d],A::Interval{-error.Upper,error.Upper,true});
            if(!A::Valid(corner.Axis[d]))return false;
        }
    }
    return true;
}

static bool TightenFixedChord(const FNativeSolidStaticValues& v,C::FDistanceBudget& budget,
    C::FGroupedBoxVolumeBounds& volume,C::FClosedGroupedBoxVolumePtr* closedOutput)
{
    constexpr int32 chordIndex=8; // Four original motion groups for each of the two closed halves.
    const auto& q=v.Query;
    if(volume.Groups.Num()!=14)return false;
    C::FBoxVolumeGroup chord;
    if(!BuildTightFixedChord(q,budget,chord))return false;
    auto groups=volume.Groups;
    groups[chordIndex]=MoveTemp(chord);

    // The last two original groups are the fixed query footprint over the
    // native and reference centers on the second half of H. Their old
    // independent coordinate extrema hide that these centers lift as they
    // move. Preserve the same footprint on two smaller closed intervals.
    const float half=.5f*v.Horizon,split=half+.5f*(v.Horizon-half);
    if(!(half>0&&split>half&&split<v.Horizon))return false;
    C::FGeneralBoxMotionInput motion;
    motion.Owner=v.OwnerState;motion.Horizon=v.Horizon;motion.LocalCOM=v.LocalCOM;
    motion.LocalOffset=v.LocalOffset;motion.LocalRotation=v.LocalRotation;
    motion.PhysicalHalfExtent=v.PhysicalHalfExtent;motion.QueryHalfExtent=v.QueryHalfExtent;
    auto origin=q;origin.Start=origin.End=FVector3d::ZeroVector;
    C::FBoxVolumeGroup footprint;
    if(!C::NativeGroupedBoxDetail::FixedQueryGroup(origin,footprint))return false;
    TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> late;
    for(const auto& interval:{TPair<float,float>(half,split),TPair<float,float>(split,v.Horizon)})
    {
        C::FGeneralBoxMotionBounds bounds;
        if(!C::NativeGroupedBoxDetail::ContractedGlobalMotionPiece(motion,interval.Key,interval.Value,budget,bounds,
            C::NativeBoxAntipodal::Enabled.GetValueOnAnyThread()==1))return false;
        for(const auto& center:{bounds.NativeCenter,bounds.ReferenceCenter})
        {
            C::FBoxVolumeGroup group;
            for(const auto& corner:footprint.Corners)
            {
                C::FIntervalPoint point;
                if(!budget.Consume())return false;
                for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],corner.Axis[d]);
                if(!C::Detail::ValidPoint(point))return false;
                group.Corners.Add(point);
            }
            late.Add(MoveTemp(group));
        }
    }
    if(late.Num()!=4||!C::NativeGroupedBoxDetail::Inflate(late,q.DomainTolerance,q.InitialOverlapTolerance))return false;
    groups[12]=MoveTemp(late[0]);groups[13]=MoveTemp(late[1]);
    groups.Add(MoveTemp(late[2]));groups.Add(MoveTemp(late[3]));
    C::FGroupedBoxVolumeBounds result;C::FClosedGroupedBoxVolumePtr closed;
    if(closedOutput)
    {
        if(!C::FClosedGroupedBoxVolume::Build(volume.BroadHull,groups,volume.Axes,budget,closed))return false;
        result=closed->GetVolumeForInspection();
    }
    else if(!C::CloseGroupedBoxVolumeBounds(volume.BroadHull,groups,volume.Axes,budget,result))return false;
    if(!C::GroupedBoxDetail::SamePoint(result.BroadHull,volume.BroadHull))return false;
    volume=MoveTemp(result);
    if(closedOutput)*closedOutput=MoveTemp(closed);
    return true;
}

// Preserve the original time origin and all four roles of each indexed
// corner. Narrower time intervals avoid combining distant positions with
// the orientation of a different instant. The chord and pose are retained.
static bool TightenMotionGroups(const FNativeSolidStaticValues& v,C::FDistanceBudget& budget,
    C::FGroupedBoxVolumeBounds& volume,C::FClosedGroupedBoxVolumePtr* closedOutput)
{
    if(volume.Groups.Num()!=14&&volume.Groups.Num()!=16)return false;
    C::FGeneralBoxMotionInput motion;
    motion.Owner=v.OwnerState;motion.Horizon=v.Horizon;motion.LocalCOM=v.LocalCOM;
    motion.LocalOffset=v.LocalOffset;motion.LocalRotation=v.LocalRotation;
    motion.PhysicalHalfExtent=v.PhysicalHalfExtent;motion.QueryHalfExtent=v.QueryHalfExtent;
    const auto& q=v.Query;
    auto origin=q;origin.Start=origin.End=FVector3d::ZeroVector;
    C::FBoxVolumeGroup footprint;
    if(!C::NativeGroupedBoxDetail::FixedQueryGroup(origin,footprint))return false;
    TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> groups;
    constexpr int32 pieces=7;
    float first=0;
    const bool antipodal=C::NativeBoxAntipodal::Enabled.GetValueOnAnyThread()==1;
    for(int32 i=1;i<=pieces;++i)
    {
        const float last=i==pieces?v.Horizon:v.Horizon*(float(i)/float(pieces));
        if(last<=first||!budget.Consume())return false;
        C::FGeneralBoxMotionBounds bounds;
        if(!C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,first,last,bounds,antipodal)||
            bounds.Corners.Num()!=32)return false;
        C::FBoxVolumeGroup moving,fixed;
        const auto center=C::TimeDetail::Hull(bounds.NativeCenter,bounds.ReferenceCenter);
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;
            const auto native=C::TimeDetail::Hull(bounds.Corners[k],bounds.Corners[8+k]);
            const auto reference=C::TimeDetail::Hull(bounds.Corners[16+k],bounds.Corners[24+k]);
            moving.Corners.Add(C::TimeDetail::Hull(native,reference));
            C::FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],footprint.Corners[k].Axis[d]);
            if(!C::Detail::ValidPoint(point))return false;
            fixed.Corners.Add(point);
        }
        groups.Add(MoveTemp(moving));groups.Add(MoveTemp(fixed));
        first=last;
    }
    if(!C::NativeGroupedBoxDetail::Inflate(groups,q.DomainTolerance,q.InitialOverlapTolerance))return false;
    C::FBoxVolumeGroup chord;
    if(q.Start==q.End)chord=volume.Groups[8];
    else if(!BuildTightFixedChord(q,budget,chord))return false;
    groups.Add(MoveTemp(chord));groups.Add(volume.Groups[9]);
    // The fixed swept OBB is a zonotope. In addition to its original
    // box normals, propose the normals of its faces parallel to the chord.
    // All groups and tensor cells are projected using certified intervals.
    auto axes=volume.Axes;
    for(const auto& local:{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector})
    {
        if(!budget.Consume())return false;
        auto axis=FVector3d::CrossProduct(q.End-q.Start,q.Rotation.RotateVector(local));
        if(!CanonicalAxis(axis)||axes.Num()>=12)continue;
        if(!axes.ContainsByPredicate([&](const auto& other){return SameDirection(axis,other);}))axes.Add(axis);
    }
    C::FGroupedBoxVolumeBounds result;C::FClosedGroupedBoxVolumePtr closed;
    if(closedOutput)
    {
        if(!C::FClosedGroupedBoxVolume::Build(volume.BroadHull,groups,axes,budget,closed))return false;
        result=closed->GetVolumeForInspection();
    }
    else if(!C::CloseGroupedBoxVolumeBounds(volume.BroadHull,groups,axes,budget,result))return false;
    if(!C::GroupedBoxDetail::SamePoint(result.BroadHull,volume.BroadHull))return false;
    volume=MoveTemp(result);
    if(closedOutput)*closedOutput=MoveTemp(closed);
    return true;
}

// This separate operation proves only the continuous native/reference body
// families and the captured pose. The fixed-orientation query proxy is not
// the trajectory the prepared transport consumes. Its broad envelope still
// bounds the complete candidate/filter search and external geometry.
static bool CloseActualMotionGroups(const FNativeSolidStaticValues& v,C::FDistanceBudget& budget,
    C::FGroupedBoxVolumeBounds& volume,C::FClosedGroupedBoxVolumePtr* closedOutput,int32 pieces=15,bool refineTail=false)
{
    // A retry accepts only the output of the original fifteen-piece actual-motion
    // closure from this same call site. Its last group is the captured pose.
    if(refineTail?(volume.Groups.Num()!=16||pieces!=15):(volume.Groups.Num()!=14||pieces<1||pieces>15))return false;
    constexpr uint16 tailTicks[]={64,128,160,192,208,224,232,240,244,248,250,252,254,255,256};
    static_assert(UE_ARRAY_COUNT(tailTicks)==15);
    C::FGeneralBoxMotionInput motion;
    motion.Owner=v.OwnerState;motion.Horizon=v.Horizon;motion.LocalCOM=v.LocalCOM;
    motion.LocalOffset=v.LocalOffset;motion.LocalRotation=v.LocalRotation;
    motion.PhysicalHalfExtent=v.PhysicalHalfExtent;motion.QueryHalfExtent=v.QueryHalfExtent;
    const auto& q=v.Query;
    TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> actual;
    const bool antipodal=C::NativeBoxAntipodal::Enabled.GetValueOnAnyThread()==1;
    float first=0;
    for(int32 i=1;i<=pieces;++i)
    {
        const float fraction=refineTail?float(tailTicks[i-1])*(1.f/256.f):float(i)/float(pieces);
        const float last=i==pieces?v.Horizon:v.Horizon*fraction;
        if(last<=first||!budget.Consume())return false;
        C::FGeneralBoxMotionBounds bounds;
        if(!C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,first,last,bounds,antipodal)||
            bounds.Corners.Num()!=32)return false;
        C::FBoxVolumeGroup group;
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;
            const auto native=C::TimeDetail::Hull(bounds.Corners[k],bounds.Corners[8+k]);
            const auto reference=C::TimeDetail::Hull(bounds.Corners[16+k],bounds.Corners[24+k]);
            group.Corners.Add(C::TimeDetail::Hull(native,reference));
        }
        actual.Add(MoveTemp(group));first=last;
    }
    if(!C::NativeGroupedBoxDetail::Inflate(actual,q.DomainTolerance,q.InitialOverlapTolerance))return false;
    actual.Add(refineTail?volume.Groups.Last():volume.Groups[9]); // Authenticated captured body pose.
    C::FGroupedBoxVolumeBounds result;C::FClosedGroupedBoxVolumePtr closed;
    if(closedOutput)
    {
        if(!C::FClosedGroupedBoxVolume::Build(volume.BroadHull,actual,volume.Axes,budget,closed))return false;
        result=closed->GetVolumeForInspection();
    }
    else if(!C::CloseGroupedBoxVolumeBounds(volume.BroadHull,actual,volume.Axes,budget,result))return false;
    if(!C::GroupedBoxDetail::SamePoint(result.BroadHull,volume.BroadHull))return false;
    volume=MoveTemp(result);
    if(closedOutput)*closedOutput=MoveTemp(closed);
    return true;
}

// The ordinals only scope the proposal. The retained full partition and strict
// separation producer remain the authority for every candidate and bound.
static bool WantsRefinedActualTail(TConstArrayView<int32> candidates,const C::FGroupedBoxSeparationObservation& failure)
{
    if(RefinedActualTail.GetValueOnAnyThread()!=1||failure.Group!=14||
        FCString::Strcmp(failure.Stage,TEXT("split"))!=0)return false;
    if(candidates.Num()==2)
        return (failure.Provider==726||failure.Provider==1118)&&candidates.Contains(726)&&candidates.Contains(1118);
    return candidates.Num()==4&&failure.Provider==2431&&candidates.Contains(2431)&&candidates.Contains(2419)&&
        candidates.Contains(1734)&&candidates.Contains(2316);
}

static bool WantsCase750RefinedWholeGroups(const FNativeSolidStaticValues& source,
    const C::FReferenceBoundsCandidateSet& partition)
{
    const auto candidates=partition.GetCandidateIndices();
    const auto* args=FCommandLine::Get();
    FString ids;
    if(candidates.Num()!=4||!candidates.Contains(2431)||!candidates.Contains(2419)||
        !candidates.Contains(1734)||!candidates.Contains(2316)||!partition.GetScene()||
        partition.GetExcludedIndices().Num()+candidates.Num()!=partition.GetScene()->Num()||
        source.BodyIndex!=0||source.FrameClock.OwnerLocalFrame!=446||
        C::KinematicCenterDetail::Bits(source.Horizon)!=995783694u||
        !source.Trace.IgnoredComponents.IsEmpty()||!source.Trace.IgnoredSourceObjects.IsEmpty()||
        !FParse::Param(args,TEXT("SLStadiumNativeFastBatch"))||
        !FParse::Value(args,TEXT("SLTestIds="),ids)||ids!=TEXT("750"))return false;
    auto expected=source.OwnerState;
    expected.Location=FVector(-5827.5671515929653,-499.99916211166584,54.315787806450373);
    expected.Rotation=FQuat(0.10240123958916378,0.00018355249325102899,0.99474315838223104,-3.5928104961898784e-05);
    expected.Velocity=FVector(-1955.4559993024186,0.14218607458068341,387.93167453407318);
    expected.Acceleration=FVector(1111.8047919804069,11.955807483356594,3291.4086125968533);
    expected.AngularVelocity=FVector(-0.036594976264791204,5.4998688608203548,0.010164717940019089);
    expected.AngularAcceleration=FVector(1.2653643874712477,0.0086834975932088675,-0.43666956234573079);
    return SameKinematicBits(expected,source.OwnerState);
}

// Keep the actual rotating families, fixed footprints, literal chord and
// initial pose. Only the grouping of the original time domain changes.
static bool CloseRefinedWholeGroups(const FNativeSolidStaticValues& v,const C::FGroupedBoxVolumeBounds& original,
    C::FDistanceBudget& budget,C::FGroupedBoxVolumeBounds& volume,C::FClosedGroupedBoxVolumePtr* closedOutput)
{
    if(original.Groups.Num()!=14||!budget.Consume())return false;
    auto footprintQuery=v.Query;footprintQuery.Start=footprintQuery.End=FVector3d::ZeroVector;
    C::FBoxVolumeGroup footprint;
    if(!C::NativeGroupedBoxDetail::FixedQueryGroup(footprintQuery,footprint)||footprint.Corners.Num()!=8)return false;
    const auto motion=CurvedBoxArrivalDetail::Motion(v,v.OwnerState,v.Horizon);
    const bool antipodal=C::NativeBoxAntipodal::Enabled.GetValueOnAnyThread()==1;
    constexpr int32 pieces=6;
    TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> groups;groups.SetNum(2*pieces+2);
    float first=0.f;
    for(int32 i=0;i<pieces;++i)
    {
        const float last=i==pieces-1?v.Horizon:first+.5f*(v.Horizon-first);
        if(!(first<last)||!budget.Consume())return false;
        C::FGeneralBoxMotionBounds bounds;
        if(!C::BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(motion,first,last,bounds,antipodal)||
            bounds.Corners.Num()!=32)return false;
        const auto center=C::TimeDetail::Hull(bounds.NativeCenter,bounds.ReferenceCenter);
        if(!C::Detail::ValidPoint(center))return false;
        auto& moving=groups[i];
        auto& fixed=groups[i<2?6+i:8+i];
        for(int32 k=0;k<8;++k)
        {
            if(!budget.Consume())return false;
            const auto native=C::TimeDetail::Hull(bounds.Corners[k],bounds.Corners[8+k]);
            const auto reference=C::TimeDetail::Hull(bounds.Corners[16+k],bounds.Corners[24+k]);
            moving.Corners.Add(C::TimeDetail::Hull(native,reference));
            C::FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],footprint.Corners[k].Axis[d]);
            if(!C::Detail::ValidPoint(point))return false;
            fixed.Corners.Add(point);
        }
        first=last;
    }
    if(!SameFloat(first,v.Horizon)||
        !C::NativeGroupedBoxDetail::Inflate(groups,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance))return false;
    groups[8]=original.Groups[8]; // Already inflated literal chord.
    groups[9]=original.Groups[9]; // Already inflated captured pose.
    if(closedOutput)
    {
        if(!C::FClosedGroupedBoxVolume::Build(original.BroadHull,groups,original.Axes,budget,*closedOutput))return false;
        volume=(*closedOutput)->GetVolumeForInspection();
    }
    else if(!C::CloseGroupedBoxVolumeBounds(original.BroadHull,groups,original.Axes,budget,volume))return false;
    return C::GroupedBoxDetail::SamePoint(volume.BroadHull,original.BroadHull);
}
}

namespace NativeDirectionalCoarseDetail
{
struct FCounters
{
    std::atomic<uint64> Calls{0},Built{0},Selected{0},SourceClear{0},Clear{0},FallbackFine{0},Invalid{0},Work{0},Ticks{0},
        Candidates{0},Nodes{0},Directions{0},SupportBuilds{0},SupportHits{0},AabbClears{0},DirectionalClears{0},
        PrimitiveReuse{0},EndpointCalls{0},EndpointRejected{0};
};
static FCounters Counts;
static void Publish()
{
    const auto& c=Counts;
    UE_LOG(LogTemp,Display,TEXT("[NativeDirectionalMotionClearance] Scope=ProgramPrefixAtPreExit NoPASS=1 AlternativeCoarse=1 OriginalFineReserve=1 Calls=%llu Built=%llu Selected=%llu SourceClear=%llu Clear=%llu FallbackFine=%llu Invalid=%llu Work=%llu Ticks=%llu SecondsPerCycle=%.17g Candidates=%llu Nodes=%llu Directions=%llu SupportBuilds=%llu SupportHits=%llu AabbClears=%llu DirectionalClears=%llu PrimitiveReuse=%llu EndpointCalls=%llu EndpointRejected=%llu"),
        c.Calls.load(),c.Built.load(),c.Selected.load(),c.SourceClear.load(),c.Clear.load(),c.FallbackFine.load(),c.Invalid.load(),c.Work.load(),c.Ticks.load(),FPlatformTime::GetSecondsPerCycle64(),
        c.Candidates.load(),c.Nodes.load(),c.Directions.load(),c.SupportBuilds.load(),c.SupportHits.load(),c.AabbClears.load(),c.DirectionalClears.load(),c.PrimitiveReuse.load(),c.EndpointCalls.load(),c.EndpointRejected.load());
}
struct FRegister{FRegister(){FCoreDelegates::OnPreExit.AddStatic(&Publish);}};static FRegister Register;
static C::EDirectionalProof Try(const C::FIndexedIntervalUnionPtr& scene,const FNativeSolidStaticValues& values,
    C::FDistanceBudget& budget,C::FReferenceBoundsCandidateSetPtr& partition,C::FReferenceDirectionalMotionStrictSeparationPtr& output)
{
    output.Reset();partition.Reset();const bool observe=C::GeneralBoxMotionMemo::Observe();
    const uint64 initial=budget.Used,began=observe?FPlatformTime::Cycles64():0;C::FDirectionalMotionObservation observation;
    if(observe)++Counts.Calls;
    const auto finish=[&](C::EDirectionalProof result)
    {
        if(result!=C::EDirectionalProof::Clear)output.Reset();
        if(observe)
        {
            Counts.Ticks.fetch_add(FPlatformTime::Cycles64()-began);if(budget.Used>=initial)Counts.Work.fetch_add(budget.Used-initial);
            Counts.Nodes.fetch_add(observation.Nodes);Counts.Directions.fetch_add(observation.Directions);
            Counts.SupportBuilds.fetch_add(observation.SupportBuilds);Counts.SupportHits.fetch_add(observation.SupportHits);
            Counts.AabbClears.fetch_add(observation.AabbClears);Counts.DirectionalClears.fetch_add(observation.DirectionalClears);
            if(result==C::EDirectionalProof::Clear)++Counts.SourceClear;else if(result==C::EDirectionalProof::Invalid)++Counts.Invalid;else ++Counts.FallbackFine;
        }
        return result;
    };
    C::InstantDirectionalSupportDetail::FCompletePacket packet;C::FIntervalPoint hull;
    if(!budget.Consume())return finish(C::EDirectionalProof::Work);
    if(!C::InstantMotionClearanceDetail::BuildCompleteVolume(values,hull,&packet,&budget))return finish(C::EDirectionalProof::Unavailable);
    if(observe)++Counts.Built;
    if(!C::FReferenceBoundsCandidateSet::Select(scene,hull,budget,partition))return finish(C::EDirectionalProof::Work);
    if(observe){++Counts.Selected;Counts.Candidates.fetch_add(partition->GetCandidateIndices().Num());}
    const auto support=[&](const FVector3d& axis,C::FDistanceBudget& current,C::Interval& slab){return packet.TryProject(axis,current,slab);};
    return finish(C::FReferenceDirectionalMotionStrictSeparation::TryProve(partition,support,budget,output,&observation));
}
static bool ContainsEndpoint(const C::FReferenceDirectionalMotionStrictSeparationPtr& proof,
    Analytic::FWorldQuery pose,const FVector3d& physical,const FVector3d& query)
{
    const bool observe=C::GeneralBoxMotionMemo::Observe();if(observe)++Counts.EndpointCalls;
    const auto refuse=[&](){if(observe)++Counts.EndpointRejected;return false;};
    if(!proof)return refuse();
    for(const auto& extent:{physical,query})
    {
        pose.HalfExtent=extent;C::FBoxVolumeGroup actual;
        if(!C::NativeGroupedBoxDetail::FixedQueryGroup(pose,actual)||!proof->ContainsCompleteEndpointGroup(actual.Corners))return refuse();
    }
    return true;
}
}

namespace NativePredictionCoarseDetail
{
// This is the complete PREDICTION volume. Unlike the actual-primitive producer,
// it also contains the original World chord, captured pose and both fixed-query
// footprints. Four motion + chord + pose + two footprints = eight groups.
static bool BuildVolume(const FNativeSolidStaticValues& v,C::FDistanceBudget& budget,
    C::FGroupedBoxVolumeBounds& output,const TCHAR** stage=nullptr,C::FClosedGroupedBoxVolumePtr* closedOutput=nullptr)
{
    if(C::CorrelatedNativeBoxGroups::TryBuild(v,false,budget,output,stage,closedOutput))return true;
    output={};const auto note=[&](const TCHAR* value){if(stage)*stage=value;};
    note(TEXT("coarse_native_sweep_input"));C::FIntervalPoint originalHull;
    if(!budget.Consume()||!C::BuildGeneralNativeBoxSweepVolumeMemo(v,originalHull))return false;
    C::FGeneralBoxMotionInput input;input.Owner=v.OwnerState;input.Horizon=v.Horizon;
    input.LocalCOM=v.LocalCOM;input.LocalOffset=v.LocalOffset;input.LocalRotation=v.LocalRotation;
    input.PhysicalHalfExtent=v.PhysicalHalfExtent;input.QueryHalfExtent=v.QueryHalfExtent;
    C::FGeneralBoxMotionBounds whole;note(TEXT("coarse_whole_motion"));
    if(!budget.Consume()||!C::BuildGeneralBoxMotionBoundsMemo(input,whole,0))return false;
    TArray<C::FBoxVolumeGroup,TInlineAllocator<16>> groups;
    if(!C::NativeGroupedBoxDetail::MotionGroups(whole,groups)||groups.Num()!=4)return false;
    note(TEXT("coarse_fixed_query_chord"));C::FBoxVolumeGroup chord;
    if(!C::NativeGroupedBoxDetail::FixedQueryGroup(v.Query,chord))return false;
    groups.Add(MoveTemp(chord));
    auto pose=v.Query;pose.Start=pose.End=v.BodyState.Location;pose.HalfExtent=v.PhysicalHalfExtent;
    note(TEXT("coarse_physical_pose"));C::FBoxVolumeGroup physicalPose;
    if(!C::NativeGroupedBoxDetail::FixedQueryGroup(pose,physicalPose))return false;
    groups.Add(MoveTemp(physicalPose));
    auto footprintQuery=v.Query;footprintQuery.Start=footprintQuery.End=FVector3d::ZeroVector;
    note(TEXT("coarse_footprint"));C::FBoxVolumeGroup footprint;
    if(!C::NativeGroupedBoxDetail::FixedQueryGroup(footprintQuery,footprint))return false;
    for(const auto& center:{whole.NativeCenter,whole.ReferenceCenter})
    {
        C::FBoxVolumeGroup group;
        for(const auto& corner:footprint.Corners)
        {
            C::FIntervalPoint point;
            for(int32 d=0;d<3;++d)point.Axis[d]=::AnalyticCertified::Add(center.Axis[d],corner.Axis[d]);
            if(!C::Detail::ValidPoint(point))return false;group.Corners.Add(point);
        }
        groups.Add(MoveTemp(group));
    }
    note(TEXT("coarse_close_grouped_volume"));
    return groups.Num()==8&&C::NativeGroupedBoxDetail::Close(originalHull,groups,v.Query.Rotation,
        v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,budget,output,closedOutput);
}
enum class EAttempt:uint8 {KeepFine,Clear,Invalid};
template<typename FExternalGuard>
static EAttempt Attempt(const C::FIndexedIntervalUnionPtr& scene,const FNativeSolidStaticValues& v,
    C::FDistanceBudget& budget,int32 allowance,FExternalGuard&& externalGuard,
    C::FGroupedBoxVolumeBounds& volume,C::FReferenceBoundsCandidateSetPtr& partition,
    C::FReferenceBoxVolumeStrictSeparationPtr& proof,C::FClosedGroupedBoxVolumePtr* closedOutput=nullptr,C::FReferenceDirectionalMotionStrictSeparationPtr* directionalOutput=nullptr)
{
    volume={};partition.Reset();proof.Reset();if(closedOutput)closedOutput->Reset();if(directionalOutput)directionalOutput->Reset();
    if(!allowance)return EAttempt::KeepFine;
    if(!scene||allowance!=int64(scene->Num())+2048||budget.ConsumptionFloor<0||
        int64(budget.Remaining)-budget.ConsumptionFloor<=allowance)return EAttempt::Invalid;
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    // The original fine allowance is protected in THIS same account. Failed
    // coarse work stays spent; neither a retry nor a floor release credits it.
    C::FScopedDistanceBudgetFloor reserve(budget,budget.Remaining-allowance);
    if(!reserve.IsValid())return EAttempt::Invalid;
    if(directionalOutput&&C::DirectionalMotionControl::RuntimeEnabled())
    {
        // Alternative coarse producer, not a second allocation/prepass. The
        // same untouched fine reserve remains below this existing floor.
        const auto result=NativeDirectionalCoarseDetail::Try(scene,v,budget,partition,*directionalOutput);
        const auto completed=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(completed.ControlFingerprint()!=numeric.ControlFingerprint()||result==C::EDirectionalProof::Invalid)return EAttempt::Invalid;
        if(result==C::EDirectionalProof::Clear&&*directionalOutput&&
            (*directionalOutput)->Matches(scene,(*directionalOutput)->GetBroadHullForInspection())&&externalGuard((*directionalOutput)->GetBroadHullForInspection()))
        {if(C::GeneralBoxMotionMemo::Observe())++NativeDirectionalCoarseDetail::Counts.Clear;return EAttempt::Clear;}
        if(result==C::EDirectionalProof::Clear&&C::GeneralBoxMotionMemo::Observe())++NativeDirectionalCoarseDetail::Counts.FallbackFine;
        directionalOutput->Reset();partition.Reset();return EAttempt::KeepFine;
    }
    bool built=false;
    {
        NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionCoarseVolume,budget);
        built=BuildVolume(v,budget,volume,nullptr,closedOutput);phase.Result(built);
    }
    bool selected=false;
    if(built)
    {
        NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionCoarseSelect,budget);
        selected=C::FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,budget,partition);
        phase.Result(selected);if(partition)phase.Work(0,partition->GetCandidateIndices().Num());
    }
    if(selected&&NativePredictionObliqueAxes::Enabled.GetValueOnAnyThread()==1)
    {
        NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionExtraAxes,budget);
        bool changed=false;
        const FVector3d target=.5*FVector3d(v.Query.Start)+.5*FVector3d(v.Query.End);
        selected=NativePredictionObliqueAxes::Augment(partition,target,budget,volume,changed,closedOutput);
        phase.Result(selected);
    }
    bool clear=false;
    if(selected)
    {
        NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionCoarseProve,budget);
        C::FGroupedBoxSeparationObservation observed;
        clear=(closedOutput?C::FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,*closedOutput,budget,proof,phase.Active?&observed:nullptr):
            C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,budget,proof,phase.Active?&observed:nullptr))&&
            proof&&proof->Matches(scene,volume)&&externalGuard(volume.BroadHull);
        phase.Result(clear);phase.Work(observed.VisitedCells,partition->GetCandidateIndices().Num());
    }
    const auto completed=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(completed.ControlFingerprint()!=numeric.ControlFingerprint())return EAttempt::Invalid;
    return clear?EAttempt::Clear:EAttempt::KeepFine;
}
}

bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr geometry,const FNativeSolidStaticValues& v,
    C::FDistanceBudget& budget,FNativeCanonicalBoxSweepClearanceValuesPtr& out,FString& reason)
{
    return BuildNativeCanonicalBoxSweepClearanceValues(MoveTemp(geometry),v,budget,out,reason,FPreparedWorldSnapshotPtr{},0);
}
bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr geometry,const FNativeSolidStaticValues& v,
    C::FDistanceBudget& budget,FNativeCanonicalBoxSweepClearanceValuesPtr& out,FString& reason,
    FPreparedWorldSnapshotPtr snapshot,int32 coarseWorkAllowance)
{
    return BuildNativeCanonicalBoxSweepClearanceValues(MoveTemp(geometry),v,budget,out,reason,
        MoveTemp(snapshot),coarseWorkAllowance,nullptr);
}
bool BuildNativeCanonicalBoxSweepClearanceValues(FPreparedGeometrySelectionPtr geometry,const FNativeSolidStaticValues& v,
    C::FDistanceBudget& budget,FNativeCanonicalBoxSweepClearanceValuesPtr& out,FString& reason,
    FPreparedWorldSnapshotPtr snapshot,int32 coarseWorkAllowance,const FNativeSolidStaticRequest* liveRemainderRequest)
{
    out.Reset();reason.Reset();const auto fail=[&](const TCHAR* why){reason=why;return false;};
    const auto predictionNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        v.Shape!=ENativeSolidQueryShape::Box)return fail(TEXT("Canonical reference Box geometry/shape unavailable"));
    int32 fineWork=0;
    if(coarseWorkAllowance)
    {
        const int64 required=int64(geometry->GetSelectedUnion()->Num())+2048;
        if(!snapshot||snapshot->GetGeometrySelection()!=geometry||coarseWorkAllowance<0||required>MAX_int32||
            coarseWorkAllowance!=required||budget.ConsumptionFloor<0||
            int64(budget.Remaining)-budget.ConsumptionFloor<=coarseWorkAllowance)
            return fail(TEXT("Invalid explicit coarse prediction allocation or snapshot"));
        fineWork=budget.Remaining-budget.ConsumptionFloor-coarseWorkAllowance;
    }
    TOptional<C::FScopedDistanceBudgetFloor> fineLimit;
    const auto startFine=[&]()
    {
        if(!coarseWorkAllowance||fineLimit.IsSet())return true;
        fineLimit.Emplace(budget,budget.Remaining-fineWork);
        return fineLimit->IsValid();
    };
    bool coarseClear=false;
    C::FFloorNativeSolidModel fixed;C::FIntervalPoint envelope;C::FYawFloorBoxSweepBounds yaw;bool general=false;
    C::FGroupedBoxVolumeBounds groupedVolume;C::FReferenceBoxVolumeStrictSeparationPtr groupedSeparation;
    C::FClosedGroupedBoxVolumePtr closedVolume;C::FReferenceDirectionalMotionStrictSeparationPtr directionalSeparation;
    auto* closedSink=C::ClosedGroupedBoxControl::RuntimeEnabled()?&closedVolume:nullptr;
    C::FReferenceBoundsCandidateSetPtr partition;C::FGroupedBoxSeparationObservation observation;
    const TCHAR* volumeStage=TEXT("not_entered");
    const auto groupedFail=[&](const TCHAR* step)
    {
        static const IConsoleVariable* timing=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
        if(timing&&timing->GetInt()!=0)
        {
            NativePrimitiveCoarseProfile::PublishSnapshot(TEXT("BeforePredictionRefusal"));
            ::StadiumProbeProfile70::PublishSnapshot(TEXT("BeforePredictionRefusal"));
            ::StadiumSphereProfile70::PublishSnapshot(TEXT("BeforePredictionRefusal"));
        }
        reason=FString::Printf(TEXT("%s VolumeStage=%s SeparationStage=%s Used=%llu Remaining=%d Floor=%d Groups=%d Axes=%d Slabs=%d Candidates=%d Excluded=%d Scene=%d Provider=%d Group=%d Axis=%d Degrees=%d,%d UV=[%.17g,%.17g]x[%.17g,%.17g] Cells=%llu Stack=%d ActiveMask=%u TestedGroups=%llu InheritedSkips=%llu"),
            step,volumeStage,observation.Stage,budget.Used,budget.Remaining,budget.ConsumptionFloor,
            groupedVolume.Groups.Num(),groupedVolume.Axes.Num(),groupedVolume.Slabs.Num(),
            partition?partition->GetCandidateIndices().Num():-1,partition?partition->GetExcludedIndices().Num():-1,
            partition?partition->GetScene()->Num():-1,observation.Provider,observation.Group,observation.Axis,
            observation.DegreeU,observation.DegreeV,observation.MinU,observation.MaxU,observation.MinV,observation.MaxV,
            observation.VisitedCells,observation.StackDepth,observation.ActiveGroupMask,observation.TestedGroups,observation.InheritedGroupSkips);return false;
    };
    // Passive subphase attribution for prediction, separate from actual primitive validation.
    const auto buildPredictionVolume=[&]()
    {
        NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionVolume,budget);
        const bool result=C::CorrelatedNativeBoxGroups::BuildFineOrOriginal(v,budget,groupedVolume,&volumeStage,closedSink);
        phase.Result(result);return result;
    };
    const auto selectPrediction=[&]()
    {
        NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionSelect,budget);
        const bool result=C::FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),envelope,budget,partition);
        phase.Result(result);phase.Work(0,partition?partition->GetCandidateIndices().Num():0);return result;
    };
    const auto proveGroupedPrediction=[&]()
    {
        NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionProve,budget);
        const bool result=closedSink?C::FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,closedVolume,budget,groupedSeparation,&observation):
            C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,groupedVolume,budget,groupedSeparation,&observation);
        phase.Result(result);phase.Work(observation.VisitedCells,partition?partition->GetCandidateIndices().Num():0);return result;
    };
    if(v.Frozen)
    {
        // The real frozen path returns without integration. Model a local
        // stationary value only; retain the ORIGINAL frozen capture below.
        const auto zero=[](const FVector& p){return C::FloorNativeSweepDetail::ZeroVector(p);};
        if(v.Prescribed||!zero(v.OwnerState.Velocity)||!zero(v.OwnerState.Acceleration)||
            !zero(v.OwnerState.AngularVelocity)||!zero(v.OwnerState.AngularAcceleration)||
            !zero(v.BodyState.Velocity)||!zero(v.BodyState.Acceleration)||
            !zero(v.BodyState.AngularVelocity)||!zero(v.BodyState.AngularAcceleration)||
            !C::FloorNativeSweepDetail::SameVector(v.Query.Start,v.BodyState.Location)||
            !C::FloorNativeSweepDetail::SameVector(v.Query.End,v.BodyState.Location))
            return fail(TEXT("Frozen Box is not an exact stationary native path"));
        auto stationary=v;stationary.Frozen=false;
        if(!C::BuildNativeSolidSweepVolume(stationary,fixed,envelope))
            return fail(TEXT("Frozen Box fixed stationary volume unavailable"));
    }
    else if(!C::BuildNativeSolidSweepVolume(v,fixed,envelope))
    {
        if(C::BuildYawFloorBoxSweepBounds(v,yaw))envelope=yaw.QueryEnvelope;
        else
        {
            general=true;
            if(coarseWorkAllowance)
            {
                const auto attempt=NativePredictionCoarseDetail::Attempt(geometry->GetSelectedUnion(),v,budget,coarseWorkAllowance,
                    [&](const C::FIntervalPoint& broadHull)
                    {return !snapshot->GetExternalLandscape()||snapshot->GetExternalLandscape()->StrictlySeparated(v.Query,broadHull);},
                    groupedVolume,partition,groupedSeparation,closedSink,&directionalSeparation);
                if(attempt==NativePredictionCoarseDetail::EAttempt::Invalid)return groupedFail(TEXT("coarse_prediction_context"));
                coarseClear=attempt==NativePredictionCoarseDetail::EAttempt::Clear;
            }
            // A failed/overlapping coarse attempt authorizes nothing. The
            // literal original fourteen-group producer retains all fine work.
            if(!coarseClear&&(!startFine()||!buildPredictionVolume()))return groupedFail(TEXT("grouped_volume"));
            envelope=directionalSeparation?directionalSeparation->GetBroadHullForInspection():groupedVolume.BroadHull;
        }
    }
    C::FReferenceBoundsStrictSeparationPtr separation;
    if(!coarseClear)
    {
        if(!startFine())return groupedFail(TEXT("prediction_fine_reservation"));
        if(!selectPrediction())
            return general?groupedFail(TEXT("candidate_partition")):fail(TEXT("Canonical reference Box complete volume partition unavailable"));
        if(!general)
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionSimpleProve,budget);
            const bool proved=C::FReferenceBoundsStrictSeparation::TryProve(partition,budget,separation)&&
                separation&&separation->Matches(geometry->GetSelectedUnion(),envelope);
            phase.Result(proved);
            if(!proved)
            {
                if(v.Frozen||v.Prescribed||NativePredictionObliqueAxes::Enabled.GetValueOnAnyThread()!=1)
                    return fail(TEXT("Canonical Box prediction is not strictly clear of EVERY retained provider"));
                // An axis-aligned envelope may cover a curved patch even
                // for a fixed orientation. Prove the original motion groups
                // with the same account before returning any clear result.
                separation.Reset();
                if(!buildPredictionVolume())return groupedFail(TEXT("fixed_grouped_volume"));
                envelope=groupedVolume.BroadHull;
                if(!selectPrediction())return groupedFail(TEXT("fixed_grouped_partition"));
                general=true;
            }
        }
        if(general)
        {
            if(CVarCurvedBoxFirstArrival.GetValueOnAnyThread()==1&&snapshot&&snapshot->GetGeometrySelection()==geometry&&
                partition->GetCandidateIndices().Num()==1)
            {
                FNativeCurvedBoxArrivalValuesPtr proof;
                if(FNativeCurvedBoxArrivalValues::Build(snapshot,v,partition,budget,10240,proof,
                    INDEX_NONE,false,false,false,nullptr,liveRemainderRequest)&&proof&&proof->Matches(snapshot,v))
                {
                    TSharedPtr<FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe> values(new FNativeCanonicalBoxSweepClearanceValues);
                    values->Geometry=geometry;values->Captured=v;values->Envelope=proof->GetEnvelopeForInspection();
                    values->GeneralTransportRequired=true;values->CurvedArrival=MoveTemp(proof);
                    out=MoveTemp(values);return true;
                }

                // The arrival producer cannot infer clearance from a missing
                // inside witness. Prove the complete original groups instead.
                const uint64 began=budget.Used;
                FNativeCurvedBoxWholeClearValuesPtr whole;
                if(FNativeCurvedBoxWholeClearValues::Build(snapshot,v,partition,groupedVolume,budget,whole)&&
                    whole&&whole->Matches(geometry,v,envelope))
                {
                    const auto after=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
                    if(after.ControlFingerprint()!=predictionNumeric.ControlFingerprint())return false;
                    TSharedPtr<FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe> values(new FNativeCanonicalBoxSweepClearanceValues);
                    values->Geometry=geometry;values->Captured=v;values->Envelope=envelope;
                    values->GeneralTransportRequired=true;values->CurvedActualMotionOnly=whole->IsActualMotionOnly();
                    values->CurvedWholeClear=MoveTemp(whole);
                    values->NumericFingerprint=after.ControlFingerprint();
                    static const IConsoleVariable* timing=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.AnalyticWorld.PhaseTiming"));
                    if(timing&&timing->GetInt()!=0)
                        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxWholeClearClosed] Frame=%u Provider=%d Groups=%d Work=%llu ActualMotionOnly=%d"),
                            v.FrameClock.OwnerLocalFrame,partition->GetCandidateIndices()[0],groupedVolume.Groups.Num(),budget.Used-began,
                            int32(values->CurvedActualMotionOnly));
                    out=MoveTemp(values);return true;
                }
            }
            if(CVarCurvedBoxNativeAdjacentBandIgnoredTailClear.GetValueOnAnyThread()==1&&
                CVarCurvedBoxNativeAdjacentBandContinuation.GetValueOnAnyThread()==1&&
                snapshot&&snapshot->GetGeometrySelection()==geometry&&
                v.FrameClock.OwnerLocalFrame==50&&v.Trace.IgnoredComponents.Num()==1&&
                partition->GetCandidateIndices().Num()==2&&
                CurvedBoxArrivalDetail::IsCase976ContinuationFixture())
            {
                constexpr int32 tailWork=4864;
                const int32 available=budget.Remaining-budget.ConsumptionFloor;
                const uint64 began=budget.Used;
                C::FScopedDistanceBudgetFloor tailFloor(budget,budget.Remaining-FMath::Min(available,tailWork));
                FNativeCurvedBoxWholeClearValuesPtr whole;
                if(available>0&&tailFloor.IsValid()&&FNativeCurvedBoxWholeClearValues::BuildNativeAdjacentBandWholeClear(
                    snapshot,v,partition,budget,whole)&&whole&&!whole->IsActualMotionOnly()&&whole->Matches(geometry,v,envelope))
                {
                    const auto after=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
                    if(after.ControlFingerprint()!=predictionNumeric.ControlFingerprint())return false;
                    TSharedPtr<FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe> values(new FNativeCanonicalBoxSweepClearanceValues);
                    values->Geometry=geometry;values->Captured=v;values->Envelope=envelope;
                    values->GeneralTransportRequired=true;values->CurvedWholeClear=MoveTemp(whole);
                    values->NumericFingerprint=after.ControlFingerprint();
                    UE_LOG(LogTemp,Display,TEXT("[CurvedBoxNativeAdjacentBandIgnoredTailClear] Frame=%u H=%.17g Candidates=2 Cells=16 Work=%llu"),
                        v.FrameClock.OwnerLocalFrame,double(v.Horizon),budget.Used-began);
                    out=MoveTemp(values);return true;
                }
            }
            bool refinedWhole=false;
            constexpr int32 refinedWork=8192;
            if(NativePredictionObliqueAxes::RefinedWholeGroups.GetValueOnAnyThread()==1&&snapshot&&
                snapshot->GetGeometrySelection()==geometry&&!v.Frozen&&!v.Prescribed&&groupedVolume.Groups.Num()==14&&
                (partition->GetCandidateIndices().Num()==2||
                 NativePredictionObliqueAxes::WantsCase750RefinedWholeGroups(v,*partition))&&
                budget.Remaining-budget.ConsumptionFloor>=refinedWork)
            {
                const uint64 began=budget.Used;
                C::FScopedDistanceBudgetFloor optional(budget,budget.Remaining-refinedWork);
                C::FGroupedBoxVolumeBounds refined;
                C::FClosedGroupedBoxVolumePtr refinedClosed;
                C::FReferenceBoxVolumeStrictSeparationPtr refinedProof;
                C::FGroupedBoxSeparationObservation observed;
                if(optional.IsValid()&&NativePredictionObliqueAxes::CloseRefinedWholeGroups(
                    v,groupedVolume,budget,refined,closedSink?&refinedClosed:nullptr))
                {
                    // Recheck every candidate in the original complete
                    // partition. No graph-only or filtered claim escapes.
                    refinedWhole=closedSink?
                        C::FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,refinedClosed,budget,refinedProof,&observed):
                        C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,refined,budget,refinedProof,&observed);
                    if(refinedWhole)
                    {
                        groupedVolume=MoveTemp(refined);
                        closedVolume=MoveTemp(refinedClosed);
                        groupedSeparation=MoveTemp(refinedProof);
                    }
                }
                UE_LOG(LogTemp,Display,TEXT("[NativePredictionRefinedWholeGroups] Frame=%u Candidates=%d Proved=%d Provider=%d Stage=%s Cells=%llu Work=%llu"),
                    v.FrameClock.OwnerLocalFrame,partition->GetCandidateIndices().Num(),int32(refinedWhole),observed.Provider,observed.Stage,observed.VisitedCells,budget.Used-began);
            }
            bool actualFirst=false;
            constexpr int32 earlyWork=1536;
            if(NativePredictionObliqueAxes::ActualFirst.GetValueOnAnyThread()==1&&
                NativePredictionObliqueAxes::Enabled.GetValueOnAnyThread()==1&&
                snapshot&&snapshot->GetGeometrySelection()==geometry&&!v.Frozen&&!v.Prescribed&&
                groupedVolume.Groups.Num()==14&&partition->GetCandidateIndices().Num()==1&&
                budget.Remaining-budget.ConsumptionFloor>=earlyWork)
            {
                NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::PredictionActualFirst,budget);
                C::FScopedDistanceBudgetFloor optional(budget,budget.Remaining-earlyWork);
                auto actualVolume=groupedVolume;
                auto actualClosed=closedVolume;
                auto* actualSink=closedSink?&actualClosed:nullptr;
                C::FGroupedBoxSeparationObservation actualObservation;
                C::FReferenceBoxVolumeStrictSeparationPtr actualProof;
                bool added=false;
                const FVector3d target=.5*FVector3d(v.Query.Start)+.5*FVector3d(v.Query.End);
                // Proposed axes affect only the work order. Reclose every
                // slab on the exact original motion and prove the complete
                // already selected partition before publishing anything.
                if(optional.IsValid()&&
                    NativePredictionObliqueAxes::Augment(partition,target,budget,actualVolume,added,actualSink)&&
                    NativePredictionObliqueAxes::CloseActualMotionGroups(v,budget,actualVolume,actualSink,4))
                {
                    actualFirst=actualSink?
                        C::FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,actualClosed,budget,actualProof,&actualObservation):
                        C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,actualVolume,budget,actualProof,&actualObservation);
                    if(actualFirst)
                    {
                        groupedVolume=MoveTemp(actualVolume);
                        closedVolume=MoveTemp(actualClosed);
                        groupedSeparation=MoveTemp(actualProof);
                    }
                }
                phase.Result(actualFirst);
                phase.Work(actualObservation.VisitedCells,1);
            }
            if(!refinedWhole&&!actualFirst&&!proveGroupedPrediction())
            {
                // The directional coarse path does not run the oblique
                // proposal. On a fine spatial split refusal, keep the paid
                // work and try the candidate-derived axes on this same
                // complete partition and all of its original motion groups.
                if(NativePredictionObliqueAxes::Enabled.GetValueOnAnyThread()!=1||
                    FCString::Strcmp(observation.Stage,TEXT("split"))!=0)
                    return groupedFail(TEXT("grouped_strict_separation"));
                const FVector3d target=.5*FVector3d(v.Query.Start)+.5*FVector3d(v.Query.End);
                bool changed=false;
                if(!NativePredictionObliqueAxes::Augment(partition,target,budget,groupedVolume,changed,closedSink,&observation)||
                    !changed)return groupedFail(TEXT("grouped_strict_separation_oblique"));
                bool proved=false;
                if(snapshot&&snapshot->GetGeometrySelection()==geometry&&!v.Frozen&&!v.Prescribed)
                {
                    // The real prepared transport consumes the rotating
                    // trajectory. Prove it before spending another search on
                    // the fixed-orientation proxy from the captured query.
                    if(NativePredictionObliqueAxes::CloseActualMotionGroups(v,budget,groupedVolume,closedSink))
                    {
                        proved=proveGroupedPrediction();
                        constexpr int32 tailWork=6144;
                        if(!proved&&NativePredictionObliqueAxes::WantsRefinedActualTail(partition->GetCandidateIndices(),observation)&&
                            budget.Remaining-budget.ConsumptionFloor>=tailWork)
                        {
                            const uint64 began=budget.Used;
                            C::FScopedDistanceBudgetFloor optional(budget,budget.Remaining-tailWork);
                            auto refined=groupedVolume;
                            C::FClosedGroupedBoxVolumePtr refinedClosed;
                            auto* refinedSink=closedSink?&refinedClosed:nullptr;
                            C::FReferenceBoxVolumeStrictSeparationPtr refinedProof;
                            C::FGroupedBoxSeparationObservation refinedObservation;
                            bool added=false;
                            if(optional.IsValid()&&NativePredictionObliqueAxes::CloseActualMotionGroups(v,budget,refined,refinedSink,15,true)&&
                                NativePredictionObliqueAxes::Augment(partition,target,budget,refined,added,refinedSink,&observation))
                            {
                                proved=refinedSink?
                                    C::FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,refinedClosed,budget,refinedProof,&refinedObservation):
                                    C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,refined,budget,refinedProof,&refinedObservation);
                                if(proved)
                                {
                                    groupedVolume=MoveTemp(refined);
                                    closedVolume=MoveTemp(refinedClosed);
                                    groupedSeparation=MoveTemp(refinedProof);
                                }
                            }
                            UE_LOG(LogTemp,Display,TEXT("[NativePredictionRefinedActualTail] Frame=%u Candidates=%d Proved=%d AddedAxis=%d Provider=%d Group=%d Stage=%s Work=%llu"),
                                v.FrameClock.OwnerLocalFrame,partition->GetCandidateIndices().Num(),int32(proved),int32(added),
                                refinedObservation.Provider,refinedObservation.Group,refinedObservation.Stage,budget.Used-began);
                        }
                    }
                }
                else
                {
                    proved=proveGroupedPrediction();
                    if(!proved&&observation.Group==8&&FCString::Strcmp(observation.Stage,TEXT("split"))==0&&
                        NativePredictionObliqueAxes::TightenFixedChord(v,budget,groupedVolume,closedSink))
                        proved=proveGroupedPrediction();
                    if(!proved&&observation.Group>=0&&observation.Group<8&&
                        FCString::Strcmp(observation.Stage,TEXT("split"))==0&&
                        NativePredictionObliqueAxes::TightenMotionGroups(v,budget,groupedVolume,closedSink))
                        proved=proveGroupedPrediction();
                }
                if(!proved)
                {
                    const int32 available=budget.Remaining-budget.ConsumptionFloor;
                    constexpr int32 maxTwoBandWork=14336;
                    const bool twoBands=CVarCurvedBoxTwoBandPrefix.GetValueOnAnyThread()==1;
                    const bool adjacentBandPair=CVarCurvedBoxNativeAdjacentBandTiming.GetValueOnAnyThread()==1&&
                        partition->GetCandidateIndices().Contains(607)&&partition->GetCandidateIndices().Contains(726);
                    const bool nativeAdjacentBand=adjacentBandPair&&v.FrameClock.OwnerLocalFrame==50;
                    const bool step51NativeBand=adjacentBandPair&&CVarCurvedBoxNativeAdjacentBandStep51.GetValueOnAnyThread()==1&&
                        (CurvedBoxArrivalDetail::IsCase976Step51ProvedTimingRequest(v)||
                         (liveRemainderRequest&&CurvedBoxArrivalDetail::IsCase976Step51RemainderCandidate(v)));
                    if((twoBands||nativeAdjacentBand||step51NativeBand)&&CVarCurvedBoxFirstArrival.GetValueOnAnyThread()==1&&
                        snapshot&&snapshot->GetGeometrySelection()==geometry&&partition->GetCandidateIndices().Num()==2&&
                        FCString::Strcmp(observation.Stage,TEXT("split"))==0&&available>=partition->GetScene()->Num()+1024)
                    {
                        C::FScopedDistanceBudgetFloor limit(budget,budget.Remaining-FMath::Min(available,maxTwoBandWork));
                        FNativeCurvedBoxArrivalValuesPtr arrival;
                        if(limit.IsValid()&&
                            ((twoBands&&FNativeCurvedBoxArrivalValues::Build(snapshot,v,partition,budget,10240,
                                arrival,INDEX_NONE,true,false,false,nullptr,liveRemainderRequest))||
                             (nativeAdjacentBand&&FNativeCurvedBoxArrivalValues::Build(snapshot,v,partition,budget,10240,
                                arrival,INDEX_NONE,false,true))||
                             ((nativeAdjacentBand||step51NativeBand)&&CVarCurvedBoxNativeAdjacentBandContinuation.GetValueOnAnyThread()==1&&
                                CurvedBoxArrivalDetail::IsCase976ContinuationFixture()&&
                                FNativeCurvedBoxArrivalValues::Build(snapshot,v,partition,budget,10240,
                                    arrival,INDEX_NONE,false,true,true,nullptr,liveRemainderRequest)))&&arrival&&arrival->Matches(snapshot,v))
                        {
                            const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
                            if(numeric.ControlFingerprint()!=predictionNumeric.ControlFingerprint())return false;
                            TSharedPtr<FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe> values(new FNativeCanonicalBoxSweepClearanceValues);
                            values->Geometry=geometry;values->Captured=v;values->Envelope=arrival->GetEnvelopeForInspection();
                            values->GeneralTransportRequired=true;values->NumericFingerprint=numeric.ControlFingerprint();
                            values->CurvedArrival=MoveTemp(arrival);out=MoveTemp(values);return true;
                        }
                    }
                    if(FCString::Strcmp(observation.Stage,TEXT("split"))==0)
                        CurvedBoxArrivalDetail::TraceMirrorArcTiming(snapshot,v,partition,budget);
                    return groupedFail(TEXT("grouped_strict_separation_oblique"));
                }
            }
            if(!groupedSeparation||!groupedSeparation->Matches(geometry->GetSelectedUnion(),groupedVolume))
                return groupedFail(TEXT("grouped_match"));
        }
    }
    TSharedPtr<FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe> value(new FNativeCanonicalBoxSweepClearanceValues);
    value->GeneralTransportRequired=general;value->Geometry=MoveTemp(geometry);value->Captured=v;value->Envelope=envelope;value->Separation=MoveTemp(separation);
    value->GroupedVolume=MoveTemp(groupedVolume);value->GroupedSeparation=MoveTemp(groupedSeparation);value->DirectionalSeparation=MoveTemp(directionalSeparation);
    const auto afterPrediction=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(afterPrediction.ControlFingerprint()!=predictionNumeric.ControlFingerprint())return false;
    value->NumericFingerprint=afterPrediction.ControlFingerprint();out=MoveTemp(value);return true;
}
bool MatchesNativeCanonicalBoxSweepClearanceValues(const FNativeSolidStaticValues& v,const FPreparedGeometrySelectionPtr& geometry,
    const FNativeCanonicalBoxSweepClearanceValuesPtr& value,const C::FIntervalPoint& envelope)
{
    if(!value||!geometry||value->Geometry!=geometry||!geometry->HasValidAliases()||
        geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||v.Shape!=ENativeSolidQueryShape::Box||
        !SameNativeSolidStaticValues(v,value->Captured)||!C::GroupedBoxDetail::SamePoint(value->Envelope,envelope))return false;
    if(value->CurvedWholeClear)
        return !value->CurvedActualMotionOnly&&!value->CurvedWholeClear->IsActualMotionOnly()&&
            value->GeneralTransportRequired&&!value->CurvedArrival&&!value->Separation&&!value->GroupedSeparation&&
            !value->DirectionalSeparation&&value->GroupedVolume.Groups.IsEmpty()&&value->CurvedWholeClear->Matches(geometry,v,envelope);
    if(value->DirectionalSeparation)
        return value->GeneralTransportRequired&&!value->Separation&&!value->GroupedSeparation&&value->GroupedVolume.Groups.IsEmpty()&&
            value->DirectionalSeparation->Matches(geometry->GetSelectedUnion(),envelope);
    if(value->GeneralTransportRequired)
        return !value->Separation&&value->GroupedSeparation&&
            C::GroupedBoxDetail::SamePoint(value->Envelope,value->GroupedVolume.BroadHull)&&
            value->GroupedSeparation->Matches(geometry->GetSelectedUnion(),value->GroupedVolume);
    return !value->GroupedSeparation&&value->Separation&&
        value->Separation->Matches(geometry->GetSelectedUnion(),value->Envelope)&&
        value->Separation->Matches(geometry->GetSelectedUnion(),envelope);
}
bool MatchesNativeCurvedBoxActualMotionClearValues(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& v,
    const FNativeCanonicalBoxSweepClearanceValuesPtr& value,const C::FIntervalPoint& envelope)
{
    return value&&snapshot&&value->IsCurvedActualMotionClear()&&value->CurvedWholeClear->IsActualMotionOnly()&&
        value->GeneralTransportRequired&&!value->CurvedArrival&&!value->Separation&&!value->GroupedSeparation&&
        !value->DirectionalSeparation&&value->GroupedVolume.Groups.IsEmpty()&&value->Geometry==snapshot->GetGeometrySelection()&&
        SameNativeSolidStaticValues(v,value->Captured)&&C::GroupedBoxDetail::SamePoint(value->Envelope,envelope)&&
        value->CurvedWholeClear->MatchesSnapshot(snapshot,v,envelope);
}
namespace PredictionPrimitiveReuseDetail
{
// Capture identity is stronger than equality of the 35 model coefficients:
// current source query/configuration and the captured body also remain exact.
static bool SameInputs(const FNativeSolidStaticValues& source,const FKinematicState& actual,float seconds,
    const FNativeSolidStaticValues& captured,uint64 predictionNumeric,uint64 currentNumeric)
{
    return predictionNumeric==currentNumeric&&source.Shape==ENativeSolidQueryShape::Box&&!source.Frozen&&!source.Prescribed&&
        SameNativeSolidStaticValues(source,captured)&&SameKinematicBits(actual,captured.OwnerState)&&SameFloat(seconds,captured.Horizon);
}
}
static bool MatchesStep51LiveRemainderPhysicalProjection(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,const CurvedBoxArrivalDetail::FProduct& product,
    const FNativeCurvedBoxArrivalValues* arrival)
{
    return arrival&&arrival->MatchesLiveRemainderPhysicalProjection(snapshot,query,product);
}

#if !UE_BUILD_SHIPPING
static bool MatchesCase205PhysicalProjection(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,const CurvedBoxArrivalDetail::FProduct& product,
    const FNativeCurvedBoxArrivalValues* arrival)
{
    return arrival&&arrival->MatchesCase205PhysicalProjection(snapshot,query,product);
}
#endif

bool ReadNativeCurvedBoxArrivalValues(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const FNativeCanonicalBoxSweepClearanceValuesPtr& values,float& seconds,Analytic::FWorldHit& hit)
{
    if(!values||!values->CurvedArrival||!values->CurvedArrival->Matches(snapshot,source)||
        !values->GeneralTransportRequired||values->Geometry!=snapshot->GetGeometrySelection()||
        !SameNativeSolidStaticValues(values->Captured,source)||
        !C::GroupedBoxDetail::SamePoint(values->Envelope,values->CurvedArrival->GetEnvelopeForInspection()))return false;
    seconds=values->CurvedArrival->MaximumAdvanceSeconds();
    hit=values->CurvedArrival->GetHitForInspection();return true;
}

static TAutoConsoleVariable<int32> CVarCurvedBoxWheelSupportObstructionPreflight(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxWheelSupportObstructionPreflight"),0,
    TEXT("Reject an unpublished wheel-support translation only on a proved physical obstruction. Default off."));

#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarCurvedBoxCase585Owner88WheelSupportObstruction(
    TEXT("p.IAmSpeed.AnalyticWorld.CurvedBoxCase585Owner88WheelSupportObstruction"),0,
    TEXT("Default OFF. Reuse the positive physical wheel-support obstruction preflight in case 585 owner88 TwoBand."));
#endif

#if !UE_BUILD_SHIPPING
bool FStepScope::MatchesCase205CoupledHitboxOverlap(const Analytic::FWorldQuery& query)
{
    namespace C=Analytic::Certified;
    if(CVarCase205Q112PhysicalProjection.GetValueOnAnyThread()!=1||
        C::Case205ReferenceTiming::Enabled.GetValueOnAnyThread()!=1)return false;
    const auto* scope=StepScope;
    const auto* phase=OwnerProbePhase;
    FStepIdentity step;
    if(!scope||!scope->Active||!scope->Snapshot||!phase||!phase->State||phase->State->Returned||
        phase->State->Phase!=ENativeOwnerProbePhase::PostIntegrate||phase->State->Snapshot!=scope->Snapshot||
        WheelProbeScope||NativeOwnerPoseMutation||!scope->OrdinaryIntegration||
        !ReadCurrentStep(step)||step.Frame!=219||step.Iteration!=2||step.QuerySerial!=2||!phase->Check())return false;
    const auto* args=FCommandLine::Get();
    FString cases;
    if(!FParse::Param(args,TEXT("SLStadiumNativeFastBatch"))||!FParse::Param(args,TEXT("NullRHI"))||
        !FParse::Param(args,TEXT("unattended"))||!FParse::Value(args,TEXT("SLTestIds="),cases)||cases!=TEXT("205"))
        return false;
    const auto& snapshot=scope->Snapshot;
    const auto& tx=*scope->OrdinaryIntegration;
    if(tx.Snapshot!=snapshot||!tx.HasActualSelection||!tx.SelectorJoined||tx.Restoring||tx.Closed||
        !tx.Owners.IsValidIndex(tx.ActiveIndex))return false;
    const auto& row=tx.Owners[tx.ActiveIndex];
    if(row.Owner!=phase->State->Owner||row.OpenPoseSetter||
        row.PoseAssignmentsBegun!=row.PoseAssignmentsCompleted||row.SyncVersion!=row.PoseVersion||
        row.GeneralOpen||!row.GeneralReturned||!row.GeneralCount||row.GeneralNext!=row.GeneralCount||
        row.GeneralBindings.Num()!=1||row.GeneralProofs.Num()!=1||!row.GeneralProofs[0])return false;
    const auto& binding=row.GeneralBindings[0];
    const auto& request=binding.Request;
    const auto& prediction=binding.Prediction;
    if(!binding.HasExactlyOneKind()||!request||request->GetSnapshot()!=snapshot||
        !prediction||!prediction->IsCurvedArrivalPrefix())return false;
    auto revoked=request->GetStep();
    if(revoked.QueryEpoch==MAX_uint64)return false;
    ++revoked.QueryEpoch;
    if(revoked.QuerySerial!=step.QuerySerial||!OrdinaryWorldDetail::Epoch(revoked,step))return false;
    const auto& source=request->GetValues();
    const auto* box=Cast<UBoxSubBody>(source.Body);
    if(!box||box->GetSubBodyType()!=USSubBody::ESubBodyType::Hitbox||source.Owner!=row.Owner||
        source.OwnerStableId!=3||source.FrameClock.OwnerLocalFrame!=220||source.BodyIndex!=0||
        !source.Trace.IgnoredComponents.IsEmpty()||!source.Trace.IgnoredSourceObjects.IsEmpty()||
        !GeneralBoxTransportDetail::Configuration(source))return false;
    const auto actual=box->GetKinematicState();
    const auto filters=snapshot->GetFilters();
    if(!SameVector(query.Start,actual.Location)||!SameVector(query.End,actual.Location)||
        !SameQuat(query.Rotation,actual.Rotation)||!SameVector(query.HalfExtent,source.QueryHalfExtent)||
        !filters||!filters->CoversPhysicalFilter(true,step.InputGeneration,source.Owner,source.Body,query))return false;
    const auto& arrival=prediction->GetCurvedArrivalForInspection();
    float seconds=0.f;
    Analytic::FWorldHit selected;
    return arrival&&arrival->HasLiveCase205Reference(snapshot)&&
        ReadNativeCurvedBoxArrivalValues(snapshot,source,prediction,seconds,selected)&&
        selected.bHit&&selected.SurfaceId==10310501707698981028ull&&
        C::KinematicCenterDetail::Bits(seconds)==0x2f343419u&&SameFloat(seconds,tx.Seconds);
}

bool FStepScope::TryProveCase585CoupledHitboxOverlapClear(const FPreparedWorldSnapshotPtr& snapshot,
    const Analytic::FWorldQuery& query,Analytic::Certified::FDistanceBudget& budget)
{
    namespace C=Analytic::Certified;
    namespace G=C::BoxGraphFirstArrival;
    namespace A=::AnalyticCertified;
    auto* scope=StepScope;const auto* phase=OwnerProbePhase;
    FStepIdentity step;
    if(!snapshot||!scope||!scope->Active||scope->Snapshot!=snapshot||!phase||!phase->State||
        phase->State->Returned||phase->State->Phase!=ENativeOwnerProbePhase::PostIntegrate||
        phase->State->Snapshot!=snapshot||WheelProbeScope||NativeOwnerPoseMutation||!scope->OrdinaryIntegration||
        !ReadCurrentStep(step)||step.Frame!=87||step.Iteration<=1||step.QuerySerial<=1||
        !CurvedBoxArrivalDetail::IsCase585Owner88Fixture()||!phase->Check())return false;
    const auto& tx=*scope->OrdinaryIntegration;
    if(tx.Snapshot!=snapshot||!tx.HasActualSelection||!tx.SelectorJoined||tx.Restoring||tx.Closed||
        !tx.Owners.IsValidIndex(tx.ActiveIndex))return false;
    const int32 index=tx.ActiveIndex;
    const auto& row=tx.Owners[index];
    if(row.Owner!=phase->State->Owner||row.OpenPoseSetter||
        row.PoseAssignmentsBegun!=row.PoseAssignmentsCompleted||row.SyncVersion!=row.PoseVersion||
        row.GeneralOpen||!row.GeneralReturned||!row.GeneralCount||row.GeneralNext!=row.GeneralCount||
        row.GeneralBindings.Num()!=1||row.GeneralProofs.Num()!=1||!row.GeneralProofs[0])return false;
    const auto& binding=row.GeneralBindings[0];
    const auto request=binding.Request;const auto prediction=binding.Prediction;
    if(!binding.HasExactlyOneKind()||!request||request->GetSnapshot()!=snapshot||
        !prediction||!prediction->IsCurvedArrivalPrefix())return false;
    auto revoked=request->GetStep();
    if(revoked.QueryEpoch==MAX_uint64)return false;
    ++revoked.QueryEpoch;
    if(revoked.QuerySerial!=step.QuerySerial||!OrdinaryWorldDetail::Epoch(revoked,step))return false;
    const auto& source=request->GetValues();
    const auto* box=Cast<UBoxSubBody>(source.Body);
    if(!box||box->GetSubBodyType()!=USSubBody::ESubBodyType::Hitbox||source.Owner!=row.Owner||
        source.OwnerStableId!=3||source.FrameClock.OwnerLocalFrame!=88||source.BodyIndex!=0||
        !source.Trace.IgnoredComponents.IsEmpty()||!source.Trace.IgnoredSourceObjects.IsEmpty()||
        !GeneralBoxTransportDetail::Configuration(source))return false;
    const auto actual=box->GetKinematicState();
    const auto filters=snapshot->GetFilters();
    if(query.Shape!=Analytic::EQueryShape::Box||!query.bObjectQuery||
        query.ObjectTypes!=(uint64(1)<<ECC_WorldStatic)||query.TraceChannel!=0||
        query.BlockingObjectTypes!=MAX_uint64||!query.bApplyCollisionFilter||!query.bAuthorityOnly||
        query.bExcludeAuthorityEligible||!query.bIncludeCompactPatches||query.bIncludeTriangles||
        query.RequiredSourceId||query.RequiredSurfaceId||query.RequiredCanonicalGroupId||
        query.ReferenceNormal!=FVector3d::ZeroVector||query.MinimumReferenceNormalDot!=-1.||
        query.DomainTolerance!=1.e-6||query.InitialOverlapTolerance!=.05||
        !SameVector(query.Start,actual.Location)||!SameVector(query.End,actual.Location)||
        !SameQuat(query.Rotation,actual.Rotation)||!SameVector(query.HalfExtent,source.QueryHalfExtent)||
        !filters||!filters->CoversPhysicalFilter(true,step.InputGeneration,source.Owner,source.Body,query))return false;
    float seconds=0;Analytic::FWorldHit selected;
    if(!ReadNativeCurvedBoxArrivalValues(snapshot,source,prediction,seconds,selected)||
        !selected.bHit||selected.SurfaceId!=12602828015134877967ull||!SameFloat(seconds,tx.Seconds))return false;

    // The deepest-overlap operation keeps both original shells for its
    // candidate census. The 0.05 tolerance rejects shallow penetration; it
    // is not a positive-depth expansion of the literal oriented Box.
    const auto geometry=snapshot->GetGeometrySelection();
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !geometry->GetReference()||geometry->GetReference()->GetExternalPacketIdentity()!=
            TEXT("090d41158d13f1127827b720ff467a51e415668aa205209a67ed8afcb6b10dc0")||
        !geometry->GetNominalScene()||!geometry->GetSelectedUnion())return false;
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
    const uint64 began=budget.Used;
    C::FIntervalPoint envelope;
    C::FReferenceBoundsCandidateSetPtr partition;
    const auto scene=geometry->GetSelectedUnion();
    constexpr int32 provider=726;
    if(!budget.Consume()||!C::BuildCanonicalFixedBoxOverlapEnvelope(query,envelope)||
        !C::FReferenceBoundsCandidateSet::Select(scene,envelope,budget,partition)||!partition||
        partition->GetScene()!=scene||partition->GetCandidateIndices().Num()!=1||
        partition->GetCandidateIndices()[0]!=provider||partition->GetExcludedIndices().Num()+1!=scene->Num())
        return false;
    const auto owners=geometry->GetNominalScene()->GetOwners();
    if(!budget.Consume()||owners.Num()!=scene->Num()||!owners.IsValidIndex(provider)||
        !C::CanonicalAxialPlaneDetail::Policy(query,owners[provider])||
        owners[provider].SurfaceId!=selected.SurfaceId)return false;
    const auto* product=CurvedBoxArrivalDetail::FindAuthenticatedGraphProduct(geometry,provider,budget);
    C::FBoxVolumeGroup fixed;
    G::FAccount work{budget,budget.Used,1536};
    G::FGraph graph;
    if(!product||!work.Pay()||!C::NativeGroupedBoxDetail::FixedQueryGroup(query,fixed)||
        fixed.Corners.Num()!=8||!G::FGraph::Make(*product,work,graph))return false;
    G::FGraph::FPositionMemo memo(graph,true);
    const C::Interval shell{-query.DomainTolerance,query.DomainTolerance,true};
    for(auto corner:fixed.Corners)
    {
        for(auto& axis:corner.Axis)axis=A::Add(axis,shell);
        if(!C::Detail::ValidPoint(corner)||!graph.Sign(corner,true,work,memo))return false;
    }
    // The signed graph is convex with its proven constant left extension,
    // so the original rigid Box is strictly above it between all vertices.
    // Only the canonical result is decided here; the existing caller
    // independently evaluates and reduces the exact external Landscape.
    FStepIdentity now;
    if(StepScope!=scope||OwnerProbePhase!=phase||WheelProbeScope||NativeOwnerPoseMutation||
        scope->OrdinaryIntegration.Get()!=&tx||tx.ActiveIndex!=index||scope->Snapshot!=snapshot||
        !phase->Check()||!ReadCurrentStep(now)||now.QuerySerial!=step.QuerySerial||
        !OrdinaryWorldDetail::Epoch(step,now)||row.OpenPoseSetter||row.SyncVersion!=row.PoseVersion||
        row.GeneralBindings.Num()!=1||row.GeneralBindings[0].Request!=request||
        row.GeneralBindings[0].Prediction!=prediction||!SameKinematicBits(actual,box->GetKinematicState())||
        Analytic::RequireCurrentAnalyticNumericContext(
            Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()!=numeric)return false;
    UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCase585CoupledHitboxOverlap] Frame=%llu Serial=%llu Ordinal=%llu Provider=726 Candidates=1 Scene=%d OriginalShells=1 CanonicalDepth=0 Work=%llu"),
        step.Frame,step.QuerySerial,phase->State->Ordinal,scene->Num(),budget.Used-began);
    return true;
}
#endif

bool FStepScope::TryRejectNativeCurvedBoxWheelSupportTranslation(const ISpeedComponent& owner,const FVector& candidateCOM)
{
    if(CVarCurvedBoxWheelSupportObstructionPreflight.GetValueOnAnyThread()!=1||!Requested())return false;
    auto* scope=StepScope;const auto* phase=OwnerProbePhase;
    if(!scope||!scope->Active||!phase||!phase->State||phase->State->Returned||WheelProbeScope||
        NativeOwnerPoseMutation||phase->State->Phase!=ENativeOwnerProbePhase::PostIntegrate||
        phase->State->Owner!=&owner||phase->State->Snapshot!=scope->Snapshot||!phase->Check()||
        !scope->OrdinaryIntegration||!scope->Identity.Provider)return false;
    auto& tx=*scope->OrdinaryIntegration;
    if(tx.Restoring||tx.Closed||!tx.Owners.IsValidIndex(tx.ActiveIndex))return false;
    const int32 index=tx.ActiveIndex;
    const auto& row=tx.Owners[index];
    const auto before=owner.GetKinematicState();auto candidate=before;candidate.Location=candidateCOM;
    FStepIdentity step;
    if(row.Owner!=&owner||row.OpenPoseSetter||row.PoseAssignmentsBegun!=row.PoseAssignmentsCompleted||
        row.PoseVersion!=0||row.SyncVersion!=0||!row.LatestPoseCorrections.IsEmpty()||
        row.GeneralOpen||!row.GeneralReturned||!row.GeneralCount||row.GeneralNext!=row.GeneralCount||
        row.GeneralBindings.Num()!=1||row.GeneralProofs.Num()!=1||!row.GeneralProofs[0]||
        !FiniteState(before)||!FiniteState(candidate)||!GeneralBoxTransportDetail::Pose(before,row.GeneralExpected)||
        SameVector(before.Location,candidateCOM)||!ReadCurrentStep(step))return false;
    const auto snapshot=scope->Snapshot;const auto request=row.GeneralBindings[0].Request;
    const auto prediction=row.GeneralBindings[0].Prediction;
    if(!row.GeneralBindings[0].HasExactlyOneKind()||!request||request->GetSnapshot()!=snapshot||
        !prediction||!prediction->IsCurvedArrivalPrefix())return false;
    const auto& source=request->GetValues();
    const auto service=scope->Identity.Provider->GetPreparedQueryService();
    float seconds=0;Analytic::FWorldHit hit;
    if(source.Owner!=&owner||!source.Trace.IgnoredComponents.IsEmpty()||!source.Trace.IgnoredSourceObjects.IsEmpty()||
        !service||!service->MatchesPreparedSnapshot(snapshot)||!GeneralBoxTransportDetail::Configuration(source)||
        !ReadNativeCurvedBoxArrivalValues(snapshot,source,prediction,seconds,hit)||!hit.bHit)return false;
    const auto& arrival=prediction->GetCurvedArrivalForInspection();
    Analytic::Certified::FDistanceBudget budget{128};
    bool case585Owner88=false;
#if !UE_BUILD_SHIPPING
    if(CVarCurvedBoxCase585Owner88WheelSupportObstruction.GetValueOnAnyThread()==1&&
        source.FrameClock.OwnerLocalFrame==88&&source.OwnerStableId==3&&source.BodyIndex==0&&
        step.Frame==87&&step.QuerySerial==1&&step.Iteration==1)
    {
        const auto& captured=request->GetStep();
        auto revoked=captured;
        if(revoked.QueryEpoch!=MAX_uint64)
        {
            // BeginOrdinaryIntegration requires exactly one sweep invalidation.
            ++revoked.QueryEpoch;
            case585Owner88=captured.QuerySerial==1&&OrdinaryWorldDetail::Epoch(revoked,step)&&
                source.Horizon==1.f/300.f&&SameFloat(captured.Delta,source.Horizon)&&
                SameFloat(captured.Remaining,source.Horizon)&&SameFloat(captured.TimePassed,0.f)&&
                CurvedBoxArrivalDetail::IsCase585Owner88Fixture()&&snapshot->GetFilters()&&
                snapshot->GetFilters()->CoversPhysicalFilter(true,captured.InputGeneration,source.Owner,source.Body,source.Query);
        }
    }
#endif
    bool provedTwoBand=false;
    if(!arrival||!arrival->ProveObstructedWheelSupportTranslation(snapshot,source,before,candidate,budget,
        case585Owner88,provedTwoBand))return false;

    // There is no setter ticket or grant here. Rejoin the same active operands
    // before telling the caller to abandon the still-unpublished transaction.
    FStepIdentity now;
    if(StepScope!=scope||OwnerProbePhase!=phase||WheelProbeScope||NativeOwnerPoseMutation||
        scope->Snapshot!=snapshot||scope->OrdinaryIntegration.Get()!=&tx||tx.ActiveIndex!=index||
        !phase->Check()||!ReadCurrentStep(now)||!OrdinaryWorldDetail::Epoch(step,now)||step.QuerySerial!=now.QuerySerial||
        row.OpenPoseSetter||row.PoseAssignmentsBegun!=row.PoseAssignmentsCompleted||
        row.PoseVersion!=0||row.SyncVersion!=0||row.GeneralBindings.Num()!=1||
        row.GeneralBindings[0].Request!=request||row.GeneralBindings[0].Prediction!=prediction||
        !SameKinematicBits(before,owner.GetKinematicState())||!GeneralBoxTransportDetail::Configuration(source))return false;
#if !UE_BUILD_SHIPPING
    if(provedTwoBand)
    {
        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCase585Owner88WheelSupportObstruction] Frame=%llu Serial=%llu Winner=726 Companion=607 CompleteChord=1 Work=%llu"),
            step.Frame,step.QuerySerial,budget.Used);
    }
#endif
    if(Analytic::Certified::BoxGraphFirstArrival::CVarOrdinaryCurvedBoxTrace.GetValueOnAnyThread()==1)
        UE_LOG(LogTemp,Display,TEXT("[CurvedBoxWheelSupportObstructionPreflight] Frame=%llu Serial=%llu Body=%d Work=%llu"),
            step.Frame,step.QuerySerial,source.BodyIndex,budget.Used);
    return true;
}

bool BuildCase202Serial3CurveArrival(const FNativeSolidStaticRequest& request,
    const FNativeCase202AtlasObservationPtr& atlas,FNativeCanonicalBoxSweepClearanceValuesPtr& out)
{
    out.Reset();
#if PLATFORM_WINDOWS && defined(_M_X64) && !UE_BUILD_SHIPPING && defined(SPEED_CASE202_SERIAL3_REVIEWER_KEY)
    if(!atlas||!atlas->PositivePrefixPartition)return false;
    const auto snapshot=request.GetSnapshot();const auto& source=request.GetValues();const auto& step=request.GetStep();
    if(!snapshot||step.Frame!=126||step.Iteration!=3||step.QuerySerial!=3||step.QueryEpoch!=6||
        !RevalidateNativeSolidCCD(request))return false;
    const FString manifest=FPlatformMisc::GetEnvironmentVariable(TEXT("SL_CASE202_SERIAL3_MODEL_SEAL"));
    const auto integrate=&FKinematicState::Integrate;
    const void* integrateAddress=nullptr;
    static_assert(sizeof(integrate)==sizeof(integrateAddress),"review only the x64 single-inheritance member layout");
    FMemory::Memcpy(&integrateAddress,&integrate,sizeof(integrateAddress));
    FCase202Serial3CurveInput serial3;
    serial3.Request=&request;serial3.Atlas=atlas;
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(
        Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
    auto seal=Case202Serial3::ReviewedLoadedImageVerifier::TrySeal(*manifest,
        Case202Serial3::TrustedReviewerPublicKey,sizeof(Case202Serial3::TrustedReviewerPublicKey),
        integrateAddress,reinterpret_cast<const void*>(&::Speed::SBox::IntegrateRotation),
        reinterpret_cast<const void*>(&C::BoxGraphFirstArrival::FProof::TryBuildSerial3),numeric,serial3.Live);
    if(!seal)
    {
        UE_LOG(LogTemp,Display,TEXT("[Case202Serial3Model] Stage=unattested"));
        return false;
    }
    serial3.Seal=MakeShareable(seal.release());
    UE_LOG(LogTemp,Display,TEXT("[Case202Serial3Model] Stage=verified Serial=%llu Numeric=%016llx"),step.QuerySerial,numeric);
    C::FDistanceBudget budget{10240};
    FNativeCurvedBoxArrivalValuesPtr curve;
    if(!FNativeCurvedBoxArrivalValues::Build(snapshot,source,atlas->PositivePrefixPartition,budget,10240,
        curve,1931,false,false,false,&serial3)||!curve||!curve->Matches(snapshot,source))return false;
    TSharedPtr<FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe> values(new FNativeCanonicalBoxSweepClearanceValues);
    values->Geometry=snapshot->GetGeometrySelection();values->Captured=source;values->Envelope=curve->GetEnvelopeForInspection();
    values->GeneralTransportRequired=true;values->Case202Serial3=true;values->NumericFingerprint=numeric;
    values->CurvedArrival=MoveTemp(curve);out=MoveTemp(values);
    return true;
#else
    UE_LOG(LogTemp,Display,TEXT("[Case202Serial3Model] Stage=review_key_unprovisioned"));
    return false;
#endif
}

bool BuildNativeEstablishedFloorCurveArrival(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const C::FNativeGeneralBoxFloorDescriptorBounds& prelude,C::FDistanceBudget& budget,FNativeCanonicalBoxSweepClearanceValuesPtr& out)
{
    out.Reset();const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!snapshot||!prelude.Complete||prelude.PhysicalClear||!prelude.ObservationFinitePartition||
        prelude.ObservationCurvedProvider==INDEX_NONE||prelude.NumericFingerprint!=numeric.ControlFingerprint()||
        !SameNativeSolidStaticValues(prelude.Captured,source)||prelude.SideNormal!=FVector::UpVector)return false;
    FNativeCurvedBoxArrivalValuesPtr curve;
    if(!FNativeCurvedBoxArrivalValues::Build(snapshot,source,prelude.ObservationFinitePartition,budget,10240,
        curve,prelude.ObservationCurvedProvider)||!curve||!curve->Matches(snapshot,source))return false;
    TSharedPtr<FNativeCanonicalBoxSweepClearanceValues,ESPMode::ThreadSafe> values(new FNativeCanonicalBoxSweepClearanceValues);
    values->Geometry=snapshot->GetGeometrySelection();values->Captured=source;values->Envelope=curve->GetEnvelopeForInspection();
    values->GeneralTransportRequired=true;values->NumericFingerprint=numeric.ControlFingerprint();values->CurvedArrival=MoveTemp(curve);
    out=MoveTemp(values);return true;
}

bool TryBuildNativeBoxPrimitiveFromPrediction(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
    const FKinematicState& actual,float seconds,const FNativeCanonicalBoxSweepClearanceValuesPtr& prediction,
    C::FDistanceBudget& budget,FNativeBoxPrimitiveGeometryValuesPtr& out)
{
    out.Reset();const auto before=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    // One bounded identity/proof-copy group. It is paid on misses too, on the
    // caller's same account; no geometry or advance authority is transferred.
    if(!budget.Consume())return false;
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    if(prediction&&prediction->CurvedArrival)
    {
        C::FIntervalPoint envelope;
        C::FGeneralBoxPhysicalFloorClearBoundsPtr rebasedFloor;
        TOptional<FNativeBoxRebasedRoof202> rebasedRoof;
        if(!prediction->CurvedArrival->CoversActualPrimitive(snapshot,source,actual,seconds,budget,envelope,&rebasedFloor,&rebasedRoof))return false;
        TSharedPtr<FNativeBoxPrimitiveGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPrimitiveGeometryValues);
        value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Actual=actual;value->Seconds=seconds;
        value->Envelope=envelope;value->CurvedArrival=prediction->CurvedArrival;value->PhysicalFloorClear=MoveTemp(rebasedFloor);
        value->RebasedRoof202=MoveTemp(rebasedRoof);
        value->Provenance=ENativeBoxPrimitiveGeometryProvenance::CurvedArrivalPrefix;
        if(!value->Matches(snapshot,source,actual,seconds))return false;
        out=MoveTemp(value);return true;
    }
    if(prediction&&prediction->CurvedWholeClear)
    {
        const bool actualMotionOnly=prediction->IsCurvedActualMotionClear();
        if(actualMotionOnly)
        {
            if(!MatchesNativeCurvedBoxActualMotionClearValues(snapshot,source,prediction,prediction->Envelope)||
                !prediction->CurvedWholeClear->CoversActualPrimitive(snapshot,source,actual,seconds,prediction->Envelope,budget))return false;
        }
        else if(!MatchesNativeCanonicalBoxSweepClearanceValues(source,geometry,prediction,prediction->Envelope)||
            !prediction->CurvedWholeClear->MatchesActual(snapshot,source,actual,seconds,prediction->Envelope))return false;
        TSharedPtr<FNativeBoxPrimitiveGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPrimitiveGeometryValues);
        value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Actual=actual;value->Seconds=seconds;
        value->Envelope=prediction->Envelope;value->CurvedWholeClear=prediction->CurvedWholeClear;
        value->Provenance=actualMotionOnly?ENativeBoxPrimitiveGeometryProvenance::CurvedGraphActualMotionClear:
            ENativeBoxPrimitiveGeometryProvenance::CurvedGraphWholeClear;
        if(!value->Matches(snapshot,source,actual,seconds))return false;
        out=MoveTemp(value);return true;
    }
    if(!prediction||!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !PredictionPrimitiveReuseDetail::SameInputs(source,actual,seconds,prediction->Captured,
            prediction->NumericFingerprint,before.ControlFingerprint())||!prediction->GeneralTransportRequired||
        (!prediction->GroupedSeparation&&!prediction->DirectionalSeparation)||!MatchesNativeCanonicalBoxSweepClearanceValues(source,geometry,prediction,prediction->Envelope))return false;
    // Both prediction producers contain the complete original motion: the
    // eight-group coarse route contains its four whole motion groups, and the
    // fourteen-group fine route contains the same two original motion pieces.
    // With identical input bits this is already a complete actual-motion proof.
    // Fresh external geometry may reject the larger retained envelope; miss
    // falls back to the original narrower primitive proof on the same account.
    if(snapshot->GetExternalLandscape()&&!snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,prediction->Envelope))return false;
    TSharedPtr<FNativeBoxPrimitiveGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPrimitiveGeometryValues);
    value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Actual=actual;value->Seconds=seconds;
    value->Envelope=prediction->Envelope;value->GroupedVolume=prediction->GroupedVolume;value->GroupedSeparation=prediction->GroupedSeparation;value->DirectionalSeparation=prediction->DirectionalSeparation;
    const auto after=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.ControlFingerprint()!=before.ControlFingerprint())return false;
    if(!value->Matches(snapshot,source,actual,seconds))return false;
    if(value->DirectionalSeparation&&C::GeneralBoxMotionMemo::Observe())++NativeDirectionalCoarseDetail::Counts.PrimitiveReuse;
    out=MoveTemp(value);return true;
}
// Pure retained geometry. This does not manufacture a World query or revive
// its revoked sweep epoch. The actual integration scope binds these values.
bool BuildNativeBoxPrimitiveGeometryValues(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
    const FKinematicState& actual,float seconds,C::FDistanceBudget& budget,FNativeBoxPrimitiveGeometryValuesPtr& out,
    int32 coarseWorkAllowance)
{
    out.Reset();const auto before=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    // Validate the shared account even when the optional coarse route is OFF.
    // In particular, never subtract a negative floor in signed int32.
    if(budget.ConsumptionFloor<0||budget.Remaining<budget.ConsumptionFloor)return false;
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed)return false;
    const auto scene=geometry->GetSelectedUnion();const int64 allowance=int64(scene->Num())+1024;
    if(coarseWorkAllowance<0||(coarseWorkAllowance&&
        (allowance>MAX_int32||coarseWorkAllowance!=allowance||budget.ConsumptionFloor<0||
         int64(budget.Remaining)-budget.ConsumptionFloor<=allowance)))return false;
    const int32 fineWork=budget.Remaining-budget.ConsumptionFloor-coarseWorkAllowance;
    C::FGeneralBoxMotionInput input;input.Owner=actual;input.Horizon=seconds;
    input.LocalCOM=source.LocalCOM;input.LocalOffset=source.LocalOffset;input.LocalRotation=source.LocalRotation;
    input.PhysicalHalfExtent=source.PhysicalHalfExtent;input.QueryHalfExtent=source.QueryHalfExtent;
    const auto rotation=actual.Rotation*source.LocalRotation;
    C::FGroupedBoxVolumeBounds groupedVolume;C::FGeneralBoxMotionBounds whole;
    C::FReferenceBoundsCandidateSetPtr partition;C::FReferenceBoxVolumeStrictSeparationPtr separation;
    bool coarseClear=false;
    if(coarseWorkAllowance)
    {
        // Same account throughout. This floor protects ALL original fine work.
        C::FScopedDistanceBudgetFloor reserve(budget,budget.Remaining-coarseWorkAllowance);
        if(!reserve.IsValid())return false;
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::WholeModel,budget);
            const bool built=budget.Consume()&&C::BuildGeneralBoxMotionBoundsMemo(input,whole,1);
            phase.Result(built);if(!built)return false; // Original fine model also required this.
        }
        bool coarseBuilt=false;
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::CoarseVolume,budget);
            coarseBuilt=C::NativeGroupedBoxDetail::CoarsePrimitiveFromWhole(whole,rotation,
                source.Query.DomainTolerance,source.Query.InitialOverlapTolerance,budget,groupedVolume);
            phase.Result(coarseBuilt);
        }
        bool selected=false;
        if(coarseBuilt)
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::CoarseSelect,budget);
            selected=C::FReferenceBoundsCandidateSet::Select(scene,groupedVolume.BroadHull,budget,partition);
            phase.Result(selected);if(partition)phase.Work(0,partition->GetCandidateIndices().Num());
        }
        if(selected)
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::CoarseProve,budget);
            C::FGroupedBoxSeparationObservation work;
            coarseClear=C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,groupedVolume,budget,separation,phase.Active?&work:nullptr)&&
                separation&&separation->Matches(scene,groupedVolume)&&
                (!snapshot->GetExternalLandscape()||snapshot->GetExternalLandscape()->StrictlySeparated(source.Query,groupedVolume.BroadHull));
            // A coarser canonical hull may overlap external Landscape where
            // the original fine hull did not. In that case retain the fine
            // route; the consumer still repeats its own fresh Landscape guard.
            phase.Result(coarseClear);phase.Work(work.VisitedCells,partition->GetCandidateIndices().Num());
        }
    }
    const auto afterCoarse=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(afterCoarse.ControlFingerprint()!=before.ControlFingerprint())return false;
    if(!coarseClear)
    {
        // Unspent coarse allowance is not transferred to the fine producer.
        // Releasing either floor never increases Remaining or decreases Used.
        C::FScopedDistanceBudgetFloor fineLimit(budget,budget.Remaining-fineWork);
        if(!fineLimit.IsValid())return false;
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::FineVolume,budget);
            const bool built=coarseWorkAllowance?
                C::NativeGroupedBoxDetail::RefinedPrimitiveFromWhole(input,whole,rotation,
                    source.Query.DomainTolerance,source.Query.InitialOverlapTolerance,budget,groupedVolume):
                C::BuildGeneralBoxPrimitiveGroupedVolumeMemo(input,rotation,
                    source.Query.DomainTolerance,source.Query.InitialOverlapTolerance,budget,groupedVolume);
            phase.Result(built);if(!built)return false;
        }
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::FineSelect,budget);
            const bool reused=partition&&partition->GetScene()==scene&&C::GroupedBoxDetail::SamePoint(partition->GetQueryHull(),groupedVolume.BroadHull);
            const bool selected=reused||C::FReferenceBoundsCandidateSet::Select(scene,groupedVolume.BroadHull,budget,partition);
            phase.Result(selected);if(partition)phase.Work(0,partition->GetCandidateIndices().Num(),reused);
            if(!selected)return false;
        }
        {
            NativePrimitiveCoarseProfile::FScope phase(NativePrimitiveCoarseProfile::FineProve,budget);
            C::FGroupedBoxSeparationObservation work;
            const bool proved=C::FReferenceBoxVolumeStrictSeparation::TryProve(partition,groupedVolume,budget,separation,phase.Active?&work:nullptr)&&
                separation&&separation->Matches(scene,groupedVolume);
            phase.Result(proved);phase.Work(work.VisitedCells,partition->GetCandidateIndices().Num());if(!proved)return false;
        }
    }
    TSharedPtr<FNativeBoxPrimitiveGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPrimitiveGeometryValues);
    value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Actual=actual;value->Seconds=seconds;
    value->Envelope=groupedVolume.BroadHull;value->GroupedVolume=MoveTemp(groupedVolume);value->GroupedSeparation=MoveTemp(separation);
    const auto after=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.ControlFingerprint()!=before.ControlFingerprint())return false;
    out=MoveTemp(value);return true;
}

namespace ReleasedFloorPrimitiveDetail
{
bool Within(const C::FIntervalPoint& outer,const C::FIntervalPoint& inner)
{
    if(!C::Detail::ValidPoint(outer)||!C::Detail::ValidPoint(inner))return false;
    for(int32 d=0;d<3;++d)if(inner.Axis[d].Lower<outer.Axis[d].Lower||inner.Axis[d].Upper>outer.Axis[d].Upper)return false;
    return true;
}
C::FGeneralBoxMotionInput Motion(const FNativeSolidStaticValues& source,const FKinematicState& actual,float seconds)
{
    C::FGeneralBoxMotionInput input;input.Owner=actual;input.Horizon=seconds;
    input.LocalCOM=source.LocalCOM;input.LocalOffset=source.LocalOffset;input.LocalRotation=source.LocalRotation;
    input.PhysicalHalfExtent=source.PhysicalHalfExtent;input.QueryHalfExtent=source.QueryHalfExtent;return input;
}
bool Descriptor(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const FPreparedNativeSolidGeometryPtr& descriptor)
{
    return snapshot&&descriptor&&descriptor->GetRequest()&&descriptor->GetRequest()->GetSnapshot()==snapshot&&
        descriptor->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorRefinerDescriptor&&
        SameNativeSolidStaticValues(descriptor->GetRequest()->GetValues(),source)&&
        SameNativeSolidStaticValues(descriptor->GetValues().GeneralBoxFloorDescriptor.Captured,source)&&
        C::MatchesNativeGeneralBoxFloorPhysicalClear(descriptor->GetValues().GeneralBoxFloorDescriptor);
}
bool EndpointFloor(const Analytic::FWorldQuery& pose,const FNativeSolidStaticValues& source,float seconds,
    const C::FNativeBoxCachedPlaneValues& plane)
{
    // Enclose the actual already-synchronized BODY pose, not a guessed owner
    // pose or a radius/SAT proxy. Native/reference physical corners only.
    C::FGeneralBoxMotionInput point;point.Owner.Location=pose.Start;point.Owner.Rotation=pose.Rotation;
    point.PhysicalHalfExtent=point.QueryHalfExtent=source.PhysicalHalfExtent;point.Horizon=seconds;
    C::FGeneralBoxMotionBounds bounds;
    if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(point,0.f,0.f,bounds)||bounds.Corners.Num()!=32)return false;
    for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
    {
        AnalyticCertified::Interval gap;
        if(!C::GeneralPhysicalFloorClearDetail::RawGap(bounds.Corners[family*16+k],plane.Point,plane.Normal,gap)||
            !C::GeneralPhysicalFloorClearDetail::StrictPositive(gap))return false;
    }
    return true;
}
}
bool BuildNativeReleasedBoxFloorPrimitiveGeometryValues(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
    const FKinematicState& actual,float seconds,const FPreparedNativeSolidGeometryPtr& descriptor,
    C::FDistanceBudget& budget,FNativeBoxPrimitiveGeometryValuesPtr& out)
{
    out.Reset();Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed||
        !ReleasedFloorPrimitiveDetail::Descriptor(snapshot,source,descriptor))return false;
    const auto input=ReleasedFloorPrimitiveDetail::Motion(source,actual,seconds);
    C::FGroupedBoxVolumeBounds grouped;
    if(!C::BuildGeneralBoxPrimitiveGroupedVolume(input,actual.Rotation*source.LocalRotation,
        source.Query.DomainTolerance,source.Query.InitialOverlapTolerance,budget,grouped))return false;
    // ONLY the raw original envelope was authenticated against Landscape.
    // Its contained finite query/chord/SAT/shell corridor keeps all NONfloor
    // provider and filter exclusions. No original trajectory is substituted.
    if(!ReleasedFloorPrimitiveDetail::Within(descriptor->GetValues().QueryEnvelope,grouped.BroadHull))return false;
    const auto& floor=descriptor->GetValues().GeneralBoxFloorDescriptor.CachedPlane;
    C::FGeneralBoxPhysicalFloorClearBoundsPtr physical;
    const bool established=descriptor->GetValues().GeneralBoxFloorDescriptor.ObservationRequestIdentity!=nullptr;
    if(!C::TryProveGeneralBoxPhysicalFloorClear(input,floor.Point,floor.Normal,budget,physical,established)||!physical)return false;
    TSharedPtr<FNativeBoxPrimitiveGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPrimitiveGeometryValues);
    value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Actual=actual;value->Seconds=seconds;
    value->Envelope=grouped.BroadHull;value->GroupedVolume=MoveTemp(grouped);
    value->ReleasedDescriptor=descriptor;value->PhysicalFloorClear=MoveTemp(physical);
    value->Provenance=ENativeBoxPrimitiveGeometryProvenance::Released59;
    // The old whole-world GroupedSeparation is deliberately absent.
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    out=MoveTemp(value);return true;
}

namespace FloorMotionPrimitiveDetail
{

bool Cached(const FPreparedNativeSolidGeometryPtr& descriptor)
{return descriptor&&descriptor->GetValues().Status==ENativeSolidGeometryStatus::GeneralBoxFloorCachedArrivalDescriptor;}
const C::FNativeBoxCachedPlaneValues& Plane(const FPreparedNativeSolidGeometryPtr& descriptor)
{return Cached(descriptor)?descriptor->GetValues().GeneralBoxCachedFloorArrivalDescriptor.Plane:descriptor->GetValues().GeneralBoxFloorMotionDescriptor.Plane;}
bool CachedDescriptor(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const FPreparedNativeSolidGeometryPtr& descriptor,float primitiveSeconds,float worldSeconds)
{
    if(!snapshot||!Cached(descriptor)||!descriptor->GetRequest()||descriptor->GetRequest()->GetSnapshot()!=snapshot||
        !SameNativeSolidStaticValues(descriptor->GetRequest()->GetValues(),source)||source.Shape!=ENativeSolidQueryShape::Box||
        source.Frozen||source.Prescribed||!FMath::IsFinite(primitiveSeconds)||!(primitiveSeconds>0)||
        !FMath::IsFinite(worldSeconds)||!(worldSeconds>0)||primitiveSeconds>worldSeconds)return false;
    const auto& g=descriptor->GetValues().GeneralBoxCachedFloorArrivalDescriptor;
    return g.HasMatchingPhysicalPrefix()&&SameNativeSolidStaticValues(g.Captured,source)&&
        C::GroupedBoxDetail::SamePoint(g.QueryEnvelope,descriptor->GetValues().QueryEnvelope)&&
        worldSeconds<=g.CachedPrefix->GetMaximumAdvanceSeconds();
}
bool EndpointFixedCardinalCachedFloor(const Analytic::FWorldQuery& pose,const FNativeSolidStaticValues& source,
    const C::FNativeBoxCachedPlaneValues& plane)
{
    namespace A=::AnalyticCertified;
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto zero=[](double x){return (A::Bits(x)&A::Magnitude)==0;};
    const auto& q=pose.Rotation;
    const auto& extent=source.PhysicalHalfExtent;
    const auto boundedPositive=[](double x){return A::Finite(x)&&x>=1.&&x<=0x1p16;};
    if(!C::GeneralPhysicalFloorClearDetail::CardinalFloor(plane.Point,plane.Normal)||
        A::Bits(plane.Normal.Z)!=A::Bits(1.)||!zero(q.X)||(A::Bits(q.Y)&A::Magnitude)!=A::Bits(1.)||
        !zero(q.Z)||!zero(q.W)||pose.Start.ContainsNaN()||
        !boundedPositive(extent.X)||!boundedPositive(extent.Y)||!boundedPositive(extent.Z)||
        !boundedPositive(pose.Start.Z))return false;
    const double center=pose.Start.Z,half=extent.Z;
    if(center>2.*half||half>2.*center)return false;
    // The actual synchronized BODY has a cardinal rotation, so the native
    // and reference Z offsets are exactly +/-PhysicalHalfExtent.Z. The four
    // lower corners therefore use the exact Sterbenz difference below.
    // Each upper corner is higher by a positive amount. With the guarded
    // normal operands a nonzero difference is normal, including under FTZ.
    volatile double gap=center-half;
    return C::ClosedPhysicalFloorPoseDetail::NonNegative(A::Interval::Exact(gap))&&
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==
            numeric.ControlFingerprint();
}
bool EndpointCachedFloor(const Analytic::FWorldQuery& pose,const FNativeSolidStaticValues& source,float seconds,
    const C::FNativeBoxCachedPlaneValues& plane)
{
    // Enclose the actual already-synchronized BODY pose, not a guessed owner
    // pose or a radius/SAT proxy. Native/reference physical corners only.
    C::FGeneralBoxMotionInput point;point.Owner.Location=pose.Start;point.Owner.Rotation=pose.Rotation;
    point.PhysicalHalfExtent=point.QueryHalfExtent=source.PhysicalHalfExtent;point.Horizon=seconds;
    C::FGeneralBoxMotionBounds bounds;
    if(!BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(point,0.f,0.f,bounds)||bounds.Corners.Num()!=32)return false;
    for(int32 family=0;family<2;++family)for(int32 k=0;k<8;++k)
    {
        AnalyticCertified::Interval gap;
        if(!C::GeneralPhysicalFloorClearDetail::RawGap(bounds.Corners[family*16+k],plane.Point,plane.Normal,gap)||
            !C::ClosedPhysicalFloorPoseDetail::NonNegative(gap))return false;
    }
    return true;
}

bool Descriptor(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const FPreparedNativeSolidGeometryPtr& descriptor,float primitiveSeconds,float worldSeconds)
{
    if(Cached(descriptor))return CachedDescriptor(snapshot,source,descriptor,primitiveSeconds,worldSeconds);
    if(!snapshot||!descriptor||!descriptor->GetRequest()||descriptor->GetRequest()->GetSnapshot()!=snapshot||
        descriptor->GetValues().Status!=ENativeSolidGeometryStatus::GeneralBoxFloorMotionDescriptor||
        !SameNativeSolidStaticValues(descriptor->GetRequest()->GetValues(),source)||
        source.Shape!=ENativeSolidQueryShape::Box||source.Frozen||source.Prescribed||
        !FMath::IsFinite(primitiveSeconds)||primitiveSeconds<=0||!FMath::IsFinite(worldSeconds)||worldSeconds<=0||
        primitiveSeconds>worldSeconds)return false;
    const auto& g=descriptor->GetValues().GeneralBoxFloorMotionDescriptor;
    if(!SameNativeSolidStaticValues(g.Captured,source)||!g.HasMatchingPhysicalMotion()||
        !C::GroupedBoxDetail::SamePoint(g.QueryEnvelope,descriptor->GetValues().QueryEnvelope))return false;
    const auto status=g.Motion->GetStatus();
    if(status==C::EGeneralPhysicalFloorMotion::WholePhysicalClear)return worldSeconds<=source.Horizon;
    return status==C::EGeneralPhysicalFloorMotion::FirstAdjacentArrival&&g.Motion->GetClearSeconds()>0&&
        worldSeconds<=g.Motion->GetClearSeconds();
}
}
bool BuildNativeGeneralBoxFloorMotionPrimitiveGeometryValues(FPreparedWorldSnapshotPtr snapshot,const FNativeSolidStaticValues& source,
    const FKinematicState& actual,float seconds,float actualWorldSeconds,const FPreparedNativeSolidGeometryPtr& descriptor,
    C::FDistanceBudget& budget,FNativeBoxPrimitiveGeometryValuesPtr& out)
{
    out.Reset();Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto geometry=snapshot?snapshot->GetGeometrySelection():FPreparedGeometrySelectionPtr{};
    const auto fail=[&](const TCHAR* stage)
    {
        UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorMotionPrimitiveUnknown] Stage=%s PrimitiveH=%.17g WorldSeconds=%.17g Used=%llu Remaining=%d"),
            stage,double(seconds),double(actualWorldSeconds),budget.Used,budget.Remaining);
        UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorMotionPrimitiveActual] CapturedOwnerFrame=%u Stage=%s OriginalH=%.17g P=(%.17g,%.17g,%.17g) Q=(%.17g,%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) W=(%.17g,%.17g,%.17g) Alpha=(%.17g,%.17g,%.17g)"),
            source.FrameClock.OwnerLocalFrame,stage,double(source.Horizon),actual.Location.X,actual.Location.Y,actual.Location.Z,
            actual.Rotation.X,actual.Rotation.Y,actual.Rotation.Z,actual.Rotation.W,
            actual.Velocity.X,actual.Velocity.Y,actual.Velocity.Z,actual.Acceleration.X,actual.Acceleration.Y,actual.Acceleration.Z,
            actual.AngularVelocity.X,actual.AngularVelocity.Y,actual.AngularVelocity.Z,
            actual.AngularAcceleration.X,actual.AngularAcceleration.Y,actual.AngularAcceleration.Z);return false;
    };
    if(!geometry||!geometry->HasValidAliases()||geometry->GetModel()!=EPreparedGeometryModel::RetainedReference||
        !FloorMotionPrimitiveDetail::Descriptor(snapshot,source,descriptor,seconds,actualWorldSeconds))return fail(TEXT("descriptor_or_world_time"));
    const auto input=ReleasedFloorPrimitiveDetail::Motion(source,actual,seconds);C::FGroupedBoxVolumeBounds grouped;
    if(!C::BuildGeneralBoxPrimitiveGroupedVolume(input,actual.Rotation*source.LocalRotation,
        source.Query.DomainTolerance,source.Query.InitialOverlapTolerance,budget,grouped))return fail(TEXT("actual_grouped_volume"));
    const bool cached=FloorMotionPrimitiveDetail::Cached(descriptor);const auto& floor=FloorMotionPrimitiveDetail::Plane(descriptor);
    const auto& raw=descriptor->GetValues().QueryEnvelope;
    if(!ReleasedFloorPrimitiveDetail::Within(raw,grouped.BroadHull))return fail(TEXT("raw_authenticated_envelope"));
    C::FGeneralBoxPhysicalFloorClearBoundsPtr physical;C::FGeneralBoxPhysicalFloorPrefixBoundsPtr prefix;
    if(cached)
    {
        const bool fixedCardinal=descriptor->GetValues().GeneralBoxCachedFloorArrivalDescriptor.CachedPrefix->IsFixedCardinal();
        const bool proved=fixedCardinal?
            C::TryProveFixedCardinalBoxPhysicalFloorPrefix(input,floor.Point,floor.Normal,seconds,budget,prefix):
            C::TryProveGeneralBoxPhysicalFloorPrefix(input,floor.Point,floor.Normal,seconds,budget,prefix);
        if(!proved||!prefix)return fail(TEXT("actual_cached_physical_floor_prefix"));
    }
    else if(!C::TryProveGeneralBoxPhysicalFloorClear(input,floor.Point,floor.Normal,budget,physical)||!physical)
        return fail(TEXT("actual_physical_floor"));
    TSharedPtr<FNativeBoxPrimitiveGeometryValues,ESPMode::ThreadSafe> value(new FNativeBoxPrimitiveGeometryValues);
    value->Snapshot=snapshot;value->Geometry=geometry;value->Source=source;value->Actual=actual;value->Seconds=seconds;
    value->Envelope=grouped.BroadHull;value->GroupedVolume=MoveTemp(grouped);value->MotionDescriptor=descriptor;
    value->MotionWorldSeconds=actualWorldSeconds;value->PhysicalFloorClear=MoveTemp(physical);value->PhysicalFloorPrefix=MoveTemp(prefix);
    value->Provenance=cached?ENativeBoxPrimitiveGeometryProvenance::Cached65First:descriptor->GetValues().GeneralBoxFloorMotionDescriptor.Motion->GetStatus()==C::EGeneralPhysicalFloorMotion::FirstAdjacentArrival?
        ENativeBoxPrimitiveGeometryProvenance::Motion60First:ENativeBoxPrimitiveGeometryProvenance::Motion60Whole;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    out=MoveTemp(value);return true;
}
bool FNativeBoxPrimitiveGeometryValues::MatchesFloorMotion(const FPreparedWorldSnapshotPtr& snapshot,
    const FNativeSolidStaticValues& source,const FKinematicState& actual,float seconds,float actualWorldSeconds,
    const FPreparedNativeSolidGeometryPtr& descriptor) const
{
    if(Snapshot!=snapshot||!snapshot||Geometry!=snapshot->GetGeometrySelection()||!Geometry||!Geometry->HasValidAliases()||
        !SameNativeSolidStaticValues(Source,source)||!SameKinematicBits(Actual,actual)||!SameFloat(Seconds,seconds)||
        !SameFloat(MotionWorldSeconds,actualWorldSeconds)||MotionDescriptor!=descriptor||ReleasedDescriptor||GroupedSeparation||
        (FloorMotionPrimitiveDetail::Cached(descriptor)?!PhysicalFloorPrefix||PhysicalFloorClear:!PhysicalFloorClear||PhysicalFloorPrefix)||
        !C::GroupedBoxDetail::SamePoint(Envelope,GroupedVolume.BroadHull)||
        !FloorMotionPrimitiveDetail::Descriptor(snapshot,source,descriptor,seconds,actualWorldSeconds))return false;
    const bool cached=FloorMotionPrimitiveDetail::Cached(descriptor);const auto& floor=FloorMotionPrimitiveDetail::Plane(descriptor);
    const auto expected=cached?ENativeBoxPrimitiveGeometryProvenance::Cached65First:descriptor->GetValues().GeneralBoxFloorMotionDescriptor.Motion->GetStatus()==C::EGeneralPhysicalFloorMotion::FirstAdjacentArrival?
        ENativeBoxPrimitiveGeometryProvenance::Motion60First:ENativeBoxPrimitiveGeometryProvenance::Motion60Whole;
    return Provenance==expected&&ReleasedFloorPrimitiveDetail::Within(descriptor->GetValues().QueryEnvelope,Envelope)&&
        (cached?PhysicalFloorPrefix->Matches(ReleasedFloorPrimitiveDetail::Motion(source,actual,seconds),floor.Point,floor.Normal,seconds):
            PhysicalFloorClear->Matches(ReleasedFloorPrimitiveDetail::Motion(source,actual,seconds),floor.Point,floor.Normal));
}
bool FNativeBoxPrimitiveGeometryValues::ContainsFloorMotionEndpoint(const Analytic::FWorldQuery& pose,float actualWorldSeconds,
    const FPreparedNativeSolidGeometryPtr& descriptor) const
{
    const bool fixed=PhysicalFloorPrefix&&PhysicalFloorPrefix->IsFixedCardinal();
    const auto trace=[&](const TCHAR* stage,int32 extentRole=INDEX_NONE,const C::FIntervalPoint* actualVolume=nullptr)
    {
        if(!fixed)return;
        const auto& p=pose.Start;const auto& q=pose.Rotation;
        const auto& h=Source.PhysicalHalfExtent;const auto& r=Source.QueryHalfExtent;
        UE_LOG(LogTemp,Display,TEXT("[NativeRoofFixedEndpoint] OwnerFrame=%u Stage=%s ExtentRole=%d PrimitiveH=%.17g WorldSeconds=%.17g BodyP=(%.17g,%.17g,%.17g) BodyQ=(%.17g,%.17g,%.17g,%.17g) Physical=(%.17g,%.17g,%.17g) Rounded=(%.17g,%.17g,%.17g)"),
            Source.FrameClock.OwnerLocalFrame,stage,extentRole,double(Seconds),double(actualWorldSeconds),
            p.X,p.Y,p.Z,q.X,q.Y,q.Z,q.W,h.X,h.Y,h.Z,r.X,r.Y,r.Z);
        if(!actualVolume||!descriptor)return;
        const auto& a=*actualVolume;const auto& g=Envelope;const auto& d=descriptor->GetValues().QueryEnvelope;
        UE_LOG(LogTemp,Display,TEXT("[NativeRoofFixedEndpointEnvelopes] OwnerFrame=%u Stage=%s ExtentRole=%d ActualX=[%.17g,%.17g] ActualY=[%.17g,%.17g] ActualZ=[%.17g,%.17g] GroupX=[%.17g,%.17g] GroupY=[%.17g,%.17g] GroupZ=[%.17g,%.17g] DescriptorX=[%.17g,%.17g] DescriptorY=[%.17g,%.17g] DescriptorZ=[%.17g,%.17g]"),
            Source.FrameClock.OwnerLocalFrame,stage,extentRole,
            a.Axis[0].Lower,a.Axis[0].Upper,a.Axis[1].Lower,a.Axis[1].Upper,a.Axis[2].Lower,a.Axis[2].Upper,
            g.Axis[0].Lower,g.Axis[0].Upper,g.Axis[1].Lower,g.Axis[1].Upper,g.Axis[2].Lower,g.Axis[2].Upper,
            d.Axis[0].Lower,d.Axis[0].Upper,d.Axis[1].Lower,d.Axis[1].Upper,d.Axis[2].Lower,d.Axis[2].Upper);
    };
    if(!MatchesFloorMotion(Snapshot,Source,Actual,Seconds,actualWorldSeconds,descriptor))
    {trace(TEXT("fresh_matches"));return false;}
    if(!C::FloorNativeSweepDetail::SameVector(pose.Start,pose.End))
    {trace(TEXT("stationary"));return false;}
    int32 extentRole=0;
    for(const auto extent:{Source.PhysicalHalfExtent,Source.QueryHalfExtent})
    {
        auto query=pose;query.HalfExtent=extent;C::FIntervalPoint volume;
        if(!C::BuildFixedWorldQueryVolumeBounds(query,volume))
        {trace(TEXT("fixed_world_volume"),extentRole);return false;}
        if(!ReleasedFloorPrimitiveDetail::Within(Envelope,volume))
        {trace(TEXT("grouped_containment"),extentRole,&volume);return false;}
        if(!ReleasedFloorPrimitiveDetail::Within(descriptor->GetValues().QueryEnvelope,volume))
        {trace(TEXT("descriptor_containment"),extentRole,&volume);return false;}
        ++extentRole;
    }
    const auto& plane=FloorMotionPrimitiveDetail::Plane(descriptor);
    if(!FloorMotionPrimitiveDetail::Cached(descriptor))
        return ReleasedFloorPrimitiveDetail::EndpointFloor(pose,Source,Seconds,plane);
    if(fixed)
    {
        const bool proved=FloorMotionPrimitiveDetail::EndpointFixedCardinalCachedFloor(pose,Source,plane);
        trace(proved?TEXT("signed_floor_closed"):TEXT("signed_floor"));
        return proved;
    }
    return FloorMotionPrimitiveDetail::EndpointCachedFloor(pose,Source,Seconds,plane);
}

bool FNativeBoxPrimitiveGeometryValues::Matches(const FPreparedWorldSnapshotPtr& snapshot,const FNativeSolidStaticValues& source,
    const FKinematicState& actual,float seconds) const
{
    if(MotionDescriptor||PhysicalFloorPrefix||(RebasedRoof202.IsSet()&&(!CurvedArrival||!PhysicalFloorClear||
        !RebasedRoof202->Matches(C::KinematicCenterDetail::Bits(seconds))))||
        Snapshot!=snapshot||!snapshot||Geometry!=snapshot->GetGeometrySelection()||!Geometry||!Geometry->HasValidAliases()||
        !SameNativeSolidStaticValues(Source,source)||!SameKinematicBits(Actual,actual)||!SameFloat(Seconds,seconds))return false;
    if(CurvedWholeClear)
    {
        if(CurvedArrival||ReleasedDescriptor||PhysicalFloorClear||DirectionalSeparation||GroupedSeparation||
            !GroupedVolume.Groups.IsEmpty())return false;
        if(CurvedWholeClear->IsActualMotionOnly())
            return Provenance==ENativeBoxPrimitiveGeometryProvenance::CurvedGraphActualMotionClear&&
                CurvedWholeClear->MatchesSnapshot(snapshot,source,Envelope)&&seconds>0.f&&seconds<=source.Horizon;
        return Provenance==ENativeBoxPrimitiveGeometryProvenance::CurvedGraphWholeClear&&
            CurvedWholeClear->MatchesActual(snapshot,source,actual,seconds,Envelope);
    }
    if(CurvedArrival)
        return Provenance==ENativeBoxPrimitiveGeometryProvenance::CurvedArrivalPrefix&&
            !ReleasedDescriptor&&!DirectionalSeparation&&!GroupedSeparation&&
            C::Detail::ValidPoint(Envelope)&&CurvedArrival->Matches(snapshot,source)&&
            (!PhysicalFloorClear||CurvedArrival->MatchesRebasedSplitFloor(source,actual,seconds,Envelope,PhysicalFloorClear))&&
            CurvedArrival->CoversCompetingPrefixEnvelope(Envelope)&&
            seconds>0.f&&seconds<=CurvedArrival->MaximumAdvanceSeconds();
    if(DirectionalSeparation)
        return Provenance==ENativeBoxPrimitiveGeometryProvenance::WholeWorldStrict&&!ReleasedDescriptor&&!PhysicalFloorClear&&!GroupedSeparation&&GroupedVolume.Groups.IsEmpty()&&
            DirectionalSeparation->Matches(Geometry->GetSelectedUnion(),Envelope);
    if(!C::GroupedBoxDetail::SamePoint(Envelope,GroupedVolume.BroadHull))return false;
    if(ReleasedDescriptor)
    {
        if(Provenance!=ENativeBoxPrimitiveGeometryProvenance::Released59||GroupedSeparation||!PhysicalFloorClear||!ReleasedFloorPrimitiveDetail::Descriptor(snapshot,source,ReleasedDescriptor)||
            !ReleasedFloorPrimitiveDetail::Within(ReleasedDescriptor->GetValues().QueryEnvelope,Envelope))return false;
        const auto& floor=ReleasedDescriptor->GetValues().GeneralBoxFloorDescriptor.CachedPlane;
        return PhysicalFloorClear->Matches(ReleasedFloorPrimitiveDetail::Motion(source,actual,seconds),floor.Point,floor.Normal);
    }
    return Provenance==ENativeBoxPrimitiveGeometryProvenance::WholeWorldStrict&&!PhysicalFloorClear&&GroupedSeparation&&GroupedSeparation->Matches(Geometry->GetSelectedUnion(),GroupedVolume);
}
bool FNativeBoxPrimitiveGeometryValues::ContainsEndpoint(const Analytic::FWorldQuery& pose) const
{
    if(MotionDescriptor)return false;
    if(CurvedWholeClear)
    {
        C::FDistanceBudget work{3072};
        return Matches(Snapshot,Source,Actual,Seconds)&&CurvedWholeClear->ContainsActualEndpoint(pose,work);
    }
    if(CurvedArrival)
    {
        C::FDistanceBudget work{1536};
        return Matches(Snapshot,Source,Actual,Seconds)&&
            CurvedArrival->ContainsActualEndpoint(Snapshot,Source,pose,Envelope,work,1536);
    }
    if(DirectionalSeparation)
        return Matches(Snapshot,Source,Actual,Seconds)&&NativeDirectionalCoarseDetail::ContainsEndpoint(DirectionalSeparation,pose,Source.PhysicalHalfExtent,Source.QueryHalfExtent);
    if(ReleasedDescriptor)
    {
        if(!Matches(Snapshot,Source,Actual,Seconds)||!C::FloorNativeSweepDetail::SameVector(pose.Start,pose.End))return false;
        for(const auto extent:{Source.PhysicalHalfExtent,Source.QueryHalfExtent})
        {
            auto query=pose;query.HalfExtent=extent;C::FIntervalPoint volume;
            if(!C::BuildFixedWorldQueryVolumeBounds(query,volume)||!ReleasedFloorPrimitiveDetail::Within(Envelope,volume)||
                !ReleasedFloorPrimitiveDetail::Within(ReleasedDescriptor->GetValues().QueryEnvelope,volume))return false;
        }
        return ReleasedFloorPrimitiveDetail::EndpointFloor(pose,Source,Seconds,
            ReleasedDescriptor->GetValues().GeneralBoxFloorDescriptor.CachedPlane);
    }
    return Provenance==ENativeBoxPrimitiveGeometryProvenance::WholeWorldStrict&&!PhysicalFloorClear&&Geometry&&Geometry->HasValidAliases()&&GroupedSeparation&&
        GroupedSeparation->Matches(Geometry->GetSelectedUnion(),GroupedVolume)&&
        C::GroupedBoxProofContainsEndpoint(GroupedSeparation,pose,Source.PhysicalHalfExtent,Source.QueryHalfExtent);
}

}


#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGeneralBoxMotionValueMemoTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.GeneralBoxMotionValueMemo",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGeneralBoxMotionValueMemoTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Certified;
    namespace M=GeneralBoxMotionMemo;namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxMotionValueMemo"),1,FName(TEXT("GeneralBoxMotionValueMemoTest")));
    FGeneralBoxMotionInput input;input.Owner.Location=FVector(10,-20,30);input.Owner.Velocity=FVector(2,3,-1);
    input.Owner.Acceleration=FVector(.1,.2,.3);input.Owner.Rotation=FQuat(FVector::UpVector,.2);
    input.Owner.AngularVelocity=FVector(.03,.02,.01);input.Owner.AngularAcceleration=FVector(.004,.005,.006);
    input.LocalCOM=FVector(1,2,3);input.LocalOffset=FVector(2,-1,4);input.LocalRotation=FQuat(FVector::ForwardVector,.1);
    input.PhysicalHalfExtent=FVector(1,2,3);input.QueryHalfExtent=FVector(1.1,2.1,3.1);input.Horizon=1.f/300.f;
    const auto samePoint=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||A::Bits(a.Axis[d].Lower)!=A::Bits(b.Axis[d].Lower)||A::Bits(a.Axis[d].Upper)!=A::Bits(b.Axis[d].Upper))return false;return true;};
    const auto same=[&](const FGeneralBoxMotionBounds& a,const FGeneralBoxMotionBounds& b)
    {if(!samePoint(a.Hull,b.Hull)||!samePoint(a.NativeCenter,b.NativeCenter)||!samePoint(a.ReferenceCenter,b.ReferenceCenter)||a.Corners.Num()!=b.Corners.Num())return false;for(int32 i=0;i<a.Corners.Num();++i)if(!samePoint(a.Corners[i],b.Corners[i]))return false;return true;};
    const auto fields=[](FGeneralBoxMotionInput& v)
    {
        auto& o=v.Owner;return std::array<double*,35>{&o.Location.X,&o.Location.Y,&o.Location.Z,
            &o.Velocity.X,&o.Velocity.Y,&o.Velocity.Z,&o.Acceleration.X,&o.Acceleration.Y,&o.Acceleration.Z,
            &o.Rotation.X,&o.Rotation.Y,&o.Rotation.Z,&o.Rotation.W,&o.AngularVelocity.X,&o.AngularVelocity.Y,&o.AngularVelocity.Z,
            &o.AngularAcceleration.X,&o.AngularAcceleration.Y,&o.AngularAcceleration.Z,
            &v.LocalCOM.X,&v.LocalCOM.Y,&v.LocalCOM.Z,&v.LocalOffset.X,&v.LocalOffset.Y,&v.LocalOffset.Z,
            &v.LocalRotation.X,&v.LocalRotation.Y,&v.LocalRotation.Z,&v.LocalRotation.W,
            &v.PhysicalHalfExtent.X,&v.PhysicalHalfExtent.Y,&v.PhysicalHalfExtent.Z,&v.QueryHalfExtent.X,&v.QueryHalfExtent.Y,&v.QueryHalfExtent.Z};
    };
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        const auto key=M::Key(input,numeric);
        bool complete=true;for(int32 i=0;i<35;++i)
        {auto changed=input;auto operands=fields(changed);*operands[i]=A::FromBits(A::Bits(*operands[i])+1);complete&=!(M::Key(changed,numeric)==key);}
        auto changedH=input;uint32 h=KinematicCenterDetail::Bits(input.Horizon)+1;FMemory::Memcpy(&changedH.Horizon,&h,sizeof(h));
        complete&=!(M::Key(changedH,numeric)==key);complete&=!(M::Key(input,numeric^uint64(0x40))==key);
        TestTrue(TEXT("all 35 numeric fields, float horizon and numeric mode discriminate the key"),complete);
        for(auto& entry:M::Cache().Entries)entry.Ready=false;
        FGeneralBoxMotionBounds reference,first,second;const uint64 misses=M::Cache().Misses;
        const bool referenceKnown=BuildGeneralBoxMotionBounds(input,reference);
        const bool firstKnown=BuildGeneralBoxMotionBoundsMemo(input,first,0);
        const uint64 hits=M::Cache().Hits;
        const bool secondKnown=BuildGeneralBoxMotionBoundsMemo(input,second,1);
        TestTrue(TEXT("first successful model stores and identical cross-route request hits"),referenceKnown&&firstKnown&&secondKnown&&M::Cache().Misses==misses+1&&M::Cache().Hits==hits+1);
        TestTrue(TEXT("all three bounds and all 32 corners are bit-identical"),same(reference,first)&&same(reference,second)&&second.Corners.Num()==32);
        auto changed=input;changed.Owner.Velocity.X=A::FromBits(A::Bits(changed.Owner.Velocity.X)+1);
        FGeneralBoxMotionBounds changedReference,changedMemo;const uint64 changedBefore=M::Cache().Misses;
        TestTrue(TEXT("changed owner state misses and agrees with original model"),BuildGeneralBoxMotionBounds(changed,changedReference)&&BuildGeneralBoxMotionBoundsMemo(changed,changedMemo,1)&&M::Cache().Misses==changedBefore+1&&same(changedReference,changedMemo));
        auto invalid=input;invalid.Owner.Velocity.X=A::FromBits(A::Infinity|1);FGeneralBoxMotionBounds refused;
        const uint64 failedBefore=M::Cache().Misses;
        TestFalse(TEXT("invalid state refuses"),BuildGeneralBoxMotionBoundsMemo(invalid,refused,0));
        TestFalse(TEXT("invalid state is never cached"),BuildGeneralBoxMotionBoundsMemo(invalid,refused,1));
        TestTrue(TEXT("both failed calls remain misses"),M::Cache().Misses==failedBefore+2);
        Speed::PreparedExecution::FNativeSolidStaticValues v;v.Shape=Speed::PreparedExecution::ENativeSolidQueryShape::Box;
        v.OwnerState=input.Owner;v.BodyState=input.Owner;v.LocalCOM=input.LocalCOM;v.LocalOffset=input.LocalOffset;v.LocalRotation=input.LocalRotation;
        v.Horizon=input.Horizon;v.PhysicalHalfExtent=input.PhysicalHalfExtent;v.QueryHalfExtent=input.QueryHalfExtent;
        v.BodyState.Location=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+v.OwnerState.Rotation.RotateVector(v.LocalOffset);
        v.BodyState.Rotation=v.OwnerState.Rotation*v.LocalRotation;
        v.Query.Shape=EQueryShape::Box;v.Query.HalfExtent=v.QueryHalfExtent;v.Query.Start=v.BodyState.Location;
        v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
        v.Query.Rotation=v.BodyState.Rotation;v.Query.DomainTolerance=0;v.Query.InitialOverlapTolerance=0;
        FIntervalPoint oldEnvelope,newEnvelope;
        TestTrue(TEXT("literal native wrapper preserves complete query envelope bits"),BuildGeneralNativeBoxSweepVolume(v,oldEnvelope)&&BuildGeneralNativeBoxSweepVolumeMemo(v,newEnvelope)&&samePoint(oldEnvelope,newEnvelope));
        auto badChord=v;badChord.Query.End.X+=1;
        TestFalse(TEXT("memo does not bypass actual native chord validation"),BuildGeneralNativeBoxSweepVolumeMemo(badChord,newEnvelope));
        auto frozen=v;frozen.Frozen=true;TestFalse(TEXT("memo does not bypass frozen guard"),BuildGeneralNativeBoxSweepVolumeMemo(frozen,newEnvelope));
        FDistanceBudget oldBudget{8192,17},newBudget{8192,17};FGroupedBoxVolumeBounds oldGrouped,newGrouped;
        const auto axes=input.Owner.Rotation*input.LocalRotation;
        const bool oldGroupedKnown=BuildGeneralBoxPrimitiveGroupedVolume(input,axes,0,0,oldBudget,oldGrouped);
        const bool newGroupedKnown=BuildGeneralBoxPrimitiveGroupedVolumeMemo(input,axes,0,0,newBudget,newGrouped);
        TestTrue(TEXT("model hit does not refund or expand primitive work account"),oldGroupedKnown==newGroupedKnown&&oldBudget.Remaining==newBudget.Remaining&&oldBudget.Used==newBudget.Used&&oldBudget.ConsumptionFloor==newBudget.ConsumptionFloor);
        if(oldGroupedKnown)TestTrue(TEXT("primitive broad hull remains bit-identical"),samePoint(oldGrouped.BroadHull,newGrouped.BroadHull));
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativePredictionCoarseCompleteVolumeTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.PredictionCoarseCompleteVolume",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativePredictionCoarseCompleteVolumeTest::RunTest(const FString&)
{
    using namespace Speed::Analytic;using namespace Certified;using namespace Speed::PreparedExecution;
    namespace P=Speed::PreparedExecution::NativePredictionCoarseDetail;
    namespace A=::AnalyticCertified;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> memo(TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxMotionValueMemo"),1,FName(TEXT("PredictionCoarseCompleteVolume")));
    FNativeSolidStaticValues v;v.Shape=ENativeSolidQueryShape::Box;
    v.OwnerState.Location=FVector(10,-20,30);v.OwnerState.Velocity=FVector(2,3,-1);
    v.OwnerState.Acceleration=FVector(.1,.2,.3);v.OwnerState.Rotation=FQuat(FVector::UpVector,.2);
    v.OwnerState.AngularVelocity=FVector(.03,.02,.01);v.OwnerState.AngularAcceleration=FVector(.004,.005,.006);
    v.BodyState=v.OwnerState;v.LocalCOM=FVector(1,2,3);v.LocalOffset=FVector(2,-1,4);
    v.LocalRotation=FQuat(FVector::ForwardVector,.1);v.Horizon=1.f/300.f;
    v.PhysicalHalfExtent=FVector(1,2,3);v.QueryHalfExtent=FVector(1.1,2.1,3.1);
    v.BodyState.Location=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    v.BodyState.Rotation=v.OwnerState.Rotation*v.LocalRotation;
    // The captured World chord is deliberately distinct from the owner path.
    v.BodyState.Velocity=FVector(1000,0,0);
    v.Query.Shape=EQueryShape::Box;v.Query.Start=v.BodyState.Location;v.Query.Rotation=v.BodyState.Rotation;
    v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    v.Query.HalfExtent=v.QueryHalfExtent;v.Query.DomainTolerance=.01;v.Query.InitialOverlapTolerance=.02;
    const auto samePoint=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||A::Bits(a.Axis[d].Lower)!=A::Bits(b.Axis[d].Lower)||A::Bits(a.Axis[d].Upper)!=A::Bits(b.Axis[d].Upper))return false;return true;};
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FIntervalPoint originalHull,memoHull;
        if(!TestTrue(TEXT("same complete native capture and query envelope before grouping"),
            BuildGeneralNativeBoxSweepVolume(v,originalHull)&&BuildGeneralNativeBoxSweepVolumeMemo(v,memoHull)&&samePoint(originalHull,memoHull)))return false;
        FGeneralBoxMotionInput input;input.Owner=v.OwnerState;input.Horizon=v.Horizon;input.LocalCOM=v.LocalCOM;input.LocalOffset=v.LocalOffset;
        input.LocalRotation=v.LocalRotation;input.PhysicalHalfExtent=v.PhysicalHalfExtent;input.QueryHalfExtent=v.QueryHalfExtent;
        FGeneralBoxMotionBounds whole;
        if(!TestTrue(TEXT("original whole motion oracle"),BuildGeneralBoxMotionBounds(input,whole)))return false;
        FDistanceBudget coarseBudget{8192};FGroupedBoxVolumeBounds coarse;
        if(!TestTrue(TEXT("eight complete prediction groups and six original axes"),P::BuildVolume(v,coarseBudget,coarse)&&coarse.Groups.Num()==8&&coarse.Axes.Num()==6&&coarse.Slabs.Num()==48))return false;
        TArray<FBoxVolumeGroup,TInlineAllocator<16>> expected;
        if(!NativeGroupedBoxDetail::MotionGroups(whole,expected))return false;
        FBoxVolumeGroup chord;if(!NativeGroupedBoxDetail::FixedQueryGroup(v.Query,chord))return false;expected.Add(chord);
        auto pose=v.Query;pose.Start=pose.End=v.BodyState.Location;pose.HalfExtent=v.PhysicalHalfExtent;
        FBoxVolumeGroup physical;if(!NativeGroupedBoxDetail::FixedQueryGroup(pose,physical))return false;expected.Add(physical);
        auto q=v.Query;q.Start=q.End=FVector3d::ZeroVector;FBoxVolumeGroup footprint;
        if(!NativeGroupedBoxDetail::FixedQueryGroup(q,footprint))return false;
        for(const auto& center:{whole.NativeCenter,whole.ReferenceCenter})
        {
            FBoxVolumeGroup group;
            for(const auto& corner:footprint.Corners)
            {FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]=A::Add(center.Axis[d],corner.Axis[d]);group.Corners.Add(point);}
            expected.Add(MoveTemp(group));
        }
        FDistanceBudget oracleBudget{8192};FGroupedBoxVolumeBounds oracle;
        if(!TestTrue(TEXT("all motion/chord/pose/footprint values, shells and axes match literal oracle"),
            NativeGroupedBoxDetail::Close(originalHull,expected,v.Query.Rotation,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,oracleBudget,oracle)&&
            GroupedBoxDetail::Same(coarse,oracle)&&coarseBudget.Used==oracleBudget.Used+2))return false;
        auto changed=v;changed.Query.End.X+=1;FGroupedBoxVolumeBounds rejected;FDistanceBudget denied{8192};
        TestTrue(TEXT("chord guard is not bypassed by model memo"),!P::BuildVolume(changed,denied,rejected)&&rejected.Groups.IsEmpty());
        changed=v;changed.Frozen=true;denied={8192};TestFalse(TEXT("frozen capture is outside this coarse model"),P::BuildVolume(changed,denied,rejected));
        changed=v;changed.Prescribed=true;denied={8192};TestFalse(TEXT("prescribed capture is outside this coarse model"),P::BuildVolume(changed,denied,rejected));
        changed=v;changed.Query.HalfExtent.X+=1;denied={8192};TestFalse(TEXT("physical/query extents remain authenticated"),P::BuildVolume(changed,denied,rejected));
        TArray<FIntervalTensorSurface> farSurfaces;FIntervalTensorSurface far;
        far.ControlPoints.Add(Detail::Point(FVector3d(10000,10000,10000)));farSurfaces.Add(far);
        FDistanceBudget sceneBudget{128};FIndexedIntervalUnionPtr scene;
        if(!TestTrue(TEXT("owned far scene"),FIndexedIntervalUnion::Build(farSurfaces,sceneBudget,scene)))return false;
        const int32 allowance=scene->Num()+2048,originalFine=8192;
        FDistanceBudget off{originalFine};FReferenceBoundsCandidateSetPtr partition;FReferenceBoxVolumeStrictSeparationPtr proof;int32 guardCalls=0;
        auto guard=[&](const FIntervalPoint&){++guardCalls;return true;};
        TestTrue(TEXT("zero optional allowance performs no work and gives no coarse authority"),
            P::Attempt(scene,v,off,0,guard,rejected,partition,proof)==P::EAttempt::KeepFine&&off.Used==0&&guardCalls==0&&!proof);
        FDistanceBudget clearBudget{originalFine+allowance};FGroupedBoxVolumeBounds clearVolume;
        if(!TestTrue(TEXT("complete coarse proof covers every provider before external guard"),
            P::Attempt(scene,v,clearBudget,allowance,guard,clearVolume,partition,proof)==P::EAttempt::Clear&&guardCalls==1&&
            proof&&proof->Matches(scene,clearVolume)&&clearVolume.Groups.Num()==8&&clearBudget.Remaining>=originalFine&&
            clearBudget.Used+uint64(clearBudget.Remaining)==uint64(originalFine+allowance)&&clearBudget.ConsumptionFloor==0))return false;
        FDistanceBudget shared{originalFine+allowance};guardCalls=0;
        const auto refused=P::Attempt(scene,v,shared,allowance,[&](const FIntervalPoint&){++guardCalls;return false;},clearVolume,partition,proof);
        const uint64 coarseSpent=shared.Used;FDistanceBudget referenceFine{originalFine};FGroupedBoxVolumeBounds fine,reference;
        if(!TestTrue(TEXT("failed external coarse guard preserves the complete original fine allowance"),
            refused==P::EAttempt::KeepFine&&guardCalls==1&&shared.Remaining>=originalFine&&coarseSpent>0&&shared.ConsumptionFloor==0))return false;
        bool fineKnown=false;
        {
            FScopedDistanceBudgetFloor fineFloor(shared,shared.Remaining-originalFine);
            fineKnown=fineFloor.IsValid()&&BuildGeneralNativeBoxGroupedVolume(v,shared,fine);
        }
        if(!TestTrue(TEXT("literal fourteen-group fallback remains bit-identical with every real debit retained"),
            fineKnown&&BuildGeneralNativeBoxGroupedVolume(v,referenceFine,reference)&&fine.Groups.Num()==14&&
            GroupedBoxDetail::Same(fine,reference)&&shared.Used==coarseSpent+referenceFine.Used&&shared.ConsumptionFloor==0&&
            shared.Used+uint64(shared.Remaining)==uint64(originalFine+allowance)))return false;
        FDistanceBudget badFloor{originalFine+allowance,0,-1};guardCalls=0;
        TestTrue(TEXT("invalid account cannot enter optional prediction"),P::Attempt(scene,v,badFloor,allowance,guard,rejected,partition,proof)==P::EAttempt::Invalid&&badFloor.Used==0&&guardCalls==0);
    }
    TestTrue(TEXT("numeric controls restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeLooseWholeSweepTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.LooseWholeCompleteVolume",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeLooseWholeSweepTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const auto contains=[](const FIntervalPoint& outer,const FIntervalPoint& inner)
    {if(!Detail::ValidPoint(outer)||!Detail::ValidPoint(inner))return false;for(int32 d=0;d<3;++d)if(outer.Axis[d].Lower>inner.Axis[d].Lower||outer.Axis[d].Upper<inner.Axis[d].Upper)return false;return true;};
    const auto same=[](const FIntervalPoint& a,const FIntervalPoint& b)
    {for(int32 d=0;d<3;++d)if(!GroupedBoxDetail::SameInterval(a.Axis[d],b.Axis[d]))return false;return true;};
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(int32 sample=0;sample<16;++sample)
        {
            const double k=double(sample+1);
            FGeneralBoxMotionInput input;
            input.Owner.Location=FVector(10*k,-7*k,15*k);input.Owner.Velocity=FVector(2*k,-3*k,1.1*k);
            input.Owner.Acceleration=FVector(.2*k,.1*k,-.3*k);
            input.Owner.Rotation=FQuat(FVector(1,2,3).GetSafeNormal(),.09*k);
            input.Owner.AngularVelocity=FVector(.07*k,-.03*k,.02*k);input.Owner.AngularAcceleration=FVector(.004*k,.005*k,-.006*k);
            input.LocalCOM=FVector(1,-2,3);input.LocalOffset=FVector(-2,1,4);
            input.LocalRotation=FQuat(FVector(2,-1,1).GetSafeNormal(),-.07*k);
            input.PhysicalHalfExtent=FVector(1,2,3);input.QueryHalfExtent=FVector(1.2,1.8,3.1);
            input.Horizon=float(double(1+sample%4)/300.);
            FGeneralBoxMotionBounds exact;NativeLooseWholeDetail::FLooseMotionBounds loose;
            if(!TestTrue(TEXT("same true 3D rotation/acceleration domain builds both models"),
                BuildGeneralBoxMotionBounds(input,exact)&&NativeLooseWholeDetail::BuildMotion(input,loose)))return false;
            bool covered=exact.Corners.Num()==32&&contains(loose.Hull,exact.Hull)&&
                same(exact.NativeCenter,loose.NativeCenter)&&same(exact.ReferenceCenter,loose.ReferenceCenter);
            for(const auto& point:exact.Corners)covered&=contains(loose.Hull,point);
            if(!TestTrue(TEXT("two interval boxes cover all 32 original native/reference physical/query corners with unchanged centers"),covered))return false;
            PreparedExecution::FNativeSolidStaticValues v;v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;
            v.OwnerState=input.Owner;v.BodyState=input.Owner;v.LocalCOM=input.LocalCOM;v.LocalOffset=input.LocalOffset;v.LocalRotation=input.LocalRotation;
            v.Horizon=input.Horizon;v.PhysicalHalfExtent=input.PhysicalHalfExtent;v.QueryHalfExtent=input.QueryHalfExtent;
            v.BodyState.Location=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+v.OwnerState.Rotation.RotateVector(v.LocalOffset);
            v.BodyState.Rotation=v.OwnerState.Rotation*v.LocalRotation;
            // The captured World chord may be much longer than owner motion.
            v.BodyState.Velocity=FVector(1000*k,-500*k,200*k);v.BodyState.Acceleration=FVector(3*k,-7*k,11*k);
            v.Query.Shape=EQueryShape::Box;v.Query.HalfExtent=v.QueryHalfExtent;v.Query.Start=v.BodyState.Location;
            v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
            v.Query.Rotation=v.BodyState.Rotation;v.Query.DomainTolerance=.003;v.Query.InitialOverlapTolerance=.007;
            FIntervalPoint full,outer;
            if(!TestTrue(TEXT("complete loose wrapper retains original chord, captured pose, both footprints and shells"),
                BuildGeneralNativeBoxSweepVolume(v,full)&&BuildGeneralNativeBoxLooseSweepVolume(v,outer)&&contains(outer,full)))return false;
            auto bad=v;bad.Query.End.X+=1;
            if(!TestTrue(TEXT("wrong chord rejected by both producers"),!BuildGeneralNativeBoxSweepVolume(bad,full)&&!BuildGeneralNativeBoxLooseSweepVolume(bad,full)))return false;
            bad=v;bad.Frozen=true;if(!TestFalse(TEXT("frozen input remains unsupported"),BuildGeneralNativeBoxLooseSweepVolume(bad,full)))return false;
            bad=v;bad.Prescribed=true;if(!TestFalse(TEXT("prescribed input remains unsupported"),BuildGeneralNativeBoxLooseSweepVolume(bad,full)))return false;
            bad=v;bad.Query.HalfExtent.X+=1;if(!TestFalse(TEXT("query shape identity guard remains exact"),BuildGeneralNativeBoxLooseSweepVolume(bad,full)))return false;
            if(sample!=0)continue;
            for(bool near:{false,true})
            {
                FVector location(1000000,1000000,1000000);
                if(near)for(int32 d=0;d<3;++d)location[d]=.5*outer.Axis[d].Lower+.5*outer.Axis[d].Upper;
                FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=0;surface.ControlPoints.Add(Detail::Point(location));
                FDistanceBudget sceneWork{100};FIndexedIntervalUnionPtr scene;
                if(!FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),sceneWork,scene))return false;
                FDistanceBudget proofWork{100};
                if(!TestTrue(TEXT("near scene never clears the complete loose envelope"),
                    scene->EmptyBoundsProof91(outer,proofWork)==(near?EEmptyBounds91::Near:EEmptyBounds91::Clear)))return false;
                if(!near)continue;
                // A near result keeps the entire old fine allocation. Partial
                // optional work is retained on the same account, never refunded.
                constexpr int32 fineWork=64,optionalWork=4;
                FDistanceBudget combined{fineWork+optionalWork,19,0};const uint64 begin=combined.Used;
                {
                    FScopedDistanceBudgetFloor reserve(combined,combined.Remaining-optionalWork);
                    if(!reserve.IsValid()||!combined.Consume()||!BuildGeneralNativeBoxLooseSweepVolume(v,full)||
                        scene->EmptyBoundsProof91(full,combined)!=EEmptyBounds91::Near)return false;
                }
                const uint64 spent=combined.Used-begin;
                if(!TestTrue(TEXT("optional near keeps original full fine reservation"),spent==2&&combined.Remaining>=fineWork))return false;
                FDistanceBudget old{fineWork,begin,0};FIntervalPoint expected,continued;
                {
                    FScopedDistanceBudgetFloor fine(combined,combined.Remaining-fineWork);
                    if(!fine.IsValid()||!old.Consume()||!BuildGeneralNativeBoxSweepVolume(v,expected)||
                        !combined.Consume()||!BuildGeneralNativeBoxSweepVolume(v,continued))return false;
                }
                if(!TestTrue(TEXT("original fine fallback is bit-identical with cumulative nonrefunded charges"),
                    same(expected,continued)&&combined.Used==old.Used+spent&&combined.Remaining==old.Remaining+optionalWork-int32(spent)))return false;
            }
        }
    }
    return !HasAnyErrors();
}
#endif
#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPredictionPrimitiveReuseBindingTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.PredictionPrimitiveReuseBinding",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPredictionPrimitiveReuseBindingTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;using namespace PreparedExecution;
    namespace P=PreparedExecution::PredictionPrimitiveReuseDetail;namespace A=::AnalyticCertified;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FNativeSolidStaticValues v;v.Shape=ENativeSolidQueryShape::Box;v.Horizon=1.f/300.f;
    v.OwnerState.Location=FVector(10,-20,30);v.OwnerState.Velocity=FVector(2,3,-1);v.OwnerState.Acceleration=FVector(.1,.2,.3);
    v.OwnerState.Rotation=FQuat(FVector::UpVector,.2);v.OwnerState.AngularVelocity=FVector(.03,.02,.01);
    v.OwnerState.AngularAcceleration=FVector(.004,.005,.006);v.BodyState=v.OwnerState;
    v.LocalCOM=FVector(1,2,3);v.LocalOffset=FVector(2,-1,4);v.LocalRotation=FQuat(FVector::ForwardVector,.1);
    v.PhysicalHalfExtent=FVector(1,2,3);v.QueryHalfExtent=FVector(1.1,2.1,3.1);
    v.Query.Shape=EQueryShape::Box;v.Query.Start=v.OwnerState.Location;v.Query.End=v.Query.Start+FVector(1,2,3);
    v.Query.Rotation=v.OwnerState.Rotation;v.Query.HalfExtent=v.QueryHalfExtent;
    const auto stateFields=[](FKinematicState& s)
    {return std::array<double*,19>{&s.Location.X,&s.Location.Y,&s.Location.Z,&s.Velocity.X,&s.Velocity.Y,&s.Velocity.Z,
        &s.Acceleration.X,&s.Acceleration.Y,&s.Acceleration.Z,&s.Rotation.X,&s.Rotation.Y,&s.Rotation.Z,&s.Rotation.W,
        &s.AngularVelocity.X,&s.AngularVelocity.Y,&s.AngularVelocity.Z,&s.AngularAcceleration.X,&s.AngularAcceleration.Y,&s.AngularAcceleration.Z};};
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        const uint64 numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        TestTrue(TEXT("exact source/current model/delta/numeric binding"),P::SameInputs(v,v.OwnerState,v.Horizon,v,numeric,numeric));
        bool changedStateRefused=true;
        for(int32 k=0;k<19;++k)
        {auto state=v.OwnerState;auto fields=stateFields(state);*fields[k]=A::FromBits(A::Bits(*fields[k])+1);
            changedStateRefused&=!P::SameInputs(v,state,v.Horizon,v,numeric,numeric);}
        TestTrue(TEXT("every current kinematic coefficient has exact identity"),changedStateRefused);
        bool changedCaptureRefused=true;
        for(int32 k=0;k<12;++k)
        {
            auto changed=v;
            switch(k)
            {
                case 0:changed.OwnerStableId++;break;case 1:changed.BodyIndex++;break;
                case 2:changed.LocalCOM.X+=1;break;case 3:changed.LocalOffset.Y+=1;break;
                case 4:changed.PhysicalHalfExtent.X+=1;break;case 5:changed.QueryHalfExtent.Y+=1;break;
                case 6:changed.Query.End.Z+=1;break;case 7:changed.Query.DomainTolerance+=.125;break;
                case 8:changed.Query.InitialOverlapTolerance+=.25;break;
                case 9:changed.Trace.TraceComplex=!changed.Trace.TraceComplex;break;
                case 10:changed.ResponseConfig.Restitution+=.5f;break;case 11:changed.BodyState.Velocity.X+=1;break;
            }
            changedCaptureRefused&=!P::SameInputs(changed,v.OwnerState,v.Horizon,v,numeric,numeric);
        }
        TestTrue(TEXT("source geometry, query, trace and response policy are fresh"),changedCaptureRefused);
        auto changedH=v;uint32 h=KinematicCenterDetail::Bits(v.Horizon)+1;FMemory::Memcpy(&changedH.Horizon,&h,sizeof(h));
        TestFalse(TEXT("different primitive delta"),P::SameInputs(v,v.OwnerState,changedH.Horizon,v,numeric,numeric));
        TestFalse(TEXT("different captured horizon"),P::SameInputs(changedH,v.OwnerState,v.Horizon,v,numeric,numeric));
        TestFalse(TEXT("different numeric context"),P::SameInputs(v,v.OwnerState,v.Horizon,v,numeric,numeric^uint64(0x40)));
        auto forbidden=v;forbidden.Frozen=true;
        TestFalse(TEXT("frozen cannot reuse even self-identical"),P::SameInputs(forbidden,forbidden.OwnerState,forbidden.Horizon,forbidden,numeric,numeric));
        forbidden=v;forbidden.Prescribed=true;
        TestFalse(TEXT("prescribed cannot reuse even self-identical"),P::SameInputs(forbidden,forbidden.OwnerState,forbidden.Horizon,forbidden,numeric,numeric));
        FNativeBoxPrimitiveGeometryValuesPtr rejected;FDistanceBudget empty{0};
        TestTrue(TEXT("zero allowance refuses before missing snapshot or prediction"),!TryBuildNativeBoxPrimitiveFromPrediction({},v,v.OwnerState,v.Horizon,{},empty,rejected)&&!rejected&&empty.Used==0);
        FDistanceBudget miss{8193};const int32 fine=8192;
        {FScopedDistanceBudgetFloor protectedFine(miss,fine);
            TestTrue(TEXT("missing prediction is a paid miss without consuming reserved fine work"),
                protectedFine.IsValid()&&!TryBuildNativeBoxPrimitiveFromPrediction({},v,v.OwnerState,v.Horizon,{},miss,rejected)&&
                !rejected&&miss.Remaining==fine&&miss.Used==1);}
        TestTrue(TEXT("miss leaves full original fine reserve and no floor"),miss.Remaining==fine&&miss.ConsumptionFloor==0);
    }
    TestTrue(TEXT("numeric controls restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPredictionPrimitiveReuseGeometryTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.PredictionPrimitiveReuseGeometry",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPredictionPrimitiveReuseGeometryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;using namespace PreparedExecution;
    namespace P=PreparedExecution::NativePredictionCoarseDetail;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FNativeSolidStaticValues v;v.Shape=ENativeSolidQueryShape::Box;v.Horizon=1.f/300.f;
    v.OwnerState.Location=FVector(10,-20,30);v.OwnerState.Velocity=FVector(2,3,-1);v.OwnerState.Acceleration=FVector(.1,.2,.3);
    v.OwnerState.Rotation=FQuat(FVector::UpVector,.2);v.OwnerState.AngularVelocity=FVector(.03,.02,.01);
    v.OwnerState.AngularAcceleration=FVector(.004,.005,.006);v.BodyState=v.OwnerState;
    v.LocalCOM=FVector(1,2,3);v.LocalOffset=FVector(2,-1,4);v.LocalRotation=FQuat(FVector::ForwardVector,.1);
    v.PhysicalHalfExtent=FVector(1,2,3);v.QueryHalfExtent=FVector(1.1,2.1,3.1);
    v.BodyState.Location=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    v.BodyState.Rotation=v.OwnerState.Rotation*v.LocalRotation;
    v.BodyState.Velocity=FVector(1000,0,0); // A separate, authenticated World chord remains covered.
    v.Query.Shape=EQueryShape::Box;v.Query.Start=v.BodyState.Location;v.Query.Rotation=v.BodyState.Rotation;
    v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    v.Query.HalfExtent=v.QueryHalfExtent;v.Query.DomainTolerance=.01;v.Query.InitialOverlapTolerance=.02;
    const auto prefix=[](const FGroupedBoxVolumeBounds& prediction,const FGroupedBoxVolumeBounds& primitive)
    {
        if(prediction.Groups.Num()<primitive.Groups.Num()||prediction.Axes.Num()!=primitive.Axes.Num())return false;
        for(int32 g=0;g<primitive.Groups.Num();++g)
        {
            if(prediction.Groups[g].Corners.Num()!=primitive.Groups[g].Corners.Num())return false;
            for(int32 k=0;k<primitive.Groups[g].Corners.Num();++k)
                if(!GroupedBoxDetail::SamePoint(prediction.Groups[g].Corners[k],primitive.Groups[g].Corners[k]))return false;
        }
        return true;
    };
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FGeneralBoxMotionInput input;input.Owner=v.OwnerState;input.Horizon=v.Horizon;input.LocalCOM=v.LocalCOM;input.LocalOffset=v.LocalOffset;
        input.LocalRotation=v.LocalRotation;input.PhysicalHalfExtent=v.PhysicalHalfExtent;input.QueryHalfExtent=v.QueryHalfExtent;
        FGeneralBoxMotionBounds whole;if(!TestTrue(TEXT("complete original actual model"),BuildGeneralBoxMotionBounds(input,whole)))return false;
        FGroupedBoxVolumeBounds coarsePrediction,coarsePrimitive,finePrediction,finePrimitive;
        FDistanceBudget b0{8192},b1{8192},b2{8192},b3{8192};
        if(!TestTrue(TEXT("both prediction producers retain their exact original primitive groups"),
            P::BuildVolume(v,b0,coarsePrediction)&&NativeGroupedBoxDetail::CoarsePrimitiveFromWhole(whole,
                v.OwnerState.Rotation*v.LocalRotation,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,b1,coarsePrimitive)&&
            BuildGeneralNativeBoxGroupedVolume(v,b2,finePrediction)&&BuildGeneralBoxPrimitiveGroupedVolume(input,
                v.OwnerState.Rotation*v.LocalRotation,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,b3,finePrimitive)&&
            coarsePrediction.Groups.Num()==8&&coarsePrimitive.Groups.Num()==4&&finePrediction.Groups.Num()==14&&finePrimitive.Groups.Num()==8&&
            prefix(coarsePrediction,coarsePrimitive)&&prefix(finePrediction,finePrimitive)))return false;
        TArray<FIntervalTensorSurface> surfaces;FIntervalTensorSurface far;far.ControlPoints.Add(Detail::Point(FVector3d(10000,10000,10000)));surfaces.Add(far);
        FDistanceBudget sb0{128},sb1{128};FIndexedIntervalUnionPtr scene,foreign;
        if(!FIndexedIntervalUnion::Build(surfaces,sb0,scene)||!FIndexedIntervalUnion::Build(surfaces,sb1,foreign))return false;
        for(const auto* volume:{&coarsePrediction,&finePrediction})
        {
            FDistanceBudget proofBudget{8192};FReferenceBoundsCandidateSetPtr partition;FReferenceBoxVolumeStrictSeparationPtr proof;
            if(!TestTrue(TEXT("one immutable complete prediction proof against exact scene"),
                FReferenceBoundsCandidateSet::Select(scene,volume->BroadHull,proofBudget,partition)&&
                FReferenceBoxVolumeStrictSeparation::TryProve(partition,*volume,proofBudget,proof)&&proof&&proof->Matches(scene,*volume)))return false;
            TestFalse(TEXT("identical coefficients in a different scene do not transfer proof"),proof->Matches(foreign,*volume));
            auto changed=*volume;changed.BroadHull.Axis[0].Upper+=1;
            TestFalse(TEXT("changed grouped volume cannot use retained proof"),proof->Matches(scene,changed));
            auto pose=v.Query;pose.End=pose.Start;
            TestTrue(TEXT("same fresh endpoint consumer accepts prediction's complete motion/pose cover"),
                GroupedBoxProofContainsEndpoint(proof,pose,v.PhysicalHalfExtent,v.QueryHalfExtent));
            pose.Start=pose.End=FVector(10000,10000,10000);
            TestFalse(TEXT("uncaptured endpoint does not obtain clearance"),GroupedBoxProofContainsEndpoint(proof,pose,v.PhysicalHalfExtent,v.QueryHalfExtent));
        }
        FDistanceBudget oldBudget{8192},fallbackBudget{8193};FGroupedBoxVolumeBounds oldVolume,fallbackVolume;
        FNativeBoxPrimitiveGeometryValuesPtr refused;
        {FScopedDistanceBudgetFloor reserve(fallbackBudget,8192);
            if(!reserve.IsValid()||TryBuildNativeBoxPrimitiveFromPrediction({},v,v.OwnerState,v.Horizon,{},fallbackBudget,refused))return false;}
        TestTrue(TEXT("failed reuse keeps literal original fine volume bits and debits its extra work once"),
            BuildGeneralBoxPrimitiveGroupedVolume(input,v.OwnerState.Rotation*v.LocalRotation,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,oldBudget,oldVolume)&&
            BuildGeneralBoxPrimitiveGroupedVolume(input,v.OwnerState.Rotation*v.LocalRotation,v.Query.DomainTolerance,v.Query.InitialOverlapTolerance,fallbackBudget,fallbackVolume)&&
            GroupedBoxDetail::Same(oldVolume,fallbackVolume)&&fallbackBudget.Used==oldBudget.Used+1&&fallbackBudget.Remaining==oldBudget.Remaining&&fallbackBudget.ConsumptionFloor==0);
    }
    TestTrue(TEXT("numeric controls restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
// Private reproduction of the sole-car Freeplay refusal, frame 2530.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGutterBox2530Observation,
    "IAmSpeed.AnalyticWorld.NativeSolid.GutterBox2530Observation",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGutterBox2530Observation::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace P=PreparedExecution::NativePredictionObliqueAxes;
    const auto original=CaptureFrameFloatingPointContext();
    FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!original.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
    PreparedExecution::FNativeSolidStaticValues v;
    v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;v.Horizon=1.f/300.f;
    v.OwnerState.Location=FVector(-14.50115676039159,3858.6899312328796,24.29588860683684);
    v.OwnerState.Rotation=FQuat(0.0098330259214597857,-0.0079171938728273654,0.68531016534852307,0.72814188652513867);
    v.OwnerState.Velocity=FVector(139.97044830616511,2264.6783783551227,129.14032139380689);
    v.OwnerState.Acceleration=FVector(38.651479400967794,-1300.3184660242107,3232.3972730479754);
    v.OwnerState.AngularVelocity=FVector(4.4545839396663531,0.062908419178704433,0.005794880483085435);
    v.OwnerState.AngularAcceleration=FVector(142.15695296732545,-4.078008450648702,0.10952418423952673);
    v.BodyState.Location=FVector(-13.93191087927277,3867.374711333453,36.611511238941681);
    v.BodyState.Rotation=FQuat(0.0098330259214597857,-0.0079171938728273654,0.68531016534852307,0.72814188652513867);
    v.BodyState.Velocity=FVector(140.69487739444759,2209.8207022830038,167.79159299085006);
    v.BodyState.Acceleration=FVector(-9.7735431517366429,-3223.1786428327919,4224.9068433634238);
    v.BodyState.AngularVelocity=FVector(4.4545839396663531,0.062908419178704433,0.005794880483085435);
    v.BodyState.AngularAcceleration=FVector(142.15695296732545,-4.078008450648702,0.10952418423952673);
    v.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);v.LocalOffset=FVector(-5,0,29.510000000000002);v.LocalRotation=FQuat(0,-0,0,1);
    v.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);v.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
    v.Query.Shape=EQueryShape::Box;v.Query.Start=v.BodyState.Location;
    v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    v.Query.Rotation=v.BodyState.Rotation;v.Query.HalfExtent=v.QueryHalfExtent;
    v.Query.DomainTolerance=1.e-6;v.Query.InitialOverlapTolerance=.05;
    FIntervalTensorSurface provider;provider.DegreeU=5;provider.DegreeV=5;
    // Exact interval endpoints from provider 1118 (C2_OUTER_HIGH_BODY_V1/XNYP/side/LowerArc1).
    const Interval points[][3]={
        {{-3599.9157452141471,-3599.9157452141467,true},{3937.9669586854629,3937.9669586854634,true},{19.486839677110588,19.486839677110591,true}},
        {{-3599.9157452141471,-3599.9157452141467,true},{3956.5426588591549,3956.5426588591554,true},{27.181146619630223,27.181146619630226,true}},
        {{-3599.9157452141471,-3599.9157452141467,true},{3974.3629722151886,3974.3629722151891,true},{36.699118662179806,36.699118662179814,true}},
        {{-3599.9157452141471,-3599.9157452141467,true},{3991.1891123396144,3991.1891123396149,true},{47.941986051958061,47.941986051958068,true}},
        {{-3599.9157452141471,-3599.9157452141467,true},{4006.8021105816492,4006.8021105816497,true},{60.763438614137058,60.763438614137065,true}},
        {{-3599.9157452141471,-3599.9157452141467,true},{4021.0193359837558,4021.0193359837563,true},{74.980664016243821,74.980664016243836,true}},
        {{-2879.932596171318,-2879.9325961713175,true},{3937.9669586854629,3937.9669586854634,true},{19.486839677110588,19.486839677110591,true}},
        {{-2879.932596171318,-2879.9325961713175,true},{3956.5426588591549,3956.5426588591554,true},{27.181146619630223,27.181146619630226,true}},
        {{-2879.932596171318,-2879.9325961713175,true},{3974.3629722151886,3974.3629722151891,true},{36.699118662179806,36.699118662179814,true}},
        {{-2879.932596171318,-2879.9325961713175,true},{3991.1891123396144,3991.1891123396149,true},{47.941986051958061,47.941986051958068,true}},
        {{-2879.932596171318,-2879.9325961713175,true},{4006.8021105816492,4006.8021105816497,true},{60.763438614137058,60.763438614137065,true}},
        {{-2879.932596171318,-2879.9325961713175,true},{4021.0193359837558,4021.0193359837563,true},{74.980664016243821,74.980664016243836,true}},
        {{-2159.9494471284884,-2159.9494471284879,true},{3937.9669586854629,3937.9669586854634,true},{19.486839677110588,19.486839677110591,true}},
        {{-2159.9494471284884,-2159.9494471284879,true},{3956.5426588591549,3956.5426588591554,true},{27.181146619630223,27.181146619630226,true}},
        {{-2159.9494471284884,-2159.9494471284879,true},{3974.3629722151886,3974.3629722151891,true},{36.699118662179806,36.699118662179814,true}},
        {{-2159.9494471284884,-2159.9494471284879,true},{3991.1891123396144,3991.1891123396149,true},{47.941986051958061,47.941986051958068,true}},
        {{-2159.9494471284884,-2159.9494471284879,true},{4006.8021105816492,4006.8021105816497,true},{60.763438614137058,60.763438614137065,true}},
        {{-2159.9494471284884,-2159.9494471284879,true},{4021.0193359837558,4021.0193359837563,true},{74.980664016243821,74.980664016243836,true}},
        {{-1439.966298085659,-1439.9662980856588,true},{3937.9669586854629,3937.9669586854634,true},{19.486839677110588,19.486839677110591,true}},
        {{-1439.966298085659,-1439.9662980856588,true},{3956.5426588591549,3956.5426588591554,true},{27.181146619630223,27.181146619630226,true}},
        {{-1439.966298085659,-1439.9662980856588,true},{3974.3629722151886,3974.3629722151891,true},{36.699118662179806,36.699118662179814,true}},
        {{-1439.966298085659,-1439.9662980856588,true},{3991.1891123396144,3991.1891123396149,true},{47.941986051958061,47.941986051958068,true}},
        {{-1439.966298085659,-1439.9662980856588,true},{4006.8021105816492,4006.8021105816497,true},{60.763438614137058,60.763438614137065,true}},
        {{-1439.966298085659,-1439.9662980856588,true},{4021.0193359837558,4021.0193359837563,true},{74.980664016243821,74.980664016243836,true}},
        {{-719.9831490428295,-719.98314904282938,true},{3937.9669586854629,3937.9669586854634,true},{19.486839677110588,19.486839677110591,true}},
        {{-719.9831490428295,-719.98314904282938,true},{3956.5426588591549,3956.5426588591554,true},{27.181146619630223,27.181146619630226,true}},
        {{-719.9831490428295,-719.98314904282938,true},{3974.3629722151886,3974.3629722151891,true},{36.699118662179806,36.699118662179814,true}},
        {{-719.9831490428295,-719.98314904282938,true},{3991.1891123396144,3991.1891123396149,true},{47.941986051958061,47.941986051958068,true}},
        {{-719.9831490428295,-719.98314904282938,true},{4006.8021105816492,4006.8021105816497,true},{60.763438614137058,60.763438614137065,true}},
        {{-719.9831490428295,-719.98314904282938,true},{4021.0193359837558,4021.0193359837563,true},{74.980664016243821,74.980664016243836,true}},
        {{0,0,true},{3937.9669586854629,3937.9669586854634,true},{19.486839677110588,19.486839677110591,true}},
        {{0,0,true},{3956.5426588591549,3956.5426588591554,true},{27.181146619630223,27.181146619630226,true}},
        {{0,0,true},{3974.3629722151886,3974.3629722151891,true},{36.699118662179806,36.699118662179814,true}},
        {{0,0,true},{3991.1891123396144,3991.1891123396149,true},{47.941986051958061,47.941986051958068,true}},
        {{0,0,true},{4006.8021105816492,4006.8021105816497,true},{60.763438614137058,60.763438614137065,true}},
        {{0,0,true},{4021.0193359837558,4021.0193359837563,true},{74.980664016243821,74.980664016243836,true}},
    };
    for(const auto& row:points){FIntervalPoint point;for(int32 d=0;d<3;++d)point.Axis[d]=row[d];provider.ControlPoints.Add(point);}
    FIndexedIntervalUnionPtr scene;FDistanceBudget make{100000};
    if(!TestTrue(TEXT("Exact blocking provider admitted"),FIndexedIntervalUnion::Build(MakeArrayView(&provider,1),make,scene)))return false;
    FGroupedBoxVolumeBounds volume;
    if(!TestTrue(TEXT("Captured original whole-time Box"),BuildGeneralNativeBoxGroupedVolume(v,make,volume)))return false;
    FReferenceBoundsCandidateSetPtr partition;
    if(!TestTrue(TEXT("Blocking provider selected"),FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,make,partition)&&
        partition&&partition->GetCandidateIndices().Num()==1))return false;
    FReferenceBoxVolumeStrictSeparationPtr baseProof,obliqueProof;
    FDistanceBudget baseWork{34588},obliqueWork{34588};FGroupedBoxSeparationObservation base;
    const bool baseClear=FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,baseWork,baseProof,&base);
    bool changed=false;
    const bool admitted=P::Augment(partition,v.BodyState.Location,obliqueWork,volume,changed);
    const bool obliqueClear=admitted&&changed&&FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,obliqueWork,obliqueProof);
    UE_LOG(LogTemp,Display,TEXT("[GutterBox2530Observation] BaseClear=%d BaseStage=%s BaseWork=%llu Augment=%d ExtraAxes=%d ObliqueClear=%d ObliqueWork=%llu"),
        int32(baseClear),base.Stage,baseWork.Used,int32(admitted),volume.Axes.Num()-6,int32(obliqueClear),obliqueWork.Used);
    TestTrue(TEXT("FP context restored"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativePredictionObliqueAxesTest,"IAmSpeed.AnalyticWorld.NativeSolid.ObliqueAxesStoredInCompleteVolume",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativePredictionObliqueAxesTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;namespace P=PreparedExecution::NativePredictionObliqueAxes;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FBoxVolumeGroup group;const double diagonal=FMath::Sqrt(.5);
        for(int32 i=0;i<8;++i){const double x=(i&1)?1.:-1.,y=(i&2)?1.:-1.;group.Corners.Add(Detail::Point(FVector3d((x-y)*diagonal,(x+y)*diagonal,(i&4)?1.:-1.)));}
        FIntervalPoint hull;for(const auto& point:group.Corners)hull=Detail::ValidPoint(hull)?TimeDetail::Hull(hull,point):point;
        TArray<FBoxVolumeGroup> groups{group};TArray<FVector3d> axes{FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector,FVector3d::ForwardVector,FVector3d::RightVector,FVector3d::UpVector};
        FGroupedBoxVolumeBounds base;FDistanceBudget close{4096};if(!CloseGroupedBoxVolumeBounds(hull,groups,axes,close,base))return false;
        FIntervalTensorSurface patch;patch.DegreeU=patch.DegreeV=1;
        for(double x:{.3,1.3})for(double z:{-2.,2.})patch.ControlPoints.Add(Detail::Point(FVector3d(x,1.6-x,z)));
        FIndexedIntervalUnionPtr scene;FDistanceBudget build{4096};FReferenceBoundsCandidateSetPtr partition;
        if(!FIndexedIntervalUnion::Build(MakeArrayView(&patch,1),build,scene)||!FReferenceBoundsCandidateSet::Select(scene,base.BroadHull,build,partition))return false;
        FReferenceBoxVolumeStrictSeparationPtr absent;FDistanceBudget oldWork{512};
        if(!TestFalse(TEXT("six supplied axes cannot certify the larger region through the patch"),FReferenceBoxVolumeStrictSeparation::TryProve(partition,base,oldWork,absent)))return false;
        auto augmented=base;bool changed=false;FDistanceBudget propose{2048,11,1000};
        if(!TestTrue(TEXT("one finite candidate proposal is stored by a complete reclose"),P::Augment(partition,FVector3d::ZeroVector,propose,augmented,changed)&&changed&&augmented.Axes.Num()>base.Axes.Num()&&augmented.Axes.Num()<=12&&GroupedBoxDetail::SamePoint(augmented.BroadHull,base.BroadHull)&&augmented.Groups.Num()==base.Groups.Num()))return false;
        if(!TestTrue(TEXT("same account and inherited reserve, no refund"),propose.Remaining>=1000&&propose.Used==uint64(2059-propose.Remaining)&&propose.ConsumptionFloor==1000))return false;
        FDistanceBudget ownedBuild{4096};FClosedGroupedBoxVolumePtr owned;
        if(!FClosedGroupedBoxVolume::Build(hull,groups,axes,ownedBuild,owned))return false;
        const auto oldOwner=owned;auto ownedAugmented=base;bool ownedChanged=false;FDistanceBudget ownedWork{2048,11,1000};
        if(!TestTrue(TEXT("actual ObliqueAugment replaces the owner only after complete new closure"),
            P::Augment(partition,FVector3d::ZeroVector,ownedWork,ownedAugmented,ownedChanged,&owned)&&ownedChanged&&owned&&owned!=oldOwner&&
            GroupedBoxDetail::Same(ownedAugmented,augmented)&&GroupedBoxDetail::Same(owned->GetVolumeForInspection(),augmented)&&
            GroupedBoxDetail::Same(oldOwner->GetVolumeForInspection(),base)&&ownedWork.Used==propose.Used&&ownedWork.Remaining==propose.Remaining))return false;
        FDistanceBudget ownedProofWork{2048};FReferenceBoxVolumeStrictSeparationPtr ownedProof;
        if(!TestTrue(TEXT("augmented owner carries every new slab into the real proof"),
            FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,owned,ownedProofWork,ownedProof)&&ownedProof&&
            ownedProof->Matches(scene,augmented)&&!ownedProof->Matches(scene,base)))return false;
        FReferenceBoxVolumeStrictSeparationPtr proof;FDistanceBudget newWork{2048};
        if(!TestTrue(TEXT("new full volume has its own exact separation proof"),FReferenceBoxVolumeStrictSeparation::TryProve(partition,augmented,newWork,proof)&&proof&&proof->Matches(scene,augmented)&&!proof->Matches(scene,base)))return false;
        if(!TestTrue(TEXT("complete original endpoint still fits its exact stored region"),proof->ContainsCompleteEndpointGroup(group.Corners)))return false;
        TArray<FIntervalPoint> escape;for(int32 i=0;i<8;++i)escape.Add(Detail::Point(FVector3d(.8+((i&1)?.02:-.02),.8+((i&2)?.02:-.02),(i&4)?.02:-.02)));
        if(!TestTrue(TEXT("adversarial endpoint remains inside only the old broad box"),GroupedBoxDetail::Within(base.BroadHull,escape[0])))return false;
        if(!TestFalse(TEXT("new oblique slab is enforced downstream, not discarded after proof"),proof->ContainsCompleteEndpointGroup(escape)))return false;
        if(!TestFalse(TEXT("owned closure also preserves the new oblique endpoint obligation"),ownedProof->ContainsCompleteEndpointGroup(escape)))return false;
        auto low=base;FDistanceBudget limited{767};bool limitedChanged=true;
        if(!TestTrue(TEXT("absent optional allowance leaves the old value and debit intact"),P::Augment(partition,FVector3d::ZeroVector,limited,low,limitedChanged)&&!limitedChanged&&limited.Used==0&&GroupedBoxDetail::Same(low,base)))return false;
        TArray<FVector3d> cappedAxes=axes;while(cappedAxes.Num()<12)cappedAxes.Add(FVector3d::ForwardVector);
        FGroupedBoxVolumeBounds capped;FDistanceBudget capBuild{4096};if(!CloseGroupedBoxVolumeBounds(hull,groups,cappedAxes,capBuild,capped))return false;
        const auto expected=capped;FDistanceBudget cappedWork{2048};bool capChanged=true;
        if(!TestTrue(TEXT("maximum12 is retained without replacing any old axis"),P::Augment(partition,FVector3d::ZeroVector,cappedWork,capped,capChanged)&&!capChanged&&cappedWork.Used==0&&GroupedBoxDetail::Same(capped,expected)))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace GeneralBoxPrologueReuseTests
{
using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
namespace A=::AnalyticCertified;
static FGeneralBoxMotionInput Input(int32 sample=0)
{
    const double k=double(sample+1);FGeneralBoxMotionInput v;
    v.Owner.Location=FVector(10*k,-7*k,15*k);v.Owner.Velocity=FVector(2*k,-3*k,1.1*k);
    v.Owner.Acceleration=FVector(.2*k,.1*k,-.3*k);
    v.Owner.Rotation=FQuat(FVector(1,2,3).GetSafeNormal(),.09*k);
    v.Owner.AngularVelocity=FVector(.07*k,-.03*k,.02*k);
    v.Owner.AngularAcceleration=FVector(.004*k,.005*k,-.006*k);
    v.LocalCOM=FVector(1,-2,3);v.LocalOffset=FVector(-2,1,4);
    v.LocalRotation=FQuat(FVector(2,-1,1).GetSafeNormal(),-.07*k);
    v.PhysicalHalfExtent=FVector(1,2,3);v.QueryHalfExtent=FVector(1.2,1.8,3.1);
    v.Horizon=float(double(1+sample%4)/300.);return v;
}
static bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{for(int32 d=0;d<3;++d)if(!GroupedBoxDetail::SameInterval(a.Axis[d],b.Axis[d]))return false;return true;}
static bool SameFamily(const GeneralBoxPrologueReuse::FFamily& a,const GeneralBoxPrologueReuse::FFamily& b)
{
    if(!SamePoint(a.Center,b.Center))return false;
    for(int32 d=0;d<4;++d)if(!GroupedBoxDetail::SameInterval(a.Rotation.Axis[d].Range,b.Rotation.Axis[d].Range)||
        A::Bits(a.Rotation.Axis[d].Error)!=A::Bits(b.Rotation.Axis[d].Error))return false;return true;
}
static bool SameWhole(const FGeneralBoxMotionBounds& a,const FGeneralBoxMotionBounds& b)
{
    if(!SamePoint(a.Hull,b.Hull)||!SamePoint(a.NativeCenter,b.NativeCenter)||!SamePoint(a.ReferenceCenter,b.ReferenceCenter)||a.Corners.Num()!=b.Corners.Num())return false;
    for(int32 i=0;i<a.Corners.Num();++i)if(!SamePoint(a.Corners[i],b.Corners[i]))return false;return true;
}
static bool SameLoose(const NativeLooseWholeDetail::FLooseMotionBounds& a,const NativeLooseWholeDetail::FLooseMotionBounds& b)
{return SamePoint(a.Hull,b.Hull)&&SamePoint(a.NativeCenter,b.NativeCenter)&&SamePoint(a.ReferenceCenter,b.ReferenceCenter);}
static Speed::PreparedExecution::FNativeSolidStaticValues Native(const FGeneralBoxMotionInput& input)
{
    Speed::PreparedExecution::FNativeSolidStaticValues v;v.Shape=Speed::PreparedExecution::ENativeSolidQueryShape::Box;
    v.OwnerState=input.Owner;v.BodyState=input.Owner;v.LocalCOM=input.LocalCOM;v.LocalOffset=input.LocalOffset;v.LocalRotation=input.LocalRotation;
    v.Horizon=input.Horizon;v.PhysicalHalfExtent=input.PhysicalHalfExtent;v.QueryHalfExtent=input.QueryHalfExtent;
    v.BodyState.Location=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    v.BodyState.Rotation=v.OwnerState.Rotation*v.LocalRotation;
    v.BodyState.Velocity=FVector(1000,-500,200);v.BodyState.Acceleration=FVector(3,-7,11);
    v.Query.Shape=EQueryShape::Box;v.Query.HalfExtent=v.QueryHalfExtent;v.Query.Start=v.BodyState.Location;
    v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    v.Query.Rotation=v.BodyState.Rotation;v.Query.DomainTolerance=.003;v.Query.InitialOverlapTolerance=.007;return v;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGeneralBoxPrologueReuseParityTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.GeneralBoxPrologueReuseParity",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGeneralBoxPrologueReuseParityTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    using namespace GeneralBoxPrologueReuseTests;namespace M=GeneralBoxPrologueReuse;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxPrologueReuse"),1,FName(TEXT("PrologueParity")));
    FScopedCVar<int32> wholeMemo(TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxMotionValueMemo"),0,FName(TEXT("PrologueParity")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(int32 sample=0;sample<8;++sample)
        {
            auto input=GeneralBoxPrologueReuseTests::Input(sample);
            if(sample&1)input.Owner.Rotation=FQuat(-input.Owner.Rotation.X,-input.Owner.Rotation.Y,-input.Owner.Rotation.Z,-input.Owner.Rotation.W);
            if(sample&2)input.LocalRotation=FQuat(-input.LocalRotation.X,-input.LocalRotation.Y,-input.LocalRotation.Z,-input.LocalRotation.W);
            NativeLooseWholeDetail::FLooseMotionBounds looseOracle,looseCached;
            FGeneralBoxMotionBounds oracle,cold,warm;
            if(!TestTrue(TEXT("original exact general and loose models known"),BuildGeneralBoxMotionBounds(input,oracle)&&NativeLooseWholeDetail::BuildMotion(input,looseOracle,false)))return false;
            M::Cache()={};
            if(!TestTrue(TEXT("cold General route agrees with unchanged header oracle"),M::BuildGeneral(input,cold)&&SameWhole(cold,oracle)&&M::Cache().Misses==2))return false;
            M::Cache()={};
            if(!TestTrue(TEXT("Loose seeds two independent consumed/reference families"),NativeLooseWholeDetail::BuildMotion(input,looseCached)&&SameLoose(looseCached,looseOracle)&&M::Cache().Misses==2&&M::Cache().Stores==2))return false;
            const auto hits=M::Cache().Hits,cross=M::Cache().CrossRouteHits;
            if(!TestTrue(TEXT("General reuses both Loose families with all 32 corners bit-identical"),BuildGeneralBoxMotionBoundsMemo(input,warm,0)&&SameWhole(warm,oracle)&&warm.Corners.Num()==32&&M::Cache().Hits==hits+2&&M::Cache().CrossRouteHits==cross+2))return false;
            auto changed=input;changed.Owner.AngularAcceleration.Y=A::FromBits(A::Bits(changed.Owner.AngularAcceleration.Y)+1);
            FGeneralBoxMotionBounds changedOracle,changedCached;const auto misses=M::Cache().Misses;
            if(!TestTrue(TEXT("changed true motion replaces both families and matches original bits"),BuildGeneralBoxMotionBounds(changed,changedOracle)&&M::BuildGeneral(changed,changedCached)&&SameWhole(changedOracle,changedCached)&&M::Cache().Misses==misses+2))return false;
            if(sample!=0)continue;
            auto native=Native(input);FIntervalPoint fullOracle,fullCached,loose;
            if(!TestTrue(TEXT("native route preserves chord pose footprints and shells"),BuildGeneralNativeBoxLooseSweepVolume(native,loose)&&BuildGeneralNativeBoxSweepVolume(native,fullOracle)&&BuildGeneralNativeBoxSweepVolumeMemo(native,fullCached)&&SamePoint(fullOracle,fullCached)))return false;
            for(int32 guard=0;guard<6;++guard)
            {
                auto bad=native;
                if(guard==0)bad.Query.End.X+=1;
                if(guard==1)bad.Frozen=true;
                if(guard==2)bad.Prescribed=true;
                if(guard==3)bad.Query.HalfExtent.X+=1;
                if(guard==4)bad.Query.Rotation=FQuat::Identity;
                if(guard==5)bad.BodyState.Location.X+=1;
                if(!TestTrue(TEXT("native guards remain fresh despite warm prologue"),!BuildGeneralNativeBoxSweepVolume(bad,fullOracle)&&!BuildGeneralNativeBoxSweepVolumeMemo(bad,fullCached)&&!BuildGeneralNativeBoxLooseSweepVolume(bad,loose)))return false;
            }
            FDistanceBudget oldWork{8192,17,11},newWork{8192,17,11};FGroupedBoxVolumeBounds oldGrouped,newGrouped;
            const auto axes=input.Owner.Rotation*input.LocalRotation;
            const bool oldKnown=BuildGeneralBoxPrimitiveGroupedVolume(input,axes,.003,.007,oldWork,oldGrouped);
            const bool newKnown=BuildGeneralBoxPrimitiveGroupedVolumeMemo(input,axes,.003,.007,newWork,newGrouped);
            if(!TestTrue(TEXT("cache never credits or changes logical work and floors"),oldKnown==newKnown&&oldWork.Remaining==newWork.Remaining&&oldWork.Used==newWork.Used&&oldWork.ConsumptionFloor==newWork.ConsumptionFloor&&(!oldKnown||SamePoint(oldGrouped.BroadHull,newGrouped.BroadHull))))return false;
        }
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGeneralBoxPrologueReuseGuardsTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.GeneralBoxPrologueReuseKeyStickyAndFailure",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGeneralBoxPrologueReuseGuardsTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    using namespace GeneralBoxPrologueReuseTests;namespace M=GeneralBoxPrologueReuse;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.GeneralBoxPrologueReuse"),1,FName(TEXT("PrologueGuards")));
    const auto input=GeneralBoxPrologueReuseTests::Input();
    const auto fields=[](FGeneralBoxMotionInput& v)
    {
        auto& o=v.Owner;return std::array<double*,35>{&o.Location.X,&o.Location.Y,&o.Location.Z,
            &o.Velocity.X,&o.Velocity.Y,&o.Velocity.Z,&o.Acceleration.X,&o.Acceleration.Y,&o.Acceleration.Z,
            &o.Rotation.X,&o.Rotation.Y,&o.Rotation.Z,&o.Rotation.W,&o.AngularVelocity.X,&o.AngularVelocity.Y,&o.AngularVelocity.Z,
            &o.AngularAcceleration.X,&o.AngularAcceleration.Y,&o.AngularAcceleration.Z,
            &v.LocalCOM.X,&v.LocalCOM.Y,&v.LocalCOM.Z,&v.LocalOffset.X,&v.LocalOffset.Y,&v.LocalOffset.Z,
            &v.LocalRotation.X,&v.LocalRotation.Y,&v.LocalRotation.Z,&v.LocalRotation.W,
            &v.PhysicalHalfExtent.X,&v.PhysicalHalfExtent.Y,&v.PhysicalHalfExtent.Z,&v.QueryHalfExtent.X,&v.QueryHalfExtent.Y,&v.QueryHalfExtent.Z};
    };
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        const auto fingerprint=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
        const auto key=GeneralBoxMotionMemo::Key(input,fingerprint);bool complete=true;
        for(int32 field=0;field<35;++field){auto changed=input;auto values=fields(changed);*values[field]=A::FromBits(A::Bits(*values[field])+1);complete&=!(key==GeneralBoxMotionMemo::Key(changed,fingerprint));}
        auto changedH=input;const uint32 changedBits=KinematicCenterDetail::Bits(input.Horizon)+1;FMemory::Memcpy(&changedH.Horizon,&changedBits,sizeof(changedBits));
        complete&=!(key==GeneralBoxMotionMemo::Key(changedH,fingerprint));complete&=!(key==GeneralBoxMotionMemo::Key(input,fingerprint^0x40));
        auto sign=input;sign.LocalOffset.X=0.;auto minusZero=sign;minusZero.LocalOffset.X=A::FromBits(A::Sign);
        complete&=!(GeneralBoxMotionMemo::Key(sign,fingerprint)==GeneralBoxMotionMemo::Key(minusZero,fingerprint));
        if(!TestTrue(TEXT("every input word horizon numeric mode and signed zero discriminate"),complete))return false;
        FZeroOffsetKinematicCenterReference center;
        if(!BuildZeroOffsetKinematicCenterReference(input.Owner.Location,input.Owner.Velocity,input.Owner.Acceleration,input.Horizon,FVector::ZeroVector,FVector::ZeroVector,center))return false;
        for(bool consumed:{true,false})
        {
            const uint32 control=VectorGetControlRegister()&~uint32(0x3f);M::Cache()={};
            VectorSetControlRegister(control);M::FFamily oracle,cold,rebuilt,hit;
            if(!M::BuildFamilyOriginal(input,center,consumed,oracle))return false;
            const uint32 expectedFlags=VectorGetControlRegister()&0x3f;
            VectorSetControlRegister(control);
            if(!TestTrue(TEXT("cold family matches every quaternion range/error bit and FP flags"),M::BuildFamily(input,center,consumed,cold,1)&&SameFamily(oracle,cold)&&(VectorGetControlRegister()&0x3f)==expectedFlags))return false;
            if(!TestTrue(TEXT("fixture actually generates sticky arithmetic flags"),expectedFlags!=0))return false;
            VectorSetControlRegister(control);const auto misses=M::Cache().Misses,sticky=M::Cache().StickyMisses;
            if(!TestTrue(TEXT("cleared sticky flags force original arithmetic without changing result"),M::BuildFamily(input,center,consumed,rebuilt,0)&&M::Cache().Misses==misses+1&&M::Cache().StickyMisses==sticky+1&&SameFamily(oracle,rebuilt)&&(VectorGetControlRegister()&0x3f)==expectedFlags))return false;
            const auto hits=M::Cache().Hits;
            if(!TestTrue(TEXT("ready family hits once required flags already exist"),M::BuildFamily(input,center,consumed,hit,0)&&M::Cache().Hits==hits+1&&SameFamily(oracle,hit)&&(VectorGetControlRegister()&0x3f)==expectedFlags))return false;
            // Nested calls cannot mutate the outer single-entry storage.
            M::Cache().Busy=true;const auto nestedHits=M::Cache().Hits,nestedMisses=M::Cache().Misses;
            const bool nested=M::BuildFamily(input,center,consumed,hit,0);M::Cache().Busy=false;
            if(!TestTrue(TEXT("nested operation uses private fresh values without replacement"),nested&&SameFamily(oracle,hit)&&M::Cache().Hits==nestedHits&&M::Cache().Misses==nestedMisses))return false;
        }
        FGeneralBoxMotionBounds valid;if(!M::BuildGeneral(input,valid))return false;
        for(int32 guard=0;guard<5;++guard)
        {
            auto bad=input;
            if(guard==0)bad.PhysicalHalfExtent.X=0;
            if(guard==1)bad.Owner.Rotation.W=7;
            if(guard==2)bad.LocalRotation.W=7;
            if(guard==3)bad.Owner.Velocity.X=A::FromBits(A::Infinity|1);
            if(guard==4)bad.Horizon=-1.f;
            FGeneralBoxMotionBounds oldBad,newBad;NativeLooseWholeDetail::FLooseMotionBounds loose;
            const auto stores=M::Cache().Stores;
            if(!TestTrue(TEXT("invalid inputs cannot publish or reuse a valid model"),!BuildGeneralBoxMotionBounds(bad,oldBad)&&!M::BuildGeneral(bad,newBad)&&!NativeLooseWholeDetail::BuildMotion(bad,loose)&&M::Cache().Stores==stores&&!Detail::ValidPoint(newBad.Hull)&&newBad.Corners.Num()==0))return false;
        }
        // Repeating exactly one input under another valid control mode must miss.
        const auto misses=M::Cache().Misses;
        VectorSetControlRegister(VectorGetControlRegister()^uint32(0x40));
        FGeneralBoxMotionBounds otherMode,otherOracle;
        if(!TestTrue(TEXT("FTZ/DAZ input identity never crosses control mode"),M::BuildGeneral(input,otherMode)&&BuildGeneralBoxMotionBounds(input,otherOracle)&&SameWhole(otherMode,otherOracle)&&M::Cache().Misses==misses+2))return false;
        const uint32 good=VectorGetControlRegister();VectorSetControlRegister(good|uint32(0x2000));bool refused=false;
        try{FGeneralBoxMotionBounds bad;M::BuildGeneral(input,bad);}catch(const FAnalyticNumericUnknown&){refused=true;}
        VectorSetControlRegister(good);if(!TestTrue(TEXT("unsupported rounding remains Unknown even for warm input"),refused))return false;
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace InstantMotionClearanceTests
{
using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
namespace A=::AnalyticCertified;namespace X=InstantMotionClearanceDetail;
static bool Contains(const FIntervalPoint& bounds,const FVector3d& point)
{for(int32 d=0;d<3;++d)if(!A::Valid(bounds.Axis[d])||point[d]<bounds.Axis[d].Lower||point[d]>bounds.Axis[d].Upper)return false;return true;}
static bool Contains(const FQuat4d& q0,const X::FQuaternionDelta& bounds,const FQuat4d& point)
{
    const double base[4]={q0.X,q0.Y,q0.Z,q0.W},value[4]={point.X,point.Y,point.Z,point.W};
    for(int32 d=0;d<4;++d){const auto full=A::Add(A::Interval::Exact(base[d]),bounds.Axis[d]);
        if(!A::Valid(full)||value[d]<full.Lower||value[d]>full.Upper)return false;}return true;
}
static PreparedExecution::FNativeSolidStaticValues Native(int32 index)
{
    PreparedExecution::FNativeSolidStaticValues v;const double k=double(index+1);
    v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;v.OwnerState.Location=FVector(100,-200,300);
    v.OwnerState.Velocity=FVector(30,-40,5);v.OwnerState.Acceleration=FVector(3,-4,-9);
    v.OwnerState.Rotation=FQuat(FVector(1,2,3).GetSafeNormal(),.03*k);
    if(index&1)v.OwnerState.Rotation=FQuat(-v.OwnerState.Rotation.X,-v.OwnerState.Rotation.Y,-v.OwnerState.Rotation.Z,-v.OwnerState.Rotation.W);
    v.OwnerState.AngularVelocity=FVector(.007*k,-.003*k,.2*k);
    v.OwnerState.AngularAcceleration=FVector(.004*k,.005*k,-.006*k);
    if(index==0){v.OwnerState.AngularVelocity=FVector::ZeroVector;v.OwnerState.AngularAcceleration=FVector::ZeroVector;}
    if(index>=8)
    {
        const FVector axis=(index&1)?FVector(0,1,0):FVector(1,0,0);
        v.OwnerState.Rotation=FQuat(axis,(index&1)?UE_PI:UE_PI*.5);
        if(index==10||index==11)
        {
            const auto q=v.OwnerState.Rotation;const double scale=1.+1.e-5;
            v.OwnerState.Rotation=FQuat(q.X*scale,q.Y*scale,q.Z*scale,q.W*scale);
        }
        v.OwnerState.AngularVelocity=FVector(5.5,.007,-.25);
    }
    v.LocalCOM=FVector(1,-2,3);v.LocalOffset=FVector(-2,1,4);
    v.LocalRotation=FQuat(FVector(2,-1,1).GetSafeNormal(),-.07*k);
    v.PhysicalHalfExtent=FVector(50,25,14);v.QueryHalfExtent=FVector(49,26,15);v.Horizon=float(double(1+index%4)/300.);
    if(index>=8)v.Horizon=1.f/300.f;if(index==13)v.Horizon=0.f;
    v.BodyState=v.OwnerState;
    v.BodyState.Location=(FVector(v.OwnerState.Location)-v.OwnerState.Rotation.RotateVector(v.LocalCOM))+v.OwnerState.Rotation.RotateVector(v.LocalOffset);
    v.BodyState.Rotation=v.OwnerState.Rotation*v.LocalRotation;
    // A deliberately different literal World chord prevents accidentally
    // proving only the owner COM trajectory.
    v.BodyState.Velocity=FVector(1000,-500,200);v.BodyState.Acceleration=FVector(3,-7,11);
    v.Query.Shape=EQueryShape::Box;v.Query.HalfExtent=v.QueryHalfExtent;v.Query.Start=v.BodyState.Location;
    v.Query.End=v.BodyState.Location+v.BodyState.Velocity*v.Horizon+0.5f*v.BodyState.Acceleration*v.Horizon*v.Horizon;
    v.Query.Rotation=v.BodyState.Rotation;v.Query.DomainTolerance=.003;v.Query.InitialOverlapTolerance=.007;return v;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInstantMotionRotationDifferenceTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.InstantMotionRotationDifference",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInstantMotionRotationDifferenceTest::RunTest(const FString&)
{
    using namespace InstantMotionClearanceTests;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(int32 sample=0;sample<16;++sample)
        {
            // All arithmetic of this bounded polynomial oracle is dyadic
            // and exactly representable, including deliberately nonunit q.
            const FQuat4d q0(double(sample-7)/16.,double(3-sample)/16.,double(sample%5)/8.,double(9-sample)/16.);
            const double changes[4]={double(sample%3-1)/256.,double(sample%5-2)/256.,double(sample%7-3)/256.,1./256.};
            X::FQuaternionDelta delta;for(int32 d=0;d<4;++d)delta.Axis[d]=A::Interval::Exact(changes[d]);
            const FQuat4d q(q0.X+changes[0],q0.Y+changes[1],q0.Z+changes[2],q0.W+changes[3]);
            X::FRotationDeltaMatrix matrix;FIntervalPoint bounds;
            if(!TestTrue(TEXT("nonunit exact polynomial difference admitted"),X::RotationDeltaMatrix(q0,delta,matrix)&&X::RotationDeltaSupport(matrix,FVector3d(1,2,3),bounds)))return false;
            for(int32 corner=0;corner<8;++corner)
            {
                const FVector3d p((corner&1)?1.:-1.,(corner&2)?2.:-2.,(corner&4)?3.:-3.);
                if(!TestTrue(TEXT("all exact dyadic corner differences enclosed"),Contains(bounds,q.RotateVector(p)-q0.RotateVector(p))))return false;
            }
        }
        X::FQuaternionDelta yaw;for(int32 d=0;d<4;++d)yaw.Axis[d]=A::Interval::Exact(d==2?1./16.:0.);
        X::FRotationDeltaMatrix matrix;FIntervalPoint bounds;
        if(!TestTrue(TEXT("pure yaw has no geometric vertical movement, without asserting a unit quaternion"),
            X::RotationDeltaMatrix(FQuat4d::Identity,yaw,matrix)&&X::RotationDeltaSupport(matrix,FVector3d(50,25,14),bounds)&&
            bounds.Axis[2].Lower<=0&&bounds.Axis[2].Upper>=0&&bounds.Axis[2].Upper<1.e-100))return false;
        yaw.Axis[0].Known=false;
        if(!TestFalse(TEXT("unknown difference fails closed"),X::RotationDeltaMatrix(FQuat4d::Identity,yaw,matrix)))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInstantMotionNativeFamiliesTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.InstantMotionNativeFamiliesAndGuards",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInstantMotionNativeFamiliesTest::RunTest(const FString&)
{
    using namespace InstantMotionClearanceTests;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.InstantMotionClearance"),1,FName(TEXT("InstantMotionNativeFamilies")));
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x807f))|flush);X::FStaticNumericalBounds constants;
        if(!X::GetStaticNumericalBounds(constants))return false;
        const uint32 required=constants.RequiredSticky;
        if(!TestTrue(TEXT("error/trig table actually generates sticky flags"),required!=0))return false;
        VectorSetControlRegister(VectorGetControlRegister()&~uint32(0x3f));X::FStaticNumericalBounds rebuilt;
        if(!TestTrue(TEXT("cleared sticky flags rebuild the numeric table rather than borrowing its old effects"),
            X::GetStaticNumericalBounds(rebuilt)&&rebuilt.Numeric==constants.Numeric&&rebuilt.RequiredSticky==required&&
            (VectorGetControlRegister()&required)==required))return false;
        for(int32 sample=0;sample<14;++sample)
        {
            const auto v=Native(sample);X::FQuaternionDelta delta;FIntervalPoint hull;
            if(!TestTrue(TEXT("wall ceiling and slightly nonunit captures satisfy the REAL normalized predicate"),
                v.OwnerState.Rotation.IsNormalized()&&v.BodyState.Rotation.IsNormalized()))return false;
            if(v.Horizon==0.f)
            {
                if(!TestTrue(TEXT("zero horizon preserves original model nonadmission and its literal ThetaZero value"),
                    X::BoundOwnerQuaternionDelta(v.OwnerState,0.f,constants,delta)&&Contains(v.OwnerState.Rotation,delta,v.OwnerState.Rotation)&&
                    !X::BuildCompleteVolume(v,hull)&&!BuildGeneralNativeBoxLooseSweepVolume(v,hull)&&!Detail::ValidPoint(hull)))return false;
                continue;
            }
            if(!TestTrue(TEXT("complete current native and reference model admitted"),
                X::BoundOwnerQuaternionDelta(v.OwnerState,v.Horizon,constants,delta)&&X::BuildCompleteVolume(v,hull)))return false;
            FIntervalPoint routed;if(!BuildGeneralNativeBoxLooseSweepVolume(v,routed))return false;
            for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("production dispatch uses the new complete geometric model when enabled"),
                routed.Axis[d].Known==hull.Axis[d].Known&&A::Bits(routed.Axis[d].Lower)==A::Bits(hull.Axis[d].Lower)&&
                A::Bits(routed.Axis[d].Upper)==A::Bits(hull.Axis[d].Upper)))return false;
            Private::FBoxSweepContext sat;sat.Initialize(v.Query);
            for(int32 step=0;step<=16;++step)
            {
                const float t=float(double(v.Horizon)*double(step)/16.);
                const auto ownerQ=::Speed::SBox::IntegrateRotation(v.OwnerState.Rotation,v.OwnerState.AngularVelocity,v.OwnerState.AngularAcceleration,t);
                if(!TestTrue(TEXT("literal native normalization and ThetaZero included"),Contains(v.OwnerState.Rotation,delta,ownerQ)))return false;
                const auto ownerP=::Speed::SBox::AdvancePosition(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,t);
                const auto bodyP=(ownerP-ownerQ.RotateVector(v.LocalCOM))+ownerQ.RotateVector(v.LocalOffset);
                const auto bodyQ=ownerQ*v.LocalRotation;
                const double td=double(t);
                const auto theta=v.OwnerState.AngularVelocity*td+.5*v.OwnerState.AngularAcceleration*(td*td);
                FQuat referenceOwner=v.OwnerState.Rotation;
                if(!theta.IsZero())
                {
                    const double x=theta.SizeSquared(),angle=FMath::Sqrt(x);
                    const double scale=x<1.e-8?.5-x/48.+x*x/3840.:FMath::Sin(angle*.5)/angle;
                    const double cosine=FMath::Cos(angle*.5);
                    if(!GeneralBoxDetail::CheckedNativeExpResults(x,scale,cosine))return false;
                    referenceOwner=FQuat(theta.X*scale,theta.Y*scale,theta.Z*scale,cosine)*v.OwnerState.Rotation;
                }
                if(!TestTrue(TEXT("raw and normalized reference exponential remain in the quaternion family"),
                    Contains(v.OwnerState.Rotation,delta,referenceOwner)&&
                    Contains(v.OwnerState.Rotation,delta,referenceOwner.GetNormalized())))return false;
                const auto referenceP=v.OwnerState.Location+v.OwnerState.Velocity*td+.5*v.OwnerState.Acceleration*(td*td);
                const auto referenceBodyP=(referenceP-referenceOwner.RotateVector(v.LocalCOM))+referenceOwner.RotateVector(v.LocalOffset);
                const auto referenceBodyQ=referenceOwner*v.LocalRotation;
                const auto chordP=v.Query.Start+(v.Query.End-v.Query.Start)*(double(step)/16.);
                for(int32 representation=0;representation<2;++representation)for(int32 corner=0;corner<8;++corner)
                {
                    const auto extent=representation?v.QueryHalfExtent:v.PhysicalHalfExtent;
                    FVector3d p;for(int32 d=0;d<3;++d)p[d]=(corner&(1<<d))?extent[d]:-extent[d];
                    const auto fixed=v.Query.Rotation.RotateVector(p);
                    const auto axes=sat.Axis[0]*p.X+sat.Axis[1]*p.Y+sat.Axis[2]*p.Z;
                    if(!TestTrue(TEXT("native corners plus literal chord, captured pose and moving SAT footprints remain enclosed"),
                        Contains(hull,bodyP+bodyQ.RotateVector(p))&&Contains(hull,referenceBodyP+referenceBodyQ.RotateVector(p))&&
                        Contains(hull,v.BodyState.Location+fixed)&&
                        (!representation||(Contains(hull,chordP+fixed)&&Contains(hull,chordP+axes)&&Contains(hull,bodyP+axes)))))return false;
                }
            }
            if(sample!=0)continue;
            for(int32 guard=0;guard<7;++guard)
            {
                auto bad=v;if(guard==0)bad.Query.End.X+=1.;if(guard==1)bad.Frozen=true;if(guard==2)bad.Prescribed=true;
                if(guard==3)bad.Query.HalfExtent.X+=1.;if(guard==4)bad.BodyState.Location.X+=1.;
                if(guard==5)bad.LocalRotation.W=3.;if(guard==6)bad.Query.DomainTolerance=-.001;
                if(!TestTrue(TEXT("captured identities and query guards are fresh"),!X::BuildCompleteVolume(bad,hull)&&!Detail::ValidPoint(hull)))return false;
            }
            auto fast=v.OwnerState;fast.AngularVelocity=FVector(1.e6,0,0);
            if(!TestFalse(TEXT("unsupported angular domain remains optional failure"),X::BoundOwnerQuaternionDelta(fast,v.Horizon,constants,delta)))return false;
            const uint32 good=VectorGetControlRegister();VectorSetControlRegister(good|uint32(0x2000));
            const bool invalid=X::BuildCompleteVolume(v,hull);VectorSetControlRegister(good);
            if(!TestTrue(TEXT("unsupported live rounding cannot reuse a numerical table"),!invalid&&!Detail::ValidPoint(hull)))return false;
        }
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace CorrelatedGroupsTests
{
using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
namespace A=::AnalyticCertified;namespace Q=CorrelatedNativeBoxGroups;
static bool Contains(const FIntervalPoint& box,const FVector3d& point)
{for(int32 d=0;d<3;++d)if(!A::Valid(box.Axis[d])||point[d]<box.Axis[d].Lower||point[d]>box.Axis[d].Upper)return false;return true;}
static bool FarScene(FIndexedIntervalUnionPtr& output)
{
    FIntervalTensorSurface surface;surface.DegreeU=surface.DegreeV=1;
    surface.ControlPoints={Detail::Point(FVector3d(1.e6,-10,-10)),Detail::Point(FVector3d(1.e6,10,-10)),
        Detail::Point(FVector3d(1.e6,-10,10)),Detail::Point(FVector3d(1.e6,10,10))};
    TArray<FIntervalTensorSurface> surfaces{surface};FDistanceBudget budget{4096};return FIndexedIntervalUnion::Build(surfaces,budget,output);
}
static bool Prove(FIndexedIntervalUnionPtr scene,const FGroupedBoxVolumeBounds& volume,FReferenceBoxVolumeStrictSeparationPtr& output)
{
    FDistanceBudget budget{100000};FReferenceBoundsCandidateSetPtr partition;
    return FReferenceBoundsCandidateSet::Select(scene,volume.BroadHull,budget,partition)&&
        FReferenceBoxVolumeStrictSeparation::TryProve(partition,volume,budget,output)&&output&&output->Matches(scene,volume);
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCorrelatedGroupFrameTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.CorrelatedGroupFrameAndCompleteRegions",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCorrelatedGroupFrameTest::RunTest(const FString&)
{
    using namespace CorrelatedGroupsTests;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(double sign:{1.,-1.})
        {
            Private::FBoxSweepContext axes;axes.Axis[0]=FVector3d(sign,sign*.125,0);axes.Axis[1]=FVector3d(0,1,.25);axes.Axis[2]=FVector3d(0,0,1);
            Q::FFrame frame;const FVector3d origin(4,-8,16);const A::Interval slabs[3]={{-2,3,true},{-1,4,true},{-5,6,true}};
            FBoxVolumeGroup group;
            if(!TestTrue(TEXT("nonorthogonal frame with either determinant sign is certified"),
                Q::PrepareFrame(origin,axes,frame)&&Q::SlabsToGroup(frame,slabs,group)&&group.Corners.Num()==8))return false;
            for(int32 corner=0;corner<8;++corner)
            {
                const double x=(corner&1)?3.:-2.,y=(corner&2)?4.:-1.,z=(corner&4)?6.:-5.;
                const FVector3d exact=origin+FVector3d(sign*x-.125*y+.03125*z,y-.25*z,z);
                if(!TestTrue(TEXT("each inverse-vertex box contains its exact dyadic rational vertex"),Contains(group.Corners[corner],exact)))return false;
            }
            axes.Axis[1]=axes.Axis[0];
            if(!TestFalse(TEXT("singular fixed frame cannot manufacture a box"),Q::PrepareFrame(origin,axes,frame)))return false;
        }
        // Complete-region endpoint admission must not become pointwise.
        Private::FBoxSweepContext identity;identity.Axis[0]=FVector3d(1,0,0);identity.Axis[1]=FVector3d(0,1,0);identity.Axis[2]=FVector3d(0,0,1);
        Q::FFrame frame;if(!Q::PrepareFrame(FVector3d::ZeroVector,identity,frame))return false;
        const A::Interval left[3]={{-11,-9,true},{-1,1,true},{-1,1,true}},right[3]={{9,11,true},{-1,1,true},{-1,1,true}};
        FBoxVolumeGroup a,b;if(!Q::SlabsToGroup(frame,left,a)||!Q::SlabsToGroup(frame,right,b))return false;
        TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups{a,b};FIntervalPoint broad;
        for(const auto& g:groups)for(const auto& p:g.Corners)if(!YawFloorBoxDetail::Join(broad,p))return false;
        TArray<FVector3d,TInlineAllocator<8>> axes{FVector3d(1,0,0),FVector3d(0,1,0),FVector3d(0,0,1)};
        FDistanceBudget close{4096};FGroupedBoxVolumeBounds volume;FIndexedIntervalUnionPtr scene;FReferenceBoxVolumeStrictSeparationPtr proof;
        if(!CloseGroupedBoxVolumeBounds(broad,groups,axes,close,volume)||!FarScene(scene)||!Prove(scene,volume,proof))return false;
        TArray<FIntervalPoint,TInlineAllocator<16>> mixed;
        for(int32 k=0;k<8;++k)mixed.Add(k<4?a.Corners[k]:b.Corners[k]);
        if(!TestTrue(TEXT("one complete endpoint group must fit one complete region"),
            proof->ContainsCompleteEndpointGroup(a.Corners)&&!proof->ContainsCompleteEndpointGroup(mixed)))return false;
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCorrelatedNativeGroupsTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.CorrelatedGroupsGlobalNativeFamilies",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCorrelatedNativeGroupsTest::RunTest(const FString&)
{
    using namespace CorrelatedGroupsTests;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.CorrelatedNativeBoxGroups"),1,FName(TEXT("CorrelatedGroupsNativeFamilies")));
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);FIndexedIntervalUnionPtr scene;if(!FarScene(scene))return false;
        for(int32 sample:{0,3,8,9,10,11})
        {
            const auto v=InstantMotionClearanceTests::Native(sample);
            for(bool fine:{false,true})
            {
                FDistanceBudget work{100000,17,13};FGroupedBoxVolumeBounds volume;
                if(!TestTrue(TEXT("same8/14complete roles and original bounded debit"),Q::Build(v,fine,work,volume)&&
                    volume.Groups.Num()==(fine?14:8)&&volume.Axes.Num()==6&&volume.Slabs.Num()==volume.Groups.Num()*6&&
                    work.Used==uint64(17+(fine?426:120))&&work.Remaining==100000-(fine?426:120)&&work.ConsumptionFloor==13))return false;
                if(sample==0)
                {
                    FDistanceBudget dispatchedWork{100000,17,13};FGroupedBoxVolumeBounds dispatched;
                    const bool built=fine?Q::BuildFineOrOriginal(v,dispatchedWork,dispatched):
                        Speed::PreparedExecution::NativePredictionCoarseDetail::BuildVolume(v,dispatchedWork,dispatched);
                    FGroupedBoxVolumeBounds expected=volume;FDistanceBudget expectedWork=work;
                    if(fine){expectedWork={100000,17,13};if(!BuildGeneralNativeBoxGroupedVolume(v,expectedWork,expected))return false;}
                    if(!TestTrue(TEXT("production dispatch retains correlated coarse and literal original fine geometry/debit"),built&&
                        GroupedBoxDetail::Same(expected,dispatched)&&dispatchedWork.Used==expectedWork.Used&&
                        dispatchedWork.Remaining==expectedWork.Remaining&&dispatchedWork.ConsumptionFloor==expectedWork.ConsumptionFloor))return false;
                }
                FReferenceBoxVolumeStrictSeparationPtr proof;if(!Prove(scene,volume,proof))return false;
                for(int32 step=0;step<=8;++step)
                {
                    const float t=float(double(v.Horizon)*double(step)/8.);
                    const auto ownerQ=::Speed::SBox::IntegrateRotation(v.OwnerState.Rotation,v.OwnerState.AngularVelocity,v.OwnerState.AngularAcceleration,t);
                    const auto ownerP=::Speed::SBox::AdvancePosition(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,t);
                    auto pose=v.Query;pose.Start=pose.End=(ownerP-ownerQ.RotateVector(v.LocalCOM))+ownerQ.RotateVector(v.LocalOffset);
                    pose.Rotation=ownerQ*v.LocalRotation;
                    if(!TestTrue(TEXT("actual native endpoint complete groups fit current correlated regions"),
                        GroupedBoxProofContainsEndpoint(proof,pose,v.PhysicalHalfExtent,v.QueryHalfExtent)))return false;
                    auto chord=v.Query;chord.Start=chord.End=v.Query.Start+(v.Query.End-v.Query.Start)*(double(step)/8.);
                    FBoxVolumeGroup chordGroup;if(!NativeGroupedBoxDetail::FixedQueryGroup(chord,chordGroup))return false;
                    if(!TestTrue(TEXT("literal World chord remains a complete query group"),proof->ContainsCompleteEndpointGroup(chordGroup.Corners)))return false;
                }
            }
            if(sample!=0)continue;
            auto cancelled=v.OwnerState;cancelled.AngularVelocity=FVector(0,1,0);cancelled.AngularAcceleration=FVector(0,-512,0);
            InstantMotionClearanceDetail::FStaticNumericalBounds constants;
            InstantMotionClearanceDetail::FQuaternionDelta delta;
            if(!InstantMotionClearanceDetail::GetStaticNumericalBounds(constants)||
                !InstantMotionClearanceDetail::BoundOwnerQuaternionDeltaRange(cancelled,1.f/512.f,3.f/512.f,constants,delta))return false;
            if(!TestTrue(TEXT("positive-time exact angular cancellation keeps original q0 instead of rebasing"),
                InstantMotionClearanceTests::Contains(cancelled.Rotation,delta,::Speed::SBox::IntegrateRotation(cancelled.Rotation,
                    cancelled.AngularVelocity,cancelled.AngularAcceleration,1.f/256.f))))return false;
            FDistanceBudget shortWork{119,17};FGroupedBoxVolumeBounds absent;
            if(!TestTrue(TEXT("partial bounded build never refunds work or publishes an incomplete volume"),
                !Q::Build(v,false,shortWork,absent)&&shortWork.Remaining==0&&shortWork.Used==136&&absent.Groups.IsEmpty()))return false;
            auto bad=v;bad.Query.End.X+=1.;FDistanceBudget badWork{100000};
            if(!TestTrue(TEXT("fresh literal chord identity cannot be borrowed from prepared numerical data"),
                !Q::Build(bad,true,badWork,absent)&&absent.Groups.IsEmpty()))return false;
        }
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace CorrelatedCaptured791Tests
{
using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
static PreparedExecution::FNativeSolidStaticValues Capture()
{
    PreparedExecution::FNativeSolidStaticValues v;v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;
    v.Horizon=KinematicCenterDetail::FromBits(995783694u);
    v.OwnerState.Location=FVector(2.422663690465976e-06,3885.3402943902852,28.042697524124478);
    v.OwnerState.Rotation=FQuat(.048239821411397911,-.0482371207952016,.70545966886761824,.70545925141625032);
    v.OwnerState.Velocity=FVector(.0015124224444943467,1365.3452508797245,158.71854344242035);
    v.OwnerState.Acceleration=FVector(.91497898690658142,-663.60582297932842,1348.1113150614788);
    v.OwnerState.AngularVelocity=FVector(5.4999994095103473,.0024855134762629713,.00056356773446590515);
    v.OwnerState.AngularAcceleration=FVector(3.4913140066240713e-05,-.077453863039274873,-.017809311445560472);
    v.BodyState=v.OwnerState;
    v.BodyState.Location=FVector(4.6034091264607688e-05,3892.6187380898023,41.25058397109089);
    v.BodyState.Velocity=FVector(.03023890617520595,1292.7018832451074,198.74997938352155);
    v.BodyState.Acceleration=FVector(.16203936131620636,-883.77914273339593,948.57302205737767);
    v.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);
    v.LocalOffset=FVector(-5,0,29.510000000000002);v.LocalRotation=FQuat(0,-0.,0,1);
    v.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
    v.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
    v.Query.Shape=EQueryShape::Box;v.Query.Start=v.BodyState.Location;
    v.Query.End=FVector(.00014773066634025817,3896.9228345872616,41.918353768789075);
    v.Query.Rotation=v.BodyState.Rotation;v.Query.HalfExtent=v.QueryHalfExtent;
    v.Query.DomainTolerance=9.9999999999999995e-07;v.Query.InitialOverlapTolerance=.050000000000000003;
    v.Start=v.Query.Start;v.End=v.Query.End;v.ResponseConfig.OwnerMaxSpeed=2300;v.ResponseConfig.OwnerMaxAngularSpeed=5.5f;
    v.ResponseConfig.ApplyRestForce=true;return v;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCorrelatedCaptured791FineFallbackTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.CorrelatedCaptured791OriginalFineReserve",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCorrelatedCaptured791FineFallbackTest::RunTest(const FString&)
{
    using namespace CorrelatedCaptured791Tests;namespace Q=CorrelatedNativeBoxGroups;
    namespace P=Speed::PreparedExecution::NativePredictionCoarseDetail;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.CorrelatedNativeBoxGroups"),1,FName(TEXT("Captured791FineReserve")));
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();const auto v=Capture();
    constexpr int32 total=34544,initialFloor=72,coarseAllowance=2992+2048,fineAllowance=total-initialFloor-coarseAllowance;
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        FDistanceBudget account{total,0,initialFloor};FGroupedBoxVolumeBounds coarse;
        {
            FScopedDistanceBudgetFloor reserve(account,account.Remaining-coarseAllowance);
            if(!TestTrue(TEXT("captured frame791 admits only the bounded correlated coarse volume"),reserve.IsValid()&&
                P::BuildVolume(v,account,coarse)&&coarse.Groups.Num()==8))return false;
            // Emulate the worst permitted coarse proof. No refund/reset and
            // no additional allowance can be hidden by a cheap fixture scene.
            while(account.Remaining>account.ConsumptionFloor)if(!account.Consume())return false;
        }
        if(!TestTrue(TEXT("all original fine work survives a completely spent coarse attempt"),
            account.Used==coarseAllowance&&account.Remaining==initialFloor+fineAllowance&&account.ConsumptionFloor==initialFloor))return false;
        FDistanceBudget oracle{fineAllowance+initialFloor,coarseAllowance,initialFloor};FGroupedBoxVolumeBounds expected,actual;
        if(!TestTrue(TEXT("original fine model remains available for the exact failed capture"),
            BuildGeneralNativeBoxGroupedVolume(v,oracle,expected)&&expected.Groups.Num()==14))return false;
        {
            FScopedDistanceBudgetFloor fine(account,account.Remaining-fineAllowance);
            if(!TestTrue(TEXT("production fine dispatch is literally the original geometry/debit after coarse exhaustion"),fine.IsValid()&&
                Q::BuildFineOrOriginal(v,account,actual)&&GroupedBoxDetail::Same(expected,actual)&&
                account.Used==oracle.Used&&account.Remaining==oracle.Remaining&&account.ConsumptionFloor==oracle.ConsumptionFloor))return false;
        }
        if(!TestTrue(TEXT("no coarse work has been credited back"),account.Used>coarseAllowance&&
            account.Used+uint64(account.Remaining)==total&&account.ConsumptionFloor==initialFloor))return false;
        if(flush==0)
        {
            // Diagnostic geometry only: the wider fine candidate remains
            // callable for audit, but is not selected by production dispatch.
            FDistanceBudget diagnostic{100000};FGroupedBoxVolumeBounds candidate;
            if(!Q::Build(v,true,diagnostic,candidate))return false;
            for(int32 group:{4,5,6,7})for(int32 axis=0;axis<6;++axis)
            {
                const auto old=expected.Slabs[group*6+axis],now=candidate.Slabs[group*6+axis];
                AddInfo(FString::Printf(TEXT("Captured791 Group=%d Axis=%d Old=[%.17g,%.17g] Correlated=[%.17g,%.17g] LowerDelta=%.17g UpperDelta=%.17g"),
                    group,axis,old.Lower,old.Upper,now.Lower,now.Upper,now.Lower-old.Lower,now.Upper-old.Upper));
            }
        }
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCorrelatedTrueCornerRolesTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.CorrelatedTrueCornerRolesAndCaptured791",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCorrelatedTrueCornerRolesTest::RunTest(const FString&)
{
    using namespace CorrelatedGroupsTests;namespace X=InstantMotionClearanceDetail;
    const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 saved=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);
        for(int32 sample:{-1,0,3,8,9,10,11})
        {
            const auto v=sample<0?CorrelatedCaptured791Tests::Capture():InstantMotionClearanceTests::Native(sample);
            Q::FPrepared prepared;if(!Q::Prepare(v,prepared))return false;
            for(const auto& range:{TPair<float,float>(0.f,v.Horizon),TPair<float,float>(.5f*v.Horizon,v.Horizon)})
            {
                FGeneralBoxMotionBounds bounds;
                if(!TestTrue(TEXT("global-time model retains32 original signed-corner roles"),
                    Q::MotionRange(prepared,range.Key,range.Value,bounds)&&bounds.Corners.Num()==32))return false;
                for(int32 step=0;step<=8;++step)
                {
                    const float t=range.Key+(range.Value-range.Key)*(float(step)/8.f);const double td=double(t);
                    const auto ownerQ=::Speed::SBox::IntegrateRotation(v.OwnerState.Rotation,v.OwnerState.AngularVelocity,v.OwnerState.AngularAcceleration,t);
                    const auto ownerP=::Speed::SBox::AdvancePosition(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,t);
                    const auto bodyP=(ownerP-ownerQ.RotateVector(v.LocalCOM))+ownerQ.RotateVector(v.LocalOffset);
                    const auto bodyQ=ownerQ*v.LocalRotation;
                    const auto theta=v.OwnerState.AngularVelocity*td+.5*v.OwnerState.AngularAcceleration*(td*td);
                    FQuat referenceOwner=v.OwnerState.Rotation;
                    if(!theta.IsZero())
                    {
                        const double squared=theta.SizeSquared(),angle=FMath::Sqrt(squared);
                        const double scale=squared<1.e-8?.5-squared/48.+squared*squared/3840.:FMath::Sin(angle*.5)/angle;
                        const double cosine=FMath::Cos(angle*.5);
                        if(!GeneralBoxDetail::CheckedNativeExpResults(squared,scale,cosine))return false;
                        referenceOwner=FQuat(theta.X*scale,theta.Y*scale,theta.Z*scale,cosine)*v.OwnerState.Rotation;
                    }
                    const auto referenceP=v.OwnerState.Location+v.OwnerState.Velocity*td+.5*v.OwnerState.Acceleration*(td*td);
                    for(int32 representation=0;representation<2;++representation)for(int32 corner=0;corner<8;++corner)
                    {
                        const auto extent=representation?v.QueryHalfExtent:v.PhysicalHalfExtent;
                        FVector3d local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
                        if(!TestTrue(TEXT("native float-square path is enclosed in its SAME vertex, not only a box hull"),
                            Contains(bounds.Corners[representation*8+corner],bodyP+bodyQ.RotateVector(local))))return false;
                        for(const auto& q:{referenceOwner,referenceOwner.GetNormalized()})
                        {
                            const auto referenceBodyP=(referenceP-q.RotateVector(v.LocalCOM))+q.RotateVector(v.LocalOffset);
                            if(!TestTrue(TEXT("raw and normalized reference paths keep every original vertex role"),
                                Contains(bounds.Corners[16+representation*8+corner],referenceBodyP+(q*v.LocalRotation).RotateVector(local))))return false;
                        }
                    }
                }
            }
            // Fixed Rotate/SAT representations share each literal local sign;
            // their hull must not be confused with inverse-frame box vertices.
            FBoxVolumeGroup fixed;if(!Q::FixedGroup(prepared,Detail::Point(v.BodyState.Location),v.QueryHalfExtent,fixed))return false;
            Private::FBoxSweepContext sat;sat.Initialize(v.Query);
            for(int32 corner=0;corner<8;++corner)
            {
                FVector3d local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?v.QueryHalfExtent[d]:-v.QueryHalfExtent[d];
                const auto native=v.BodyState.Location+v.BodyState.Rotation.RotateVector(local);
                const auto axes=v.BodyState.Location+(sat.Axis[0]*local.X+sat.Axis[1]*local.Y+sat.Axis[2]*local.Z);
                if(!TestTrue(TEXT("fixed representations stay bound to the same original local corner"),
                    Contains(fixed.Corners[corner],native)&&Contains(fixed.Corners[corner],axes)))return false;
            }
        }
    }
    TestTrue(TEXT("context restored"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeBoxAntipodalOffsetTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.AntipodalOffsetBitsAndSticky",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeBoxAntipodalOffsetTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace A=::AnalyticCertified;namespace Y=YawFloorBoxDetail;namespace M=NativeBoxAntipodal;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE antipodal offset oracle"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const uint32 saved=VectorGetControlRegister();int32 cases=0,known=0,unknown=0;
 const double tiny=A::FromBits(A::MinNormal),below=A::FromBits(A::MinNormal-1),above=A::FromBits(A::MinNormal+1),sub=A::FromBits(1);
 const FVector extents[]={FVector(1,2,3),FVector(.1,.3,.7),FVector(0.,-0.,tiny),
  FVector(below,above,sub),FVector(1.e-200,2.e-200,3.e-200),FVector(1.e150,2.e150,3.e150),
  FVector(std::numeric_limits<double>::max(),1,2)};
 const FQuat rotations[]={FQuat::Identity,FQuat(.1,-.2,.3,.9),FQuat(2.,-1.,.5,1.3),FQuat(tiny,-tiny,sub,1.)};
 for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,1u,2u,4u,8u,16u,32u,63u})
 for(const auto& q:rotations)for(bool family:{false,true})for(const auto& extent:extents)for(int32 corner=0;corner<4;++corner)
 {
  Y::FQuaternion model=Y::Quaternion(q);
  if(family)for(auto& axis:model.Axis)
  {axis.Range=A::Add(axis.Range,Interval{-1.e-12,1.e-12,true});axis.Error=1.e-14;}
  FVector local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
  const uint32 control=(saved&~uint32(0x807f))|flush|sticky;
  VectorSetControlRegister(control);
  const auto firstOracle=GeneralBoxDetail::PointRanges(Y::Rotate(model,local));FIntervalPoint oppositeOracle;
  if(Detail::ValidPoint(firstOracle))oppositeOracle=GeneralBoxDetail::PointRanges(Y::Rotate(model,-local));
  const uint32 oldFlags=VectorGetControlRegister();
  VectorSetControlRegister(control);
  const auto first=GeneralBoxDetail::PointRanges(Y::Rotate(model,local));FIntervalPoint opposite;
  if(Detail::ValidPoint(first))opposite=M::OppositeOffset(first);
  const uint32 newFlags=VectorGetControlRegister();
  const bool valid=Detail::ValidPoint(firstOracle)&&Detail::ValidPoint(oppositeOracle);
  if(!TestTrue(TEXT("independent opposite rotation equals reflected enclosure, including refusals and fullMXCSR"),
   GeneralBoxPrologueReuseTests::SamePoint(firstOracle,first)&&GeneralBoxPrologueReuseTests::SamePoint(oppositeOracle,opposite)&&oldFlags==newFlags))return false;
  if(valid)
  {
   ++known;
   // Independently evaluate an actual coefficient realization through the
   // native polynomial RotateVector, including admitted non-unit quaternions.
   const auto actual=q.RotateVector(-local);
   for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("actual opposite realization remains contained"),
    A::Finite(actual[d])&&actual[d]>=opposite.Axis[d].Lower&&actual[d]<=opposite.Axis[d].Upper))return false;
  }
  else ++unknown;
  ++cases;
 }
 if(!TestTrue(TEXT("nonvacuous paired oracles and arithmetic refusal cases"),cases==7168&&known>0&&unknown>0))return false;
 FIntervalPoint zero;for(auto& axis:zero.Axis)axis={0.,tiny,true};
 const auto negative=M::OppositeOffset(zero);
 if(!TestTrue(TEXT("zero endpoint is canonical+0, not accidental-0"),A::Bits(negative.Axis[0].Upper)==0&&negative.Axis[0].Lower==-tiny))return false;
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeBoxAntipodalGlobalRangeTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.AntipodalGlobalRangeParity",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeBoxAntipodalGlobalRangeTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=GeneralBoxPrologueReuseTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE global range parity"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const uint32 saved=VectorGetControlRegister();int32 cases=0,positive=0;
 for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,63u})for(int32 sample=0;sample<8;++sample)
 {
  auto input=T::Input(sample);
  if(sample==1){input.Owner.AngularVelocity=FVector::ZeroVector;input.Owner.AngularAcceleration=FVector::ZeroVector;}
  if(sample==2){input.Owner.AngularAcceleration=FVector(-2,4,-6);input.Owner.AngularVelocity=-.25*double(input.Horizon)*input.Owner.AngularAcceleration;}
  if(sample==3){input.LocalCOM=FVector::ZeroVector;input.LocalOffset=FVector::ZeroVector;input.Owner.Location=FVector::ZeroVector;}
  if(sample==4)input.Owner.Rotation=FQuat(-input.Owner.Rotation.X,-input.Owner.Rotation.Y,-input.Owner.Rotation.Z,-input.Owner.Rotation.W);
  if(sample==5)input.LocalRotation=FQuat(-input.LocalRotation.X,-input.LocalRotation.Y,-input.LocalRotation.Z,-input.LocalRotation.W);
  if(sample==6)input.PhysicalHalfExtent.X=0.;
  if(sample==7)input.Owner.Rotation.W=7.;
  for(const auto range:{TPair<float,float>(0.f,input.Horizon),TPair<float,float>(.25f*input.Horizon,.75f*input.Horizon),TPair<float,float>(input.Horizon,input.Horizon)})
  {
   const uint32 control=(saved&~uint32(0x807f))|flush|sticky;FGeneralBoxMotionBounds reference,result;
   VectorSetControlRegister(control);const bool oldKnown=BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,range.Key,range.Value,reference,false);const uint32 oldFlags=VectorGetControlRegister();
   VectorSetControlRegister(control);const bool newKnown=BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,range.Key,range.Value,result,true);const uint32 newFlags=VectorGetControlRegister();
   if(!TestTrue(TEXT("all32ordered corners, bothcenterfamilies, wholehull and fullFPstatus match"),
    oldKnown==newKnown&&T::SameWhole(reference,result)&&oldFlags==newFlags))return false;
   if(sample<6&&!TestTrue(TEXT("nontrivial admitted motion cannot pass vacuously"),oldKnown&&result.Corners.Num()==32))return false;
   if(oldKnown)++positive;++cases;
  }
 }
 if(!TestTrue(TEXT("192 actualglobal comparisons including failure guards"),cases==192&&positive==144))return false;
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeBoxAntipodalFineBudgetTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.AntipodalFineBudgetAndConsumer",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeBoxAntipodalFineBudgetTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=GeneralBoxPrologueReuseTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE fine consumer"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.NativeBoxAntipodal"),1,FName(TEXT("AntipodalFineRoute")));
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})
 {
  VectorSetControlRegister((saved&~uint32(0x8040))|flush);const auto input=T::Input();const auto v=T::Native(input);
  for(int32 available:{0,1,13,14,100,223,224,225,425,426,427,5000})
  {
   FDistanceBudget oldWork{available+7,13,7},newWork=oldWork;FGroupedBoxVolumeBounds oldVolume,newVolume;
   const bool oldKnown=BuildGeneralNativeBoxGroupedVolume(v,oldWork,oldVolume,nullptr,false);
   const bool newKnown=CorrelatedNativeBoxGroups::BuildFineOrOriginal(v,newWork,newVolume);
   if(!TestTrue(TEXT("same fine14 budgetincludingpartialfailures, same floor, no refund and same result"),
    oldKnown==newKnown&&oldWork.Used==newWork.Used&&oldWork.Remaining==newWork.Remaining&&newWork.ConsumptionFloor==7&&newWork.Remaining>=7&&
    (oldKnown?GroupedBoxDetail::Same(oldVolume,newVolume):newVolume.Groups.IsEmpty())))return false;
   if(available==426&&!TestTrue(TEXT("full original426units remain charged despite fewer rotations"),newKnown&&newWork.Used==439&&newWork.Remaining==7&&newVolume.Groups.Num()==14))return false;
   if(available!=5000)continue;
   FIntervalTensorSurface far;far.DegreeU=far.DegreeV=1;
   for(int32 bits=0;bits<4;++bits)far.ControlPoints.Add(Detail::Point(FVector3d(bits&1?100:-100,bits&2?100:-100,-10000)));
   FIndexedIntervalUnionPtr scene;FDistanceBudget make{10000};if(!FIndexedIntervalUnion::Build(MakeArrayView(&far,1),make,scene))return false;
   FReferenceBoundsCandidateSetPtr partition;if(!FReferenceBoundsCandidateSet::Select(scene,newVolume.BroadHull,make,partition))return false;
   FReferenceBoxVolumeStrictSeparationPtr proof;
   if(!TestTrue(TEXT("same original scene proof and fresh current-pose endpoint consumer"),
    FReferenceBoxVolumeStrictSeparation::TryProve(partition,newVolume,make,proof)&&proof&&proof->Matches(scene,newVolume)))return false;
   auto pose=v.Query;pose.Start=pose.End=v.BodyState.Location;
   if(!TestTrue(TEXT("physical/query endpoint remains covered without reflected world position"),GroupedBoxProofContainsEndpoint(proof,pose,v.PhysicalHalfExtent,v.QueryHalfExtent)))return false;
   auto changed=v;changed.Query.End.X+=1;FGroupedBoxVolumeBounds rejected=newVolume;FDistanceBudget guard{5000};
   if(!TestTrue(TEXT("current captured chord mismatch still refuses and clears output"),!CorrelatedNativeBoxGroups::BuildFineOrOriginal(changed,guard,rejected)&&rejected.Groups.IsEmpty()))return false;
  }
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace NodeErrorUpperTests
{
using namespace Speed::Analytic::Certified;
namespace A=::AnalyticCertified;namespace Y=YawFloorBoxDetail;
// Frozen original error DAG. No call into the optional error implementation.
static Y::FNode OriginalSum(const Y::FNode& a,const Y::FNode& b)
{
    if(!Y::ValidNode(a)||!Y::ValidNode(b))return {};
    Y::FNode out;out.Range=A::Add(a.Range,b.Range);
    const auto error=A::Add(A::Add(Y::InputError(a),Y::InputError(b)),Y::RoundError(out.Range));
    if(!A::Valid(error))return {};out.Error=error.Upper;return out;
}
static Y::FNode OriginalProduct(const Y::FNode& a,const Y::FNode& b)
{
    if(!Y::ValidNode(a)||!Y::ValidNode(b))return {};
    Y::FNode out;out.Range=A::Multiply(a.Range,b.Range);
    const auto ea=Y::InputError(a),eb=Y::InputError(b);
    const auto error=A::Add(A::Add(A::Add(
        A::Multiply(Interval::Exact(Y::MagnitudeOf(a.Range)),eb),
        A::Multiply(Interval::Exact(Y::MagnitudeOf(b.Range)),ea)),A::Multiply(ea,eb)),Y::RoundError(out.Range));
    if(!A::Valid(error))return {};out.Error=error.Upper;return out;
}
static bool Same(const Y::FNode& a,const Y::FNode& b)
{return a.Range.Known==b.Range.Known&&A::Bits(a.Range.Lower)==A::Bits(b.Range.Lower)&&
    A::Bits(a.Range.Upper)==A::Bits(b.Range.Upper)&&A::Bits(a.Error)==A::Bits(b.Error);}
static TArray<Y::FNode> Edges()
{
    const double tiny=A::FromBits(A::MinNormal),sub=A::FromBits(1),max=A::FromBits(0x7fefffffffffffffULL);
    const double rootTiny=A::FromBits(uint64(512)<<52); // 2^-511.
    return {
        {Interval::Exact(0.),0.},{Interval::Exact(1.),0.},{Interval::Exact(-1.),tiny},
        {{-2.,3.,true},rootTiny},{{0.,tiny,true},sub},{{-tiny,0.,true},A::FromBits(A::MinNormal+1)},
        {Interval::Exact(A::FromBits(A::MinNormal-1)),A::FromBits(A::MinNormal-1)},
        {Interval::Exact(rootTiny),rootTiny},{Interval::Exact(max),0.},
        {{1.e150,2.e150,true},1.e150},{Interval::Exact(1.),max},
        {Interval::Exact(1.),A::FromBits(A::Infinity)},{Interval::Exact(1.),A::FromBits(A::Sign)},
        {{sub,tiny,true},0.},{{2.,1.,true},0.},{{},0.},
        {Interval::Exact(1.),-tiny},{Interval::Exact(-max),0.}
    };
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNodeErrorUpperArithmeticTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.NodeErrorUpperBoundsAndControls",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNodeErrorUpperArithmeticTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace T=NodeErrorUpperTests;namespace Y=YawFloorBoxDetail;namespace U=Y::NodeErrorUpper;namespace A=::AnalyticCertified;
    const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!saved.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
    FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.NodeErrorUpper"),1,FName(TEXT("NodeErrorUpperArithmetic")));
    if(!TestTrue(TEXT("real CVar callback enables the arithmetic route"),U::RuntimeEnabled()))return false;
    const uint32 initial=VectorGetControlRegister();const auto edges=T::Edges();uint64 tested=0,known=0,unknown=0;
    for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,1u,2u,4u,8u,16u,32u,63u})
    {
        const uint32 controls=(initial&~uint32(0x807f))|flush|sticky;
        const auto compare=[&](const Y::FNode& a,const Y::FNode& b)->bool
        {
            for(bool product:{false,true})
            {
                VectorSetControlRegister(controls);
                const auto old=product?T::OriginalProduct(a,b):T::OriginalSum(a,b);
                VectorSetControlRegister(controls);
                const auto before=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
                const auto actual=product?Y::Product(a,b):Y::Sum(a,b);
                const uint32 flags=VectorGetControlRegister();
                const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
                if(!TestTrue(TEXT("same upper bits, Range and refusal; controls unchanged, historical flags never cleared"),
                    T::Same(old,actual)&&(flags&~uint32(63))==(controls&~uint32(63))&&
                    (flags&sticky)==sticky&&before.ControlFingerprint()==after.ControlFingerprint()))return false;
                ++tested;if(Y::ValidNode(actual))++known;else ++unknown;
            }
            return true;
        };
        for(const auto& a:edges)for(const auto& b:edges)if(!compare(a,b))return false;
        uint64 state=0x315907a6245d8b3fULL;
        const auto next=[&](){state^=state<<13;state^=state>>7;state^=state<<17;return state;};
        for(int32 sample=0;sample<128;++sample)
        {
            const auto node=[&]()
            {
                uint64 x=next(),y=next();
                const uint64 xb=(x&A::Sign)|((1+(x%2046))<<52)|(x&0xfffffffffffffULL);
                const uint64 yb=(y&A::Sign)|((1+(y%2046))<<52)|(y&0xfffffffffffffULL);
                const double lo=A::FromBits(xb),hi=A::FromBits(yb);const uint64 e=next();
                const uint64 eb=(sample%7==0)?(e&0xfffffffffffffULL):(((1+(e%2046))<<52)|(e&0xfffffffffffffULL));
                return Y::FNode{Interval{FMath::Min(lo,hi),FMath::Max(lo,hi),true},A::FromBits(eb)};
            };
            const auto a=node(),b=node();if(!compare(a,b))return false;
        }
    }
    TestTrue(TEXT("both accepted and refused arithmetic are exercised"),tested>28000&&known>0&&unknown>0);
    // An omitted lower product may underflow while its upper product does not.
    // This is an intentional status-history difference, never a changed bound.
    const Y::FNode boundary{Interval::Exact(1.),A::FromBits(uint64(512)<<52)};
    const uint32 nearest=(initial&~uint32(0x807f));
    VectorSetControlRegister(nearest);const auto old=T::OriginalProduct(boundary,boundary);const uint32 oldFlags=VectorGetControlRegister();
    VectorSetControlRegister(nearest);const auto actual=Y::Product(boundary,boundary);const uint32 newFlags=VectorGetControlRegister();
    TestTrue(TEXT("lower-only underflow is not forged or replayed"),T::Same(old,actual)&&(oldFlags&16u)!=0&&(newFlags&16u)==0);
    const auto positiveSubnormal=U::ExactUpper(A::FromBits(1));
    TestTrue(TEXT("positive subnormal Error retains Exact upper allowance before DAZ"),positiveSubnormal.Known&&A::Bits(positiveSubnormal.Value)==A::MinNormal);
    TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNodeErrorUpperMotionTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.NodeErrorUpperMotionAndBudget",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNodeErrorUpperMotionTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Analytic;using namespace Certified;
    namespace T=GeneralBoxPrologueReuseTests;namespace U=YawFloorBoxDetail::NodeErrorUpper;
    const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
    if(!saved.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
    const uint32 initial=VectorGetControlRegister();int32 models=0;
    for(uint32 flush:{0u,64u,32768u,32832u})for(int32 sample=0;sample<6;++sample)for(bool antipodal:{false,true})
    {
        VectorSetControlRegister((initial&~uint32(0x8040))|flush);auto input=T::Input(sample);
        if(sample==1){input.Owner.AngularVelocity=FVector::ZeroVector;input.Owner.AngularAcceleration=FVector::ZeroVector;}
        if(sample==2){input.Owner.AngularAcceleration=FVector(-2,4,-6);input.Owner.AngularVelocity=-.25*double(input.Horizon)*input.Owner.AngularAcceleration;}
        if(sample==3){input.LocalCOM=FVector::ZeroVector;input.LocalOffset=FVector::ZeroVector;input.Owner.Location=FVector::ZeroVector;}
        if(sample==4)input.Owner.Rotation=FQuat(-input.Owner.Rotation.X,-input.Owner.Rotation.Y,-input.Owner.Rotation.Z,-input.Owner.Rotation.W);
        if(sample==5)input.Owner.Rotation.W=7.;
        for(const auto range:{TPair<float,float>(0.f,input.Horizon),TPair<float,float>(.25f*input.Horizon,.75f*input.Horizon)})
        {
            FGeneralBoxMotionBounds oracle,actual;bool oldKnown,newKnown;
            {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.NodeErrorUpper"),0,FName(TEXT("NodeErrorUpperMotionOracle")));
             if(!TestFalse(TEXT("OFF callback route"),U::RuntimeEnabled()))return false;
             oldKnown=BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,range.Key,range.Value,oracle,antipodal);}
            {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.NodeErrorUpper"),1,FName(TEXT("NodeErrorUpperMotionCandidate")));
             if(!TestTrue(TEXT("ON callback route"),U::RuntimeEnabled()))return false;
             newKnown=BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(input,range.Key,range.Value,actual,antipodal);}
            if(!TestTrue(TEXT("same complete 32-corner model, both centers and guards"),oldKnown==newKnown&&T::SameWhole(oracle,actual)&&
                (sample==5?!oldKnown:oldKnown)))return false;
            ++models;
        }
    }
    TestEqual(TEXT("96 original motion/antipodal/FTZ comparisons"),models,96);
    for(uint32 flush:{0u,64u,32768u,32832u})for(int32 available:{0,425,426})
    {
        VectorSetControlRegister((initial&~uint32(0x8040))|flush);const auto v=T::Native(T::Input());
        FDistanceBudget before{available+7,13,7},after=before;FGroupedBoxVolumeBounds oracle,actual;bool oldKnown,newKnown;
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.NodeErrorUpper"),0,FName(TEXT("NodeErrorUpperBudgetOracle")));
         oldKnown=BuildGeneralNativeBoxGroupedVolume(v,before,oracle,nullptr,false);}
        {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.NodeErrorUpper"),1,FName(TEXT("NodeErrorUpperBudgetCandidate")));
         newKnown=BuildGeneralNativeBoxGroupedVolume(v,after,actual,nullptr,false);}
        if(!TestTrue(TEXT("same fine model and every consumed unit, including exhaustion"),oldKnown==newKnown&&before.Used==after.Used&&
            before.Remaining==after.Remaining&&after.ConsumptionFloor==7&&(oldKnown?GroupedBoxDetail::Same(oracle,actual):actual.Groups.IsEmpty())))return false;
        if(available==426&&!TestTrue(TEXT("full fine426 units preserved"),newKnown&&after.Used==439&&after.Remaining==7))return false;
    }
    {FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.NodeErrorUpper"),1,FName(TEXT("NodeErrorUpperNumericGuard")));
     TestTrue(TEXT("install wrong rounding"),std::fesetround(FE_UPWARD)==0);bool caught=false;FGeneralBoxMotionBounds unknown;
     try{BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(T::Input(),0.f,1.f/300.f,unknown);}catch(const FAnalyticNumericUnknown&){caught=true;}
     TestTrue(TEXT("fresh entry numeric guard is not bypassed"),caught);}
    TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FClosedGroupedVolumeNativeProductionTest,
 "IAmSpeed.AnalyticWorld.ClosedGroupedVolume.NativeProduction",EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FClosedGroupedVolumeNativeProductionTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;namespace T=GeneralBoxPrologueReuseTests;
 const auto saved=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(saved);std::fenv_t held;
 if(!saved.bValid||!restore.IsApplied()||std::feholdexcept(&held)!=0||std::fesetround(FE_TONEAREST)!=0)return false;
 const uint32 control=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})for(int32 correlated:{0,1})
 {
  VectorSetControlRegister((control&~uint32(0x8040))|flush);
  FScopedCVar<int32> flag(TEXT("p.IAmSpeed.AnalyticWorld.CorrelatedNativeBoxGroups"),correlated,FName(TEXT("ClosedNativeCorrelated")));
  const auto input=T::Input();const auto v=T::Native(input);
  for(bool fine:{false,true})for(int32 available:{0,1,117,118,119,201,202,203,425,426,427,5000})
  {
   FDistanceBudget a{available+7,13,7},b=a;FGroupedBoxVolumeBounds before,after;FClosedGroupedBoxVolumePtr closed;
   const bool old=fine?CorrelatedNativeBoxGroups::BuildFineOrOriginal(v,a,before):PreparedExecution::NativePredictionCoarseDetail::BuildVolume(v,a,before);
   const bool now=fine?CorrelatedNativeBoxGroups::BuildFineOrOriginal(v,b,after,nullptr,&closed):PreparedExecution::NativePredictionCoarseDetail::BuildVolume(v,b,after,nullptr,&closed);
   if(!TestTrue(TEXT("real coarse/fine builders preserve caps, partial cost and exact closed values"),old==now&&a.Used==b.Used&&a.Remaining==b.Remaining&&
    (now?(closed&&GroupedBoxDetail::Same(before,after)&&GroupedBoxDetail::Same(after,closed->GetVolumeForInspection())):!closed)))return false;
   if(!now)continue;
   FIntervalTensorSurface far;far.DegreeU=far.DegreeV=1;
   for(int32 bits=0;bits<4;++bits)far.ControlPoints.Add(Detail::Point(FVector3d(bits&1?100:-100,bits&2?100:-100,-10000)));
   FIndexedIntervalUnionPtr scene;FDistanceBudget make{10000};if(!FIndexedIntervalUnion::Build(MakeArrayView(&far,1),make,scene))return false;
   FReferenceBoundsCandidateSetPtr partition;if(!FReferenceBoundsCandidateSet::Select(scene,after.BroadHull,make,partition))return false;
   FDistanceBudget pa{10000},pb=pa;FReferenceBoxVolumeStrictSeparationPtr x,y;
   if(!TestTrue(TEXT("real native certificate and endpoint consumer agree"),
    FReferenceBoxVolumeStrictSeparation::TryProve(partition,before,pa,x)&&FReferenceBoxVolumeStrictSeparation::TryProveClosed(partition,closed,pb,y)&&
    x&&y&&pa.Used==pb.Used&&x->Matches(scene,after)&&y->Matches(scene,after)))return false;
   auto pose=v.Query;pose.Start=pose.End=v.BodyState.Location;
   if(!TestTrue(TEXT("fresh physical and query endpoint checks remain authoritative"),GroupedBoxProofContainsEndpoint(x,pose,v.PhysicalHalfExtent,v.QueryHalfExtent)&&
    GroupedBoxProofContainsEndpoint(y,pose,v.PhysicalHalfExtent,v.QueryHalfExtent)))return false;
  }
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace Speed::Analytic::Certified::InstantDirectionalSupportTests
{
namespace A=::AnalyticCertified;
namespace Y=YawFloorBoxDetail;
namespace D=InstantDirectionalSupportDetail;
using I=::AnalyticCertified::Interval;
static_assert(sizeof(D::FPreparedFamily)<=512,"Bounded owned directional family; no hidden allocation");
static bool MatrixFromQuaternion(const Y::FQuaternion& q,D::FMatrix& output)
{
    output={};
    const auto two=I::Exact(2.);D::FMatrix result;
    for(int32 i=0;i<3;++i)
    {
        const int32 j=(i+1)%3,k=(i+2)%3;
        result.Element[i][i]=A::Subtract(I::Exact(1.),A::Multiply(two,A::Add(A::Square(q.Axis[j].Range),A::Square(q.Axis[k].Range))));
        result.Element[i][j]=A::Multiply(two,A::Subtract(A::Multiply(q.Axis[i].Range,q.Axis[j].Range),A::Multiply(q.Axis[3].Range,q.Axis[k].Range)));
        result.Element[i][k]=A::Multiply(two,A::Add(A::Multiply(q.Axis[i].Range,q.Axis[k].Range),A::Multiply(q.Axis[3].Range,q.Axis[j].Range)));
        for(int32 d=0;d<3;++d)if(!A::Valid(result.Element[i][d]))return false;
    }
    output=result;return true;
}
static bool UniformRotateError(const Y::FQuaternion& q,const FVector3d& extent,FVector3d& output)
{
    output=FVector3d::ZeroVector;Y::FPoint p,v;
    for(int32 d=0;d<3;++d){p.Axis[d]={I{-extent[d],extent[d],true},0.};v.Axis[d]=q.Axis[d];}
    auto twice=Y::Cross(v,p);for(int32 d=0;d<3;++d)twice.Axis[d]=Y::Product(Y::Leaf(2.),twice.Axis[d]);
    const auto cross=Y::Cross(v,twice);
    for(int32 d=0;d<3;++d)
    {
        const auto value=Y::Sum(Y::Sum(p.Axis[d],Y::Product(q.Axis[3],twice.Axis[d])),cross.Axis[d]);
        if(!Y::ValidNode(value))return false;output[d]=value.Error;
    }
    return true;
}
static double Dot(const FVector3d& a,const FVector3d& b)
{return (a.X*b.X+a.Y*b.Y)+a.Z*b.Z;}
static bool Contains(I bound,double point)
{return A::Valid(bound)&&A::Finite(point)&&point>=bound.Lower&&point<=bound.Upper;}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInstantDirectionalSupportFamilyTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.DirectionalSupportFamilyContainment",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInstantDirectionalSupportFamilyTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace A=::AnalyticCertified;namespace T=InstantDirectionalSupportTests;namespace D=InstantDirectionalSupportDetail;namespace Y=YawFloorBoxDetail;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("guarded RNE support fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const uint32 saved=VectorGetControlRegister();int32 nativeChecks=0,exactChecks=0;
 const FVector3d axes[]={FVector3d(1,0,0),FVector3d(0,-2,0),FVector3d(.5,-1,.25),FVector3d(0,1,-1),FVector3d(0,0,0)};
 const FQuat4d rotations[]={FQuat4d::Identity,FQuat4d(.125,-.25,.375,.875),FQuat4d(-.5,.25,.125,1.25)};
 for(uint32 flush:{0u,64u,32768u,32832u})for(uint32 sticky:{0u,63u})
 {
  VectorSetControlRegister((saved&~uint32(0x807f))|flush|sticky);
  for(const auto& q:rotations)for(bool family:{false,true})
  {
   Y::FQuaternion qp=Y::Quaternion(q);
   if(family)for(auto& component:qp.Axis)component.Range=A::Add(component.Range,Interval{-.015625,.015625,true});
   const FVector3d extent(32.,8.,4.);D::FMatrix matrix;FVector3d error;
   const FIntervalPoint center={{{9.,11.,true},{-5.,-3.,true},{19.,21.,true}}};
   FDistanceBudget prepare{1000};D::FPreparedFamily prepared;
   if(!TestTrue(TEXT("certified quaternion-family matrix/error are admitted"),T::MatrixFromQuaternion(qp,matrix)&&T::UniformRotateError(qp,extent,error)&&
    D::FPreparedFamily::TryPrepare(matrix,extent,center,error,prepare,prepared)&&prepared.IsPrepared()&&prepare.Used==D::FPreparedFamily::PrepareWork))return false;
   for(const auto& axis:axes)
   {
    Interval slab;FDistanceBudget project{1000};
    if(!TestTrue(TEXT("same-control directional projection"),prepared.TryProject(axis,project,slab)&&project.Used==D::FPreparedFamily::ProjectWork))return false;
    for(int32 realization=0;realization<3;++realization)for(int32 corner=0;corner<8;++corner)
    {
     FQuat4d actual=q;
     if(family){const double change=(realization-1)*.0078125;actual.X+=change;actual.Y-=change;actual.Z+=change;actual.W-=change;}
     FVector3d local,actualCenter;
     for(int32 d=0;d<3;++d){local[d]=(corner&(1<<d))?extent[d]:-extent[d];actualCenter[d]=(corner&(1<<d))?center.Axis[d].Upper:center.Axis[d].Lower;}
     const auto rotated=actual.RotateVector(local);
     const FVector3d point=actualCenter+rotated;
     if(!TestTrue(TEXT("each real native Rotate and final Add realization remains inside the projected family"),T::Contains(slab,T::Dot(axis,point))))return false;
     ++nativeChecks;
    }
   }
  }
  // Independent dyadic matrix families, not only quaternion-generated nets.
  // All scalar products here are small dyadic rationals exactly representable
  // in binary64; this is a distinct oracle from the interval implementation.
  D::FMatrix independent;for(int32 d=0;d<3;++d)for(int32 j=0;j<3;++j)
  {const double value=double(d*3+j-4)/8.;independent.Element[d][j]={value-.125,value+.125,true};}
  const FVector3d extent(4,2,1);FIntervalPoint center=Detail::Point(FVector3d(1,-2,3));
  D::FPreparedFamily prepared;FDistanceBudget work{1000};
  if(!D::FPreparedFamily::TryPrepare(independent,extent,center,FVector3d::ZeroVector,work,prepared))return false;
  for(const auto& axis:axes)
  {
   Interval slab;if(!prepared.TryProject(axis,work,slab))return false;
   for(int32 realization=0;realization<16;++realization)for(int32 corner=0;corner<8;++corner)
   {
    FVector3d point(1,-2,3);
    for(int32 d=0;d<3;++d)for(int32 j=0;j<3;++j)
    {const auto c=independent.Element[d][j];const double coefficient=((realization+(1<<d)+j)&1)?c.Upper:c.Lower;
     point[d]+=coefficient*((corner&(1<<j))?extent[j]:-extent[j]);}
    if(!TestTrue(TEXT("independent coefficient realizations and local corners remain enclosed"),T::Contains(slab,T::Dot(axis,point))))return false;
    ++exactChecks;
   }
  }
  // Material cancellation: the long local X direction is orthogonal to n.
  // Projecting the world AABB would give radius202; projection before abs
  // retains radius2. The helper is still only a bound, never a clear result.
  D::FMatrix cancel;const double entries[3][3]={{1,-1,0},{1,1,0},{0,0,1}};
  for(int32 d=0;d<3;++d)for(int32 j=0;j<3;++j)cancel.Element[d][j]=Interval::Exact(entries[d][j]);
  D::FPreparedFamily tight;FDistanceBudget cancelWork{100};Interval tightSlab;
  if(!TestTrue(TEXT("orientation survives until projection"),D::FPreparedFamily::TryPrepare(cancel,FVector3d(100,1,1),Detail::Point(FVector3d::ZeroVector),FVector3d::ZeroVector,cancelWork,tight)&&
   tight.TryProject(FVector3d(1,-1,0),cancelWork,tightSlab)&&tightSlab.Lower>-3.&&tightSlab.Upper<3.))return false;
 }
 if(!TestTrue(TEXT("nonvacuous actual-native and independent-family oracles"),nativeChecks==5760&&exactChecks==5120))return false;
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInstantDirectionalSupportGuardsTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.DirectionalSupportControlsAndBudget",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInstantDirectionalSupportGuardsTest::RunTest(const FString&)
{
 using namespace Speed;using namespace Analytic;using namespace Certified;
 namespace A=::AnalyticCertified;namespace D=InstantDirectionalSupportDetail;namespace T=InstantDirectionalSupportTests;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("guarded RNE support controls"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})
 {
  const uint32 control=(saved&~uint32(0x807f))|flush;VectorSetControlRegister(control);
  D::FMatrix matrix;for(int32 d=0;d<3;++d)for(int32 j=0;j<3;++j)matrix.Element[d][j]=Interval::Exact(d==j?1.:0.);
  const auto center=Detail::Point(FVector3d(3,-5,7));D::FPreparedFamily ready;FDistanceBudget enough{100};
  if(!D::FPreparedFamily::TryPrepare(matrix,FVector3d(2,3,4),center,FVector3d::ZeroVector,enough,ready))return false;
  for(int32 cap=0;cap<=D::FPreparedFamily::PrepareWork+1;++cap)
  {
   FDistanceBudget budget{cap+7,11,7};auto result=ready;
   const bool known=D::FPreparedFamily::TryPrepare(matrix,FVector3d(2,3,4),center,FVector3d::ZeroVector,budget,result);
   const int32 paid=FMath::Min(cap,D::FPreparedFamily::PrepareWork);
   if(!TestTrue(TEXT("prepare partial debit/floor and stale output"),known==(cap>=D::FPreparedFamily::PrepareWork)&&result.IsPrepared()==known&&
    budget.Remaining==cap+7-paid&&budget.Used==uint64(11+paid)&&budget.ConsumptionFloor==7))return false;
  }
  for(int32 cap=0;cap<=D::FPreparedFamily::ProjectWork+1;++cap)
  {
   FDistanceBudget budget{cap+7,11,7};Interval output=Interval::Exact(42.);
   const bool known=ready.TryProject(FVector3d(1,-2,.5),budget,output);const int32 paid=FMath::Min(cap,D::FPreparedFamily::ProjectWork);
   if(!TestTrue(TEXT("projection partial debit/floor and stale output"),known==(cap>=D::FPreparedFamily::ProjectWork)&&output.Known==known&&
    budget.Remaining==cap+7-paid&&budget.Used==uint64(11+paid)&&budget.ConsumptionFloor==7))return false;
  }
  FDistanceBudget overflow{100,MAX_uint64-2,7};Interval unknown=Interval::Exact(42.);
  if(!TestTrue(TEXT("Used overflow is sticky with no publication"),!ready.TryProject(FVector3d(1,0,0),overflow,unknown)&&!unknown.Known&&overflow.Used==MAX_uint64&&overflow.Remaining==98))return false;
  FDistanceBudget invalidFloor{100,0,-1};if(!TestTrue(TEXT("invalid floor refuses before geometry"),!ready.TryProject(FVector3d(1,0,0),invalidFloor,unknown)&&invalidFloor.Used==0&&!unknown.Known))return false;
  FDistanceBudget changed{100};VectorSetControlRegister(control^64u);
  if(!TestTrue(TEXT("prepared values cannot cross FTZ/DAZ controls"),!ready.TryProject(FVector3d(1,0,0),changed,unknown)&&!unknown.Known&&changed.Used==0))return false;
  VectorSetControlRegister(control|0x2000u);
  if(!TestTrue(TEXT("non-RNE is refused"),!ready.TryProject(FVector3d(1,0,0),changed,unknown)&&changed.Used==0))return false;
  VectorSetControlRegister(control|63u);Interval withFlags;FDistanceBudget flags{100};
  if(!TestTrue(TEXT("history does not change a prepared geometry's mathematical support"),ready.TryProject(FVector3d(1,0,0),flags,withFlags)&&T::Contains(withFlags,5.)))return false;
  VectorSetControlRegister(control);
  auto bad=matrix;bad.Element[1][2]={};auto failed=ready;FDistanceBudget badWork{100};
  if(!TestTrue(TEXT("unknown matrix cannot retain an old packet"),!D::FPreparedFamily::TryPrepare(bad,FVector3d(1,1,1),center,FVector3d::ZeroVector,badWork,failed)&&!failed.IsPrepared()))return false;
  bad=matrix;bad.Element[0][0]=Interval::Exact(std::numeric_limits<double>::max());failed=ready;
  if(!TestTrue(TEXT("arithmetic overflow does not publish"),!D::FPreparedFamily::TryPrepare(bad,FVector3d(2,1,1),center,FVector3d::ZeroVector,badWork,failed)&&!failed.IsPrepared()))return false;
  if(!TestTrue(TEXT("negative error does not publish"),!D::FPreparedFamily::TryPrepare(matrix,FVector3d(1,1,1),center,FVector3d(-1,0,0),badWork,failed)&&!failed.IsPrepared()))return false;
  FDistanceBudget axisWork{100};unknown=Interval::Exact(42.);
  if(!TestTrue(TEXT("nonfinite direction is not a support"),!ready.TryProject(FVector3d(std::numeric_limits<double>::infinity(),0,0),axisWork,unknown)&&!unknown.Known))return false;
  // Signed zeros and an actual subnormal coordinate/error stay fail-closed
  // under all four controls, without fabricating or restoring sticky flags.
  const double tiny=A::FromBits(A::MinNormal),sub=A::FromBits(1);D::FPreparedFamily small;FDistanceBudget smallWork{100};
  if(!TestTrue(TEXT("negative nonzero extent refuses even under DAZ"),!D::FPreparedFamily::TryPrepare(matrix,FVector3d(-sub,1,1),center,FVector3d::ZeroVector,smallWork,small)&&!small.IsPrepared()))return false;
  if(!TestTrue(TEXT("small family is covered under flush controls"),D::FPreparedFamily::TryPrepare(matrix,FVector3d(tiny,0.,-0.),Detail::Point(FVector3d(sub,0,0)),FVector3d(sub,0,0),smallWork,small)&&
   small.TryProject(FVector3d(1,-0.,0),smallWork,unknown)&&T::Contains(unknown,0.)&&T::Contains(unknown,tiny)))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
namespace NativeDirectionalClearanceTests
{
using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
static PreparedExecution::FNativeSolidStaticValues Capture()
{
    PreparedExecution::FNativeSolidStaticValues v;v.Shape=PreparedExecution::ENativeSolidQueryShape::Box;
    v.OwnerState.Location=FVector::ZeroVector;v.OwnerState.Rotation=FQuat(FVector::ForwardVector,UE_DOUBLE_PI*.25);
    v.OwnerState.AngularVelocity=FVector(.1,0,0);v.LocalRotation=FQuat::Identity;v.Horizon=1.f/300.f;
    v.PhysicalHalfExtent=FVector(1,3,.1);v.QueryHalfExtent=FVector(1,3,.1);v.BodyState=v.OwnerState;
    v.Query.Shape=EQueryShape::Box;v.Query.Start=v.Query.End=v.BodyState.Location;v.Query.Rotation=v.BodyState.Rotation;
    v.Query.HalfExtent=v.QueryHalfExtent;v.Query.DomainTolerance=.001;v.Query.InitialOverlapTolerance=.001;return v;
}
static FIntervalTensorSurface Surface(double shift=1.)
{
    FIntervalTensorSurface s;s.DegreeU=s.DegreeV=5;
    for(int32 u=0;u<6;++u)for(int32 v=0;v<6;++v)
    {
        const double y=-1.+2.*double(v)/5.;
        // y + shift + .1*y^2, represented in degree5 Bernstein.
        const double z=-1.+shift+.1+1.6*double(v)/5.+.4*double(v*(v-1))/20.;
        s.ControlPoints.Add(Detail::Point(FVector3d(-10.+20.*double(u)/5.,y,z)));
    }
    return s;
}
static bool Scene(double shift,FIndexedIntervalUnionPtr& scene)
{
    auto surface=Surface(shift);FDistanceBudget init{100000};
    if(!FIndexedIntervalUnion::Build(MakeArrayView(&surface,1),init,scene))return false;
    scene->PrepareStaticBodyHierarchyAtPublication(init);return !!scene->GetStaticBodyHierarchy();
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeDirectionalCompleteFamiliesTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.DirectionalCompleteMotionFamilies",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeDirectionalCompleteFamiliesTest::RunTest(const FString&)
{
 using namespace InstantMotionClearanceTests;namespace D=InstantDirectionalSupportDetail;
 const bool readPrepared=FParse::Param(FCommandLine::Get(),TEXT("SpeedCase202PreparedRoofEightCorners"));
 if(readPrepared)
 {
  FString command;
  PreparedExecution::FStepIdentity step;
  if(!TestTrue(TEXT("case 202 Prepared readback has a dedicated headless unit invocation"),
   PreparedExecution::Requested()&&!PreparedExecution::ReadCurrentStep(step)&&
   FParse::Param(FCommandLine::Get(),TEXT("NullRHI"))&&FParse::Param(FCommandLine::Get(),TEXT("unattended"))&&
   !FParse::Param(FCommandLine::Get(),TEXT("SLStadiumNativeFastBatch"))&&
   FParse::Value(FCommandLine::Get(),TEXT("ExecCmds="),command)&&
   command==TEXT("Automation RunTests IAmSpeed.AnalyticWorld.NativeSolid.DirectionalCompleteMotionFamilies")))return false;
 }
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE complete directional families"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 const uint32 saved=VectorGetControlRegister();int32 checks=0;
 if(!readPrepared)
 {
 const FVector3d directions[]={FVector3d(1,0,0),FVector3d(0,1,-1),FVector3d(.25,-.5,1)};
 for(uint32 flush:{0u,64u,32768u,32832u})for(int32 sample=0;sample<13;++sample)
 {
  VectorSetControlRegister((saved&~uint32(0x807f))|flush);const auto v=Native(sample);
  D::FCompletePacket packet;FIntervalPoint oldHull,newHull;FDistanceBudget make{1000};
  if(!TestTrue(TEXT("same capture/prologue and exact broad-hull bits with optional owned packet"),X::BuildCompleteVolume(v,oldHull)&&
   X::BuildCompleteVolume(v,newHull,&packet,&make)&&GroupedBoxDetail::SamePoint(oldHull,newHull)&&make.Used==40))return false;
  Private::FBoxSweepContext sat;sat.Initialize(v.Query);
  for(const auto& direction:directions)
  {
   Interval support;FDistanceBudget work{1000};
   if(!TestTrue(TEXT("all seven native/query families projected together"),packet.TryProject(direction,work,support)&&work.Used==60))return false;
   for(int32 step=0;step<=8;++step)
   {
    const float time=float(double(v.Horizon)*double(step)/8.);
    const auto ownerQ=::Speed::SBox::IntegrateRotation(v.OwnerState.Rotation,v.OwnerState.AngularVelocity,v.OwnerState.AngularAcceleration,time);
    const auto ownerP=::Speed::SBox::AdvancePosition(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,time);
    const auto bodyP=(ownerP-ownerQ.RotateVector(v.LocalCOM))+ownerQ.RotateVector(v.LocalOffset);const auto bodyQ=ownerQ*v.LocalRotation;
    const double td=double(time);const auto theta=v.OwnerState.AngularVelocity*td+.5*v.OwnerState.AngularAcceleration*(td*td);
    FQuat referenceOwner=v.OwnerState.Rotation;
    if(!theta.IsZero())
    {
     const double x=theta.SizeSquared(),angle=FMath::Sqrt(x);
     const double scale=x<1.e-8?.5-x/48.+x*x/3840.:FMath::Sin(angle*.5)/angle;
     const double cosine=FMath::Cos(angle*.5);
     if(!GeneralBoxDetail::CheckedNativeExpResults(x,scale,cosine))return false;
     referenceOwner=FQuat(theta.X*scale,theta.Y*scale,theta.Z*scale,cosine)*v.OwnerState.Rotation;
    }
    const auto referenceP=v.OwnerState.Location+v.OwnerState.Velocity*td+.5*v.OwnerState.Acceleration*(td*td);
    const auto referenceBodyP=(referenceP-referenceOwner.RotateVector(v.LocalCOM))+referenceOwner.RotateVector(v.LocalOffset);
    const auto referenceBodyQ=referenceOwner*v.LocalRotation;
    const auto chord=v.Query.Start+(v.Query.End-v.Query.Start)*(double(step)/8.);
    for(int32 shape=0;shape<2;++shape)for(int32 corner=0;corner<8;++corner)
    {
     const auto extent=shape?v.QueryHalfExtent:v.PhysicalHalfExtent;FVector3d p;
     for(int32 d=0;d<3;++d)p[d]=(corner&(1<<d))?extent[d]:-extent[d];
     const auto fixed=v.Query.Rotation.RotateVector(p),axes=sat.Axis[0]*p.X+sat.Axis[1]*p.Y+sat.Axis[2]*p.Z;
     const FVector3d actual[]={bodyP+bodyQ.RotateVector(p),referenceBodyP+referenceBodyQ.RotateVector(p),v.BodyState.Location+fixed,v.BodyState.Location+axes};
     for(const auto& point:actual)
     {if(!TestTrue(TEXT("native motion and captured pose Rotate/SAT stay inside exact support"),InstantDirectionalSupportTests::Contains(support,InstantDirectionalSupportTests::Dot(direction,point))))return false;++checks;}
     if(shape)
     {
      for(const auto& point:{chord+fixed,chord+axes,bodyP+fixed,bodyP+axes,referenceBodyP+fixed,referenceBodyP+axes})
      {if(!TestTrue(TEXT("literal World chord and moving fixed footprints cannot be omitted"),InstantDirectionalSupportTests::Contains(support,InstantDirectionalSupportTests::Dot(direction,point))))return false;++checks;}
     }
    }
   }
  }
  auto bad=v;bad.Query.End.X+=1.;D::FCompletePacket refused=packet;FIntervalPoint missing;
  if(!TestTrue(TEXT("query delta refuses and clears packet before projection"),!X::BuildCompleteVolume(bad,missing,&refused,&make)&&!Detail::ValidPoint(missing)))return false;
  Interval stale=Interval::Exact(1.);FDistanceBudget after{1000};if(!TestTrue(TEXT("failed capture cannot project stale support"),!refused.TryProject(FVector3d(1,0,0),after,stale)&&!stale.Known&&after.Used==0))return false;
 }
 if(!TestTrue(TEXT("nonvacuous native/reference complete-family sample coverage"),checks==157248))return false;
 // A stationary exact box makes the two shells independently observable,
 // unlike a moving capture whose wider bounds could hide a missing shell.
 D::FMatrix identity,zero;
 for(int32 i=0;i<3;++i)for(int32 j=0;j<3;++j){identity.Element[i][j]=Interval::Exact(i==j?1.:0.);zero.Element[i][j]=Interval::Exact(0.);}
 FWorldQuery query;query.Shape=EQueryShape::Box;query.HalfExtent=FVector3d(1.);query.Rotation=FQuat4d::Identity;
 Private::FBoxSweepContext sat;sat.Initialize(query);const auto origin=Detail::Point(FVector3d::ZeroVector);
 D::FCompletePacket shellPacket;FDistanceBudget shellWork{1000};Interval shellSlab;
 if(!TestTrue(TEXT("both literal nonnegative shells expand a nonunit directional support"),D::FCompletePacket::TryBuild(identity,zero,origin,sat,FVector3d(1.),FVector3d(1.),0.,origin,origin,1.,2.,shellWork,shellPacket)&&
  shellPacket.TryProject(FVector3d(1,-2,.5),shellWork,shellSlab)&&shellSlab.Lower<=-14.&&shellSlab.Upper>=14.&&shellSlab.Upper<14.001))return false;
 FDistanceBudget partial{39};auto stalePacket=shellPacket;FIntervalPoint empty;
 if(!TestTrue(TEXT("partial complete-packet construction publishes neither hull nor old packet"),!X::BuildCompleteVolume(Native(0),empty,&stalePacket,&partial)&&!Detail::ValidPoint(empty)&&partial.Used==39&&partial.Remaining==0))return false;
 FDistanceBudget noStale{100};if(stalePacket.TryProject(FVector3d(1,0,0),noStale,shellSlab)||shellSlab.Known||noStale.Used!=0)return false;
 }
 const bool readCorner5=FParse::Param(FCommandLine::Get(),TEXT("SpeedCase202NativeRoofReadback"));
 const bool readEight=FParse::Param(FCommandLine::Get(),TEXT("SpeedCase202NativeRoofEightCorners"));
 if(readPrepared||((readCorner5||readEight)&&!PreparedExecution::Requested()))
 {
  // Private frame 127 serial 2 fixture. Observe the existing compiled evaluator
  // after the ordinary checks, or in a dedicated Prepared unit invocation.
  // Any Prepared failure is confined to that diagnostic actor.
  namespace A=::AnalyticCertified;
  const auto Double=[](uint64 bits){return A::FromBits(bits);};
  FKinematicState initial;
  initial.Location=FVector(Double(0x406163b749882f08ull),Double(0x40ad561dc0fb2199ull),Double(0x403c0a9930be2b32ull));
  initial.Rotation=FQuat(Double(0xbfe81c40529b70c8ull),Double(0xbfe50a4ba0cd84b6ull),Double(0xbbed2d08e9395abaull),Double(0xbc032a098de15c18ull));
  initial.Velocity=FVector(Double(0x4083bc0000000000ull),Double(0x40a068eb80000000ull),Double(0x0000000000000000ull));
  initial.Acceleration=FVector(Double(0x0000000000000000ull),Double(0x0000000000000000ull),Double(0x3d50000000000000ull));
  initial.AngularVelocity=FVector(Double(0x8000000000000000ull),Double(0x8000000000000000ull),Double(0xbd0d00caef468948ull));
  initial.AngularAcceleration=FVector(Double(0x0000000000000000ull),Double(0xbcf0000000000000ull),Double(0x0000000000000000ull));
  const FVector localCOM(Double(0xc02c0463497b7415ull),Double(0x0000000000000000ull),Double(0x40316a71de69ad43ull)),localOffset(Double(0xc014000000000000ull),Double(0x0000000000000000ull),Double(0x403d828f5c28f5c3ull)),extent(Double(0x40506a3d70a3d70aull),Double(0x4045cae7d566cf42ull),Double(0x402fe4f765fd8adbull));
  const FQuat localQ(Double(0x0000000000000000ull),Double(0x8000000000000000ull),Double(0x0000000000000000ull),Double(0x3ff0000000000000ull));
  const FVector corner5(extent.X,-extent.Y,extent.Z);
  constexpr uint32 first=0x3b1e2085u,eightLast=0x3b1e21deu;
  const uint32 last=readCorner5&&!readPrepared?0x3b1e2325u:eightLast;
  for(const uint32 flush:{0u,64u,32768u,32832u})
  {
   for(uint32 bits=first;bits<=last;++bits)
   {
    VectorSetControlRegister((saved&~uint32(0x807f))|flush);
    constexpr auto contract=Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush;
    const auto before=readPrepared?Analytic::InspectCurrentAnalyticNumericContext(contract):Analytic::FAnalyticNumericEvidence{};
    if(readPrepared&&before.Status!=Analytic::EAnalyticNumericStatus::Complete)return false;
    const float time=KinematicCenterDetail::FromBits(bits);
    auto pose=initial.Integrate(time);
    pose.Location-=pose.Rotation.RotateVector(localCOM);
    pose.Location+=pose.Rotation.RotateVector(localOffset);
    pose.Rotation=pose.Rotation*localQ;
    if(readCorner5&&!readPrepared)
    {
     const auto vertex=pose.Location+pose.Rotation.RotateVector(corner5);
     UE_LOG(LogTemp,Display,TEXT("[Case202NativeRoofReadback] Frame=127 Serial=2 Provider=1931 Corner=5 Time=%08x Flush=%04x Prepared=0 X=%016llx Y=%016llx Z=%016llx"),
      bits,flush,A::Bits(vertex.X),A::Bits(vertex.Y),A::Bits(vertex.Z));
    }
    if((readEight||readPrepared)&&bits<=eightLast)
    {
     for(int32 k=0;k<8;++k)
     {
      const FVector corner((k&1)?extent.X:-extent.X,(k&2)?extent.Y:-extent.Y,(k&4)?extent.Z:-extent.Z);
      const auto vertex=pose.Location+pose.Rotation.RotateVector(corner);
      if(readPrepared)
      {
       const auto after=Analytic::InspectCurrentAnalyticNumericContext(contract);
       if(after.Status!=Analytic::EAnalyticNumericStatus::Complete||
          after.ControlFingerprint()!=before.ControlFingerprint()||vertex.ContainsNaN())return false;
       UE_LOG(LogTemp,Display,TEXT("[Case202PreparedRoofEightCorners] Frame=127 Serial=2 Provider=1931 Corner=%d Time=%08x Flush=%04x Prepared=1 X=%016llx Y=%016llx Z=%016llx Numeric=%016llx"),
        k,bits,flush,A::Bits(vertex.X),A::Bits(vertex.Y),A::Bits(vertex.Z),after.ControlFingerprint());
      }
      else
       UE_LOG(LogTemp,Display,TEXT("[Case202NativeRoofEightCorners] Frame=127 Serial=2 Provider=1931 Corner=%d Time=%08x Flush=%04x Prepared=0 X=%016llx Y=%016llx Z=%016llx"),
        k,bits,flush,A::Bits(vertex.X),A::Bits(vertex.Y),A::Bits(vertex.Z));
     }
    }
   }
  }
  VectorSetControlRegister(saved);
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeDirectionalCoarseTransportTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.DirectionalCoarseReserveAndEndpoint",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeDirectionalCoarseTransportTest::RunTest(const FString&)
{
 using namespace NativeDirectionalClearanceTests;namespace P=PreparedExecution::NativePredictionCoarseDetail;
 const auto original=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
 if(!TestTrue(TEXT("RNE directional coarse"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
 FScopedCVar<int32> enabled(TEXT("p.IAmSpeed.AnalyticWorld.NativeDirectionalMotionClearance"),1,FName(TEXT("DirectionalCoarseOn")));
 FScopedCVar<int32> atlas(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy"),3,FName(TEXT("DirectionalCoarseAtlas")));
 FScopedCVar<int32> depth(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.Depth"),3,FName(TEXT("DirectionalCoarseDepth")));
 FScopedCVar<int32> reduction(TEXT("p.IAmSpeed.AnalyticWorld.StaticBodyHierarchy.DegreeReduction"),1,FName(TEXT("DirectionalCoarseReduced")));
 const uint32 saved=VectorGetControlRegister();
 for(uint32 flush:{0u,64u,32768u,32832u})
 {
  VectorSetControlRegister((saved&~uint32(0x807f))|flush);const auto v=Capture();FIndexedIntervalUnionPtr scene;
  if(!TestTrue(TEXT("published original-source hierarchy and supports"),Scene(1.,scene)))return false;
  const int32 allowance=scene->Num()+2048,fine=8192;FDistanceBudget budget{fine+allowance};
  FGroupedBoxVolumeBounds volume;FReferenceBoundsCandidateSetPtr partition;FReferenceBoxVolumeStrictSeparationPtr oldProof;FReferenceDirectionalMotionStrictSeparationPtr proof;
  int32 externalCalls=0;
  const auto result=P::Attempt(scene,v,budget,allowance,[&](const FIntervalPoint&){++externalCalls;return true;},volume,partition,oldProof,nullptr,&proof);
  if(!TestTrue(TEXT("real new coarse route publishes distinct directional region before external guard"),result==P::EAttempt::Clear&&proof&&!oldProof&&volume.Groups.IsEmpty()&&
   proof->GetDirectionCountForInspection()>0&&partition&&partition->GetCandidateIndices().Num()==1&&externalCalls==1&&
   budget.Remaining>=fine&&budget.ConsumptionFloor==0&&budget.Used+uint64(budget.Remaining)==uint64(fine+allowance)&&proof->Matches(scene,proof->GetBroadHullForInspection())))return false;
  auto pose=v.Query;pose.Start=pose.End=v.BodyState.Location;
  if(!TestTrue(TEXT("fresh original endpoint representations satisfy EVERY retained slab"),PreparedExecution::NativeDirectionalCoarseDetail::ContainsEndpoint(proof,pose,v.PhysicalHalfExtent,v.QueryHalfExtent)))return false;
  pose.Start=pose.End=FVector3d(0,-.25,.75);
  if(!TestFalse(TEXT("endpoint escaping a retained direction cannot borrow old broad-region permission"),PreparedExecution::NativeDirectionalCoarseDetail::ContainsEndpoint(proof,pose,FVector3d(.01),FVector3d(.01))))return false;
  const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint();
  if(!TestTrue(TEXT("same captured inputs remain the primitive-reuse prerequisite"),PreparedExecution::PredictionPrimitiveReuseDetail::SameInputs(v,v.OwnerState,v.Horizon,v,numeric,numeric)))return false;
  auto changed=v;changed.Query.End.X+=1.;
  if(!TestFalse(TEXT("changed query cannot reuse motion proof"),PreparedExecution::PredictionPrimitiveReuseDetail::SameInputs(changed,v.OwnerState,v.Horizon,v,numeric,numeric)))return false;
  auto actual=v.OwnerState;actual.Location.X+=1.;
  if(!TestFalse(TEXT("changed actual state cannot reuse motion proof"),PreparedExecution::PredictionPrimitiveReuseDetail::SameInputs(v,actual,v.Horizon,v,numeric,numeric)))return false;
  if(!TestFalse(TEXT("changed numeric controls cannot reuse motion proof"),PreparedExecution::PredictionPrimitiveReuseDetail::SameInputs(v,v.OwnerState,v.Horizon,v,numeric,numeric^64u)))return false;
  FIndexedIntervalUnionPtr crossing;if(!Scene(0.,crossing))return false;FDistanceBudget failed{fine+allowance};externalCalls=0;
  const auto refused=P::Attempt(crossing,v,failed,allowance,[&](const FIntervalPoint&){++externalCalls;return true;},volume,partition,oldProof,nullptr,&proof);
  const uint64 spent=failed.Used;
  if(!TestTrue(TEXT("real intersection keeps fine allocation and publishes no region"),refused==P::EAttempt::KeepFine&&!proof&&!oldProof&&externalCalls==0&&spent>0&&failed.Remaining>=fine&&failed.ConsumptionFloor==0))return false;
  FGroupedBoxVolumeBounds fallback,oracle;FDistanceBudget ordinary{fine};bool built=false;
  {FScopedDistanceBudgetFloor protect(failed,failed.Remaining-fine);built=protect.IsValid()&&BuildGeneralNativeBoxGroupedVolume(v,failed,fallback);}
  if(!TestTrue(TEXT("original fine14 construction remains literal with cumulative charges"),built&&BuildGeneralNativeBoxGroupedVolume(v,ordinary,oracle)&&
   fallback.Groups.Num()==14&&GroupedBoxDetail::Same(fallback,oracle)&&failed.Used==spent+ordinary.Used&&failed.ConsumptionFloor==0))return false;
  FDistanceBudget noAllowance{fine};externalCalls=0;
  if(!TestTrue(TEXT("no coarse grant means no directional attempt"),P::Attempt(scene,v,noAllowance,0,[&](const FIntervalPoint&){++externalCalls;return true;},volume,partition,oldProof,nullptr,&proof)==P::EAttempt::KeepFine&&
   noAllowance.Used==0&&!proof&&externalCalls==0))return false;
 }
 TestTrue(TEXT("restore"),restore.Restore());return !HasAnyErrors();
}
#endif
