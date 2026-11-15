#pragma once
#include "TensorSphereContactBounds.h"
namespace Speed::Analytic::Certified
{
class FTensorPointRootBounds;
using FTensorPointRootBoundsPtr=TSharedPtr<const FTensorPointRootBounds,ESPMode::ThreadSafe>;
namespace TensorPointRootDetail
{
inline bool SamePoint(const FIntervalPoint& a,const FIntervalPoint& b)
{
    for(int32 d=0;d<3;++d)if(a.Axis[d].Known!=b.Axis[d].Known||
        AnalyticCertified::Bits(a.Axis[d].Lower)!=AnalyticCertified::Bits(b.Axis[d].Lower)||
        AnalyticCertified::Bits(a.Axis[d].Upper)!=AnalyticCertified::Bits(b.Axis[d].Upper))return false;
    return true;
}
inline Interval MagnitudeBound(Interval a)
{
    if(!AnalyticCertified::Valid(a))return {};
    // Integer magnitude selection does not flush a subnormal upper bound to0.
    const uint64 bits=FMath::Max(AnalyticCertified::Bits(a.Lower)&AnalyticCertified::Magnitude,
        AnalyticCertified::Bits(a.Upper)&AnalyticCertified::Magnitude);
    return {0.,AnalyticCertified::FromBits(bits),true};
}
inline bool GuessInverse(const Interval (&jacobian)[3][3],double (&out)[3][3])
{
    double a[3][3];
    for(int32 i=0;i<3;++i)for(int32 j=0;j<3;++j)
    {
        if(!AnalyticCertified::Valid(jacobian[i][j]))return false;
        a[i][j]=.5*jacobian[i][j].Lower+.5*jacobian[i][j].Upper;
        if(!FMath::IsFinite(a[i][j]))return false;
    }
    const double determinant=a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])-
        a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])+a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]);
    if(!FMath::IsFinite(determinant)||determinant==0)return false;
    // This is ONLY a finite binary64 preconditioner guess. Neither this
    // nominal determinant nor a small residual can certify any root.
    for(int32 i=0;i<3;++i)for(int32 j=0;j<3;++j)
    {
        const int32 r0=(j+1)%3,r1=(j+2)%3,c0=(i+1)%3,c1=(i+2)%3;
        out[i][j]=(a[r0][c0]*a[r1][c1]-a[r0][c1]*a[r1][c0])/determinant;
        if(!FMath::IsFinite(out[i][j]))return false;
    }
    return true;
}
inline bool PointDerivative(const FIntervalBezierCenterPath& path,FDistanceBudget& budget,FIntervalPoint& out)
{
    out={};if(!CenterDetail::Valid(path)||path.Degree<1)return false;
    for(int32 k=0;k<path.Degree;++k)
    {
        if(!budget.Consume())return false;FIntervalPoint next;
        for(int32 d=0;d<3;++d)next.Axis[d]=AnalyticCertified::Multiply(Interval::Exact(double(path.Degree)),
            AnalyticCertified::Subtract(path.ControlPoints[k+1].Axis[d],path.ControlPoints[k].Axis[d]));
        if(!Detail::ValidPoint(next))return false;out=k?TimeDetail::Hull(out,next):next;
    }
    return Detail::ValidPoint(out);
}
}
// A unique REFERENCE polynomial point/tensor root on this LOCAL parameter
// cube only. For EACH FIXED selection of coefficients from the stored boxes
// there is one root, enclosed by Parameters=(u,v,tau). These roots may differ.
// Not a Box CCD, corner-feature/normal-cone proof, earliest contact, float time,
// World hit, native response, C2 admission, physical effect or readiness grant.
class FTensorPointRootBounds final
{
    FIntervalTensorSurface Tensor;FIntervalBezierCenterPath PointPath;
    FIntervalPoint Parameters,SurfaceDu,SurfaceDv,PointVelocity;
    double ContractionUpper=1;FTensorPointRootBounds()=default;
public:
    static bool TryProve(const FIntervalTensorSurface& tensor,const FIntervalBezierCenterPath& path,
        FDistanceBudget& budget,FTensorPointRootBoundsPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(tensor.DegreeU<1||tensor.DegreeU>16||tensor.DegreeV<1||tensor.DegreeV>16||
            tensor.ControlPoints.Num()!=(tensor.DegreeU+1)*(tensor.DegreeV+1)||!CenterDetail::Valid(path)||
            path.Degree<1||!budget.Consume())return false;
        Detail::FNode node;node.DegreeU=tensor.DegreeU;node.DegreeV=tensor.DegreeV;
        for(const auto& cp:tensor.ControlPoints)
        {if(!budget.Consume()||!Detail::ValidPoint(cp))return false;node.Net.Add(cp);}
        for(const auto& cp:path.ControlPoints)if(!budget.Consume()||!Detail::ValidPoint(cp))return false;
        // BOTH surface and point derivatives use their own restricted LOCAL
        // [0,1] parameter. No untracked original-cell width/seconds factor.
        const int32 derivativeWork=tensor.DegreeU*(tensor.DegreeV+1)+(tensor.DegreeU+1)*tensor.DegreeV;
        for(int32 k=0;k<derivativeWork;++k)if(!budget.Consume())return false;
        FIntervalPoint du,dv,velocity;
        if(!ContactDetail::DerivativeHull(node,true,du)||!ContactDetail::DerivativeHull(node,false,dv)||
            !TensorPointRootDetail::PointDerivative(path,budget,velocity))return false;
        const auto surfaceMidpoint=Detail::Midpoint(node),pointMidpoint=CenterDetail::Evaluate(path,Interval::Exact(.5));
        if(!Detail::ValidPoint(surfaceMidpoint)||!Detail::ValidPoint(pointMidpoint))return false;
        Interval residual[3],jacobian[3][3];
        for(int32 i=0;i<3;++i)
        {
            residual[i]=AnalyticCertified::Subtract(surfaceMidpoint.Axis[i],pointMidpoint.Axis[i]);
            jacobian[i][0]=du.Axis[i];jacobian[i][1]=dv.Axis[i];
            jacobian[i][2]=AnalyticCertified::Multiply(Interval::Exact(-1),velocity.Axis[i]);
            if(!AnalyticCertified::Valid(residual[i]))return false;
        }
        double r[3][3];if(!budget.Consume()||!TensorPointRootDetail::GuessInverse(jacobian,r))return false;
        Interval error[3][3];uint64 normUpperBits=0;
        for(int32 i=0;i<3;++i)
        {
            auto norm=Interval::Exact(0);
            for(int32 j=0;j<3;++j)
            {
                auto product=Interval::Exact(0);
                for(int32 k=0;k<3;++k)
                {
                    if(!budget.Consume())return false;
                    product=AnalyticCertified::Add(product,AnalyticCertified::Multiply(Interval::Exact(r[i][k]),jacobian[k][j]));
                }
                error[i][j]=AnalyticCertified::Subtract(Interval::Exact(i==j?1.:0.),product);
                norm=AnalyticCertified::Add(norm,TensorPointRootDetail::MagnitudeBound(error[i][j]));
            }
            if(!AnalyticCertified::Valid(norm))return false;const uint64 bits=AnalyticCertified::Bits(norm.Upper);
            if(bits>=AnalyticCertified::Bits(1.))return false;
            normUpperBits=FMath::Max(normUpperBits,bits);
        }
        FIntervalPoint parameters;
        for(int32 i=0;i<3;++i)
        {
            auto midpoint=Interval::Exact(.5),tail=Interval::Exact(0);
            for(int32 j=0;j<3;++j)
            {
                if(!budget.Consume())return false;
                midpoint=AnalyticCertified::Subtract(midpoint,AnalyticCertified::Multiply(Interval::Exact(r[i][j]),residual[j]));
                tail=AnalyticCertified::Add(tail,AnalyticCertified::Multiply(error[i][j],Interval{-.5,.5,true}));
            }
            parameters.Axis[i]=AnalyticCertified::Add(midpoint,tail);
            if(!AnalyticCertified::Valid(parameters.Axis[i])||!(parameters.Axis[i].Lower>0&&parameters.Axis[i].Upper<1))return false;
        }
        // F=S-C, T(x)=x-RF(x), E=I-R[J]. The mean-value bound gives
        // T(X) subset K=m-RF(m)+E(X-m) subset int(X). The norm bound makes
        // T a strict contraction, hence it has exactly one fixed point in X.
        // Also ||I-RJ||<1 makes RJ invertible for any J in this nonempty
        // enclosure, so R is nonsingular and a fixed point is a zero of F.
        // The SAME tests hold uniformly for every fixed CP selection; no
        // coefficients are reselected between residual/Jacobian evaluations.
        TSharedPtr<FTensorPointRootBounds,ESPMode::ThreadSafe> value(new FTensorPointRootBounds);
        value->Tensor=tensor;value->PointPath=path;value->Parameters=parameters;
        value->SurfaceDu=du;value->SurfaceDv=dv;value->PointVelocity=velocity;
        value->ContractionUpper=AnalyticCertified::FromBits(normUpperBits);
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FIntervalTensorSurface& tensor,const FIntervalBezierCenterPath& path) const
    {
        if(Tensor.DegreeU!=tensor.DegreeU||Tensor.DegreeV!=tensor.DegreeV||
            Tensor.ControlPoints.Num()!=tensor.ControlPoints.Num()||PointPath.Degree!=path.Degree||
            PointPath.ControlPoints.Num()!=path.ControlPoints.Num())return false;
        for(int32 i=0;i<Tensor.ControlPoints.Num();++i)
            if(!TensorPointRootDetail::SamePoint(Tensor.ControlPoints[i],tensor.ControlPoints[i]))return false;
        for(int32 i=0;i<PointPath.ControlPoints.Num();++i)
            if(!TensorPointRootDetail::SamePoint(PointPath.ControlPoints[i],path.ControlPoints[i]))return false;
        return true;
    }
    // A separate REFERENCE feature test for a cloud of eight fixed offsets
    // translated with this point path as the selected vertex. This does not
    // authenticate those offsets as an actual Box or close global earliest CCD.
    // Pairwise support inequalities retain the actual supplied corner values;
    // no orthonormal/linear-Q or exact native-axis equality is assumed.
    bool ProveTranslatedCloudVertexApproach(const FIntervalPoint (&offsets)[8],int32 vertex,int32 normalSign,
        FDistanceBudget& budget,FIntervalPoint& outNormal) const
    {
        // Preserve the original inputs even when outNormal aliases offsets[k].
        FIntervalPoint captured[8];for(int32 k=0;k<8;++k)captured[k]=offsets[k];outNormal={};
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(vertex<0||vertex>=8||(normalSign!=1&&normalSign!=-1))return false;
        for(const auto& p:captured)if(!Detail::ValidPoint(p))return false;
        FIntervalPoint normal;
        for(int32 d=0;d<3;++d)
        {
            if(!budget.Consume())return false;
            normal.Axis[d]=AnalyticCertified::Multiply(Interval::Exact(double(normalSign)),
                AnalyticCertified::Subtract(AnalyticCertified::Multiply(SurfaceDu.Axis[(d+1)%3],SurfaceDv.Axis[(d+2)%3]),
                    AnalyticCertified::Multiply(SurfaceDu.Axis[(d+2)%3],SurfaceDv.Axis[(d+1)%3])));
        }
        if(!Detail::ValidPoint(normal))return false;
        for(int32 k=0;k<8;++k)if(k!=vertex)
        {
            if(!budget.Consume())return false;const auto gap=ContactDetail::Dot(normal,ContactDetail::Subtract(captured[k],captured[vertex]));
            if(!AnalyticCertified::Valid(gap)||!(gap.Lower>0))return false;
        }
        if(!budget.Consume())return false;const auto approach=ContactDetail::Dot(normal,PointVelocity);
        if(!AnalyticCertified::Valid(approach)||!(approach.Upper<0))return false;
        // Strict approach also excludes a zero raw normal. A positive normal
        // scale suffices: no unit-normal approximation/error is issued here.
        outNormal=normal;return true;
    }
    const auto& GetTensorForInspection() const{return Tensor;}
    const auto& GetPointPathForInspection() const{return PointPath;}
    const auto& GetLocalRootBounds() const{return Parameters;}
    const auto& GetLocalSurfaceDuBounds() const{return SurfaceDu;}
    const auto& GetLocalSurfaceDvBounds() const{return SurfaceDv;}
    const auto& GetLocalPointVelocityBounds() const{return PointVelocity;}
    double GetContractionUpperBound() const{return ContractionUpper;}
};
}
