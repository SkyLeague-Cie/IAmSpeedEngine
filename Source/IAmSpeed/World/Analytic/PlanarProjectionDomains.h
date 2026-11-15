#pragma once
#include "TensorDistanceBounds.h"

namespace Speed::Analytic::Certified::TensorPlanarFamilyControl { IAMSPEED_API bool RuntimeEnabled(); }

namespace Speed::Analytic::Certified::ContactDetail
{
// Optional necessary-domain proof ONLY. No hit, normal quantization, owner
// selection or infinite-plane substitution is provided by this structure.
enum class EPlanarProjectionProof:uint8 {NotApplicable,Proved,Arithmetic,Work};
struct FProjectedProviderUV {bool Outside=false,Bounded=false;Interval U,V;};
IAMSPEED_API EPlanarProjectionProof TensorPlanarProjectedDomain(const FIntervalTensorSurface& surface,int32 normal,
    const FIntervalPoint& foot,FDistanceBudget& budget,FProjectedProviderUV& out,bool& covers);
struct FPlanarProjectionDomains
{
    int32 NormalAxis=INDEX_NONE;FIntervalPoint Foot;
    TArray<FProjectedProviderUV> Providers;
};
inline bool ProjectionSingleton(const FIntervalPoint& p)
{
    if(!Detail::ValidPoint(p))return false;
    for(const auto& a:p.Axis)if(AnalyticCertified::Bits(a.Lower)!=AnalyticCertified::Bits(a.Upper))return false;
    return true;
}
inline Interval ProjectionOrient(const FIntervalPoint& a,const FIntervalPoint& b,const FIntervalPoint& p,int32 x,int32 y)
{
    return AnalyticCertified::Subtract(
        AnalyticCertified::Multiply(AnalyticCertified::Subtract(b.Axis[x],a.Axis[x]),AnalyticCertified::Subtract(p.Axis[y],a.Axis[y])),
        AnalyticCertified::Multiply(AnalyticCertified::Subtract(b.Axis[y],a.Axis[y]),AnalyticCertified::Subtract(p.Axis[x],a.Axis[x])));
}
inline bool ProjectionDividePositive(Interval a,Interval b,Interval& out)
{
    out={};if(!AnalyticCertified::Valid(a)||!AnalyticCertified::Valid(b)||!(b.Lower>0))return false;
    double lower=0,upper=0;bool have=false;
    for(double n:{a.Lower,a.Upper})for(double d:{b.Lower,b.Upper})
    {
        volatile double q=n/d;double lo=0,hi=0;
        if(!AnalyticCertified::Outward(q,false,lo)||!AnalyticCertified::Outward(q,true,hi))return false;
        lower=have?FMath::Min(lower,lo):lo;upper=have?FMath::Max(upper,hi):hi;have=true;
    }
    out={lower,upper,true};return AnalyticCertified::Valid(out);
}
template<class FSurfaceAt>
inline EPlanarProjectionProof ProjectedPlanarDomainsImpl(int32 count,FSurfaceAt at,
    const FIntervalPoint& center,FDistanceBudget& budget,FPlanarProjectionDomains& output,bool independentBilinearApex=false)
{
    output={};RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(count<=0||at(0).ControlPoints.IsEmpty()||!Detail::ValidPoint(center))return EPlanarProjectionProof::Arithmetic;
    int32 axis=INDEX_NONE;double height=0;
    for(int32 d=0;d<3;++d)
    {
        const auto h=at(0).ControlPoints[0].Axis[d];bool same=AnalyticCertified::Valid(h)&&AnalyticCertified::Bits(h.Lower)==AnalyticCertified::Bits(h.Upper);
        if(!same)continue;
        for(int32 surfaceIndex=0;surfaceIndex<count;++surfaceIndex)
        {
            const auto& surface=at(surfaceIndex);
            if(surface.ControlPoints.IsEmpty())return EPlanarProjectionProof::Arithmetic;
            for(const auto& p:surface.ControlPoints)
            {
                if(!budget.Consume())return EPlanarProjectionProof::Work;
                if(!Detail::ValidPoint(p))return EPlanarProjectionProof::Arithmetic;
                const auto a=p.Axis[d];
                if(AnalyticCertified::Bits(a.Lower)!=AnalyticCertified::Bits(h.Lower)||AnalyticCertified::Bits(a.Upper)!=AnalyticCertified::Bits(h.Lower)){same=false;break;}
            }
            if(!same)break;
        }
        if(same){axis=d;height=h.Lower;break;}
    }
    if(axis==INDEX_NONE)return EPlanarProjectionProof::NotApplicable;
    FPlanarProjectionDomains result;result.NormalAxis=axis;result.Foot=center;result.Foot.Axis[axis]=Interval::Exact(height);result.Providers.SetNum(count);
    const int32 x=(axis+1)%3,y=(axis+2)%3;bool witness=false;
    for(int32 index=0;index<count;++index)
    {
        if(!budget.Consume())return EPlanarProjectionProof::Work;
        const auto& surface=at(index);
        if(independentBilinearApex&&TensorPlanarFamilyControl::RuntimeEnabled()&&
            surface.DegreeU==5&&surface.DegreeV==5&&surface.ControlPoints.Num()==36)
        {
            FProjectedProviderUV domain;bool covers=false;
            const auto status=TensorPlanarProjectedDomain(surface,axis,result.Foot,budget,domain,covers);
            if(status==EPlanarProjectionProof::Work||status==EPlanarProjectionProof::Arithmetic)return status;
            if(status==EPlanarProjectionProof::Proved)
            {result.Providers[index]=domain;witness|=covers;continue;}
        }
        if(surface.DegreeU!=1||surface.DegreeV!=1||surface.ControlPoints.Num()!=4)continue;
        const auto& cp=surface.ControlPoints;
        // Equal interval boxes do NOT imply equal independent coefficients.
        // C must be the SAME exact singleton point in both repeated positions.
        FIntervalPoint projectedFoot=result.Foot;
        if(independentBilinearApex)
        {
            // For arbitrary fixed A,B,C,D, S(u,v)=TriangleABC(u,v)+u*v*(D-C).
            // Thus every preimage of Foot maps into this expanded triangle
            // foot box. This does NOT infer equality from identical CP boxes.
            for(int32 d=0;d<3;++d)
            {
                if(!budget.Consume())return EPlanarProjectionProof::Work;
                const auto shift=AnalyticCertified::Multiply(Interval{0,1,true},
                    AnalyticCertified::Subtract(cp[3].Axis[d],cp[2].Axis[d]));
                projectedFoot.Axis[d]=AnalyticCertified::Subtract(result.Foot.Axis[d],shift);
            }
            if(!Detail::ValidPoint(projectedFoot))return EPlanarProjectionProof::Arithmetic;
        }
        else
        {
            if(!ProjectionSingleton(cp[2])||!ProjectionSingleton(cp[3]))continue;
            bool repeated=true;for(int32 d=0;d<3;++d)repeated&=AnalyticCertified::Bits(cp[2].Axis[d].Lower)==AnalyticCertified::Bits(cp[3].Axis[d].Lower);
            if(!repeated)continue;
        }
        if(!budget.Consume())return EPlanarProjectionProof::Work;
        auto determinant=ProjectionOrient(cp[0],cp[1],cp[2],x,y);
        if(!AnalyticCertified::Valid(determinant))return EPlanarProjectionProof::Arithmetic;
        const int32 sign=determinant.Lower>0?1:determinant.Upper<0?-1:0;
        if(!sign)continue; // Valid but not universally regular: generic UV remains.
        Interval numerators[3];
        for(int32 k=0;k<3;++k)
        {
            if(!budget.Consume())return EPlanarProjectionProof::Work;
            numerators[k]=ProjectionOrient(cp[(k+1)%3],cp[(k+2)%3],projectedFoot,x,y);
            if(!AnalyticCertified::Valid(numerators[k]))return EPlanarProjectionProof::Arithmetic;
        }
        if(sign<0)
        {
            determinant={-determinant.Upper,-determinant.Lower,true};
            for(auto& n:numerators)n={-n.Upper,-n.Lower,true};
        }
        Interval lambda[3];
        for(int32 k=0;k<3;++k)
        {if(!budget.Consume())return EPlanarProjectionProof::Work;if(!ProjectionDividePositive(numerators[k],determinant,lambda[k]))return EPlanarProjectionProof::Arithmetic;}
        // Strict positivity over the expanded foot proves finite coverage:
        // in the homotopy TriangleABC+s*u*v*(D-C), no boundary of [0,1]^2
        // can reach Foot. Its initial degree is one, so a preimage persists.
        // The same expanded barycentrics bound ALL possible original UVs.
        witness|=lambda[0].Lower>0&&lambda[1].Lower>0&&lambda[2].Lower>0;
        auto& domain=result.Providers[index];
        if(lambda[0].Upper<0||lambda[1].Upper<0||lambda[2].Upper<0){domain.Outside=true;continue;}
        const double uLo=FMath::Max(0.,lambda[2].Lower),uHi=FMath::Min(1.,lambda[2].Upper);
        if(uLo>uHi){domain.Outside=true;continue;}
        if(!budget.Consume())return EPlanarProjectionProof::Work;
        const auto denominator=AnalyticCertified::Add(lambda[0],lambda[1]);
        if(!AnalyticCertified::Valid(denominator))return EPlanarProjectionProof::Arithmetic;
        if(!(denominator.Lower>0))continue; // Apex/uncertain inverse: KEEP provider.
        Interval v;
        if(!budget.Consume())return EPlanarProjectionProof::Work;
        if(!ProjectionDividePositive(lambda[1],denominator,v))return EPlanarProjectionProof::Arithmetic;
        const double vLo=FMath::Max(0.,v.Lower),vHi=FMath::Min(1.,v.Upper);
        if(vLo>vHi){domain.Outside=true;continue;}
        domain.U={uLo,uHi,true};domain.V={vLo,vHi,true};domain.Bounded=true;
    }
    // Common plane alone is NOT sufficient. For EVERY center and EVERY fixed
    // CP realization a true finite member must contain the projected point.
    // Then all first closest points in this PROVIDED union equal that foot.
    if(!witness)return EPlanarProjectionProof::NotApplicable;
    output=MoveTemp(result);return EPlanarProjectionProof::Proved;
}

// Original API and debit order stay intact.
inline EPlanarProjectionProof ProjectedPlanarDomains(TConstArrayView<FIntervalTensorSurface> surfaces,
    const FIntervalPoint& center,FDistanceBudget& budget,FPlanarProjectionDomains& output,bool independentBilinearApex=false)
{
    return ProjectedPlanarDomainsImpl(surfaces.Num(),
        [&](int32 i)->const FIntervalTensorSurface& {return surfaces[i];},center,budget,output,independentBilinearApex);
}
// The view refers to original immutable surfaces. No CP copy or provider reorder.
inline EPlanarProjectionProof ProjectedPlanarDomainsIndexed(TConstArrayView<FIntervalTensorSurface> surfaces,
    TConstArrayView<int32> orderedOriginalLocals,const FIntervalPoint& center,
    FDistanceBudget& budget,FPlanarProjectionDomains& output,bool independentBilinearApex=false)
{
    output={};int32 previous=INDEX_NONE;
    for(int32 local:orderedOriginalLocals)
    {
        if(!budget.Consume())return EPlanarProjectionProof::Work;
        if(local<=previous||local<0||local>=surfaces.Num())return EPlanarProjectionProof::Arithmetic;
        previous=local;
    }
    return ProjectedPlanarDomainsImpl(orderedOriginalLocals.Num(),
        [&](int32 i)->const FIntervalTensorSurface& {return surfaces[orderedOriginalLocals[i]];},
        center,budget,output,independentBilinearApex);
}
}
