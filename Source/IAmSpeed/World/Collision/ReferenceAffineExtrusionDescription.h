#pragma once
#include "IAmSpeed/World/Analytic/ReferenceAffineExtrusionMath.h"

namespace Speed::Analytic::Prepared
{
// A producer's exact FIXED SOURCE conclusion. Generic structural validation
// does not authenticate it or promote any owner to physical authority. The
// closed game must derive it from its authenticated master and bind it into
// fresh transport/model description equality before a native consumer uses it.
struct FReferenceAffineExtrusionDescription
{
    int32 Provider=INDEX_NONE,SourceFace=INDEX_NONE,OriginalDegreeU=0,OriginalDegreeV=0;
    FString SourceFaceIdentity,CanonicalRecordIdentity,BoundRecordIdentity,CorrelationProofIdentity;
    FString ExactProjectedCurveIdentity; // Same fixed rational projected set, up to v reversal.
    TArray<FString> OriginalDomains;
    Certified::AffineReferenceDetail::FProduct Product;

    static bool ChargeText(Certified::FDistanceBudget& budget,const FString& text)
    {
        const int64 blocks=(int64(text.Len())*sizeof(TCHAR)+63)/64;
        if(blocks>budget.Remaining||uint64(blocks)>MAX_uint64-budget.Used)return false;
        for(int64 i=0;i<blocks;++i)if(!budget.Consume())return false;
        return true;
    }
    bool ChargeIdentity(Certified::FDistanceBudget& budget) const
    {
        if(!ChargeText(budget,SourceFaceIdentity)||!ChargeText(budget,CanonicalRecordIdentity)||
            !ChargeText(budget,BoundRecordIdentity)||!ChargeText(budget,CorrelationProofIdentity)||
            !ChargeText(budget,ExactProjectedCurveIdentity))return false;
        for(const auto& domain:OriginalDomains)if(!ChargeText(budget,domain))return false;
        return true;
    }

    static bool SameInterval(const AnalyticCertified::Interval& a,const AnalyticCertified::Interval& b)
    {return a.Known==b.Known&&AnalyticCertified::Bits(a.Lower)==AnalyticCertified::Bits(b.Lower)&&
        AnalyticCertified::Bits(a.Upper)==AnalyticCertified::Bits(b.Upper);}
    static bool SamePoint(const Certified::FIntervalPoint& a,const Certified::FIntervalPoint& b)
    {for(int32 d=0;d<3;++d)if(!SameInterval(a.Axis[d],b.Axis[d]))return false;return true;}

    bool SameDescription(const FReferenceAffineExtrusionDescription& b,Certified::FDistanceBudget& budget) const
    {
        if(!budget.Consume()||!ChargeIdentity(budget)||!b.ChargeIdentity(budget)||
            Provider!=b.Provider||SourceFace!=b.SourceFace||OriginalDegreeU!=b.OriginalDegreeU||
            OriginalDegreeV!=b.OriginalDegreeV||SourceFaceIdentity!=b.SourceFaceIdentity||
            CanonicalRecordIdentity!=b.CanonicalRecordIdentity||BoundRecordIdentity!=b.BoundRecordIdentity||
            CorrelationProofIdentity!=b.CorrelationProofIdentity||ExactProjectedCurveIdentity!=b.ExactProjectedCurveIdentity||
            OriginalDomains!=b.OriginalDomains||
            Product.FreeWorldAxis!=b.Product.FreeWorldAxis||Product.FreeAlongU!=b.Product.FreeAlongU||
            !SameInterval(Product.Start,b.Product.Start)||!SameInterval(Product.End,b.Product.End)||
            !SamePoint(Product.ExactCurveDerivativeHull,b.Product.ExactCurveDerivativeHull)||
            Product.Curve.DegreeU!=b.Product.Curve.DegreeU||Product.Curve.DegreeV!=b.Product.Curve.DegreeV||
            Product.Curve.ControlPoints.Num()!=b.Product.Curve.ControlPoints.Num())return false;
        for(int32 i=0;i<Product.Curve.ControlPoints.Num();++i)
            if(!budget.Consume()||!SamePoint(Product.Curve.ControlPoints[i],b.Product.Curve.ControlPoints[i]))return false;
        return true;
    }

    // Necessary binding checks only. They intentionally do NOT infer exact
    // equalities between independent CP intervals. The authenticated producer
    // supplies the additional rational-source factorization and derivative fact.
    bool StructurallyMatches(const Certified::FIntervalTensorSurface& original,Certified::FDistanceBudget& budget) const
    {
        RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        namespace A=::AnalyticCertified;
        const int32 axis=Product.FreeWorldAxis;
        if(!budget.Consume()||!ChargeIdentity(budget)||Provider<0||SourceFace<0||SourceFaceIdentity.IsEmpty()||
            CanonicalRecordIdentity.IsEmpty()||BoundRecordIdentity.IsEmpty()||CorrelationProofIdentity.IsEmpty()||
            OriginalDomains.Num()!=4||axis<0||axis>=3||OriginalDegreeU!=original.DegreeU||OriginalDegreeV!=original.DegreeV||
            original.DegreeU<0||original.DegreeU>16||original.DegreeV<0||original.DegreeV>16||
            original.ControlPoints.Num()!=(original.DegreeU+1)*(original.DegreeV+1)||
            !A::Valid(Product.Start)||!A::Valid(Product.End)||
            !(Product.End.Lower>Product.Start.Upper||Product.Start.Lower>Product.End.Upper)||
            !Certified::Detail::ValidPoint(Product.ExactCurveDerivativeHull))return false;
        for(const auto& domain:OriginalDomains)if(domain.IsEmpty())return false;
        if(!ExactProjectedCurveIdentity.IsEmpty())
        {
            if(ExactProjectedCurveIdentity.Len()!=64)return false;
            for(TCHAR c:ExactProjectedCurveIdentity)if(!((c>=TEXT('0')&&c<=TEXT('9'))||(c>=TEXT('a')&&c<=TEXT('f'))))return false;
        }
        const int32 freeDegree=Product.FreeAlongU?original.DegreeU:original.DegreeV;
        const int32 curveDegree=Product.FreeAlongU?original.DegreeV:original.DegreeU;
        if(freeDegree<1||curveDegree<1||Product.Curve.DegreeU!=0||Product.Curve.DegreeV!=curveDegree||
            Product.Curve.ControlPoints.Num()!=curveDegree+1||
            A::Bits(Product.ExactCurveDerivativeHull.Axis[axis].Lower)!=0||
            A::Bits(Product.ExactCurveDerivativeHull.Axis[axis].Upper)!=0)return false;
        bool regular=false;
        for(int32 d=0;d<3;++d)if(d!=axis)
            regular|=Product.ExactCurveDerivativeHull.Axis[d].Lower>0||Product.ExactCurveDerivativeHull.Axis[d].Upper<0;
        if(!regular)return false;
        const auto at=[&](int32 i,int32 j)->const Certified::FIntervalPoint&
        {return original.ControlPoints[Product.FreeAlongU?i*(original.DegreeV+1)+j:j*(original.DegreeV+1)+i];};
        if(!SameInterval(Product.Start,at(0,0).Axis[axis])||!SameInterval(Product.End,at(freeDegree,0).Axis[axis]))return false;
        for(int32 j=0;j<=curveDegree;++j)
        {
            const auto& curve=Product.Curve.ControlPoints[j];
            if(!budget.Consume()||!Certified::Detail::ValidPoint(curve)||
                A::Bits(curve.Axis[axis].Lower)!=0||A::Bits(curve.Axis[axis].Upper)!=0)return false;
            for(int32 d=0;d<3;++d)if(d!=axis&&!SameInterval(curve.Axis[d],at(0,j).Axis[d]))return false;
            for(int32 i=0;i<=freeDegree;++i)
            {
                if(!budget.Consume()||!Certified::Detail::ValidPoint(at(i,j)))return false;
                if(!SameInterval(at(i,j).Axis[axis],at(i,0).Axis[axis]))return false;
                for(int32 d=0;d<3;++d)if(d!=axis&&!SameInterval(at(i,j).Axis[d],at(0,j).Axis[d]))return false;
            }
        }
        return true;
    }
};
}
