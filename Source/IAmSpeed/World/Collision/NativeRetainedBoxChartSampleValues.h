#pragma once
#include "NativeRetainedBoxFloatProviderDomainValues.h"
#include "IAmSpeed/World/Analytic/ScalarSphereContactBounds.h"
#include "BoxWallResponseGeometry.h"
namespace Speed::PreparedExecution
{
class FNativeRetainedBoxChartSampleValues;
using FNativeRetainedBoxChartSampleValuesPtr=TSharedPtr<const FNativeRetainedBoxChartSampleValues,ESPMode::ThreadSafe>;
// Informative scalar sample of the proven provider/openUV, not a contact.
// Bounds cover ALL points and regular chart normals throughout that UV cell,
// for every fixed retained CP selection. No native first-contact vertex,
// oriented collision normal, scalar time, response allowance or Ready follows.
// In particular a geometric intersection at the first admitted FLOAT may
// already penetrate: its physical response normal is not selected by this type.
class FNativeRetainedBoxChartSampleValues final
{
    FNativeRetainedBoxFloatProviderDomainValuesPtr Domain;
    Analytic::FTensorBezierSurface Representative;
    FVector2d UV=FVector2d::ZeroVector;
    FVector3d Point=FVector3d::ZeroVector,Stored=FVector3d::ZeroVector,Consumed=FVector3d::ZeroVector;
    Analytic::Certified::FIntervalPoint Points,OrientedNormals,UnorientedNormals;
    double PointErrorCm=0,StoredChartErrorL1=0,ConsumedChartErrorL1=0,StoredEitherErrorL1=0,ConsumedEitherErrorL1=0;
    Analytic::FAnalyticNumericEvidence Numeric;
    FNativeRetainedBoxChartSampleValues()=default;
    static bool UnitHull(const Analytic::Certified::FIntervalPoint& raw,Analytic::Certified::FDistanceBudget& budget,
        Analytic::Certified::FIntervalPoint& out)
    {
        namespace C=Analytic::Certified;namespace A=AnalyticCertified;out={};
        if(!budget.Consume()||!C::Detail::ValidPoint(raw))return false;
        const auto squared=C::ContactDetail::SquaredNorm(raw);
        if(!A::Valid(squared)||!(squared.Lower>0))return false;
        // Sqrt is a GUESS only. Outward interval-square checks certify each
        // denominator endpoint, so no unproved libm ULP/error allowance enters.
        double lo=0,hi=0;
        if(!A::Outward(FMath::Sqrt(squared.Lower),false,lo)||!A::Outward(FMath::Sqrt(squared.Upper),true,hi)||
            !A::Finite(lo)||!A::Finite(hi)||!(lo>0)||lo>hi||
            (A::Bits(lo)&A::Magnitude)<A::MinNormal)return false;
        const auto lo2=A::Square(A::Interval::Exact(lo)),hi2=A::Square(A::Interval::Exact(hi));
        if(!A::Valid(lo2)||!A::Valid(hi2)||lo2.Upper>squared.Lower||hi2.Lower<squared.Upper)return false;
        C::FIntervalPoint result;
        for(int32 d=0;d<3;++d)
        {
            bool first=true;
            for(double numerator:{raw.Axis[d].Lower,raw.Axis[d].Upper})for(double denominator:{lo,hi})
            {
                if(!budget.Consume())return false;volatile double quotient=numerator/denominator;
                double lower=0,upper=0;
                if(!A::Outward(quotient,false,lower)||!A::Outward(quotient,true,upper))return false;
                if(first){result.Axis[d]={lower,upper,true};first=false;}
                else{result.Axis[d].Lower=FMath::Min(result.Axis[d].Lower,lower);result.Axis[d].Upper=FMath::Max(result.Axis[d].Upper,upper);}
            }
            // Intersect with the exact universal bounds of a unit-vector
            // component; this is not a numerical quality tolerance.
            result.Axis[d].Lower=FMath::Max(-1.,result.Axis[d].Lower);
            result.Axis[d].Upper=FMath::Min(1.,result.Axis[d].Upper);
        }
        if(!C::Detail::ValidPoint(result))return false;out=result;return true;
    }
public:
    static bool TryBuild(FNativeRetainedBoxFloatProviderDomainValuesPtr domain,Analytic::Certified::FDistanceBudget& budget,
        FNativeRetainedBoxChartSampleValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();namespace C=Analytic::Certified;namespace A=AnalyticCertified;
        const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!domain||!budget.Consume())return false;
        const auto corner=domain->GetCornerForInspection();if(!corner)return false;
        const auto geometry=corner->GetGeometryForInspection();const auto motion=corner->GetCornerReferenceForInspection()->GetMotionForInspection();
        if(!geometry||!motion||!domain->MatchesInputs(domain->GetBracketForInspection(),geometry,motion->GetCapturedForInspection()))return false;
        C::FIntervalTensorSurface original,restricted;
        if(!geometry->GetSelectedUnion()->CopySurfaceForQuery(domain->GetProviderIndex(),budget,original)||
            original.DegreeU<1||original.DegreeV<1||original.DegreeU>16||original.DegreeV>16||
            original.ControlPoints.Num()!=(original.DegreeU+1)*(original.DegreeV+1)||
            !C::RestrictReferenceTensorDyadic(original,corner->GetUCellForInspection(),corner->GetVCellForInspection(),budget,restricted))return false;
        TSharedPtr<FNativeRetainedBoxChartSampleValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxChartSampleValues);
        value->Domain=MoveTemp(domain);value->Representative.DegreeU=original.DegreeU;value->Representative.DegreeV=original.DegreeV;
        // Derive the representative ORIGINAL net internally. Evaluating a
        // restricted net would rescale derivatives and could change the native
        // GetSafeNormal small-vector branch, even for the same exact chart.
        for(const auto& cp:original.ControlPoints)
        {
            if(!budget.Consume()||!C::Detail::ValidPoint(cp))return false;FVector3d point;
            for(int32 d=0;d<3;++d)
            {
                const auto x=cp.Axis[d];double middle=.5*x.Lower+.5*x.Upper;
                if(!A::Finite(middle)||middle<x.Lower||middle>x.Upper)return false;
                point[d]=middle;
            }
            value->Representative.ControlPoints.Add(point);
        }
        const auto u=corner->GetUCellForInspection().GetRange(),v=corner->GetVCellForInspection().GetRange();
        if(!A::Valid(u)||!A::Valid(v))return false;
        // Dyadic cell endpoints and midpoint are exact for Depth<=52.
        value->UV=FVector2d((u.Lower+u.Upper)*.5,(v.Lower+v.Upper)*.5);
        C::Detail::FNode node;node.DegreeU=restricted.DegreeU;node.DegreeV=restricted.DegreeV;node.Net=restricted.ControlPoints;
        const int32 work=node.DegreeU*(node.DegreeV+1)+(node.DegreeU+1)*node.DegreeV;
        for(int32 k=0;k<work;++k)if(!budget.Consume())return false;
        C::FIntervalPoint du,dv,raw;
        if(!C::ContactDetail::NetHull(node,value->Points)||!C::ContactDetail::DerivativeHull(node,true,du)||
            !C::ContactDetail::DerivativeHull(node,false,dv))return false;
        const int32 sign=corner->GetCornerReferenceForInspection()->GetNormalSign();if(sign!=1&&sign!=-1)return false;
        for(int32 d=0;d<3;++d)
        {
            if(!budget.Consume())return false;
            raw.Axis[d]=A::Multiply(A::Interval::Exact(double(sign)),A::Subtract(A::Multiply(du.Axis[(d+1)%3],dv.Axis[(d+2)%3]),
                A::Multiply(du.Axis[(d+2)%3],dv.Axis[(d+1)%3])));
        }
        if(!UnitHull(raw,budget,value->OrientedNormals))return false;
        C::FIntervalPoint opposite;
        for(int32 d=0;d<3;++d)opposite.Axis[d]=A::Multiply(A::Interval::Exact(-1),value->OrientedNormals.Axis[d]);
        value->UnorientedNormals=C::TimeDetail::Hull(value->OrientedNormals,opposite);
        if(!budget.Consume())return false;
        // ACTUAL public evaluator on the original representative net/globalUV,
        // including its special bicubic DAG. Re-normalization is a separate
        // operation consumed by Box ResolveWallOrGutter, not exact unit algebra.
        value->Point=value->Representative.Evaluate(value->UV.X,value->UV.Y);
        const auto normal=value->Representative.EvaluateNormal(value->UV.X,value->UV.Y);
        value->Stored=sign==1?normal:-normal;value->Consumed=Speed::BoxWallResponseGeometry::SelectNormal(true,value->Stored);
        if(!C::ScalarContactDetail::Finite(value->Point)||!C::ScalarContactDetail::Finite(value->Stored)||
            !C::ScalarContactDetail::Finite(value->Consumed)||!C::ScalarContactDetail::Nonzero(value->Stored)||
            !C::ScalarContactDetail::Nonzero(value->Consumed)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Point,value->Points,value->PointErrorCm)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Stored,value->OrientedNormals,value->StoredChartErrorL1)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Consumed,value->OrientedNormals,value->ConsumedChartErrorL1)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Stored,value->UnorientedNormals,value->StoredEitherErrorL1)||
            !C::ScalarContactDetail::ErrorToEveryPoint(value->Consumed,value->UnorientedNormals,value->ConsumedEitherErrorL1)||
            !value->Domain->MatchesInputs(value->Domain->GetBracketForInspection(),geometry,motion->GetCapturedForInspection()))return false;
        const auto finalNumeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(finalNumeric.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        value->Numeric=finalNumeric;out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FNativeRetainedBoxFloatProviderDomainValuesPtr& domain,const FNativeSolidStaticValues& captured) const
    {
        const auto numeric=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        return numeric.Status==Analytic::EAnalyticNumericStatus::Complete&&numeric.ControlFingerprint()==Numeric.ControlFingerprint()&&
            Domain&&Domain==domain&&Domain->MatchesInputs(Domain->GetBracketForInspection(),Domain->GetCornerForInspection()->GetGeometryForInspection(),captured);
    }
    const auto& GetDomainForInspection() const{return Domain;}
    const auto& GetRepresentativeOriginalTensorForInspection() const{return Representative;}
    const auto& GetGlobalUVForInspection() const{return UV;}
    const auto& GetPointForInspection() const{return Point;}
    const auto& GetStoredNormalForInspection() const{return Stored;}
    const auto& GetConsumedNormalForInspection() const{return Consumed;}
    const auto& GetPointHullForInspection() const{return Points;}
    const auto& GetOrientedChartNormalHullForInspection() const{return OrientedNormals;}
    const auto& GetEitherChartNormalHullForInspection() const{return UnorientedNormals;}
    double GetPointErrorCm() const{return PointErrorCm;}
    double GetStoredOrientedChartErrorL1() const{return StoredChartErrorL1;}
    double GetConsumedOrientedChartErrorL1() const{return ConsumedChartErrorL1;}
    double GetStoredEitherChartErrorL1() const{return StoredEitherErrorL1;}
    double GetConsumedEitherChartErrorL1() const{return ConsumedEitherErrorL1;}
};
}
