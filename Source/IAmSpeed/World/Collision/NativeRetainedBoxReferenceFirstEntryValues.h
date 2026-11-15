#pragma once
#include "NativeRetainedBoxRootDomainClearanceValues.h"
namespace Speed::PreparedExecution
{
class FNativeRetainedBoxReferenceFirstEntryValues;
using FNativeRetainedBoxReferenceFirstEntryValuesPtr=TSharedPtr<const FNativeRetainedBoxReferenceFirstEntryValues,ESPMode::ThreadSafe>;
// First intersection of the WHOLE fixed REFERENCE Box with the ENTIRE retained
// union. Conditional mathematical values only: not consumed-float Integrate,
// query-SAT/native TOI, point/unitnormal error, contact/response/Frame or Ready.
class FNativeRetainedBoxReferenceFirstEntryValues final
{
    FNativeRetainedBoxPreCellClearanceValuesPtr Prefix;
    FNativeRetainedBoxRootDomainClearanceValuesPtr Domain;
    AnalyticCertified::Interval DimensionDeterminant;
    FNativeRetainedBoxReferenceFirstEntryValues()=default;
public:
    // Pure generic reference-cloud helper. Its result cannot construct this
    // first-entry type: TryProve always derives E from its SAME owned Motion.
    static bool BoundReferenceCloudFullDimension(const Analytic::Certified::FIntervalPoint (&offsets)[8],
        Analytic::Certified::FDistanceBudget& budget,AnalyticCertified::Interval& out)
    {
        // Copy before resetting out, which may alias offsets[k].Axis[d].
        Analytic::Certified::FIntervalPoint captured[8];for(int32 k=0;k<8;++k)captured[k]=offsets[k];out={};
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        for(const auto& p:captured)if(!budget.Consume()||!Analytic::Certified::Detail::ValidPoint(p))return false;
        const auto a=Analytic::Certified::ContactDetail::Subtract(captured[1],captured[0]);
        const auto b=Analytic::Certified::ContactDetail::Subtract(captured[2],captured[0]);
        const auto c=Analytic::Certified::ContactDetail::Subtract(captured[4],captured[0]);
        Analytic::Certified::FIntervalPoint cross;
        for(int32 d=0;d<3;++d)
        {
            if(!budget.Consume())return false;
            cross.Axis[d]=AnalyticCertified::Subtract(
                AnalyticCertified::Multiply(b.Axis[(d+1)%3],c.Axis[(d+2)%3]),
                AnalyticCertified::Multiply(b.Axis[(d+2)%3],c.Axis[(d+1)%3]));
        }
        if(!budget.Consume())return false;
        const auto determinant=Analytic::Certified::ContactDetail::Dot(a,cross);
        if(!AnalyticCertified::Valid(determinant)||!(determinant.Lower>0||determinant.Upper<0))return false;
        // An outward interval determinant, not a nominal preconditioner guess.
        // Four affine-independent actual offsets make their convex hull full.
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=determinant;return true;
    }
    static bool TryProve(FNativeRetainedBoxPreCellClearanceValuesPtr prefix,
        FNativeRetainedBoxRootDomainClearanceValuesPtr domain,Analytic::Certified::FDistanceBudget& budget,
        FNativeRetainedBoxReferenceFirstEntryValuesPtr& out)
    {
        const auto keepPrior=out;out.Reset();
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(!prefix||!domain||!budget.Consume())return false;
        const auto corner=prefix->GetCornerForInspection();
        if(!corner||domain->GetCornerForInspection()!=corner)return false;
        const auto root=corner->GetCornerReferenceForInspection();const auto motion=root->GetMotionForInspection();
        if(!prefix->MatchesInputs(corner,motion->GetCapturedForInspection())||
            !domain->MatchesInputs(corner,motion->GetCapturedForInspection()))return false;
        AnalyticCertified::Interval dimension;
        if(!BoundReferenceCloudFullDimension(motion->GetPhysicalOffsetsForInspection(),budget,dimension))return false;
        const auto time=root->GetOriginalNormalizedTimeBounds(),seconds=root->GetOriginalReferenceSecondsBounds();
        if(!AnalyticCertified::Valid(time)||!AnalyticCertified::Valid(seconds))return false;
        // SAME realization throughout: the actual fixed Box is the full
        // convex hull of these eight constant offsets plus ONE common
        // polynomial translation. All A-E enclosures contain this realization.
        // A: Prefix excludes the complete Box/union through cell.Lo.
        // B/C: Domain excludes all other providers, UV complement and closed
        // boundaries for the entire time cell. First touch is interior here.
        // D: the sealed corner root owns uniform whole-patch derivatives,
        // strict selected-vertex support and N.translation'<0 on the WHOLE
        // cell; not merely on K. Hence the patch is regular everywhere.
        // E: this interval determinant proves the SAME convex Box is full.
        // At a first interior contact, the tangent plane supports the convex
        // Box, else nearby surface points would already be inside it. The
        // opposite orientation implies an earlier intersection by strict
        // approach. The remaining support is the unique selected vertex.
        // Thus first touch is S-Corner=0, identified by its unique sealed root.
        // No global surface injectivity, orthonormal Q, float cast or new
        // native error allowance is inferred from this reference argument.
        TSharedPtr<FNativeRetainedBoxReferenceFirstEntryValues,ESPMode::ThreadSafe> value(new FNativeRetainedBoxReferenceFirstEntryValues);
        value->Prefix=MoveTemp(prefix);value->Domain=MoveTemp(domain);value->DimensionDeterminant=dimension;
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        out=MoveTemp(value);return true;
    }
    bool MatchesInputs(const FPreparedGeometrySelectionPtr& geometry,const FNativeSolidStaticValues& captured) const
    {
        if(!Prefix||!Domain)return false;const auto corner=Prefix->GetCornerForInspection();
        return corner&&corner->GetGeometryForInspection()==geometry&&Domain->GetCornerForInspection()==corner&&
            Prefix->MatchesInputs(corner,captured)&&Domain->MatchesInputs(corner,captured);
    }
    const auto& GetPrefixForInspection() const{return Prefix;}
    const auto& GetDomainForInspection() const{return Domain;}
    const auto& GetDimensionDeterminantForInspection() const{return DimensionDeterminant;}
    AnalyticCertified::Interval GetFirstReferenceNormalizedTimeBounds() const
    {return Prefix->GetCornerForInspection()->GetCornerReferenceForInspection()->GetOriginalNormalizedTimeBounds();}
    AnalyticCertified::Interval GetFirstReferenceSecondsBounds() const
    {return Prefix->GetCornerForInspection()->GetCornerReferenceForInspection()->GetOriginalReferenceSecondsBounds();}
};
}
