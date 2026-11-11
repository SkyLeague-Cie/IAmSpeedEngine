#pragma once
#include "CanonicalBoxOverlapBounds.h"
#include "NativeGroupedBoxVolumeBounds.h"

namespace Speed::Analytic::Certified
{
class FCanonicalGroupedBoxOverlapBounds;
using FCanonicalGroupedBoxOverlapBoundsPtr=TSharedPtr<const FCanonicalGroupedBoxOverlapBounds,ESPMode::ThreadSafe>;
// Opaque PURE query-volume binding for the EXISTING canonical pose operation.
// This never issues a World-call, nominal facet-kernel miss, MTD, support,
// integration/response ticket or permission to change a physical pose.
class FCanonicalGroupedBoxOverlapBounds final
{
    FWorldQuery Captured;
    FGroupedBoxVolumeBounds Volume;
    FClosedGroupedBoxVolumePtr Closed;
    FAnalyticNumericEvidence Numeric;
    FCanonicalGroupedBoxOverlapBounds()=default;
    static bool SameQueryBits(const FWorldQuery& a,const FWorldQuery& b)
    {
        if(a.Shape!=b.Shape||a.TraceChannel!=b.TraceChannel||a.ObjectTypes!=b.ObjectTypes||
            a.BlockingObjectTypes!=b.BlockingObjectTypes||a.RequiredSourceId!=b.RequiredSourceId||
            a.RequiredSurfaceId!=b.RequiredSurfaceId||a.RequiredCanonicalGroupId!=b.RequiredCanonicalGroupId||
            a.bAllowEstablishedFaceContactAtBoundary!=b.bAllowEstablishedFaceContactAtBoundary||
            a.bUseFiniteContactDomain!=b.bUseFiniteContactDomain||a.bObjectQuery!=b.bObjectQuery||
            a.bApplyCollisionFilter!=b.bApplyCollisionFilter||a.bAuthorityOnly!=b.bAuthorityOnly||
            a.bExcludeAuthorityEligible!=b.bExcludeAuthorityEligible||a.bExcludeTori!=b.bExcludeTori||
            a.bIncludeCompactPatches!=b.bIncludeCompactPatches||a.bIncludeTriangles!=b.bIncludeTriangles)return false;
        const auto same=[](double x,double y){return AnalyticCertified::Bits(x)==AnalyticCertified::Bits(y);};
        if(!same(a.Radius,b.Radius)||!same(a.DomainTolerance,b.DomainTolerance)||
            !same(a.InitialOverlapTolerance,b.InitialOverlapTolerance)||!same(a.MinimumReferenceNormalDot,b.MinimumReferenceNormalDot))return false;
        for(int32 d=0;d<3;++d)
            if(!same(a.Start[d],b.Start[d])||!same(a.End[d],b.End[d])||
                !same(a.HalfExtent[d],b.HalfExtent[d])||!same(a.ReferenceNormal[d],b.ReferenceNormal[d]))return false;
        return same(a.Rotation.X,b.Rotation.X)&&same(a.Rotation.Y,b.Rotation.Y)&&
            same(a.Rotation.Z,b.Rotation.Z)&&same(a.Rotation.W,b.Rotation.W);
    }
public:
    static bool Build(const FWorldQuery& query,FDistanceBudget& budget,FCanonicalGroupedBoxOverlapBoundsPtr& output)
    {
        // Preserve aliases before resetting an old output whose captured query
        // could have been supplied as the argument. No partially built value.
        const FWorldQuery captured=query;const auto keepPrior=output;output.Reset();
        const auto numeric=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        FIntervalPoint originalEnvelope;
        // Keep ALL existing authority/filter/finite-reference/tolerance/shape/
        // rotation/extent and bit-exact stationary Start==End validations.
        if(!budget.Consume()||!BuildCanonicalFixedBoxOverlapEnvelope(captured,originalEnvelope))return false;
        FBoxVolumeGroup complete;
        if(!NativeGroupedBoxDetail::FixedQueryGroup(captured,complete)||complete.Corners.Num()!=8)return false;
        TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;groups.Add(MoveTemp(complete));
        FGroupedBoxVolumeBounds volume;FClosedGroupedBoxVolumePtr closed;
        // originalEnvelope is a broadphase envelope already inflated once.
        // Close inflates the PER-CORNER groups exactly once, not that envelope.
        if(!NativeGroupedBoxDetail::Close(originalEnvelope,groups,captured.Rotation,
            captured.DomainTolerance,captured.InitialOverlapTolerance,budget,volume,
            ClosedGroupedBoxControl::RuntimeEnabled()?&closed:nullptr)||
            volume.Groups.Num()!=1||volume.Axes.Num()!=6||volume.Slabs.Num()!=6)return false;
        const auto after=RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(after.ControlFingerprint()!=numeric.ControlFingerprint())return false;
        TSharedPtr<FCanonicalGroupedBoxOverlapBounds,ESPMode::ThreadSafe> value(new FCanonicalGroupedBoxOverlapBounds);
        value->Captured=captured;value->Volume=MoveTemp(volume);value->Closed=MoveTemp(closed);value->Numeric=after;
        output=MoveTemp(value);return true;
    }
    bool MatchesQuery(const FWorldQuery& query) const
    {
        const auto numeric=InspectCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
        return numeric.Status==EAnalyticNumericStatus::Complete&&
            numeric.ControlFingerprint()==Numeric.ControlFingerprint()&&SameQueryBits(Captured,query);
    }
    const FGroupedBoxVolumeBounds& GetVolumeForInspection() const{return Volume;}
    FClosedGroupedBoxVolumePtr GetClosedVolumeForInspection() const{return Closed;}
    const FWorldQuery& GetCapturedForInspection() const{return Captured;}
};
}
