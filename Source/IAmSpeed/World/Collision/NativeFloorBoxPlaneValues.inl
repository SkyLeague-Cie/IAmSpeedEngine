// Included once inside PreparedExecution in the existing IAS TU.
namespace NativeFloorBoxPlaneDetail
{
bool Build(const FNativeSolidStaticValues& v,const FNativeSolidGeometryValues& g,SHitResult& out)
{
    out={};using namespace Analytic::Certified;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const bool initial=g.Status==ENativeSolidGeometryStatus::BoxInitialPlaneDescriptor;
    if(v.Shape!=ENativeSolidQueryShape::Box||(!initial&&(g.Status!=ENativeSolidGeometryStatus::FirstArrivalBracket||
        g.Sweep.Status!=EFloorNativeSweepStatus::ArrivalBracket))||g.Sweep.CapturedSphereGapSatisfied||
        !g.SourceId||!g.SurfaceId||!g.FeatureId||!g.PrimitiveId||!Detail::ValidPoint(g.QueryEnvelope))return false;
    Interval a,b;FYawFloorBoxMotionModel moving;
    if(initial)
    {
        if(g.YawBoxFirstArrival||g.Sweep.Status!=EFloorNativeSweepStatus::Unknown)return false;
        if(NativeBoxNeedsGeneralFloorDescriptor(v))
        {
            const auto& general=g.GeneralBoxFloorMotionDescriptor;
            if(!general.HasMatchingPhysicalMotion()||!SameNativeSolidStaticValues(general.Captured,v)||
                general.Motion->GetStatus()!=EGeneralPhysicalFloorMotion::InitialNativeBand||
                !general.Motion->GetCommonInsideMask()||!SameFloat(general.Motion->GetInsideSeconds(),v.Horizon)||
                !SameFloat(general.Motion->GetClearSeconds(),0.f)||
                !GroupedBoxDetail::SamePoint(g.QueryEnvelope,general.QueryEnvelope)||
                !FloorNativeSweepDetail::SameVector(general.Plane.Point,FVector(v.BodyState.Location.X,v.BodyState.Location.Y,0.))||
                !FloorNativeSweepDetail::SameVector(general.Plane.Normal,FVector::UpVector)||
                general.Plane.SourceId!=g.SourceId||general.Plane.SurfaceId!=g.SurfaceId||
                general.Plane.FeatureId!=g.FeatureId||general.Plane.PrimitiveId!=g.PrimitiveId||
                general.Plane.CanonicalGroupId!=g.CanonicalGroupId||general.Plane.MaterialId!=g.MaterialId)return false;
        }
        else
        {
            FFloorNativeSolidModel fixed;
            if(!BuildFloorNativeSolidModel(v,fixed)||!fixed.IsFixedBox())
            {if(!BuildYawFloorBoxMotionModel(v,moving))return false;}
        }
    }
    else
    {
        if(g.YawBoxFirstArrival)
        {
            if(!BuildYawFloorBoxMotionModel(v,moving))return false;
            const auto n=moving.GetYawNormSquared();
            if(!AnalyticCertified::Valid(g.YawNormSquared)||!SameDouble(n.Lower,g.YawNormSquared.Lower)||
                !SameDouble(n.Upper,g.YawNormSquared.Upper))return false;
            a=moving.BoundGap(0,0,true);b=moving.BoundGap(0,0,false);
        }
        else
        {
            FFloorNativeSolidModel fixed;if(!BuildFloorNativeSolidModel(v,fixed)||!fixed.IsFixedBox())return false;
            a=fixed.BoundGap(0,0,true);b=fixed.BoundGap(0,0,false);
        }
        uint32 low=0,high=0;
        if(!AnalyticCertified::Valid(a)||!AnalyticCertified::Valid(b)||a.Lower<=0||b.Lower<=0||
            !FloorNativeSweepDetail::Ordinal(g.Sweep.ClearPrefixSeconds,low)||
            !FloorNativeSweepDetail::Ordinal(g.Sweep.InsideSeconds,high)||high<=low||g.Sweep.InsideSeconds>v.Horizon)return false;
        if(g.YawBoxFirstArrival)
        {
            const auto ln=moving.BoundGap(g.Sweep.ClearPrefixSeconds,g.Sweep.ClearPrefixSeconds,true);
            const auto lr=moving.BoundGap(g.Sweep.ClearPrefixSeconds,g.Sweep.ClearPrefixSeconds,false);
            const auto hn=moving.BoundGap(g.Sweep.InsideSeconds,g.Sweep.InsideSeconds,true);
            const auto hr=moving.BoundGap(g.Sweep.InsideSeconds,g.Sweep.InsideSeconds,false);
            if(!low||high!=low+1||!AnalyticCertified::Valid(ln)||!AnalyticCertified::Valid(lr)||
                !AnalyticCertified::Valid(hn)||!AnalyticCertified::Valid(hr)||ln.Lower<=0||lr.Lower<=0||hn.Upper>=0||hr.Upper>=0)return false;
        }
    }
    const FVector normal(0,0,v.BodyState.Location.Z<0?-1.:1.);
    const FVector point(v.BodyState.Location.X,v.BodyState.Location.Y,0.);
    for(int32 d=0;d<3;++d)if(point[d]<g.QueryEnvelope.Axis[d].Lower||point[d]>g.QueryEnvelope.Axis[d].Upper)return false;
    SHitResult descriptor;descriptor.ImpactNormal=normal;descriptor.ImpactPoint=point;
    descriptor.SourceId=g.SourceId;descriptor.SurfaceId=g.SurfaceId;descriptor.FeatureId=g.FeatureId;
    descriptor.PrimitiveId=g.PrimitiveId;descriptor.CanonicalGroupId=g.CanonicalGroupId;descriptor.MaterialId=g.MaterialId;
    descriptor.ContactFeatureOther=EContactFeatureKind::Face;out=MoveTemp(descriptor);return true;
}
bool MatchesRefinedWitness(const FNativeSolidStaticValues& v,const Analytic::Certified::FIntervalPoint& envelope,
    const SHitResult& descriptor,const SHitResult& hit)
{
    if(hit.bHit||hit.bBlockingHit||hit.bStartPenetrating||hit.PenetrationDepth!=0||
        hit.GeometricErrorBoundCm!=0||hit.bSurfaceNormalMayVary||!hit.Component.IsExplicitlyNull()||
        !hit.SubBody.IsExplicitlyNull()||hit.PreparedGroundEvidence||hit.FirstRsResponseData||
        hit.SourceId!=descriptor.SourceId||hit.SurfaceId!=descriptor.SurfaceId||hit.FeatureId!=descriptor.FeatureId||
        hit.PrimitiveId!=descriptor.PrimitiveId||hit.CanonicalGroupId!=descriptor.CanonicalGroupId||hit.MaterialId!=descriptor.MaterialId||
        !SameVector(hit.ImpactNormal,descriptor.ImpactNormal)||hit.ContactFeatureThis!=EContactFeatureKind::Vertex||
        hit.ContactFeatureOther!=EContactFeatureKind::Face||hit.ContactFeatureIndexThis<0||hit.ContactFeatureIndexThis>=8||
        hit.ContactFeatureIndexOther!=INDEX_NONE)return false;
    // Reuse the actual consumed Integrate DAG and the Box member's exact
    // origin/offset/Qlocal order. Physical extents and bitXYZ corner indices
    // are distinct from rounded query extents and GetBoxVertices ordering.
    auto pose=v.OwnerState.Integrate(hit.TOI);
    pose.Location-=pose.Rotation.RotateVector(v.LocalCOM);
    pose.Location+=pose.Rotation.RotateVector(v.LocalOffset);
    pose.Rotation=pose.Rotation*v.LocalRotation;
    const int32 index=hit.ContactFeatureIndexThis;
    const FVector local((index&1)?v.PhysicalHalfExtent.X:-v.PhysicalHalfExtent.X,
        (index&2)?v.PhysicalHalfExtent.Y:-v.PhysicalHalfExtent.Y,
        (index&4)?v.PhysicalHalfExtent.Z:-v.PhysicalHalfExtent.Z);
    const FVector vertex=pose.Location+pose.Rotation.RotateVector(local);
    const FVector witness(vertex.X,vertex.Y,0.);
    if(!SameVector(hit.Location,pose.Location)||!SameVector(hit.ContactPointThis,vertex)||
        !SameVector(hit.ImpactPoint,witness)||!SameVector(hit.ContactPointOther,witness))return false;
    const FVector points[]={FVector(pose.Location),vertex,witness};
    for(const FVector& point:points)for(int32 d=0;d<3;++d)
        if(!FMath::IsFinite(point[d])||point[d]<envelope.Axis[d].Lower||point[d]>envelope.Axis[d].Upper)return false;
    return true;
}
bool MatchesRefinement(const FNativeSolidStaticValues& v,const FNativeSolidGeometryValues& g,const SHitResult& hit)
{
    SHitResult descriptor;if(!Build(v,g,descriptor))return false;
    const bool initial=g.Status==ENativeSolidGeometryStatus::BoxInitialPlaneDescriptor;
    uint32 ordinal=0;
    if(!Analytic::Certified::FloorNativeSweepDetail::Ordinal(hit.TOI,ordinal)||
        (initial?Analytic::Certified::KinematicCenterDetail::Bits(hit.TOI)!=0:(!ordinal||
            hit.TOI<g.Sweep.ClearPrefixSeconds||hit.TOI>g.Sweep.InsideSeconds)))return false;
    if(g.YawBoxFirstArrival&&!SameFloat(hit.TOI,g.Sweep.ClearPrefixSeconds))return false;
    return MatchesRefinedWitness(v,g.QueryEnvelope,descriptor,hit);
}
bool MatchesReadbackInputsAtCallH(const FNativeSolidStaticValues& v,const FVector& normal,const FVector& point,
    float callH,const FNativeBoxPlanarRefinementValues& r)
{
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!r.Computed||r.Outcome==ENativeBoxPlanarRefinementOutcome::Incomplete||!SameFloat(r.Horizon,callH)||
        !SameKinematicBits(r.Initial,v.OwnerState)||!SameVector(r.LocalCOM,v.LocalCOM)||!SameVector(r.LocalCenter,v.LocalOffset)||
        !SameQuat(r.LocalRotation,v.LocalRotation)||!SameVector(r.Extent,v.PhysicalHalfExtent)||
        !SameVector(r.Normal,normal)||!SameVector(r.Point,point)||!FiniteState(r.StartPose)||
        !FMath::IsFinite(r.MinimumInitial)||!FMath::IsFinite(r.Roundoff)||r.Roundoff<=0||
        !FMath::IsFinite(r.SpeedRoundoff)||r.SpeedRoundoff<=0||!FMath::IsFinite(r.FirstTime))return false;
    for(int32 k=0;k<8;++k)if(!FMath::IsFinite(r.InitialGaps[k])||!FMath::IsFinite(r.InitialNormalSpeeds[k]))return false;
    // Verify the real PoseAt(0) readback, without substituting BodyState for
    // this DIFFERENT native origin/offset/Qlocal DAG or changing the readback.
    auto expected=v.OwnerState.Integrate(0.f);
    expected.Location-=expected.Rotation.RotateVector(v.LocalCOM);
    expected.Location+=expected.Rotation.RotateVector(v.LocalOffset);expected.Rotation=expected.Rotation*v.LocalRotation;
    return SameKinematicBits(expected,r.StartPose);
}
bool MatchesReadbackInputs(const FNativeSolidStaticValues& v,const FVector& normal,const FVector& point,
    const FNativeBoxPlanarRefinementValues& r)
{return MatchesReadbackInputsAtCallH(v,normal,point,v.Horizon,r);}
bool MatchesInitialReadback(const FNativeSolidStaticValues& v,const SHitResult& hit,const FNativeBoxPlanarRefinementValues& r)
{
    // Existing native roundoff is a copied pose/decision check, NOT a Box
    // margin or a declaration that measured native/reference gaps equal zero.
    return r.Computed&&r.Outcome==ENativeBoxPlanarRefinementOutcome::Hit&&r.InitialEvent&&!r.Established&&!r.EstablishedMask&&
        r.FirstCorner==hit.ContactFeatureIndexThis&&r.FirstCorner>=0&&r.FirstCorner<8&&
        Analytic::Certified::KinematicCenterDetail::Bits(r.FirstTime)==0&&Analytic::Certified::KinematicCenterDetail::Bits(hit.TOI)==0&&
        SameFloat(r.Horizon,v.Horizon)&&FMath::IsFinite(r.MinimumInitial)&&FMath::IsFinite(r.Roundoff)&&r.Roundoff>0&&
        FMath::Abs(r.MinimumInitial)<=r.Roundoff;
}
bool BuildSupportDescriptor(const FNativeSolidStaticValues& v,const FNativeSolidGeometryValues& g,SHitResult& out)
{
    out={};const auto& b=g.BoxSupport;
    if(v.Shape!=ENativeSolidQueryShape::Box||g.Status!=ENativeSolidGeometryStatus::BoxFloorSupportDescriptor||!b.Complete||
        !Analytic::Certified::Detail::ValidPoint(g.QueryEnvelope)||!g.SourceId||!g.SurfaceId||!g.PrimitiveId)return false;
    SHitResult result;result.ImpactNormal=b.CachedPlane.Normal;result.ImpactPoint=b.CachedPlane.Point;
    result.SourceId=g.SourceId;result.SurfaceId=g.SurfaceId;result.FeatureId=g.FeatureId;result.PrimitiveId=g.PrimitiveId;
    result.CanonicalGroupId=g.CanonicalGroupId;result.MaterialId=g.MaterialId;result.ContactFeatureOther=EContactFeatureKind::Face;
    out=MoveTemp(result);return true;
}
bool MatchesNoNewReadback(const FNativeSolidStaticValues& v,const FNativeSolidGeometryValues& g,const FNativeBoxPlanarRefinementValues& r)
{
    SHitResult descriptor;
    if(!BuildSupportDescriptor(v,g,descriptor)||!MatchesReadbackInputs(v,descriptor.ImpactNormal,descriptor.ImpactPoint,r)||
        r.Outcome!=ENativeBoxPlanarRefinementOutcome::NoCorner||!r.Established||r.InitialEvent||!r.EstablishedMask||
        r.FirstCorner!=INDEX_NONE||!SameFloat(r.FirstTime,v.Horizon)||FMath::Abs(r.MinimumInitial)>r.Roundoff)return false;
    // Preserve all masked gap intervals. Only the unmasked corners acquire a
    // strict whole-H no-arrival statement; physical support/forces do not.
    return Analytic::Certified::UnmaskedBoxCornersStayClear(g.BoxSupport,r.EstablishedMask);
}
}

namespace NativeGeneralFloorMotionDetail
{
bool Within(const C::FIntervalPoint& envelope,const FVector& point)
{
    if(!C::Detail::ValidPoint(envelope)||point.ContainsNaN())return false;
    for(int32 d=0;d<3;++d)if(!FMath::IsFinite(point[d])||point[d]<envelope.Axis[d].Lower||point[d]>envelope.Axis[d].Upper)return false;
    return true;
}
IAMSPEED_API bool Match(const FNativeSolidStaticValues& v,const C::FNativeGeneralBoxFloorMotionDescriptorBounds& g,
    const SHitResult& input,const SHitResult& hit,bool returned,const FNativeBoxPlanarRefinementValues& r,float callH)
{
    if(!g.HasMatchingPhysicalMotion()||!SameNativeSolidStaticValues(g.Captured,v)||
        !NativeFloorBoxPlaneDetail::MatchesReadbackInputsAtCallH(v,input.ImpactNormal,input.ImpactPoint,callH,r)||
        r.Established||r.InitialEvent||r.EstablishedMask||r.MinimumInitial<=r.Roundoff)return false;
    const auto status=g.Motion->GetStatus();
    if(status==C::EGeneralPhysicalFloorMotion::WholePhysicalClear)
        return !returned&&!r.WitnessComputed&&r.Outcome==ENativeBoxPlanarRefinementOutcome::NoCorner&&
            r.FirstCorner==INDEX_NONE&&SameFloat(callH,v.Horizon)&&SameFloat(r.FirstTime,v.Horizon)&&
            SameNativeAnalyticContactValues(input,hit);
    if(status!=C::EGeneralPhysicalFloorMotion::FirstAdjacentArrival||!returned||!r.WitnessComputed||
        r.Outcome!=ENativeBoxPlanarRefinementOutcome::Hit||r.FirstCorner<0||r.FirstCorner>=8||
        !(g.Motion->GetCommonInsideMask()&(1u<<r.FirstCorner))||!SameFloat(callH,g.Motion->GetInsideSeconds())||
        !SameFloat(r.FirstTime,g.Motion->GetClearSeconds())||!SameFloat(hit.TOI,g.Motion->GetClearSeconds())||
        hit.bHit||hit.bBlockingHit||hit.bStartPenetrating||hit.bSurfaceNormalMayVary||
        !SameFloat(hit.PenetrationDepth,0.f)||!SameFloat(hit.GeometricErrorBoundCm,0.f)||
        !hit.Component.IsExplicitlyNull()||!hit.SubBody.IsExplicitlyNull()||hit.PreparedGroundEvidence||hit.FirstRsResponseData||
        hit.SourceId!=input.SourceId||hit.SurfaceId!=input.SurfaceId||hit.FeatureId!=input.FeatureId||
        hit.PrimitiveId!=input.PrimitiveId||hit.CanonicalGroupId!=input.CanonicalGroupId||hit.MaterialId!=input.MaterialId||
        !SameVector(hit.ImpactNormal,input.ImpactNormal)||hit.ContactFeatureThis!=EContactFeatureKind::Vertex||
        hit.ContactFeatureIndexThis!=r.FirstCorner||hit.ContactFeatureOther!=EContactFeatureKind::Face||hit.ContactFeatureIndexOther!=INDEX_NONE)return false;
    auto pose=v.OwnerState.Integrate(hit.TOI);pose.Location-=pose.Rotation.RotateVector(v.LocalCOM);
    pose.Location+=pose.Rotation.RotateVector(v.LocalOffset);pose.Rotation=pose.Rotation*v.LocalRotation;
    FVector local;for(int32 d=0;d<3;++d)local[d]=(r.FirstCorner&(1<<d))?v.PhysicalHalfExtent[d]:-v.PhysicalHalfExtent[d];
    const FVector vertex=pose.Location+pose.Rotation.RotateVector(local);
    const FVector projected=vertex-FVector::DotProduct(vertex-input.ImpactPoint,input.ImpactNormal)*input.ImpactNormal;
    const auto& q=r.WitnessQuery;const auto& w=r.Witness;
    if(!SameVector(hit.Location,pose.Location)||!SameVector(hit.ContactPointThis,vertex)||
        !SameVector(hit.ImpactPoint,w.Point)||!SameVector(hit.ContactPointOther,w.Point)||
        q.Shape!=Analytic::EQueryShape::Ray||!SameVector(q.Start,projected+input.ImpactNormal*.01)||
        !SameVector(q.End,projected-input.ImpactNormal*.01)||q.RequiredSourceId!=input.SourceId||q.RequiredSurfaceId!=input.SurfaceId||
        q.RequiredCanonicalGroupId||!q.bAuthorityOnly||!q.bIncludeCompactPatches||!q.bApplyCollisionFilter||
        q.bObjectQuery||q.bExcludeAuthorityEligible||q.TraceChannel!=v.Query.TraceChannel||q.BlockingObjectTypes!=v.Query.BlockingObjectTypes||
        q.DomainTolerance!=0||q.InitialOverlapTolerance!=0||!w.bHit||w.bStartPenetrating||w.bSurfaceNormalMayVary||
        w.GeometricErrorBoundCm!=0||w.PenetrationDepth!=0||!SameVector(w.Normal,input.ImpactNormal)||
        w.SurfaceFeatureKind!=EContactFeatureKind::Face||w.SourceId!=input.SourceId||w.SurfaceId!=input.SurfaceId||
        w.FeatureId!=input.FeatureId||w.PrimitiveId!=input.PrimitiveId||w.CanonicalGroupId!=input.CanonicalGroupId||w.MaterialId!=input.MaterialId)return false;
    // The service authenticated this WHOLE raw envelope in BOTH finite
    // domains of the complete (possibly concave) FLOOR atlas. Keep the actual
    // witness and ray inside it; never substitute a triangle/convexity theorem.
    return Within(g.QueryEnvelope,vertex)&&Within(g.QueryEnvelope,w.Point)&&Within(g.QueryEnvelope,w.QueryPoint)&&
        Within(g.QueryEnvelope,q.Start)&&Within(g.QueryEnvelope,q.End)&&
        g.NativePlane.ContainsProjectedPoint(w.Point,0)&&g.NativePlane.ContainsProjectedPoint(w.QueryPoint,0);
}
}

namespace NativeGeneralCachedFloorArrivalDetail
{
IAMSPEED_API bool Match(const FNativeSolidStaticValues& v,const C::FNativeGeneralCachedFloorArrivalDescriptorBounds& g,
    const SHitResult& input,const SHitResult& hit,bool returned,const FNativeBoxPlanarRefinementValues& r,float callH)
{
    const auto numeric=Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(v.Shape!=ENativeSolidQueryShape::Box||!g.HasMatchingPhysicalPrefix()||!SameNativeSolidStaticValues(g.Captured,v)||
        !SameFloat(callH,v.Horizon)||
        !NativeFloorBoxPlaneDetail::MatchesReadbackInputsAtCallH(v,input.ImpactNormal,input.ImpactPoint,callH,r)||
        !returned||!r.Established||r.InitialEvent||!r.WitnessComputed||
        r.Outcome!=ENativeBoxPlanarRefinementOutcome::Hit||r.FirstCorner<0||r.FirstCorner>=8||
        (r.EstablishedMask&~0xffu)||(r.EstablishedMask&(1u<<r.FirstCorner))||
        !FMath::IsFinite(hit.TOI)||!(hit.TOI>0.f)||hit.TOI>v.Horizon||!SameFloat(r.FirstTime,hit.TOI)||
        !g.CachedPrefix->Matches(g.GetMotionInputForInspection(),g.Plane.Point,g.Plane.Normal,hit.TOI))return false;
    SHitResult expectedInput;expectedInput.ImpactPoint=g.Plane.Point;expectedInput.ImpactNormal=g.Plane.Normal;
    expectedInput.SourceId=g.Plane.SourceId;expectedInput.SurfaceId=g.Plane.SurfaceId;expectedInput.FeatureId=g.Plane.FeatureId;
    expectedInput.PrimitiveId=g.Plane.PrimitiveId;expectedInput.CanonicalGroupId=g.Plane.CanonicalGroupId;expectedInput.MaterialId=g.Plane.MaterialId;
    expectedInput.ContactFeatureOther=EContactFeatureKind::Face;
    if(!SameNativeAnalyticContactValues(input,expectedInput)||hit.bHit||hit.bBlockingHit||hit.bStartPenetrating||hit.bSurfaceNormalMayVary||
        !SameFloat(hit.PenetrationDepth,0.f)||!SameFloat(hit.GeometricErrorBoundCm,0.f)||
        !hit.Component.IsExplicitlyNull()||!hit.SubBody.IsExplicitlyNull()||hit.PreparedGroundEvidence||hit.FirstRsResponseData||
        hit.SourceId!=input.SourceId||hit.SurfaceId!=input.SurfaceId||hit.FeatureId!=input.FeatureId||hit.PrimitiveId!=input.PrimitiveId||
        hit.CanonicalGroupId!=input.CanonicalGroupId||hit.MaterialId!=input.MaterialId||!SameVector(hit.ImpactNormal,input.ImpactNormal)||
        hit.ContactFeatureThis!=EContactFeatureKind::Vertex||hit.ContactFeatureIndexThis!=r.FirstCorner||
        hit.ContactFeatureOther!=EContactFeatureKind::Face||hit.ContactFeatureIndexOther!=INDEX_NONE)return false;
    // The native TOI is retained exactly, with the same pure validation DAG
    // as FirstFloor60. Neither the original H nor the owner's state is retimed.
    auto pose=v.OwnerState.Integrate(hit.TOI);pose.Location-=pose.Rotation.RotateVector(v.LocalCOM);
    pose.Location+=pose.Rotation.RotateVector(v.LocalOffset);pose.Rotation=pose.Rotation*v.LocalRotation;
    FVector local;for(int32 d=0;d<3;++d)local[d]=(r.FirstCorner&(1<<d))?v.PhysicalHalfExtent[d]:-v.PhysicalHalfExtent[d];
    const FVector vertex=pose.Location+pose.Rotation.RotateVector(local);
    const FVector projected=vertex-FVector::DotProduct(vertex-input.ImpactPoint,input.ImpactNormal)*input.ImpactNormal;
    const auto& q=r.WitnessQuery;const auto& w=r.Witness;
    if(!SameVector(hit.Location,pose.Location)||!SameVector(hit.ContactPointThis,vertex)||
        !SameVector(hit.ImpactPoint,w.Point)||!SameVector(hit.ContactPointOther,w.Point)||
        q.Shape!=Analytic::EQueryShape::Ray||!SameVector(q.Start,projected+input.ImpactNormal*.01)||
        !SameVector(q.End,projected-input.ImpactNormal*.01)||q.RequiredSourceId!=input.SourceId||q.RequiredSurfaceId!=input.SurfaceId||
        q.RequiredCanonicalGroupId||!q.bAuthorityOnly||!q.bIncludeCompactPatches||!q.bApplyCollisionFilter||q.bObjectQuery||
        q.bExcludeAuthorityEligible||q.TraceChannel!=v.Query.TraceChannel||q.BlockingObjectTypes!=v.Query.BlockingObjectTypes||
        q.DomainTolerance!=0||q.InitialOverlapTolerance!=0||!w.bHit||w.bStartPenetrating||w.bSurfaceNormalMayVary||
        w.GeometricErrorBoundCm!=0||w.PenetrationDepth!=0||!SameVector(w.Normal,input.ImpactNormal)||
        w.SurfaceFeatureKind!=EContactFeatureKind::Face||w.SourceId!=input.SourceId||w.SurfaceId!=input.SurfaceId||
        w.FeatureId!=input.FeatureId||w.PrimitiveId!=input.PrimitiveId||w.CanonicalGroupId!=input.CanonicalGroupId||w.MaterialId!=input.MaterialId)return false;
    auto expectedHit=input;expectedHit.TOI=r.FirstTime;expectedHit.Location=pose.Location;expectedHit.ImpactPoint=w.Point;
    expectedHit.ContactPointThis=vertex;expectedHit.ContactPointOther=w.Point;
    expectedHit.ContactFeatureThis=EContactFeatureKind::Vertex;expectedHit.ContactFeatureIndexThis=static_cast<int8>(r.FirstCorner);
    if(!SameNativeAnalyticContactValues(expectedHit,hit))return false;
    using NativeGeneralFloorMotionDetail::Within;
    return Within(g.QueryEnvelope,vertex)&&Within(g.QueryEnvelope,w.Point)&&Within(g.QueryEnvelope,w.QueryPoint)&&
        Within(g.QueryEnvelope,q.Start)&&Within(g.QueryEnvelope,q.End)&&g.NativePlane.ContainsProjectedPoint(w.Point,0)&&
        g.NativePlane.ContainsProjectedPoint(w.QueryPoint,0)&&
        Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush).ControlFingerprint()==numeric.ControlFingerprint();
}
}
