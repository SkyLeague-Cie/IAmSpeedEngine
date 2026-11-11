// Included inside Speed::PreparedExecution in the existing IAS TU.
namespace NativeFloorSphereContactDetail
{
// Pure payload geometry. Raw values used by this helper carry no World,
// response, MTD, event or advance permission; the public entry below requires
// the sealed complete static-geometry result and its exact live query serial.
bool Build(const FNativeSolidStaticValues& v,const FNativeSolidGeometryValues& g,
    FNativeFloorSphereContactGeometryValues& output)
{
    output={};using namespace Analytic;using namespace Analytic::Certified;
    RequireCurrentAnalyticNumericContext(EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(v.Shape!=ENativeSolidQueryShape::Sphere||!g.SourceId||!g.SurfaceId||!g.FeatureId||!g.PrimitiveId||
        !Analytic::Certified::Detail::ValidPoint(g.QueryEnvelope))return false;
    const auto initial=ClassifyCapturedFloorSphereGeometry(v);
    const bool ceiling=g.NativeCeilingSphereArrivalZ>0;
    if(!AnalyticCertified::Finite(g.NativeCeilingSphereArrivalZ)||g.NativeCeilingSphereArrivalZ<0||
        (ceiling&&g.Status!=ENativeSolidGeometryStatus::FirstArrivalBracket)||
        (!ceiling&&initial.SupportSide==EInitialFloorSphereSide::Unknown))return false;
    float seconds=0;FVector center=initial.CapturedCenter;double error=0,depth=0;bool overlap=false;
    if(g.Status==ENativeSolidGeometryStatus::CapturedSphereTouch)
    {
        if(initial.SupportSide!=EInitialFloorSphereSide::ExactTouch)return false;
        // Captured +0 event, not SBox::AdvancePosition(...,0) / Integrate(0).
    }
    else if(g.Status==ENativeSolidGeometryStatus::CapturedSphereOverlap)
    {
        // Exact Rs geometry at the captured pose. Reuse the existing Sterbenz
        // range/FTZ guards; an interval gap endpoint is NOT an exact depth.
        // Deep/ambiguous overlaps remain Unknown, with no invented normal or
        // correction and no Integrate(0). Rp and rounded sweep Rc are separate.
        const auto height=AnalyticCertified::FromBits(AnalyticCertified::Bits(center.Z)&AnalyticCertified::Magnitude);
        if(initial.SupportSide!=EInitialFloorSphereSide::Overlap||
            !Analytic::Prepared::OriginPlaneDetail::ShallowDepth(height,v.SupportRadius,depth)||!(depth>0))return false;
        overlap=true;
    }
    else if(g.Status==ENativeSolidGeometryStatus::FirstArrivalBracket)
    {
        seconds=g.Sweep.ClearPrefixSeconds;uint32 ordinal=0,inside=0;
        if((!ceiling&&initial.SupportSide!=EInitialFloorSphereSide::Separated)||
            g.Sweep.Status!=EFloorNativeSweepStatus::ArrivalBracket||!g.Sweep.CapturedSphereGapSatisfied||
            !FloorNativeSweepDetail::Ordinal(seconds,ordinal)||!ordinal||
            !FloorNativeSweepDetail::Ordinal(g.Sweep.InsideSeconds,inside)||inside<=ordinal||g.Sweep.InsideSeconds>v.Horizon)return false;
        FFloorNativeSolidModel model;
        if(ceiling?!BuildNativeCeilingSphereModel(v,g.NativeCeilingSphereArrivalZ,model):
            !BuildFloorNativeSolidModel(v,model))return false;
        if(ceiling)
        {
            const auto startNative=model.BoundGap(0,0,true),startReference=model.BoundGap(0,0,false);
            const auto insideNative=model.BoundGap(g.Sweep.InsideSeconds,g.Sweep.InsideSeconds,true);
            const auto insideReference=model.BoundGap(g.Sweep.InsideSeconds,g.Sweep.InsideSeconds,false);
            if(!AnalyticCertified::Valid(startNative)||startNative.Lower<=0||
                !AnalyticCertified::Valid(startReference)||startReference.Lower<=0||
                !AnalyticCertified::Valid(insideNative)||insideNative.Upper>=0||
                !AnalyticCertified::Valid(insideReference)||insideReference.Upper>=0)return false;
        }
        const auto native=model.BoundGap(seconds,seconds,true),reference=model.BoundGap(seconds,seconds,false);
        if(!AnalyticCertified::Valid(native)||!AnalyticCertified::Valid(reference)||native.Lower<=0||reference.Lower<=0||
            native.Upper>double(v.SphereMargin)||reference.Upper>double(v.SphereMargin))return false;
        // Call the real compiled consumed-position function: its float(s*s)
        // DAG differs from the initial World-call endpoint/chord expression.
        center=SBox::AdvancePosition(v.OwnerState.Location,v.OwnerState.Velocity,v.OwnerState.Acceleration,seconds);
        const auto bound=model.BoundCenter(seconds,seconds,true);
        if(center.ContainsNaN()||!Analytic::Certified::Detail::ValidPoint(bound))return false;
        for(int32 d=0;d<3;++d)if(center[d]<bound.Axis[d].Lower||center[d]>bound.Axis[d].Upper)return false;
        error=FMath::Max(native.Upper,reference.Upper);
    }
    else return false; // No Box contact or unsupported overlap is fabricated.
    const double planeZ=g.NativeCeilingSphereArrivalZ;
    const FVector normal=ceiling?FVector(0.,0.,-1.):initial.SideNormal;
    if((center.Z<planeZ?-1.:1.)!=normal.Z)return false;
    const FVector witness(center.X,center.Y,planeZ);
    const FVector queryPoint=center-v.SupportRadius*normal;
    if(queryPoint.ContainsNaN())return false;
    for(const auto& point:{center,witness,queryPoint})for(int32 d=0;d<3;++d)
        if(point[d]<g.QueryEnvelope.Axis[d].Lower||point[d]>g.QueryEnvelope.Axis[d].Upper)return false;
    if(!FMath::IsFinite(v.Horizon)||v.Horizon<=0||seconds>v.Horizon||!FMath::IsFinite(error)||error<0)return false;
    FNativeFloorSphereContactGeometryValues value;value.Seconds=seconds;value.OwnerFrame=v.FrameClock.OwnerLocalFrame;
    auto& hit=value.Hit;hit.bHit=true;hit.bStartPenetrating=overlap;hit.PenetrationDepth=depth;
    // Time is normalized descriptor data only; native delivery MUST retain
    // Seconds directly and must never recover it through Time*Horizon.
    hit.Time=double(seconds)/double(v.Horizon);hit.Location=center;hit.Point=witness;hit.QueryPoint=queryPoint;hit.Normal=normal;
    hit.GeometricErrorBoundCm=error;hit.bSurfaceNormalMayVary=false;
    hit.QueryFeatureKind=EContactFeatureKind::Vertex;hit.SurfaceFeatureKind=EContactFeatureKind::Face;
    hit.SourceId=g.SourceId;hit.SurfaceId=g.SurfaceId;hit.FeatureId=g.FeatureId;hit.PrimitiveId=g.PrimitiveId;
    hit.CanonicalGroupId=g.CanonicalGroupId;hit.MaterialId=g.MaterialId;
    // QueryPoint/depth use SUPPORT Rs. The existing response-point policy
    // remains separate (offset-zero planar overlap keeps the floor witness).
    // Point stays on the real floor for the existing Ball support-plane hook.
    output=MoveTemp(value);return true;
}
}
// Contact GEOMETRY VALUES ONLY: false is Unknown/unsupported, never a miss.
// No native hit/evidence/typed Contact delivery is installed here. The ordinary
// member, selected event, consumed state, response/constraint/projection/Ball
// contracts remain mandatory before using these values in existing resolves.
bool BuildNativeFloorSphereContactGeometry(const FPreparedNativeSolidGeometryPtr& geometry,
    FNativeFloorSphereContactGeometryValues& output)
{
    output={};Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    const auto current=[&]()
    {
        FStepIdentity now;return geometry&&geometry->GetRequest()&&RevalidateNativeSolidCCD(*geometry->GetRequest())&&
            ReadCurrentStep(now)&&now.QuerySerial==geometry->GetRequest()->GetStep().QuerySerial;
    };
    if(!current())return false;FNativeFloorSphereContactGeometryValues result;
    if(!NativeFloorSphereContactDetail::Build(geometry->GetRequest()->GetValues(),geometry->GetValues(),result))return false;
    Analytic::RequireCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!current())return false;output=MoveTemp(result);return true;
}
