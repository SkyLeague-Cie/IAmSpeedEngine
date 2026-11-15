#include "InitialFloorSphereGeometry.h"
#include "IAmSpeed/World/Collision/NativeFloorSphereContactGeometry.h"
#include "IAmSpeed/World/Collision/PreparedSleepingSphereSupport.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FInitialFloorSphereGeometryTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.InitialCapturedSphereSupportAndSkin",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FInitialFloorSphereGeometryTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;using namespace Speed::PreparedExecution;
    FNativeSolidStaticValues v;v.Shape=ENativeSolidQueryShape::Sphere;v.PhysicalRadius=1.f;
    v.StaticContactOffset=1.f;v.SupportRadius=2.;v.SphereMarginCaptured=true;v.SphereMargin=0.01f;
    auto at=[&](double z){v.OwnerState.Location=FVector(3,-4,z);v.BodyState=v.OwnerState;return ClassifyCapturedFloorSphereGeometry(v);};
    auto result=at(2.);
    TestTrue(TEXT("captured exact Rs touch proved without Integrate(0)"),result.SupportSide==EInitialFloorSphereSide::ExactTouch&&
        result.SupportGap.Known&&result.SupportGap.Lower==0&&result.SupportGap.Upper==0);
    TestTrue(TEXT("static skin touch is separate from physical sphere gap"),result.PhysicalSide==EInitialFloorSphereSide::Separated&&result.PhysicalGap.Lower>0);
    TestTrue(TEXT("surface witness remains z0 and captured center unchanged"),result.PlaneWitness==FVector(3,-4,0)&&result.CapturedCenter==FVector(3,-4,2)&&result.SideNormal==FVector(0,0,1));
    result=at(2.001);TestTrue(TEXT("positive gap within margin remains separated and creates no hit"),result.SupportSide==EInitialFloorSphereSide::Separated&&result.PositiveSupportGapWithinCapturedMargin);
    result=at(1.5);TestTrue(TEXT("overlap of support skin need not penetrate physical radius"),result.SupportSide==EInitialFloorSphereSide::Overlap&&result.PhysicalSide==EInitialFloorSphereSide::Separated);
    result=at(1.);TestTrue(TEXT("physical touch within support overlap retained"),result.SupportSide==EInitialFloorSphereSide::Overlap&&result.PhysicalSide==EInitialFloorSphereSide::ExactTouch);
    result=at(0.5);TestTrue(TEXT("physical and skin overlap both explicit"),result.SupportSide==EInitialFloorSphereSide::Overlap&&result.PhysicalSide==EInitialFloorSphereSide::Overlap);
    result=at(-2.);TestTrue(TEXT("opposite-side mathematical geometry symmetric"),result.SupportSide==EInitialFloorSphereSide::ExactTouch&&result.SideNormal==FVector(0,0,-1));
    result=at(-0.);TestTrue(TEXT("on-plane center has no fabricated unique MTD normal"),result.SupportSide==EInitialFloorSphereSide::Unknown);
    at(2.);v.SupportRadius=2.0001;TestTrue(TEXT("Rs must match actual Rp plus actual offset addition bits"),ClassifyCapturedFloorSphereGeometry(v).SupportSide==EInitialFloorSphereSide::Unknown);
    v.SupportRadius=2.;const uint32 control=VectorGetControlRegister();VectorSetControlRegister(control|uint32(0x8040));
    v.StaticContactOffset=KinematicCenterDetail::FromBits(0x80000001u);
    TestTrue(TEXT("negative subnormal offset cannot flush into valid zero"),ClassifyCapturedFloorSphereGeometry(v).SupportSide==EInitialFloorSphereSide::Unknown);
    v.StaticContactOffset=1.f;v.OwnerState.Location.Z=AnalyticCertified::FromBits(0x8000000000000001ULL);v.BodyState=v.OwnerState;
    result=ClassifyCapturedFloorSphereGeometry(v);
    TestTrue(TEXT("bit sign/magnitude geometry survives DAZ but is not native normal permission"),result.SupportSide==EInitialFloorSphereSide::Overlap&&result.SideNormal.Z==-1.);
    // Pure geometry fixtures, not gameplay profiles/spawns or physical PASS.
    using namespace Speed;
    v={};v.Shape=ENativeSolidQueryShape::Sphere;v.Horizon=1;v.PhysicalRadius=1;v.SupportRadius=1.25;
    v.StaticContactOffset=0.25f;v.QueryRadius=1.5f;v.SphereMarginCaptured=true;v.SphereMargin=0.001f;
    v.OwnerState.Location=FVector(4,-3,1.25);v.BodyState=v.OwnerState;
    FNativeSolidGeometryValues g;g.Status=ENativeSolidGeometryStatus::CapturedSphereTouch;
    g.SourceId=1;g.SurfaceId=2;g.FeatureId=3;g.PrimitiveId=4;
    for(int32 d=0;d<3;++d)g.QueryEnvelope.Axis[d]=Interval{-100,100,true};
    FNativeFloorSphereContactGeometryValues payload;
    TestTrue(TEXT("captured support touch yields +0 geometric payload"),NativeFloorSphereContactDetail::Build(v,g,payload));
    TestTrue(TEXT("floor witness stays on actual plane, not physical response arm Rp or rounded Rc"),payload.Seconds==0&&
        payload.Hit.Location==FVector(4,-3,1.25)&&payload.Hit.Point==FVector(4,-3,0)&&payload.Hit.QueryPoint==FVector(4,-3,0)&&
        !payload.Hit.bStartPenetrating&&payload.Hit.GeometricErrorBoundCm==0);
    v.OwnerState.Location.Z=-1.25;v.BodyState=v.OwnerState;
    TestTrue(TEXT("two-sided floor payload preserves negative normal"),NativeFloorSphereContactDetail::Build(v,g,payload)&&payload.Hit.Normal==FVector(0,0,-1));
    g.Status=ENativeSolidGeometryStatus::CapturedSphereOverlap;
    v.OwnerState.Location.Z=1.125;v.BodyState=v.OwnerState;
    TestTrue(TEXT("captured skin overlap gives exact support depth without physical overlap"),NativeFloorSphereContactDetail::Build(v,g,payload)&&
        payload.Seconds==0&&payload.Hit.bStartPenetrating&&payload.Hit.PenetrationDepth==0.125&&
        payload.Hit.Location==v.BodyState.Location&&payload.Hit.QueryPoint==FVector(4,-3,-0.125)&&payload.Hit.Point==FVector(4,-3,0)&&
        ClassifyCapturedFloorSphereGeometry(v).PhysicalSide==EInitialFloorSphereSide::Separated);
    v.OwnerState.Location.Z=-1.125;v.BodyState=v.OwnerState;
    TestTrue(TEXT("captured overlap geometry is bidirectional with the real floor witness"),NativeFloorSphereContactDetail::Build(v,g,payload)&&
        payload.Hit.Normal==FVector(0,0,-1)&&payload.Hit.PenetrationDepth==0.125&&payload.Hit.QueryPoint==FVector(4,-3,0.125)&&payload.Hit.Point.Z==0);
    v.OwnerState.Location.Z=0.875;v.BodyState=v.OwnerState;
    TestTrue(TEXT("support depth remains Rs depth when Rp also overlaps"),NativeFloorSphereContactDetail::Build(v,g,payload)&&
        payload.Hit.PenetrationDepth==0.375&&ClassifyCapturedFloorSphereGeometry(v).PhysicalSide==EInitialFloorSphereSide::Overlap);
    v.OwnerState.Location.Z=0.5;v.BodyState=v.OwnerState;
    TestFalse(TEXT("overlap outside the existing exact shallow-depth range remains Unknown"),NativeFloorSphereContactDetail::Build(v,g,payload));
    TestFalse(TEXT("unsupported overlap clears the previous hit"),payload.Hit.bHit);
    v.OwnerState.Location.Z=-0.;v.BodyState=v.OwnerState;
    TestFalse(TEXT("on-plane overlap cannot invent a unique bidirectional normal"),NativeFloorSphereContactDetail::Build(v,g,payload));
    v.OwnerState.Location.Z=AnalyticCertified::FromBits(0x8000000000000001ULL);v.BodyState=v.OwnerState;
    TestFalse(TEXT("DAZ mathematical overlap is not a native depth capability"),NativeFloorSphereContactDetail::Build(v,g,payload));
    v.OwnerState.Location.Z=1.25;v.BodyState=v.OwnerState;
    TestFalse(TEXT("overlap status cannot impersonate captured touch"),NativeFloorSphereContactDetail::Build(v,g,payload));
    g.Status=ENativeSolidGeometryStatus::CapturedSphereTouch;
    v.OwnerState.Location.Z=1.125;v.BodyState=v.OwnerState;
    TestFalse(TEXT("touch status cannot hide positive support penetration"),NativeFloorSphereContactDetail::Build(v,g,payload));
    v.OwnerState.Location.Z=1.2505;v.BodyState=v.OwnerState;
    TestFalse(TEXT("positive gap within margin cannot impersonate captured touch"),NativeFloorSphereContactDetail::Build(v,g,payload));
    g.Status=ENativeSolidGeometryStatus::FirstArrivalBracket;g.Sweep.Status=EFloorNativeSweepStatus::ArrivalBracket;
    g.Sweep.CapturedSphereGapSatisfied=true;g.Sweep.ClearPrefixSeconds=0;g.Sweep.InsideSeconds=0.1f;
    TestFalse(TEXT("positive-gap arrival bracket cannot fabricate +0 native event"),NativeFloorSphereContactDetail::Build(v,g,payload));
    // Positive initial gap within the CAPTURED margin still requires a real
    // positive consumed prefix. Keep the +0 refusal above, with no synthetic
    // time, margin increase, pose/profile change or budget refill.
    const uint32 arrivalControls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((arrivalControls&~uint32(0x8040))|flush);
        v.Horizon=1.f;v.OwnerState.Location=FVector(4,-3,1.2505);
        v.OwnerState.Velocity=FVector(0,0,-4);v.OwnerState.Acceleration=FVector(0,0,-8);v.BodyState=v.OwnerState;
        FFloorNativeSolidModel closeModel;
        if(!TestTrue(TEXT("near-floor positive-gap fixture model"),BuildFloorNativeSolidModel(v,closeModel)))return false;
        FDistanceBudget closeBudget{8192};g.Sweep=BoundFloorNativeFirstArrival(closeModel,closeBudget);
        TestTrue(TEXT("near-floor arrival never returns a fabricated +0 prefix"),g.Sweep.Status==EFloorNativeSweepStatus::ArrivalBracket&&
            g.Sweep.ClearPrefixSeconds>0.f&&g.Sweep.CapturedSphereGapSatisfied&&
            g.Sweep.ConsumedGap.Lower>0&&g.Sweep.ReferenceGap.Lower>0&&
            g.Sweep.ConsumedGap.Upper<=double(v.SphereMargin)&&g.Sweep.ReferenceGap.Upper<=double(v.SphereMargin));
        TestTrue(TEXT("positive initial margin-band arrival reaches unchanged native payload consumer"),
            NativeFloorSphereContactDetail::Build(v,g,payload)&&payload.Seconds>0.f&&payload.Hit.Point.Z==0&&
            !payload.Hit.bStartPenetrating&&payload.Hit.GeometricErrorBoundCm<=double(v.SphereMargin));
        FDistanceBudget noBudget{0};TestTrue(TEXT("no budget cannot manufacture near-floor positive-time arrival"),
            BoundFloorNativeFirstArrival(closeModel,noBudget).Status==EFloorNativeSweepStatus::Unknown);
    }
    VectorSetControlRegister(arrivalControls);

    v.OwnerState.Location.Z=4;v.OwnerState.Velocity=FVector(0,0,-4);v.OwnerState.Acceleration=FVector(0,0,-8);v.BodyState=v.OwnerState;
    FFloorNativeSolidModel model;TestTrue(TEXT("arrival payload model constructed"),BuildFloorNativeSolidModel(v,model));
    FDistanceBudget arrivalBudget{8192};g.Sweep=BoundFloorNativeFirstArrival(model,arrivalBudget);
    TestTrue(TEXT("positive-time bounded arrival produces geometry at actual native float seconds"),NativeFloorSphereContactDetail::Build(v,g,payload)&&
        payload.Seconds>0&&payload.Hit.Point.Z==0&&payload.Hit.QueryPoint.Z>0&&payload.Hit.GeometricErrorBoundCm<=double(v.SphereMargin));
    // Support-description fixtures are pure geometry, not a native sleep
    // decision, pose/profile, query, tangency or nonpenetration PASS.
    v.OwnerState.Location=FVector(4,-3,1.5);v.OwnerState.Velocity=FVector::ZeroVector;
    v.OwnerState.Acceleration=FVector::ZeroVector;v.BodyState=v.OwnerState;
    FSerializedSphereSupportManifold manifold{FVector(0,0,1),FVector(4,-3,0)};
    FSerializedSphereSupportEnvelope support;
    TestTrue(TEXT("serialized floor presence preserves a positive Rs gap"),BuildSerializedSphereSupportEnvelope(v,manifold,support)&&
        support.Initial.SupportSide==EInitialFloorSphereSide::Separated&&support.Initial.SupportGap.Lower>0&&support.NativeSupportGap.Lower>0);
    v.OwnerState.Location.Z=-1.5;v.BodyState=v.OwnerState;manifold.Normal=FVector(0,0,-1);
    TestTrue(TEXT("serialized coordinate-plane geometry is bidirectional"),BuildSerializedSphereSupportEnvelope(v,manifold,support)&&support.Initial.SideNormal.Z==-1);
    manifold.Point.Z=AnalyticCertified::FromBits(1);
    TestFalse(TEXT("DAZ cannot turn serialized nonzero plane height into z0"),BuildSerializedSphereSupportEnvelope(v,manifold,support));
    manifold.Point.Z=0;manifold.Normal.X=AnalyticCertified::FromBits(1);
    TestFalse(TEXT("DAZ cannot turn nonzero manifold normal into coordinate axis"),BuildSerializedSphereSupportEnvelope(v,manifold,support));
    FPreparedNativeSolidGeometryPtr unowned;
    TestFalse(TEXT("unowned caller cannot convert raw geometry into a sealed native result"),BuildNativeFloorSphereContactGeometry(unowned,payload));
    TestFalse(TEXT("failed public conversion clears prior geometric hit"),payload.Hit.bHit);
    VectorSetControlRegister(control);TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
