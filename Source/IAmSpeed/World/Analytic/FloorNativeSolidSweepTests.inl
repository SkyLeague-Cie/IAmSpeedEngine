#include "FloorNativeSolidSweepBounds.h"
#include "FixedAxialBoxFirstArrival.h"
#include "CanonicalAxialPlaneBounds.h"
#include "NativeSolidSweepVolume.h"
#include "IAmSpeed/World/Collision/NativeBoxGroundReuseValues.h"
#include <limits>
#include <cmath>
#include "IAmSpeed/World/Collision/NativeFloorBoxPlaneValues.h"
#include "IAmSpeed/World/Collision/NativeCanonicalBoxSweepClearanceValues.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFloorNativeSolidSweepBoundsTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.FloorArrivalAndWholePrefix",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FFloorNativeSolidSweepBoundsTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    using namespace Speed::PreparedExecution;
    FNativeBoxGroundReuseDecisionValues reuse;
    using ReuseOutcome=ENativeBoxGroundReuseOutcome;
    TestTrue(TEXT("no persistent cache observation differs from separation"),ClassifyNativeBoxGroundReuseDecision(reuse)==ReuseOutcome::NoPersistent);
    reuse.Persistent=true;
    TestTrue(TEXT("unevaluated velocity cannot close native persistent branch"),ClassifyNativeBoxGroundReuseDecision(reuse)==ReuseOutcome::Incomplete);
    reuse.VelocityComputed=true;reuse.NormalSpeed=std::numeric_limits<float>::quiet_NaN();
    TestTrue(TEXT("actual false unordered comparison is not labelled separating"),ClassifyNativeBoxGroundReuseDecision(reuse)==ReuseOutcome::NormalComparisonFalse);
    reuse.NormalSpeed=0;reuse.NormalComparison=true;
    TestTrue(TEXT("missing real ignore predicate cannot imply cache reuse"),ClassifyNativeBoxGroundReuseDecision(reuse)==ReuseOutcome::Incomplete);
    reuse.IgnoreComputed=true;reuse.Ignored=true;
    TestTrue(TEXT("ignored branch distinct from returned zero event"),ClassifyNativeBoxGroundReuseDecision(reuse)==ReuseOutcome::Ignored);
    reuse.Ignored=false;
    TestTrue(TEXT("native positive reuse outcome retained"),ClassifyNativeBoxGroundReuseDecision(reuse)==ReuseOutcome::Reused);
    reuse.Persistent=false;
    TestTrue(TEXT("computed subbranches cannot be attached to absent persistent decision"),ClassifyNativeBoxGroundReuseDecision(reuse)==ReuseOutcome::Incomplete);
    // Pure mathematical inputs, not a spawned ball/profile/World acceptance.
    FNativeSolidStaticValues sphere;sphere.Shape=ENativeSolidQueryShape::Sphere;
    sphere.OwnerState.Location=FVector(0,0,4);sphere.OwnerState.Velocity=FVector(0,0,-4);
    sphere.OwnerState.Acceleration=FVector(0,0,-8);sphere.BodyState=sphere.OwnerState;
    sphere.Horizon=1.f;sphere.SupportRadius=1.;sphere.PhysicalRadius=1.f;sphere.QueryRadius=1.001f;
    sphere.SphereMarginCaptured=true;sphere.SphereMargin=0.001f;
    FFloorNativeSolidModel model;TestTrue(TEXT("actual zero-offset sphere model built"),BuildFloorNativeSolidModel(sphere,model));
    FDistanceBudget budget{8192};auto hit=BoundFloorNativeFirstArrival(model,budget);
    TestTrue(TEXT("accelerated arrival has precontact/inside bracket"),hit.Status==EFloorNativeSweepStatus::ArrivalBracket);
    if(hit.Status==EFloorNativeSweepStatus::ArrivalBracket)
    {
        TestTrue(TEXT("exact root1/2 enclosed with strict clear consumed prefix"),hit.ClearPrefixSeconds<0.5f&&hit.InsideSeconds>0.5f);
        TestTrue(TEXT("native and reference gap use actual captured margin"),hit.ConsumedGap.Lower>0&&hit.ConsumedGap.Upper<=double(sphere.SphereMargin)&&
            hit.ReferenceGap.Lower>0&&hit.ReferenceGap.Upper<=double(sphere.SphereMargin));
        TestTrue(TEXT("Sphere gap requirement separately recorded"),hit.CapturedSphereGapSatisfied);
    }
    FDistanceBudget empty{0};TestTrue(TEXT("exhausted budget cannot certify anything"),BoundFloorNativeFirstArrival(model,empty).Status==EFloorNativeSweepStatus::Unknown);
    const uint32 originalControl=VectorGetControlRegister();VectorSetControlRegister(originalControl|uint32(0x8040));
    sphere.SphereMargin=KinematicCenterDetail::FromBits(0x80000001u);
    TestFalse(TEXT("negative subnormal margin cannot flush into accepted zero budget"),BuildFloorNativeSolidModel(sphere,model));
    sphere.SphereMargin=0.001f;VectorSetControlRegister(originalControl);
    TestTrue(TEXT("valid model restored after margin refusal"),BuildFloorNativeSolidModel(sphere,model));
    VectorSetControlRegister((originalControl&~uint32(0x6000))|uint32(0x2000));bool refused=false;
    try{(void)model.BoundCenter(0.f,0.5f,false);}catch(const FAnalyticNumericUnknown&){refused=true;}
    VectorSetControlRegister(originalControl);TestTrue(TEXT("direct reference bound refuses non-RNE mode"),refused);
    sphere.OwnerState.Location.Z=2;sphere.OwnerState.Velocity.Z=-4;sphere.OwnerState.Acceleration.Z=8;
    sphere.BodyState=sphere.OwnerState;TestTrue(TEXT("returning tangent model built"),BuildFloorNativeSolidModel(sphere,model));
    FDistanceBudget tangentBudget{8192};TestTrue(TEXT("interior tangency with clear endpoints never becomes separated"),BoundFloorNativeFirstArrival(model,tangentBudget).Status==EFloorNativeSweepStatus::Unknown);
    sphere.OwnerState.Location.Z=3;sphere.BodyState=sphere.OwnerState;
    TestTrue(TEXT("whole-clear returning path model built"),BuildFloorNativeSolidModel(sphere,model));
    FDistanceBudget clearBudget{8192};TestTrue(TEXT("whole prefix rather than endpoints proves separation"),BoundFloorNativeFirstArrival(model,clearBudget).Status==EFloorNativeSweepStatus::Separated);
    sphere.OwnerState.Location.Z=1;sphere.BodyState=sphere.OwnerState;
    TestTrue(TEXT("initial tangent geometry builds but needs support operation"),BuildFloorNativeSolidModel(sphere,model));
    FDistanceBudget initialBudget{8192};TestTrue(TEXT("initial tangent is conservatively unknown"),BoundFloorNativeFirstArrival(model,initialBudget).Status==EFloorNativeSweepStatus::Unknown);
    sphere.Horizon=KinematicCenterDetail::FromBits(1);TestFalse(TEXT("subnormal horizon refused"),BuildFloorNativeSolidModel(sphere,model));
    TestFalse(TEXT("refusal clears previous model"),model.IsKnown());
    FNativeSolidStaticValues box;box.Shape=ENativeSolidQueryShape::Box;box.Horizon=0.25f;
    box.OwnerState.Location=FVector(1.e16,-2,64);box.OwnerState.Velocity=FVector(-4,3,1);box.OwnerState.Acceleration=FVector(2,-1,-4);
    box.LocalCOM=FVector(1,0,5);box.LocalOffset=FVector(-1,2,3);box.PhysicalHalfExtent=FVector(3,4,2);
    box.LocalRotation=FQuat(FVector(0,0,1),0.4);box.BodyState=box.OwnerState;
    box.BodyState.Rotation=box.OwnerState.Rotation*box.LocalRotation;
    const FVector rc=box.OwnerState.Rotation.RotateVector(box.LocalCOM),ro=box.OwnerState.Rotation.RotateVector(box.LocalOffset);
    box.BodyState.Location=(FVector(box.OwnerState.Location)-rc)+ro;
    TestTrue(TEXT("fixed rotated offset/COM Box with physical double extents builds"),BuildFloorNativeSolidModel(box,model));
    for(float seconds:{0.f,0.01f,0.125f,0.25f})
    {
        const auto actualOwner=box.OwnerState.Integrate(seconds);
        const FVector actualCenter=(FVector(actualOwner.Location)-actualOwner.Rotation.RotateVector(box.LocalCOM))+
            actualOwner.Rotation.RotateVector(box.LocalOffset);
        const auto center=model.BoundCenter(seconds,seconds,true),hull=model.BoundSupportHull();
        TestTrue(TEXT("native center/hull finite"),Detail::ValidPoint(center)&&Detail::ValidPoint(hull));
        for(int32 d=0;d<3;++d)TestTrue(TEXT("actual two-stage offset DAG enclosed"),center.Axis[d].Lower<=actualCenter[d]&&center.Axis[d].Upper>=actualCenter[d]);
        for(int32 k=0;k<8;++k)
        {
            FVector corner=FVector::ZeroVector;for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?box.PhysicalHalfExtent[d]:-box.PhysicalHalfExtent[d];
            const FVector point=actualCenter+box.BodyState.Rotation.RotateVector(corner);
            for(int32 d=0;d<3;++d)TestTrue(TEXT("all eight actual fixed corners in whole sweep hull"),hull.Axis[d].Lower<=point[d]&&hull.Axis[d].Upper>=point[d]);
        }
    }
    FDistanceBudget boxBudget{8192};TestTrue(TEXT("fixed Box can prove complete clear motion"),BoundFloorNativeFirstArrival(model,boxBudget).Status==EFloorNativeSweepStatus::Separated);
    box.OwnerState.Location=FVector(0,0,4);box.OwnerState.Velocity=FVector(0,0,-8);box.OwnerState.Acceleration=FVector::ZeroVector;
    box.LocalCOM=FVector::ZeroVector;box.LocalOffset=FVector::ZeroVector;box.PhysicalHalfExtent=FVector(1,1,1);box.Horizon=1;
    box.LocalRotation=FQuat::Identity;box.BodyState=box.OwnerState;
    TestTrue(TEXT("fixed translating Box arrival model built"),BuildFloorNativeSolidModel(box,model));
    FDistanceBudget boxArrivalBudget{8192};const auto boxArrival=BoundFloorNativeFirstArrival(model,boxArrivalBudget);
    TestTrue(TEXT("Box obtains geometric arrival bracket without invented margin"),boxArrival.Status==EFloorNativeSweepStatus::ArrivalBracket&&
        boxArrival.ClearPrefixSeconds<0.375f&&boxArrival.InsideSeconds>0.375f&&!boxArrival.CapturedSphereGapSatisfied);
    // Box refinement consumes a plane DESCRIPTOR, not a hit at the positive
    // bracket low. Raw fixture values below confer no World/native authority.
    FNativeSolidGeometryValues boxGeometry;boxGeometry.Status=ENativeSolidGeometryStatus::FirstArrivalBracket;
    boxGeometry.Sweep=boxArrival;boxGeometry.SourceId=1;boxGeometry.SurfaceId=2;boxGeometry.FeatureId=3;boxGeometry.PrimitiveId=4;
    for(int32 d=0;d<3;++d)boxGeometry.QueryEnvelope.Axis[d]={-16,16,true};
    SHitResult planeDescriptor;
    TestTrue(TEXT("fixed Box geometry supplies a plane descriptor"),NativeFloorBoxPlaneDetail::Build(box,boxGeometry,planeDescriptor));
    TestFalse(TEXT("plane descriptor is never a reported contact"),planeDescriptor.bHit||planeDescriptor.bBlockingHit);
    TestTrue(TEXT("plane descriptor carries no UObject identity"),planeDescriptor.Component.IsExplicitlyNull()&&planeDescriptor.SubBody.IsExplicitlyNull());
    auto refined=planeDescriptor;refined.TOI=.375f;
    refined.Location=box.OwnerState.Integrate(refined.TOI).Location;
    refined.ContactFeatureThis=EContactFeatureKind::Vertex;refined.ContactFeatureIndexThis=0;
    refined.ContactPointThis=refined.Location-FVector(1,1,1);
    refined.ContactPointOther=refined.ImpactPoint=FVector(-1,-1,0);
    TestTrue(TEXT("actual centre/physical corner/finite floor values match"),NativeFloorBoxPlaneDetail::MatchesRefinement(box,boxGeometry,refined));
    auto wrong=refined;wrong.ContactFeatureIndexThis=4;
    TestFalse(TEXT("wrong physical corner index cannot alias same point"),NativeFloorBoxPlaneDetail::MatchesRefinement(box,boxGeometry,wrong));
    wrong=refined;wrong.TOI=0;
    TestFalse(TEXT("initial contact is not a positive first arrival"),NativeFloorBoxPlaneDetail::MatchesRefinement(box,boxGeometry,wrong));
    wrong=refined;wrong.bHit=true;
    TestFalse(TEXT("a precreated contact cannot stand in for descriptor refinement"),NativeFloorBoxPlaneDetail::MatchesRefinement(box,boxGeometry,wrong));
    const auto savedBox=box;box.LocalRotation=FQuat(FVector(0,1,0),-.55*PI/180.);
    box.BodyState.Rotation=box.OwnerState.Rotation*box.LocalRotation;
    FFloorNativeSolidModel pitchedModel;FDistanceBudget pitchedBudget{8192};
    TestTrue(TEXT("fixed-pitch model retains the actual captured quaternion"),BuildFloorNativeSolidModel(box,pitchedModel));
    boxGeometry.Sweep=BoundFloorNativeFirstArrival(pitchedModel,pitchedBudget);
    TestTrue(TEXT("fixed-pitch geometry obtains its own first-arrival bracket"),boxGeometry.Sweep.Status==EFloorNativeSweepStatus::ArrivalBracket);
    TestTrue(TEXT("fixed pitch descriptor uses that matching geometry without yaw-only retuning"),NativeFloorBoxPlaneDetail::Build(box,boxGeometry,planeDescriptor));
    box.OwnerState.AngularVelocity.X=1;box.BodyState.AngularVelocity.X=1;
    TestFalse(TEXT("rotating Box remains explicitly unsupported"),NativeFloorBoxPlaneDetail::Build(box,boxGeometry,planeDescriptor));
    box=savedBox;
    // The geometric union envelope must retain actual Rc independently from
    // Rs, physical Box corners independently from rounded query extents, and
    // the full accelerated excursion rather than just its chord endpoints.
    sphere.OwnerState.Location=FVector(0,0,3);sphere.OwnerState.Velocity=FVector(0,0,-4);
    sphere.OwnerState.Acceleration=FVector(0,0,8);sphere.BodyState=sphere.OwnerState;sphere.Horizon=1;
    sphere.QueryRadius=2.f;sphere.Query.Shape=EQueryShape::Sphere;sphere.Query.Start=sphere.BodyState.Location;
    sphere.Query.End=FVector(0,0,3);sphere.Query.Radius=2.;sphere.Query.Rotation=sphere.BodyState.Rotation;
    FIntervalPoint volume;
    TestTrue(TEXT("native union envelope combines reference/consumed/captured/rounded chord"),BuildNativeSolidSweepVolume(sphere,model,volume));
    TestTrue(TEXT("rounded Rc excursion reaches z0 despite clear chord endpoints"),Detail::ValidPoint(volume)&&volume.Axis[2].Lower<=0);
    sphere.Query.Radius=1.;TestFalse(TEXT("mismatched captured query shape cannot produce union envelope"),BuildNativeSolidSweepVolume(sphere,model,volume));
    TestFalse(TEXT("failed envelope clears prior volume"),Detail::ValidPoint(volume));
    box.QueryHalfExtent=FVector(2,3,4);box.Query.Shape=EQueryShape::Box;box.Query.HalfExtent=box.QueryHalfExtent;
    box.Query.Start=box.BodyState.Location;box.Query.End=FVector(0,0,-4);box.Query.Rotation=box.BodyState.Rotation;
    TestTrue(TEXT("Box physical and independently rounded query extents retained"),BuildNativeSolidSweepVolume(box,model,volume));
    TestTrue(TEXT("query corner z-8 remains in complete Box envelope"),Detail::ValidPoint(volume)&&volume.Axis[2].Lower<=-8);
    box.OwnerState.AngularVelocity.Z=0.1;box.BodyState.AngularVelocity.Z=0.1;
    TestFalse(TEXT("rotating Box requires real rotating CCD, never fixed/chord promotion"),BuildFloorNativeSolidModel(box,model));
    // Pure yaw mathematical fixtures exercise geometry only; this is not a
    // spawned car/profile, live World admission or an executed physics result.
    FNativeSolidStaticValues yaw;const float yawH=0.25f;
    yaw.Shape=ENativeSolidQueryShape::Box;yaw.Horizon=yawH;
    yaw.OwnerState.Location=FVector(0,0,64);yaw.OwnerState.Rotation=FQuat(FVector(0,0,1),0.3);
    yaw.OwnerState.AngularVelocity=FVector(0,0,2);yaw.OwnerState.AngularAcceleration=FVector(0,0,-0.5);
    yaw.OwnerState.Velocity=FVector(3,-2,-1);yaw.OwnerState.Acceleration=FVector(0,0,2);
    yaw.LocalCOM=FVector(0.25,-0.5,3);yaw.LocalOffset=FVector(-1,2,4);
    yaw.LocalRotation=FQuat(FVector(0,1,0),-.55*PI/180.);yaw.PhysicalHalfExtent=FVector(59,42,18);
    auto captureYaw=[&]()
    {
        yaw.BodyState=yaw.OwnerState;
        yaw.BodyState.Rotation=yaw.OwnerState.Rotation*yaw.LocalRotation;
        yaw.BodyState.Location=(FVector(yaw.OwnerState.Location)-yaw.OwnerState.Rotation.RotateVector(yaw.LocalCOM))+
            yaw.OwnerState.Rotation.RotateVector(yaw.LocalOffset);
        // Deliberately distinct from owner linear PVA, as for rotating offsets.
        yaw.BodyState.Velocity+=FVector(1,2,0);yaw.BodyState.Acceleration+=FVector(-2,1,0);
        yaw.Query.Shape=EQueryShape::Box;yaw.QueryHalfExtent=FVector(59.001f,42.001f,18.001f);
        yaw.Query.HalfExtent=yaw.QueryHalfExtent;yaw.Query.Start=yaw.BodyState.Location;yaw.Query.Rotation=yaw.BodyState.Rotation;
        yaw.Query.End=yaw.BodyState.Location+yaw.BodyState.Velocity*yaw.Horizon+
            0.5f*yaw.BodyState.Acceleration*yaw.Horizon*yaw.Horizon;
    };
    captureYaw();FYawFloorBoxSweepBounds yawBounds;
    TestTrue(TEXT("yaw whole-H envelope retains local pitch and rotating-offset body PVA"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    TestTrue(TEXT("yaw physical complete interval is strictly clear"),yawBounds.WholeFloorSeparated&&
        yawBounds.ConsumedGap.Lower>0&&yawBounds.ReferenceGap.Lower>0);
    for(float seconds:{0.f,0.01f,0.125f,0.25f})
    {
        const auto ownerPose=yaw.OwnerState.Integrate(seconds);
        const FVector center=(FVector(ownerPose.Location)-ownerPose.Rotation.RotateVector(yaw.LocalCOM))+
            ownerPose.Rotation.RotateVector(yaw.LocalOffset);
        const FQuat bodyRotation=ownerPose.Rotation*yaw.LocalRotation;
        for(int32 k=0;k<8;++k)
        {
            FVector corner=FVector::ZeroVector;for(int32 d=0;d<3;++d)corner[d]=(k&(1<<d))?yaw.PhysicalHalfExtent[d]:-yaw.PhysicalHalfExtent[d];
            const auto actual=center+bodyRotation.RotateVector(corner);
            for(int32 d=0;d<3;++d)TestTrue(TEXT("actual integrated yaw Box corner enclosed"),
                yawBounds.QueryEnvelope.Axis[d].Lower<=actual[d]&&yawBounds.QueryEnvelope.Axis[d].Upper>=actual[d]);
        }
    }
    const auto yawSaved=yaw;
    yaw.OwnerState.Location.Z=-64;captureYaw();
    TestTrue(TEXT("opposite floor side separated without one-sided yaw assumption"),BuildYawFloorBoxSweepBounds(yaw,yawBounds)&&yawBounds.WholeFloorSeparated);
    yaw=yawSaved;yaw.OwnerState.Location.Z=0;yaw.LocalCOM=yaw.LocalOffset=FVector::ZeroVector;captureYaw();
    TestFalse(TEXT("on-plane owner/body has no fabricated unique support side"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    TestFalse(TEXT("failed yaw envelope clears previous output"),Detail::ValidPoint(yawBounds.QueryEnvelope));
    yaw=yawSaved;yaw.OwnerState.Location.Z=5;captureYaw();
    TestTrue(TEXT("yaw overlap geometry may be bounded but is never separated"),BuildYawFloorBoxSweepBounds(yaw,yawBounds)&&!yawBounds.WholeFloorSeparated);
    yaw=yawSaved;yaw.OwnerState.Location.Z=64;yaw.OwnerState.Velocity.Z=-1024;yaw.OwnerState.Acceleration.Z=8192;captureYaw();
    TestTrue(TEXT("clear endpoints cannot hide an interior floor excursion"),BuildYawFloorBoxSweepBounds(yaw,yawBounds)&&!yawBounds.WholeFloorSeparated);
    yaw=yawSaved;yaw.OwnerState.AngularVelocity.X=0.01;captureYaw();
    TestFalse(TEXT("roll/pitch angular motion remains Unknown"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    yaw=yawSaved;yaw.OwnerState.Rotation.X=AnalyticCertified::FromBits(1);captureYaw();
    const uint32 yawControl=VectorGetControlRegister();VectorSetControlRegister(yawControl|uint32(0x8040));
    TestFalse(TEXT("subnormal owner tilt cannot DAZ into admitted pure yaw"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    VectorSetControlRegister(yawControl);
    yaw=yawSaved;yaw.Horizon=1.e20f;captureYaw();
    TestFalse(TEXT("angular float(t*t) overflow is refused before zero-times-infinity"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    yaw=yawSaved;yaw.Query.HalfExtent.X+=1;
    TestFalse(TEXT("changed rounded query extent cannot alias captured shape"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    yaw=yawSaved;yaw.Query.End.X+=1;
    TestFalse(TEXT("changed actual World-call chord cannot alias owner integration"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    // DAZ input error must be amplified by the other operand, not replaced
    // by one smallest-normal allowance at the product output.
    const uint32 errorControl=VectorGetControlRegister();VectorSetControlRegister(errorControl&~uint32(0x8040));
    volatile double subnormal=AnalyticCertified::FromBits(0x0008000000000000ULL),large=1.e300;
    volatile double unflushed=subnormal*large;
    VectorSetControlRegister(errorControl|uint32(0x8040));
    const auto errorProduct=YawFloorBoxDetail::Product(YawFloorBoxDetail::Leaf(subnormal),YawFloorBoxDetail::Leaf(large));
    volatile double flushed=subnormal*large;
    TestTrue(TEXT("DAZ product error retains large-operand amplification"),YawFloorBoxDetail::ValidNode(errorProduct)&&
        errorProduct.Error>=FMath::Abs(double(unflushed)-double(flushed)));
    VectorSetControlRegister(errorControl);
    // Distinct yaw first-arrival model. These are pure mathematical fixture
    // values and matchers, never a real Box member/refiner or a live profile.
    yaw=yawSaved;yaw.Horizon=1.f;yaw.OwnerState.Location.Z=4;
    yaw.OwnerState.Velocity.Z=-7;yaw.OwnerState.Acceleration.Z=0;
    yaw.PhysicalHalfExtent=FVector(1,1,1);
    const auto captureFirstYaw=[&]()
    {
        captureYaw();yaw.QueryHalfExtent=FVector(float(yaw.PhysicalHalfExtent.X),float(yaw.PhysicalHalfExtent.Y),float(yaw.PhysicalHalfExtent.Z));
        yaw.Query.HalfExtent=yaw.QueryHalfExtent;
    };
    captureFirstYaw();FYawFloorBoxMotionModel yawModel;
    TestTrue(TEXT("tight yaw model retains local pitch and distinct owner/body PVA"),BuildYawFloorBoxMotionModel(yaw,yawModel));
    const auto norm=yawModel.GetYawNormSquared();
    const auto capturedNorm=AnalyticCertified::Add(AnalyticCertified::Square(AnalyticCertified::Interval::Exact(yaw.OwnerState.Rotation.Z)),
        AnalyticCertified::Square(AnalyticCertified::Interval::Exact(yaw.OwnerState.Rotation.W)));
    TestTrue(TEXT("tight yaw norm includes native normalization, Q0 return and Identity"),AnalyticCertified::Valid(norm)&&
        norm.Lower<=AnalyticCertified::FromBits(0x3feffffffffffff2ULL)&&norm.Upper>=AnalyticCertified::FromBits(0x3ff0000000000008ULL)&&
        norm.Lower<=capturedNorm.Lower&&norm.Upper>=capturedNorm.Upper&&norm.Lower>0&&norm.Upper<=2);
    FDistanceBudget yawArrivalBudget{8192};const auto yawArrival=BoundYawFloorBoxFirstArrival(yawModel,yawArrivalBudget);
    uint32 yawLow=0,yawHigh=0;
    TestTrue(TEXT("yaw first arrival is the first unique adjacent float leaf"),yawArrival.Status==EFloorNativeSweepStatus::ArrivalBracket&&
        FloorNativeSweepDetail::Ordinal(yawArrival.ClearPrefixSeconds,yawLow)&&FloorNativeSweepDetail::Ordinal(yawArrival.InsideSeconds,yawHigh)&&
        yawLow>0&&yawHigh==yawLow+1&&yawArrival.ConsumedGap.Lower>0&&yawArrival.ReferenceGap.Lower>0&&!yawArrival.CapturedSphereGapSatisfied);
    FNativeSolidGeometryValues yawGeometry;yawGeometry.Status=ENativeSolidGeometryStatus::FirstArrivalBracket;
    yawGeometry.Sweep=yawArrival;yawGeometry.YawBoxFirstArrival=true;yawGeometry.YawNormSquared=norm;
    yawGeometry.SourceId=1;yawGeometry.SurfaceId=2;yawGeometry.FeatureId=3;yawGeometry.PrimitiveId=4;
    TestTrue(TEXT("complete yaw query envelope separately retained"),BuildYawFloorBoxSweepBounds(yaw,yawBounds));
    yawGeometry.QueryEnvelope=yawBounds.QueryEnvelope;SHitResult yawDescriptor;
    TestTrue(TEXT("yaw earliest geometry supplies a false-hit descriptor"),NativeFloorBoxPlaneDetail::Build(yaw,yawGeometry,yawDescriptor));
    TestFalse(TEXT("yaw descriptor is no native contact"),yawDescriptor.bHit||yawDescriptor.bBlockingHit);
    auto yawMatched=yawDescriptor;yawMatched.TOI=yawArrival.ClearPrefixSeconds;
    auto yawPose=yaw.OwnerState.Integrate(yawMatched.TOI);
    yawPose.Location-=yawPose.Rotation.RotateVector(yaw.LocalCOM);yawPose.Location+=yawPose.Rotation.RotateVector(yaw.LocalOffset);
    yawPose.Rotation=yawPose.Rotation*yaw.LocalRotation;
    yawMatched.Location=yawPose.Location;yawMatched.ContactFeatureThis=EContactFeatureKind::Vertex;yawMatched.ContactFeatureIndexThis=0;
    yawMatched.ContactPointThis=yawPose.Location+yawPose.Rotation.RotateVector(-yaw.PhysicalHalfExtent);
    yawMatched.ImpactPoint=yawMatched.ContactPointOther=FVector(yawMatched.ContactPointThis.X,yawMatched.ContactPointThis.Y,0);
    TestTrue(TEXT("yaw matcher accepts exact native-DAG geometry at last clear float"),NativeFloorBoxPlaneDetail::MatchesRefinement(yaw,yawGeometry,yawMatched));
    auto lateYaw=yawMatched;lateYaw.TOI=yawArrival.InsideSeconds;
    TestFalse(TEXT("inside endpoint cannot replace true last-clear native return"),NativeFloorBoxPlaneDetail::MatchesRefinement(yaw,yawGeometry,lateYaw));
    auto wrongYawGeometry=yawGeometry;wrongYawGeometry.Sweep.ClearPrefixSeconds=FloorNativeSweepDetail::Seconds(yawLow-1);
    TestFalse(TEXT("a wider anchored bracket cannot acquire yaw contact values"),NativeFloorBoxPlaneDetail::Build(yaw,wrongYawGeometry,yawDescriptor));
    wrongYawGeometry=yawGeometry;wrongYawGeometry.YawNormSquared.Lower=AnalyticCertified::FromBits(AnalyticCertified::Bits(norm.Lower)-1);
    TestFalse(TEXT("changed yaw norm cannot alias the captured motion model"),NativeFloorBoxPlaneDetail::Build(yaw,wrongYawGeometry,yawDescriptor));
    const auto firstYawSaved=yaw;
    yaw.OwnerState.Velocity.Z=-32;yaw.OwnerState.Acceleration.Z=64;captureFirstYaw();
    TestTrue(TEXT("returning-clear yaw path model builds"),BuildYawFloorBoxMotionModel(yaw,yawModel));
    TestTrue(TEXT("returning yaw final endpoint is clear in both DAG families"),yawModel.BoundGap(1,1,true).Lower>0&&yawModel.BoundGap(1,1,false).Lower>0);
    FDistanceBudget returningYawBudget{8192};const auto returningYaw=BoundYawFloorBoxFirstArrival(yawModel,returningYawBudget);
    TestTrue(TEXT("interior yaw arrival survives clear end-of-frame endpoint"),returningYaw.Status==EFloorNativeSweepStatus::ArrivalBracket&&returningYaw.InsideSeconds<yaw.Horizon);
    yaw=firstYawSaved;yaw.OwnerState.Location.Z=2;yaw.OwnerState.Velocity.Z=-4;yaw.OwnerState.Acceleration.Z=8;
    yaw.LocalCOM=yaw.LocalOffset=FVector::ZeroVector;yaw.LocalRotation=FQuat::Identity;captureFirstYaw();
    TestTrue(TEXT("yaw tangent model builds"),BuildYawFloorBoxMotionModel(yaw,yawModel));
    FDistanceBudget yawTangentBudget{8192};TestTrue(TEXT("uncertain tangent leaf never skipped to a later yaw crossing"),
        BoundYawFloorBoxFirstArrival(yawModel,yawTangentBudget).Status==EFloorNativeSweepStatus::Unknown);
    yaw=firstYawSaved;yaw.OwnerState.AngularVelocity.Z=1.e152;captureFirstYaw();
    TestFalse(TEXT("finite raw quaternion with possible Dot4 overflow cannot use near-one band"),BuildYawFloorBoxMotionModel(yaw,yawModel));
    TestFalse(TEXT("failed tight model clears previous known state"),yawModel.IsKnown());
    yaw=firstYawSaved;captureFirstYaw();TestTrue(TEXT("tight yaw model restored"),BuildYawFloorBoxMotionModel(yaw,yawModel));
    FDistanceBudget noYawBudget{0};TestTrue(TEXT("no budget cannot create yaw first arrival"),BoundYawFloorBoxFirstArrival(yawModel,noYawBudget).Status==EFloorNativeSweepStatus::Unknown);
    // Pure-data fixtures for native readback matchers. NO UBox member or
    // refiner execution, GT binding, operation/force/advance or live profile is
    // represented by these manually assembled observations.
    yaw=firstYawSaved;yaw.OwnerState.Velocity.Z=0;yaw.OwnerState.Acceleration.Z=0;captureFirstYaw();
    const auto nativeBodyQ=yaw.OwnerState.Rotation*yaw.LocalRotation;
    const auto nativeCOM=yaw.OwnerState.Rotation.RotateVector(yaw.LocalCOM);
    const auto nativeOffset=yaw.OwnerState.Rotation.RotateVector(yaw.LocalOffset);
    const auto lowerCorner=nativeBodyQ.RotateVector(-yaw.PhysicalHalfExtent);
    yaw.OwnerState.Location.Z=(nativeCOM.Z-nativeOffset.Z)-lowerCorner.Z;captureFirstYaw();
    FNativeBoxFloorSupportBounds supportBounds;
    TestTrue(TEXT("whole-H per-corner support geometry retains actual local pitch"),BuildNativeBoxFloorSupportBounds(yaw,supportBounds));
    FNativeSolidGeometryValues supportGeometry;supportGeometry.Status=ENativeSolidGeometryStatus::BoxFloorSupportDescriptor;
    supportGeometry.SourceId=1;supportGeometry.SurfaceId=2;supportGeometry.FeatureId=3;supportGeometry.PrimitiveId=4;
    supportGeometry.BoxSupport=supportBounds;supportGeometry.QueryEnvelope=supportBounds.QueryEnvelope;
    supportGeometry.BoxSupport.CachedPlane.Point=FVector(yaw.BodyState.Location.X,yaw.BodyState.Location.Y,0);
    supportGeometry.BoxSupport.CachedPlane.Normal=FVector(0,0,1);
    SHitResult supportDescriptor;TestTrue(TEXT("support supplies descriptor without event"),NativeFloorBoxPlaneDetail::BuildSupportDescriptor(yaw,supportGeometry,supportDescriptor));
    TestFalse(TEXT("support descriptor cannot be a hit"),supportDescriptor.bHit||supportDescriptor.bBlockingHit);
    FNativeBoxPlanarRefinementValues observed;observed.Computed=true;observed.Outcome=ENativeBoxPlanarRefinementOutcome::NoCorner;
    observed.Initial=yaw.OwnerState;observed.StartPose=yaw.OwnerState.Integrate(0);
    observed.StartPose.Location-=observed.StartPose.Rotation.RotateVector(yaw.LocalCOM);
    observed.StartPose.Location+=observed.StartPose.Rotation.RotateVector(yaw.LocalOffset);
    observed.StartPose.Rotation=observed.StartPose.Rotation*yaw.LocalRotation;
    observed.LocalCOM=yaw.LocalCOM;observed.LocalCenter=yaw.LocalOffset;observed.LocalRotation=yaw.LocalRotation;
    observed.Extent=yaw.PhysicalHalfExtent;observed.Normal=supportDescriptor.ImpactNormal;observed.Point=supportDescriptor.ImpactPoint;
    observed.Roundoff=64*DBL_EPSILON*FMath::Max(1.,observed.StartPose.Location.GetAbsMax()+yaw.PhysicalHalfExtent.GetMax());
    observed.SpeedRoundoff=4096*DBL_EPSILON*FMath::Max(1.,yaw.OwnerState.Velocity.Size()+yaw.OwnerState.AngularVelocity.Size()*yaw.PhysicalHalfExtent.Size());
    observed.Established=true;observed.EstablishedMask=uint8((1u<<0)|(1u<<2));observed.Horizon=observed.FirstTime=yaw.Horizon;
    observed.MinimumInitial=DBL_MAX;FVector observedVertices[8];int32 minimumCorner=0;
    for(int32 k=0;k<8;++k)
    {
        FVector local=FVector::ZeroVector;for(int32 d=0;d<3;++d)local[d]=(k&(1<<d))?yaw.PhysicalHalfExtent[d]:-yaw.PhysicalHalfExtent[d];
        observedVertices[k]=observed.StartPose.Location+observed.StartPose.Rotation.RotateVector(local);
        observed.InitialGaps[k]=FVector::DotProduct(observedVertices[k]-observed.Point,observed.Normal);
        observed.InitialNormalSpeeds[k]=FVector::DotProduct(yaw.OwnerState.Velocity+
            FVector::CrossProduct(yaw.OwnerState.AngularVelocity,observedVertices[k]-yaw.OwnerState.Location),observed.Normal);
        if(observed.InitialGaps[k]<observed.MinimumInitial){observed.MinimumInitial=observed.InitialGaps[k];minimumCorner=k;}
    }
    TestTrue(TEXT("native NoCorner data remains distinct from geometric separation"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,supportGeometry,observed));
    auto badSupport=supportGeometry;badSupport.BoxSupport.NativeCornerGap[1].Lower=0;
    TestFalse(TEXT("unmasked indeterminate corner never closes NoCorner"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,badSupport,observed));
    badSupport=supportGeometry;badSupport.BoxSupport.ReferenceCornerGap[1].Lower=-1;
    TestFalse(TEXT("reference unmasked penetration prevents NoCorner proof"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,badSupport,observed));
    badSupport=supportGeometry;badSupport.BoxSupport.NativeCornerGap[0].Lower=-1;
    TestTrue(TEXT("masked future gap is retained without nonpenetration claim"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,badSupport,observed));
    badSupport=supportGeometry;badSupport.BoxSupport.NativeCornerGap[0]={};
    TestFalse(TEXT("incomplete masked band cannot impersonate complete geometry"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,badSupport,observed));
    auto wrongObserved=observed;wrongObserved.Outcome=ENativeBoxPlanarRefinementOutcome::WitnessRejected;
    TestFalse(TEXT("witness rejection is not NoCorner"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,supportGeometry,wrongObserved));
    wrongObserved=observed;wrongObserved.EstablishedMask=0;
    TestFalse(TEXT("empty mask cannot acquire established status"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,supportGeometry,wrongObserved));
    wrongObserved=observed;wrongObserved.InitialEvent=true;
    TestFalse(TEXT("initial event cannot alias NoCorner"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,supportGeometry,wrongObserved));
    wrongObserved=observed;wrongObserved.StartPose.Location.X+=1;
    TestFalse(TEXT("changed actual PoseAt0 is not BodyState substitution"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,supportGeometry,wrongObserved));
    wrongObserved=observed;wrongObserved.FirstTime=0;
    TestFalse(TEXT("NoCorner H initialization is not a zero-time event"),NativeFloorBoxPlaneDetail::MatchesNoNewReadback(yaw,supportGeometry,wrongObserved));
    FNativeSolidGeometryValues initialBox=supportGeometry;initialBox.Status=ENativeSolidGeometryStatus::BoxInitialPlaneDescriptor;
    initialBox.BoxSupport={};SHitResult initialDescriptor;
    TestTrue(TEXT("native initial plane is only a false-hit descriptor"),NativeFloorBoxPlaneDetail::Build(yaw,initialBox,initialDescriptor));
    auto initialObserved=observed;initialObserved.Outcome=ENativeBoxPlanarRefinementOutcome::Hit;
    initialObserved.Established=false;initialObserved.InitialEvent=true;initialObserved.EstablishedMask=0;
    initialObserved.FirstCorner=minimumCorner;initialObserved.FirstTime=0;
    initialObserved.Normal=initialDescriptor.ImpactNormal;initialObserved.Point=initialDescriptor.ImpactPoint;
    auto initialValue=initialDescriptor;initialValue.TOI=0;initialValue.Location=initialObserved.StartPose.Location;
    initialValue.ContactFeatureThis=EContactFeatureKind::Vertex;initialValue.ContactFeatureIndexThis=int8(minimumCorner);
    initialValue.ContactPointThis=observedVertices[minimumCorner];
    initialValue.ContactPointOther=initialValue.ImpactPoint=FVector(initialValue.ContactPointThis.X,initialValue.ContactPointThis.Y,0);
    TestTrue(TEXT("initial matcher joins input DAG and actual zero-time corner geometry"),
        NativeFloorBoxPlaneDetail::MatchesReadbackInputs(yaw,initialDescriptor.ImpactNormal,initialDescriptor.ImpactPoint,initialObserved)&&
        NativeFloorBoxPlaneDetail::MatchesInitialReadback(yaw,initialValue,initialObserved)&&NativeFloorBoxPlaneDetail::MatchesRefinement(yaw,initialBox,initialValue));
    auto deepInitial=initialObserved;deepInitial.MinimumInitial=-2*deepInitial.Roundoff;
    TestFalse(TEXT("deep initial gap cannot be disguised as zero-depth contact"),NativeFloorBoxPlaneDetail::MatchesInitialReadback(yaw,initialValue,deepInitial));
    auto positiveGap=initialObserved;positiveGap.MinimumInitial=2*positiveGap.Roundoff;
    TestFalse(TEXT("finite positive initial gap outside native roundoff is not touch"),NativeFloorBoxPlaneDetail::MatchesInitialReadback(yaw,initialValue,positiveGap));
    auto staleInitial=initialObserved;staleInitial.Established=true;
    TestFalse(TEXT("established event cannot alias unestablished native initial event"),NativeFloorBoxPlaneDetail::MatchesInitialReadback(yaw,initialValue,staleInitial));
    // Mathematical counterexample, not a production pose/profile or a new
    // query lane: clearance from the exact master does NOT close the separate
    // native chord/facet representation. Keep both joins before any Box miss.
    const uint32 facetControls=VectorGetControlRegister();
    {
        ON_SCOPE_EXIT{VectorSetControlRegister(facetControls);};
        for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
        {
            VectorSetControlRegister((facetControls&~uint32(0x8040))|flush);
            // Mathematical fixtures only: the new axial fixed-Box producer
            // shares this EXISTING unit and every supported FTZ/DAZ mode.
            FNativeSolidStaticValues axial;axial.Shape=ENativeSolidQueryShape::Box;axial.Horizon=1;
            axial.OwnerState.Location=FVector(4,0,64);axial.OwnerState.Velocity=FVector(-3,0,0);
            axial.BodyState=axial.OwnerState;axial.PhysicalHalfExtent=FVector(1,1,1);
            FFixedAxialBoxMotionModel axialModel;FDistanceBudget axialWork{8192};
            if(!TestTrue(TEXT("explicit nonzero X plane retains the actual fixed Box DAG"),
                BuildFixedAxialBoxMotionModel(axial,FVector(1,0,64),FVector(1,0,0),axialModel)))return false;
            const auto axialArrival=BoundFixedAxialBoxFirstArrival(axialModel,axialWork);
            TestTrue(TEXT("axial Box first arrival brackets2/3 with no radius or margin"),
                axialArrival.Status==EFloorNativeSweepStatus::ArrivalBracket&&
                double(axialArrival.ClearPrefixSeconds)<2./3.&&double(axialArrival.InsideSeconds)>2./3.&&
                !axialArrival.CapturedSphereGapSatisfied);
            FDistanceBudget emptyAxial{0};
            TestTrue(TEXT("axial exhausted work cannot leave an event"),
                BoundFixedAxialBoxFirstArrival(axialModel,emptyAxial).Status==EFloorNativeSweepStatus::Unknown);
            auto reflected=axial;reflected.OwnerState.Location.X=-4;reflected.OwnerState.Velocity.X=3;reflected.BodyState=reflected.OwnerState;
            FDistanceBudget reflectedWork{8192};
            if(!TestTrue(TEXT("negative X side is explicit rather than inherited from positive Z"),
                BuildFixedAxialBoxMotionModel(reflected,FVector(-1,0,64),FVector(-1,0,0),axialModel)))return false;
            const auto reflectedArrival=BoundFixedAxialBoxFirstArrival(axialModel,reflectedWork);
            TestTrue(TEXT("reflected axial first arrival has the same float bracket"),
                reflectedArrival.Status==EFloorNativeSweepStatus::ArrivalBracket&&
                KinematicCenterDetail::Bits(reflectedArrival.ClearPrefixSeconds)==KinematicCenterDetail::Bits(axialArrival.ClearPrefixSeconds)&&
                KinematicCenterDetail::Bits(reflectedArrival.InsideSeconds)==KinematicCenterDetail::Bits(axialArrival.InsideSeconds));
            auto tangentAxial=axial;tangentAxial.OwnerState.Velocity.X=-8;tangentAxial.OwnerState.Acceleration.X=16;
            tangentAxial.BodyState=tangentAxial.OwnerState;FDistanceBudget axialTangentWork{8192};
            if(!TestTrue(TEXT("axial returning tangent retains acceleration"),
                BuildFixedAxialBoxMotionModel(tangentAxial,FVector(1,0,64),FVector(1,0,0),axialModel)))return false;
            TestTrue(TEXT("first uncertain tangent cannot be skipped or relabelled clear"),
                BoundFixedAxialBoxFirstArrival(axialModel,axialTangentWork).Status==EFloorNativeSweepStatus::Unknown);
            FVector tinyTilt(1,AnalyticCertified::FromBits(1),0);
            TestFalse(TEXT("subnormal tilt is not flushed into an axial normal"),
                BuildFixedAxialBoxMotionModel(axial,FVector(1,0,64),tinyTilt,axialModel));
            TestFalse(TEXT("failed axial builder clears prior motion"),axialModel.IsKnown());
            auto overflow=axial;overflow.OwnerState.Location.Y=std::numeric_limits<double>::max();overflow.BodyState=overflow.OwnerState;
            TestFalse(TEXT("full Dot refuses tangential Inf times zero"),BuildFixedAxialBoxMotionModel(overflow,
                FVector(1,-std::numeric_limits<double>::max(),64),FVector(1,0,0),axialModel));
            FCanonicalAxialPlaneBounds finite;finite.NormalAxis=0;finite.UAxis=1;finite.VAxis=2;
            finite.Plane.Origin=FVector3d(1,0,64);finite.Plane.Normal=FVector3d(1,0,0);
            finite.Plane.AxisU=FVector3d(0,1,0);finite.Plane.AxisV=FVector3d(0,0,1);finite.Plane.HalfExtents=FVector2d(8,8);
            finite.Plane.DomainVertices={FVector2d(-4,-4),FVector2d(4,-4),FVector2d(4,4),FVector2d(-4,4)};
            finite.Triangle.DegreeU=finite.Triangle.DegreeV=1;
            for(const FVector3d cp:{FVector3d(1,-4,60),FVector3d(1,4,64),FVector3d(1,-4,68),FVector3d(1,-4,68)})
                finite.Triangle.ControlPoints.Add(Detail::Point(cp));
            TestTrue(TEXT("canonical plane triangle has exact normal-coordinate singleton"),
                CanonicalAxialPlaneDetail::ExactPlaneTriangle(finite,finite.Triangle));
            auto nonDyadic=finite.Triangle;nonDyadic.ControlPoints[0].Axis[0].Upper=AnalyticCertified::FromBits(0x3ff0000000000001ULL);
            TestFalse(TEXT("non-singleton canonical normal interval is not the rounded native plane"),
                CanonicalAxialPlaneDetail::ExactPlaneTriangle(finite,nonDyadic));
            // Positive fixed-Sphere native query fixture. q.Radius is the
            // rounded FCollisionShape radius, not an unrounded wheel Radius().
            finite.Plane.SourceId=1;finite.Plane.SurfaceId=2;finite.Plane.FeatureId=3;finite.Plane.PrimitiveId=4;
            finite.Plane.ObjectType=uint32(ECC_WorldStatic);finite.Plane.BlockingChannels=MAX_uint64;
            finite.Plane.bQueryCollisionEnabled=true;finite.Plane.bAuthorityEligible=false;
            FWorldQuery probe;probe.Shape=EQueryShape::Sphere;probe.Radius=double(float(.1));
            probe.Start=FVector3d(2+probe.Radius,0,64);probe.End=FVector3d(probe.Radius,0,64);
            probe.bAuthorityOnly=true;probe.bApplyCollisionFilter=true;probe.TraceChannel=uint8(ECC_PhysicsBody);
            probe.DomainTolerance=1.e-6;probe.InitialOverlapTolerance=.05;
            FWorldHit probeHit;FIntervalPoint probeEnvelope;FDistanceBudget probeWork{256};
            TestTrue(TEXT("probe retains rounded shape radius rather than decimal surrogate"),
                AnalyticCertified::Bits(probe.Radius)!=AnalyticCertified::Bits(.1));
            if(!TestTrue(TEXT("same native finite kernel gives the real positive Sphere probe"),
                BuildFixedWorldQueryVolumeBounds(probe,probeEnvelope)&&
                FWorldQueryService::EvaluateFinitePlaneGeometry(probe,finite.Plane,false,probeHit)&&
                probeHit.bHit&&!probeHit.bStartPenetrating&&probeHit.Time==.5))return false;
            TestTrue(TEXT("native probe keeps actual normalized-time Lerp center and radius witness"),
                probeHit.Location==FMath::Lerp(probe.Start,probe.End,probeHit.Time)&&
                probeHit.QueryPoint==probeHit.Location-probe.Radius*probeHit.Normal&&
                probeHit.Point==finite.Plane.ClosestPoint(probeHit.QueryPoint));
            TestTrue(TEXT("full positive Sphere footprint lies in the separate finite interiors"),
                CanonicalAxialPlaneDetail::TriangleInterior(finite,probeEnvelope)&&
                CanonicalAxialPlaneDetail::NativeInterior(finite,probeEnvelope,probeWork));
            TestTrue(TEXT("exact planar provider introduces no curved geometry or eligibility credit"),
                probeHit.GeometricErrorBoundCm==0&&!probeHit.bSurfaceNormalMayVary&&
                probe.bAuthorityOnly&&!finite.Plane.bAuthorityEligible);
            auto zeroProbe=probe;zeroProbe.Radius=0;FWorldHit zeroProbeHit;
            TestFalse(TEXT("zero-radius sphere cannot borrow the fixed wheel probe path"),
                FWorldQueryService::EvaluateFinitePlaneGeometry(zeroProbe,finite.Plane,false,zeroProbeHit));
            finite.Complete=true;FDistanceBudget finiteWork{256};
            TestTrue(TEXT("finite canonical and native interiors are proved separately"),
                CanonicalAxialPlaneContains(finite,FVector3d(1,0,64),finiteWork));
            TestFalse(TEXT("canonical triangle boundary is never an infinite plane contact"),
                CanonicalAxialPlaneContains(finite,FVector3d(1,-4,64),finiteWork));
            TestFalse(TEXT("native rounded witness must retain exact canonical normal coordinate"),
                CanonicalAxialPlaneContains(finite,FVector3d(1.0001,0,64),finiteWork));
            finite.Plane.DomainVertices={FVector2d(-4,-4),FVector2d(4,-4),FVector2d(0,0),FVector2d(4,4),FVector2d(-4,4)};
            TestFalse(TEXT("concave native domain is not inferred convex from local turns"),
                CanonicalAxialPlaneDetail::NativeInterior(finite,Detail::Point(FVector3d(1,0,64)),finiteWork));
            FString assignmentReason;
            const bool assignmentMatched=TestOrdinarySetFutureHitValueClosure(assignmentReason);
            if(!TestTrue(FString::Printf(TEXT("all-owner plain setter footprint value counterexamples: %s"),*assignmentReason),
                assignmentMatched))return false;
            FIntervalTensorSurface curved;curved.DegreeU=2;curved.DegreeV=1;
            // x=4u-2, y=(2u-1)^2, z=2v-1/2. The endpoint chord is y=1.
            for(const FVector3d cp:{FVector3d(-2,1,-.5),FVector3d(-2,1,1.5),
                FVector3d(0,-1,-.5),FVector3d(0,-1,1.5),FVector3d(2,1,-.5),FVector3d(2,1,1.5)})
                curved.ControlPoints.Add(Detail::Point(cp));
            FWorldQuery query;query.Shape=EQueryShape::Box;query.bAuthorityOnly=true;
            query.bApplyCollisionFilter=true;query.TraceChannel=uint8(ECC_PhysicsBody);
            query.bUseFiniteContactDomain=true;query.DomainTolerance=0;query.InitialOverlapTolerance=0;
            query.Start=FVector3d(0,1.5,.5);query.End=FVector3d(0,.75,.5);query.HalfExtent=FVector3d(.25,.125,.25);
            FNativeSolidStaticValues native;native.Shape=ENativeSolidQueryShape::Box;native.Horizon=1;
            native.Query=query;native.Start=query.Start;native.End=query.End;
            native.OwnerState.Location=query.Start;native.OwnerState.Velocity=query.End-query.Start;
            native.OwnerState.Rotation=FQuat::Identity;native.BodyState=native.OwnerState;
            native.LocalRotation=FQuat::Identity;native.PhysicalHalfExtent=native.QueryHalfExtent=query.HalfExtent;
            FFloorNativeSolidModel physical;FIntervalPoint envelope,coarse;
            if(!TestTrue(TEXT("counterexample retains complete native/reference/captured/query Box volume"),
                BuildNativeSolidSweepVolume(native,physical,envelope)&&CandidateDetail::SurfaceHull(curved,coarse)))return false;
            const auto boxDistance=CandidateDetail::BoxDistanceSquared(envelope,coarse);
            bool coarseOverlap=true;
            for(int32 axis=0;axis<3;++axis)
                coarseOverlap&=envelope.Axis[axis].Lower<=coarse.Axis[axis].Upper&&
                    coarse.Axis[axis].Lower<=envelope.Axis[axis].Upper;
            // Outward-rounded addition of exact zero gaps may have a negative
            // lower bound. Assert closed overlap and enclosure, not Lower==0.
            TestTrue(TEXT("coarse CP box overlaps despite exact-curve clearance"),
                coarseOverlap&&AnalyticCertified::Valid(boxDistance)&&boxDistance.Lower<=0&&boxDistance.Upper>=0);
            FDistanceBudget spatial{256};const auto threshold=Interval::Exact(0);
            const auto exact=BoundTensorDistanceSquaredIntervalCenter(curved,envelope,spatial,0,spatial.Remaining,&threshold);
            if(!TestTrue(TEXT("subdivision proves positive distance to the entire exact master patch"),
                exact.Status!=EDistanceBoundStatus::Unknown&&AnalyticCertified::Finite(exact.LowerSquared)&&exact.LowerSquared>0))return false;
            TArray<FIntervalTensorSurface> provided;provided.Add(curved);
            FDistanceBudget clearWork{1024};FIndexedIntervalUnionPtr whole;
            FReferenceBoundsCandidateSetPtr partition;FReferenceBoundsStrictSeparationPtr clear;
            if(!TestTrue(TEXT("distinct canonical Box operation refines the complete retained volume"),
                FIndexedIntervalUnion::Build(MakeArrayView(provided),clearWork,whole)&&
                FReferenceBoundsCandidateSet::Select(whole,envelope,clearWork,partition)&&
                FReferenceBoundsStrictSeparation::TryProve(partition,clearWork,clear)&&clear&&clear->Matches(whole,envelope)))return false;
            TestTrue(TEXT("all coarse candidates retain universal positive lower bounds"),
                clear->GetPartition()->GetCandidateIndices().Num()==1&&clear->GetRefinedProviders().Num()==1&&
                clear->GetRefinedProviders()[0].Bounds.LowerSquared>0);
            if(!TestTrue(TEXT("partition borrowed from sole output remains owned across reset"),
                FReferenceBoundsStrictSeparation::TryProve(clear->GetPartition(),clearWork,clear)&&clear))return false;
            auto altered=envelope;altered.Axis[0].Upper+=1;
            TestFalse(TEXT("different swept volume cannot reuse canonical clearance"),clear->Matches(whole,altered));
            FDistanceBudget noClearWork{0};
            TestFalse(TEXT("exhausted work clears previous canonical clearance"),FReferenceBoundsStrictSeparation::TryProve(partition,noClearWork,clear));
            TestFalse(TEXT("no stale canonical volume proof"),bool(clear));
            auto touching=envelope;touching.Axis[1]={-.125,0,true};
            FDistanceBudget touchWork{128};FReferenceBoundsCandidateSetPtr touchPartition;
            if(!TestTrue(TEXT("touching curve volume retains a complete candidate partition"),
                FReferenceBoundsCandidateSet::Select(whole,touching,touchWork,touchPartition)))return false;
            TestFalse(TEXT("non-strict master contact is never a canonical clear path"),
                FReferenceBoundsStrictSeparation::TryProve(touchPartition,touchWork,clear));
            TestFalse(TEXT("touching refusal leaves no output"),bool(clear));
            FIntervalTensorSurface obstructing;obstructing.DegreeU=obstructing.DegreeV=1;
            for(const FVector3d cp:{FVector3d(-2,1,-.5),FVector3d(-2,1,1.5),FVector3d(2,1,-.5),FVector3d(2,1,1.5)})
                obstructing.ControlPoints.Add(Detail::Point(cp));
            provided.Add(obstructing);FDistanceBudget obstructedWork{256};FIndexedIntervalUnionPtr obstructed;
            FReferenceBoundsCandidateSetPtr obstructedPartition;
            if(!TestTrue(TEXT("additional intersecting provider remains in the complete union"),
                FIndexedIntervalUnion::Build(MakeArrayView(provided),obstructedWork,obstructed)&&
                FReferenceBoundsCandidateSet::Select(obstructed,envelope,obstructedWork,obstructedPartition)))return false;
            TestFalse(TEXT("one clear provider never hides a later contact"),
                FReferenceBoundsStrictSeparation::TryProve(obstructedPartition,obstructedWork,clear));
            TestFalse(TEXT("partial all-provider scan publishes no clearance"),bool(clear));
            FBoundedPlane chord;chord.Origin=FVector3d(0,1,.5);chord.AxisU=FVector3d(1,0,0);
            chord.AxisV=FVector3d(0,0,1);chord.Normal=FVector3d(0,-1,0);chord.HalfExtents=FVector2d(2,1);
            chord.DomainVertices={FVector2d(-2,-1),FVector2d(2,-1),FVector2d(2,1),FVector2d(-2,1)};
            chord.SourceId=1;chord.SurfaceId=2;chord.FeatureId=3;chord.PrimitiveId=4;
            chord.ObjectType=uint32(ECC_WorldStatic);chord.BlockingChannels=MAX_uint64;
            chord.bQueryCollisionEnabled=true;chord.bAuthorityEligible=false;
            FWorldHit chordHit;
            if(!TestTrue(TEXT("the separate finite chord kernel still sees a Box contact"),
                FWorldQueryService::EvaluateFinitePlaneGeometry(query,chord,false,chordHit)&&chordHit.bHit))return false;
            TestTrue(TEXT("geometry-only counterexample changes neither authority query nor provider eligibility"),
                query.bAuthorityOnly&&!chord.bAuthorityEligible);
            // No World request, Source binding, compiled phase, impulse,
            // ordinary miss or readiness is issued by either mathematical fact.
        }
    }
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}

#include "YawFloorBoxSweepBounds.h"
#include "IAmSpeed/Base/SUtils.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGeneralNativeBoxEnvelopeTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.GeneralRotationCheckedEnvelope",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGeneralNativeBoxEnvelopeTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic::Certified;using namespace GeneralBoxDetail;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        TestTrue(TEXT("zero actual exponential accepted"),CheckedNativeExpResults(0,.5,1));
        TestFalse(TEXT("outward native cosine rejected"),CheckedNativeExpResults(0,.5,AnalyticCertified::FromBits(AnalyticCertified::Bits(1.)+1)));
        TestFalse(TEXT("actual invalid scale rejected"),CheckedNativeExpResults(0,.6,1));
        for(double bad:{AnalyticCertified::FromBits(0x8000000000000001ULL),
            std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()})
            TestFalse(TEXT("negative raw subnormal/NaN/Inf never certified"),CheckedNativeExpResults(bad,.5,1));
        const double threshold=1.e-8;
        for(const Interval range:{Interval{0,AnalyticCertified::FromBits(AnalyticCertified::MinNormal),true},
            Interval{AnalyticCertified::FromBits(AnalyticCertified::Bits(threshold)-1),AnalyticCertified::FromBits(AnalyticCertified::Bits(threshold)+1),true},
            Interval{0,1,true}})
        {
            Interval scaleRange,cosRange;
            if(!TestTrue(TEXT("whole scalar range bounded"),TrigPolynomialBounds(range,scaleRange,cosRange)))return false;
            for(double x:{range.Lower,range.Upper})
            {
                Interval scale,cosine;
                if(!TestTrue(TEXT("point certificate bounded"),TrigPolynomialBounds(Interval::Exact(x),scale,cosine)))return false;
                TestTrue(TEXT("point scalar certificate is contained in range certificate"),
                    scale.Lower>=scaleRange.Lower&&scale.Upper<=scaleRange.Upper&&cosine.Lower>=cosRange.Lower&&cosine.Upper<=cosRange.Upper);
                const double angle=FMath::Sqrt(x);
                const double actualScale=x<threshold?.5-x/48.+x*x/3840.:FMath::Sin(angle*.5)/angle;
                TestTrue(TEXT("actual native scalar values are checked"),CheckedNativeExpResults(x,actualScale,FMath::Cos(angle*.5)));
            }
        }
        Interval unusedA,unusedB;
        TestFalse(TEXT("unsupported x successor is never clipped to domain"),
            TrigPolynomialBounds(Interval{0,AnalyticCertified::FromBits(AnalyticCertified::Bits(1.)+1),true},unusedA,unusedB));
        FGeneralBoxMotionInput input;input.Horizon=1.f/300.f;
        input.Owner.Location=FVector(13,-9,70);input.Owner.Rotation=FQuat(FVector::UpVector,.7);
        input.Owner.Velocity=FVector(12,-3,2);input.Owner.Acceleration=FVector(-7,4,-650);
        input.Owner.AngularVelocity=FVector(.4,-.25,.7);input.Owner.AngularAcceleration=FVector(.05,.1,-.2);
        input.LocalCOM=FVector(2,-3,1);input.LocalOffset=FVector(7,1,-2);input.LocalRotation=FQuat(FVector::RightVector,-.55);
        input.PhysicalHalfExtent=FVector(65.66,43.5852,15.9472);
        input.QueryHalfExtent=FVector(float(65.66),float(43.5852),float(15.9472));
        FGeneralBoxMotionBounds bounds;
        if(!TestTrue(TEXT("nonzero COM/offset/local pitch/full angular axes are bounded"),BuildGeneralBoxMotionBounds(input,bounds)))return false;
        for(float t:{0.f,input.Horizon*.125f,input.Horizon*.5f,input.Horizon})
        {
            const auto actual=input.Owner.Integrate(t);
            const FVector center=(actual.Location-actual.Rotation.RotateVector(input.LocalCOM))+actual.Rotation.RotateVector(input.LocalOffset);
            const FQuat rotation=actual.Rotation*input.LocalRotation;
            for(const auto& extent:{input.PhysicalHalfExtent,input.QueryHalfExtent})for(int32 corner=0;corner<8;++corner)
            {
                FVector local;for(int32 d=0;d<3;++d)local[d]=(corner&(1<<d))?extent[d]:-extent[d];
                const auto point=center+rotation.RotateVector(local);
                for(int32 d=0;d<3;++d)TestTrue(TEXT("actual native corner contained"),Contains(bounds.Hull.Axis[d],point[d]));
            }
        }
        FKinematicState early;early.Rotation=FQuat(0,0,0,AnalyticCertified::FromBits(AnalyticCertified::Bits(1.)+1));
        YawFloorBoxDetail::FQuaternion earlyFamily;
        if(!TestTrue(TEXT("existing zero-theta early return is enclosed"),RotationFamily(early,input.Horizon,true,earlyFamily)))return false;
        TestTrue(TEXT("captured slightly greater than unit W is retained"),Contains(Expanded(earlyFamily.Axis[3]),early.Rotation.W));
        const auto zero=YawFloorBoxDetail::Quaternion(FQuat(0,0,0,0));YawFloorBoxDetail::FQuaternion normalized;
        TestFalse(TEXT("normalization identity fallback is refused by theorem"),NormalizedFamily(zero,FQuat::Identity,normalized));
        const auto overflow=YawFloorBoxDetail::Quaternion(FQuat(1.e308,1.e308,1.e308,1.e308));
        TestFalse(TEXT("normalization overflow is refused"),NormalizedFamily(overflow,FQuat::Identity,normalized));
    }
    TestTrue(TEXT("floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}

#include "CanonicalBoxOverlapBounds.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalFixedBoxOverlapBoundsTest,
 "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalFixedBoxOverlapClearance",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalFixedBoxOverlapBoundsTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        FWorldQuery q;q.Shape=EQueryShape::Box;q.Start=q.End=FVector3d(4841.4506261183424,1232.1100759931123,30.704558110447195);
        q.Rotation=FQuat4d(-9.4877524127911624e-06,0.0037871005545362366,0.067756379714798115,0.99769470820853645);
        q.HalfExtent=FVector3d(65.660003662109375,43.585201263427734,15.947199821472168);
        q.DomainTolerance=1.e-6;q.InitialOverlapTolerance=.05;q.bAuthorityOnly=true;q.bApplyCollisionFilter=true;
        FIntervalPoint hull;if(!TestTrue(TEXT("actual N29 rounded stationary Box query enclosed"),BuildCanonicalFixedBoxOverlapEnvelope(q,hull)))return false;
        for(int32 v=0;v<8;++v)
        {
            FVector3d corner;for(int32 d=0;d<3;++d)corner[d]=(v&(1<<d))?q.HalfExtent[d]:-q.HalfExtent[d];
            const auto point=q.Start+q.Rotation.RotateVector(corner);
            for(int32 d=0;d<3;++d)TestTrue(TEXT("actual rotated corner and declared shell enclosed"),
                point[d]-.050001>=hull.Axis[d].Lower&&point[d]+.050001<=hull.Axis[d].Upper);
        }
        for(int32 bad=0;bad<7;++bad)
        {
            auto changed=q;
            switch(bad){case 0:changed.Shape=EQueryShape::Sphere;break;case 1:changed.End.X+=1;break;
            case 2:changed.InitialOverlapTolerance=-1;break;case 3:changed.DomainTolerance=std::numeric_limits<double>::infinity();break;
            case 4:changed.Rotation=FQuat4d(0,0,0,0);break;case 5:changed.bAuthorityOnly=false;break;
            case 6:changed.bExcludeAuthorityEligible=true;break;}
            FIntervalPoint rejected=hull;TestFalse(TEXT("unsupported pose cannot inherit previous volume"),BuildCanonicalFixedBoxOverlapEnvelope(changed,rejected));
            TestFalse(TEXT("refusal clears prior pose volume"),Detail::ValidPoint(rejected));
        }
        // An entire plane crossing just the tolerance shell is NOT clear.
        FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        const double z=hull.Axis[2].Upper;
        for(int32 v=0;v<4;++v)plane.ControlPoints.Add(Detail::Point(FVector3d(
            (v&2)?hull.Axis[0].Upper+1:hull.Axis[0].Lower-1,
            (v&1)?hull.Axis[1].Upper+1:hull.Axis[1].Lower-1,z)));
        TArray<FIntervalTensorSurface> surfaces{plane};FIndexedIntervalUnionPtr scene;
        FReferenceBoundsCandidateSetPtr partition;FReferenceBoundsStrictSeparationPtr proof;FDistanceBudget work{2048};
        if(!TestTrue(TEXT("complete shell-contact fixture retained"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),work,scene)&&
            FReferenceBoundsCandidateSet::Select(scene,hull,work,partition)))return false;
        TestFalse(TEXT("shell tangency never produces empty canonical penetration proof"),FReferenceBoundsStrictSeparation::TryProve(partition,work,proof));
        for(auto& cp:surfaces[0].ControlPoints)cp.Axis[2]=Interval::Exact(z+1);
        work={2048};
        if(!TestTrue(TEXT("strictly clear complete plane fixture proved"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),work,scene)&&
            FReferenceBoundsCandidateSet::Select(scene,hull,work,partition)&&FReferenceBoundsStrictSeparation::TryProve(partition,work,proof)))return false;
        TestTrue(TEXT("proof joins exact retained union and query shell"),proof&&proof->Matches(scene,hull));
        auto other=hull;other.Axis[0].Upper+=1;TestFalse(TEXT("other pose envelope cannot consume this proof"),proof->Matches(scene,other));
    }
    // Regression for the actual Fast2 query outside the finite floor.
    // Synthetic coefficient structure only: no actor, authenticated stadium,
    // World capture, physical profile or native acceptance is manufactured.
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        using namespace Speed;using namespace Speed::PreparedExecution;
        FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(int32 u=0;u<2;++u)for(int32 v=0;v<2;++v)
            plane.ControlPoints.Add(Speed::Analytic::Certified::Detail::Point(FVector3d(u?1:-1,v?1:-1,0)));
        FDistanceBudget fixture{65536};FPreparedGeometrySelectionPtr geometry;
        if(!TestTrue(TEXT("finite retained unit floor structure"),FContactUnitRetainedProducer::Build(plane,fixture,geometry)))return false;
        FNativeSolidStaticValues frozen;frozen.Shape=ENativeSolidQueryShape::Box;frozen.Frozen=true;frozen.Horizon=1.f/300.f;
        frozen.OwnerState.Location=FVector(-5,7500,32.809999952316289);frozen.OwnerState.Rotation=FQuat::Identity;
        frozen.BodyState=frozen.OwnerState;frozen.LocalRotation=FQuat::Identity;
        frozen.PhysicalHalfExtent=frozen.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
        frozen.Start=frozen.End=frozen.BodyState.Location;frozen.Query.Shape=EQueryShape::Box;
        frozen.Query.Start=frozen.Query.End=frozen.BodyState.Location;frozen.Query.Rotation=FQuat::Identity;
        frozen.Query.HalfExtent=frozen.QueryHalfExtent;frozen.Query.DomainTolerance=1.e-6;frozen.Query.InitialOverlapTolerance=.05;
        FNativeCanonicalBoxSweepClearanceValuesPtr clear;FString why;FDistanceBudget work{65536};
        if(!TestTrue(TEXT("outside finite floor frozen Box uses complete retained union"),BuildNativeCanonicalBoxSweepClearanceValues(geometry,frozen,work,clear,why)))return false;
        TestTrue(TEXT("frozen clearance retains original capture without general transport"),clear&&!clear->RequiresGeneralBoxTransport()&&
            MatchesNativeCanonicalBoxSweepClearanceValues(frozen,geometry,clear,clear->GetEnvelopeForInspection()));
        auto nonfrozen=frozen;nonfrozen.Frozen=false;
        TestFalse(TEXT("frozen payload cannot be consumed as nonfrozen"),MatchesNativeCanonicalBoxSweepClearanceValues(nonfrozen,geometry,clear,clear->GetEnvelopeForInspection()));
        for(int32 index=0;index<8;++index)for(double speed:{1.,std::numeric_limits<double>::denorm_min()})
        {
            auto changed=frozen;FVector* motion[]={&changed.OwnerState.Velocity,&changed.OwnerState.Acceleration,
                &changed.OwnerState.AngularVelocity,&changed.OwnerState.AngularAcceleration,&changed.BodyState.Velocity,
                &changed.BodyState.Acceleration,&changed.BodyState.AngularVelocity,&changed.BodyState.AngularAcceleration};
            motion[index]->X=speed;auto rejected=clear;FDistanceBudget attempt{65536};
            TestFalse(TEXT("each frozen motion coefficient including DAZ subnormal refuses"),BuildNativeCanonicalBoxSweepClearanceValues(geometry,changed,attempt,rejected,why));
            TestFalse(TEXT("frozen refusal clears an older proof"),rejected.IsValid());
        }
        for(int32 invalid=0;invalid<3;++invalid)
        {
            auto changed=frozen;if(invalid==0)changed.Prescribed=true;else if(invalid==1)changed.Query.Start.X+=1;else changed.Query.End.X+=1;
            auto rejected=clear;FDistanceBudget attempt{65536};
            TestFalse(TEXT("prescribed or changed chord is not frozen stationary clearance"),BuildNativeCanonicalBoxSweepClearanceValues(geometry,changed,attempt,rejected,why));
        }
        auto emptyResult=clear;FDistanceBudget empty{0};
        TestFalse(TEXT("frozen whole union requires the existing finite budget"),BuildNativeCanonicalBoxSweepClearanceValues(geometry,frozen,empty,emptyResult,why));
        auto overlapping=frozen;overlapping.OwnerState.Location=FVector(0,0,.5);overlapping.BodyState=overlapping.OwnerState;
        overlapping.Start=overlapping.End=overlapping.Query.Start=overlapping.Query.End=overlapping.BodyState.Location;
        auto overlapMath=overlapping;overlapMath.Frozen=false;FFloorNativeSolidModel overlapModel;FIntervalPoint overlapHull;
        if(!TestTrue(TEXT("overlap negative reaches a valid fixed geometric model"),BuildNativeSolidSweepVolume(overlapMath,overlapModel,overlapHull)))return false;
        FDistanceBudget overlapWork{65536};FNativeCanonicalBoxSweepClearanceValuesPtr rejected;
        TestFalse(TEXT("frozen overlap cannot become a clear Box"),BuildNativeCanonicalBoxSweepClearanceValues(geometry,overlapping,overlapWork,rejected,why));
        FPreparedGeometrySelectionPtr blocked;FDistanceBudget otherFixture{65536};
        if(!TestTrue(TEXT("second retained provider fixture"),FContactUnitRetainedProducer::Build(plane,otherFixture,blocked,true,frozen.BodyState.Location.Z)))return false;
        auto near=frozen;near.OwnerState.Location.X=near.OwnerState.Location.Y=0;near.BodyState=near.OwnerState;
        near.Start=near.End=near.Query.Start=near.Query.End=near.BodyState.Location;FDistanceBudget blockerWork{65536};
        TestFalse(TEXT("an interior second provider cannot be ignored by floor-only reasoning"),BuildNativeCanonicalBoxSweepClearanceValues(blocked,near,blockerWork,rejected,why));
        // The final projector visits a circumsphere, independently of the
        // already-closed Box sweep. Prove this actual broad volume separately.
        auto broad=frozen.Query;broad.Shape=EQueryShape::Sphere;
        broad.Radius=frozen.PhysicalHalfExtent.Size()+64*DBL_EPSILON*FMath::Max(1.,frozen.Query.Start.GetAbsMax()+frozen.PhysicalHalfExtent.GetMax());
        FIntervalPoint broadHull;
        if(!TestTrue(TEXT("projector circumsphere fixed query enclosed"),BuildFixedWorldQueryVolumeBounds(broad,broadHull)))return false;
        for(double shell:{broad.DomainTolerance,broad.InitialOverlapTolerance})
            for(int32 d=0;d<3;++d)broadHull.Axis[d]=AnalyticCertified::Add(broadHull.Axis[d],Interval{-shell,shell,true});
        FReferenceBoundsCandidateSetPtr emptyPartition;FDistanceBudget visitorWork{65536};
        TestTrue(TEXT("outside finite floor projector has complete empty retained partition"),
            FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),broadHull,visitorWork,emptyPartition)&&
            emptyPartition&&emptyPartition->GetScene()==geometry->GetSelectedUnion()&&emptyPartition->GetCandidateIndices().IsEmpty());
        FDistanceBudget noVisitorBudget{0};FReferenceBoundsCandidateSetPtr noPartition;
        TestFalse(TEXT("empty visitor cannot bypass a finite work budget"),FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),broadHull,noVisitorBudget,noPartition));
        auto intersecting=broad;intersecting.Start=intersecting.End=FVector3d(0,0,.5);FIntervalPoint intersectionHull;
        if(!TestTrue(TEXT("intersecting projector query enclosed"),BuildFixedWorldQueryVolumeBounds(intersecting,intersectionHull)))return false;
        FDistanceBudget intersectionWork{65536};FReferenceBoundsCandidateSetPtr nonempty;
        if(!TestTrue(TEXT("intersecting complete projector partition constructed"),FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),intersectionHull,intersectionWork,nonempty)))return false;
        TestFalse(TEXT("intersecting provider cannot become empty visitor enumeration"),nonempty->GetCandidateIndices().IsEmpty());
        auto shellOnly=intersectionHull;for(int32 d=0;d<3;++d)shellOnly.Axis[d]=Interval::Exact(d==2?.5:0.);
        shellOnly.Axis[2]=AnalyticCertified::Add(shellOnly.Axis[2],Interval{-.5,.5,true});
        FDistanceBudget shellWork{65536};FReferenceBoundsCandidateSetPtr tangent;
        if(!TestTrue(TEXT("declared shell tangency partition constructed"),FReferenceBoundsCandidateSet::Select(geometry->GetSelectedUnion(),shellOnly,shellWork,tangent)))return false;
        TestFalse(TEXT("shell-only tangency cannot become empty enumeration"),tangent->GetCandidateIndices().IsEmpty());
    }
    // Actual Fast6 support-query geometry after the test car's frame60 thaw.
    // Pure coefficient fixtures only: no actor/profile/World acceptance.
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        FWorldQuery box;box.Shape=EQueryShape::Box;box.Start=box.End=FVector3d(-5,7500,32.809999952316289);
        box.Rotation=FQuat4d::Identity;box.HalfExtent=FVector3d(65.66,43.5852,15.9472);
        box.DomainTolerance=box.InitialOverlapTolerance=0;box.TraceChannel=1;box.BlockingObjectTypes=1;
        box.bAuthorityOnly=box.bApplyCollisionFilter=true;box.bIncludeCompactPatches=true;
        FWorldQuery probes[4];
        const auto capture=[&](const FWorldQuery& pose)
        {
            for(int32 i=0;i<4;++i)
            {
                const FVector3d corner=pose.Start+pose.Rotation.RotateVector(FVector3d(
                    (i&1)?pose.HalfExtent.X:-pose.HalfExtent.X,(i&2)?pose.HalfExtent.Y:-pose.HalfExtent.Y,-pose.HalfExtent.Z));
                probes[i]=pose;probes[i].Shape=EQueryShape::Ray;probes[i].HalfExtent=FVector3d::ZeroVector;
                probes[i].Start=corner+FVector3d(0,0,.01);probes[i].End=corner-FVector3d(0,0,.01);
            }
        };
        capture(box);FIntervalPoint combined;
        if(!TestTrue(TEXT("actual double-extent support pose and four real ray volumes enclosed"),
            BuildCanonicalFixedBoxSupportProbeEnvelope(box,MakeArrayView(probes),combined)))return false;
        for(const auto& ray:probes)for(const auto& point:{ray.Start,ray.End})for(int32 d=0;d<3;++d)
            TestTrue(TEXT("captured native ray endpoints lie inside compound envelope"),point[d]>=combined.Axis[d].Lower&&point[d]<=combined.Axis[d].Upper);
        FIntervalTensorSurface plane;plane.DegreeU=plane.DegreeV=1;
        for(int32 u=0;u<2;++u)for(int32 v=0;v<2;++v)plane.ControlPoints.Add(Detail::Point(FVector3d(u?2:-2,v?2:-2,0)));
        TArray<FIntervalTensorSurface> surfaces{plane};FIndexedIntervalUnionPtr scene;FDistanceBudget work{65536};
        FReferenceBoundsCandidateSetPtr partition;FReferenceBoundsStrictSeparationPtr proof;
        if(!TestTrue(TEXT("complete unit coefficient scene and off-field support exclusion"),
            FIndexedIntervalUnion::Build(MakeArrayView(surfaces),work,scene)&&
            FReferenceBoundsCandidateSet::Select(scene,combined,work,partition)&&
            FReferenceBoundsStrictSeparation::TryProve(partition,work,proof)))return false;
        TestTrue(TEXT("support exclusion remains exact geometry only"),proof&&proof->Matches(scene,combined));
        for(int32 bad=0;bad<8;++bad)
        {
            capture(box);auto changed=box;
            switch(bad){case 0:probes[0].Shape=EQueryShape::Sphere;break;case 1:probes[0].TraceChannel=2;break;
            case 2:probes[0].RequiredSurfaceId=1;break;case 3:probes[0].bExcludeTori=true;break;
            case 4:probes[0].ReferenceNormal.X=1;break;case 5:probes[0].DomainTolerance=1;break;
            case 6:changed.End.X+=1;break;case 7:changed.InitialOverlapTolerance=AnalyticCertified::FromBits(0x8000000000000001ULL);break;}
            auto rejected=combined;
            TestFalse(TEXT("other ray policy/shape or malformed parent cannot inherit support exclusion"),
                BuildCanonicalFixedBoxSupportProbeEnvelope(changed,MakeArrayView(probes),rejected));
            TestFalse(TEXT("refused support probe envelope clears prior output"),Detail::ValidPoint(rejected));
        }
        capture(box);FIntervalPoint rejected;
        TestFalse(TEXT("four actual support probes are mandatory"),BuildCanonicalFixedBoxSupportProbeEnvelope(box,MakeArrayView(probes,3),rejected));
        FDistanceBudget none{0};FReferenceBoundsCandidateSetPtr noPartition;
        TestFalse(TEXT("compound support exclusion cannot refill a spent budget"),FReferenceBoundsCandidateSet::Select(scene,combined,none,noPartition));
        // A positive gap within the pre-existing roundoff allowance can still
        // support the box: actual +/-0.01 rays cross the plane. Box-only clear
        // must therefore NEVER skip these probes.
        box.HalfExtent=FVector3d(1,1,1);box.Start=box.End=FVector3d(0,0,1.+32.*DBL_EPSILON*2.);
        capture(box);FIntervalPoint poseOnly;
        if(!TestTrue(TEXT("near-roundoff support volumes constructed"),BuildCanonicalFixedBoxOverlapEnvelope(box,poseOnly)&&
            BuildCanonicalFixedBoxSupportProbeEnvelope(box,MakeArrayView(probes),combined)))return false;
        work={65536};
        if(!TestTrue(TEXT("box-only positive-gap fixture really is strictly clear"),FReferenceBoundsCandidateSet::Select(scene,poseOnly,work,partition)&&
            FReferenceBoundsStrictSeparation::TryProve(partition,work,proof)))return false;
        work={65536};if(!TestTrue(TEXT("real support-ray crossing partition constructed"),FReferenceBoundsCandidateSet::Select(scene,combined,work,partition)))return false;
        TestFalse(TEXT("real ray tangency/crossing prevents premature no-support result"),FReferenceBoundsStrictSeparation::TryProve(partition,work,proof));
        box.Start=box.End=FVector3d(0,0,3);capture(box);work={65536};
        if(!TestTrue(TEXT("all four raised probes initially strictly clear"),BuildCanonicalFixedBoxSupportProbeEnvelope(box,MakeArrayView(probes),combined)&&
            FReferenceBoundsCandidateSet::Select(scene,combined,work,partition)&&FReferenceBoundsStrictSeparation::TryProve(partition,work,proof)))return false;
        probes[0].End=FVector3d(0,0,-2);work={65536};
        if(!TestTrue(TEXT("one extended actual support ray remains part of envelope"),BuildCanonicalFixedBoxSupportProbeEnvelope(box,MakeArrayView(probes),combined)&&
            FReferenceBoundsCandidateSet::Select(scene,combined,work,partition)))return false;
        TestFalse(TEXT("one intersecting support ray cannot be discarded"),FReferenceBoundsStrictSeparation::TryProve(partition,work,proof));
        capture(box);for(auto& cp:surfaces[0].ControlPoints)cp.Axis[2]=Interval::Exact(3.);work={65536};
        if(!TestTrue(TEXT("provider inside actual box but away from bottom rays constructed"),
            FIndexedIntervalUnion::Build(MakeArrayView(surfaces),work,scene)&&
            BuildCanonicalFixedBoxSupportProbeEnvelope(box,MakeArrayView(probes),combined)&&
            FReferenceBoundsCandidateSet::Select(scene,combined,work,partition)))return false;
        TestFalse(TEXT("actual box cannot be discarded when rays alone are clear"),FReferenceBoundsStrictSeparation::TryProve(partition,work,proof));
    }
    TestTrue(TEXT("floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}

#include "CanonicalFixedSphereWheelProbeMathTests.inl"

#include "CanonicalRawCensusBoundaryTests.inl"

static Speed::PreparedExecution::FNativeSolidStaticValues MakeCapturedFloor266NativeValues()
{
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::PreparedExecution;
    FNativeSolidStaticValues v;v.Shape=ENativeSolidQueryShape::Box;v.Horizon=0.0017187551129609346f;
    v.OwnerState.Location=FVector(-7.1054273576010019e-15,1485.8248973858342,28.75686783937633);
    v.OwnerState.Rotation=FQuat(-0.70709863641110304,-0.70709863641110293,0.0033938747116983277,-0.0033938747116982401);
    v.OwnerState.Velocity=FVector(-1.1546319456101628e-14,-39.176413872545112,-310.07633094189123);
    v.OwnerState.Acceleration=FVector(0,0,-650);
    v.OwnerState.AngularVelocity=FVector(5.5,-9.2111697081334875e-16,-1.7467563404269913e-16);
    v.OwnerState.AngularAcceleration=FVector::ZeroVector;
    v.BodyState.Location=FVector(-4.4408920985006262e-15,1494.7169574822105,16.576749846215009);
    v.BodyState.Rotation=v.OwnerState.Rotation;
    v.BodyState.Velocity=FVector(1.226220169188608e-15,27.814235089842185,-261.17000041182166);
    v.BodyState.Acceleration=FVector(-3.3346816949017223e-14,-268.98481791538279,-281.55143070686995);
    v.BodyState.AngularVelocity=v.OwnerState.AngularVelocity;v.BodyState.AngularAcceleration=v.OwnerState.AngularAcceleration;
    v.LocalOffset=FVector(-5,0,29.510000000000002);v.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);
    v.LocalRotation=FQuat(0,-0.,0,1);v.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
    v.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
    v.Start=v.BodyState.Location;v.End=FVector(-4.4388337815502928e-15,1494.7643660343849,16.127446704420244);
    v.Query.Shape=EQueryShape::Box;v.Query.Start=v.Start;v.Query.End=v.End;
    v.Query.Rotation=v.BodyState.Rotation;v.Query.HalfExtent=v.QueryHalfExtent;
    return v;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeGeneralFloorCaptured266Test,
 "IAmSpeed.AnalyticWorld.GeneralBox.CapturedFloorDescriptor266",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeGeneralFloorCaptured266Test::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;using namespace Speed::PreparedExecution;
    // Exact17digit copied F36 frame266 values, observation707850a8... . Only
    // mathematical fields are reconstructed; no owner/GT/request/profile,
    // real refiner return, mask, SourceId or physics admission is manufactured.
    const auto v=MakeCapturedFloor266NativeValues();
    // Source-derived defaults: MakeStrictSingleQuery1075 -> MakeQuery leaves
    // these two FWorldQuery defaults untouched. Filters are not used by bounds.
    TestEqual(TEXT("captured factory keeps domain tolerance default"),v.Query.DomainTolerance,1.e-6);
    TestEqual(TEXT("captured factory keeps overlap tolerance default"),v.Query.InitialOverlapTolerance,.05);
    const uint32 original=VectorGetControlRegister();
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        FNativeBoxFloorSupportBounds old;TestFalse(TEXT("actual3D operands are outside old Fixed/Yaw scope"),BuildNativeBoxFloorSupportBounds(v,old));
        TestTrue(TEXT("general applicability is explicit"),NativeBoxNeedsGeneralFloorDescriptor(v));
        FDistanceBudget budget{14472};FNativeGeneralBoxFloorDescriptorBounds g;
        if(!TestTrue(TEXT("complete geometry descriptor without NoNew claim"),BuildNativeGeneralBoxFloorDescriptorBounds(v,budget,g)))return false;
        TestTrue(TEXT("captured horizon and32roles retained"),g.Complete&&g.OriginalMotion.Corners.Num()==32&&g.AtZero.Corners.Num()==32&&g.Captured.Horizon==v.Horizon);
        // An independent actual native Integrate sampling regression supports
        // the interval theorem; samples alone are not its coverage proof.
        for(int32 sample=0;sample<=16;++sample)
        {
            const float time=v.Horizon*(float(sample)/16.f);auto pose=v.OwnerState.Integrate(time);
            pose.Location-=pose.Rotation.RotateVector(v.LocalCOM);pose.Location+=pose.Rotation.RotateVector(v.LocalOffset);
            pose.Rotation=pose.Rotation*v.LocalRotation;
            for(int32 role=0;role<2;++role)for(int32 k=0;k<8;++k)
            {
                const auto extent=role?v.QueryHalfExtent:v.PhysicalHalfExtent;
                const FVector local((k&1)?extent.X:-extent.X,(k&2)?extent.Y:-extent.Y,(k&4)?extent.Z:-extent.Z);
                const FVector point=pose.Location+pose.Rotation.RotateVector(local);const auto& bound=g.OriginalMotion.Corners[role*8+k];
                for(int32 d=0;d<3;++d)if(!TestTrue(TEXT("same indexed native corner enclosed across originalH"),
                    bound.Axis[d].Lower<=point[d]&&point[d]<=bound.Axis[d].Upper))return false;
            }
        }
        FDistanceBudget empty{0};TestFalse(TEXT("budget failure leaves no descriptor"),BuildNativeGeneralBoxFloorDescriptorBounds(v,empty,g));
        TestFalse(TEXT("no stale complete result after budget failure"),g.Complete);
        auto altered=v;altered.Query.End.Z=std::nextafter(altered.Query.End.Z,std::numeric_limits<double>::infinity());
        FDistanceBudget bad{14472};TestFalse(TEXT("changed actual chord cannot borrow descriptor"),BuildNativeGeneralBoxFloorDescriptorBounds(altered,bad,g));
    }
    VectorSetControlRegister(original);TestTrue(TEXT("original numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}

// UNINSTALLED / UNCOMPILED pure regression definitions. No physical result.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNativeGeneralPhysicalFloorClearTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.PhysicalFloorClearClosedCover",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FNativeGeneralPhysicalFloorClearTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE established"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    using namespace GeneralPhysicalFloorClearDetail;
    const uint32 original=VectorGetControlRegister();
    const auto fixture=[]
    {
        FGeneralBoxMotionInput v;v.Owner.Location=FVector(0,0,4);
        v.PhysicalHalfExtent=v.QueryHalfExtent=FVector(1,1,1);v.Horizon=.25f;return v;
    };
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        const uint32 current=(original&~uint32(0x8040))|flush;VectorSetControlRegister(current);
        const FVector plane(0,0,0),normal(0,0,1);auto v=fixture();
        FGeneralBoxPhysicalFloorClearBoundsPtr proof;FDistanceBudget enough{33};
        if(!TestTrue(TEXT("all16 physical corners over one complete static cell"),
            TryProveGeneralBoxPhysicalFloorClear(v,plane,normal,enough,proof)))return false;
        TestTrue(TEXT("immutable result matches exact operands"),proof&&proof->Matches(v,plane,normal));
        TestTrue(TEXT("one cell has exact17+16 debits"),enough.Used==33&&enough.Remaining==0&&
            proof->GetVisitedCellCountForInspection()==1&&proof->GetClosedCoverForInspection().Num()==1);
        for(int32 k=0;k<16;++k)TestTrue(TEXT("native/reference physical gaps strictly positive"),
            StrictPositive(proof->GetMinimumGapForInspection(k)));
        for(int32 bad=0;bad<4;++bad)
        {
            auto changed=v;auto point=plane,n=normal;
            if(bad==0)changed.Owner.Location.X=std::nextafter(0.,1.);
            if(bad==1)changed.Horizon=std::nextafter(v.Horizon,1.f);
            if(bad==2)point.X=1.;if(bad==3)n.Z=-1.;
            TestFalse(TEXT("changed original input or plane cannot borrow result"),proof->Matches(changed,point,n));
        }
        VectorSetControlRegister(current^64u);
        TestFalse(TEXT("proof cannot cross numeric contexts"),proof->Matches(v,plane,normal));
        VectorSetControlRegister(current);
        // Factory clears output without invalidating an input reference into
        // that exact previous immutable output.
        const auto& aliased=proof->GetInputForInspection();FDistanceBudget aliasBudget{33};
        if(!TestTrue(TEXT("aliased input survives old output reset"),
            TryProveGeneralBoxPhysicalFloorClear(aliased,plane,normal,aliasBudget,proof)))return false;
        for(int32 allowance:{0,1,16,17,18,32})
        {
            FDistanceBudget partial{allowance};
            TestFalse(TEXT("incomplete model/gap/hull work publishes nothing"),
                TryProveGeneralBoxPhysicalFloorClear(v,plane,normal,partial,proof));
            TestFalse(TEXT("no stale result after partial work"),proof.IsValid());
            TestTrue(TEXT("no refund or negative remaining"),partial.Used==uint64(allowance)&&partial.Remaining==0);
        }
        FDistanceBudget reserved{34};
        {
            FScopedDistanceBudgetFloor reservation(reserved,2);
            TestTrue(TEXT("reservation established"),reservation.IsValid());
            TestFalse(TEXT("reserved work cannot complete last hull debit"),
                TryProveGeneralBoxPhysicalFloorClear(v,plane,normal,reserved,proof));
            TestTrue(TEXT("reserved units untouched and no partial proof"),reserved.Used==32&&reserved.Remaining==2&&!proof);
        }
        TestTrue(TEXT("release only restores consumption floor"),reserved.ConsumptionFloor==0&&reserved.Used==32&&reserved.Remaining==2);
        // [0,1] is the entire continuous [0,FLT_MIN], not just two endpoints.
        v.Horizon=std::numeric_limits<float>::min();FDistanceBudget firstCell{33};
        if(!TestTrue(TEXT("first compressed float cell entirely enclosed"),
            TryProveGeneralBoxPhysicalFloorClear(v,plane,normal,firstCell,proof)))return false;
        const auto cover=proof->GetClosedCoverForInspection();
        TestTrue(TEXT("first cell includes continuous FTZ gap"),cover.Num()==1&&cover[0].First==0&&cover[0].Last==1);
        for(double signedZero:{0.,-0.})
        {
            auto touching=v;touching.Owner.Location.Z=1.;FDistanceBudget tangent{33};
            TestFalse(TEXT("uncertain adjacent tangency never uses endpoint shortcut"),
                TryProveGeneralBoxPhysicalFloorClear(touching,FVector(0,0,signedZero),FVector(signedZero,0,1),tangent,proof));
            TestFalse(TEXT("tangency leaves no result"),proof.IsValid());
        }
        // The reference continuously rotates through pi/4 inside this ONE
        // compressed cell. Both endpoint Boxes clear Z=0; its interior bottom
        // is 1.38-sqrt(2)<0. No subnormal midpoint float is substituted.
        auto adjacent=fixture();adjacent.Horizon=std::numeric_limits<float>::min();
        adjacent.Owner.Location.Z=1.38;
        adjacent.Owner.Rotation=FQuat(FVector::ForwardVector,UE_DOUBLE_PI/4.-.45);
        adjacent.Owner.AngularVelocity=FVector(.9/double(adjacent.Horizon),0,0);
        for(float time:{0.f,adjacent.Horizon})
        {
            FGeneralBoxMotionBounds endpoints;
            if(!TestTrue(TEXT("adjacent counterexample endpoint bounds exist"),
                BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(adjacent,time,time,endpoints)))return false;
            for(int32 role:{0,16})for(int32 k=0;k<8;++k)
            {
                Interval gap;
                if(!TestTrue(TEXT("adjacent counterexample endpoints really clear"),
                    RawGap(endpoints.Corners[role+k],plane,normal,gap)&&StrictPositive(gap)))return false;
            }
        }
        FDistanceBudget adjacentBudget{33};
        TestFalse(TEXT("positive adjacent endpoints cannot close continuous FTZ gap"),
            TryProveGeneralBoxPhysicalFloorClear(adjacent,plane,normal,adjacentBudget,proof));
        TestTrue(TEXT("uncertain adjacent interval stops after one complete16gap model"),!proof&&adjacentBudget.Used==17);
        // Both endpoints are above the plane; the actual interior goes below.
        auto crossing=fixture();crossing.Horizon=1.f;crossing.Owner.Location.Z=2.;
        crossing.Owner.Velocity.Z=-8.;crossing.Owner.Acceleration.Z=16.;
        for(float time:{0.f,1.f})
        {
            FGeneralBoxMotionBounds endpoints;
            if(!TestTrue(TEXT("counterexample endpoint bounds exist"),
                BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(crossing,time,time,endpoints)))return false;
            for(int32 role:{0,16})for(int32 k=0;k<8;++k)
            {
                Interval gap;
                if(!TestTrue(TEXT("each endpoint physical corner is strictly clear"),
                    RawGap(endpoints.Corners[role+k],plane,normal,gap)&&StrictPositive(gap)))return false;
            }
        }
        FGeneralBoxMotionBounds interior;
        if(!TestTrue(TEXT("counterexample actual interior bounds exist"),
            BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(crossing,.5f,.5f,interior)))return false;
        Interval below;
        TestTrue(TEXT("interior is certified strictly below despite endpoints"),
            RawGap(interior.Corners[0],plane,normal,below)&&below.Upper<0.);
        FDistanceBudget crossBudget{4096};
        TestFalse(TEXT("positive endpoints cannot admit interior crossing"),
            TryProveGeneralBoxPhysicalFloorClear(crossing,plane,normal,crossBudget,proof));
        TestFalse(TEXT("crossing never publishes partial cover"),proof.IsValid());
        FIntervalPoint overflow=Detail::Point(FVector(-std::numeric_limits<double>::max(),0,2));Interval invalid;
        TestFalse(TEXT("fullXYZ arithmetic rejects overflowing subtraction times zero"),
            RawGap(overflow,FVector(std::numeric_limits<double>::max(),0,0),normal,invalid));
        auto invalidH=fixture();invalidH.Horizon=0;FDistanceBudget zero{33};
        TestFalse(TEXT("zero horizon cannot publish motion proof"),TryProveGeneralBoxPhysicalFloorClear(invalidH,plane,normal,zero,proof));
        TestFalse(TEXT("zero horizon leaves no result"),proof.IsValid());
        auto unsupported=fixture();unsupported.Horizon=1.f;
        unsupported.Owner.AngularVelocity=FVector(3,0,0);unsupported.Owner.AngularAcceleration=FVector(-6,0,0);
        FDistanceBudget failedModel{33};
        TestFalse(TEXT("unsupported original global angular model is terminal"),
            TryProveGeneralBoxPhysicalFloorClear(unsupported,plane,normal,failedModel,proof));
        TestTrue(TEXT("model failure is not subdivided or retried"),!proof&&failedModel.Used==1&&failedModel.Remaining==32);
        // Captured operands are reconstructed in the same existing helper as
        // the descriptor regression. No real refiner outcome is fabricated.
        const auto captured=MakeCapturedFloor266NativeValues();FGeneralBoxMotionInput originalInput;
        originalInput.Owner=captured.OwnerState;originalInput.LocalCOM=captured.LocalCOM;originalInput.LocalOffset=captured.LocalOffset;
        originalInput.LocalRotation=captured.LocalRotation;originalInput.PhysicalHalfExtent=captured.PhysicalHalfExtent;
        originalInput.QueryHalfExtent=captured.QueryHalfExtent;originalInput.Horizon=captured.Horizon;
        const FVector capturedPoint(-43.585199999999979,1560.2208516644996,0);FDistanceBudget capturedBudget{14472};
        if(!TestTrue(TEXT("captured266 physical native/reference complete FLOOR cover"),
            TryProveGeneralBoxPhysicalFloorClear(originalInput,capturedPoint,normal,capturedBudget,proof)))return false;
        TestTrue(TEXT("captured proof matches original H and full input"),proof->Matches(originalInput,capturedPoint,normal));
        AddInfo(FString::Printf(TEXT("[GeneralPhysicalFloorClear266] FTZ=%u Used=%llu Cells=%llu"),flush,capturedBudget.Used,proof->GetVisitedCellCountForInspection()));
    }
    VectorSetControlRegister(original);TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}

// UNINSTALLED / UNCOMPILED pure targets; no current native outcome is assumed.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGeneralPhysicalFloorFirstEntryTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.PhysicalFloorFirstEntry",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGeneralPhysicalFloorFirstEntryTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE established"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const uint32 original=VectorGetControlRegister();const FVector plane(0,0,0),normal(0,0,1);
    const auto fixture=[]
    {
        FGeneralBoxMotionInput input;input.Owner.Location=FVector(0,0,4);input.PhysicalHalfExtent=input.QueryHalfExtent=FVector(1,1,1);
        input.Horizon=1.f;return input;
    };
    const auto validPrefix=[&](const FGeneralPhysicalFloorMotionBoundsPtr& proof)
    {
        if(!proof)return false;uint32 target=0;
        if(!FloorNativeSweepDetail::Ordinal(proof->GetClearSeconds(),target))return false;
        const auto cover=proof->GetPrefixForInspection();if(cover.IsEmpty()||cover[0].First!=0||cover.Last().Last!=target)return false;
        for(int32 i=1;i<cover.Num();++i)if(cover[i-1].Last!=cover[i].First)return false;
        return true;
    };
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        const uint32 current=(original&~uint32(0x8040))|flush;VectorSetControlRegister(current);
        auto flat=fixture();FGeneralPhysicalFloorMotionBoundsPtr proof;FDistanceBudget whole{34};
        if(!TestTrue(TEXT("new one-search whole-clear classification"),BoundGeneralPhysicalFloorMotion(flat,plane,normal,whole,proof)==
            EGeneralPhysicalFloorMotion::WholePhysicalClear))return false;
        TestTrue(TEXT("one initial and one closed cell cost34"),whole.Used==34&&whole.Remaining==0&&proof->GetClassificationsForInspection()==2);
        TestTrue(TEXT("whole prefix and exact input retained"),validPrefix(proof)&&proof->Matches(flat,plane,normal)&&proof->GetClearSeconds()==flat.Horizon);
        const auto saved=proof->GetInputForInspection();const auto& alias=proof->GetInputForInspection();FDistanceBudget aliasWork{34};
        if(!TestTrue(TEXT("one-search input/output alias safe"),BoundGeneralPhysicalFloorMotion(alias,plane,normal,aliasWork,proof)==
            EGeneralPhysicalFloorMotion::WholePhysicalClear&&proof->Matches(saved,plane,normal)))return false;
        for(int32 amount:{0,1,16,17,18,33})
        {
            FDistanceBudget partial{amount};TestTrue(TEXT("partial classification publishes no status or cover"),
                BoundGeneralPhysicalFloorMotion(flat,plane,normal,partial,proof)==EGeneralPhysicalFloorMotion::Unknown&&!proof);
            TestTrue(TEXT("no budget refund"),partial.Used==uint64(amount)&&partial.Remaining==0);
        }
        auto third=fixture();third.Owner.Location.Z=2.;third.Owner.Velocity.Z=-3.;FDistanceBudget entryWork{4096};
        if(!TestTrue(TEXT("unrepresentable1/3 root gets first adjacent physical arrival"),
            BoundGeneralPhysicalFloorMotion(third,plane,normal,entryWork,proof)==EGeneralPhysicalFloorMotion::FirstAdjacentArrival))return false;
        uint32 c=0,u=0;TestTrue(TEXT("adjacent positive ordinal endpoints"),FloorNativeSweepDetail::Ordinal(proof->GetClearSeconds(),c)&&
            FloorNativeSweepDetail::Ordinal(proof->GetInsideSeconds(),u)&&c>0&&u==c+1);
        TestTrue(TEXT("first event retains complete continuous prefix and bottom-face mask"),validPrefix(proof)&&proof->GetCommonInsideMask()==0x0f);
        const auto cost=entryWork.Used;TestTrue(TEXT("entry cost available for budget-boundary negatives"),cost>34&&cost<=4096);
        // Interrupt precisely within the last two terminal singleton models.
        for(uint64 missing:{1ull,16ull,17ull,33ull})
        {
            FDistanceBudget partial{int32(cost-missing)};TestTrue(TEXT("terminal singleton classification cannot publish a partial event"),
                BoundGeneralPhysicalFloorMotion(third,plane,normal,partial,proof)==EGeneralPhysicalFloorMotion::Unknown&&!proof);
            TestTrue(TEXT("terminal partial costs stay charged"),partial.Used==cost-missing&&partial.Remaining==0);
        }
        auto exact=third;exact.Owner.Velocity.Z=-2.;FDistanceBudget zeroWork{4096};
        TestTrue(TEXT("unresolved representable zero is never skipped for later inside"),
            BoundGeneralPhysicalFloorMotion(exact,plane,normal,zeroWork,proof)==EGeneralPhysicalFloorMotion::Unknown&&!proof);
        auto bad=fixture();bad.Owner.AngularVelocity=FVector(3,0,0);bad.Owner.AngularAcceleration=FVector(-6,0,0);
        FDistanceBudget badWork{128};TestTrue(TEXT("global model failure after initial singleton is terminal"),
            BoundGeneralPhysicalFloorMotion(bad,plane,normal,badWork,proof)==EGeneralPhysicalFloorMotion::Unknown&&!proof&&badWork.Used==18);
        // Exact17digit actual F38 next-step OWNER/local operands. Mathematical
        // fixture only: no request/GT/profile/real corner or native TOI made up.
        FGeneralBoxMotionInput captured;captured.Horizon=0.0033333334140479565f;
        captured.Owner.Location=FVector(-7.1252726532020568e-15,1485.7575627241833,28.222962471469152);
        captured.Owner.Rotation=FQuat(-0.70710677929701116,-0.70710677929701105,5.1693404612106082e-05,-5.169340461201917e-05);
        captured.Owner.Velocity=FVector(-1.1546319456101628e-14,-39.176413872545112,-311.19352176531584);
        captured.Owner.Acceleration=FVector(0,0,-650);
        captured.Owner.AngularVelocity=FVector(5.5,-9.2111697081334875e-16,-1.7467563404269913e-16);
        captured.Owner.AngularAcceleration=FVector::ZeroVector;
        captured.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);
        captured.LocalOffset=FVector(-5,0,29.510000000000002);captured.LocalRotation=FQuat(0,-0.,0,1);
        captured.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
        captured.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
        const FVector point(-8.0134510729021821e-15,1494.7643643224849,0);FDistanceBudget capturedWork{26439};
        if(!TestTrue(TEXT("actual next-step operands get a fresh pure first-arrival target"),
            BoundGeneralPhysicalFloorMotion(captured,point,normal,capturedWork,proof)==EGeneralPhysicalFloorMotion::FirstAdjacentArrival))return false;
        c=u=0;TestTrue(TEXT("actual mathematical target retains adjacent closed prefix and nonzero common mask"),validPrefix(proof)&&
            FloorNativeSweepDetail::Ordinal(proof->GetClearSeconds(),c)&&FloorNativeSweepDetail::Ordinal(proof->GetInsideSeconds(),u)&&c>0&&u==c+1&&proof->GetCommonInsideMask()!=0);
        TestTrue(TEXT("actual original owner/H identity preserved"),proof->Matches(captured,point,normal));
        AddInfo(FString::Printf(TEXT("[GeneralPhysicalFloorFirst267] FTZ=%u Used=%llu Classifications=%llu C=%.17g U=%.17g Mask=%u"),
            flush,capturedWork.Used,proof->GetClassificationsForInspection(),double(proof->GetClearSeconds()),double(proof->GetInsideSeconds()),uint32(proof->GetCommonInsideMask())));
        auto changed=captured;changed.Horizon=std::nextafter(captured.Horizon,1.f);TestFalse(TEXT("changed horizon cannot inherit entry"),proof->Matches(changed,point,normal));
        VectorSetControlRegister(current^64u);TestFalse(TEXT("entry cannot cross numeric control fingerprint"),proof->Matches(captured,point,normal));VectorSetControlRegister(current);
    }
    VectorSetControlRegister(original);TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}

// Pure mathematical fixtures only; no native actor/profile or response grant.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FClosedPhysicalFloorPoseCorridorTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.ClosedPhysicalFloorPoseCorridor",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FClosedPhysicalFloorPoseCorridorTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE established"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const uint32 original=VectorGetControlRegister();const FVector point(0,0,0),normal(0,0,1);
    const auto fixture=[]
    {
        FBoxPoseCorridorInput v;v.BeforeCOM=FVector(0,0,4);v.AfterCOM=FVector(.125,0,4);
        v.PhysicalHalfExtent=FVector(1,1,1);v.QueryHalfExtent=FVector(9,9,9);
        v.DomainShell=.125;v.OverlapShell=.0625;return v;
    };
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        const uint32 control=(original&~uint32(0x8040))|flush;VectorSetControlRegister(control);
        uint64 negativeTinyBits=0x8000000000000001ull,positiveTinyBits=1;double negativeTiny=0,positiveTiny=0;
        FMemory::Memcpy(&negativeTiny,&negativeTinyBits,8);FMemory::Memcpy(&positiveTiny,&positiveTinyBits,8);
        TestTrue(TEXT("signed zero is closed nonnegative"),ClosedPhysicalFloorPoseDetail::NonNegative({-0.,0.,true}));
        TestFalse(TEXT("forged positive subnormal interval is invalid under every mode"),
            ClosedPhysicalFloorPoseDetail::NonNegative({positiveTiny,positiveTiny,true}));
        TestTrue(TEXT("outward exact positive subnormal is closed nonnegative"),
            ClosedPhysicalFloorPoseDetail::NonNegative(AnalyticCertified::Interval::Exact(positiveTiny)));
        TestFalse(TEXT("negative subnormal cannot masquerade as zero"),
            ClosedPhysicalFloorPoseDetail::NonNegative({negativeTiny,0.,true}));
        TestFalse(TEXT("straddling zero interval is unknown"),
            ClosedPhysicalFloorPoseDetail::NonNegative({negativeTiny,positiveTiny,true}));
        TestFalse(TEXT("valid normal straddle is unknown"),
            ClosedPhysicalFloorPoseDetail::NonNegative({-DBL_MIN,DBL_MIN,true}));
        auto v=fixture();FIntervalPoint raw;
        if(!TestTrue(TEXT("synthetic complete shell envelope"),BuildBoxPoseCorridorBounds(v,raw)))return false;
        TestTrue(TEXT("query shell intersects floor while physical shape stays clear"),raw.Axis[2].Lower<0);
        FClosedPhysicalFloorPoseCorridorBoundsPtr proof;FDistanceBudget work{10};
        if(!TestTrue(TEXT("closed physical proof ignores query extents and shells for FLOOR"),
            TryProveClosedPhysicalFloorPoseCorridor(v,raw,point,normal,work,proof)))return false;
        TestTrue(TEXT("two prepaid constructions plus eight physical gaps"),work.Used==10&&work.Remaining==0&&proof->Matches(v,raw,point,normal));
        const auto& aliasInput=proof->GetInputForInspection();const auto& aliasRaw=proof->GetRawEnvelopeForInspection();FDistanceBudget alias{10};
        if(!TestTrue(TEXT("input and raw output aliases stay alive"),
            TryProveClosedPhysicalFloorPoseCorridor(aliasInput,aliasRaw,point,normal,alias,proof)))return false;
        for(int32 amount=0;amount<10;++amount)
        {
            FDistanceBudget partial{amount};TestFalse(TEXT("partial budget never publishes a corridor"),
                TryProveClosedPhysicalFloorPoseCorridor(v,raw,point,normal,partial,proof));
            TestTrue(TEXT("partial costs remain charged, no refill or result"),partial.Used==uint64(amount)&&partial.Remaining==0&&!proof);
        }
        FDistanceBudget reserved{11};reserved.ConsumptionFloor=1;
        TestTrue(TEXT("existing reservation is preserved"),TryProveClosedPhysicalFloorPoseCorridor(v,raw,point,normal,reserved,proof)&&
            reserved.Used==10&&reserved.Remaining==1);
        FDistanceBudget shortReserved{10};shortReserved.ConsumptionFloor=1;
        TestFalse(TEXT("cannot consume reserved last unit"),TryProveClosedPhysicalFloorPoseCorridor(v,raw,point,normal,shortReserved,proof));
        TestTrue(TEXT("reservation failure leaves no partial proof"),shortReserved.Used==9&&shortReserved.Remaining==1&&!proof);
        auto physicalBad=v;physicalBad.PhysicalHalfExtent=FVector(1,1,9);physicalBad.QueryHalfExtent=FVector(1,1,1);
        FIntervalPoint badRaw;if(!TestTrue(TEXT("negative physical fixture hull"),BuildBoxPoseCorridorBounds(physicalBad,badRaw)))return false;
        FDistanceBudget badWork{10};TestFalse(TEXT("positive query gap cannot hide physical penetration"),
            TryProveClosedPhysicalFloorPoseCorridor(physicalBad,badRaw,point,normal,badWork,proof));TestFalse(TEXT("no negative physical publication"),proof.IsValid());
        auto noShell=v;noShell.DomainShell=noShell.OverlapShell=0;FIntervalPoint small;
        if(!TestTrue(TEXT("unshelled outer fixture"),BuildBoxPoseCorridorBounds(noShell,small)))return false;
        FDistanceBudget escaped{10};TestFalse(TEXT("physical clearance cannot widen authenticated NONfloor corridor"),
            TryProveClosedPhysicalFloorPoseCorridor(v,small,point,normal,escaped,proof));
        TestTrue(TEXT("shell containment failure stops before physical model"),escaped.Used==1&&!proof);
        auto outer=raw,inner=raw;outer.Axis[0]={0.,1.,true};inner.Axis[0]={negativeTiny,1.,true};
        TestFalse(TEXT("DAZ cannot hide a negative subnormal envelope escape"),ClosedPhysicalFloorPoseDetail::Within(outer,inner));
        auto antipodal=v;antipodal.AfterRotation=FQuat(0,0,0,-1);FDistanceBudget antipodalWork{10};
        TestFalse(TEXT("unknown quaternion normalization not sign-flipped or retried"),
            TryProveClosedPhysicalFloorPoseCorridor(antipodal,raw,point,normal,antipodalWork,proof));
        TestTrue(TEXT("failed initial model still charged"),antipodalWork.Used==1&&!proof);
        // ACTUAL F40 FIRST SETTER before/RHS only: Pnew/Qold, not the local
        // final solver Q. This diagnostic does NOT assume interval success.
        FBoxPoseCorridorInput copied;
        copied.BeforeCOM=FVector(-7.1287924524990529e-15,1485.7456201207822,28.128067517081476);
        copied.AfterCOM=FVector(-7.1287924524990529e-15,1485.7456201207822,28.128067504035116);
        copied.BeforeRotation=copied.AfterRotation=FQuat(-0.70710657416516209,-0.70710657416516198,-0.00054108447425419803,0.00054108447425428488);
        copied.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);
        copied.LocalOffset=FVector(-5,0,29.510000000000002);copied.LocalRotation=FQuat(0,-0.,0,1);
        copied.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
        copied.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
        copied.DomainShell=.000001;copied.OverlapShell=.05;
        FIntervalPoint copiedSyntheticRaw;
        if(!TestTrue(TEXT("copied operand pure shell envelope"),BuildBoxPoseCorridorBounds(copied,copiedSyntheticRaw)))return false;
        FDistanceBudget copiedWork{10};const bool copiedProved=TryProveClosedPhysicalFloorPoseCorridor(copied,copiedSyntheticRaw,point,normal,copiedWork,proof);
        AddInfo(FString::Printf(TEXT("[ClosedPhysicalFloorPoseF40FirstRHS] FTZ=%u Proved=%d Used=%llu Remaining=%d QFinalNotCaptured=1 AuthenticatedWorldRawNotUsed=1"),
            flush,int32(copiedProved),copiedWork.Used,copiedWork.Remaining));
        TestTrue(TEXT("copied target remains true proof or explicit unknown"),copiedProved?
            proof&&proof->Matches(copied,copiedSyntheticRaw,point,normal):!proof);
        // Reestablish a normal accepted value for identity negatives.
        FDistanceBudget identity{10};if(!TestTrue(TEXT("normal proof for identity checks"),TryProveClosedPhysicalFloorPoseCorridor(v,raw,point,normal,identity,proof)))return false;
        auto altered=v;altered.QueryHalfExtent.X=std::nextafter(v.QueryHalfExtent.X,10.);
        TestFalse(TEXT("query extent cannot inherit physical corridor identity"),proof->Matches(altered,raw,point,normal));
        VectorSetControlRegister(control^64u);TestFalse(TEXT("numeric control fingerprint is exact"),proof->Matches(v,raw,point,normal));VectorSetControlRegister(control);
    }
    VectorSetControlRegister(original);TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}

// UNINSTALLED pure geometry fixtures. No new native scenario/profile or pose.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPhysicalBodyBoxPoseBoundsTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.PhysicalBodyPoseCorrelatedNormalization",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPhysicalBodyBoxPoseBoundsTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture ready"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const uint32 original=VectorGetControlRegister();const FVector plane(0,0,0),normal(0,0,1);
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        const uint32 control=(original&~uint32(0x8040))|flush;VectorSetControlRegister(control);
        FPhysicalBodyBoxPoseInput v;v.BeforeCenter=FVector(0,0,5);v.AfterCenter=FVector(.125,0,5);v.PhysicalHalfExtent=FVector(2,2,2);
        FClosedPhysicalBodyBoxFloorBoundsPtr proof;FDistanceBudget work{17};
        if(!TestTrue(TEXT("both raw native and ideal unit physical families are closed"),TryProveClosedPhysicalBodyBoxFloor(v,plane,normal,work,proof)))return false;
        TestTrue(TEXT("one model plus sixteen prepaid distinct gaps"),work.Used==17&&work.Remaining==0&&proof->Matches(v,plane,normal));
        const auto& alias=proof->GetBodyBoundsForInspection().Captured;FDistanceBudget aliasWork{17};
        if(!TestTrue(TEXT("input alias survives replacing output"),TryProveClosedPhysicalBodyBoxFloor(alias,plane,normal,aliasWork,proof)))return false;
        for(int32 amount=0;amount<17;++amount)
        {
            FDistanceBudget partial{amount};TestFalse(TEXT("no partial proof"),TryProveClosedPhysicalBodyBoxFloor(v,plane,normal,partial,proof));
            TestTrue(TEXT("partial work charged and no refill"),partial.Used==uint64(amount)&&partial.Remaining==0&&!proof);
        }
        FDistanceBudget reserved{18};reserved.ConsumptionFloor=1;
        TestTrue(TEXT("reservation retained"),TryProveClosedPhysicalBodyBoxFloor(v,plane,normal,reserved,proof)&&reserved.Used==17&&reserved.Remaining==1);
        FDistanceBudget shortReserved{17};shortReserved.ConsumptionFloor=1;
        TestFalse(TEXT("reservation cannot be consumed"),TryProveClosedPhysicalBodyBoxFloor(v,plane,normal,shortReserved,proof));
        TestTrue(TEXT("reserved unit remains"),shortReserved.Used==16&&shortReserved.Remaining==1&&!proof);
        auto below=v;below.BeforeCenter.Z=below.AfterCenter.Z=1;FDistanceBudget bad{17};
        TestFalse(TEXT("physical penetration is not hidden by normalization"),TryProveClosedPhysicalBodyBoxFloor(below,plane,normal,bad,proof));
        auto antipodal=v;antipodal.AfterRotation=FQuat(0,0,0,-1);FDistanceBudget zeroChord{17};
        TestFalse(TEXT("zero-containing chord remains unknown"),TryProveClosedPhysicalBodyBoxFloor(antipodal,plane,normal,zeroChord,proof));
        TestTrue(TEXT("zero chord charged once"),zeroChord.Used==1&&!proof);
        auto referenceBad=v;referenceBad.BeforeCenter=referenceBad.AfterCenter=FVector(0,0,1.-1.e-8);
        referenceBad.PhysicalHalfExtent=FVector(1,1,1);
        referenceBad.BeforeRotation=referenceBad.AfterRotation=FQuat(.70710677,.70710677,0,0);
        FDistanceBudget referenceWork{17};
        // Equal X/Y, Z/W0 gives exact ideal-unit z inversion independently
        // of the slightly short raw quaternion. The unit-reference top corner
        // is below0 because the supplied center is the predecessor of1.
        TestFalse(TEXT("short raw quaternion cannot omit the negative unit reference"),
            TryProveClosedPhysicalBodyBoxFloor(referenceBad,plane,normal,referenceWork,proof));
        TestTrue(TEXT("all eight native gaps passed before the unit-reference refusal"),referenceWork.Used>9&&!proof);
        // Genuine F41 current BODY pose + genuine solver P/Q. The following
        // live-origin target is DERIVED pure math, not the future actual sync:
        // an actual history route/origin must still be observed at runtime.
        FPhysicalBodyBoxPoseInput copied;
        copied.BeforeCenter=FVector(-8.9051492918993034e-15,1494.7726887469412,16.047668554153429);
        copied.BeforeRotation=FQuat(-.70710657416516209,-.70710657416516198,-.00054108447425419803,.00054108447425428488);
        const FVector solverP(-7.1287924524990529e-15,1485.7456201207822,28.128067504035116);
        const FQuat solverQ(-.70710657416516209,-.70710657416516198,-.00054108447425424151,.0005410844742542414);
        const FVector localCOM(-14.008570000000001,0,17.415800000000001),offset(-5,0,29.510000000000002);
        const FVector origin=solverP-solverQ.RotateVector(localCOM);
        copied.AfterCenter=origin+solverQ.RotateVector(offset);copied.AfterRotation=solverQ*FQuat(0,-0.,0,1);
        copied.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
        FDistanceBudget copiedWork{17};const bool proved=TryProveClosedPhysicalBodyBoxFloor(copied,plane,normal,copiedWork,proof);
        AddInfo(FString::Printf(TEXT("[PhysicalBodyPoseF41DerivedLiveOrigin] FTZ=%u Proved=%d Used=%llu Remaining=%d ActualFutureSyncNotObserved=1 NoWorldRawOrSetterGrant=1"),flush,int32(proved),copiedWork.Used,copiedWork.Remaining));
        if(!TestTrue(TEXT("captured body and derived target pass both independent physical families"),proved&&proof&&proof->Matches(copied,plane,normal)))return false;
        auto changed=copied;changed.AfterCenter.Z=std::nextafter(changed.AfterCenter.Z,100.);
        TestFalse(TEXT("actual target bits are bound"),proof->Matches(changed,plane,normal));
        VectorSetControlRegister(control^64u);TestFalse(TEXT("numeric context exact"),proof->Matches(copied,plane,normal));VectorSetControlRegister(control);
    }
    VectorSetControlRegister(original);TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}

// New pure mathematical contracts, no gameplay profile/pose or native result.
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGeneralBoxOriginalHPhysicalPrefixTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.PhysicalFloorOriginalHPrefix",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGeneralBoxOriginalHPhysicalPrefixTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture ready"),context.Ready))return false;
    using namespace Speed;using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const uint32 original=VectorGetControlRegister();const FVector plane(0,0,0),normal(0,0,1);
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        const uint32 control=(original&~uint32(0x8040))|flush;VectorSetControlRegister(control);
        FGeneralBoxMotionInput v;v.Owner.Location=FVector(0,0,5);v.Owner.Rotation=FQuat::Identity;
        v.Owner.Velocity=FVector(0,0,-8);v.LocalRotation=FQuat::Identity;v.PhysicalHalfExtent=FVector(1,1,1);
        v.QueryHalfExtent=FVector(10,10,10);v.Horizon=1.f;
        FGeneralBoxPhysicalFloorPrefixBoundsPtr proof;FDistanceBudget work{33};
        if(!TestTrue(TEXT("closed physical prefix excludes later penetration without rewriting original H"),
            TryProveGeneralBoxPhysicalFloorPrefix(v,plane,normal,.25f,work,proof)))return false;
        TestTrue(TEXT("exact first cell work and same ORIGINAL H"),work.Used==33&&work.Remaining==0&&proof&&
            proof->Matches(v,plane,normal,.25f)&&proof->GetInputForInspection().Horizon==1.f&&proof->GetClosedCoverForInspection().Num()==1);
        const auto& aliased=proof->GetInputForInspection();FDistanceBudget aliasWork{33};
        if(!TestTrue(TEXT("alias input survives output replacement"),TryProveGeneralBoxPhysicalFloorPrefix(aliased,plane,normal,.25f,aliasWork,proof)))return false;
        auto shortened=v;shortened.Horizon=.25f;
        TestFalse(TEXT("same geometry with rewritten input H has another identity"),proof->Matches(shortened,plane,normal,.25f));
        TestFalse(TEXT("actual advance time is bound exactly"),proof->Matches(v,plane,normal,.125f));
        for(int32 amount=0;amount<33;++amount)
        {
            FDistanceBudget partial{amount};TestFalse(TEXT("no partial geometry publication"),TryProveGeneralBoxPhysicalFloorPrefix(v,plane,normal,.25f,partial,proof));
            TestTrue(TEXT("each partial debit retained without refill"),partial.Used==uint64(amount)&&partial.Remaining==0&&!proof);
        }
        FDistanceBudget reserved{34};reserved.ConsumptionFloor=1;
        TestTrue(TEXT("reserved final unit retained"),TryProveGeneralBoxPhysicalFloorPrefix(v,plane,normal,.25f,reserved,proof)&&reserved.Used==33&&reserved.Remaining==1);
        FDistanceBudget shortReserved{33};shortReserved.ConsumptionFloor=1;
        TestFalse(TEXT("reservation cannot be consumed"),TryProveGeneralBoxPhysicalFloorPrefix(v,plane,normal,.25f,shortReserved,proof));
        TestTrue(TEXT("no borrowed reserved work"),shortReserved.Used==32&&shortReserved.Remaining==1&&!proof);
        for(float badTime:{0.f,-1.f,2.f})
        {
            FDistanceBudget bad{33};TestFalse(TEXT("invalid actual prefix end"),TryProveGeneralBoxPhysicalFloorPrefix(v,plane,normal,badTime,bad,proof));
            TestTrue(TEXT("invalid time rejected before model, no output"),!bad.Used&&!proof);
        }
        FDistanceBudget full{4096};TestFalse(TEXT("prefix cannot hide later physical penetration"),TryProveGeneralBoxPhysicalFloorPrefix(v,plane,normal,1.f,full,proof));
        TestTrue(TEXT("failed longer prefix stays unknown and charged"),full.Used>0&&!proof);
        auto valley=v;valley.Owner.Location.Z=3;valley.Owner.Velocity.Z=-16;valley.Owner.Acceleration.Z=32;
        FDistanceBudget valleyWork{4096};TestFalse(TEXT("positive endpoints do not hide interior quadratic penetration"),
            TryProveGeneralBoxPhysicalFloorPrefix(valley,plane,normal,1.f,valleyWork,proof));
        TestFalse(TEXT("no endpoint-only proof"),proof.IsValid());
        FDistanceBudget identityWork{33};if(!TestTrue(TEXT("identity fixture rebuilt"),TryProveGeneralBoxPhysicalFloorPrefix(v,plane,normal,.25f,identityWork,proof)))return false;
        VectorSetControlRegister(control^64u);TestFalse(TEXT("numeric context is exact"),proof->Matches(v,plane,normal,.25f));VectorSetControlRegister(control);
        // Historical real Fast42 post-response owner operands, paired with
        // actual Fast43 TOI only as a PURE regression fixture. These do not
        // claim that a new native request or its input identity was observed.
        FGeneralBoxMotionInput copied;
        copied.Owner.Location=FVector(-7.1287924524990529e-15,1485.7456201207822,28.128067504035116);
        copied.Owner.Rotation=FQuat(-.70710657416516209,-.70710657416516198,-.00054108447425424151,.0005410844742542414);
        copied.Owner.Velocity=FVector(4.5237887510060573e-14,43.06554147351514,-23.013520045000007);
        copied.Owner.Acceleration=FVector(0,0,-650);copied.Owner.AngularVelocity=FVector(-3.7067782900319806,5.0335342515635393e-15,-2.404090837119531e-16);
        copied.LocalCOM=FVector(-14.008570000000001,0,17.415800000000001);copied.LocalOffset=FVector(-5,0,29.510000000000002);
        copied.LocalRotation=FQuat(0,-0.,0,1);copied.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
        copied.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
        copied.Horizon=.0030284917447715998f;FDistanceBudget copiedWork{10384};
        const bool copiedKnown=TryProveGeneralBoxPhysicalFloorPrefix(copied,plane,normal,.00066982320277020335f,copiedWork,proof);
        AddInfo(FString::Printf(TEXT("[OriginalHPhysicalPrefixHistorical42] FTZ=%u Proved=%d Used=%llu Remaining=%d CurrentRequestNotClaimed=1 NoContactOrTransportGrant=1"),
            flush,int32(copiedKnown),copiedWork.Used,copiedWork.Remaining));
        TestTrue(TEXT("historical operands yield a matching geometry value or explicit unknown"),copiedKnown?
            proof&&proof->Matches(copied,plane,normal,.00066982320277020335f):!proof);
        // Inspect the already existing model, never manufacture a tolerance.
        FGeneralBoxMotionBounds atZero;
        if(!TestTrue(TEXT("historical zero-time model exists"),BuildGeneralBoxMotionBoundsOnGlobalFloatSeconds(copied,0.f,0.f,atZero)))return false;
        for(int32 family=0;family<2;++family)for(int32 corner:{4,5,6,7})
        {
            AnalyticCertified::Interval gap;
            if(!TestTrue(TEXT("historical zero-time physical raw gap is finite"),GeneralPhysicalFloorClearDetail::RawGap(atZero.Corners[family*16+corner],plane,normal,gap)))return false;
            AddInfo(FString::Printf(TEXT("[OriginalHPhysicalPrefixHistorical42ZeroGap] FTZ=%u Family=%d Corner=%d Lower=%.17g Upper=%.17g"),flush,family,corner,gap.Lower,gap.Upper));
        }
    }
    VectorSetControlRegister(original);TestTrue(TEXT("numeric context restored"),context.Restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalNativeCollinearPlaneTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalAxialNativeCollinearBoundary",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalNativeCollinearPlaneTest::RunTest(const FString&)
{
    using namespace Speed::Analytic::Certified;namespace A=::AnalyticCertified;namespace N=CanonicalAxialPlaneDetail;
    const auto original=Speed::CaptureFrameFloatingPointContext();Speed::FScopedFrameFloatingPointContext restore(original);std::fenv_t held;
    if(!TestTrue(TEXT("RNE native polygon fixture"),original.bValid&&restore.IsApplied()&&std::feholdexcept(&held)==0&&std::fesetround(FE_TONEAREST)==0))return false;
    const uint32 controls=VectorGetControlRegister();
    // Actual fixed native52-core bits, from expected-data primitive d32de5d9b7cc4ce2,
    // DomainBits224..275. No canonical interval-family or gameplay fixture claim.
    const uint64 nativeBits[][2]={
        {0x0000000000000000ull,0xc0ab800000000000ull},
        {0x40ac1fd4dc8e8476ull,0xc0ab800000000000ull},
        {0x40ac99d0b2a4ec7bull,0xc0ab73fc539ad090ull},
        {0x40ad0f1c774373b4ull,0xc0ab506780b2d534ull},
        {0x40ad7b36372425ecull,0xc0ab169f934fbed2ull},
        {0x40add9f675b04f7cull,0xc0aac8dcfd7be7b2ull},
        {0x40b163fbb31ed723ull,0xc0a5dadc0cee88e9ull},
        {0x40b18b0dbc0e347cull,0xc0a57ba5055bc39full},
        {0x40b1a815eac15a80ull,0xc0a50f03c012cda6ull},
        {0x40b1b9f6a1e1b384ull,0xc0a49924eee9e3bbull},
        {0x40b1c00000000000ull,0xc0a41e902bcf92f6ull},
        {0x40b1c00000000000ull,0xc0a21e902bcf92f6ull},
        {0x40b1c00000000000ull,0xc07bcc147ae147aeull},
        {0x40b1c00000000000ull,0x0000000000000000ull},
        {0x40b1c00000000000ull,0x407bcc147ae147aeull},
        {0x40b1c00000000000ull,0x40a21e902bcf92f6ull},
        {0x40b1c00000000000ull,0x40a41e902bcf92f6ull},
        {0x40b1b9f6a1e1b384ull,0x40a49924eee9e3bbull},
        {0x40b1a815eac15a80ull,0x40a50f03c012cda6ull},
        {0x40b18b0dbc0e347cull,0x40a57ba5055bc39full},
        {0x40b163fbb31ed723ull,0x40a5dadc0cee88e9ull},
        {0x40add9f675b04f7cull,0x40aac8dcfd7be7b2ull},
        {0x40ad7b36372425ecull,0x40ab169f934fbed2ull},
        {0x40ad0f1c774373b4ull,0x40ab506780b2d534ull},
        {0x40ac99d0b2a4ec7bull,0x40ab73fc539ad090ull},
        {0x40ac1fd4dc8e8476ull,0x40ab800000000000ull},
        {0x0000000000000000ull,0x40ab800000000000ull},
        {0xc0ac1fd4dc8e8476ull,0x40ab800000000000ull},
        {0xc0ac99d0b2a4ec7bull,0x40ab73fc539ad090ull},
        {0xc0ad0f1c774373b4ull,0x40ab506780b2d534ull},
        {0xc0ad7b36372425ecull,0x40ab169f934fbed2ull},
        {0xc0add9f675b04f7cull,0x40aac8dcfd7be7b2ull},
        {0xc0b163fbb31ed723ull,0x40a5dadc0cee88e9ull},
        {0xc0b18b0dbc0e347cull,0x40a57ba5055bc39full},
        {0xc0b1a815eac15a80ull,0x40a50f03c012cda6ull},
        {0xc0b1b9f6a1e1b384ull,0x40a49924eee9e3bbull},
        {0xc0b1c00000000000ull,0x40a41e902bcf92f6ull},
        {0xc0b1c00000000000ull,0x40a21e902bcf92f6ull},
        {0xc0b1c00000000000ull,0x407bcc147ae147aeull},
        {0xc0b1c00000000000ull,0x0000000000000000ull},
        {0xc0b1c00000000000ull,0xc07bcc147ae147aeull},
        {0xc0b1c00000000000ull,0xc0a21e902bcf92f6ull},
        {0xc0b1c00000000000ull,0xc0a41e902bcf92f6ull},
        {0xc0b1b9f6a1e1b384ull,0xc0a49924eee9e3bbull},
        {0xc0b1a815eac15a80ull,0xc0a50f03c012cda6ull},
        {0xc0b18b0dbc0e347cull,0xc0a57ba5055bc39full},
        {0xc0b163fbb31ed723ull,0xc0a5dadc0cee88e9ull},
        {0xc0add9f675b04f7cull,0xc0aac8dcfd7be7b2ull},
        {0xc0ad7b36372425ecull,0xc0ab169f934fbed2ull},
        {0xc0ad0f1c774373b4ull,0xc0ab506780b2d534ull},
        {0xc0ac99d0b2a4ec7bull,0xc0ab73fc539ad090ull},
        {0xc0ac1fd4dc8e8476ull,0xc0ab800000000000ull},
    };
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        FCanonicalAxialPlaneBounds value;value.NormalAxis=2;value.UAxis=0;value.VAxis=1;
        value.Plane.Origin=FVector3d(0,0,2048);value.Plane.Normal=FVector3d(0,0,-1);
        value.Plane.AxisU=FVector3d(1,0,0);value.Plane.AxisV=FVector3d(0,-1,0);value.Plane.HalfExtents=FVector2d(4544,3520);
        for(const auto& bits:nativeBits)value.Plane.DomainVertices.Add(FVector2d(A::FromBits(bits[0]),A::FromBits(bits[1])));
        FDistanceBudget work{100000};auto center=Detail::Point(FVector3d(0,0,2048));
        TestTrue(TEXT("actual weakly convex native52-core admits strict footprint interior"),N::NativeInterior(value,center,work));
        auto box=center;box.Axis[0]={-10,10,true};box.Axis[1]={-10,10,true};
        TestTrue(TEXT("whole interior rectangle remains inside native polygon"),N::NativeInterior(value,box,work));
        FCanonicalAxialPlaneBounds simple;simple.NormalAxis=2;simple.UAxis=0;simple.VAxis=1;
        simple.Plane.AxisU=FVector3d(1,0,0);simple.Plane.AxisV=FVector3d(0,1,0);simple.Plane.HalfExtents=FVector2d(3,3);
        simple.Plane.DomainVertices={FVector2d(-1,-1),FVector2d(0,-1),FVector2d(1,-1),FVector2d(1,1),FVector2d(-1,1)};
        TestTrue(TEXT("redundant collinear boundary point preserves strict interior"),N::NativeInterior(simple,Detail::Point(FVector3d(0,0,0)),work));
        TestFalse(TEXT("query on finite edge is still refused"),N::NativeInterior(simple,Detail::Point(FVector3d(1,0,0)),work));
        auto crossing=Detail::Point(FVector3d(.9,0,0));crossing.Axis[0]={.9,1.1,true};
        TestFalse(TEXT("query crossing finite edge is still refused"),N::NativeInterior(simple,crossing,work));
        auto bad=simple;bad.Plane.DomainVertices={FVector2d(-1,-1),FVector2d(1,-1),FVector2d(0,-1),FVector2d(1,1),FVector2d(-1,1)};
        TestFalse(TEXT("backtracking collinear run is not a convex footprint"),N::NativeInterior(bad,Detail::Point(FVector3d(0,0,0)),work));
        bad.Plane.DomainVertices={FVector2d(-2,-2),FVector2d(2,-2),FVector2d(2,-1),FVector2d(-1,-1),FVector2d(-1,2),FVector2d(-2,2)};
        TestFalse(TEXT("concave footprint remains refused"),N::NativeInterior(bad,Detail::Point(FVector3d(-1.5,-1.5,0)),work));
        bad.Plane.DomainVertices={FVector2d(-1,0),FVector2d(0,0),FVector2d(1,0)};
        TestFalse(TEXT("zero-area polygon remains refused"),N::NativeInterior(bad,Detail::Point(FVector3d(0,.5,0)),work));
        bad=simple;const FVector2d duplicate=bad.Plane.DomainVertices[0];bad.Plane.DomainVertices.Insert(duplicate,1);
        TestFalse(TEXT("zero-length edge remains refused"),N::NativeInterior(bad,Detail::Point(FVector3d(0,0,0)),work));
        TestTrue(TEXT("signed zero equality is exact literal equality"),N::SameFiniteNativeCoordinate(0.,A::FromBits(A::Sign)));
        TestFalse(TEXT("DAZ cannot equate unequal subnormal coordinates"),N::SameFiniteNativeCoordinate(A::FromBits(1),0.));
        TestFalse(TEXT("near collinearity without exact axis proof is not skipped"),N::ProveFixedAxisCollinearity(FVector2d(0,0),FVector2d(1,1),FVector2d(A::FromBits(1),0)));
        FDistanceBudget exhausted{0};TestFalse(TEXT("failed polygon work grants no interior"),N::NativeInterior(simple,Detail::Point(FVector3d(0,0,0)),exhausted));
    }
    TestTrue(TEXT("numeric context restored"),restore.Restore());return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGeneralBoxCorrelatedAxialPrefixTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.CorrelatedAxialFloorPrefix",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FGeneralBoxCorrelatedAxialPrefixTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;
    if(!TestTrue(TEXT("RNE fixture ready"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    // Captured 202 frame 114, serial 2, original H and actual consumed TOI.
    FGeneralBoxMotionInput v;
    v.Owner.Location=FVector(111.98430009549362,3664.8137588255531,28.044016230619178);
    v.Owner.Rotation=FQuat(-.75344863999514122,-.65750676525514495,-1.7397777354034861e-5,1.518239691925709e-5);
    v.Owner.Velocity=FVector(631.5,2100.4599609375,-40.67984886070294);
    v.Owner.Acceleration=FVector(0.,0.,-507.10281869240106);
    v.Owner.AngularVelocity=FVector(-.71147928011502759,.09720752157069526,-1.3012513900203059e-14);
    v.Owner.AngularAcceleration=FVector(-8.6161696360733231,1.1772043391151141,4.5025445302053438e-14);
    v.LocalCOM=FVector(-14.008570000000001,0.,17.415800000000001);
    v.LocalOffset=FVector(-5.,0.,29.510000000000002);
    v.LocalRotation=FQuat(0.,-0.,0.,1.);
    v.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
    v.QueryHalfExtent=FVector(65.660003662109375,43.585201263427734,15.947199821472168);
    v.Horizon=KinematicCenterDetail::FromBits(0x3b424130u);
    const float last=KinematicCenterDetail::FromBits(0x3886d1c8u);
    Interval gaps[8];
    const uint32 original=VectorGetControlRegister();
    ON_SCOPE_EXIT{VectorSetControlRegister(original);};
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((original&~uint32(0x8040))|flush);
        FDistanceBudget work{32};
        if(!TestTrue(TEXT("original small prefix is in the numeric domain"),
            CorrelatedAxialBoxFloorProof::Bound(v,last,work,gaps)))return false;
        TestEqual(TEXT("one finite proof debit"),work.Used,uint64(32));
        for(int32 k=0;k<8;++k)TestTrue(TEXT("physical corner positive throughout prefix"),gaps[k].Lower>0.);
        auto lower=v;lower.Owner.Location.Z-=4.e-12;work={32};
        if(!TestTrue(TEXT("lowered pose admits the same proof formula"),
            CorrelatedAxialBoxFloorProof::Bound(lower,last,work,gaps)))return false;
        TestTrue(TEXT("a penetration cannot acquire a positive bound"),gaps[4].Lower<0.&&gaps[6].Lower<0.);
        work={31};TestFalse(TEXT("no work cap extension"),CorrelatedAxialBoxFloorProof::Bound(v,last,work,gaps));
        work={32};TestFalse(TEXT("original full horizon exceeds the admitted domain"),
            CorrelatedAxialBoxFloorProof::Bound(v,v.Horizon,work,gaps));
    }
    return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FOrderedPhysicalChildBoxFloorTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.OrderedPhysicalChildBoxFloor",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FOrderedPhysicalChildBoxFloorTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;
    if(!TestTrue(TEXT("RNE fixture ready"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    FPhysicalBodyBoxPoseInput input;
    // Exact C22 case 202, frame 114, captured Box BASE request.
    input.BeforeCenter=FVector(113.24438476986943,3673.8744384667448,15.947200000293194);
    input.AfterCenter=FVector(113.24438476986272,3673.8744384666961,15.947200000001661);
    input.BeforeRotation=FQuat(-.7534486401960061,-.65750676543043229,-1.54449647872264e-12,1.3478911509844678e-12);
    input.AfterRotation=FQuat(-.7534486401960061,-.65750676543043229,4.0389678347315804e-28,-3.5340968553901329e-28);
    input.PhysicalHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
    const FVector point=FVector::ZeroVector,normal=FVector::UpVector;
    FDistanceBudget work{128};
    FClosedPhysicalBodyBoxFloorBoundsPtr old;
    TestFalse(TEXT("Cartesian corridor includes the location-first penetration"),
        TryProveClosedPhysicalBodyBoxFloor(input,point,normal,work,old));
    FIntervalPoint hull;
    TArray<FBoxVolumeGroup,TInlineAllocator<16>> groups;
    const uint32 saved=VectorGetControlRegister();
    ON_SCOPE_EXIT{VectorSetControlRegister(saved);};
    for(uint32 flush:{0u,64u,32768u,32832u})
    {
        VectorSetControlRegister((saved&~uint32(0x8040))|flush);work={128};
        if(!TestTrue(TEXT("Both actual rotation-first legs stay above the floor"),
            TryBuildRotationFirstPhysicalBoxFloor(input,point,normal,work,hull,groups)))return false;
        TestEqual(TEXT("Four rotation cells and one translation cell, raw and unit"),groups.Num(),10);
        auto penetrating=input;penetrating.AfterCenter.Z-=1.e-8;work={128};
        TestFalse(TEXT("An actually penetrating endpoint remains refused"),
            TryBuildRotationFirstPhysicalBoxFloor(penetrating,point,normal,work,hull,groups));
        TestTrue(TEXT("Failed geometry is not published"),groups.IsEmpty());
    }
    return !HasAnyErrors();
}


IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEstablishedWorldYawFloorClearTest,
 "IAmSpeed.AnalyticWorld.GeneralBox.EstablishedWorldYawFloorClear",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FEstablishedWorldYawFloorClearTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;
    if(!TestTrue(TEXT("RNE fixture ready"),context.Ready))return false;
    using namespace Speed::Analytic::Certified;
    FGeneralBoxMotionInput input;
    input.Owner.Location=FVector(113.24438476986272,3673.8744384666961,15.947200000001661);
    input.Owner.Rotation=FQuat(-.7534486401960061,-.65750676543043229,4.0389678347315804e-28,-3.5340968553901329e-28);
    input.Owner.AngularVelocity=FVector(0,0,.5);
    input.PhysicalHalfExtent=input.QueryHalfExtent=FVector(65.659999999999997,43.5852,15.9472);
    input.Horizon=.0028998036868870258f;
    const FVector point=FVector::ZeroVector,normal=FVector::UpVector;
    FDistanceBudget work{1024};
    FGeneralBoxPhysicalFloorClearBoundsPtr clear;
    if(!TestTrue(TEXT("complete directed inverted world-yaw clearance"),
        TryProveGeneralBoxPhysicalFloorClear(input,point,normal,work,clear,true)&&clear&&
        clear->Matches(input,point,normal)))return false;
    for(int32 k=0;k<16;++k)
        TestTrue(TEXT("every actual physical corner strictly clears"),clear->GetMinimumGapForInspection(k).Lower>0.);
    auto closing=input;
    closing.Owner.Acceleration.Z=-1.;
    FDistanceBudget reject{1024};
    TestFalse(TEXT("genuine vertical closing remains unproved"),
        TryProveGeneralBoxPhysicalFloorClear(closing,point,normal,reject,clear,true)||clear.IsValid());
    return true;
}
