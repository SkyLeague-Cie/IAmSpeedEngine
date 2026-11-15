#include "KinematicCenterPathBounds.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticQuadraticCenterContactTest,
 "IAmSpeed.AnalyticWorld.CenterPath.ContinuousQuadraticContact",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticQuadraticCenterContactTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FZeroOffsetKinematicCenterReference model;
    if(!TestTrue(TEXT("fixed zero-offset reference constructed"),BuildZeroOffsetKinematicCenterReference(FVector3d(0,0,1),FVector3d::ZeroVector,FVector3d(0,0,-4),1.f,FVector3d::ZeroVector,FVector3d::ZeroVector,model)))return false;
    const auto half=CenterDetail::Evaluate(model.GetReferenceCurve(),Interval::Exact(0.5));
    TestTrue(TEXT("quadratic center at half is z1/2, not linear-chord z0"),half.Axis[2].Lower<=0.5&&half.Axis[2].Upper>=0.5&&half.Axis[2].Lower>0.49);
    FIntervalTensorSurface plane;TestTrue(TEXT("plane promotes"),Detail::PromoteTensor(TimeUnitPlane(),plane));TArray<FIntervalTensorSurface> surfaces={plane};FDistanceBudget budget{131072};
    const auto r=BoundTensorSphereFirstContact(MakeArrayView(surfaces),model.GetReferenceCurve(),0.25,budget,1./65536.,1.e-7,256,0.01,0.05);
    TestTrue(TEXT("continuous quadratic contact is enclosed by same global kernel"),r.Status==ETensorSphereContactStatus::FirstContactEnclosed);
    if(r.Status==ETensorSphereContactStatus::FirstContactEnclosed)
    {
        const auto lo2=AnalyticCertified::Square(Interval::Exact(r.LowerTime)),hi2=AnalyticCertified::Square(Interval::Exact(r.UpperTime));
        TestTrue(TEXT("first root satisfies t^2=3/8 and differs from chord time3/8"),r.LowerTime>0.5&&lo2.Lower<=0.375&&hi2.Upper>=0.375&&r.UpperTime-r.LowerTime<=1./65536.);
        const double point[3]={0,0,0},normal[3]={0,0,1};for(int32 d=0;d<3;++d)
        {
            TestTrue(TEXT("actual reference contact origin enclosed"),r.SurfacePoint.Axis[d].Lower<=point[d]&&r.SurfacePoint.Axis[d].Upper>=point[d]);
            TestTrue(TEXT("reference radial normal+Z enclosed"),r.RadialUnitNormal.Axis[d].Lower<=normal[d]&&r.RadialUnitNormal.Axis[d].Upper>=normal[d]);
        }
    }
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticConsumedCenterEnvelopeTest,
 "IAmSpeed.AnalyticWorld.CenterPath.ConsumedFloatEnvelopeAndJump",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticConsumedCenterEnvelopeTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const FVector3d start(10,-4,1),velocity(128,3,-5),acceleration(4,-8,0.125);const float delta=1.f/300.f;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);FZeroOffsetKinematicCenterReference model;
        TestTrue(TEXT("fixed model in allowed flush mode"),BuildZeroOffsetKinematicCenterReference(start,velocity,acceleration,delta,FVector3d::ZeroVector,FVector3d::ZeroVector,model));
        for(double t:{0.,0.125,0.5,1.})
        {
            const float seconds=static_cast<float>(t*double(delta));volatile float squared=seconds*seconds;
            const FVector3d actual=start+velocity*double(seconds)+0.5*acceleration*double(squared);
            const auto bound=BoundConsumedCenterRange(model,Interval::Exact(t)),atSeconds=BoundConsumedCenterAtFloatSeconds(model,seconds);
            TestTrue(TEXT("actual rounded DAG has finite bounds"),Detail::ValidPoint(bound)&&Detail::ValidPoint(atSeconds));
            for(int32 d=0;d<3;++d)TestTrue(TEXT("actual float seconds/square then vector64 position enclosed"),bound.Axis[d].Lower<=actual[d]&&bound.Axis[d].Upper>=actual[d]&&atSeconds.Axis[d].Lower<=actual[d]&&atSeconds.Axis[d].Upper>=actual[d]);
        }
    }
    const float jumpDelta=KinematicCenterDetail::FromBits(0x3f800001u);FZeroOffsetKinematicCenterReference jump;
    TestTrue(TEXT("adjacent float-time model constructed"),BuildZeroOffsetKinematicCenterReference(FVector3d::ZeroVector,FVector3d(1,0,0),FVector3d::ZeroVector,jumpDelta,FVector3d::ZeroVector,FVector3d::ZeroVector,jump));
    const double t=(1.+1./16777216.)/double(jumpDelta);const auto consumed=BoundConsumedCenterRange(jump,Interval::Exact(t));const auto continuous=CenterDetail::Evaluate(jump.GetReferenceCurve(),Interval::Exact(t));
    TestTrue(TEXT("consumed float transition enclosure includes both adjacent states"),consumed.Axis[0].Lower<=1.&&consumed.Axis[0].Upper>=double(jumpDelta));
    TestTrue(TEXT("continuous reference enclosure is distinct and narrower than float jump"),continuous.Axis[0].Upper-continuous.Axis[0].Lower<double(jumpDelta)-1.);
    const double radius=1./33554432.;TestTrue(TEXT("outside-to-inside float jump has no equality state at the continuous sphere boundary"),double(jumpDelta)-1.>radius&&double(jumpDelta)-double(jumpDelta)<radius&&KinematicCenterDetail::Bits(jumpDelta)==KinematicCenterDetail::Bits(1.f)+1);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticCenterPrefixAndTimeFloorTest,
 "IAmSpeed.AnalyticWorld.CenterPath.PrefixFloorAndOffsetRefusals",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticCenterPrefixAndTimeFloorTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FZeroOffsetKinematicCenterReference model;
    TestTrue(TEXT("reference model constructed"),BuildZeroOffsetKinematicCenterReference(FVector3d::ZeroVector,FVector3d(1,0,0),FVector3d::ZeroVector,1.f/300.f,FVector3d::ZeroVector,FVector3d::ZeroVector,model));
    const double lower=0.123456789;const auto floor=FloorReferenceTimeToFloatSeconds(model,lower);const auto product=AnalyticCertified::Multiply(Interval::Exact(lower),Interval::Exact(double(1.f/300.f)));
    TestTrue(TEXT("candidate float seconds never exceeds directed lower bound on reference time"),floor.Known&&double(floor.Seconds)<=product.Lower);
    TestFalse(TEXT("negative time refuses"),FloorReferenceTimeToFloatSeconds(model,-0.1).Known);
    VectorSetControlRegister(VectorGetControlRegister()|0x40u);const FVector3d tiny(AnalyticCertified::FromBits(1),0,0);FZeroOffsetKinematicCenterReference invalid=model;
    TestFalse(TEXT("subnormal body offset never treated as zero under DAZ"),BuildZeroOffsetKinematicCenterReference(FVector3d::ZeroVector,FVector3d::ZeroVector,FVector3d::ZeroVector,1.f,tiny,FVector3d::ZeroVector,invalid));
    TestFalse(TEXT("body-offset refusal erased a previously known model"),invalid.IsKnown());invalid=model;
    TestFalse(TEXT("subnormal local COM never treated as zero under DAZ"),BuildZeroOffsetKinematicCenterReference(FVector3d::ZeroVector,FVector3d::ZeroVector,FVector3d::ZeroVector,1.f,FVector3d::ZeroVector,tiny,invalid));TestFalse(TEXT("refusal leaves unknown model"),invalid.IsKnown());
    FZeroOffsetKinematicCenterReference returning;
    TestTrue(TEXT("returning quadratic constructed"),BuildZeroOffsetKinematicCenterReference(FVector3d(0,0,1),FVector3d(0,0,-4),FVector3d(0,0,8),1.f,FVector3d::ZeroVector,FVector3d::ZeroVector,returning));
    FIntervalTensorSurface plane;TestTrue(TEXT("plane promotes"),Detail::PromoteTensor(TimeUnitPlane(),plane));TArray<FIntervalTensorSurface> surfaces={plane};const auto r2=Interval::Exact(0.0625);
    FDistanceBudget endpointBudget{16},prefixBudget{16};const auto endpoint=TimeDetail::Evaluate(MakeArrayView(surfaces),BoundConsumedCenterAtFloatSeconds(returning,1.f),endpointBudget,r2,1.e-7,1);
    const auto prefix=TimeDetail::Evaluate(MakeArrayView(surfaces),BoundConsumedCenterRange(returning,Interval{0,1,true}),prefixBudget,r2,1.e-7,1);
    TestTrue(TEXT("clear consumed endpoint is proved"),endpoint.Known&&endpoint.AllClear);
    TestTrue(TEXT("clear endpoint does not certify the consumed swept prefix"),prefix.Known&&!prefix.AllClear);
    volatile float half=0.5f,squared=half*half;const double actualMid=1.-4.*double(half)+4.*double(squared);TestEqual(TEXT("independent actual middle pose touches plane despite both endpoints clear"),actualMid,0.);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand CenterPathUnits(TEXT("Speed.Analytic.RunCenterPathUnits"),TEXT("Run three continuous-reference/consumed-DAG units; no World authority."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool Used=false;auto& f=FAutomationTestFramework::Get();
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticCenterPathUnits"))||Used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Center path units refused"));return;}Used=true;
    const TCHAR* classes[]={TEXT("FAnalyticQuadraticCenterContactTest"),TEXT("FAnalyticConsumedCenterEnvelopeTest"),TEXT("FAnalyticCenterPrefixAndTimeFloorTest")};
    const TCHAR* names[]={TEXT("IAmSpeed.AnalyticWorld.CenterPath.ContinuousQuadraticContact"),TEXT("IAmSpeed.AnalyticWorld.CenterPath.ConsumedFloatEnvelopeAndJump"),TEXT("IAmSpeed.AnalyticWorld.CenterPath.PrefixFloorAndOffsetRefusals")};
    for(int32 i=0;i<3;++i)
    {
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()||!f.ContainsTest(classes[i])){UE_LOG(LogTemp,Error,TEXT("Center path unit ownership refused"));return;}
        f.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);f.StartTestByName(classes[i],0,names[i]);
        if(!GIsAutomationTesting||!f.GetCurrentTest()||f.GetCurrentTestFullPath()!=names[i]||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Center path unit synchronous ownership refused"));return;}
        FAutomationTestExecutionInfo info;const bool stopped=f.StopTest(info);
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Center path unit final ownership refused"));return;}
        const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
        UE_LOG(LogTemp,Display,TEXT("ANALYTIC_CENTER_PATH_UNIT name=%s passed=%d errors=%d warnings=%d"),names[i],passed,info.GetErrorTotal(),info.GetWarningTotal());
    }
 }));
}
