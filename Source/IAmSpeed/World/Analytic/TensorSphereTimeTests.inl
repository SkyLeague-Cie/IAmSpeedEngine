
#include "TensorSphereTimeBounds.h"
namespace
{
Speed::Analytic::FTensorBezierSurface TimeUnitPlane()
{
    Speed::Analytic::FTensorBezierSurface p;p.DegreeU=p.DegreeV=1;
    p.ControlPoints={FVector3d(-2,-2,0),FVector3d(-2,2,0),FVector3d(2,-2,0),FVector3d(2,2,0)};
    return p;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorIntervalCenterTest,
 "IAmSpeed.AnalyticWorld.TensorTime.IntervalCenterModel",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorIntervalCenterTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const double next=AnalyticCertified::FromBits(0x3ff0000000000001ULL);
    const auto cm=Detail::Half(Detail::Point(FVector3d(1,0,0)),Detail::Point(FVector3d(next,0,0)));
    TestTrue(TEXT("interval midpoint covers exact unrepresentable1+2^-53"),cm.Axis[0].Lower<=1.&&cm.Axis[0].Upper>=next);
    FTensorBezierSurface point;point.ControlPoints.Add(FVector3d(next,0,0));
    FDistanceBudget b{40};const auto r=BoundTensorDistanceSquaredIntervalCenter(point,cm,b,0.,1);
    const double exactDistanceSquared=AnalyticCertified::FromBits(0x3950000000000000ULL); //2^-106
    TestTrue(TEXT("distance bounds cover exact center-to-point squared2^-106"),r.Status!=EDistanceBoundStatus::Unknown&&r.LowerSquared<=exactDistanceSquared&&r.UpperSquared>=exactDistanceSquared);
    TestEqual(TEXT("spatial per-attempt limit debits only actual work and preserves global remainder"),b.Used,uint64(1));
    TestEqual(TEXT("unused budget is never reserved/refunded"),b.Remaining,39);
    TestTrue(TEXT("irreducible center interval yields Bounded, not fake convergence"),r.Status==EDistanceBoundStatus::Bounded);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorSphereTimeCrossingTest,
 "IAmSpeed.AnalyticWorld.TensorTime.TransverseAndFiniteDomain",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorSphereTimeCrossingTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    TArray<FTensorBezierSurface> surfaces={TimeUnitPlane()};
    FDistanceBudget b{4096};const auto r=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,1),FVector3d(0,0,-1),0.25,b,1./64.,1.e-8,1);
    TestTrue(TEXT("exact dyadic touch3/8 followed by inside has a certified bracket"),r.Status==ETensorSphereTimeStatus::FirstContactBracket);
    TestTrue(TEXT("first boundary time3/8 is contained"),r.LowerTime<=0.375&&r.UpperTime>=0.375&&r.UpperTime-r.LowerTime<=1./64.);
    TestTrue(TEXT("initial separation is proved, endpoint samples alone were both outside"),r.InitialSeparationProved);
    TestTrue(TEXT("shared budget includes temporal and provider work"),b.Used>1&&b.Used<=4096);
    FTensorBezierSurface finite;finite.DegreeU=finite.DegreeV=1;
    finite.ControlPoints={FVector3d(0,0,0),FVector3d(0,1,0),FVector3d(1,0,0),FVector3d(1,1,0)};
    surfaces={finite};FDistanceBudget finiteBudget{128};const auto no=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(2,0.5,1),FVector3d(2,0.5,-1),0.25,finiteBudget,1./64.,1.e-8,1);
    TestTrue(TEXT("finite patch excludes intersection although the infinite plane crosses"),no.Status==ETensorSphereTimeStatus::NoIntersectionWithProvidedTensors);
    surfaces={TimeUnitPlane()};FDistanceBudget initialBudget{64};const auto initial=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,0),FVector3d(0,0,1),0.25,initialBudget,1./64.,1.e-8,1);
    TestTrue(TEXT("initial inside is distinct from CCD and is not a penetration correction"),initial.Status==ETensorSphereTimeStatus::InitialInsideTube&&initial.UpperTime==0.);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticTensorSphereTimeUnknownTest,
 "IAmSpeed.AnalyticWorld.TensorTime.UnknownPrefixAndBudget",
 EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FAnalyticTensorSphereTimeUnknownTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FTensorBezierSurface touch;touch.ControlPoints.Add(FVector3d(0.25,0.125,0));
    FTensorBezierSurface later;later.ControlPoints.Add(FVector3d(0.75,0,0));
    TArray<FTensorBezierSurface> surfaces={touch,later};FDistanceBudget b{8192};const auto r=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,0),FVector3d(1,0,0),0.125,b,1./32.,1.e-8,1);
    TestTrue(TEXT("earlier tangency is never skipped to publish later piercing"),r.Status==ETensorSphereTimeStatus::Unknown);
    TestTrue(TEXT("earliest unresolved prefix remains before the tangency quarter"),r.InitialSeparationProved&&r.LowerTime<=0.25);
    surfaces={later,touch};FDistanceBudget reverseBudget{8192};const auto reverse=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,0),FVector3d(1,0,0),0.125,reverseBudget,1./32.,1.e-8,1);
    TestTrue(TEXT("provider permutation cannot erase the earlier unresolved touch"),reverse.Status==ETensorSphereTimeStatus::Unknown);
    FDistanceBudget empty{0};const auto stopped=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,0),FVector3d(1,0,0),0.125,empty,1./32.,1.e-8,1);
    TestTrue(TEXT("empty global budget cannot expand any time/provider tree"),stopped.Status==ETensorSphereTimeStatus::Unknown&&empty.Used==0);
    FDistanceBudget invalid{16};VectorSetControlRegister(VectorGetControlRegister()|0x40u);const auto bad=BoundTensorSphereSweep(MakeArrayView(surfaces),FVector3d(0,0,0),FVector3d(1,0,0),0.125,invalid,AnalyticCertified::FromBits(0x8000000000000001ULL),1.e-8,1);
    TestTrue(TEXT("negative subnormal temporal tolerance refused under DAZ"),bad.Status==ETensorSphereTimeStatus::Unknown&&invalid.Used==0);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand TensorTimeUnits(TEXT("Speed.Analytic.RunTensorTimeUnits"),
    TEXT("Run three existing-framework tensor time bound tests in an isolated commandlet."),
    FConsoleCommandDelegate::CreateLambda([]()
    {
        static bool Used=false;
        auto& Framework=FAutomationTestFramework::Get();
        if (!IsRunningCommandlet() || !FParse::Param(FCommandLine::Get(),TEXT("AnalyticTensorTimeUnits")) ||
            Used || GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld ||
            Framework.GetCurrentTest() || !Framework.IsLatentCommandQueueEmpty())
        { UE_LOG(LogTemp,Error,TEXT("Analytic tensor time unit commandlet refused")); return; }
        Used=true;
        const TCHAR* Classes[]={TEXT("FAnalyticTensorIntervalCenterTest"),TEXT("FAnalyticTensorSphereTimeCrossingTest"),TEXT("FAnalyticTensorSphereTimeUnknownTest")};
        const TCHAR* Names[]={TEXT("IAmSpeed.AnalyticWorld.TensorTime.IntervalCenterModel"),TEXT("IAmSpeed.AnalyticWorld.TensorTime.TransverseAndFiniteDomain"),TEXT("IAmSpeed.AnalyticWorld.TensorTime.UnknownPrefixAndBudget")};
        for (int32 I=0; I<3; ++I)
        {
            if (GIsAutomationTesting || GIsSlowTask || GIsPlayInEditorWorld || Framework.GetCurrentTest() ||
                !Framework.IsLatentCommandQueueEmpty() || !Framework.ContainsTest(Classes[I]))
            { UE_LOG(LogTemp,Error,TEXT("Analytic tensor time unit ownership refused")); return; }
            Framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);
            Framework.StartTestByName(Classes[I],0,Names[I]);
            if (!GIsAutomationTesting || !Framework.GetCurrentTest() || Framework.GetCurrentTestFullPath()!=Names[I] || !Framework.IsLatentCommandQueueEmpty())
            { UE_LOG(LogTemp,Error,TEXT("Analytic tensor time unit synchronous ownership refused")); return; }
            FAutomationTestExecutionInfo Info;
            const bool Stopped=Framework.StopTest(Info);
            const bool Passed=Stopped && Info.bSuccessful && Info.GetErrorTotal()==0 && Info.GetWarningTotal()==0;
            UE_LOG(LogTemp,Display,TEXT("ANALYTIC_TENSOR_TIME_UNIT name=%s passed=%d errors=%d warnings=%d"),Names[I],Passed,Info.GetErrorTotal(),Info.GetWarningTotal());
        }
    }));
}
