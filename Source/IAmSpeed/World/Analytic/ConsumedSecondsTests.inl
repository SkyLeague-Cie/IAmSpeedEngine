#include "IndexedIntervalUnion.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticConsumedFloatSecondsRangeTest,
 "IAmSpeed.AnalyticWorld.ConsumedSeconds.DirectRangeAndNoEndpointShortcut",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticConsumedFloatSecondsRangeTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    const FVector3d x(10,-4,1),v(128,3,-5),a(4,-8,0.125);const float delta=1.f/300.f;
    const uint32 controls=VectorGetControlRegister();
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);FZeroOffsetKinematicCenterReference model;
        TestTrue(TEXT("fixed model in all four allowed flush modes"),BuildZeroOffsetKinematicCenterReference(x,v,a,delta,FVector3d::ZeroVector,FVector3d::ZeroVector,model));
        const float first=delta/8.f,last=delta*0.75f;
        const auto whole=BoundConsumedCenterBetweenFloatSeconds(model,first,last);
        TestTrue(TEXT("direct consumed seconds hull available"),Detail::ValidPoint(whole));
        for(float seconds:{first,(first+last)*0.5f,last})
        {
            volatile float squared=seconds*seconds;
            const FVector3d actual=x+v*double(seconds)+0.5*a*double(squared);
            const auto at=BoundConsumedCenterBetweenFloatSeconds(model,seconds,seconds);
            for(int32 d=0;d<3;++d)TestTrue(TEXT("actual vector64 DAG enclosed with original float seconds"),
                whole.Axis[d].Lower<=actual[d]&&whole.Axis[d].Upper>=actual[d]&&at.Axis[d].Lower<=actual[d]&&at.Axis[d].Upper>=actual[d]);
        }
        TestFalse(TEXT("reversed seconds range refused"),Detail::ValidPoint(BoundConsumedCenterBetweenFloatSeconds(model,last,first)));
        TestFalse(TEXT("seconds after fixed delta refused"),Detail::ValidPoint(BoundConsumedCenterBetweenFloatSeconds(model,0.f,delta*2.f)));
        TestFalse(TEXT("negative zero endpoint explicitly refused"),Detail::ValidPoint(BoundConsumedCenterBetweenFloatSeconds(model,KinematicCenterDetail::FromBits(0x80000000u),last)));
        TestFalse(TEXT("subnormal time cannot turn into zero under DAZ"),Detail::ValidPoint(BoundConsumedCenterBetweenFloatSeconds(model,KinematicCenterDetail::FromBits(1),last)));
        TestFalse(TEXT("non-finite time refused before arithmetic"),Detail::ValidPoint(BoundConsumedCenterBetweenFloatSeconds(model,0.f,KinematicCenterDetail::FromBits(0x7f800000u))));
    }
    FZeroOffsetKinematicCenterReference returning;
    TestTrue(TEXT("returning model"),BuildZeroOffsetKinematicCenterReference(FVector3d(0,0,1),FVector3d(0,0,-4),FVector3d(0,0,8),1.f,FVector3d::ZeroVector,FVector3d::ZeroVector,returning));
    FIntervalTensorSurface plane;Detail::PromoteTensor(TimeUnitPlane(),plane);TArray<FIntervalTensorSurface> surfaces={plane};
    FDistanceBudget budget{65536};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("same complete owned plane union"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),budget,scene)))return false;
    TestTrue(TEXT("last endpoint clear"),ProveConsumedCenterFloatSecondsRangeClear(scene,returning,1.f,1.f,0.25,budget,1.e-7,16));
    TestTrue(TEXT("short whole consumed prefix clear"),ProveConsumedCenterFloatSecondsRangeClear(scene,returning,0.f,1.f/16.f,0.25,budget,1.e-7,16));
    TestFalse(TEXT("clear endpoint never certifies full crossing range"),ProveConsumedCenterFloatSecondsRangeClear(scene,returning,0.f,1.f,0.25,budget,1.e-7,16));
    TestFalse(TEXT("middle singleton on plane is not clear"),ProveConsumedCenterFloatSecondsRangeClear(scene,returning,0.5f,0.5f,0.25,budget,1.e-7,16));
    FDistanceBudget none{0};TestFalse(TEXT("exhausted scan cannot become miss"),ProveConsumedCenterFloatSecondsRangeClear(scene,returning,1.f,1.f,0.25,none,1.e-7,16));
    FIndexedIntervalUnionPtr absent;TestFalse(TEXT("absent scene cannot become clear"),ProveConsumedCenterFloatSecondsRangeClear(absent,returning,1.f,1.f,0.25,budget,1.e-7,16));
    bool refused=false;VectorSetControlRegister((controls&~uint32(0x6000))|0x2000u);
    try{ProveConsumedCenterFloatSecondsRangeClear(scene,returning,1.f,1.f,0.25,budget,1.e-7,16);}catch(const FAnalyticNumericUnknown&){refused=true;}
    VectorSetControlRegister(controls);TestTrue(TEXT("invalid FP mode terminal before any proof use"),refused);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand ConsumedSecondsUnits(TEXT("Speed.Analytic.RunConsumedSecondsUnits"),TEXT("Run direct consumed binary32 seconds bounds/refusal unit; no World authority."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool Used=false;auto& f=FAutomationTestFramework::Get();
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticConsumedSecondsUnits"))||Used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Consumed seconds units refused"));return;}Used=true;
    const TCHAR* type=TEXT("FAnalyticConsumedFloatSecondsRangeTest");const TCHAR* name=TEXT("IAmSpeed.AnalyticWorld.ConsumedSeconds.DirectRangeAndNoEndpointShortcut");
    if(!f.ContainsTest(type)){UE_LOG(LogTemp,Error,TEXT("Consumed seconds test unavailable"));return;}
    f.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);f.StartTestByName(type,0,name);
    if(!GIsAutomationTesting||!f.GetCurrentTest()||f.GetCurrentTestFullPath()!=name||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Consumed seconds synchronous ownership refused"));return;}
    FAutomationTestExecutionInfo info;const bool stopped=f.StopTest(info);
    if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Consumed seconds final ownership refused"));return;}
    UE_LOG(LogTemp,Display,TEXT("ANALYTIC_CONSUMED_SECONDS_UNIT name=%s passed=%d errors=%d warnings=%d"),name,stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0,info.GetErrorTotal(),info.GetWarningTotal());
 }));
}
