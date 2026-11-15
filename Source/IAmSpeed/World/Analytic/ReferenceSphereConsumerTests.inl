#include "ReferenceSphereConsumer.h"
#include "IAmSpeed/World/Collision/PreparedSphereReferenceConsumer.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticReferenceSphereConsumerTest,
 "IAmSpeed.AnalyticWorld.Prepared.ReferenceSphereConsumer",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticReferenceSphereConsumerTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FIntervalTensorSurface plane;if(!TestTrue(TEXT("finite plane mathematical fixture"),Detail::PromoteTensor(TimeUnitPlane(),plane)))return false;
    TArray<FIntervalTensorSurface> surfaces={plane};FDistanceBudget build{1024};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("complete mathematical union built"),FIndexedIntervalUnion::Build(surfaces,build,scene)))return false;
    FReferenceSphereOptions options;options.TimeGap=1./65536.;options.SpatialGap=1.e-7;options.SpatialNodesPerAttempt=512;
    options.PointWidth=0.01;options.NormalWidth=0.05;options.PrefixRangeAttempts=256;
    FIntervalTensorSurface bowl,far;TestTrue(TEXT("curved polynomial mathematical fixture"),Detail::PromoteTensor(ContactUnitBowl(),bowl));
    far.ControlPoints.Add(Detail::Point(FVector3d(100,100,100)));TArray<FIntervalTensorSurface> curvedSurfaces={far,bowl,far};
    FDistanceBudget curvedBuild{1024};FIndexedIntervalUnionPtr curvedScene;
    if(!TestTrue(TEXT("complete curved union with excluded providers"),FIndexedIntervalUnion::Build(curvedSurfaces,curvedBuild,curvedScene)))return false;
    FDistanceBudget positiveBudget{1048576};int32 checked=0;
    const auto positive=EvaluateReferenceSphere(curvedScene,FVector3d(0,0,1),FVector3d::ZeroVector,FVector3d(0,0,-4),1.f,0.25,0.125,
        options,positiveBudget,[&](int32 i){++checked;return i==1;});
    TestTrue(TEXT("quadratic reference contact plus complete consumed-prefix proof"),positive.Status==EReferenceSphereStatus::ReferenceContactBounded&&positive.ConsumedPrefixClear);
    if(positive.Status==EReferenceSphereStatus::ReferenceContactBounded)
    {
        TestTrue(TEXT("candidate policy checked and all owners retained"),checked==1&&positive.Candidates&&positive.Candidates->GetCandidateIndices().Num()==1&&positive.Candidates->GetExcludedIndices().Num()==2&&positive.Contact.Leaves.Num()>0);
        for(const auto& leaf:positive.Contact.Leaves)TestEqual(TEXT("every curved contact leaf remaps to global surface index"),leaf.SurfaceIndex,1);
        TestTrue(TEXT("reference root differs from endpoint chord"),positive.Contact.LowerTime>0.5&&positive.Contact.UpperTime<0.7);
        const auto actual=Speed::SBox::AdvancePosition(FVector3d(0,0,1),FVector3d::ZeroVector,FVector3d(0,0,-4),positive.ConsumedSeconds);
        for(int32 d=0;d<3;++d)TestTrue(TEXT("scalar center retains actual SBox operation bits"),AnalyticCertified::Bits(positive.Scalar.Location[d])==AnalyticCertified::Bits(actual[d]));
        TestTrue(TEXT("collision radius bits and distinct physical response arm retained"),positive.Contact.CollisionRadiusBits==AnalyticCertified::Bits(0.25)&&positive.Scalar.RadialResponsePoint.Z>0.12);
        FScalarSphereContact wrongRadius=positive.Scalar;const double adjacent=double(KinematicCenterDetail::FromBits(KinematicCenterDetail::Bits(0.25f)+1));
        TestTrue(TEXT("one float ULP radius change refuses and clears scalar output"),ConvertSphereContact(positive.Contact,0,positive.Scalar.ImpactPoint,positive.Scalar.Location,adjacent,0.125,wrongRadius)==EScalarSphereContactStatus::Unknown&&wrongRadius.RepresentativeSurfaceIndex==INDEX_NONE);
    }
    FDistanceBudget deniedBudget{1048576};int32 deniedChecks=0;
    const auto denied=EvaluateReferenceSphere(scene,FVector3d(0,0,1),FVector3d::ZeroVector,FVector3d(0,0,-4),1.f,0.25,0.125,
        options,deniedBudget,[&](int32){++deniedChecks;return false;});
    TestTrue(TEXT("candidate filter refusal never becomes a later hit or a miss"),denied.Status==EReferenceSphereStatus::Unknown&&!denied.Candidates&&!denied.ConsumedPrefixClear&&deniedChecks==1);
    FDistanceBudget insideBudget{1048576};
    const auto inside=EvaluateReferenceSphere(scene,FVector3d(0,0,0.2),FVector3d::ZeroVector,FVector3d::ZeroVector,1.f,0.25,0.125,
        options,insideBudget,[](int32){return true;});
    TestTrue(TEXT("inside inflated tube while outside physical radius emits no MTD or fake zero-time hit"),inside.Status==EReferenceSphereStatus::InitialInsideTube&&!inside.ConsumedPrefixClear&&inside.Scalar.RepresentativeSurfaceIndex==INDEX_NONE);
    FDistanceBudget clearBudget{1048576};int32 excludedChecks=0;
    const auto clear=EvaluateReferenceSphere(scene,FVector3d(0,0,3),FVector3d(0,0,1),FVector3d::ZeroVector,1.f,0.25,0.125,
        options,clearBudget,[&](int32){++excludedChecks;return false;});
    TestTrue(TEXT("complete geometric exclusions plus separate consumed range prove reference no-contact"),clear.Status==EReferenceSphereStatus::ReferenceNoContact&&clear.ConsumedPrefixClear&&clear.Candidates->GetExcludedIndices().Num()==1&&excludedChecks==0);
    FDistanceBudget exhausted{0};const auto unknown=EvaluateReferenceSphere(scene,FVector3d(0,0,3),FVector3d::ZeroVector,FVector3d::ZeroVector,1.f,0.25,0.125,options,exhausted,[](int32){return true;});
    TestTrue(TEXT("exhaustion publishes no stale result or clear prefix"),unknown.Status==EReferenceSphereStatus::Unknown&&!unknown.Candidates&&!unknown.ConsumedPrefixClear);
    FZeroOffsetKinematicCenterReference returning;TestTrue(TEXT("returning quadratic built"),BuildZeroOffsetKinematicCenterReference(FVector3d(0,0,1),FVector3d(0,0,-4),FVector3d(0,0,8),1.f,FVector3d::ZeroVector,FVector3d::ZeroVector,returning));
    FDistanceBudget prefixBudget{1048576};
    TestFalse(TEXT("both clear endpoints never prove the middle of a returning quadratic clear"),ReferenceSphereDetail::PrefixClear(scene,returning,1.f,0.25,options,prefixBudget));
    FDistanceBudget unownedBudget{1024};
    const auto unowned=Speed::PreparedExecution::EvaluateNativeSphereReference({},options,unownedBudget);
    TestTrue(TEXT("mathematical fixture cannot fabricate authentic native request"),!unowned.Request&&!unowned.Scene&&unowned.Reference.Status==EReferenceSphereStatus::Unknown);
    TestTrue(TEXT("original floating context restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand ReferenceSphereConsumerUnit(TEXT("Speed.Analytic.RunReferenceSphereConsumerUnit"),
 TEXT("Complete reference-contact chain unit; no World, actor, save or physical admission."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool used=false;auto& framework=FAutomationTestFramework::Get();const TCHAR* type=TEXT("FAnalyticReferenceSphereConsumerTest");const TCHAR* name=TEXT("IAmSpeed.AnalyticWorld.Prepared.ReferenceSphereConsumer");
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticReferenceSphereConsumerUnit"))||used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty()||!framework.ContainsTest(type))
    {UE_LOG(LogTemp,Error,TEXT("Prepared native execution unit refused"));return;}
    used=true;framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);framework.StartTestByName(type,0,name);
    if(!GIsAutomationTesting||!framework.GetCurrentTest()||framework.GetCurrentTestFullPath()!=name||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Prepared native synchronous test ownership refused"));return;}
    FAutomationTestExecutionInfo info;const bool stopped=framework.StopTest(info);
    if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Prepared native final test ownership refused"));return;}
    const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
    UE_LOG(LogTemp,Display,TEXT("ANALYTIC_REFERENCE_SPHERE_CONSUMER_UNIT name=%s passed=%d errors=%d warnings=%d"),name,passed,info.GetErrorTotal(),info.GetWarningTotal());
 }));
}
