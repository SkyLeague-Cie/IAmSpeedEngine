#include "IndexedIntervalUnion.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticFirstContactLeavesTest,
 "IAmSpeed.AnalyticWorld.ContactLeaves.AllOwnersRemappedNoPartialOutput",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticFirstContactLeavesTest::RunTest(const FString&)
{
    FTensorBoundsTestContext context;if(!TestTrue(TEXT("RNE fixture established"),context.Ready))return false;
    using namespace Speed::Analytic;using namespace Speed::Analytic::Certified;
    FIntervalTensorSurface far,plane;far.ControlPoints.Add(Detail::Point(FVector3d(100,100,100)));
    plane.DegreeU=1;plane.DegreeV=1;
    for(const auto& p:TArray<FVector3d>{FVector3d(-2,-2,0),FVector3d(-2,2,0),FVector3d(2,-2,0),FVector3d(2,2,0)})plane.ControlPoints.Add(Detail::Point(p));
    TArray<FIntervalTensorSurface> surfaces={far,plane,far,plane};FDistanceBudget budget{1048576};FIndexedIntervalUnionPtr scene;
    if(!TestTrue(TEXT("complete mathematical union"),FIndexedIntervalUnion::Build(MakeArrayView(surfaces),budget,scene)))return false;
    FIntervalBezierCenterPath path;CenterDetail::Linear(FVector3d(0,0,2),FVector3d::ZeroVector,path);FReferenceCandidateSetPtr proof;
    if(!TestTrue(TEXT("complete candidate partition"),FReferenceCandidateSet::Select(scene,path,1.,budget,proof)))return false;
    const auto indices=proof->GetCandidateIndices();TestTrue(TEXT("sparse global indices1and3 retained"),indices.Num()==2&&indices[0]==1&&indices[1]==3);
    const uint64 usedBeforeContact=budget.Used;
    const auto hit=proof->Contact(budget,1./1048576.,1.e-7,512,0.001,0.001);
    const uint64 contactCost=budget.Used-usedBeforeContact;
    if(TestTrue(TEXT("first contact and every possible owner enclosed"),hit.Status==ETensorSphereContactStatus::FirstContactEnclosed))
    {
        TSet<int32> owners;TestTrue(TEXT("accepted leaves published"),!hit.Leaves.IsEmpty());
        for(const auto& leaf:hit.Leaves)
        {
            owners.Add(leaf.SurfaceIndex);TestTrue(TEXT("no local index or later-inside-only substitution"),leaf.SurfaceIndex==1||leaf.SurfaceIndex==3);
            TestTrue(TEXT("fixed dyadic sample is in retained domain"),leaf.FixedParameter.X>=leaf.MinU&&leaf.FixedParameter.X<=leaf.MaxU&&leaf.FixedParameter.Y>=leaf.MinV&&leaf.FixedParameter.Y<=leaf.MaxV);
            const double exact[3]={4.*leaf.FixedParameter.X-2.,4.*leaf.FixedParameter.Y-2.,0.};
            for(int32 d=0;d<3;++d)TestTrue(TEXT("fixed-parameter evaluation has its own enclosure, not a root claim"),leaf.FixedParameterPoint.Axis[d].Known&&leaf.FixedParameterPoint.Axis[d].Lower<=exact[d]&&leaf.FixedParameterPoint.Axis[d].Upper>=exact[d]);
        }
        TestTrue(TEXT("both coincident providers survive, not only inside witness"),owners.Num()==2&&owners.Contains(1)&&owners.Contains(3));
        for(int32 d=0;d<3;++d)TestTrue(TEXT("global hull contains true first point"),hit.SurfacePoint.Axis[d].Lower<=0&&hit.SurfacePoint.Axis[d].Upper>=0);
    }
    if(TestTrue(TEXT("successful Contact has measurable bounded cost"),contactCost>1&&contactCost<=uint64(MAX_int32)))
    {
        FDistanceBudget late{int32(contactCost-1)};
        const auto lateUnknown=proof->Contact(late,1./1048576.,1.e-7,512,0.001,0.001);
        TestTrue(TEXT("late budget exhaustion discards all accepted and remapped partial leaves"),lateUnknown.Status==ETensorSphereContactStatus::Unknown&&lateUnknown.Leaves.IsEmpty());
    }
    FDistanceBudget zero{0};const auto unknown=proof->Contact(zero,1./1048576.,1.e-7,512,0.001,0.001);
    TestTrue(TEXT("Unknown never exposes a partial/remapped leaf list"),unknown.Status==ETensorSphereContactStatus::Unknown&&unknown.Leaves.IsEmpty());
    TArray<FIntervalTensorSurface> onlyFar={far};FIndexedIntervalUnionPtr clearScene;FReferenceCandidateSetPtr clear;
    if(TestTrue(TEXT("far union and selection"),FIndexedIntervalUnion::Build(MakeArrayView(onlyFar),budget,clearScene)&&FReferenceCandidateSet::Select(clearScene,path,1.,budget,clear)))
    {
        const auto miss=clear->Contact(budget,1./1048576.,1.e-7,512,0.001,0.001);
        TestTrue(TEXT("proved clear is distinct from Unknown and has no leaves"),miss.Status==ETensorSphereContactStatus::NoContactWithProvidedTensors&&miss.Leaves.IsEmpty());
    }
    TestTrue(TEXT("original floating controls restored"),context.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand ContactLeavesUnit(TEXT("Speed.Analytic.RunContactLeavesUnit"),
 TEXT("One all-owner contact-leaf remap/refusal unit; no World, actor, save or physical admission."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool used=false;auto& framework=FAutomationTestFramework::Get();const TCHAR* type=TEXT("FAnalyticFirstContactLeavesTest");const TCHAR* name=TEXT("IAmSpeed.AnalyticWorld.ContactLeaves.AllOwnersRemappedNoPartialOutput");
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticContactLeavesUnit"))||used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty()||!framework.ContainsTest(type))
    {UE_LOG(LogTemp,Error,TEXT("Contact leaf unit refused"));return;}
    used=true;framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);framework.StartTestByName(type,0,name);
    if(!GIsAutomationTesting||!framework.GetCurrentTest()||framework.GetCurrentTestFullPath()!=name||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Contact leaf synchronous test ownership refused"));return;}
    FAutomationTestExecutionInfo info;const bool stopped=framework.StopTest(info);
    if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Contact leaf final test ownership refused"));return;}
    const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
    UE_LOG(LogTemp,Display,TEXT("ANALYTIC_CONTACT_LEAVES_UNIT name=%s passed=%d errors=%d warnings=%d"),name,passed,info.GetErrorTotal(),info.GetWarningTotal());
 }));
}
