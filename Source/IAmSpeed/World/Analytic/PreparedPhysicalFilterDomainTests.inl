#include "IAmSpeed/World/Collision/PreparedPhysicalQueryDomain.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPreparedPhysicalFilterDomainTest,
 "IAmSpeed.AnalyticWorld.Prepared.PhysicalFilterDomain",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPreparedPhysicalFilterDomainTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::Analytic;
    int ownerA=0,ownerB=0,emptyOwner=0,otherEmptyOwner=0,bodyA=0,bodyB=0,unknownBody=0;
    TArray<FPreparedPhysicalFilterRecord> records={{&ownerA,&bodyA,TEXT("BodyA"),uint8(ECC_WorldDynamic),uint64(1)<<ECC_WorldStatic},
        {&ownerB,&bodyB,TEXT("BodyB"),uint8(ECC_WorldDynamic),0}};
    TArray<const void*> adapters={&ownerA,&ownerB,&emptyOwner};
    FPreparedPhysicalQueryDomainPtr domain;
    if(!TestTrue(TEXT("effective body inventory freezes at its owned generation"),FPreparedPhysicalQueryDomain::Build(7,records,adapters,domain)))return false;
    TestTrue(TEXT("WorldStatic participates even when every trace response ignores"),FPreparedPhysicalQueryDomain::PrimitiveCanParticipate(ECC_WorldStatic,0,domain->GetRecords()));
    TestFalse(TEXT("Pawn blocking channels is excluded by actual query-side object response"),FPreparedPhysicalQueryDomain::PrimitiveCanParticipate(ECC_Pawn,MAX_uint64,domain->GetRecords()));
    TestFalse(TEXT("Vehicle camera/visibility/high-bit responses do not enter the actual physical filter"),FPreparedPhysicalQueryDomain::PrimitiveCanParticipate(ECC_Vehicle,(uint64(1)<<ECC_Camera)|(uint64(1)<<ECC_Visibility)|0xFFFFFFFF00000000ull,domain->GetRecords()));
    auto widened=records;widened[0].BlockingObjectTypes|=uint64(1)<<ECC_Pawn;
    TestTrue(TEXT("widened actual query-side response includes Pawn without type exemption"),FPreparedPhysicalQueryDomain::PrimitiveCanParticipate(ECC_Pawn,uint64(1)<<ECC_WorldDynamic,widened));
    widened[0].TraceChannel=63;widened[0].BlockingObjectTypes=uint64(1)<<ECC_Vehicle;
    TestTrue(TEXT("high channel is preserved when an actual body uses it"),FPreparedPhysicalQueryDomain::PrimitiveCanParticipate(ECC_Vehicle,uint64(1)<<63,widened));
    TestFalse(TEXT("primitive-side response must also block a trace"),FPreparedPhysicalQueryDomain::PrimitiveCanParticipate(ECC_Vehicle,0,widened));
    TestTrue(TEXT("unordered same inventory matches without selecting contact order"),domain->Matches({records[1],records[0]},{&emptyOwner,&ownerB,&ownerA}));
    TestFalse(TEXT("changed policy refuses readback"),domain->Matches(widened,adapters));
    TestFalse(TEXT("removed body refuses readback"),domain->Matches({records[0]},adapters));
    auto replaced=records;replaced[0].Body=&unknownBody;
    TestFalse(TEXT("same path cannot hide replaced body"),domain->Matches(replaced,adapters));
    TestTrue(TEXT("zero-body adapter retained as separate membership"),domain->GetAdapterCount()==3&&domain->GetEmptyAdapterCount()==1);
    TestFalse(TEXT("removing empty adapter refuses even with identical bodies"),domain->Matches(records,{&ownerA,&ownerB}));
    TestFalse(TEXT("replacing empty adapter refuses even with identical bodies"),domain->Matches(records,{&ownerA,&ownerB,&otherEmptyOwner}));
    auto firstBody=records;firstBody.Add({&emptyOwner,&unknownBody,TEXT("NewBody"),uint8(ECC_WorldDynamic),0});
    TestFalse(TEXT("empty adapter adding its first body invalidates readback"),domain->Matches(firstBody,adapters));
    FWorldQuery query;query.bApplyCollisionFilter=true;query.TraceChannel=ECC_WorldDynamic;query.BlockingObjectTypes=uint64(1)<<ECC_WorldStatic;
    TestFalse(TEXT("zero-body adapter cannot mint query coverage"),domain->CoversPhysicalFilter(true,7,&emptyOwner,nullptr,query));
    TestTrue(TEXT("actual physical pair is covered only with its owner/body/generation"),domain->CoversPhysicalFilter(true,7,&ownerA,&bodyA,query));
    TestFalse(TEXT("unknown or camera purpose is outside physical closure"),domain->CoversPhysicalFilter(false,7,&ownerA,&bodyA,query));
    TestFalse(TEXT("another generation cannot reuse closure"),domain->CoversPhysicalFilter(true,8,&ownerA,&bodyA,query));
    TestFalse(TEXT("another body cannot reuse closure"),domain->CoversPhysicalFilter(true,7,&ownerA,&unknownBody,query));
    TestFalse(TEXT("another owner cannot reuse closure"),domain->CoversPhysicalFilter(true,7,&ownerB,&bodyA,query));
    query.bApplyCollisionFilter=false;TestFalse(TEXT("unfiltered query refuses"),domain->CoversPhysicalFilter(true,7,&ownerA,&bodyA,query));query.bApplyCollisionFilter=true;
    query.TraceChannel=ECC_Camera;TestFalse(TEXT("a real camera channel is not the physical trace pair"),domain->CoversPhysicalFilter(true,7,&ownerA,&bodyA,query));query.TraceChannel=ECC_WorldDynamic;
    query.BlockingObjectTypes|=uint64(1)<<ECC_Pawn;TestFalse(TEXT("wider consumer response cannot reuse closure"),domain->CoversPhysicalFilter(true,7,&ownerA,&bodyA,query));
    query.bObjectQuery=true;query.ObjectTypes=uint64(1)<<ECC_WorldStatic;TestTrue(TEXT("WorldStatic projection object query is covered"),domain->CoversPhysicalFilter(true,7,&ownerA,&bodyA,query));
    query.ObjectTypes=uint64(1)<<ECC_Pawn;TestFalse(TEXT("a new object-query purpose/type refuses"),domain->CoversPhysicalFilter(true,7,&ownerA,&bodyA,query));
    auto duplicate=records;duplicate.Add(records[0]);TestFalse(TEXT("duplicate actual body refuses atomic build"),FPreparedPhysicalQueryDomain::Build(7,duplicate,adapters,domain));TestFalse(TEXT("refusal leaves no stale domain"),domain.IsValid());
    TestFalse(TEXT("empty adapter inventory refuses"),FPreparedPhysicalQueryDomain::Build(7,{}, {},domain));
    TestTrue(TEXT("all-empty adapters can be frozen without granting any body query"),FPreparedPhysicalQueryDomain::Build(7,{}, {&emptyOwner},domain)&&domain->GetAdapterCount()==1&&domain->GetEmptyAdapterCount()==1&&!domain->CoversPhysicalFilter(true,7,&emptyOwner,nullptr,query));
    TestFalse(TEXT("duplicate adapters refuse"),FPreparedPhysicalQueryDomain::Build(7,records,{&ownerA,&ownerB,&ownerA},domain));
    TestFalse(TEXT("body owner outside membership refuses"),FPreparedPhysicalQueryDomain::Build(7,records,{&ownerA},domain));
    TestFalse(TEXT("zero generation refuses"),FPreparedPhysicalQueryDomain::Build(0,records,adapters,domain));
    return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand PreparedPhysicalFilterDomainUnit(TEXT("Speed.Analytic.RunPreparedPhysicalFilterDomainUnit"),
 TEXT("One physical filter-domain closure unit; no World, actor, save or physical admission."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool used=false;auto& framework=FAutomationTestFramework::Get();const TCHAR* type=TEXT("FPreparedPhysicalFilterDomainTest");const TCHAR* name=TEXT("IAmSpeed.AnalyticWorld.Prepared.PhysicalFilterDomain");
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticPhysicalFilterDomainUnit"))||used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty()||!framework.ContainsTest(type))
    {UE_LOG(LogTemp,Error,TEXT("Physical filter domain unit refused"));return;}
    used=true;framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);framework.StartTestByName(type,0,name);
    if(!GIsAutomationTesting||!framework.GetCurrentTest()||framework.GetCurrentTestFullPath()!=name||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Physical filter synchronous test ownership refused"));return;}
    FAutomationTestExecutionInfo info;const bool stopped=framework.StopTest(info);
    if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Physical filter final test ownership refused"));return;}
    const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
    UE_LOG(LogTemp,Display,TEXT("ANALYTIC_PHYSICAL_FILTER_DOMAIN_UNIT name=%s passed=%d errors=%d warnings=%d"),name,passed,info.GetErrorTotal(),info.GetWarningTotal());
 }));
}
