#include "StaticWorldQueryAudit.h"
#include "Misc/ScopeExit.h"
#include <stdexcept>
namespace
{
bool RefusalCountersAreTerminal(const Speed::Analytic::FStaticWorldQueryCounters& before,const Speed::Analytic::FStaticWorldQueryCounters& after)
{
    return after.StrictMissingWorldCount==before.StrictMissingWorldCount+1 &&
        after.AuthorityAttemptCount==before.AuthorityAttemptCount+1 &&
        after.AuthorityCoveredCount==before.AuthorityCoveredCount &&
        after.AuthorityFallbackCount==before.AuthorityFallbackCount &&
        after.LegacySweepCount==before.LegacySweepCount;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrictMissingSingleRefusalTest,
 "IAmSpeed.AnalyticWorld.StrictRefusal.MissingSingleAuthorityIsTerminal",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrictMissingSingleRefusalTest::RunTest(const FString&)
{
    FAnalyticBackendFixtureScope scope;if(!TestTrue(TEXT("existing console ownership available"),scope.Ready()))return false;
    using A=Speed::Analytic::FStaticWorldQueryAudit;
    for(int32 shadow=0;shadow<2;++shadow)
    {
        if(!TestTrue(TEXT("strict selector applied"),scope.Backend(2,shadow)))return false;const auto before=A::GetCurrentFrameCounters();bool advanced=false,refused=false;FHitResult hit;bool hasHit=true;
        try{A::TryCompactAuthoritySingle(nullptr,FVector::ZeroVector,FVector::UnitX(),FQuat::Identity,FCollisionShape::MakeSphere(1.f),0,FCollisionResponseParams::DefaultResponseParam,hit,hasHit);advanced=true;}
        catch(const std::runtime_error& error){refused=FCStringAnsi::Strcmp(error.what(),"SurfaceAnalytic static collision world unavailable (single)")==0;}
        TestTrue(TEXT("strict missing single throws before continuation even with Shadow"),refused&&!advanced);
        TestTrue(TEXT("refusal does not count coverage, legacy fallback or miss"),RefusalCountersAreTerminal(before,A::GetCurrentFrameCounters()));
    }
    for(int32 backend=0;backend<2;++backend)
    {
        if(!TestTrue(TEXT("legacy/hybrid selector applied"),scope.Backend(backend,1)))return false;FHitResult hit;bool hasHit=true,threw=false,handled=true;
        try{handled=A::TryCompactAuthoritySingle(nullptr,FVector::ZeroVector,FVector::UnitX(),FQuat::Identity,FCollisionShape::MakeSphere(1.f),0,FCollisionResponseParams::DefaultResponseParam,hit,hasHit);}catch(...){threw=true;}
        TestTrue(TEXT("legacy and hybrid preserve explicit unhandled fallback"),!threw&&!handled);
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrictMissingMultiRefusalTest,
 "IAmSpeed.AnalyticWorld.StrictRefusal.MissingOverlapAuthorityIsTerminal",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrictMissingMultiRefusalTest::RunTest(const FString&)
{
    FAnalyticBackendFixtureScope scope;if(!TestTrue(TEXT("existing console ownership available"),scope.Ready()))return false;
    using A=Speed::Analytic::FStaticWorldQueryAudit;
    for(int32 shadow=0;shadow<2;++shadow)
    {
        if(!TestTrue(TEXT("strict selector applied"),scope.Backend(2,shadow)))return false;const auto before=A::GetCurrentFrameCounters();bool advanced=false,refused=false;TArray<FHitResult> hits;
        try{A::TryCompactAuthorityMulti(nullptr,FVector::ZeroVector,FVector::ZeroVector,FQuat::Identity,FCollisionShape::MakeSphere(1.f),1,hits);advanced=true;}
        catch(const std::runtime_error& error){refused=FCStringAnsi::Strcmp(error.what(),"SurfaceAnalytic static collision world unavailable (multi)")==0;}
        TestTrue(TEXT("strict missing overlap throws before continuation even with Shadow"),refused&&!advanced);
        TestTrue(TEXT("refusal does not count coverage, legacy fallback or miss"),RefusalCountersAreTerminal(before,A::GetCurrentFrameCounters()));
    }
    for(int32 backend=0;backend<2;++backend)
    {
        if(!TestTrue(TEXT("legacy/hybrid selector applied"),scope.Backend(backend,1)))return false;TArray<FHitResult> hits;bool threw=false,handled=true;
        try{handled=A::TryCompactAuthorityMulti(nullptr,FVector::ZeroVector,FVector::ZeroVector,FQuat::Identity,FCollisionShape::MakeSphere(1.f),1,hits);}catch(...){threw=true;}
        TestTrue(TEXT("legacy and hybrid preserve explicit unhandled overlap fallback"),!threw&&!handled);
    }
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStrictMissingSphereBeforeIntegrateTest,
 "IAmSpeed.AnalyticWorld.StrictRefusal.ActualSphereStopsBeforeIntegrate",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FStrictMissingSphereBeforeIntegrateTest::RunTest(const FString&)
{
    FTensorBoundsTestContext numeric;if(!TestTrue(TEXT("RNE fixture established"),numeric.Ready))return false;
    FAnalyticBackendFixtureScope scope;if(!TestTrue(TEXT("existing console ownership available"),scope.Ready()))return false;
    TStrongObjectPtr<USpeedMovementComponent> owner(NewObject<USpeedMovementComponent>());TStrongObjectPtr<USphereSubBody> sphere(NewObject<USphereSubBody>());
    owner->SetIsFrozen(false);owner->SetKinematicState(PrimitiveUnitState());sphere->Initialize(owner.Get());sphere->SetRelativeLocation(FVector::ZeroVector);sphere->SetRadiusForConfiguration(1.25f);
    const auto before=owner->GetKinematicState();if(!TestTrue(TEXT("strict selector applied"),scope.Backend(2)))return false;const float delta=1.f/300.f;float toi=delta;bool advanced=false,refused=false;
    try{sphere->SweepTOI(delta,toi);owner->IntegrateKinematics(delta);advanced=true;}
    catch(const std::runtime_error& error){refused=FCStringAnsi::Strcmp(error.what(),"SurfaceAnalytic solid sweep has no World")==0;}
    TestTrue(TEXT("production sphere null World cannot return miss and integrate"),refused&&!advanced);
    TestTrue(TEXT("no primitive or kinematic rollback needed because integration never ran"),PrimitiveStateBitsEqual(before,owner->GetKinematicState()));
    if(!TestTrue(TEXT("legacy selector applied"),scope.Backend(0)))return false;bool legacyThrew=false;try{sphere->SweepTOI(delta,toi);}catch(...){legacyThrew=true;}
    TestFalse(TEXT("unchanged legacy null-World sphere path does not throw"),legacyThrew);
    TestTrue(TEXT("floating context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand StrictAuthorityRefusalUnits(TEXT("Speed.Analytic.RunStrictAuthorityRefusalUnits"),TEXT("Three strict missing authority refusal units; no World or collision qualification."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool used=false;auto& f=FAutomationTestFramework::Get();
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticStrictAuthorityRefusalUnits"))||used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Strict authority refusal units refused"));return;}used=true;
    const TCHAR* classes[]={TEXT("FStrictMissingSingleRefusalTest"),TEXT("FStrictMissingMultiRefusalTest"),TEXT("FStrictMissingSphereBeforeIntegrateTest")};
    const TCHAR* names[]={TEXT("IAmSpeed.AnalyticWorld.StrictRefusal.MissingSingleAuthorityIsTerminal"),TEXT("IAmSpeed.AnalyticWorld.StrictRefusal.MissingOverlapAuthorityIsTerminal"),TEXT("IAmSpeed.AnalyticWorld.StrictRefusal.ActualSphereStopsBeforeIntegrate")};
    for(int32 i=0;i<3;++i)
    {
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()||!f.ContainsTest(classes[i])){UE_LOG(LogTemp,Error,TEXT("Strict authority refusal unit ownership refused"));return;}
        f.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);f.StartTestByName(classes[i],0,names[i]);
        if(!GIsAutomationTesting||!f.GetCurrentTest()||f.GetCurrentTestFullPath()!=names[i]||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Strict authority refusal unit synchronous ownership refused"));return;}
        FAutomationTestExecutionInfo info;const bool stopped=f.StopTest(info);
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Strict authority refusal unit final ownership refused"));return;}
        const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
        UE_LOG(LogTemp,Display,TEXT("ANALYTIC_STRICT_AUTHORITY_REFUSAL_UNIT name=%s passed=%d errors=%d warnings=%d"),names[i],passed,info.GetErrorTotal(),info.GetWarningTotal());
    }
 }));
}
