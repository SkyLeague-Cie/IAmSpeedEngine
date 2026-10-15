#if WITH_DEV_AUTOMATION_TESTS
#include "StaticWorldQueryAudit.h"
#include "AnalyticWorldData.h"
#include "IAmSpeed/World/Simulation/SimulationActorDiagnostics.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "Misc/AutomationTest.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
using namespace Speed::Analytic;
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticWorldIsolationTest,
    "IAmSpeed.AnalyticWorld.PrivateFrameIsolation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FStaticWorldIsolationTest::RunTest(const FString&)
{
    const auto* Policy=GetDefault<USpeedWorldSubsystem>();

    for (const auto Type : {EWorldType::None,EWorldType::Game,EWorldType::Editor,EWorldType::PIE,
        EWorldType::GamePreview,EWorldType::GameRPC,EWorldType::Inactive})
        TestEqual(TEXT("existing world admission preserved"),Policy->DoesSupportWorldType(Type),Type==EWorldType::Game || Type==EWorldType::Editor || Type==EWorldType::PIE);
    TestTrue(TEXT("private preview bridge supported"),Policy->DoesSupportWorldType(EWorldType::EditorPreview));
    const auto Options=UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
        .ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* Preview=UWorld::CreateWorld(EWorldType::EditorPreview,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Options);
    if (!TestNotNull(TEXT("ordinary preview world"),Preview)) return false;
    ON_SCOPE_EXIT { Preview->DestroyWorld(false); Preview->RemoveFromRoot(); };
    auto* Bridge=Preview->GetSubsystem<USpeedWorldSubsystem>();
    if (!TestNotNull(TEXT("preview creates bridge"),Bridge)) return false;
    TestTrue(TEXT("preview adapter registry remains empty"),Bridge->SimulationWorld.GetAdapters().IsEmpty());
    TestFalse(TEXT("preview has no automatically started driver"),bool(TActorIterator<ASpeedSimulation>(Preview)));
    TestFalse(TEXT("preview has not begun gameplay"),Preview->HasBegunPlay());
    TestNull(TEXT("preview creates no physics scene"),Preview->GetPhysicsScene());
    FStaticWorldQueryAudit::FScopedFrameIsolation PreserveCaller;
#if !UE_BUILD_SHIPPING
    Speed::ActorDiagnostics::bEnabled=true;
#endif
    IConsoleVariable* Audit=IConsoleManager::Get().FindConsoleVariable(TEXT("p.IAmSpeed.StaticWorldQuery.Audit"));
    if (!Audit) { AddError(TEXT("audit variable missing")); return false; }
    const int32 Old=Audit->GetInt(); const auto Flags=Audit->GetFlags();
    Audit->Set(1,ECVF_SetByCode);
    ON_SCOPE_EXIT { Audit->Set(Old,ECVF_SetByCode); Audit->SetFlags(Flags); };
    FAnalyticWorldData LiveData, PrivateData;
    FStaticWorldQueryAudit::BeginFrame(123,&LiveData,nullptr);
    FStaticWorldQueryAudit::RecordLegacySweep();
    const auto Before=FStaticWorldQueryAudit::GetCurrentFrameCounters();
    const auto CheckRestored=[&]()
    {
        TestTrue(TEXT("live frame and provider restored"),
            FStaticWorldQueryAudit::IsCurrentFrameContext(123,&LiveData,nullptr));
        const auto After=FStaticWorldQueryAudit::GetCurrentFrameCounters();
#if !UE_BUILD_SHIPPING
        TestTrue(TEXT("live actor diagnostics restored"),Speed::ActorDiagnostics::bEnabled);
#endif
        TestEqual(TEXT("live sweep count restored"),After.LegacySweepCount,Before.LegacySweepCount);
        TestEqual(TEXT("live query count restored"),After.QueryCount,Before.QueryCount);
        TestEqual(TEXT("live authority count restored"),After.AuthorityAttemptCount,Before.AuthorityAttemptCount);
    };
    {
        FStaticWorldQueryAudit::FScopedFrameIsolation Guard;
#if !UE_BUILD_SHIPPING
        TestFalse(TEXT("private actor diagnostics suppressed"),Speed::ActorDiagnostics::bEnabled);
#endif
        FStaticWorldQueryAudit::BeginFrame(456,&PrivateData,nullptr);
        FStaticWorldQueryAudit::RecordLegacySweep();
        FStaticWorldQueryAudit::RecordLegacySweep();
        {
            FStaticWorldQueryAudit::FScopedFrameIsolation Nested;
            FStaticWorldQueryAudit::BeginFrame(789,nullptr,nullptr);
        }
        TestTrue(TEXT("nested private provider restored"),
            FStaticWorldQueryAudit::IsCurrentFrameContext(456,&PrivateData,nullptr));
        FStaticWorldQueryAudit::EndFrame();
    }
    CheckRestored();
    const auto EarlyExit=[&]()
    {
        FStaticWorldQueryAudit::FScopedFrameIsolation Guard;
#if !UE_BUILD_SHIPPING
        TestFalse(TEXT("private actor diagnostics suppressed"),Speed::ActorDiagnostics::bEnabled);
#endif
        FStaticWorldQueryAudit::BeginFrame(456,&PrivateData,nullptr);
        return; // cancellation/failure return
    };
    EarlyExit(); CheckRestored();
    try
    {
        FStaticWorldQueryAudit::FScopedFrameIsolation Guard;
#if !UE_BUILD_SHIPPING
        TestFalse(TEXT("private actor diagnostics suppressed"),Speed::ActorDiagnostics::bEnabled);
#endif
        FStaticWorldQueryAudit::BeginFrame(456,&PrivateData,nullptr);
        throw 17;
    }
    catch (int) {}
    CheckRestored();
    return true;
}
#endif
