#if WITH_DEV_AUTOMATION_TESTS
#include "StaticWorldQueryAudit.h"
#include "AnalyticWorldData.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "Misc/AutomationTest.h"
using namespace Speed::Analytic;
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStaticWorldIsolationTest,
    "IAmSpeed.AnalyticWorld.PrivateFrameIsolation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FStaticWorldIsolationTest::RunTest(const FString&)
{
    FStaticWorldQueryAudit::FScopedFrameIsolation PreserveCaller;
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
        TestEqual(TEXT("live sweep count restored"),After.LegacySweepCount,Before.LegacySweepCount);
        TestEqual(TEXT("live query count restored"),After.QueryCount,Before.QueryCount);
        TestEqual(TEXT("live authority count restored"),After.AuthorityAttemptCount,Before.AuthorityAttemptCount);
    };
    {
        FStaticWorldQueryAudit::FScopedFrameIsolation Guard;
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
        FStaticWorldQueryAudit::BeginFrame(456,&PrivateData,nullptr);
        return; // cancellation/failure return
    };
    EarlyExit(); CheckRestored();
    try
    {
        FStaticWorldQueryAudit::FScopedFrameIsolation Guard;
        FStaticWorldQueryAudit::BeginFrame(456,&PrivateData,nullptr);
        throw 17;
    }
    catch (int) {}
    CheckRestored();
    return true;
}
#endif
