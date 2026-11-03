#include "PrivateInputFailureFile.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPrivateInputRecordBudgets01621Test,
    "IAmSpeed.Input.Diagnostics.PrivateRecordBudgets01621",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPrivateInputRecordBudgets01621Test::RunTest(const FString&)
{
    using namespace Speed::Input::Diagnostics;
    FPrivateInputRecordBudgets Budgets;
    uint32 Accepted = 0;
    // Reproduce the traveled-Garage flood before the first gameplay fault.
    for (auto Channel : {EPrivateInputChannel::Retirement, EPrivateInputChannel::Startup})
    {
        for (uint32 I=1;I<=8;++I)
        {
            const uint32 Ordinal=Budgets.Take(Channel);
            if (!TestEqual(TEXT("each diagnostic channel has its own ordinal"),Ordinal,I)) return false;
            ++Accepted;
        }
        TestEqual(TEXT("flooded channel refuses a ninth write attempt"),Budgets.Take(Channel),uint32(0));
    }
    TestTrue(TEXT("retirement and startup floods preserve real fault capacity"),Budgets.HasCapacity(EPrivateInputChannel::FirstFault));
    TestEqual(TEXT("invalid channel consumes no fault budget"),Budgets.Take(static_cast<EPrivateInputChannel>(255)),uint32(0));
    for (uint32 I=1;I<=8;++I)
    {
        if (!TestEqual(TEXT("first faults remain independently recordable"),Budgets.Take(EPrivateInputChannel::FirstFault),I)) return false;
        ++Accepted;
    }
    TestEqual(TEXT("all channels together remain bounded to twenty-four attempts"),Accepted,FPrivateInputRecordBudgets::ProcessMaximum);
    for (auto Channel : {EPrivateInputChannel::FirstFault, EPrivateInputChannel::Retirement, EPrivateInputChannel::Startup})
        TestTrue(TEXT("exhausted process can never admit another channel write"),!Budgets.HasCapacity(Channel) && Budgets.Take(Channel)==0);
    return true;
}
#endif
