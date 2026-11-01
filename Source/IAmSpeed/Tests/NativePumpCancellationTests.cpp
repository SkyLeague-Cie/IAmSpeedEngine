#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/transport/NativePredictionProcessProbe.h"
#include "Misc/AutomationTest.h"
#include <thread>
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativePumpCancellation,
    "IAmSpeed.Netcode.Transport.NativePumpCancellation",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativePumpCancellation::RunTest(const FString&)
{
    using namespace Speed::Netcode;
    // initial_suspend means no native world/socket has been acquired yet.
    // This covers cancellation BEFORE first pulse, not suspended-world cleanup.
    std::string Why;auto Pump=FNativePredictionProbePump::Create({},Why);
    if(!TestTrue(TEXT("actual initially suspended coroutine created on GT"),bool(Pump)))return false;
    ENativeProbePumpStep Foreign=ENativeProbePumpStep::Running;
    std::thread Other([&]{Foreign=Pump->Cancel();});Other.join();
    TestTrue(TEXT("foreign cancellation refuses without resuming or retiring the owner"),Foreign==ENativeProbePumpStep::WrongOwner);
    TestTrue(TEXT("original owner can cancel before initial resume"),Pump->Cancel()==ENativeProbePumpStep::Cancelled);
    TestTrue(TEXT("cancel is idempotent and remains a distinct non-success result"),Pump->Cancel()==ENativeProbePumpStep::Cancelled);
    TestTrue(TEXT("cancelled Tick never resumes the destroyed coroutine"),Pump->Tick()==ENativeProbePumpStep::Cancelled);
    TestFalse(TEXT("cancellation publishes no invented native values"),bool(Pump->ReadLatest()));
    const auto D=Pump->Diagnostics();
    TestTrue(TEXT("actual ClientDone attempts and datagrams stay zero"),D&&D->ClientDoneSendAttempts==0&&D->ClientDoneDatagrams==0);
    AddInfo(TEXT("[IASNativePumpCancellation] before_first_pulse=1 foreign_refused=1 cancelled_distinct=1 client_done_attempts=0 native_cleanup_case=NOT_RUN"));
    return !HasAnyErrors();
}
#endif
