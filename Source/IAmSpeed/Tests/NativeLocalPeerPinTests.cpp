#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/transport/NativeLocalPeerPin.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeLocalPeerPin,"IAmSpeed.Netcode.Transport.NativeLocalPeerProcessPin",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeLocalPeerPin::RunTest(const FString&)
{
    using namespace Speed::Netcode;
    const auto Current=FNativeLocalPeerPin::InspectCurrent();if(!TestTrue(TEXT("actual current OS process descriptor"),bool(Current)))return false;
    auto Pin=FNativeLocalPeerPin::PinOriginal(*Current);
    if(!TestTrue(TEXT("retained original OS handle is alive"),Pin&&Pin->OriginalProcessAlive()))return false;
    auto Wrong=*Current;++Wrong.BirthTicks;
    TestFalse(TEXT("same numeric PID with wrong birth cannot create pin"),bool(FNativeLocalPeerPin::PinOriginal(Wrong)));
    Wrong=*Current;Wrong.ImagePath+=L".wrong";
    TestFalse(TEXT("wrong actual image path cannot create pin"),bool(FNativeLocalPeerPin::PinOriginal(Wrong)));
    FNativeDatagramSocket Socket;if(!TestTrue(TEXT("actual exclusive loopback endpoint"),Socket.Bind({})==ENativeDatagram::Ready))return false;
    const auto Endpoint=Socket.LocalEndpoint();
    TestTrue(TEXT("actual OS UDP table maps endpoint to pinned current process"),Pin->MatchesObservedLoopbackEndpoint(Endpoint));
    auto NonLocal=Endpoint;NonLocal.Address={192,0,2,1};
    TestFalse(TEXT("nonlocal source cannot reuse local process evidence"),Pin->MatchesObservedLoopbackEndpoint(NonLocal));
    Socket.Close();
    TestFalse(TEXT("closed endpoint no longer found in actual UDP census"),Pin->MatchesObservedLoopbackEndpoint(Endpoint));
    TestTrue(TEXT("closing own socket did not kill or replace original process"),Pin->OriginalProcessAlive());
    AddInfo(TEXT("Actual process HANDLE/birth/image-path and OS UDP owner census only. Current process unit, not separate peer processes, socket-generation lifetime, remote authentication, trusted game-source admission, native mutable-view lease or resource/job release proof."));
    return !HasAnyErrors();
}
#endif
