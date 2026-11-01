#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/transport/NativeDatagramSocket.h"
#include "Misc/AutomationTest.h"
#include <chrono>
#include <thread>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeLatePeerDatagram,"IAmSpeed.Netcode.Transport.NativeLatePeerDatagram",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeLatePeerDatagram::RunTest(const FString&)
{
    using namespace Speed::Netcode;
    FNativeDatagramSocket Sender,Reservation,Late;
    if(!TestTrue(TEXT("actual sender and reserved exclusive endpoints"),Sender.Bind({})==ENativeDatagram::Ready&&Reservation.Bind({})==ENativeDatagram::Ready))return false;
    const auto Destination=Reservation.LocalEndpoint(),Origin=Sender.LocalEndpoint();
    Reservation.Close(); // deliberately no receiver at this actual OS port
    unsigned Notices=0;const std::vector<std::uint8_t> Payload{26,1,7};
    const auto NoticeDeadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(std::chrono::steady_clock::now()<NoticeDeadline&&!Notices)
    {
        if(!TestTrue(TEXT("unbound destination accepts a datagram send, not an ACK"),Sender.Send(Destination,Payload)==ENativeDatagram::Ready))return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        FNativeEndpoint From=Destination;std::vector<std::uint8_t> Bytes{99,98};const auto Status=Sender.Receive(From,Bytes);
        if(Status==ENativeDatagram::PeerUnavailable)++Notices;
        else if(!TestTrue(TEXT("no delivered message from the still unbound endpoint"),Status==ENativeDatagram::WouldBlock))return false;
        if(!TestTrue(TEXT("unavailable notification carries no source or input bytes"),From.Port==0&&Bytes.empty()))return false;
    }
    if(!TestTrue(TEXT("actual Windows ICMP Port Unreachable observed"),Notices>0))return false;
    if(!TestTrue(TEXT("same sender socket remains bound; late peer binds its exact reserved endpoint"),Sender.LocalEndpoint()==Origin&&Late.Bind(Destination)==ENativeDatagram::Ready))return false;
    bool Delivered=false,Replied=false;const auto DeliveryDeadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(std::chrono::steady_clock::now()<DeliveryDeadline&&!Replied)
    {
        if(!TestTrue(TEXT("retry retains actual sender endpoint"),Sender.Send(Destination,Payload)==ENativeDatagram::Ready))return false;
        FNativeEndpoint From;std::vector<std::uint8_t> Bytes;auto Status=Late.Receive(From,Bytes);
        if(Status==ENativeDatagram::Ready)
        {
            if(!TestTrue(TEXT("late peer receives exact original source and payload"),From==Origin&&Bytes==Payload))return false;
            Delivered=true;if(!TestTrue(TEXT("reply is sent only after actual receive"),Late.Send(Origin,Bytes)==ENativeDatagram::Ready))return false;
        }
        else if(!TestTrue(TEXT("late receive remains nonblocking"),Status==ENativeDatagram::WouldBlock))return false;
        Status=Sender.Receive(From,Bytes);
        if(Status==ENativeDatagram::Ready)
        {if(!TestTrue(TEXT("actual reply from late endpoint"),Delivered&&From==Destination&&Bytes==Payload))return false;Replied=true;}
        else if(!TestTrue(TEXT("remaining unreachable notices are not data"),Status==ENativeDatagram::WouldBlock||Status==ENativeDatagram::PeerUnavailable))return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    TestTrue(TEXT("same original socket communicates after genuine late bind"),Delivered&&Replied);
    AddInfo(FString::Printf(TEXT("[IASLatePeerDatagram] actual_unreachable_notifications=%u late_bind_delivery=%d reply=%d scope=real_windows_udp_unit_not_game_or_historical_error_certificate"),Notices,int(Delivered),int(Replied)));
    return !HasAnyErrors();
}
#endif
