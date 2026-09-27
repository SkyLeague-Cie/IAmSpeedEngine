#include "IAmSpeed/Input/LegacyRemotePreSlew.h"
#include <iostream>
#include <cstdlib>
using namespace Speed::Input;
using FWire = std::array<std::uint8_t, 9>;
static unsigned Checks=0;
static void Check(bool V,const char* Why) { ++Checks; if (!V) { std::cerr<<"FAIL "<<Why<<'\n'; std::exit(1); } }
int main()
{
    auto Ingress=std::make_shared<TLegacyRemotePreSlewIngress<FWire>>(91,7,10);
    std::array<std::uint16_t,ActionCount> Steps{}; Steps[Throttle]=16; Steps[Brake]=16; Steps[Steering]=16; Steps[6]=64;
    FActionValues OldApplied{};
    TLegacyRemotePreSlewOwner<FWire> Owner(Ingress,10,Steps);
    FWire Bytes{255,0,127,0,255,1,2,3,4};
    FActionValues Targets{}; Targets[Throttle]=255; Targets[Steering]=127; Targets[6]=255;
    Check(Ingress->Submit({91,7,0,10},Targets,Bytes)==ELegacyRemoteAdmission::Accepted,"first source frame admitted");
    Check(Ingress->Submit({91,7,0,10},Targets,Bytes)==ELegacyRemoteAdmission::DuplicateOrOutOfOrder,"duplicate rejected");
    Check(Ingress->Submit({92,7,1,11},Targets,Bytes)==ELegacyRemoteAdmission::WrongSource,"wrong producer rejected");
    Check(Ingress->Submit({91,8,1,11},Targets,Bytes)==ELegacyRemoteAdmission::WrongSource,"wrong epoch rejected");
    for (FFrameNumber N=10;N<36;++N)
    {
        if (N==16 || N==24)
        {
            Targets[Throttle]=N==16?0:230; Targets[Brake]=N==16?255:0; Targets[Steering]=N==16?-127:43; Targets[6]=N==16?0:170;
            Bytes[0]=static_cast<std::uint8_t>(Targets[Throttle]); Bytes[1]=static_cast<std::uint8_t>(Targets[Brake]);
            Check(Ingress->Submit({91,7,N,N},Targets,Bytes)==ELegacyRemoteAdmission::Accepted,"new remote target admitted");
        }
        for (std::size_t I=0;I<ActionCount;++I)
        {
            const int A=OldApplied[I], T=Targets[I], S=Steps[I];
            OldApplied[I]=S?static_cast<std::int16_t>(A<T?std::min(A+S,T):std::max(A-S,T)):Targets[I];
        }
        const auto Snapshot=Owner.Poll(N);
        Check(Snapshot && Snapshot->Applied==OldApplied,"Applied preserves historical multi-frame slew including holds and reversals");
        Check(Snapshot->Packet->Wire==Bytes,"wire payload untouched");
        Check(Snapshot->Packet->Address.Epoch==7 && Snapshot->Frame==N,"source and physical addresses retained");
        Check(Owner.Poll(N)==Snapshot,"repeat poll cannot slew twice");
        Check(!Owner.Poll(N+1),"cannot advance an uncommitted frame");
        Check(Owner.Complete(N),"complete exactly once");
        Check(!Owner.Complete(N),"duplicate commit rejected");
        Check(!Owner.Poll(N),"old frame cannot repoll source");
    }
    Check(Ingress->Submit({91,7,50,35},Targets,Bytes)==ELegacyRemoteAdmission::Stale,"late activation rejected");
    Check(Ingress->Submit({91,7,20,36},Targets,Bytes)==ELegacyRemoteAdmission::DuplicateOrOutOfOrder,"out-of-order source rejected");
    bool ForeignAccepted=true;
    std::thread Foreign([&] { ForeignAccepted=bool(Owner.Poll(36)); }); Foreign.join();
    Check(!ForeignAccepted,"transport thread cannot run the filter");
    const auto Pending=Owner.Poll(36); Check(bool(Pending),"next physical frame staged");
    Owner.Abort(); Check(!Owner.ReadLatest() && !Owner.Poll(36),"aborted input never retries or publishes");
    Ingress->Close(); Check(Ingress->Submit({91,7,51,37},Targets,Bytes)==ELegacyRemoteAdmission::Closed,"closed epoch rejects input");
    std::cout<<"PASS LegacyRemotePreSlewProbe checks="<<Checks<<" native_wire_serializer=not_executed\n";
}
