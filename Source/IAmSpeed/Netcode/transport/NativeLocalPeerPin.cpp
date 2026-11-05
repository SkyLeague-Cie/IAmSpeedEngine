#include "NativeLocalPeerPin.h"
#include <cstddef>
#include <cstring>
#if defined(_WIN32)
#include "Windows/AllowWindowsPlatformTypes.h"
#include <winsock2.h>
#include <windows.h>
#include <iphlpapi.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace Speed::Netcode
{
struct FNativeLocalPeerPin::FImpl
{
    FNativeProcessDescriptor Expected;
#if defined(_WIN32)
    HANDLE Process=nullptr;
    ~FImpl(){if(Process)CloseHandle(Process);}
    static std::uint64_t Ticks(const FILETIME& Time) noexcept
    {return (std::uint64_t(Time.dwHighDateTime)<<32)|Time.dwLowDateTime;}
    static std::optional<FNativeProcessDescriptor> Read(HANDLE Process)
    {
        if(!Process||WaitForSingleObject(Process,0)!=WAIT_TIMEOUT)return {};
        FILETIME Birth{},Exit{},Kernel{},User{};
        if(!GetProcessTimes(Process,&Birth,&Exit,&Kernel,&User))return {};
        std::wstring Image(32768,L'\0');::DWORD Size=static_cast<::DWORD>(Image.size());
        if(!QueryFullProcessImageNameW(Process,0,Image.data(),&Size)||!Size||Size>=Image.size())return {};
        Image.resize(Size);const auto Pid=GetProcessId(Process);
        if(!Pid||!Ticks(Birth))return {};return FNativeProcessDescriptor{Pid,Ticks(Birth),std::move(Image)};
    }
    bool Alive() const noexcept
    {
        if(!Process||WaitForSingleObject(Process,0)!=WAIT_TIMEOUT)return false;
        FILETIME Birth{},Exit{},Kernel{},User{};
        return GetProcessId(Process)==Expected.Pid&&GetProcessTimes(Process,&Birth,&Exit,&Kernel,&User)
            &&Ticks(Birth)==Expected.BirthTicks;
    }
#else
    bool Alive() const noexcept {return false;}
#endif
};
FNativeLocalPeerPin::FNativeLocalPeerPin():Impl(std::make_unique<FImpl>()){}
FNativeLocalPeerPin::~FNativeLocalPeerPin()=default;
std::optional<FNativeProcessDescriptor> FNativeLocalPeerPin::InspectCurrent()
{
#if defined(_WIN32)
    const HANDLE Process=OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION|SYNCHRONIZE,0,GetCurrentProcessId());
    const auto Value=FImpl::Read(Process);if(Process)CloseHandle(Process);return Value;
#else
    return {};
#endif
}
std::unique_ptr<FNativeLocalPeerPin> FNativeLocalPeerPin::PinOriginal(const FNativeProcessDescriptor& Expected)
{
    if(!Expected.Pid||!Expected.BirthTicks||Expected.ImagePath.empty()||Expected.ImagePath.size()>32767)return {};
#if defined(_WIN32)
    auto Pin=std::unique_ptr<FNativeLocalPeerPin>(new FNativeLocalPeerPin());
    Pin->Impl->Process=OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION|SYNCHRONIZE,0,Expected.Pid);
    const auto Actual=FImpl::Read(Pin->Impl->Process);
    if(!Actual||Actual->Pid!=Expected.Pid||Actual->BirthTicks!=Expected.BirthTicks
        ||CompareStringOrdinal(Actual->ImagePath.data(),int(Actual->ImagePath.size()),Expected.ImagePath.data(),int(Expected.ImagePath.size()),1)!=CSTR_EQUAL)return {};
    Pin->Impl->Expected=*Actual;return Pin;
#else
    return {};
#endif
}
bool FNativeLocalPeerPin::OriginalProcessAlive() const noexcept {return Impl->Alive();}
bool FNativeLocalPeerPin::MatchesObservedLoopbackEndpoint(const FNativeEndpoint& Source) const
{
    if(Source.Address!=std::array<std::uint8_t,4>{127,0,0,1}||!Source.Port||!Impl->Alive())return false;
#if defined(_WIN32)
    // Microsoft GetExtendedUdpTable / UDP_TABLE_OWNER_PID, bounded two-call
    // census. A changing/oversized/unavailable table refuses this observation.
    // https://learn.microsoft.com/windows/win32/api/iphlpapi/nf-iphlpapi-getextendedudptable
    ::DWORD Size=0;
    if(GetExtendedUdpTable(nullptr,&Size,0,AF_INET,UDP_TABLE_OWNER_PID,0)!=ERROR_INSUFFICIENT_BUFFER
        ||Size<offsetof(MIB_UDPTABLE_OWNER_PID,table)||Size>1024*1024)return false;
    std::vector<std::uint8_t> Bytes(Size);
    if(GetExtendedUdpTable(Bytes.data(),&Size,0,AF_INET,UDP_TABLE_OWNER_PID,0)!=NO_ERROR
        ||Size>Bytes.size()||Size<offsetof(MIB_UDPTABLE_OWNER_PID,table))return false;
    ::DWORD Count=0;std::memcpy(&Count,Bytes.data(),sizeof(Count));
    const std::size_t Offset=offsetof(MIB_UDPTABLE_OWNER_PID,table);
    if(Count>(Size-Offset)/sizeof(MIB_UDPROW_OWNER_PID))return false;
    unsigned Matches=0;
    for(::DWORD N=0;N<Count;++N)
    {
        MIB_UDPROW_OWNER_PID Row{};std::memcpy(&Row,Bytes.data()+Offset+std::size_t(N)*sizeof(Row),sizeof(Row));
        if(Row.dwLocalAddr==htonl(0x7f000001)&&ntohs(static_cast<u_short>(Row.dwLocalPort))==Source.Port)
        {if(Row.dwOwningPid!=Impl->Expected.Pid)return false;++Matches;}
    }
    return Matches==1&&Impl->Alive();
#else
    return false;
#endif
}
}
