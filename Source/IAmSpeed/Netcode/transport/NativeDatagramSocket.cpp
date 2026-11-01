#include "NativeDatagramSocket.h"
#include <mutex>
#if defined(_WIN32)
#include "Windows/AllowWindowsPlatformTypes.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include "Windows/HideWindowsPlatformTypes.h"
#endif

namespace Speed::Netcode
{
struct FNativeDatagramSocket::FImpl
{
    mutable std::mutex Gate;FNativeEndpoint Local;
#if defined(_WIN32)
    SOCKET Handle=INVALID_SOCKET;bool Runtime=false;
    static sockaddr_in Address(const FNativeEndpoint& E)
    {
        sockaddr_in A{};A.sin_family=AF_INET;A.sin_port=htons(E.Port);
        const std::uint32_t IP=(std::uint32_t(E.Address[0])<<24)|(std::uint32_t(E.Address[1])<<16)
            |(std::uint32_t(E.Address[2])<<8)|E.Address[3];A.sin_addr.s_addr=htonl(IP);return A;
    }
    static FNativeEndpoint Endpoint(const sockaddr_in& A)
    {
        const auto IP=ntohl(A.sin_addr.s_addr);return {{{std::uint8_t(IP>>24),std::uint8_t(IP>>16),std::uint8_t(IP>>8),std::uint8_t(IP)}},ntohs(A.sin_port)};
    }
    void Retire() noexcept
    {if(Handle!=INVALID_SOCKET){closesocket(Handle);Handle=INVALID_SOCKET;}if(Runtime){WSACleanup();Runtime=false;}Local={};Local.Port=0;}
#else
    void Retire() noexcept {Local={};Local.Port=0;}
#endif
};
FNativeDatagramSocket::FNativeDatagramSocket():Impl(std::make_unique<FImpl>()){}
FNativeDatagramSocket::~FNativeDatagramSocket(){Close();}
ENativeDatagram FNativeDatagramSocket::Bind(const FNativeEndpoint& Requested)
{
    std::lock_guard Lock(Impl->Gate);
#if defined(_WIN32)
    if(Impl->Runtime||Impl->Handle!=INVALID_SOCKET)return ENativeDatagram::Invalid;
    WSADATA W{};if(WSAStartup(MAKEWORD(2,2),&W)!=0)return ENativeDatagram::Error;Impl->Runtime=true;
    Impl->Handle=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    if(Impl->Handle==INVALID_SOCKET){Impl->Retire();return ENativeDatagram::Error;}
    const BOOL Exclusive=1;const int BufferBytes=1024*1024;u_long NonBlocking=1;
    if(setsockopt(Impl->Handle,SOL_SOCKET,SO_EXCLUSIVEADDRUSE,reinterpret_cast<const char*>(&Exclusive),sizeof(Exclusive))!=0
        ||setsockopt(Impl->Handle,SOL_SOCKET,SO_RCVBUF,reinterpret_cast<const char*>(&BufferBytes),sizeof(BufferBytes))!=0
        ||setsockopt(Impl->Handle,SOL_SOCKET,SO_SNDBUF,reinterpret_cast<const char*>(&BufferBytes),sizeof(BufferBytes))!=0
        ||ioctlsocket(Impl->Handle,FIONBIO,&NonBlocking)!=0){Impl->Retire();return ENativeDatagram::Error;}
    auto A=FImpl::Address(Requested);
    if(bind(Impl->Handle,reinterpret_cast<const sockaddr*>(&A),sizeof(A))!=0){Impl->Retire();return ENativeDatagram::Error;}
    int Size=sizeof(A);
    if(getsockname(Impl->Handle,reinterpret_cast<sockaddr*>(&A),&Size)!=0||A.sin_family!=AF_INET)
    {Impl->Retire();return ENativeDatagram::Error;}
    Impl->Local=FImpl::Endpoint(A);return ENativeDatagram::Ready;
#else
    return ENativeDatagram::Unsupported;
#endif
}
FNativeEndpoint FNativeDatagramSocket::LocalEndpoint() const
{std::lock_guard Lock(Impl->Gate);return Impl->Local;}
ENativeDatagram FNativeDatagramSocket::Send(const FNativeEndpoint& Destination,const std::vector<std::uint8_t>& Bytes)
{
    std::lock_guard Lock(Impl->Gate);
    if(Bytes.empty()||!Destination.Port)return ENativeDatagram::Invalid;
    if(Bytes.size()>NativeDatagramBudget)return ENativeDatagram::TooLarge;
#if defined(_WIN32)
    if(Impl->Handle==INVALID_SOCKET)return ENativeDatagram::Closed;
    const auto A=FImpl::Address(Destination);
    const int Sent=sendto(Impl->Handle,reinterpret_cast<const char*>(Bytes.data()),int(Bytes.size()),0,reinterpret_cast<const sockaddr*>(&A),sizeof(A));
    if(Sent==SOCKET_ERROR)return WSAGetLastError()==WSAEWOULDBLOCK?ENativeDatagram::WouldBlock:ENativeDatagram::Error;
    return std::size_t(Sent)==Bytes.size()?ENativeDatagram::Ready:ENativeDatagram::Error;
#else
    return ENativeDatagram::Unsupported;
#endif
}
ENativeDatagram FNativeDatagramSocket::Receive(FNativeEndpoint& ActualSource,std::vector<std::uint8_t>& Bytes)
{
    std::lock_guard Lock(Impl->Gate);Bytes.clear();ActualSource={};ActualSource.Port=0;
#if defined(_WIN32)
    if(Impl->Handle==INVALID_SOCKET)return ENativeDatagram::Closed;
    std::array<char,NativeDatagramBudget+1> Buffer{};sockaddr_in A{};int Size=sizeof(A);
    const int Got=recvfrom(Impl->Handle,Buffer.data(),int(Buffer.size()),0,reinterpret_cast<sockaddr*>(&A),&Size);
    if(Got==SOCKET_ERROR)
    {const int Error=WSAGetLastError();return Error==WSAEWOULDBLOCK?ENativeDatagram::WouldBlock:Error==WSAEMSGSIZE?ENativeDatagram::TooLarge:ENativeDatagram::Error;}
    if(Got<=0||A.sin_family!=AF_INET)return ENativeDatagram::Invalid;
    if(std::size_t(Got)>NativeDatagramBudget)return ENativeDatagram::TooLarge;
    ActualSource=FImpl::Endpoint(A);Bytes.assign(reinterpret_cast<const std::uint8_t*>(Buffer.data()),reinterpret_cast<const std::uint8_t*>(Buffer.data()+Got));return ENativeDatagram::Ready;
#else
    return ENativeDatagram::Unsupported;
#endif
}
void FNativeDatagramSocket::Close() noexcept {std::lock_guard Lock(Impl->Gate);Impl->Retire();}
}
