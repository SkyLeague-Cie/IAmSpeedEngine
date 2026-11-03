#include "NativeProcessCodePins.h"
#include <algorithm>
#include <array>
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include <psapi.h>
#endif

namespace Speed::Netcode::Prediction
{
std::shared_ptr<const FNativeProcessCodePins> FNativeProcessCodePins::PinObservedProcessModules()
{
    auto Report=std::shared_ptr<FNativeProcessCodePins>(new FNativeProcessCodePins);
#if PLATFORM_WINDOWS
    static_assert(sizeof(void*)==8,"Private native code pins are qualified for Win64 only");
    constexpr std::size_t MaximumModules=1024;
    std::array<HMODULE,MaximumModules> Modules{},After{};
    Report->Process=::GetCurrentProcessId();
    HMODULE Self=nullptr;
    const auto Address=reinterpret_cast<LPCWSTR>(&FNativeProcessCodePins::PinObservedProcessModules);
    if(!::GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,Address,&Self))
    {Report->Outcome=EProcessCodePinStatus::SelfModuleUnavailable;Report->FailureError=::GetLastError();return Report;}
    Report->Self=reinterpret_cast<std::uintptr_t>(Self);
    DWORD Bytes=0;
    if(!::K32EnumProcessModules(::GetCurrentProcess(),Modules.data(),DWORD(sizeof(Modules)),&Bytes))
    {Report->Outcome=EProcessCodePinStatus::EnumerationFailed;Report->FailureError=::GetLastError();return Report;}
    if(Bytes>sizeof(Modules)){Report->Outcome=EProcessCodePinStatus::BoundExceeded;return Report;}
    if(!Bytes||Bytes%sizeof(HMODULE)){Report->Outcome=EProcessCodePinStatus::InvalidEnumeration;return Report;}
    const std::size_t Count=Bytes/sizeof(HMODULE);
    // Reserve every record before the first irreversible PIN. The bounded
    // Win32 loop below performs no C++ allocation, callback or Engine access.
    Report->Observations.resize(Count);
    std::sort(Modules.begin(),Modules.begin()+Count,[](HMODULE A,HMODULE B)
        {return reinterpret_cast<std::uintptr_t>(A)<reinterpret_cast<std::uintptr_t>(B);});
    if(!Modules[0]||std::adjacent_find(Modules.begin(),Modules.begin()+Count)!=Modules.begin()+Count)
    {Report->Outcome=EProcessCodePinStatus::DuplicateModule;return Report;}
    if(std::find(Modules.begin(),Modules.begin()+Count,Self)==Modules.begin()+Count)
    {Report->Outcome=EProcessCodePinStatus::SelfModuleMissing;return Report;}
    for(std::size_t I=0;I<Count;++I)Report->Observations[I].EnumeratedBase=reinterpret_cast<std::uintptr_t>(Modules[I]);
    for(std::size_t I=0;I<Count;++I)
    {
        auto& Record=Report->Observations[I];Record.Attempted=true;
        HMODULE Pinned=nullptr;
        const BOOL Result=::GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,
            reinterpret_cast<LPCWSTR>(Modules[I]),&Pinned);
        Record.PinReturned=Result!=FALSE;Record.ReturnedBase=reinterpret_cast<std::uintptr_t>(Pinned);
        if(!Result)Record.Error=::GetLastError();
        if(!Result||Pinned!=Modules[I])
        {Report->Outcome=EProcessCodePinStatus::PinFailed;Report->FailureError=Record.Error;return Report;}
    }
    DWORD AfterBytes=0;
    if(!::K32EnumProcessModules(::GetCurrentProcess(),After.data(),DWORD(sizeof(After)),&AfterBytes))
    {Report->Outcome=EProcessCodePinStatus::EnumerationFailed;Report->FailureError=::GetLastError();return Report;}
    if(AfterBytes!=Bytes){Report->Outcome=EProcessCodePinStatus::RosterChanged;return Report;}
    std::sort(After.begin(),After.begin()+Count,[](HMODULE A,HMODULE B)
        {return reinterpret_cast<std::uintptr_t>(A)<reinterpret_cast<std::uintptr_t>(B);});
    if(!std::equal(Modules.begin(),Modules.begin()+Count,After.begin()))
    {Report->Outcome=EProcessCodePinStatus::RosterChanged;return Report;}
    // This is equality of the observed numeric base sets only. A module could
    // have unloaded/reloaded at the same base BEFORE its successful PIN call.
    // Initial roster incarnations and pre-existing callbacks remain UNKNOWN.
    // Actual returned modules stay PINned; later loads are not covered.
    Report->Outcome=EProcessCodePinStatus::PinnedBaseSetAtPinTimes;
#endif
    return Report;
}
}
