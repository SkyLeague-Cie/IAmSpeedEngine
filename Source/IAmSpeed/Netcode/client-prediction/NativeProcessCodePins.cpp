#include "NativeProcessCodePins.h"
#include "ProcessCodePinWindowForTesting.h"
#include <algorithm>
#include <array>
#include <type_traits>
#if PLATFORM_WINDOWS
#include "Windows/WindowsHWrapper.h"
#include <psapi.h>
#endif

namespace Speed::Netcode::Prediction
{
#if WITH_DEV_AUTOMATION_TESTS && PLATFORM_WINDOWS
namespace Testing
{
static thread_local FProcessPinWindowControl* ActiveWindow=nullptr;
FScopedProcessPinWindowForTesting::FScopedProcessPinWindowForTesting(FProcessPinWindowControl& Control) noexcept
{
    if(!ActiveWindow&&Control.InputCapturedEvent&&Control.ContinueEvent
        &&Control.WaitMilliseconds>0&&Control.WaitMilliseconds<=5000)
    {Bound=&Control;Installed=true;ActiveWindow=Bound;}
}
FScopedProcessPinWindowForTesting::~FScopedProcessPinWindowForTesting()
{if(Installed&&ActiveWindow==Bound)ActiveWindow=nullptr;}
}
#elif WITH_DEV_AUTOMATION_TESTS
namespace Testing
{
FScopedProcessPinWindowForTesting::FScopedProcessPinWindowForTesting(FProcessPinWindowControl&) noexcept {}
FScopedProcessPinWindowForTesting::~FScopedProcessPinWindowForTesting()=default;
}
#endif

std::shared_ptr<const FNativeProcessCodePins> FNativeProcessCodePins::PinObservedProcessModules()
{
    auto Report=std::shared_ptr<FNativeProcessCodePins>(new FNativeProcessCodePins);
#if PLATFORM_WINDOWS
    static_assert(sizeof(void*)==8,"Private native code pins are qualified for Win64 only");
    static_assert(std::is_trivially_destructible_v<FProcessCodePinRecord>);
    std::array<HMODULE,MaximumProcessPinModules> Modules{},After{},Pinned{};
    Report->Process=::GetCurrentProcessId();
    HMODULE Self=nullptr;
    const auto Address=reinterpret_cast<LPCWSTR>(&FNativeProcessCodePins::PinObservedProcessModules);
    if(!::GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,Address,&Self))
    {Report->Outcome=EProcessCodePinStatus::SelfModuleUnavailable;Report->FailureError=::GetLastError();return Report;}
    Report->Self=reinterpret_cast<std::uintptr_t>(Self);

    // Allocate ALL record/pass storage before any irreversible PIN. Later
    // resize only shrinks trivial records; it neither allocates nor calls out.
    Report->Observations.resize(MaximumProcessPinModules);
    std::size_t RecordCount=0,PinnedCount=0;
    const auto Finish=[&](EProcessCodePinStatus Status)
    {
        Report->Outcome=Status;Report->Observations.resize(RecordCount);return Report;
    };
    const auto Less=[](HMODULE A,HMODULE B)
    {return reinterpret_cast<std::uintptr_t>(A)<reinterpret_cast<std::uintptr_t>(B);};
    const auto CopyRaw=[](const auto& Source,DWORD Bytes,auto& Destination)
    {
        const auto Available=std::min<std::size_t>(Bytes/sizeof(HMODULE),MaximumProcessPinModules);
        for(std::size_t I=0;I<Available;++I)Destination[I]=reinterpret_cast<std::uintptr_t>(Source[I]);
        return static_cast<std::uint32_t>(Available);
    };
    const auto Validate=[&](auto& Set,DWORD Bytes,EProcessCodePinStatus& Failure)->bool
    {
        if(Bytes>sizeof(Modules)){Failure=EProcessCodePinStatus::BoundExceeded;return false;}
        if(!Bytes||Bytes%sizeof(HMODULE)){Failure=EProcessCodePinStatus::InvalidEnumeration;return false;}
        const auto Count=Bytes/sizeof(HMODULE);
        std::sort(Set.begin(),Set.begin()+Count,Less);
        if(!Set[0]||std::adjacent_find(Set.begin(),Set.begin()+Count)!=Set.begin()+Count)
            {Failure=EProcessCodePinStatus::DuplicateModule;return false;}
        if(!std::binary_search(Set.begin(),Set.begin()+Count,Self,Less))
            {Failure=EProcessCodePinStatus::SelfModuleMissing;return false;}
        return true; // shape validity has no positive PIN/admission status
    };
    DWORD Bytes=0;
    const BOOL Initial=::K32EnumProcessModules(::GetCurrentProcess(),Modules.data(),DWORD(sizeof(Modules)),&Bytes);
    const DWORD InitialError=Initial ? 0 : ::GetLastError();
    for(std::size_t PassIndex=0;PassIndex<MaximumProcessPinPasses;++PassIndex)
    {
        auto& Pass=Report->PassObservations[PassIndex];++Report->CompletedPassRecords;
        Pass.InputIsInitial=PassIndex==0;Pass.InputAfterPass=PassIndex?static_cast<std::uint32_t>(PassIndex-1):0;
        Pass.InputSucceeded=PassIndex?true:Initial!=0;Pass.InputBytes=Bytes;
        Pass.InputError=PassIndex?0:InitialError;
        // Later inputs are the PREVIOUS actual After enumeration, not a
        // fabricated additional OS call or a claim about module incarnations.
        if(!Pass.InputSucceeded)
        {Report->FailureError=Pass.InputError;return Finish(EProcessCodePinStatus::EnumerationFailed);}
        Pass.InputAvailable=CopyRaw(Modules,Bytes,Pass.InputBases);
        EProcessCodePinStatus InputStatus=EProcessCodePinStatus::InvalidEnumeration;
        if(!Validate(Modules,Bytes,InputStatus))
        {Pass.Transition=EProcessPinTransition::InvalidEnumeration;return Finish(InputStatus);}
        const std::size_t Count=Bytes/sizeof(HMODULE);
        if(!std::includes(Modules.begin(),Modules.begin()+Count,Pinned.begin(),Pinned.begin()+PinnedCount,Less))
        {Pass.Transition=EProcessPinTransition::MissingPinnedBase;return Finish(EProcessCodePinStatus::RosterChanged);}

#if WITH_DEV_AUTOMATION_TESTS
        if(auto* Window=Testing::ActiveWindow)
        {
            // Real kernel events only: no callback, allocation, substituted
            // OS answer or physical-world operation in this window.
            Pass.WindowInstalled=true;
            Window->ActualPass.store(static_cast<std::uint32_t>(PassIndex),std::memory_order_release);
            Pass.WindowSignaled=::SetEvent(reinterpret_cast<HANDLE>(Window->InputCapturedEvent))!=0;
            if(!Pass.WindowSignaled)
            {Pass.WindowSignalError=::GetLastError();return Finish(EProcessCodePinStatus::WindowControlFailed);}
            const DWORD Wait=::WaitForSingleObject(reinterpret_cast<HANDLE>(Window->ContinueEvent),Window->WaitMilliseconds);
            Pass.WindowWaitReturned=true;Pass.WindowWaitResult=Wait;
            if(Wait!=WAIT_OBJECT_0)
            {if(Wait==WAIT_FAILED)Pass.WindowWaitError=::GetLastError();return Finish(EProcessCodePinStatus::WindowControlFailed);}
        }
#endif
        // Already-PINned bases are never re-PINned. All new observed records
        // are retained, including unattempted ones following a refused PIN.
        const std::size_t FirstNew=RecordCount;
        for(std::size_t I=0;I<Count;++I)
            if(!std::binary_search(Pinned.begin(),Pinned.begin()+PinnedCount,Modules[I],Less))
            {
                if(RecordCount==MaximumProcessPinModules)return Finish(EProcessCodePinStatus::BoundExceeded);
                Report->Observations[RecordCount++].EnumeratedBase=reinterpret_cast<std::uintptr_t>(Modules[I]);
            }
        for(std::size_t I=FirstNew;I<RecordCount;++I)
        {
            auto& Record=Report->Observations[I];Record.Attempted=true;++Pass.NewPinAttempts;
            HMODULE Returned=nullptr;
            const BOOL Result=::GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,
                reinterpret_cast<LPCWSTR>(Record.EnumeratedBase),&Returned);
            Record.PinReturned=Result!=0;Record.ReturnedBase=reinterpret_cast<std::uintptr_t>(Returned);
            if(!Result)Record.Error=::GetLastError();
            if(!Result||Record.ReturnedBase!=Record.EnumeratedBase)
            {Report->FailureError=Record.Error;return Finish(EProcessCodePinStatus::PinFailed);}
            Pinned[PinnedCount++]=Returned;
        }
        std::sort(Pinned.begin(),Pinned.begin()+PinnedCount,Less);
        DWORD AfterBytes=0;
        const BOOL AfterResult=::K32EnumProcessModules(::GetCurrentProcess(),After.data(),DWORD(sizeof(After)),&AfterBytes);
        Pass.AfterSucceeded=AfterResult!=0;Pass.AfterBytes=AfterBytes;
        if(!AfterResult)
        {Pass.AfterError=::GetLastError();Report->FailureError=Pass.AfterError;return Finish(EProcessCodePinStatus::EnumerationFailed);}
        Pass.AfterAvailable=CopyRaw(After,AfterBytes,Pass.AfterBases);
        EProcessCodePinStatus AfterStatus=EProcessCodePinStatus::InvalidEnumeration;
        if(!Validate(After,AfterBytes,AfterStatus))
        {Pass.Transition=EProcessPinTransition::InvalidEnumeration;return Finish(AfterStatus);}
        const std::size_t AfterCount=AfterBytes/sizeof(HMODULE);
        if(AfterCount==PinnedCount&&std::equal(Pinned.begin(),Pinned.begin()+PinnedCount,After.begin()))
        {Pass.Transition=EProcessPinTransition::Equal;return Finish(EProcessCodePinStatus::PinnedBaseSetAtPinTimes);}
        if(AfterCount<=PinnedCount||!std::includes(After.begin(),After.begin()+AfterCount,Pinned.begin(),Pinned.begin()+PinnedCount,Less))
        {Pass.Transition=EProcessPinTransition::MissingPinnedBase;return Finish(EProcessCodePinStatus::RosterChanged);}
        // Only a STRICT SUPERSET of actual successful, matched PINs may be
        // expanded. No failed PIN/removal/substitution is ever retried.
        if(PassIndex+1==MaximumProcessPinPasses)
        {Pass.Transition=EProcessPinTransition::BudgetExhausted;return Finish(EProcessCodePinStatus::RosterChanged);}
        Pass.Transition=EProcessPinTransition::StrictExpansion;
        Bytes=AfterBytes;
        for(std::size_t I=0;I<AfterCount;++I)
            Modules[I]=reinterpret_cast<HMODULE>(Pass.AfterBases[I]);
    }
    return Finish(EProcessCodePinStatus::RosterChanged);
#else
    return Report;
#endif
}
}
