#if defined(IAMSPEED_SWITCHCAM_GAMEINPUT_TRACE) && IAMSPEED_SWITCHCAM_GAMEINPUT_TRACE
#include "SwitchCamGameInputTrace.h"

#include "InputActionContract.h"
#include "Containers/Ticker.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "CoreGlobals.h"
#include "Misc/CoreDelegates.h"
#include "Logging/LogMacros.h"
#include <algorithm>
#include <new>

namespace Speed::Input::V2::SwitchCamGameInputTrace
{
namespace
{
constexpr std::size_t MaxSessions = 32;
std::array<std::unique_ptr<FSession>, MaxSessions> Sessions;
std::atomic<std::size_t> Published{0};
FTSTicker::FDelegateHandle Ticker;
FDelegateHandle PreExit;
std::array<bool, MaxSessions> Reported{};
bool Stopped = false; // Game thread only.

void Drain(bool Final, const TCHAR* Hook)
{
    for (std::size_t Index = 0; Index < Published.load(std::memory_order_acquire); ++Index)
    {
        auto& Session = *Sessions[Index];
        const auto Before = Session.RawRecords + Session.MapRecords;
        // Close is acquired before draining so the footer includes its suffix.
        const bool Closed = Session.AcquisitionClosed.load(std::memory_order_acquire);
        FRawRecord Raw;
        for (unsigned Remaining = 2048; Remaining && Session.Raw.Pop(Raw); --Remaining)
        {
            UE_LOG(LogTemp, Display, TEXT("[SLSwitchCamGameInput] site=raw session=%llu device=%llu generation=%llu raw_acq_seq=%llu timestamp_us=%llu kind=%u code=%u sdk_code=%u before=%.9g after=%.9g fresh=%u"),
                Session.Id, Raw.Device, Raw.Generation, Raw.Sequence, Raw.TimestampUs,
                unsigned(Raw.Control.Kind), unsigned(Raw.Control.Code), unsigned(Raw.SdkCode),
                double(Raw.Before), double(Raw.After), unsigned(Raw.Fresh));
            ++Session.RawRecords;
        }

        FMapRecord Map;
        for (unsigned Remaining = 512; Remaining && Session.Map.Pop(Map); --Remaining)
        {
            UE_LOG(LogTemp, Display, TEXT("[SLSwitchCamGameInput] site=map session=%llu epoch=%llu frame=%llu source_sequence=%llu ordinal=%u producer_kind=0 producer_id=%llu action=%u transition=%s raw_acq_seq=%llu device=%llu generation=%llu"),
                Session.Id, Map.Epoch, Map.Frame, Map.SourceSequence, unsigned(Map.Ordinal),
                Session.Id, unsigned(Session.Action), Map.Pressed ? TEXT("pressed") : TEXT("released"),
                Map.RawSequence, Map.Device, Map.Generation);
            ++Session.MapRecords;
        }

        const auto RawDropped = Session.Raw.Dropped(), MapDropped = Session.Map.Dropped();
        if (Final || !Reported[Index] || (Closed && !Session.ReportedClosed) ||
            Session.LastReportedDrops != RawDropped + MapDropped || Before != Session.RawRecords + Session.MapRecords)
        {
            const TCHAR* Status = Closed ? TEXT("closed") :
                (Session.RawRecords ? TEXT("active") : TEXT("no_pad_reading"));
            UE_LOG(LogTemp, Display, TEXT("[SLSwitchCamGameInput] site=summary session=%llu status=%s raw_dropped=%llu map_dropped=%llu raw_records=%llu map_records=%llu final=%u drain_hook=%s acquisition_closed=%u"),
                Session.Id, Status, RawDropped, MapDropped, Session.RawRecords, Session.MapRecords,
                unsigned(Final), Hook, unsigned(Closed));
            Reported[Index] = true;
            Session.LastReportedDrops = RawDropped + MapDropped;
            Session.ReportedClosed = Closed;
        }
    }
}

void Finish(const TCHAR* Hook)
{
    if (Stopped) return;
    Stopped = true;
    if (PreExit.IsValid())
    {
        FCoreDelegates::OnPreExit.Remove(PreExit);
        PreExit.Reset();
    }
    if (Ticker.IsValid())
    {
        FTSTicker::RemoveTicker(Ticker);
        Ticker.Reset();
    }
    // PreExit is after world cleanup and before the log closes. An expired
    // ticker handle cannot suppress this drain or certify acquisition closure.
    Drain(true, Hook);
}
}

void RegisterSession(std::uint64_t Id, std::uint64_t First, const FInputActionContract& Contract)
{
    static const bool Enabled = FParse::Param(FCommandLine::Get(), TEXT("SLSwitchCamGameInputTrace"));
    if (!Enabled || Stopped) return;
    // The Steam bridge can translate and renumber a GameInput observation.
    static const bool NativeOnly = FParse::Param(FCommandLine::Get(), TEXT("nosteam"));
    const auto& Description = Contract.GetDescription();
    const FActionDefinition* Action = nullptr;
    for (const auto& Candidate : Description.Actions)
        if (Candidate.Owner == "SkyLeague" && Candidate.Name == "SwitchCam" &&
            Candidate.Type == EActionType::Bool && Candidate.Wiring == EActionWiring::Wired)
            Action = &Candidate;
    const auto Index = Published.load(std::memory_order_relaxed);
    if (!NativeOnly || !Action || Index == MaxSessions)
    {
        const TCHAR* Status = !NativeOnly ? TEXT("requires_nosteam") :
            (!Action ? TEXT("no_switchcam") : TEXT("session_limit"));
        UE_LOG(LogTemp, Display, TEXT("[SLSwitchCamGameInput] site=bind session=%llu first=%llu name=SwitchCam status=%s"),
            Id, First, Status);
        return;
    }

    auto Session = std::unique_ptr<FSession>(new (std::nothrow) FSession);
    if (!Session)
    {
        UE_LOG(LogTemp, Display, TEXT("[SLSwitchCamGameInput] site=bind session=%llu first=%llu name=SwitchCam status=trace_allocation_failed"), Id, First);
        return;
    }
    Session->Id = Id;
    Session->First = First;
    Session->Action = Action->Id;
    for (const auto& Binding : Description.Mapping)
    {
        const auto Control = Binding.Control;
        if (Binding.Action != Action->Id || !Control.IsValid() ||
            !IsControlKindForDevice(Control.Kind, ERawDeviceKind::Gamepad))
            continue;
        const auto End = Session->Controls.begin() + Session->ControlCount;
        if (std::find(Session->Controls.begin(), End, Control) != End)
            continue;
        check(Session->ControlCount < Session->Controls.size());
        Session->Controls[Session->ControlCount++] = Control;
        UE_LOG(LogTemp, Display, TEXT("[SLSwitchCamGameInput] site=bind session=%llu first=%llu action=%u name=SwitchCam kind=%u code=%u status=armed"),
            Id, First, unsigned(Action->Id), unsigned(Control.Kind), unsigned(Control.Code));
    }
    if (!Session->ControlCount)
    {
        UE_LOG(LogTemp, Display, TEXT("[SLSwitchCamGameInput] site=bind session=%llu first=%llu action=%u name=SwitchCam status=no_pad_binding"),
            Id, First, unsigned(Action->Id));
        return;
    }

    if (!PreExit.IsValid())
        PreExit = FCoreDelegates::OnPreExit.AddLambda([] { Finish(TEXT("pre_exit")); });
    if (!Ticker.IsValid())
        Ticker = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda(
            [](float) { Drain(false, TEXT("ticker")); return true; }), 0.25f);
    Sessions[Index] = std::move(Session);
    Published.store(Index + 1, std::memory_order_release);
}

FSession* FindSession(std::uint64_t Id) noexcept
{
    const auto Count = Published.load(std::memory_order_acquire);
    if (!Count) return nullptr;
    thread_local FSession* Cached = nullptr;
    if (Cached && Cached->Id == Id) return Cached;
    for (std::size_t I = Count; I > 0; --I)
    {
        if (Sessions[I - 1]->Id == Id)
        {
            Cached = Sessions[I - 1].get();
            return Cached;
        }
    }
    return nullptr;
}

void Stop()
{
    Finish(TEXT("module_shutdown"));
    // Session objects stay resident until IAS unload, after its producers stop.
}

} // namespace Speed::Input::V2::SwitchCamGameInputTrace
#endif
