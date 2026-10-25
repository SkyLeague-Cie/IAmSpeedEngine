#pragma once
#include "IAmSpeed/Input/InputHostSession.h"
#include "HAL/PlatformTLS.h"
#include <condition_variable>
#include <mutex>

namespace
{
// Authored raw readings, hardware=none. Only the real acquisition worker Pump
// publishes to the real journal. The GT neither polls nor writes physical input.
class FControllerRawAcquisition2443 final : public Speed::Input::V2::IInputAcquisition
{
    using FTicket = Speed::Input::V2::FAcquisitionTicket;
    struct FRequest { bool Held=false, Baseline=false; std::optional<FTicket> Ticket; uint64 Id=0; };
public:
    explicit FControllerRawAcquisition2443(std::shared_ptr<Speed::Input::V2::FRawAcquisitionJournal> InJournal)
        : Journal(std::move(InJournal)) {}
    bool Submit(bool Held, bool Baseline, std::optional<FTicket> Ticket = {})
    {
        std::unique_lock<std::mutex> Lock(Gate);
        if (Closed || Pending) return false;
        const uint64 Id=++Requested; Pending=FRequest{Held,Baseline,Ticket,Id};
        return Changed.wait_for(Lock,std::chrono::seconds(2),[&]{return Closed || Completed>=Id;})
            && !Closed && Completed==Id && Accepted;
    }
    Speed::Input::V2::EAcquisitionPumpResult Pump() override
    {
        using namespace Speed::Input::V2;
        Thread.store(FPlatformTLS::GetCurrentThreadId());
        FRequest Request;
        {
            std::lock_guard<std::mutex> Lock(Gate);
            if (Closed) return EAcquisitionPumpResult::Closed;
            if (!Pending) return EAcquisitionPumpResult::NoChange;
            Request=*Pending; Pending.reset();
        }
        auto Reservation=Journal->ReserveAcquisition();
        const auto Ticket=Request.Ticket.value_or(Journal->BeginAcquisition());
        FRawAcquisitionBatch Batch; Batch.Count=1; auto& R=Batch.Readings[0];
        R.DeviceId=7; R.Sequence=++Sequence; R.TimestampMicroseconds=Sequence;
        R.Generation={1}; R.Kind=ERawDeviceKind::Keyboard; R.FreshBaseline=Request.Baseline;
        R.State.Count=1; R.State.Values[0]={{ERawControlKind::KeyboardUsage,4},Request.Held?1.0f:0.0f};
        const bool Good=Journal->Publish(Ticket,Batch);
        {
            std::lock_guard<std::mutex> Lock(Gate); Accepted=Good; Completed=Request.Id;
            if (Good) ++Publications;
        }
        Changed.notify_all();
        return Good?EAcquisitionPumpResult::Installed:EAcquisitionPumpResult::Rejected;
    }
    bool Close() override
    {
        Journal->Close();
        { std::lock_guard<std::mutex> Lock(Gate); Closed=true; Pending.reset(); }
        Changed.notify_all(); return true;
    }
    bool IsClosed() const { std::lock_guard<std::mutex> Lock(Gate); return Closed; }
    std::atomic<uint32> Thread{0};
private:
    std::shared_ptr<Speed::Input::V2::FRawAcquisitionJournal> Journal;
    mutable std::mutex Gate; std::condition_variable Changed;
    std::optional<FRequest> Pending;
    uint64 Requested=0,Completed=0,Sequence=0,Publications=0;
    bool Accepted=false,Closed=false;
};
}
