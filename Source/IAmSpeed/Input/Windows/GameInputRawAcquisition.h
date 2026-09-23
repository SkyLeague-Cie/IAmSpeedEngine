#pragma once

#include "GameInputSelectedSource.h"
#include "GameInputCanonicalControls.h"
#include "../RawAcquisitionJournal.h"
#include "../InputAcquisitionWorker.h"
#include <thread>

namespace Speed::Input::Windows
{
using ERawPumpResult = V2::EAcquisitionPumpResult;

// Platform acquisition owner, separate from the physical producer. The host
// schedules Pump on its acquisition lane even while physics is paused. Pump
// binds to the first caller thread; controller/physical consumers read Journal.
// Close serializes with any in-flight Pump and prevents further OS calls.
// This class does not itself create a thread or execute game/control callbacks.
class FGameInputRawAcquisition final : public V2::IInputAcquisition
{
public:
	enum class ENeutralizeCause : std::uint8_t { FreshResumeReadRejected, PollRawRejected };
	enum class ESinkReject : std::uint8_t
	{
		None, NoTicket, Disconnected, SequenceOverflow, GenerationOverflow,
		DeviceIndexUnavailable, Canonicalization, JournalPublish
	};
	struct FNeutralizeDiagnostic
	{
		ENeutralizeCause Cause;
		EPollStatus PollStatus;
		FGameInputSelectedSource::ERawPollReject RawPollReject;
		ESinkReject SinkReject;
		std::uint64_t AcquisitionTick;
		V2::FAcquisitionInvalidation Barrier;
	};
	FGameInputRawAcquisition(std::unique_ptr<FGameInputSelectedSource> InSource,
		std::shared_ptr<V2::FRawAcquisitionJournal> InJournal)
		: Source(std::move(InSource)), Journal(std::move(InJournal)) {}
	ERawPumpResult Pump() override
	{
		if (FPresentationInputScope::IsActive()) return ERawPumpResult::Rejected;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Closed || !Source || !Journal) return ERawPumpResult::Closed;
		const auto Caller = std::this_thread::get_id();
		if (Owner && *Owner != Caller) return ERawPumpResult::Rejected;
		Owner = Caller;
		if (Tick == std::numeric_limits<std::uint64_t>::max()) return ERawPumpResult::Rejected;
		const auto Ticket = Journal->BeginAcquisition();
		if (Journal->NeedsFreshResume() && !Source->RequestFreshRawReading())
			return Neutralize(Ticket, ENeutralizeCause::FreshResumeReadRejected, ESinkReject::None);
		bool HadData = false;
		ESinkReject SinkReject = ESinkReject::None;
		const bool Accepted = Source->PollRaw(++Tick, [&](const FGameInputSelectedSource::FSelectedRawBatch& Batch) noexcept
		{
			if (!Batch.Ticket) { SinkReject = ESinkReject::NoTicket; return false; }
			if (Batch.Status == EPollStatus::Disconnected) { SinkReject = ESinkReject::Disconnected; return false; }
			if (!Batch.Readings.Count) return true;
			if (Sequence > std::numeric_limits<std::uint64_t>::max() - Batch.Readings.Count)
			{ SinkReject = ESinkReject::SequenceOverflow; return false; }
			const bool Fresh = Batch.Readings.FreshBaseline || Neutral || !LastTicket
				|| !SameTicket(*LastTicket, *Batch.Ticket);
			if (Fresh && Generation == std::numeric_limits<std::uint64_t>::max())
			{ SinkReject = ESinkReject::GenerationOverflow; return false; }
			const auto NewGeneration = Generation + (Fresh ? 1 : 0);
			const auto Id = DeviceIndex(Batch.Ticket->Device.Id);
			if (!Id) { SinkReject = ESinkReject::DeviceIndexUnavailable; return false; }
			Prepared.Count = Batch.Readings.Count;
			for (std::size_t I = 0; I < Prepared.Count; ++I)
			{
				auto& R = Prepared.Readings[I]; R = {};
				R.DeviceId = Id; R.Generation = {NewGeneration}; R.Sequence = Sequence + I + 1;
				R.Kind = Batch.Ticket->Kind == EDeviceKind::Keyboard ? V2::ERawDeviceKind::Keyboard : V2::ERawDeviceKind::Gamepad;
				R.TimestampMicroseconds = Batch.Readings.States[I].TimestampMicroseconds;
				R.FreshBaseline = Fresh && I == 0;
				if (!Canonicalize(Batch.Readings.States[I], R.Kind, R.State))
				{ SinkReject = ESinkReject::Canonicalization; return false; }
			}
			if (!Journal->Publish(Ticket, Prepared))
			{ SinkReject = ESinkReject::JournalPublish; return false; }
			Sequence += Prepared.Count; Generation = NewGeneration; LastTicket = Batch.Ticket;
			LastState = Prepared.Readings[Prepared.Count - 1]; Neutral = false; HadData = true;
			return true;
		});
		if (!Accepted) return Neutralize(Ticket, ENeutralizeCause::PollRawRejected, SinkReject);
		return HadData ? ERawPumpResult::Installed : ERawPumpResult::NoChange;
	}
	std::optional<FNeutralizeDiagnostic> TakeNeutralizeDiagnostic()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		auto Result = LastNeutralizeDiagnostic;
		LastNeutralizeDiagnostic.reset();
		return Result;
	}
	bool Close() override
	{
		std::lock_guard<std::mutex> Lock(Gate);
		Closed = true;
		if (Journal) Journal->Close();
		return !Source || Source->Shutdown();
	}
private:
	ERawPumpResult Neutralize(V2::FAcquisitionTicket Ticket, ENeutralizeCause Cause, ESinkReject SinkReject)
	{
		if (Neutral && !Journal->NeedsFreshResume()) return ERawPumpResult::Neutralized;
		LastNeutralizeDiagnostic = FNeutralizeDiagnostic{Cause, Source->GetLastPollStatus(),
			Cause == ENeutralizeCause::PollRawRejected ? Source->GetLastRawPollReject()
				: FGameInputSelectedSource::ERawPollReject::None,
			SinkReject, Tick,
			Journal->Invalidate()};
		if (Sequence == std::numeric_limits<std::uint64_t>::max() || Generation == std::numeric_limits<std::uint64_t>::max())
			return ERawPumpResult::Rejected;
		Prepared.Count = 1; auto& R = Prepared.Readings[0]; R = LastState.value_or(V2::FAcquiredRawState{});
		R.DeviceId = FDeviceDiscovery::Capacity + 1; // Reserved neutral source, never a real registry index.
		R.Generation = {Generation + 1}; R.Sequence = Sequence + 1; R.FreshBaseline = true;
		if (!R.State.Count)
		{
			R.Kind = V2::ERawDeviceKind::Gamepad;
			if (!Canonicalize({}, R.Kind, R.State)) return ERawPumpResult::Rejected;
		}
		for (std::size_t I = 0; I < R.State.Count; ++I) R.State.Values[I].Value = 0;
		if (!Journal->Publish(Ticket, Prepared)) return ERawPumpResult::Rejected;
		++Sequence; ++Generation; Neutral = true; LastTicket.reset();
		return ERawPumpResult::Neutralized;
	}
	std::uint64_t DeviceIndex(const FDeviceId& Id) noexcept
	{
		for (std::size_t I = 0; I < DeviceCount; ++I) if (Devices[I] == Id) return I + 1;
		if (DeviceCount == Devices.size()) return 0;
		Devices[DeviceCount++] = Id; return DeviceCount;
	}
	static bool SameTicket(const FDeviceSelection& A, const FDeviceSelection& B) noexcept
	{ return A.Device.Id == B.Device.Id && A.Device.Revision == B.Device.Revision && A.Kind == B.Kind && A.Generation == B.Generation; }
	std::mutex Gate;
	std::unique_ptr<FGameInputSelectedSource> Source;
	std::shared_ptr<V2::FRawAcquisitionJournal> Journal;
	std::optional<std::thread::id> Owner;
	V2::FRawAcquisitionBatch Prepared;
	std::array<FDeviceId, FDeviceDiscovery::Capacity> Devices{};
	std::size_t DeviceCount = 0;
	std::optional<FDeviceSelection> LastTicket;
	std::optional<V2::FAcquiredRawState> LastState;
	std::optional<FNeutralizeDiagnostic> LastNeutralizeDiagnostic;
	std::uint64_t Tick = 0, Sequence = 0, Generation = 0;
	bool Closed = false, Neutral = false;
};
}
