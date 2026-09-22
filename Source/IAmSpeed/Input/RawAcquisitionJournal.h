#pragma once

#include "InputProducerV2.h"
#include <mutex>
#include <limits>

namespace Speed::Input::V2
{
// Acquisition order is independent of simulation and presentation clocks.
// DeviceId is a collision-free registry index owned by the platform session.
struct FAcquiredRawState
{
	std::uint64_t DeviceId = 0, Sequence = 0, TimestampMicroseconds = 0;
	FDeviceGeneration Generation;
	ERawDeviceKind Kind = ERawDeviceKind::Keyboard;
	bool FreshBaseline = false;
	FCanonicalRawState State;
};
struct FRawAcquisitionBatch
{
	std::array<FAcquiredRawState, 64> Readings{};
	std::size_t Count = 0;
};
struct FAcquisitionCursor { std::uint64_t Session = 0, Serial = 0; };
struct FAcquisitionTicket { std::uint64_t Session = 0, PhysicalFence = 0; };
enum class EAcquisitionRead { NoChange, Batch, Gap, Closed, InvalidCursor };
struct FAcquisitionHistory
{
	EAcquisitionRead Status = EAcquisitionRead::NoChange;
	FAcquisitionCursor Next;
	std::vector<FAcquiredRawState> Readings;
};
struct FAcquisitionBaseline { FAcquisitionCursor Cursor; FAcquiredRawState State; };

// One acquisition owner publishes; one physical owner polls; the control lane
// reads independently, including during physical pause. No OS/UObject calls.
// The host must quiesce physical reservations BEFORE lifecycle operations here.
// A separate acquisition lifetime fence joins the pump before Close/destruction.
class FRawAcquisitionJournal final : public IRawInputSource
{
public:
	static constexpr std::size_t Capacity = 256;
	explicit FRawAcquisitionJournal(std::uint64_t Session, FFrameNumber FirstFrame = 0)
		: SessionId(Session), NextFrame(FirstFrame) { Closed = !Session; }

	// Atomic bounded install, suitable as a selected-device admission sink.
	// All validation/capacity checks happen before either lane is changed.
	FAcquisitionTicket BeginAcquisition() const
	{ std::lock_guard<std::mutex> Lock(Gate); return {SessionId, PhysicalFence}; }
	bool Publish(FAcquisitionTicket Ticket, const FRawAcquisitionBatch& Batch) noexcept
	{
		try
		{
			std::lock_guard<std::mutex> Lock(Gate);
			if (Closed || Ticket.Session != SessionId || !Batch.Count || Batch.Count > Batch.Readings.size()
				|| (RequiresFresh && !Batch.Readings[0].FreshBaseline)) return false;
			auto Prepared = Physical;
			auto Previous = LastAcquired;
			bool Ready = ResumeReady;
			for (std::size_t I = 0; I < Batch.Count; ++I)
			{
				const auto& R = Batch.Readings[I];
				if (!R.DeviceId || !R.Generation.Value || !R.Sequence || !R.State.IsValid(R.Kind)
					|| (Previous && (Previous->Sequence == std::numeric_limits<std::uint64_t>::max()
						|| R.Sequence != Previous->Sequence + 1))) return false;
				const bool Same = Previous && SameDevice(*Previous, R);
				if ((!Same && !R.FreshBaseline) || (Same && !SameControls(Previous->State, R.State))
					|| (Same && R.TimestampMicroseconds < Previous->TimestampMicroseconds)) return false;
				if (!PhysicalClosed && Ticket.PhysicalFence == PhysicalFence
					&& (!Paused || Ready || (ResumeRequested && R.FreshBaseline)))
				{
					if (R.FreshBaseline)
					{
						Prepared = {}; Prepared.Baseline = R; Prepared.Latest = R;
						if (ResumeRequested) Ready = true;
					}
					else
					{
						if (!Prepared.Latest || !SameDevice(*Prepared.Latest, R)) return false;
						for (std::size_t J = 0; J < R.State.Count; ++J)
							if (Prepared.Latest->State.Values[J].Value != R.State.Values[J].Value)
							{
								if (Prepared.ChangeCount == Prepared.Changes.size()) return false;
								Prepared.Changes[Prepared.ChangeCount++] = {R.State.Values[J], {}, R.Sequence};
							}
						Prepared.Latest = R;
					}
				}
				Previous = R;
			}
			if (Serial > std::numeric_limits<std::uint64_t>::max() - Batch.Count) return false;
			Physical = Prepared; LastAcquired = Previous; ResumeReady = Ready; RequiresFresh = false;
			for (std::size_t I = 0; I < Batch.Count; ++I) History[(++Serial) % Capacity] = Batch.Readings[I];
			return true;
		}
		catch (...) { return false; } // Lock failure happens before installation.
	}

	std::optional<FRawInputSample> Poll(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return {};
		std::lock_guard<std::mutex> Lock(Gate);
		if (Closed || PhysicalClosed || Paused || Frame != NextFrame || Exhausted || !Physical.Latest
			|| DeliverySequence == std::numeric_limits<std::uint64_t>::max()) return {};
		const auto& R = Physical.Baseline ? *Physical.Baseline : *Physical.Latest;
		FRawInputSample Result;
		Result.DeviceId = R.DeviceId; Result.Generation = R.Generation; Result.Kind = R.Kind;
		Result.Sequence = DeliverySequence + 1;
		Result.Status = Physical.Baseline ? ERawSampleStatus::Resync : ERawSampleStatus::Valid;
		Result.FinalState.assign(R.State.Values.begin(), R.State.Values.begin() + R.State.Count);
		if (!Physical.Baseline)
		{
			Result.Changes.reserve(Physical.ChangeCount);
			for (std::size_t I = 0; I < Physical.ChangeCount; ++I)
			{
				auto Change = Physical.Changes[I]; Change.Order = {Result.Sequence, static_cast<std::uint32_t>(I)};
				Result.Changes.push_back(Change);
			}
		}
		if (!Result.IsValid()) return {};
		// Do not discard edges after a baseline: Reset cannot contain transitions,
		// so those real later readings belong to the next physical consumption.
		if (Physical.Baseline) Physical.Baseline.reset(); else Physical.ChangeCount = 0;
		++DeliverySequence; Exhausted = Frame == std::numeric_limits<FFrameNumber>::max();
		if (!Exhausted) ++NextFrame;
		return Result;
	}

	ELifecycleResult SetLifecyclePaused(bool Value) override
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Closed || PhysicalClosed) return ELifecycleResult::Rejected;
		if (Value)
		{
			if (PhysicalFence == std::numeric_limits<std::uint64_t>::max()) return ELifecycleResult::Rejected;
			++PhysicalFence;
			Paused = true; ResumeRequested = ResumeReady = false; Physical = {};
			return ELifecycleResult::Applied;
		}
		if (!Paused) return ELifecycleResult::Unaffected;
		if (!ResumeReady) return ELifecycleResult::Rejected;
		Paused = false; ResumeRequested = ResumeReady = false;
		return ELifecycleResult::Applied;
	}
	// Request from the quiescent host authorizes logical resume at the first
	// NEW current reading acquired with the new fence. IsResumeReady acknowledges
	// that exact boundary: later edges are post-resume and must be retained even
	// if waking the physical worker is delayed. SetLifecyclePaused(false) only
	// releases consumption; it is NOT a second discard boundary.
	// Before requesting this, the host must have resolved every other pause gate.
	// Merely resubmitting a cached state is forbidden.
	bool RequestFreshResume()
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Closed || PhysicalClosed || !Paused || PhysicalFence == std::numeric_limits<std::uint64_t>::max()) return false;
		++PhysicalFence;
		ResumeRequested = true; ResumeReady = false; Physical = {}; return true;
	}
	bool NeedsFreshResume() const
	{ std::lock_guard<std::mutex> Lock(Gate); return ResumeRequested && !ResumeReady; }
	bool IsResumeReady() const
	{ std::lock_guard<std::mutex> Lock(Gate); return ResumeRequested && ResumeReady; }
	ELifecycleResult CancelLifecycle() override
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		std::lock_guard<std::mutex> Lock(Gate);
		PhysicalClosed = true; Physical = {}; ResumeRequested = ResumeReady = false;
		return ELifecycleResult::Applied; // Control acquisition has a separate owner.
	}
	FAcquisitionHistory ReadControlsSince(FAcquisitionCursor Cursor) const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		FAcquisitionHistory Out; Out.Next = {SessionId, Serial};
		if (Closed) { Out.Status = EAcquisitionRead::Closed; return Out; }
		if (Cursor.Session != SessionId || Cursor.Serial > Serial)
		{ Out.Status = EAcquisitionRead::InvalidCursor; return Out; }
		if (Serial - Cursor.Serial > Capacity || Cursor.Serial < ControlBarrier)
		{ Out.Status = EAcquisitionRead::Gap; return Out; }
		if (Cursor.Serial == Serial) return Out;
		Out.Readings.reserve(static_cast<std::size_t>(Serial - Cursor.Serial));
		for (auto S = Cursor.Serial; S < Serial; ) Out.Readings.push_back(History[(++S) % Capacity]);
		Out.Status = EAcquisitionRead::Batch; return Out;
	}
	std::optional<FAcquisitionBaseline> ReadControlBaseline() const
	{
		std::lock_guard<std::mutex> Lock(Gate);
		if (Closed || RequiresFresh || !LastAcquired) return {};
		return FAcquisitionBaseline{{SessionId, Serial}, *LastAcquired};
	}
	// An acquisition failure is explicit on both lanes; there is no partial
	// suffix or cached held state to use. Recovery requires a fresh baseline.
	void Invalidate()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		Physical = {}; RequiresFresh = true;
		if (Serial == std::numeric_limits<std::uint64_t>::max()) Closed = true;
		else ControlBarrier = Serial + 1;
		ResumeReady = false;
	}
	void Close()
	{ std::lock_guard<std::mutex> Lock(Gate); Closed = true; Physical = {}; }
private:
	static bool SameDevice(const FAcquiredRawState& A, const FAcquiredRawState& B) noexcept
	{ return A.DeviceId == B.DeviceId && A.Generation.Value == B.Generation.Value && A.Kind == B.Kind; }
	static bool SameControls(const FCanonicalRawState& A, const FCanonicalRawState& B) noexcept
	{
		if (A.Count != B.Count) return false;
		for (std::size_t I = 0; I < A.Count; ++I) if (!(A.Values[I].Control == B.Values[I].Control)) return false;
		return true;
	}
	struct FPhysicalBuffer
	{
		std::optional<FAcquiredRawState> Baseline, Latest;
		std::array<FRawChange, FRawInputSample::MaxChanges> Changes{};
		std::size_t ChangeCount = 0;
	};
	mutable std::mutex Gate;
	const std::uint64_t SessionId;
	std::array<FAcquiredRawState, Capacity> History{};
	std::optional<FAcquiredRawState> LastAcquired;
	FPhysicalBuffer Physical;
	std::uint64_t Serial = 0, DeliverySequence = 0, ControlBarrier = 0;
	std::uint64_t PhysicalFence = 1;
	FFrameNumber NextFrame = 0;
	bool Closed = false, PhysicalClosed = false, Paused = false, ResumeRequested = false, ResumeReady = false, Exhausted = false;
	bool RequiresFresh = true;
};
}
