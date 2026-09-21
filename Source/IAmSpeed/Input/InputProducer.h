#pragma once

#include "InputFrame.h"
#include "InputPresentationScope.h"
#include <limits>
#include <mutex>
#include <optional>

namespace Speed::Input
{
enum class EInputLifecycleResult : std::uint8_t { Applied, UnaffectedByPolicy, Rejected };

class IInputProducer
{
public:
	virtual ~IInputProducer() = default;
	// A missing frame is a hard boundary, not permission to sample another clock
	// or silently predict. Returned copies expose only const payload access.
	virtual std::optional<FInputFrame> Produce(FFrameNumber ConsumptionFrame) = 0;
	// Explicit suppression by the sealed test owner, never an implicit missing input.
	virtual bool Skip(FFrameNumber) { return false; }
	// Compatibility policy for sources without live acquisition state. Such a
	// source is gated by its stream but its authored timeline is never rewound.
	// Device hosts must override these hooks before production activation.
	virtual EInputLifecycleResult ApplyLifecyclePause(bool) { return EInputLifecycleResult::UnaffectedByPolicy; }
	// Mandatory non-polling cleanup; unlike ordinary control, allowed during
	// presentation. This cancels source state, not OS callback/backend shutdown.
	virtual EInputLifecycleResult CancelLifecycle() { return EInputLifecycleResult::UnaffectedByPolicy; }
};

/** Injected device-sample adapter; real acquisition backend is not supplied.
 * Complete state and ordered edges are exchanged
 * under one short lock. Live arrivals are nondeterministic; recorded frames
 * replay exactly. This is not a replacement for the simulation's sealed journal.
 */
class FDeviceInputProducer final : public IInputProducer
{
public:
	explicit FDeviceInputProducer(std::uint64_t Id) : Identity{EProducerKind::Device, Id} {}

	// Values are already mapped/filtered/quantized by an independent backend. Digital
	// transitions use 0 = released, nonzero = held; axes opt out of edge emission.
	// One acquisition writer stages a source sample, then commits it explicitly.
	// Produce never observes staging. A failed staging call requires cancel/retry.
	bool SetAction(FFrameNumber Source, FActionId Action, std::int16_t Value, bool bEmitEdges)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		auto Reject = [&]() { StagingFailed = StagedSource.has_value(); return false; };
		if (StagingFailed) return false;
		if (!Identity.Id || Action >= ActionCount || (HasCommitted && Source <= LastSource)
			|| (StagedSource && Source != *StagedSource)) return Reject();
		FActionValues Next = StagedSource ? StagedHeld : Held;
		const auto Previous = Next[Action];
		Next[Action] = Value;
		if (!FInputFrame(Source, 0, Identity, Next).IsValid()) return Reject();
		const bool bTransition = bEmitEdges && ((Previous != 0) != (Value != 0));
		if (bTransition && EdgeCount + StagedEdgeCount == MaxEdges) return Reject();
		if (bTransition) StagedEdges[StagedEdgeCount++] = {Action, Value ? EEdgeKind::Start : EEdgeKind::Stop, Source};
		StagedHeld = Next;
		StagedSource = Source;
		return true;
	}
	bool CommitSample(FFrameNumber Source)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (StagingFailed || !StagedSource || *StagedSource != Source) return false;
		Held = StagedHeld;
		for (std::size_t I = 0; I < StagedEdgeCount; ++I) Edges[EdgeCount++] = StagedEdges[I];
		LastSource = Source;
		HasCommitted = true;
		StagedSource.reset(); StagedEdgeCount = 0; StagedEdges = {};
		return true;
	}
	void CancelSample()
	{
		if (FPresentationInputScope::IsActive()) return;
		std::lock_guard<std::mutex> Lock(Mutex);
		StagingFailed = false;
		StagedSource.reset(); StagedEdgeCount = 0; StagedEdges = {};
	}
	bool Skip(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		const bool PendingReset = ResetPending;
		const auto Result = ProduceLocked(Frame);
		// A suppressed frame cannot acknowledge physical cancellation.
		ResetPending = ResetPending || PendingReset;
		return bool(Result);
	}

	// Acquisition lifecycle only. Historical physical frames are never changed.
	// Reset cancellation survives fresh samples until the next physical consume.
	bool ResetSample(FFrameNumber Source)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (!Identity.Id || (HasCommitted && Source <= LastSource)) return false;
		Held = {}; StagedHeld = {}; Edges = {}; StagedEdges = {};
		EdgeCount = 0; StagedEdgeCount = 0; StagedSource.reset(); StagingFailed = false;
		LastSource = Source; HasCommitted = true; ResetPending = true;
		return true;
	}

	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Mutex);
		return ProduceLocked(Frame);
	}

private:
	std::optional<FInputFrame> ProduceLocked(FFrameNumber Frame)
	{
		auto& Slot = History[Frame % HistoryCapacity];
		if (Slot && Slot->GetConsumptionFrame() == Frame) return Slot;
		if (!Identity.Id || (LastConsumed && (*LastConsumed == std::numeric_limits<FFrameNumber>::max()
			|| Frame != *LastConsumed + 1))) return std::nullopt;
		Slot.emplace(LastSource, Frame, Identity, Held, Edges, EdgeCount, ResetPending);
		ResetPending = false;
		LastConsumed = Frame;
		EdgeCount = 0;
		Edges = {}; // No stale event bytes in a held-only snapshot.
		return Slot;
	}

private:
	const FProducerIdentity Identity;
	std::mutex Mutex;
	FActionValues Held{};
	FActionValues StagedHeld{};
	std::optional<FFrameNumber> StagedSource;
	std::array<FActionEdge, MaxEdges> StagedEdges{};
	std::size_t StagedEdgeCount = 0;
	bool HasCommitted = false;
	bool StagingFailed = false;
	bool ResetPending = false;
	std::array<FActionEdge, MaxEdges> Edges{};
	std::size_t EdgeCount = 0;
	FFrameNumber LastSource = 0;
	std::optional<FFrameNumber> LastConsumed;
	std::array<std::optional<FInputFrame>, HistoryCapacity> History{};
};

/** Bounded exact-frame inbox and history for scripted/transport adapters.
 * No prediction, duplicate replacement, eviction of unread input, or transport.
 * A fresh stream begins at the explicitly supplied canonical frame.
 */
class FQueuedInputProducer : public IInputProducer
{
public:
	EInputLifecycleResult ApplyLifecyclePause(bool) override { return EInputLifecycleResult::UnaffectedByPolicy; }
	EInputLifecycleResult CancelLifecycle() override { return EInputLifecycleResult::UnaffectedByPolicy; }
	// Advance a suppressed exact frame even if absent. Preserve submitted payloads
	// in producer history, without claiming the stream physically consumed them.
	bool Skip(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (bExhausted || Frame != NextFrame) return false;
		auto& Slot = Pending[Frame % HistoryCapacity];
		History[Frame % HistoryCapacity] = Slot;
		Slot.reset();
		bExhausted = Frame == std::numeric_limits<FFrameNumber>::max();
		if (!bExhausted) ++NextFrame;
		return true;
	}
	bool Submit(const FInputFrame& Input)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		const auto Frame = Input.GetConsumptionFrame();
		const auto Producer = Input.GetProducer();
		if (bExhausted || !Input.IsValid() || Producer.Kind != Identity.Kind || Producer.Id != Identity.Id
			|| Frame < NextFrame || Frame - NextFrame >= HistoryCapacity) return false;
		auto& Slot = Pending[Frame % HistoryCapacity];
		if (Slot) return false;
		Slot = Input;
		return true;
	}

	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Mutex);
		const auto& Old = History[Frame % HistoryCapacity];
		if (Old && Old->GetConsumptionFrame() == Frame) return Old;
		if (bExhausted || Frame != NextFrame) return std::nullopt;
		auto& Slot = Pending[Frame % HistoryCapacity];
		if (!Slot || Slot->GetConsumptionFrame() != Frame) return std::nullopt;
		auto Result = Slot;
		History[Frame % HistoryCapacity] = Slot;
		Slot.reset();
		bExhausted = Frame == std::numeric_limits<FFrameNumber>::max();
		if (!bExhausted) ++NextFrame;
		return Result;
	}

protected:
	FQueuedInputProducer(FProducerIdentity Producer, FFrameNumber FirstFrame)
		: Identity(Producer), NextFrame(FirstFrame) {}

private:
	const FProducerIdentity Identity;
	std::mutex Mutex;
	FFrameNumber NextFrame;
	bool bExhausted = false;
	std::array<std::optional<FInputFrame>, HistoryCapacity> Pending{};
	std::array<std::optional<FInputFrame>, HistoryCapacity> History{};
};

class FAIInputProducer final : public FQueuedInputProducer
{
public:
	FAIInputProducer(std::uint64_t Id, FFrameNumber FirstFrame)
		: FQueuedInputProducer({EProducerKind::AI, Id}, FirstFrame) {}
};

class FNetworkInputProducer final : public FQueuedInputProducer
{
public:
	FNetworkInputProducer(std::uint64_t Id, FFrameNumber FirstFrame)
		: FQueuedInputProducer({EProducerKind::Network, Id}, FirstFrame) {}

	// CanMoveFrame is canonical zero-based. Signed offsets allow pre-CanMove
	// input. Packet validation/authority and remote-to-local mapping stay outside.
	static std::optional<FFrameNumber> FromCanMove(FFrameNumber CanMoveFrame, std::int64_t Offset)
	{
		if (Offset < 0)
		{
			const auto Magnitude = static_cast<FFrameNumber>(-(Offset + 1)) + 1;
			if (Magnitude > CanMoveFrame) return std::nullopt;
			return CanMoveFrame - Magnitude;
		}
		const auto Positive = static_cast<FFrameNumber>(Offset);
		if (Positive > std::numeric_limits<FFrameNumber>::max() - CanMoveFrame) return std::nullopt;
		return CanMoveFrame + Positive;
	}
};
}
