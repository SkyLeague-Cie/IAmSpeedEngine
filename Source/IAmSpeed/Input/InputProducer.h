#pragma once

#include "InputFrame.h"
#include "InputPresentationScope.h"
#include <limits>
#include <mutex>
#include <optional>

namespace Speed::Input
{
class IInputProducer
{
public:
	virtual ~IInputProducer() = default;
	// A missing frame is a hard boundary, not permission to sample another clock
	// or silently predict. Returned copies expose only const payload access.
	virtual std::optional<FInputFrame> Produce(FFrameNumber ConsumptionFrame) = 0;
};

/** GameThread/device adapter. Complete state and ordered edges are exchanged
 * under one short lock. Live arrivals are nondeterministic; recorded frames
 * replay exactly. This is not a replacement for the simulation's sealed journal.
 */
class FDeviceInputProducer final : public IInputProducer
{
public:
	explicit FDeviceInputProducer(std::uint64_t Id) : Identity{EProducerKind::Device, Id} {}

	// Values are already mapped/filtered/quantized by the game adapter. Digital
	// transitions use 0 = released, nonzero = held; axes opt out of edge emission.
	bool SetAction(FFrameNumber Source, FActionId Action, std::int16_t Value, bool bEmitEdges)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (!Identity.Id || Action >= ActionCount || Source < LastSource) return false;
		FActionValues Next = Held;
		Next[Action] = Value;
		if (!FInputFrame(Source, 0, Identity, Next).IsValid()) return false;
		const bool bTransition = bEmitEdges && ((Held[Action] != 0) != (Value != 0));
		if (bTransition && EdgeCount == MaxEdges) return false; // No partial update/drop.
		if (bTransition) Edges[EdgeCount++] = {Action, Value ? EEdgeKind::Start : EEdgeKind::Stop, Source};
		Held = Next;
		LastSource = Source;
		return true;
	}

	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Mutex);
		auto& Slot = History[Frame % HistoryCapacity];
		if (Slot && Slot->GetConsumptionFrame() == Frame) return Slot;
		if (!Identity.Id || (LastConsumed && (*LastConsumed == std::numeric_limits<FFrameNumber>::max()
			|| Frame != *LastConsumed + 1))) return std::nullopt;
		Slot.emplace(LastSource, Frame, Identity, Held, Edges, EdgeCount);
		LastConsumed = Frame;
		EdgeCount = 0;
		Edges = {}; // No stale event bytes in a held-only snapshot.
		return Slot;
	}

private:
	const FProducerIdentity Identity;
	std::mutex Mutex;
	FActionValues Held{};
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
