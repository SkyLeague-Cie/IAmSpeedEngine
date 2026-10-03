#pragma once

#include "IAmSpeed/Input/InputProducerV2.h"
#include "IAmSpeed/Input/InputPresentationScope.h"
#include <mutex>
#include <deque>
#include <thread>

namespace Speed::Input::V2
{
// Test-only sealed action frames, AFTER the mapper. No v1 conversion and no
// hidden raw-device evaluation. The same stream consumes player and test frames.
class FTestInputProducer final : public IInputProducer, public IInputProducerPollFence
{
public:
	IInputProducerPollFence* PollFence() noexcept override { return this; }
	bool SupportsPollFence() const noexcept override { return true; }
	EProducerContract GetProducerContract() const noexcept override { return EProducerContract::ExactScenario; }
	std::optional<FInputFrame> InspectBaseline(FFrameNumber Frame) const override
	{
		std::lock_guard<std::mutex> Lock(Gate);
		if (Frame < First || Frame - First != Next || Next >= RetainedFirst + Frames.size() || !Frames[Next - RetainedFirst].GetData().Reset) return {};
		return Frames[Next - RetainedFirst];
	}
	bool AppendExactScenarioFrame(const FInputFrame& Frame) override
	{
		if (std::this_thread::get_id() != AuthorThread || FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		const auto& D = Frame.GetData();
		const auto Total = RetainedFirst + Frames.size();
		if (FrozenFrame || Frames.empty() || Total > UINT64_MAX - First
			|| D.ConsumptionFrame != First + Total || !Frame.IsValidFor(*Contract)
			|| !ValidSuccessor(Frames.back().GetData(), D)) return false;
		// Keep bounded consumed history and bounded look-ahead. Never discard an
		// unconsumed frame to make room or overwrite a sealed frame.
		if (Total - Next >= HistoryCapacity) return false;
		try { Frames.push_back(Frame); } catch (...) { return false; }
		while (RetainedFirst < Next && Next - RetainedFirst > HistoryCapacity)
		{ Frames.pop_front(); ++RetainedFirst; }
		return true;
	}
	static std::unique_ptr<FTestInputProducer> Create(std::shared_ptr<const FInputActionContract> Contract,
		FStreamEpoch Epoch, FProducerIdentity Producer, FFrameNumber FirstFrame, const std::vector<FInputFrame>& Frames)
	{
		if (FPresentationInputScope::IsActive() || !Contract || !Epoch.Value || !Producer.Id
			|| Producer.Kind > EProducerKind::Network || Frames.empty()
			|| Frames.size() - 1 > std::numeric_limits<FFrameNumber>::max() - FirstFrame) return {};
		for (std::size_t I = 0; I < Frames.size(); ++I)
		{
			const auto& D = Frames[I].GetData();
			if (!Frames[I].IsValidFor(*Contract) || D.ConsumptionFrame != FirstFrame + I
				|| D.StreamEpoch.Value != Epoch.Value || D.Producer.Kind != Producer.Kind || D.Producer.Id != Producer.Id) return {};
			if (!I) { if (!D.Reset) return {}; continue; }
			if (!ValidSuccessor(Frames[I - 1].GetData(), D)) return {};
		}
		return std::unique_ptr<FTestInputProducer>(new FTestInputProducer(std::move(Contract), FirstFrame, Frames));
	}
	std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return {};
		std::lock_guard<std::mutex> Lock(Gate);
		if (FrozenFrame || Frame < First || Frame - First != Next || Next >= RetainedFirst + Frames.size()) return {};
		FrozenFrame = Frame; FrozenDelivered = false;
		const auto& D = Frames[Next - RetainedFirst].GetData();
		return FInputPollCutoff{D.SourceSequence, D.DeviceGeneration.Value, 1};
	}
	bool CloseFrozenCutoff(const FInputPollCutoff& Cutoff) noexcept override
	{
		try
		{
			std::lock_guard<std::mutex> Lock(Gate);
			bool Valid = false;
			if (FrozenFrame && FrozenDelivered)
			{
				const auto& D = Frames[static_cast<std::size_t>(*FrozenFrame - First) - RetainedFirst].GetData();
				Valid = Cutoff.LifecycleFence == 1 && Cutoff.Sequence == D.SourceSequence && Cutoff.Generation == D.DeviceGeneration.Value;
			}
			FrozenFrame.reset(); FrozenDelivered = false; return Valid;
		}
		catch (...) { return false; }
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return {};
		std::lock_guard<std::mutex> Lock(Gate);
		if (Frame < First || Frame - First < RetainedFirst || Frame - First >= RetainedFirst + Frames.size()) return {};
		if (FrozenFrame)
		{
			if (*FrozenFrame != Frame || FrozenDelivered) return {};
			FrozenDelivered = true;
		}
		const auto Index = static_cast<std::size_t>(Frame - First);
		if (Index > Next || (Index < Next && Next - Index > HistoryCapacity)) return {};
		if (Index == Next) ++Next;
		return Frames[Index - RetainedFirst];
	}
	const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Contract; }
private:
	static bool ValidSuccessor(const FInputFrameData& Previous, const FInputFrameData& D)
	{
		if (D.StreamEpoch.Value != Previous.StreamEpoch.Value || D.Producer.Kind != Previous.Producer.Kind
			|| D.Producer.Id != Previous.Producer.Id || D.ConsumptionFrame != Previous.ConsumptionFrame + 1
			|| Previous.ConsumptionFrame == UINT64_MAX) return false;
		if (D.Reset) return true;
		if (D.DeviceGeneration.Value != Previous.DeviceGeneration.Value || D.SourceSequence < Previous.SourceSequence) return false;
		if (D.SourceSequence == Previous.SourceSequence && (D.Values != Previous.Values || !D.Transitions.empty())) return false;
		std::uint32_t Active = Previous.ActiveMask;
		for (const auto& E : D.Transitions)
		{
			const auto Bit = std::uint32_t{1} << E.Action;
			const bool Started = E.State == ETransition::Started;
			if (E.Order.Sequence <= Previous.SourceSequence || ((Active & Bit) != 0) == Started) return false;
			if (Started) Active |= Bit; else Active &= ~Bit;
		}
		return Active == D.ActiveMask;
	}
	FTestInputProducer(std::shared_ptr<const FInputActionContract> InContract, FFrameNumber FirstFrame, const std::vector<FInputFrame>& Scenario)
		: Contract(std::move(InContract)), First(FirstFrame), Frames(Scenario.begin(), Scenario.end()), AuthorThread(std::this_thread::get_id()) {}
	const std::shared_ptr<const FInputActionContract> Contract;
	const FFrameNumber First;
	std::deque<FInputFrame> Frames; // Elements are immutable after append.
	const std::thread::id AuthorThread;
	std::size_t RetainedFirst = 0;
	mutable std::mutex Gate;
	std::size_t Next = 0;
	std::optional<FFrameNumber> FrozenFrame;
	bool FrozenDelivered = false;
};
}
