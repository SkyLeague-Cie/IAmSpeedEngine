#pragma once

#include "IAmSpeed/Input/InputProducerV2.h"
#include "IAmSpeed/Input/InputPresentationScope.h"
#include <mutex>

namespace Speed::Input::V2
{
// Test-only sealed action frames, AFTER the mapper. No v1 conversion and no
// hidden raw-device evaluation. The same stream consumes player and test frames.
class FTestInputProducer final : public IInputProducer, public IInputProducerPollFence
{
public:
	EProducerContract GetProducerContract() const noexcept override { return EProducerContract::ExactScenario; }
	std::optional<FInputFrame> InspectBaseline(FFrameNumber Frame) const override
	{
		std::lock_guard<std::mutex> Lock(Gate);
		if (Frame < First || Frame - First != Next || Next >= Frames.size() || !Frames[Next].GetData().Reset) return {};
		return Frames[Next];
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
			const auto& Previous = Frames[I - 1].GetData();
			if (!D.Reset)
			{
				if (D.DeviceGeneration.Value != Previous.DeviceGeneration.Value || D.SourceSequence < Previous.SourceSequence) return {};
				if (D.SourceSequence == Previous.SourceSequence && (D.Values != Previous.Values || !D.Transitions.empty())) return {};
				std::uint32_t Active = Previous.ActiveMask;
				for (const auto& E : D.Transitions)
				{
					const auto Bit = std::uint32_t{1} << E.Action;
					const bool Started = E.State == ETransition::Started;
					if (E.Order.Sequence <= Previous.SourceSequence || ((Active & Bit) != 0) == Started) return {};
					if (Started) Active |= Bit; else Active &= ~Bit;
				}
				if (Active != D.ActiveMask) return {};
			}
		}
		return std::unique_ptr<FTestInputProducer>(new FTestInputProducer(std::move(Contract), FirstFrame, Frames));
	}
	std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return {};
		std::lock_guard<std::mutex> Lock(Gate);
		if (FrozenFrame || Frame < First || Frame - First != Next || Next >= Frames.size()) return {};
		FrozenFrame = Frame; FrozenDelivered = false;
		const auto& D = Frames[Next].GetData();
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
				const auto& D = Frames[static_cast<std::size_t>(*FrozenFrame - First)].GetData();
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
		if (Frame < First || Frame - First >= Frames.size()) return {};
		if (FrozenFrame)
		{
			if (*FrozenFrame != Frame || FrozenDelivered) return {};
			FrozenDelivered = true;
		}
		const auto Index = static_cast<std::size_t>(Frame - First);
		if (Index > Next || (Index < Next && Next - Index > HistoryCapacity)) return {};
		if (Index == Next) ++Next;
		return Frames[Index];
	}
	const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Contract; }
private:
	FTestInputProducer(std::shared_ptr<const FInputActionContract> InContract, FFrameNumber FirstFrame, const std::vector<FInputFrame>& Scenario)
		: Contract(std::move(InContract)), First(FirstFrame), Frames(Scenario) {}
	const std::shared_ptr<const FInputActionContract> Contract;
	const FFrameNumber First;
	const std::vector<FInputFrame> Frames;
	mutable std::mutex Gate;
	std::size_t Next = 0;
	std::optional<FFrameNumber> FrozenFrame;
	bool FrozenDelivered = false;
};
}
