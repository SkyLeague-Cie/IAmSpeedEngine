#pragma once

#include "IAmSpeed/Input/InputProducer.h"
#include <memory>
#include <vector>

namespace Speed::Input
{
/** Explicit opt-in CI source, never a live device adapter. The entire scenario
 * is validated and copied before use; no caller-owned mutable data is retained.
 * Entries contain canonical actions, ordered edges and reset markers. Mapping
 * raw devices is a separate upstream test. No interpolation or held fallback.
 */
class FTestInputProducer final : public IInputProducer
{
public:
	static std::unique_ptr<FTestInputProducer> Create(FProducerIdentity Identity,
		FFrameNumber FirstFrame, const std::vector<FInputFrame>& Scenario)
	{
		if (FPresentationInputScope::IsActive() || !Identity.Id
			|| Identity.Kind > EProducerKind::Network || Scenario.empty()) return nullptr;
		if (Scenario.size() - 1 > std::numeric_limits<FFrameNumber>::max() - FirstFrame) return nullptr;
		for (std::size_t I = 0; I < Scenario.size(); ++I)
		{
			const auto& F = Scenario[I];
			if (!F.IsValid() || F.GetConsumptionFrame() != FirstFrame + I
				|| F.GetProducer().Kind != Identity.Kind || F.GetProducer().Id != Identity.Id
				|| (I && F.GetSourceFrame() < Scenario[I - 1].GetSourceFrame())) return nullptr;
		}
		return std::unique_ptr<FTestInputProducer>(new FTestInputProducer(FirstFrame, Scenario));
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Frame < First || Frame - First >= Frames.size()) return std::nullopt;
		const auto Index = static_cast<std::size_t>(Frame - First);
		if (Index > Next || (Index < Next && Next - Index > HistoryCapacity)) return std::nullopt;
		if (Index == Next) ++Next;
		return Frames[Index];
	}
	// Deliberately retain IInputProducer::Skip=false: the legacy sealed override
	// must not silently suppress CI actions intended for this common consumer.
private:
	FTestInputProducer(FFrameNumber FirstFrame, const std::vector<FInputFrame>& Scenario)
		: First(FirstFrame), Frames(Scenario) {}
	const FFrameNumber First;
	const std::vector<FInputFrame> Frames;
	std::mutex Gate;
	std::size_t Next = 0;
};
}
