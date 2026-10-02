#pragma once

#include "ActionMapping.h"
#include "InputProducerPollFence.h"

namespace Speed::Input::V2
{
enum class EProducerContract : std::uint8_t { Unknown, Device, ExactScenario, ExactRemote, AI };
enum class ELifecycleResult : std::uint8_t { Applied, Unaffected, Rejected };
class IInputProducer
{
public:
	virtual ~IInputProducer() = default;
	virtual IInputProducerPollFence* PollFence() noexcept { return nullptr; }
	virtual bool SupportsPollFence() const noexcept { return false; }
	virtual EProducerContract GetProducerContract() const noexcept { return EProducerContract::Unknown; }
	virtual std::optional<FInputFrame> InspectBaseline(FFrameNumber) const { return {}; }
	virtual std::optional<FInputFrame> Produce(FFrameNumber Frame) = 0;
	// Exact test authoring only: seal a future frame on the owner before poll.
	// Device/network producers deliberately keep the rejecting default.
	virtual bool AppendExactScenarioFrame(const FInputFrame&) { return false; }
	virtual const std::shared_ptr<const FInputActionContract>& GetContract() const = 0;
	// Called by the host only at a quiescent owner boundary. Sealed timelines
	// ignore device policy but remain subject to stream pause admission.
	virtual ELifecycleResult SetLifecyclePaused(bool) { return ELifecycleResult::Unaffected; }
	virtual ELifecycleResult CancelLifecycle() { return ELifecycleResult::Unaffected; }
};

// Acquisition boundary only: no controller, presentation receiver or action
// callback. Implementations are polled by the sole physical consumer.
class IRawInputSource
{
public:
	virtual ~IRawInputSource() = default;
	virtual IInputProducerPollFence* PollFence() noexcept { return nullptr; }
	virtual std::optional<FRawInputSample> Poll(FFrameNumber Frame) = 0;
	// A platform adapter must explicitly implement pause/resume cursor reset.
	// Reject by default rather than accepting stale held/edge history.
	virtual ELifecycleResult SetLifecyclePaused(bool) { return ELifecycleResult::Rejected; }
	virtual ELifecycleResult CancelLifecycle() { return ELifecycleResult::Rejected; }
};
class FDeviceInputProducer final : public IInputProducer, public IInputProducerPollFence
{
public:
	IInputProducerPollFence* PollFence() noexcept override { return this; }
	bool SupportsPollFence() const noexcept override { return true; }
	EProducerContract GetProducerContract() const noexcept override { return EProducerContract::Device; }
	static std::unique_ptr<FDeviceInputProducer> Create(std::shared_ptr<IRawInputSource> Raw,
		std::shared_ptr<const FInputActionContract> Contract, FStreamEpoch Epoch,
		FProducerIdentity Identity, FFrameNumber FirstFrame = 0, std::uint32_t ResumeRearmMask = 0)
	{
		if (FPresentationInputScope::IsActive() || !Raw || !Contract) return {};
		for (FActionId Action = 0; Action < ActionCount; ++Action)
		{
			if (!(ResumeRearmMask & (std::uint32_t{1} << Action))) continue;
			const auto* Definition = Contract->Find(Action);
			if (!Definition || Definition->Wiring != EActionWiring::Wired || Definition->Type != EActionType::Bool) return {};
		}
		auto Mapper = FActionMapper::Create(Contract, Epoch, Identity, FirstFrame);
		if (!Mapper) return {};
		return std::unique_ptr<FDeviceInputProducer>(new FDeviceInputProducer(std::move(Raw), std::move(Mapper), ResumeRearmMask));
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return {};
		auto Sample = Raw->Poll(Frame);
		if (!Sample) return {};
		auto Mapped = Mapper->Map(*Sample, Frame).Frame;
		if (!Mapped) return {};
		if (!PendingRearmMask) { HasProduced = true; return Mapped; }
		auto Data = Mapped->GetData();
		auto Pending = PendingRearmMask;
		if (!Data.Reset)
		{
			std::vector<FActionTransition> Kept;
			Kept.reserve(Data.Transitions.size());
			for (const auto& Edge : Data.Transitions)
			{
				const auto Bit = std::uint32_t{1} << Edge.Action;
				if (Pending & Bit)
				{
					if (Edge.State == ETransition::Completed) Pending &= ~Bit;
					continue;
				}
				Kept.push_back(Edge);
			}
			Data.Transitions = std::move(Kept);
		}
		for (FActionId Action = 0; Action < ActionCount; ++Action)
		{
			const auto Bit = std::uint32_t{1} << Action;
			if (!(Pending & Bit)) continue;
			if (!(Data.ActiveMask & Bit)) { Pending &= ~Bit; continue; }
			Data.Values[Action] = 0;
			Data.ActiveMask &= ~Bit;
		}
		FInputFrame Filtered(Mapper->GetContract(), std::move(Data));
		if (!Filtered.IsValidFor(*Mapper->GetContract())) return {};
		PendingRearmMask = Pending;
		HasProduced = true;
		return Filtered;
	}
	std::optional<FInputPollCutoff> FreezeForOwner(FFrameNumber N) override
	{
		auto* Fence = Raw->PollFence();
		return Fence ? Fence->FreezeForOwner(N) : std::nullopt;
	}
	bool CloseFrozenCutoff(const FInputPollCutoff& Cutoff) noexcept override
	{
		auto* Fence = Raw->PollFence();
		return Fence && Fence->CloseFrozenCutoff(Cutoff);
	}
	const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Mapper->GetContract(); }
	ELifecycleResult SetLifecyclePaused(bool Paused) override
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		const auto Result = Raw->SetLifecyclePaused(Paused);
		if (Result != ELifecycleResult::Applied) return ELifecycleResult::Rejected;
		Mapper->RequireFreshBaseline();
		if (Paused)
		{
			// Initial registration pauses before any physical frame; it is not a menu.
			RearmOnNextResume = HasProduced;
			PendingRearmMask = 0;
		}
		else
		{
			PendingRearmMask = RearmOnNextResume ? ResumeRearmMask : 0;
			RearmOnNextResume = false;
		}
		return ELifecycleResult::Applied;
	}
	ELifecycleResult CancelLifecycle() override
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		const auto Result = Raw->CancelLifecycle();
		Mapper->RequireFreshBaseline();
		PendingRearmMask = 0;
		RearmOnNextResume = false;
		return Result;
	}
private:
	FDeviceInputProducer(std::shared_ptr<IRawInputSource> InRaw, std::unique_ptr<FActionMapper> InMapper,
		std::uint32_t InResumeRearmMask)
		: Raw(std::move(InRaw)), Mapper(std::move(InMapper)), ResumeRearmMask(InResumeRearmMask) {}
	const std::shared_ptr<IRawInputSource> Raw;
	const std::unique_ptr<FActionMapper> Mapper;
	const std::uint32_t ResumeRearmMask = 0;
	std::uint32_t PendingRearmMask = 0;
	bool HasProduced = false;
	bool RearmOnNextResume = false;
};
}
