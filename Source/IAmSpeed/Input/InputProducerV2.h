#pragma once

#include "ActionMapping.h"

namespace Speed::Input::V2
{
enum class ELifecycleResult : std::uint8_t { Applied, Unaffected, Rejected };
class IInputProducer
{
public:
	virtual ~IInputProducer() = default;
	virtual std::optional<FInputFrame> Produce(FFrameNumber Frame) = 0;
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
	virtual std::optional<FRawInputSample> Poll(FFrameNumber Frame) = 0;
	// A platform adapter must explicitly implement pause/resume cursor reset.
	// Reject by default rather than accepting stale held/edge history.
	virtual ELifecycleResult SetLifecyclePaused(bool) { return ELifecycleResult::Rejected; }
	virtual ELifecycleResult CancelLifecycle() { return ELifecycleResult::Rejected; }
};
class FDeviceInputProducer final : public IInputProducer
{
public:
	static std::unique_ptr<FDeviceInputProducer> Create(std::shared_ptr<IRawInputSource> Raw,
		std::shared_ptr<const FInputActionContract> Contract, FStreamEpoch Epoch,
		FProducerIdentity Identity, FFrameNumber FirstFrame = 0)
	{
		if (FPresentationInputScope::IsActive() || !Raw) return {};
		auto Mapper = FActionMapper::Create(Contract, Epoch, Identity, FirstFrame);
		if (!Mapper) return {};
		return std::unique_ptr<FDeviceInputProducer>(new FDeviceInputProducer(std::move(Raw), std::move(Mapper)));
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		if (FPresentationInputScope::IsActive()) return {};
		auto Sample = Raw->Poll(Frame);
		if (!Sample) return {};
		return Mapper->Map(*Sample, Frame).Frame;
	}
	const std::shared_ptr<const FInputActionContract>& GetContract() const override { return Mapper->GetContract(); }
	ELifecycleResult SetLifecyclePaused(bool Paused) override
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		const auto Result = Raw->SetLifecyclePaused(Paused);
		if (Result != ELifecycleResult::Applied) return ELifecycleResult::Rejected;
		Mapper->RequireFreshBaseline();
		return ELifecycleResult::Applied;
	}
	ELifecycleResult CancelLifecycle() override
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		const auto Result = Raw->CancelLifecycle();
		Mapper->RequireFreshBaseline();
		return Result;
	}
private:
	FDeviceInputProducer(std::shared_ptr<IRawInputSource> InRaw, std::unique_ptr<FActionMapper> InMapper)
		: Raw(std::move(InRaw)), Mapper(std::move(InMapper)) {}
	const std::shared_ptr<IRawInputSource> Raw;
	const std::unique_ptr<FActionMapper> Mapper;
};
}
