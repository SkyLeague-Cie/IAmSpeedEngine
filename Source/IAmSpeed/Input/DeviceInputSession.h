#pragma once

#include "InputProducer.h"

namespace Speed::Input
{
/** Portable single-player acquisition lifecycle. No OS handles or clocks.
 * A backend supplies complete mapped readings in increasing ReadingSequence
 * order for the current generation. Lifecycle changes invalidate old readings.
 * Never hold backend locks while calling into this object.
 */
class FDeviceInputSession final : public IInputProducer
{
public:
	using FGeneration = std::uint64_t;
	explicit FDeviceInputSession(std::uint64_t Identity,
		const std::array<bool, ActionCount>& DigitalActions)
		: Device(Identity), Digital(DigitalActions) {}

	EInputLifecycleResult ApplyLifecyclePause(bool Paused) override
	{
		return SetPaused(Paused) ? EInputLifecycleResult::Applied : EInputLifecycleResult::Rejected;
	}
	EInputLifecycleResult CancelLifecycle() override
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		// Closing never enables acquisition, including when generation is exhausted.
		bPaused = true;
		return ResetLocked() ? EInputLifecycleResult::Applied : EInputLifecycleResult::Rejected;
	}

	// Each call starts a fresh generation, including a same-state reconnect.
	// The caller must restart its reading cursor from a fresh current state.
	std::optional<FGeneration> SetConnected(bool Connected)
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (!ResetLocked()) return std::nullopt;
		bConnected = Connected;
		return Generation;
	}
	std::optional<FGeneration> SetPaused(bool Paused)
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (!ResetLocked()) return std::nullopt;
		bPaused = Paused;
		return Generation;
	}
	// History loss has the same neutral/resynchronize boundary as reconnection.
	std::optional<FGeneration> Resynchronize()
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Mutex);
		return ResetLocked() ? std::optional<FGeneration>(Generation) : std::nullopt;
	}
	bool Submit(FGeneration Token, std::uint64_t ReadingSequence, const FActionValues& Values)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (!bConnected || bPaused || Token != Generation || !Token
			|| (LastReading && ReadingSequence <= *LastReading)
			|| SourceSequence == std::numeric_limits<FFrameNumber>::max()) return false;
		const auto Source = SourceSequence + 1;
		for (std::size_t I = 0; I < ActionCount; ++I)
		{
			if (!Device.SetAction(Source, static_cast<FActionId>(I), Values[I], !NeedsBaseline && Digital[I]))
			{
				Device.CancelSample();
				return false; // Backend must handle failure, never drop it silently.
			}
		}
		if (!Device.CommitSample(Source)) { Device.CancelSample(); return false; }
		SourceSequence = Source;
		LastReading = ReadingSequence;
		NeedsBaseline = false;
		return true;
	}
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		return Device.Produce(Frame);
	}
	bool Skip(FFrameNumber Frame) override
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		return Device.Skip(Frame);
	}

private:
	bool ResetLocked()
	{
		if (Generation == std::numeric_limits<FGeneration>::max()
			|| SourceSequence == std::numeric_limits<FFrameNumber>::max()) return false;
		if (!Device.ResetSample(SourceSequence + 1)) return false;
		++SourceSequence; ++Generation; LastReading.reset(); NeedsBaseline = true;
		return true;
	}
	std::mutex Mutex;
	FDeviceInputProducer Device;
	const std::array<bool, ActionCount> Digital;
	FGeneration Generation = 0;
	FFrameNumber SourceSequence = 0;
	std::optional<std::uint64_t> LastReading;
	bool bConnected = false;
	bool bPaused = false;
	bool NeedsBaseline = true;
};
}
