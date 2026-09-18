#pragma once

#include "InputProducer.h"
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace Speed::Input
{
#if WITH_DEV_AUTOMATION_TESTS
struct FControllerInputTestAccess;
#endif
struct FPublishedInputFrame
{
	FInputFrame Frame;
	std::uint64_t Serial;
};

/** Controller-owned, shared with the worker. No controller pointer is retained.
 * The worker is the only Consume caller; GT can only copy Latest/Recorded.
 * Storage is bounded to 256 completed physical frames. Polling never drains it.
 */
class FInputStream final
{
#if WITH_DEV_AUTOMATION_TESTS
	friend struct FControllerInputTestAccess;
#endif
public:
	explicit FInputStream(std::shared_ptr<IInputProducer> Producer) : Source(std::move(Producer)) {}
	std::optional<FInputFrame> Consume(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive() || !Source) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (!Active) return std::nullopt;
		const auto& Recorded = History[Frame % HistoryCapacity];
		if (Recorded && Recorded->GetConsumptionFrame() == Frame) return Recorded;
		auto Input = Source->Produce(Frame);
		if (!Input || !Input->IsValid() || Input->GetConsumptionFrame() != Frame) return std::nullopt;
		History[Frame % HistoryCapacity] = Input;
		return Input;
	}
	// Suppressed live frames (sealed test owns input) must not be published as
	// physically consumed. Producer-specific policy advances its exact cadence.
	bool Skip(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		return Active && Source && Source->Skip(Frame);
	}
	// Mandatory lifecycle cleanup is allowed during presentation. Existing copies
	// stay immutable; even a previously latched worker cannot consume/publish again.
	void Deactivate()
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		Active = false;
	}
	bool PublishCompleted(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (!Active) return false;
		const auto& Input = History[Frame % HistoryCapacity];
		if (!Input || Input->GetConsumptionFrame() != Frame) return false;
		if (Latest && Latest->Frame.GetConsumptionFrame() == Frame) return true;
		if (Serial == std::numeric_limits<std::uint64_t>::max()) return false;
		Latest = FPublishedInputFrame{*Input, ++Serial};
		return true;
	}
	std::optional<FPublishedInputFrame> ReadLatest() const
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		return Latest;
	}
	std::optional<FInputFrame> ReadRecorded(FFrameNumber Frame) const
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		const auto& Slot = History[Frame % HistoryCapacity];
		return Slot && Slot->GetConsumptionFrame() == Frame ? Slot : std::nullopt;
	}
private:
	const std::shared_ptr<IInputProducer> Source;
	mutable std::mutex Mutex;
	std::array<std::optional<FInputFrame>, HistoryCapacity> History{};
	std::optional<FPublishedInputFrame> Latest;
	std::uint64_t Serial = 0;
	bool Active = true;
};

/** GT-only named bindings. Registration order is dispatch order. Names are
 * nonempty and unique; invalid/duplicate bindings fail, no implicit rebind.
 * The callback observes the entire const frame and its bound slot. Only edges
 * in the latest polled frame are delivered: this is not a reliable event bus.
 */
class FInputPresentationBindings final
{
public:
	using FCallback = std::function<void(const FInputFrame&, FActionId)>;
	bool BindAction(std::string Name, FActionId Action, FCallback Callback)
	{
		if (FPresentationInputScope::IsActive() || Name.empty() || Action >= ActionCount
			|| !Callback || Bindings.size() == ActionCount) return false;
		for (const auto& Binding : Bindings) if (Binding.Name == Name) return false;
		Bindings.push_back({std::move(Name), Action, std::move(Callback)});
		return true;
	}
	void ResetObservation() { if (!FPresentationInputScope::IsActive()) LastSerial = 0; }
	void HandleInputs(const FPublishedInputFrame& Snapshot)
	{
		if (FPresentationInputScope::IsActive() || !Snapshot.Serial || Snapshot.Serial <= LastSerial
			|| !Snapshot.Frame.IsValid()) return;
		LastSerial = Snapshot.Serial;
		FPresentationInputScope ReadOnly;
		for (const auto& Binding : Bindings) Binding.Callback(Snapshot.Frame, Binding.Action);
	}
private:
	struct FBinding { std::string Name; FActionId Action; FCallback Callback; };
	std::vector<FBinding> Bindings;
	std::uint64_t LastSerial = 0;
};
}
