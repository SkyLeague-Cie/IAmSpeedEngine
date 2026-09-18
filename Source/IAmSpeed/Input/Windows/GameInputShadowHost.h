#pragma once

// Optional compile-only Windows leaf. No portable/UE module includes this file.
#if defined(_WIN32)
#include "GameInputSelectedSource.h"
#include "../InputStream.h"
#include <thread>

namespace Speed::Input::Windows
{
struct FGameInputShadowConfig
{
	bool Enabled = false;
	std::uint64_t ProducerId = 0;
	std::array<bool, ActionCount> Digital{};
	FGameInputMapper Keyboard, Gamepad;
	std::optional<FActivityConfig> Activity; // No automatic numeric defaults.
};

/** Diagnostic-only owner, not an IInputProducer and never a vehicle input source.
 * The caller supplies a backend factory; this leaf never creates a real runtime.
 * One explicitly bound thread owns polling/completion. Observers receive copies,
 * never the producer, stream, SDK interface or cursor. Mappers must not reenter.
 */
class FGameInputShadowHost final
{
	using FApi = GameInput::v3::IGameInput;
public:
	using FBackendFactory = std::function<Microsoft::WRL::ComPtr<FApi>()>;
	static std::unique_ptr<FGameInputShadowHost> Create(const FGameInputShadowConfig& Config,
		const FBackendFactory& Backend)
	{
		if (!Config.Enabled || FPresentationInputScope::IsActive() || !Config.ProducerId
			|| !Config.Keyboard || !Config.Gamepad || !Backend
			|| (Config.Activity && !FDeviceActivityPolicy::ValidConfig(*Config.Activity))) return nullptr;
		auto Api = Backend();
		if (!Api) return nullptr;
		auto Source = Config.Activity
			? FGameInputSelectedSource::CreateAutomatic(Api.Get(), Config.ProducerId, Config.Digital,
				Config.Keyboard, Config.Gamepad, *Config.Activity)
			: FGameInputSelectedSource::Create(Api.Get(), Config.ProducerId, Config.Digital,
				Config.Keyboard, Config.Gamepad);
		if (!Source) return nullptr;
		return std::unique_ptr<FGameInputShadowHost>(new FGameInputShadowHost(std::move(Source)));
	}
	~FGameInputShadowHost() { if (!Shutdown()) std::terminate(); }
	bool BindOwnerThread()
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping) return false;
		const auto Caller = std::this_thread::get_id();
		if (Owner && *Owner != Caller) return false;
		Owner = Caller; return true;
	}
	std::optional<FInputFrame> BeginFrame(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Gate);
		if (!IsOwner() || Stopping || (Pending && *Pending != Frame)) return std::nullopt;
		if (Pending) return Stream.ReadRecorded(Frame); // Same in-flight transaction, no repoll.
		if (LastConsumed && Frame <= *LastConsumed) return std::nullopt;
		auto Input = Stream.Consume(Frame);
		if (Input) { Pending = Frame; LastConsumed = Frame; }
		return Input;
	}
	// Call only after the corresponding physical frame completed successfully.
	bool CompleteFrame(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (!IsOwner() || Stopping || !Pending || *Pending != Frame) return false;
		if (!Stream.PublishCompleted(Frame)) return false;
		Pending.reset(); return true;
	}
	bool AbortFrame(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		if (!IsOwner() || Stopping || !Pending || *Pending != Frame) return false;
		Pending.reset(); return true; // No publication of incomplete physics.
	}
	std::optional<FPublishedInputFrame> ReadLatest() const { return Stream.ReadLatest(); }
	std::optional<FInputFrame> ReadRecorded(FFrameNumber Frame) const { return Stream.ReadRecorded(Frame); }
	bool RequestSelection(std::optional<std::pair<FDeviceId, EDeviceKind>> Choice)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		return !Stopping && Source->RequestSelection(Choice);
	}
	bool RequestLock(std::optional<FDeviceId> Id)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Gate);
		return !Stopping && Source->RequestLock(Id);
	}
	// Synchronous control acknowledgment independent of the next physical frame.
	// The epoch labels this host's controls; device generation remains in Source.
	std::optional<std::uint64_t> SetPaused(bool Paused)
	{
		if (FPresentationInputScope::IsActive()) return std::nullopt;
		std::lock_guard<std::mutex> Lock(Gate);
		if (Stopping || ControlEpoch == std::numeric_limits<std::uint64_t>::max()
			|| !Source->SetPaused(Paused)) return std::nullopt;
		Pending.reset(); // A pre-control pending frame cannot publish after this ack.
		return ++ControlEpoch;
	}
	bool Shutdown()
	{
		std::lock_guard<std::mutex> Lock(Gate);
		Stopping = true; Pending.reset(); Stream.Deactivate();
		// Failed unregister retains Source/Stream and all callback-owned state.
		return Source->Shutdown();
	}
private:
	explicit FGameInputShadowHost(std::unique_ptr<FGameInputSelectedSource> InSource)
		: Source(std::move(InSource)), Stream(Source) {}
	bool IsOwner() const { return Owner && *Owner == std::this_thread::get_id(); }
	const std::shared_ptr<FGameInputSelectedSource> Source;
	FInputStream Stream;
	mutable std::mutex Gate;
	std::optional<std::thread::id> Owner;
	std::optional<FFrameNumber> Pending, LastConsumed;
	std::uint64_t ControlEpoch = 0;
	bool Stopping = false;
};
}
#endif
