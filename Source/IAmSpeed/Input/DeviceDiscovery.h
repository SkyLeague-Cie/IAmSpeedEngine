#pragma once

#include "DeviceInputSession.h"
#include <map>
#include <vector>

namespace Speed::Input
{
// Backend-scoped opaque bytes, never a pointer, player ID or producer ID.
// The host must keep different backend catalogues in separate instances.
using FDeviceId = std::array<std::uint8_t, 32>;
enum class EDeviceKind : std::uint8_t { Keyboard = 1, Gamepad = 2 };
struct FDiscoveredDevice
{
	FDeviceId Id{};
	std::uint64_t Revision = 0; // Strictly ordered lifecycle version for this ID.
	std::uint8_t SupportedKinds = 0; // Zero: visible, but not selectable here.
	bool Connected = false;
};
struct FDeviceSelection
{
	FDiscoveredDevice Device;
	EDeviceKind Kind = EDeviceKind::Keyboard;
	FDeviceInputSession::FGeneration Generation = 0;
};
enum class EDiscoveryUpdate { Applied, Duplicate, Stale, Failed };

/** Values-only discovery and explicit selection. All calls are serialized.
 * Tombstones preserve per-ID ordering across removal; capacity exhaustion is
 * fail-closed and requires reconstruction. No callback-order device priority.
 */
class FDeviceDiscovery final : public IInputProducer
{
public:
	static constexpr std::size_t Capacity = 256;
	FDeviceDiscovery(std::uint64_t ProducerId, const std::array<bool, ActionCount>& Digital)
		: Session(ProducerId, Digital) {}

	EDiscoveryUpdate Update(const FDiscoveredDevice& Device)
	{
		if (FPresentationInputScope::IsActive()) return EDiscoveryUpdate::Failed;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (Failed) return EDiscoveryUpdate::Failed;
		const auto It = Devices.find(Device.Id);
		if (It != Devices.end())
		{
			const auto& Old = It->second;
			if (Device.Revision < Old.Revision) return EDiscoveryUpdate::Stale;
			if (Device.Revision == Old.Revision)
			{
				if (Device.Connected == Old.Connected && Device.SupportedKinds == Old.SupportedKinds)
					return EDiscoveryUpdate::Duplicate;
				FailLocked(); return EDiscoveryUpdate::Failed; // Ambiguous ordering.
			}
		}
		else if (Devices.size() == Capacity) { FailLocked(); return EDiscoveryUpdate::Failed; }
		Devices[Device.Id] = Device;
		if (Requested && Requested->first == Device.Id && !RefreshLocked()) return EDiscoveryUpdate::Failed;
		return EDiscoveryUpdate::Applied;
	}
	// Exact request persists across removal. Reconnect of that ID resumes with a
	// new generation; a different ID never steals selection. nullopt deselects.
	bool Select(std::optional<std::pair<FDeviceId, EDeviceKind>> Request)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (Failed) return false;
		if (Request && Request->second != EDeviceKind::Keyboard && Request->second != EDeviceKind::Gamepad) return false;
		Requested = Request;
		return RefreshLocked();
	}
	std::vector<FDiscoveredDevice> Snapshot() const
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		std::vector<FDiscoveredDevice> Result;
		for (const auto& Item : Devices) Result.push_back(Item.second);
		return Result; // Includes tombstones, sorted lexicographically by ID.
	}
	std::optional<FDeviceSelection> Selected() const
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		return Selection;
	}
	bool Submit(const FDeviceSelection& Ticket, std::uint64_t Sequence, const FActionValues& Values)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (Failed || !Selection || Ticket.Generation != Selection->Generation
			|| Ticket.Device.Id != Selection->Device.Id || Ticket.Device.Revision != Selection->Device.Revision
			|| Ticket.Kind != Selection->Kind) return false;
		return Session.Submit(Ticket.Generation, Sequence, Values);
	}
	bool SetPaused(bool Paused)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (Failed) return false;
		const auto Generation = Session.SetPaused(Paused);
		if (!Generation) { FailLocked(); return false; }
		if (Selection) Selection->Generation = *Generation;
		return true;
	}
	// An obsolete poll must not reset a replacement's newer generation.
	bool Resynchronize(const FDeviceSelection& Ticket)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::mutex> Lock(Mutex);
		if (Failed || !Selection || Ticket.Generation != Selection->Generation
			|| Ticket.Device.Id != Selection->Device.Id || Ticket.Device.Revision != Selection->Device.Revision
			|| Ticket.Kind != Selection->Kind) return false;
		const auto Generation = Session.Resynchronize();
		if (!Generation) { FailLocked(); return false; }
		Selection->Generation = *Generation;
		return true;
	}
	void Fail()
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		FailLocked();
	}
	bool IsFailed() const { std::lock_guard<std::mutex> Lock(Mutex); return Failed; }
	std::optional<FInputFrame> Produce(FFrameNumber Frame) override
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		return ResetFailed ? std::nullopt : Session.Produce(Frame);
	}
	bool Skip(FFrameNumber Frame) override
	{
		std::lock_guard<std::mutex> Lock(Mutex);
		return !ResetFailed && Session.Skip(Frame);
	}
private:
	void FailLocked()
	{
		if (Failed) return;
		Failed = true; Selection.reset();
		ResetFailed = !Session.SetConnected(false);
	}
	bool RefreshLocked()
	{
		std::optional<FDiscoveredDevice> Candidate;
		if (Requested)
		{
			const auto It = Devices.find(Requested->first);
			if (It != Devices.end() && It->second.Connected
				&& (It->second.SupportedKinds & static_cast<std::uint8_t>(Requested->second))) Candidate = It->second;
		}
		if (!Candidate && !Selection) return true;
		if (Candidate && Selection && Candidate->Id == Selection->Device.Id
			&& Candidate->Revision == Selection->Device.Revision && Requested->second == Selection->Kind) return true;
		const auto Generation = Session.SetConnected(Candidate.has_value());
		if (!Generation) { FailLocked(); return false; }
		Selection.reset();
		if (Candidate) Selection = FDeviceSelection{*Candidate, Requested->second, *Generation};
		return true;
	}
	mutable std::mutex Mutex;
	FDeviceInputSession Session;
	std::map<FDeviceId, FDiscoveredDevice> Devices;
	std::optional<std::pair<FDeviceId, EDeviceKind>> Requested;
	std::optional<FDeviceSelection> Selection;
	bool Failed = false;
	bool ResetFailed = false;
};
}
