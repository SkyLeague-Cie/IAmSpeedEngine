#pragma once

#include "DeviceDiscovery.h"
#include <cmath>

namespace Speed::Input
{
struct FActivityThresholds { float Enter, Exit, Delta; };
// No numeric defaults. These govern source activity, not gameplay axis mapping.
struct FActivityConfig
{
	FActivityThresholds Stick, Trigger;
	FFrameNumber MinimumResidenceFrames;
	std::size_t MaximumDevices;
	EDeviceKind HybridDeviceKind;
};
struct FActivityState
{
	std::array<bool, 256> Buttons{};
	std::array<float, 6> Axes{}; // LT, RT [0,1]; LX,LY,RX,RY [-1,1].
};
struct FActivityChoice { FDeviceId Id; EDeviceKind Kind; };

/** Acquisition-owner-only values policy; caller serializes lifecycle/Observe/
 * Decide. It neither produces frames nor writes a session. Raw reports use one
 * comparable backend timestamp domain. Decisions run at forward frame boundaries.
 */
class FDeviceActivityPolicy
{
public:
	explicit FDeviceActivityPolicy(FActivityConfig InConfig) : Config(InConfig), Failed(!ValidConfig(InConfig)) {}
	static bool ValidConfig(const FActivityConfig& C)
	{
		const auto Valid = [](FActivityThresholds T)
		{
			return std::isfinite(T.Enter) && std::isfinite(T.Exit) && std::isfinite(T.Delta)
				&& T.Exit >= 0 && T.Exit < T.Enter && T.Enter <= 1 && T.Delta > 0 && T.Delta <= 1;
		};
		return Valid(C.Stick) && Valid(C.Trigger) && C.MaximumDevices > 0
			&& C.MaximumDevices <= FDeviceDiscovery::Capacity
			&& (C.HybridDeviceKind == EDeviceKind::Keyboard || C.HybridDeviceKind == EDeviceKind::Gamepad);
	}
	bool Sync(const std::vector<FDiscoveredDevice>& Devices)
	{
		if (Failed) return false;
		std::map<FDeviceId, FTracker> Next;
		for (const auto& D : Devices)
		{
			if (!D.Connected || !(D.SupportedKinds & 3)) continue;
			if (Next.size() == Config.MaximumDevices || Next.count(D.Id)) { Failed = true; return false; }
			const auto Kind = (D.SupportedKinds & 3) == 3 ? Config.HybridDeviceKind
				: (D.SupportedKinds & 1) ? EDeviceKind::Keyboard : EDeviceKind::Gamepad;
			const auto Old = Trackers.find(D.Id);
			if (Old != Trackers.end() && Old->second.Device.Revision == D.Revision && Old->second.Kind == Kind)
				Next.emplace(D.Id, Old->second);
			else { FTracker T{}; T.Device = D; T.Kind = Kind; Next.emplace(D.Id, T); }
		}
		Trackers = std::move(Next); return true;
	}
	std::optional<EDeviceKind> Kind(const FDeviceId& Id) const
	{
		const auto It = Trackers.find(Id); return It == Trackers.end() ? std::nullopt : std::optional<EDeviceKind>(It->second.Kind);
	}
	void ResetDevice(const FDeviceId& Id)
	{
		const auto It = Trackers.find(Id);
		if (It != Trackers.end()) { auto& T = It->second; T.HasBaseline = false; T.Pending.reset(); T.Timestamp.reset(); }
	}
	void SetPaused(bool Value)
	{
		Paused = Value;
		for (auto& Item : Trackers) ResetDevice(Item.first);
	}
	void SetLock(std::optional<FDeviceId> Id) { Locked = Id; }
	bool Observe(const FDeviceId& Id, std::uint64_t Revision, std::uint64_t Timestamp, const FActivityState& State)
	{
		if (Failed || Paused) return false;
		const auto It = Trackers.find(Id);
		if (It == Trackers.end() || It->second.Device.Revision != Revision) return false;
		auto& T = It->second;
		for (std::size_t I = 0; I < State.Axes.size(); ++I)
			if (!std::isfinite(State.Axes[I]) || State.Axes[I] > 1 || State.Axes[I] < (I < 2 ? 0 : -1))
			{ ResetDevice(Id); return false; }
		if (T.Timestamp && Timestamp < *T.Timestamp) { ResetDevice(Id); return false; }
		bool Activity = false;
		if (T.HasBaseline)
			for (std::size_t I = 0; I < State.Buttons.size(); ++I)
				Activity = Activity || (State.Buttons[I] && !T.Previous.Buttons[I]); // Releases never reclaim ownership.
		for (std::size_t I = 0; I < 4; ++I)
		{
			const float X = I < 2 ? State.Axes[I] : State.Axes[2 + (I - 2) * 2];
			const float Y = I < 2 ? 0 : State.Axes[3 + (I - 2) * 2];
			const float Magnitude = std::sqrt(X * X + Y * Y);
			const auto Threshold = I < 2 ? Config.Trigger : Config.Stick;
			auto& A = T.Analog[I];
			if (!T.HasBaseline) { A = {Magnitude >= Threshold.Enter, X, Y}; continue; }
			if (Magnitude <= Threshold.Exit) { A = {false, X, Y}; continue; }
			if (Magnitude < Threshold.Enter) continue; // Hysteresis band never claims activity.
			const auto DX = X - A.X, DY = Y - A.Y;
			if (!A.Active || DX * DX + DY * DY >= Threshold.Delta * Threshold.Delta)
			{
				Activity = true; A = {true, X, Y};
			}
		}
		T.Previous = State; T.HasBaseline = true; T.Timestamp = Timestamp;
		if (Activity) T.Pending = Timestamp;
		return true;
	}
	// nullopt means neutral; all pending activity is consumed even when residence
	// or lock suppresses it. Stale suppressed activity cannot trigger a later switch.
	std::optional<FActivityChoice> Decide(FFrameNumber Frame)
	{
		if (Failed) return std::nullopt;
		if (LastFrame && Frame == *LastFrame) return LastChoice;
		if (LastFrame && (Frame < *LastFrame || *LastFrame == std::numeric_limits<FFrameNumber>::max() || Frame != *LastFrame + 1))
		{ Failed = true; return std::nullopt; }
		std::optional<FDeviceId> Winner;
		if (!Paused)
		{
			if (Locked) { if (Trackers.count(*Locked)) Winner = Locked; }
			else
			{
				if (Remembered && Trackers.count(*Remembered)) Winner = Remembered;
				std::optional<std::uint64_t> Best = Winner ? LastClaimTimestamp : std::nullopt;
				for (const auto& Item : Trackers)
				{
					const auto Stamp = Item.second.Pending;
					if (!Stamp) continue;
					if (!Best || *Stamp > *Best || (*Stamp == *Best && Remembered && Item.first == *Remembered))
					{ Best = Stamp; Winner = Item.first; }
				}
				if (Winner && Remembered && *Winner != *Remembered && Trackers.count(*Remembered)
					&& LastSwitch && Frame - *LastSwitch < Config.MinimumResidenceFrames) Winner = Remembered;
			}
		}
		if (Winner && (!Remembered || *Winner != *Remembered)) LastSwitch = Frame;
		if (Winner && Trackers.at(*Winner).Pending
			&& (!LastClaimTimestamp || *Trackers.at(*Winner).Pending > *LastClaimTimestamp)) LastClaimTimestamp = Trackers.at(*Winner).Pending;
		if (Winner) Remembered = Winner;
		for (auto& Item : Trackers) Item.second.Pending.reset();
		LastChoice.reset();
		if (Winner) LastChoice = FActivityChoice{*Winner, Trackers.at(*Winner).Kind};
		LastFrame = Frame; return LastChoice;
	}
	bool IsFailed() const { return Failed; }
private:
	struct FAnalog { bool Active = false; float X = 0, Y = 0; };
	struct FTracker
	{
		FDiscoveredDevice Device{};
		EDeviceKind Kind = EDeviceKind::Keyboard;
		FActivityState Previous{};
		std::array<FAnalog, 4> Analog{};
		std::optional<std::uint64_t> Timestamp, Pending;
		bool HasBaseline = false;
	};
	const FActivityConfig Config;
	std::map<FDeviceId, FTracker> Trackers;
	std::optional<FDeviceId> Remembered, Locked;
	std::optional<FFrameNumber> LastFrame, LastSwitch;
	std::optional<std::uint64_t> LastClaimTimestamp;
	std::optional<FActivityChoice> LastChoice;
	bool Failed = false, Paused = false;
};
}
