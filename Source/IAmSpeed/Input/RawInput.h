#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace Speed::Input::V2
{
// These clocks are deliberately different types. None is a render frame.
struct FStreamEpoch { std::uint64_t Value = 0; };
struct FMappingRevision { std::uint64_t Value = 0; };
struct FDeviceGeneration { std::uint64_t Value = 0; };
enum class ERawDeviceKind : std::uint8_t { Keyboard, Gamepad };
enum class ERawControlKind : std::uint8_t { KeyboardUsage, PadButton, PadAxis };
enum class EPadButton : std::uint16_t
{
	South, East, West, North, DPadUp, DPadDown, DPadLeft, DPadRight,
	LeftShoulder, RightShoulder, LeftStick, RightStick, Menu, View, Count
};
enum class EPadAxis : std::uint16_t { LeftX, LeftY, RightX, RightY, LeftTrigger, RightTrigger, Count };
struct FRawControl
{
	ERawControlKind Kind = ERawControlKind::KeyboardUsage;
	std::uint16_t Code = 0;
	bool IsValid() const
	{
		switch (Kind)
		{
		case ERawControlKind::KeyboardUsage:
			// HID usage page 0x07; reserved/error/vendor usages are unsupported.
			return (Code >= 4 && Code <= 0xA4) || (Code >= 0xE0 && Code <= 0xE7);
		case ERawControlKind::PadButton: return Code < static_cast<std::uint16_t>(EPadButton::Count);
		case ERawControlKind::PadAxis: return Code < static_cast<std::uint16_t>(EPadAxis::Count);
		default: return false;
		}
	}
	bool Accepts(float Value) const
	{
		if (!IsValid() || !std::isfinite(Value)) return false;
		if (Kind != ERawControlKind::PadAxis) return Value == 0.0f || Value == 1.0f;
		return Value <= 1.0f && Value >= (Code < 4 ? -1.0f : 0.0f);
	}
};
inline bool operator==(FRawControl A, FRawControl B) { return A.Kind == B.Kind && A.Code == B.Code; }
inline bool operator<(FRawControl A, FRawControl B)
{ return A.Kind < B.Kind || (A.Kind == B.Kind && A.Code < B.Code); }
struct FSourceOrder
{
	std::uint64_t Sequence = 0;
	std::uint32_t WithinSequence = 0;
};
inline bool operator<(FSourceOrder A, FSourceOrder B)
{ return A.Sequence < B.Sequence || (A.Sequence == B.Sequence && A.WithinSequence < B.WithinSequence); }
struct FRawValue { FRawControl Control; float Value = 0; };
struct FRawChange { FRawValue State; FSourceOrder Order; };
enum class ERawSampleStatus : std::uint8_t { Valid, Resync, Overflow, Unsupported };

// An acquisition DTO, copied/validated by the future mapper. FinalState lists
// EVERY supported control (including neutral values), in canonical order; this
// is also the capability list. No cross-device merging or OS codes enter here.
struct FRawInputSample
{
	static constexpr std::size_t MaxControls = 256;
	static constexpr std::size_t MaxChanges = 256;
	std::uint64_t DeviceId = 0;
	FDeviceGeneration Generation;
	ERawDeviceKind Kind = ERawDeviceKind::Keyboard;
	std::uint64_t Sequence = 0;
	ERawSampleStatus Status = ERawSampleStatus::Unsupported;
	std::vector<FRawValue> FinalState;
	std::vector<FRawChange> Changes;
	bool IsValid() const
	{
		if (!DeviceId || !Generation.Value || !Sequence || Kind > ERawDeviceKind::Gamepad
			|| Status > ERawSampleStatus::Resync || FinalState.empty()
			|| FinalState.size() > MaxControls || Changes.size() > MaxChanges
			|| (Status == ERawSampleStatus::Resync && !Changes.empty())) return false;
		for (std::size_t I = 0; I < FinalState.size(); ++I)
		{
			const auto& S = FinalState[I];
			if (!S.Control.Accepts(S.Value) || (I && !(FinalState[I - 1].Control < S.Control))
				|| ((Kind == ERawDeviceKind::Keyboard) != (S.Control.Kind == ERawControlKind::KeyboardUsage))) return false;
		}
		for (std::size_t I = 0; I < Changes.size(); ++I)
		{
			const auto& C = Changes[I];
			if (!C.State.Control.Accepts(C.State.Value) || !C.Order.Sequence || C.Order.Sequence > Sequence
				|| (I && !(Changes[I - 1].Order < C.Order))) return false;
			const FRawValue* Final = nullptr;
			for (const auto& S : FinalState) if (S.Control == C.State.Control) Final = &S;
			if (!Final) return false;
			bool Last = true;
			for (std::size_t J = I + 1; J < Changes.size(); ++J)
				if (Changes[J].State.Control == C.State.Control) Last = false;
			if (Last && Final->Value != C.State.Value) return false;
		}
		return true;
	}
};
}
