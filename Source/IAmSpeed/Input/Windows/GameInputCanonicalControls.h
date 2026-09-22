#pragma once

#include "GameInputReadings.h"
#include "../RawInput.h"

namespace Speed::Input::Windows
{
// Windows set-1 make code -> USB keyboard page 0x07 usage. This translates
// positions, not glyphs or Windows virtual-key numbers. The game mapping/UI
// must translate saved logical key names separately under its selected layout.
// Table reference: learn.microsoft.com/windows/win32/inputdev/about-keyboard-input#scan-codes
inline std::uint16_t KeyboardUsage(std::uint32_t Scan) noexcept
{
	static constexpr std::uint16_t Base[] = {
		0,0x29,0x1e,0x1f,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x2d,0x2e,0x2a,0x2b,
		0x14,0x1a,0x08,0x15,0x17,0x1c,0x18,0x0c,0x12,0x13,0x2f,0x30,0x28,0xe0,0x04,0x16,
		0x07,0x09,0x0a,0x0b,0x0d,0x0e,0x0f,0x33,0x34,0x35,0xe1,0x31,0x1d,0x1b,0x06,0x19,
		0x05,0x11,0x10,0x36,0x37,0x38,0xe5,0x55,0xe2,0x2c,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,
		0x3f,0x40,0x41,0x42,0x43,0x48,0x47,0x5f,0x60,0x61,0x56,0x5c,0x5d,0x5e,0x57,0x59,
		0x5a,0x5b,0x62,0x63,0x46,0,0x64,0x44,0x45
	};
	if (Scan < sizeof(Base) / sizeof(Base[0])) return Base[Scan];
	switch (Scan)
	{
	case 0xe01c: return 0x58; case 0xe01d: return 0xe4;
	case 0xe035: return 0x54; case 0xe036: return 0xe5;
	case 0xe037: return 0x46; case 0xe038: return 0xe6;
	case 0xe045: return 0x53; case 0xe11d45: case 0xe046: return 0x48;
	case 0xe047: return 0x4a; case 0xe048: return 0x52;
	case 0xe049: return 0x4b; case 0xe04b: return 0x50;
	case 0xe04d: return 0x4f; case 0xe04f: return 0x4d;
	case 0xe050: return 0x51; case 0xe051: return 0x4e;
	case 0xe052: return 0x49; case 0xe053: return 0x4c;
	case 0xe05b: return 0xe3; case 0xe05c: return 0xe7; case 0xe05d: return 0x65;
	default: return 0; // Unsupported positions never alias a supported key.
	}
}

using FCanonicalDeviceState = V2::FCanonicalRawState;

// Complete, sorted capability/state snapshot. Unknown keyboard scan codes are
// excluded, not guessed from glyphs. Standard pad controls use SDK semantic
// bits (A/B/X/Y) rather than manufacturer-specific button indices.
inline bool Canonicalize(const FDeviceState& Raw, V2::ERawDeviceKind Kind,
	FCanonicalDeviceState& Output) noexcept
{
	using namespace V2;
	using namespace GameInput::v3;
	FCanonicalDeviceState Prepared;
	if (Kind == ERawDeviceKind::Keyboard)
	{
		if (Raw.KeyCount > Raw.ScanCodes.size()) { Output = {}; return false; }
		std::array<bool, 256> Supported{}, Held{};
		for (std::uint32_t Scan = 1; Scan <= 0x58; ++Scan) Supported[KeyboardUsage(Scan)] = true;
		for (std::uint32_t Scan = 0xe000; Scan <= 0xe05d; ++Scan) Supported[KeyboardUsage(Scan)] = true;
		Supported[0x48] = true; Supported[0] = false;
		for (std::size_t I = 0; I < Raw.KeyCount; ++I) Held[KeyboardUsage(Raw.ScanCodes[I])] = true;
		for (std::uint16_t Usage = 1; Usage < 256; ++Usage)
			if (Supported[Usage]) Prepared.Values[Prepared.Count++] = {{ERawControlKind::KeyboardUsage, Usage}, Held[Usage] ? 1.0f : 0.0f};
	}
	else if (Kind == ERawDeviceKind::Gamepad)
	{
		static constexpr GameInputGamepadButtons Buttons[] = {
			GameInputGamepadA, GameInputGamepadB, GameInputGamepadX, GameInputGamepadY,
			GameInputGamepadDPadUp, GameInputGamepadDPadDown, GameInputGamepadDPadLeft, GameInputGamepadDPadRight,
			GameInputGamepadLeftShoulder, GameInputGamepadRightShoulder,
			GameInputGamepadLeftThumbstick, GameInputGamepadRightThumbstick, GameInputGamepadMenu, GameInputGamepadView};
		static_assert(sizeof(Buttons) / sizeof(Buttons[0]) == static_cast<std::size_t>(EPadButton::Count));
		for (std::uint16_t I = 0; I < static_cast<std::uint16_t>(EPadButton::Count); ++I)
			Prepared.Values[Prepared.Count++] = {{ERawControlKind::PadButton, I}, (Raw.GamepadButtons & Buttons[I]) ? 1.0f : 0.0f};
		static constexpr std::size_t Axes[] = {2, 3, 4, 5, 0, 1};
		for (std::uint16_t I = 0; I < static_cast<std::uint16_t>(EPadAxis::Count); ++I)
		{
			const FRawControl Control{ERawControlKind::PadAxis, I};
			const float Value = Raw.Axes[Axes[I]];
			if (!Control.Accepts(Value)) { Output = {}; return false; }
			Prepared.Values[Prepared.Count++] = {Control, Value};
		}
	}
	else { Output = {}; return false; }
	Output = Prepared; return true;
}
}
