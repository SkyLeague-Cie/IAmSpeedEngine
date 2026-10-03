#pragma once
// Test-only protocol/trace helpers; no runtime creation or production wiring.
#include "KeyboardDiagnosticModel.h"
#include "IAmSpeed/Input/DeviceDiscovery.h"
#include <string>

namespace SelectedKeyboardDiagnostic
{
using namespace Speed::Input;
inline constexpr FActionId F9Slot = 3; // Diagnostic slot only, never a game mapping.
inline std::optional<FDeviceId> ParseId(const std::string& Hex)
{
	if (Hex.size() != 64) return std::nullopt;
	const auto Digit = [](char C) -> int {
		if (C >= '0' && C <= '9') return C - '0';
		if (C >= 'a' && C <= 'f') return C - 'a' + 10;
		if (C >= 'A' && C <= 'F') return C - 'A' + 10;
		return -1;
	};
	FDeviceId Id{};
	for (std::size_t I = 0; I < Id.size(); ++I) {
		const int High = Digit(Hex[I * 2]), Low = Digit(Hex[I * 2 + 1]);
		if (High < 0 || Low < 0) return std::nullopt;
		Id[I] = static_cast<std::uint8_t>(High * 16 + Low);
	}
	return Id;
}
inline std::string FormatId(const FDeviceId& Id)
{
	const char Hex[] = "0123456789abcdef"; std::string Out;
	for (const auto Byte : Id) { Out += Hex[Byte >> 4]; Out += Hex[Byte & 15]; }
	return Out;
}
inline bool Eligible(const std::vector<FDiscoveredDevice>& Inventory, const FDeviceId& Id)
{
	unsigned Matches = 0;
	for (const auto& D : Inventory) if (D.Id == Id) {
		if (!D.Connected || !(D.SupportedKinds & static_cast<std::uint8_t>(EDeviceKind::Keyboard))) return false;
		++Matches;
	}
	return Matches == 1;
}
// Source generation is intentionally optional: FInputFrame does not expose a
// session generation. Never relabel a host control epoch/OS timestamp as one.
inline std::string FrameJson(const FInputFrame& Frame, const FDeviceId& RequestedId,
	std::uint64_t ControlEpoch, std::optional<std::uint64_t> SourceGeneration = std::nullopt)
{
	std::string Edges = "[";
	for (std::size_t I = 0; I < Frame.GetEdgeCount(); ++I) {
		const auto& Edge = Frame.GetEdges()[I];
		if (I) Edges += ',';
		Edges += "{\"action\":" + std::to_string(Edge.Action)
			+ ",\"kind\":\"" + (Edge.Kind == EEdgeKind::Start ? "start" : "stop")
			+ "\",\"source_sequence\":" + std::to_string(Edge.SourceFrame) + "}";
	}
	Edges += ']';
	return "{\"type\":\"completed_frame\",\"requested_device_id\":\"" + FormatId(RequestedId)
		+ "\",\"consumption_frame\":" + std::to_string(Frame.GetConsumptionFrame())
		+ ",\"source_sequence\":" + std::to_string(Frame.GetSourceFrame())
		+ ",\"control_epoch\":" + std::to_string(ControlEpoch)
		+ ",\"source_generation\":" + (SourceGeneration ? std::to_string(*SourceGeneration) : "null")
		+ ",\"requires_reset\":" + (Frame.RequiresReset() ? "true" : "false")
		+ ",\"f9\":" + (Frame.GetActions()[F9Slot] ? "true" : "false")
		+ ",\"edge_count\":" + std::to_string(Frame.GetEdgeCount())
		+ ",\"edges\":" + Edges + "}";
}
// The future UI uses the exact accepted FCues/KeyboardCues (seven local cues,
// 250ms display-ack limit). No second cue clock or chat-triggered phase exists.
using FCues = KeyboardDiagnostic::FCues;
inline constexpr auto& Cues = KeyboardDiagnostic::KeyboardCues;
}
