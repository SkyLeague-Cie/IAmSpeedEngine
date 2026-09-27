#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <optional>

namespace Speed::Input
{
using FFrameNumber = std::uint64_t;
using FActionId = std::uint8_t;
constexpr std::size_t ActionCount = 32;
constexpr std::size_t MaxEdges = 64;
constexpr std::size_t HistoryCapacity = 256;

// The first three slots preserve the existing wheeled quantization. Games own
// slots [3, 31] and their scales; IAmSpeed does not know Jump or Powerslide.
constexpr FActionId Throttle = 0; // [0, 255]
constexpr FActionId Brake = 1; // [0, 255]
constexpr FActionId Steering = 2; // [-127, 127]
using FActionValues = std::array<std::int16_t, ActionCount>;

// The only legacy frame conversion. Reject the pre-initialization sentinel;
// callers never wrap it into a huge canonical frame.
inline std::optional<FFrameNumber> FromLegacyLocalFrame(std::uint32_t Frame)
{
	if (!Frame) return std::nullopt;
	return static_cast<FFrameNumber>(Frame) - 1;
}

// Matches the existing FMath::RoundToInt(f * scale) ties toward +infinity.
// Invalid device data must not enter a canonical integer snapshot.
inline std::optional<std::int16_t> QuantizeAxis(float Value, bool bSigned)
{
	if (!std::isfinite(Value)) return std::nullopt;
	const float Minimum = bSigned ? -1.0f : 0.0f;
	const float Clamped = Value < Minimum ? Minimum : (Value > 1.0f ? 1.0f : Value);
	return static_cast<std::int16_t>(std::floor(Clamped * (bSigned ? 127.0f : 255.0f) + 0.5f));
}

enum class EProducerKind : std::uint8_t { Device, AI, Network };
struct FProducerIdentity
{
	EProducerKind Kind = EProducerKind::Device;
	std::uint64_t Id = 0; // Nonzero, scoped to the target's input stream/run.
};

enum class EEdgeKind : std::uint8_t { Start, Stop };
struct FActionEdge
{
	FActionId Action = 0;
	EEdgeKind Kind = EEdgeKind::Start;
	FFrameNumber SourceFrame = 0;
};

/** Values-only immutable snapshot. No UObject, clock query, mutable accessor,
 * or host pointer crosses this boundary. Source frames are producer-local;
 * ConsumptionFrame is the zero-based canonical physical frame, never SimTime.
 */
class FInputFrame final
{
public:
	FInputFrame(FFrameNumber Source, FFrameNumber Consumption,
		FProducerIdentity Producer, const FActionValues& Values,
		const std::array<FActionEdge, MaxEdges>& Edges = {}, std::size_t EdgeCount = 0,
		bool Reset = false)
		: SourceFrame(Source), ConsumptionFrame(Consumption), Identity(Producer),
		  Actions(Values), Events(Edges), EventCount(EdgeCount), ResetBeforeActions(Reset) {}

	FFrameNumber GetSourceFrame() const { return SourceFrame; }
	FFrameNumber GetConsumptionFrame() const { return ConsumptionFrame; }
	FProducerIdentity GetProducer() const { return Identity; }
	const FActionValues& GetActions() const { return Actions; }
	const std::array<FActionEdge, MaxEdges>& GetEdges() const { return Events; }
	std::size_t GetEdgeCount() const { return EventCount; }
	// Cancel prior held action state before applying this frame. This is not a
	// user release event (which may have gameplay meaning of its own).
	bool RequiresReset() const { return ResetBeforeActions; }
	bool IsValid() const
	{
		if (!Identity.Id || Identity.Kind > EProducerKind::Network || EventCount > MaxEdges
			|| Actions[Throttle] < 0 || Actions[Throttle] > 255
			|| Actions[Brake] < 0 || Actions[Brake] > 255
			|| Actions[Steering] < -127 || Actions[Steering] > 127) return false;
		for (std::size_t Index = 0; Index < EventCount; ++Index)
		{
			const auto& Edge = Events[Index];
			if (Edge.Action >= ActionCount || Edge.Kind > EEdgeKind::Stop
				|| Edge.SourceFrame > SourceFrame
				|| (Index && Edge.SourceFrame < Events[Index - 1].SourceFrame)) return false;
		}
		for (std::size_t Index = EventCount; Index < MaxEdges; ++Index)
		{
			const auto& Edge = Events[Index];
			if (Edge.Action != 0 || Edge.Kind != EEdgeKind::Start || Edge.SourceFrame != 0) return false;
		}
		return true;
	}

private:
	FFrameNumber SourceFrame;
	FFrameNumber ConsumptionFrame;
	FProducerIdentity Identity;
	FActionValues Actions;
	std::array<FActionEdge, MaxEdges> Events;
	std::size_t EventCount;
	bool ResetBeforeActions;
};
}
