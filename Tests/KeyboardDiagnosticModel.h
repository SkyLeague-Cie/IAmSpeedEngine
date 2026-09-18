#pragma once
// Test harness only. No production headers or platform dependencies.
#include <array>
#include <cstdint>
#include <optional>

namespace KeyboardDiagnostic
{
enum class EObservation { Initial, Unchanged, Updated, Invalid, Stopped };
class FReadings
{
public:
	EObservation Observe(std::uint64_t Stamp, bool F9, bool SameObject)
	{
		if (Stopped) return EObservation::Stopped;
		if (Failed) return EObservation::Invalid;
		if (Last && (Stamp < *Last || (SameObject && (Stamp != *Last || F9 != Held))))
		{ Failed = true; return EObservation::Invalid; }
		if (Last && SameObject) { ++Duplicates; return EObservation::Unchanged; }
		const bool Initial = !Last;
		if (!Initial && F9 != Held) ++Transitions;
		Last = Stamp; Held = F9; ++Samples;
		return Initial ? EObservation::Initial : EObservation::Updated;
	}
	void Focus(bool Value) { if (Focused != Value) { Focused = Value; ++FocusChanges; } }
	void Stop() { Stopped = true; }
	bool IsFailed() const { return Failed; }
	bool IsHeld() const { return Held; }
	unsigned Samples = 0, Duplicates = 0, Transitions = 0, FocusChanges = 0;
private:
	std::optional<std::uint64_t> Last;
	bool Held = false, Focused = false, Failed = false, Stopped = false;
};

struct FCue { std::uint64_t AtUs; const wchar_t* Text; bool ExpectedF9; };
inline constexpr std::array<FCue, 7> KeyboardCues{{
	{0, L"Repos : relachez F9", false},
	{5000000, L"Maintenez F9", true},
	{10000000, L"Relachez F9", false},
	{15000000, L"Maintenez F9", true},
	{20000000, L"Relachez F9", false},
	{25000000, L"Repos : ne touchez plus aux commandes", false},
	{30000000, L"Termine", false}
}};

// One outstanding visual cue. Missed/late cues invalidate the protocol instead
// of fast-forwarding instructions that the participant never saw.
class FCues
{
public:
	static constexpr std::uint64_t MaxLagUs = 250000; // Harness validity, not a product input tolerance.
	std::optional<unsigned> Advance(std::uint64_t Now)
	{
		if (Failed) return {};
		if (LastNow && Now < *LastNow) { Failed = true; return {}; }
		LastNow = Now;
		if (Pending)
		{
			if (Now - KeyboardCues[*Pending].AtUs > MaxLagUs) Failed = true;
			return {};
		}
		if (Next == KeyboardCues.size() || Now < KeyboardCues[Next].AtUs) return {};
		if (Now - KeyboardCues[Next].AtUs > MaxLagUs) { Failed = true; return {}; }
		Pending = Next++; return Pending;
	}
	bool Acknowledge(unsigned Index, std::uint64_t DisplayedAt)
	{
		if (Failed || !Pending || *Pending != Index || DisplayedAt < KeyboardCues[Index].AtUs
			|| DisplayedAt - KeyboardCues[Index].AtUs > MaxLagUs)
		{ Failed = true; return false; }
		Active = Index; Pending.reset(); return true;
	}
	void Observe(bool F9, bool Focused)
	{
		// This first diagnostic qualifies foreground phases only, even when the
		// explicitly selected API policy also allows background input.
		if (!Active || Failed) return;
		if (!Focused) { Failed = true; return; }
		if (F9 == KeyboardCues[*Active].ExpectedF9) Covered[*Active] = true;
	}
	bool IsFailed() const { return Failed; }
	bool Complete() const
	{
		return !Failed && !Pending && Next == KeyboardCues.size()
			&& Covered[0] && Covered[1] && Covered[2] && Covered[3] && Covered[4] && Covered[5];
	}
private:
	std::optional<std::uint64_t> LastNow;
	std::optional<unsigned> Pending, Active;
	std::array<bool, KeyboardCues.size()> Covered{};
	unsigned Next = 0;
	bool Failed = false;
};

struct FCleanup
{
	bool WorkerStopped = false, ReadingReleased = false, ApiReleased = false, WindowDestroyed = false;
	bool Complete() const { return WorkerStopped && ReadingReleased && ApiReleased && WindowDestroyed; }
};
}
