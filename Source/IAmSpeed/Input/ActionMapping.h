#pragma once

#include "InputFrameV2.h"
#include "InputPresentationScope.h"

namespace Speed::Input::V2
{
enum class EMappingStatus : std::uint8_t
{
	Mapped, InvalidConfiguration, ResyncRequired, InvalidRaw, DeviceChanged,
	SequenceGap, StaleBaseline, UnexplainedState, UnsupportedControl,
	Overflow, InvalidValue, WrongFrame, PresentationForbidden
};
struct FMappingResult
{
	EMappingStatus Status = EMappingStatus::InvalidConfiguration;
	std::optional<FInputFrame> Frame;
};

// Single physical-lane owner. A failed sample never commits a partial state;
// recovery requires an explicit fresh Resync baseline, not a legacy fallback.
class FActionMapper final
{
public:
	// A structurally valid A1 contract may describe a response this mapper
	// deliberately does not implement. Do not silently approximate its curve.
	static bool SupportsContract(const FInputActionContract& Candidate)
	{
		for (const auto& A : Candidate.GetDescription().Actions)
			if (A.Exponent != 1.0f && A.Exponent != 2.0f) return false;
		return true;
	}
	static std::unique_ptr<FActionMapper> Create(std::shared_ptr<const FInputActionContract> Candidate,
		FStreamEpoch InEpoch, FProducerIdentity InProducer, FFrameNumber FirstFrame = 0)
	{
		if (!Candidate || !SupportsContract(*Candidate) || !InEpoch.Value || !InProducer.Id
			|| InProducer.Kind != EProducerKind::Device) return {};
		return std::unique_ptr<FActionMapper>(new FActionMapper(std::move(Candidate), InEpoch, InProducer, FirstFrame));
	}
	FActionMapper(std::shared_ptr<const FInputActionContract> InContract,
		FStreamEpoch InEpoch, FProducerIdentity InProducer, FFrameNumber FirstFrame = 0)
		: Contract(std::move(InContract)), Epoch(InEpoch), Producer(InProducer), NextFrame(FirstFrame) {}
	const std::shared_ptr<const FInputActionContract>& GetContract() const { return Contract; }
	// Sole owner, at an acknowledged boundary. Preserve the next physical
	// address; the raw source must supply a genuinely fresh Resync baseline.
	void RequireFreshBaseline() { NeedsResync = true; }
	FMappingResult Map(const FRawInputSample& Sample, FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return {EMappingStatus::PresentationForbidden, {}};
		if (!Contract || !SupportsContract(*Contract) || !Epoch.Value || !Producer.Id || Producer.Kind != EProducerKind::Device)
			return {EMappingStatus::InvalidConfiguration, {}}; // No state/frame mutation.
		if (Exhausted || Frame != NextFrame) return Reject(EMappingStatus::WrongFrame);
		if (Sample.Status == ERawSampleStatus::Overflow || Sample.Changes.size() > FRawInputSample::MaxChanges)
			return Reject(EMappingStatus::Overflow);
		if (!Sample.IsValid()) return Reject(EMappingStatus::InvalidRaw);
		const bool Reset = Sample.Status == ERawSampleStatus::Resync;
		if (NeedsResync && !Reset) return Reject(EMappingStatus::ResyncRequired);
		if (!Reset && (!SameDevice(Sample) || !SameCapabilities(Sample))) return Reject(EMappingStatus::DeviceChanged);
		if (Reset && HasBaseline && Sample.DeviceId == LastDevice &&
			(Sample.Generation.Value < Generation.Value || (Sample.Generation.Value == Generation.Value && Sample.Sequence <= LastSequence)))
			return Reject(EMappingStatus::StaleBaseline);
		if (!Reset && !Continuous(Sample)) return Reject(EMappingStatus::SequenceGap);

		FInputFrameData Output;
		Output.SourceSequence = Sample.Sequence; Output.ConsumptionFrame = Frame;
		Output.Producer = Producer; Output.StreamEpoch = Epoch; Output.DeviceGeneration = Sample.Generation; Output.Reset = Reset;
		auto Working = Reset ? Sample.FinalState : Held;
		std::uint32_t Active = Reset ? 0 : HeldActive;
		if (Reset)
		{
			if (!Evaluate(Working, Active, Output.Values, Output.ActiveMask)) return Reject(EMappingStatus::InvalidValue);
		}
		else
		{
			FActionValues Values = HeldValues;
			std::uint64_t TransitionSequence = 0;
			std::uint32_t TransitionOrdinal = 0;
			for (std::size_t ChangeIndex = 0; ChangeIndex < Sample.Changes.size(); ++ChangeIndex)
			{
				const auto& Change = Sample.Changes[ChangeIndex];
				bool Found = false;
				for (auto& S : Working) if (S.Control == Change.State.Control) { S.Value = Change.State.Value; Found = true; break; }
				if (!Found) return Reject(EMappingStatus::UnsupportedControl);
				if (Change.AtomicGroup && ChangeIndex + 1 < Sample.Changes.size()
					&& Sample.Changes[ChangeIndex + 1].AtomicGroup == Change.AtomicGroup) continue;
				std::uint32_t NextActive = 0;
				if (!Evaluate(Working, Active, Values, NextActive)) return Reject(EMappingStatus::InvalidValue);
				if (TransitionSequence != Change.Order.Sequence) { TransitionSequence = Change.Order.Sequence; TransitionOrdinal = 0; }
				// Re-number generated events rather than reusing raw event ordinals:
				// one key may change several actions. Action IDs are catalogue-sorted.
				for (const auto& A : Contract->GetDescription().Actions)
				{
					const auto Bit = std::uint32_t{1} << A.Id;
					if ((Active & Bit) == (NextActive & Bit)) continue;
					if (Output.Transitions.size() == MaxEdges) return Reject(EMappingStatus::Overflow);
					Output.Transitions.push_back({A.Id, (NextActive & Bit) ? ETransition::Started : ETransition::Completed,
						Values[A.Id], {TransitionSequence, TransitionOrdinal++}});
				}
				Active = NextActive;
			}
			for (std::size_t I = 0; I < Working.size(); ++I)
				if (Working[I].Value != Sample.FinalState[I].Value) return Reject(EMappingStatus::UnexplainedState);
			Output.Values = Values; Output.ActiveMask = Active;
		}
		FInputFrame Result(Contract, Output);
		if (!Result.IsValidFor(*Contract)) return Reject(EMappingStatus::InvalidValue);
		Held = std::move(Working); HeldActive = Output.ActiveMask; HeldValues = Output.Values;
		LastDevice = Sample.DeviceId; Kind = Sample.Kind; Generation = Sample.Generation; LastSequence = Sample.Sequence;
		HasBaseline = true; NeedsResync = false;
		Exhausted = Frame == std::numeric_limits<FFrameNumber>::max();
		if (!Exhausted) ++NextFrame;
		return {EMappingStatus::Mapped, std::move(Result)};
	}
private:
	static bool Transform(float& Value, const std::vector<FScalarModifier>& Modifiers)
	{
		for (const auto& M : Modifiers)
		{
			const float Sign = Value < 0 ? -1.0f : 1.0f;
			switch (M.Kind)
			{
			case EScalarModifier::Scale: Value *= M.A; break;
			case EScalarModifier::Deadzone:
				Value = Sign * std::clamp((std::abs(Value) - M.A) / (M.B - M.A), 0.0f, 1.0f); break;
			case EScalarModifier::Exponent: if (M.A == 2) Value = Sign * Value * Value; break;
			case EScalarModifier::Clamp: Value = std::clamp(Value, M.A, M.B); break;
			default: return false;
			}
			if (!std::isfinite(Value)) return false;
		}
		return true;
	}
	FMappingResult Reject(EMappingStatus Status) { NeedsResync = true; return {Status, {}}; }
	bool SameDevice(const FRawInputSample& S) const
	{ return HasBaseline && S.DeviceId == LastDevice && S.Kind == Kind && S.Generation.Value == Generation.Value; }
	bool SameCapabilities(const FRawInputSample& S) const
	{
		if (S.FinalState.size() != Held.size()) return false;
		for (std::size_t I = 0; I < Held.size(); ++I) if (!(Held[I].Control == S.FinalState[I].Control)) return false;
		return true;
	}
	bool Continuous(const FRawInputSample& S) const
	{
		if (LastSequence == std::numeric_limits<std::uint64_t>::max() || S.Sequence <= LastSequence) return false;
		// A neutral/no-change reading is still one explicit acquisition sequence.
		if (S.Changes.empty()) return S.Sequence == LastSequence + 1;
		std::uint64_t Current = LastSequence;
		for (const auto& C : S.Changes)
		{
			if (C.Order.Sequence == Current && Current != LastSequence) continue;
			if (Current == std::numeric_limits<std::uint64_t>::max() || C.Order.Sequence != Current + 1) return false;
			Current = C.Order.Sequence;
		}
		return Current == S.Sequence;
	}
	bool Evaluate(const std::vector<FRawValue>& Raw, std::uint32_t PriorActive,
		FActionValues& Values, std::uint32_t& Active) const
	{
		Values = {}; Active = 0;
		const auto DeviceKind = Raw.front().Control.Kind == ERawControlKind::KeyboardUsage ? ERawDeviceKind::Keyboard : ERawDeviceKind::Gamepad;
		for (const auto& A : Contract->GetDescription().Actions)
		{
			if (A.Wiring == EActionWiring::Unwired) continue;
			float Sum = 0;
			for (const auto& B : Contract->GetDescription().Mapping)
			{
				if (B.Action != A.Id) continue;
				if ((B.Control.Kind == ERawControlKind::KeyboardUsage) != (DeviceKind == ERawDeviceKind::Keyboard)) continue;
				const FRawValue* Found = nullptr;
				for (const auto& S : Raw) if (S.Control == B.Control) { Found = &S; break; }
				if (!Found) return false;
				if (A.Type == EActionType::Bool) { if (Found->Value != 0) Sum = 1; }
				else
				{
					float Contribution = Found->Value * B.Scale;
					if (!std::isfinite(Contribution) || !Transform(Contribution, B.Modifiers)) return false;
					if (A.Accumulation == EActionAccumulation::Sum) Sum += Contribution;
					else if (std::abs(Contribution) >= std::abs(Sum)) Sum = Contribution;
					if (!std::isfinite(Sum)) return false;
				}
			}
			if (!Transform(Sum, A.Modifiers)) return false;
			const float Min = A.Signed ? -1.0f : 0.0f;
			Sum = std::clamp(Sum, Min, 1.0f);
			const float Magnitude = std::abs(Sum);
			const float Remapped = Magnitude <= A.Deadzone ? 0.0f : (Magnitude - A.Deadzone) / (1.0f - A.Deadzone);
			const float Curved = A.Exponent == 1.0f ? Remapped : Remapped * Remapped;
			float Response = Curved * A.Sensitivity;
			if (!std::isfinite(Response)) return false;
			Response = std::clamp(Sum < 0 ? -Response : Response, Min, 1.0f);
			const auto Bit = std::uint32_t{1} << A.Id;
			const float Threshold = (PriorActive & Bit) ? A.DeactivateAtOrBelow : A.ActivateAbove;
			if (std::abs(Response) > Threshold) Active |= Bit;
			Values[A.Id] = static_cast<std::int16_t>(std::floor(Response * A.Quantization + 0.5f));
		}
		return true;
	}
	const std::shared_ptr<const FInputActionContract> Contract;
	const FStreamEpoch Epoch;
	const FProducerIdentity Producer;
	FFrameNumber NextFrame;
	bool Exhausted = false, NeedsResync = true, HasBaseline = false;
	std::uint64_t LastDevice = 0, LastSequence = 0;
	ERawDeviceKind Kind = ERawDeviceKind::Keyboard;
	FDeviceGeneration Generation;
	std::vector<FRawValue> Held;
	FActionValues HeldValues{};
	std::uint32_t HeldActive = 0;
};
}
