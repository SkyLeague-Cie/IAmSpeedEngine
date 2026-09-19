#pragma once

#include "InputActionContract.h"

namespace Speed::Input::V2
{
enum class ETransition : std::uint8_t { Started, Completed };
struct FActionTransition
{
	FActionId Action = 0;
	ETransition State = ETransition::Started;
	std::int16_t ValueAtTransition = 0;
	FSourceOrder Order;
};
struct FInputFrameData
{
	std::uint16_t Version = 2;
	FFrameNumber SourceSequence = 0;
	FFrameNumber ConsumptionFrame = 0;
	FProducerIdentity Producer;
	FStreamEpoch StreamEpoch;
	FDeviceGeneration DeviceGeneration;
	FActionValues Values{};
	std::uint32_t ActiveMask = 0; // Required explicitly; never inferred from Values.
	std::vector<FActionTransition> Transitions;
	bool Reset = false;
};

// Opt-in versioned type. The legacy FInputFrame and all existing producers
// remain untouched; no implicit conversion can manufacture missing activity.
class FInputFrame final
{
public:
	FInputFrame(std::shared_ptr<const FInputActionContract> Contract, FInputFrameData Data)
		: Contract(std::move(Contract)), Data(std::move(Data)) {}
	const FInputFrameData& GetData() const { return Data; }
	const std::shared_ptr<const FInputActionContract>& GetContract() const { return Contract; }
	bool IsStructurallyValid() const
	{
		if (Data.Version != 2 || !Data.Producer.Id || Data.Producer.Kind > EProducerKind::Network
			|| !Data.StreamEpoch.Value || !Data.DeviceGeneration.Value || Data.Transitions.size() > MaxEdges
			|| (Data.Reset && !Data.Transitions.empty())) return false;
		for (std::size_t I = 0; I < Data.Transitions.size(); ++I)
		{
			const auto& E = Data.Transitions[I];
			if (E.Action >= ActionCount || E.State > ETransition::Completed || !E.Order.Sequence
				|| E.Order.Sequence > Data.SourceSequence || (I && !(Data.Transitions[I - 1].Order < E.Order))) return false;
		}
		return true;
	}
	bool IsValidFor(const FInputActionContract& Expected) const
	{
		if (!IsStructurallyValid() || !Contract || Contract->GetFingerprint() != Expected.GetFingerprint()) return false;
		for (std::size_t I = 0; I < ActionCount; ++I)
		{
			const auto* A = Contract->Find(static_cast<FActionId>(I));
			const bool Active = (Data.ActiveMask & (std::uint32_t{1} << I)) != 0;
			if (!A || A->Wiring == EActionWiring::Unwired)
			{ if (Data.Values[I] || Active) return false; }
			else if (!A->Accepts(Data.Values[I])) return false;
		}
		std::array<int, ActionCount> LastState{};
		for (const auto& E : Data.Transitions)
		{
			const auto* A = Contract->Find(E.Action);
			const int State = E.State == ETransition::Started ? 1 : -1;
			if (!A || A->Wiring == EActionWiring::Unwired || !A->Accepts(E.ValueAtTransition)
				|| LastState[E.Action] == State) return false;
			LastState[E.Action] = State;
		}
		for (std::size_t I = 0; I < ActionCount; ++I)
			if (LastState[I] && ((LastState[I] == 1) != ((Data.ActiveMask & (std::uint32_t{1} << I)) != 0))) return false;
		return true;
	}
private:
	std::shared_ptr<const FInputActionContract> Contract;
	FInputFrameData Data;
};
}
