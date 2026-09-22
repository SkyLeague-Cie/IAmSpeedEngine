#pragma once

#include "RawAcquisitionJournal.h"

namespace Speed::Input::V2
{
enum class EControlCommand : std::uint8_t { Pause, ResetWorld, AutoControl };
struct FControlBinding { FActionId Action = 0; EControlCommand Command = EControlCommand::Pause; };
struct FControlRequest
{
	std::uint64_t Session = 0;
	FProducerIdentity Producer;
	FDeviceGeneration Generation;
	std::uint64_t AcquisitionSequence = 0;
	std::uint32_t Ordinal = 0;
	FActionId Action = 0;
	EControlCommand Command = EControlCommand::Pause;
	EStateAction State = EStateAction::Started;
};
enum class EControlRead { NoChange, Batch, Resynchronized, WaitingForBaseline, Invalid };
struct FControlRequests { EControlRead Status = EControlRead::NoChange; std::vector<FControlRequest> Requests; };

// Values-only control-lane reader. The game executes only eligible requests
// (normally Started) and returns its own application receipts. No physical
// frames are invented; no callbacks run here. A replacement world must create
// a new journal session and reject requests addressed to the previous session.
class FControlActionReader final
{
public:
	static std::unique_ptr<FControlActionReader> Create(std::shared_ptr<const FInputActionContract> Contract,
		std::shared_ptr<FRawAcquisitionJournal> Journal, FProducerIdentity Producer,
		std::uint64_t Session, std::vector<FControlBinding> Bindings)
	{
		if (!Contract || !Journal || !Producer.Id || Producer.Kind > EProducerKind::Network || !Session || Bindings.empty()) return {};
		std::uint32_t Used = 0;
		for (const auto& B : Bindings)
		{
			const auto* A = Contract->Find(B.Action);
			if (!A || A->Type != EActionType::Bool || A->Wiring != EActionWiring::Wired
				|| B.Command > EControlCommand::AutoControl || (Used & (std::uint32_t{1} << B.Action))) return {};
			Used |= std::uint32_t{1} << B.Action;
		}
		std::sort(Bindings.begin(), Bindings.end(), [](const auto& A, const auto& B) { return A.Action < B.Action; });
		return std::unique_ptr<FControlActionReader>(new FControlActionReader(std::move(Contract), std::move(Journal), Producer, Session, std::move(Bindings)));
	}
	FControlRequests Read()
	{
		const auto History = Journal->ReadControlsSince(Cursor);
		if (History.Status == EAcquisitionRead::Closed || History.Status == EAcquisitionRead::InvalidCursor) return {EControlRead::Invalid, {}};
		if (History.Status == EAcquisitionRead::Gap || NeedsBaseline)
		{
			const auto Base = Journal->ReadControlBaseline();
			if (!Base) { NeedsBaseline = true; return {EControlRead::WaitingForBaseline, {}}; }
			std::uint32_t FreshActive = 0;
			if (Base->Cursor.Session != Cursor.Session || !Evaluate(Base->State, FreshActive)) return {EControlRead::Invalid, {}};
			Active = FreshActive; Cursor = Base->Cursor; NeedsBaseline = false;
			return {EControlRead::Resynchronized, {}}; // No invented Started/Completed.
		}
		if (History.Status == EAcquisitionRead::NoChange) return {};
		FControlRequests Out; Out.Status = EControlRead::Batch;
		std::uint32_t PreparedActive = Active;
		for (const auto& R : History.Readings)
		{
			std::uint32_t NextActive = 0;
			if (!Evaluate(R, NextActive)) return {EControlRead::Invalid, {}};
			if (!R.FreshBaseline)
			{
				std::uint32_t Ordinal = 0;
				for (const auto& B : Bindings)
				{
					const auto Bit = std::uint32_t{1} << B.Action;
					if ((PreparedActive & Bit) != (NextActive & Bit))
						Out.Requests.push_back({Cursor.Session, Producer, R.Generation, R.Sequence, Ordinal++, B.Action, B.Command,
							(NextActive & Bit) ? EStateAction::Started : EStateAction::Completed});
				}
			}
			PreparedActive = NextActive;
		}
		Active = PreparedActive; Cursor = History.Next; return Out;
	}
private:
	FControlActionReader(std::shared_ptr<const FInputActionContract> C, std::shared_ptr<FRawAcquisitionJournal> J,
		FProducerIdentity P, std::uint64_t Session, std::vector<FControlBinding> B)
		: Contract(std::move(C)), Journal(std::move(J)), Producer(P), Bindings(std::move(B)), Cursor{Session, 0} {}
	bool Evaluate(const FAcquiredRawState& R, std::uint32_t& Mask) const
	{
		if (!R.State.IsValid(R.Kind)) return false;
		Mask = 0;
		for (const auto& B : Bindings)
			for (const auto& M : Contract->GetDescription().Mapping)
			{
				if (M.Action != B.Action || ((M.Control.Kind == ERawControlKind::KeyboardUsage) != (R.Kind == ERawDeviceKind::Keyboard))) continue;
				const FRawValue* Found = nullptr;
				for (std::size_t I = 0; I < R.State.Count; ++I) if (R.State.Values[I].Control == M.Control) { Found = &R.State.Values[I]; break; }
				if (!Found) return false;
				if (Found->Value != 0) Mask |= std::uint32_t{1} << B.Action;
			}
		return true;
	}
	const std::shared_ptr<const FInputActionContract> Contract;
	const std::shared_ptr<FRawAcquisitionJournal> Journal;
	const FProducerIdentity Producer;
	const std::vector<FControlBinding> Bindings;
	FAcquisitionCursor Cursor;
	std::uint32_t Active = 0;
	bool NeedsBaseline = false; // The journal begins with an explicit baseline; retain later edges even before the first read.
};
}
