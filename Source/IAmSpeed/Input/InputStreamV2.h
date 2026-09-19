#pragma once

#include "InputProducerV2.h"
#include "PhysicalActionSink.h"
#include <mutex>

namespace Speed::Input::V2
{
struct FPublicationCursor { FStreamEpoch Epoch; std::uint64_t Serial = 0; };
struct FPublishedFrame { FInputFrame Frame; std::uint64_t Serial = 0; };
enum class EReadStatus : std::uint8_t { NoChange, Batch, Overflow, Detached, InvalidCursor };
struct FPublishedBatch
{
	EReadStatus Status = EReadStatus::Detached;
	FPublicationCursor Next;
	std::vector<FPublishedFrame> Frames;
};
enum class EConsumeStatus : std::uint8_t { Ready, AlreadyPending, PublishedReplay, Detached, WrongFrame, InvalidInput, PresentationForbidden };
enum class EStreamState : std::uint8_t { Configurable, ActiveIdle, Reserved, Committed, Draining, Stopped };
enum class EAbortReason : std::uint8_t { ApplicationFailed, SnapshotPublicationFailed, Cancelled };
enum class ETransactionOutcome : std::uint8_t { Completed, Aborted };

// Copyable capability, never reconstructible from an epoch/frame pair. The
// shared identity stays alive with stale copies, preventing address reuse.
class FReservationToken final
{
public:
	FReservationToken() = default;
private:
	friend class FInputStream;
	FReservationToken(FStreamEpoch InEpoch, FFrameNumber InFrame)
		: Epoch(InEpoch), Frame(InFrame), Identity(std::make_shared<const std::uint8_t>(std::uint8_t{0})) {}
	FStreamEpoch Epoch;
	FFrameNumber Frame = 0;
	std::shared_ptr<const std::uint8_t> Identity;
};
struct FTransactionOutcome
{
	FStreamEpoch Epoch;
	FFrameNumber Frame = 0;
	ETransactionOutcome Outcome = ETransactionOutcome::Aborted;
	std::optional<EAbortReason> AbortReason;
};
struct FConsumedInput
{
	EConsumeStatus Status = EConsumeStatus::Detached;
	std::optional<FInputFrame> Frame;
	FDrivingInputTargets Targets; // Neutral/invalid for every failure.
	std::optional<FReservationToken> Reservation; // Ready only; replay grants no authority.
};

// Single physical-lane producer/consumer, synchronized publication copies for
// presentation. Both player and sealed tests use this exact class and sink.
class FInputStream final
{
public:
	FInputStream(std::shared_ptr<IInputProducer> InSource, std::shared_ptr<const FInputActionContract> InContract,
		FStreamEpoch InEpoch, FProducerIdentity InIdentity, FFrameNumber FirstFrame = 0)
		: Source(std::move(InSource)), Contract(std::move(InContract)), Epoch(InEpoch), Identity(InIdentity), NextFrame(FirstFrame) {}
	const std::shared_ptr<const FInputActionContract>& GetContract() const { return Contract; }
	FStreamEpoch GetEpoch() const { return Epoch; }
	bool CanConfigure() const { std::lock_guard<std::recursive_mutex> Lock(Gate); return !CallingSource && !Started && !Terminated; }
	bool IsActive() const { std::lock_guard<std::recursive_mutex> Lock(Gate); return Active; }
	EStreamState GetState() const
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (Terminated) return EStreamState::Stopped;
		if (StopRequested) return EStreamState::Draining;
		if (Pending) return History[Pending->Frame % HistoryCapacity]->Committed ? EStreamState::Committed : EStreamState::Reserved;
		return Started ? EStreamState::ActiveIdle : EStreamState::Configurable;
	}
	bool Activate()
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource) { Fault(); return false; }
		if (Started || Terminated || !Source || !Contract || !Epoch.Value || !Identity.Id
			|| Identity.Kind > EProducerKind::Network) return false;
		std::shared_ptr<const FInputActionContract> SourceContract;
		CallingSource = true;
		try { SourceContract = Source->GetContract(); }
		catch (...) { CallingSource = false; Fault(); return false; }
		CallingSource = false;
		if (Terminated || !SourceContract || SourceContract->GetFingerprint() != Contract->GetFingerprint()) return false;
		Started = Active = true; return true;
	}
	void Deactivate()
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		Active = false;
		if (Pending) StopRequested = true; // Owner alone can drain; never wait on another lane.
		else Terminated = true;
	}
	void RequestStop() { Deactivate(); }
	FConsumedInput Consume(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return {EConsumeStatus::PresentationForbidden, {}, {}};
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource) return Fault();
		if (!Active) return {};
		const auto& Retained = History[Frame % HistoryCapacity];
		if (Retained && Retained->Frame.GetData().ConsumptionFrame == Frame)
		{
			if (!Retained->Published) return {EConsumeStatus::AlreadyPending, {}, {}};
			return {EConsumeStatus::PublishedReplay, Retained->Frame, Retained->Targets};
		}
		if (Exhausted || Frame != NextFrame || Pending) return {EConsumeStatus::WrongFrame, {}, {}};
		if (Serial == std::numeric_limits<std::uint64_t>::max()) return Fault(); // Never grant an unpublishable reservation.
		std::optional<FInputFrame> Input;
		CallingSource = true;
		try { Input = Source->Produce(Frame); }
		catch (...) { CallingSource = false; return Fault(); }
		CallingSource = false;
		if (!Active || Terminated) return Fault(); // Reentrant cancellation invalidates this in-flight reading.
		if (!Input || !Input->IsValidFor(*Contract) || !ValidContinuity(*Input, Frame)) return Fault();
		const auto Targets = AssembleDrivingTargets(*Input, *Contract, Epoch, Frame);
		if (!Targets.Valid) return Fault();
		History[Frame % HistoryCapacity] = FHistoryEntry{*Input, Targets, false, false};
		Pending = FReservationToken(Epoch, Frame);
		return {EConsumeStatus::Ready, *Input, Targets, Pending};
	}
	// The sole physical consumer calls this AFTER its grouped target commit.
	// False permanently fails closed; no incomplete frame can be published.
	bool ConfirmPhysicalCommit(const FReservationToken& Token, bool Succeeded)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource) { Fault(); return false; }
		if (!Matches(Token)) return false;
		auto& Slot = History[Token.Frame % HistoryCapacity];
		if (Slot->Committed) return false;
		if (!Succeeded) { AbortLocked(Token, EAbortReason::ApplicationFailed); return false; }
		Slot->Committed = true; return true;
	}
	bool PublishCompleted(const FReservationToken& Token)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource) { Fault(); return false; }
		if (!Matches(Token)) return false;
		const auto Frame = Token.Frame;
		auto& Slot = History[Frame % HistoryCapacity];
		if (!Slot->Committed) return false;
		if (Serial == std::numeric_limits<std::uint64_t>::max()) { AbortLocked(Token, EAbortReason::SnapshotPublicationFailed); return false; }
		const auto NextSerial = Serial + 1;
		Published[NextSerial % HistoryCapacity] = FPublishedFrame{Slot->Frame, NextSerial};
		Serial = NextSerial; Slot->Published = true; LastFrame = Slot->Frame;
		Outcome = FTransactionOutcome{Epoch, Frame, ETransactionOutcome::Completed, {}};
		Pending.reset(); Exhausted = Frame == std::numeric_limits<FFrameNumber>::max();
		if (!Exhausted) NextFrame = Frame + 1;
		if (StopRequested) Terminated = true;
		return true;
	}
	bool Abort(const FReservationToken& Token, EAbortReason Reason)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource) { Fault(); return false; }
		if (!Matches(Token) || Reason > EAbortReason::Cancelled) return false;
		AbortLocked(Token, Reason); return true;
	}
	// Diagnostic completion witness, including after stop. Never returns merely
	// consumed/committed data; bounded by the same completed publication journal.
	std::optional<FPublishedFrame> ReadCompleted(FFrameNumber Frame) const
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		for (const auto& P : Published) if (P && P->Frame.GetData().ConsumptionFrame == Frame) return P;
		return {};
	}
	std::optional<FTransactionOutcome> ReadOutcome() const
	{ std::lock_guard<std::recursive_mutex> Lock(Gate); return Outcome; }
	FPublishedBatch ReadPublishedSince(FPublicationCursor Cursor) const
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (!Active || Cursor.Epoch.Value != Epoch.Value) return {};
		const FPublicationCursor Next{Epoch, Serial};
		if (Cursor.Serial > Serial) return {EReadStatus::InvalidCursor, Next, {}};
		if (Cursor.Serial == Serial) return {EReadStatus::NoChange, Next, {}};
		if (Serial - Cursor.Serial > HistoryCapacity) return {EReadStatus::Overflow, Next, {}};
		std::vector<FPublishedFrame> Copies;
		for (std::uint64_t I = Cursor.Serial; I != Serial;)
		{
			++I;
			const auto& Slot = Published[I % HistoryCapacity];
			if (!Slot || Slot->Serial != I) return {EReadStatus::Overflow, Next, {}};
			Copies.push_back(*Slot);
		}
		return {EReadStatus::Batch, Next, std::move(Copies)};
	}
	std::optional<FPublishedFrame> ReadLatest() const
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		return Active && Serial ? Published[Serial % HistoryCapacity] : std::nullopt;
	}
	std::optional<FInputFrame> ReadRecorded(FFrameNumber Frame) const
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		const auto& Slot = History[Frame % HistoryCapacity];
		return Slot && Slot->Frame.GetData().ConsumptionFrame == Frame ? std::optional<FInputFrame>(Slot->Frame) : std::nullopt;
	}
private:
	struct FHistoryEntry { FInputFrame Frame; FDrivingInputTargets Targets; bool Committed; bool Published; };
	bool Matches(const FReservationToken& Token) const
	{
		return !Terminated && Pending && Token.Identity && Token.Identity == Pending->Identity
			&& Token.Epoch.Value == Epoch.Value && Token.Frame == Pending->Frame;
	}
	void AbortLocked(const FReservationToken& Token, EAbortReason Reason)
	{
		Outcome = FTransactionOutcome{Epoch, Token.Frame, ETransactionOutcome::Aborted, Reason};
		Pending.reset(); Active = false; Terminated = true;
	}
	FConsumedInput Fault() { Active = false; Terminated = true; Pending.reset(); return {EConsumeStatus::InvalidInput, {}, {}}; }
	bool ValidContinuity(const FInputFrame& Frame, FFrameNumber Address) const
	{
		const auto& D = Frame.GetData();
		if (D.ConsumptionFrame != Address || D.StreamEpoch.Value != Epoch.Value || D.Producer.Id != Identity.Id || D.Producer.Kind != Identity.Kind) return false;
		if (!LastFrame) return D.Reset;
		if (D.Reset) return true;
		const auto& Previous = LastFrame->GetData();
		if (D.DeviceGeneration.Value != Previous.DeviceGeneration.Value || D.SourceSequence < Previous.SourceSequence
			|| (D.SourceSequence == Previous.SourceSequence && (D.Values != Previous.Values || !D.Transitions.empty()))) return false;
		std::uint32_t ActiveMask = Previous.ActiveMask;
		for (const auto& E : D.Transitions)
		{
			const auto Bit = std::uint32_t{1} << E.Action;
			const bool StartedEdge = E.State == ETransition::Started;
			if (E.Order.Sequence <= Previous.SourceSequence || (((ActiveMask & Bit) != 0) == StartedEdge)) return false;
			if (StartedEdge) ActiveMask |= Bit; else ActiveMask &= ~Bit;
		}
		return ActiveMask == D.ActiveMask;
	}
	const std::shared_ptr<IInputProducer> Source;
	const std::shared_ptr<const FInputActionContract> Contract;
	const FStreamEpoch Epoch;
	const FProducerIdentity Identity;
	// Recursive only to detect same-thread producer reentry and fail closed.
	// Other threads still cannot mutate lifecycle during the reserved source call.
	mutable std::recursive_mutex Gate;
	bool CallingSource = false;
	bool Started = false, Active = false, Terminated = false, Exhausted = false, StopRequested = false;
	FFrameNumber NextFrame;
	std::optional<FReservationToken> Pending;
	std::optional<FTransactionOutcome> Outcome;
	std::optional<FInputFrame> LastFrame;
	std::uint64_t Serial = 0;
	std::array<std::optional<FHistoryEntry>, HistoryCapacity> History{};
	std::array<std::optional<FPublishedFrame>, HistoryCapacity> Published{};
};
}
