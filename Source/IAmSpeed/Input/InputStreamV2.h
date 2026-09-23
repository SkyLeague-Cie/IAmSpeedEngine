#pragma once

#include "InputProducerV2.h"
#include "InputPublication.h"
#include "PhysicalActionSink.h"
#include <mutex>
#include <type_traits>

namespace Speed::Input::V2
{
enum class EConsumeStatus : std::uint8_t { Ready, AlreadyPending, PublishedReplay, Detached, WrongFrame, InvalidInput, PresentationForbidden, Paused };
enum class EStreamState : std::uint8_t { Configurable, ActiveIdle, Reserved, Committed, Draining, Stopped };
enum class EAbortReason : std::uint8_t { ApplicationFailed, SnapshotPublicationFailed, Cancelled };
enum class ETransactionOutcome : std::uint8_t { Completed, Aborted };
class FInputStream; // Resolve the friend to this namespace, not UE's global FInputStream.

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
class FInputStream final : public IInputPublicationSource
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
	bool IsLifecyclePaused() const { std::lock_guard<std::recursive_mutex> Lock(Gate); return LifecyclePaused; }
	ELifecycleResult SetLifecyclePaused(bool Paused)
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource || Pending || Terminated || !Source) return ELifecycleResult::Rejected;
		ELifecycleResult Result = ELifecycleResult::Rejected;
		CallingSource = true;
		try { Result = Source->SetLifecyclePaused(Paused); }
		catch (...) { Result = ELifecycleResult::Rejected; }
		CallingSource = false;
		if (Terminated || (Result != ELifecycleResult::Applied && Result != ELifecycleResult::Unaffected))
		{ Fault(); return ELifecycleResult::Rejected; }
		LifecyclePaused = Paused;
		if (Paused) { PresentationBarrier = Serial; AwaitingFreshPublication = true; }
		return Result;
	}
	// Cancellation can call platform code and therefore cannot run inside the
	// allocation-free finalizer. The host first drains/aborts its reservation.
	ELifecycleResult CancelSourceAtBoundary()
	{
		if (FPresentationInputScope::IsActive()) return ELifecycleResult::Rejected;
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource || Pending || !Source) return ELifecycleResult::Rejected;
		if (SourceCancelled) return CancellationResult;
		Active = false; Terminated = true;
		CallingSource = true;
		ELifecycleResult Result = ELifecycleResult::Rejected;
		try { Result = Source->CancelLifecycle(); }
		catch (...) { Result = ELifecycleResult::Rejected; }
		CallingSource = false;
		if (Result != ELifecycleResult::Applied && Result != ELifecycleResult::Unaffected) return ELifecycleResult::Rejected;
		SourceCancelled = true; CancellationResult = Result;
		return Result;
	}
	FConsumedInput Consume(FFrameNumber Frame)
	{
		if (FPresentationInputScope::IsActive()) return {EConsumeStatus::PresentationForbidden, {}, {}};
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource) return Fault();
		if (!Active) return {};
		if (LifecyclePaused) return {EConsumeStatus::Paused, {}, {}};
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
		try
		{
			FHistoryEntry Entry{*Input, Targets, false, false};
			FReservationToken Token(Epoch, Frame);
			FConsumedInput Result{EConsumeStatus::Ready, *Input, Targets, Token};
			static_assert(std::is_nothrow_move_constructible_v<FConsumedInput>);
			static_assert(std::is_nothrow_move_assignable_v<std::optional<FHistoryEntry>>);
			static_assert(std::is_nothrow_move_assignable_v<std::optional<FReservationToken>>);
			History[Frame % HistoryCapacity] = std::move(Entry);
			Pending = std::move(Token);
			return Result;
		}
		catch (...) { return Fault(); } // Never strand a reservation the caller cannot own.
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
	// Allocating preparation belongs before the global snapshot publication.
	// Neither copy is visible until both have succeeded and the owner finalizes.
	bool PreparePublication(const FReservationToken& Token)
	{
		if (FPresentationInputScope::IsActive()) return false;
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		if (CallingSource) { Fault(); return false; }
		if (!Matches(Token)) return false;
		if (PublicationPrepared) return PreparedMatches(Token);
		auto& Slot = History[Token.Frame % HistoryCapacity];
		if (!Slot->Committed) return false;
		if (Serial == std::numeric_limits<std::uint64_t>::max()) { AbortLocked(Token, EAbortReason::SnapshotPublicationFailed); return false; }
		try
		{
			std::optional<FPublishedFrame> Publication{FPublishedFrame{Slot->Frame, Serial + 1}};
			std::optional<FInputFrame> Continuity{Slot->Frame};
			PreparedPublication.swap(Publication);
			PreparedContinuity.swap(Continuity);
			PublicationPrepared = true;
		}
		catch (...) { AbortLocked(Token, EAbortReason::SnapshotPublicationFailed); return false; }
		return true;
	}
	bool IsPublicationPrepared(const FReservationToken& Token) const
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		return PreparedMatches(Token);
	}
	// Acquire every potentially failing synchronization BEFORE world publication.
	// The physical owner must next finalize or abort on this same thread.
	bool PrepareFinalization(const FReservationToken& Token)
	{
		if (FPresentationInputScope::IsActive()) return false;
		try
		{
			if (FinalizationLock) return PreparedMatches(Token);
			std::unique_lock<std::recursive_mutex> Lock(Gate);
			if (CallingSource || !PreparedMatches(Token)) return false;
			FinalizationLock.emplace(std::move(Lock)); return true;
		}
		catch (...) { return false; }
	}
	void FinalizePreparedPublication() noexcept
	{
		// Preconditions established by PrepareFinalization, retained under Gate.
		const auto Frame = Pending->Frame;
		const auto NextSerial = Serial + 1;
		Published[NextSerial % HistoryCapacity].swap(PreparedPublication);
		LastFrame.swap(PreparedContinuity);
		Serial = NextSerial; History[Frame % HistoryCapacity]->Published = true;
		Outcome = FTransactionOutcome{Epoch, Frame, ETransactionOutcome::Completed, {}};
		PublicationPrepared = false; AwaitingFreshPublication = false;
		Pending.reset(); Exhausted = Frame == std::numeric_limits<FFrameNumber>::max();
		if (!Exhausted) NextFrame = Frame + 1;
		if (StopRequested) Terminated = true;
		FinalizationLock.reset();
	}
	bool FinalizePublication(const FReservationToken& Token) noexcept
	{
		static_assert(std::is_nothrow_swappable_v<std::optional<FPublishedFrame>>);
		static_assert(std::is_nothrow_swappable_v<std::optional<FInputFrame>>);
		if (FPresentationInputScope::IsActive()) return false;
		try
		{
			std::lock_guard<std::recursive_mutex> Lock(Gate);
			if (CallingSource || !PreparedMatches(Token)) return false;
			const auto Frame = Token.Frame;
			const auto NextSerial = Serial + 1;
			Published[NextSerial % HistoryCapacity].swap(PreparedPublication);
			LastFrame.swap(PreparedContinuity);
			Serial = NextSerial; History[Frame % HistoryCapacity]->Published = true;
			Outcome = FTransactionOutcome{Epoch, Frame, ETransactionOutcome::Completed, {}};
			PublicationPrepared = false;
			AwaitingFreshPublication = false;
			Pending.reset(); Exhausted = Frame == std::numeric_limits<FFrameNumber>::max();
			if (!Exhausted) NextFrame = Frame + 1;
			if (StopRequested) Terminated = true;
			return true;
		}
		catch (...) { return false; } // Lock acquisition failure occurs before mutation.
	}
	// Convenience for portable callers; the global worker MUST split these phases.
	bool PublishCompleted(const FReservationToken& Token)
	{ return PreparePublication(Token) && FinalizePublication(Token); }
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
		if (LifecyclePaused || AwaitingFreshPublication) return {EReadStatus::NoChange, Cursor, {}};
		if (Cursor.Serial < PresentationBarrier) return {EReadStatus::Overflow, Next, {}};
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
		return Active && !LifecyclePaused && !AwaitingFreshPublication && Serial ? Published[Serial % HistoryCapacity] : std::nullopt;
	}
	std::optional<FInputFrame> ReadRecorded(FFrameNumber Frame) const
	{
		std::lock_guard<std::recursive_mutex> Lock(Gate);
		const auto& Slot = History[Frame % HistoryCapacity];
		return Slot && Slot->Frame.GetData().ConsumptionFrame == Frame ? std::optional<FInputFrame>(Slot->Frame) : std::nullopt;
	}
private:
	struct FHistoryEntry { FInputFrame Frame; FDrivingInputTargets Targets; bool Committed; bool Published; };
	bool PreparedMatches(const FReservationToken& Token) const
	{
		return Matches(Token) && PublicationPrepared && PreparedPublication && PreparedContinuity
			&& History[Token.Frame % HistoryCapacity]->Committed
			&& PreparedPublication->Serial == Serial + 1
			&& PreparedPublication->Frame.GetData().ConsumptionFrame == Token.Frame
			&& PreparedContinuity->GetData().ConsumptionFrame == Token.Frame;
	}
	bool Matches(const FReservationToken& Token) const
	{
		return !Terminated && Pending && Token.Identity && Token.Identity == Pending->Identity
			&& Token.Epoch.Value == Epoch.Value && Token.Frame == Pending->Frame;
	}
	void AbortLocked(const FReservationToken& Token, EAbortReason Reason)
	{
		PublicationPrepared = false;
		Outcome = FTransactionOutcome{Epoch, Token.Frame, ETransactionOutcome::Aborted, Reason};
		Pending.reset(); Active = false; Terminated = true;
		FinalizationLock.reset();
	}
	FConsumedInput Fault() { PublicationPrepared = false; Active = false; Terminated = true; Pending.reset(); return {EConsumeStatus::InvalidInput, {}, {}}; }
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
	std::optional<std::unique_lock<std::recursive_mutex>> FinalizationLock;
	bool CallingSource = false;
	bool LifecyclePaused = false;
	bool SourceCancelled = false;
	ELifecycleResult CancellationResult = ELifecycleResult::Rejected;
	bool AwaitingFreshPublication = false;
	std::uint64_t PresentationBarrier = 0;
	bool Started = false, Active = false, Terminated = false, Exhausted = false, StopRequested = false;
	FFrameNumber NextFrame;
	std::optional<FReservationToken> Pending;
	std::optional<FTransactionOutcome> Outcome;
	std::optional<FInputFrame> LastFrame;
	bool PublicationPrepared = false;
	std::optional<FPublishedFrame> PreparedPublication;
	std::optional<FInputFrame> PreparedContinuity;
	std::uint64_t Serial = 0;
	std::array<std::optional<FHistoryEntry>, HistoryCapacity> History{};
	std::array<std::optional<FPublishedFrame>, HistoryCapacity> Published{};
};
}
