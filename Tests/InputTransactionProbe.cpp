#include "IAmSpeed/Input/ActionDispatch.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include "IAmSpeed/Input/InputEdgeIdentity.h"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <new>

static int AllocationFailureCountdown = -1;
void* operator new(std::size_t Size)
{
	if (AllocationFailureCountdown == 0) throw std::bad_alloc();
	if (AllocationFailureCountdown > 0) --AllocationFailureCountdown;
	if (void* Memory = std::malloc(Size ? Size : 1)) return Memory;
	throw std::bad_alloc();
}
void operator delete(void* Memory) noexcept { std::free(Memory); }
void operator delete(void* Memory, std::size_t) noexcept { std::free(Memory); }
void* operator new[](std::size_t Size) { return ::operator new(Size); }
void operator delete[](void* Memory) noexcept { ::operator delete(Memory); }
void operator delete[](void* Memory, std::size_t) noexcept { ::operator delete(Memory); }

using namespace Speed::Input::V2;
using Speed::Input::EProducerKind;
using Speed::Input::FFrameNumber;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{ ++Checks; if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); } }
static std::shared_ptr<const FInputActionContract> Contract()
{
	FInputActionContractDescription D; D.Revision = {1}; D.Actions = FInputActionContract::BaseActions();
	D.Mapping = {{{ERawControlKind::KeyboardUsage, 4}, 0, 1}};
	D.Physical = {{0, EPhysicalDestination::Throttle}, {1, EPhysicalDestination::Brake}, {2, EPhysicalDestination::Steering}};
	return FInputActionContract::Create(D);
}
static std::shared_ptr<FInputStream> Stream(FStreamEpoch Epoch = {1})
{
	const auto C = Contract(); Check(bool(C), "contract"); std::vector<FInputFrame> Frames;
	for (FFrameNumber N = 0; N < 3; ++N)
	{
		FInputFrameData D; D.ConsumptionFrame = N; D.SourceSequence = N + 1;
		D.StreamEpoch = Epoch; D.DeviceGeneration = {1}; D.Producer = {EProducerKind::Device, 1}; D.Reset = N == 0;
		D.Values[0] = N ? 255 : 0; D.ActiveMask = N ? 1 : 0;
		if (N == 1) D.Transitions = {{0, ETransition::Started, 255, {2, 0}}};
		Frames.emplace_back(C, D);
	}
	std::shared_ptr<IInputProducer> P = FTestInputProducer::Create(C, Epoch, {EProducerKind::Device, 1}, 0, Frames);
	Check(bool(P), "sealed fixture");
	return std::make_shared<FInputStream>(P, C, Epoch, Speed::Input::FProducerIdentity{EProducerKind::Device, 1});
}
static FReservationToken Reserve(FInputStream& S, FFrameNumber N)
{
	const auto R = S.Consume(N);
	Check(R.Status == EConsumeStatus::Ready && R.Reservation && R.Targets.Valid, "ready capability");
	return *R.Reservation;
}
static void Completed(FInputStream& S, FFrameNumber N)
{
	const auto W = S.ReadCompleted(N); const auto O = S.ReadOutcome();
	Check(W && W->Frame.GetData().ConsumptionFrame == N && O && O->Frame == N
		&& O->Outcome == ETransactionOutcome::Completed && !O->AbortReason, "completed-only immutable witness");
}
class FCancelDuringPoll final : public IInputProducer
{
public:
	std::weak_ptr<FInputStream> Target;
	std::shared_ptr<const FInputActionContract> C = Contract();
	std::optional<FInputFrame> Produce(FFrameNumber) override { Target.lock()->RequestStop(); return {}; }
	const std::shared_ptr<const FInputActionContract>& GetContract() const override { return C; }
};
struct FCountingReceiver
{
	unsigned Events = 0;
	void Event(const FActionEvent&) { ++Events; }
};
int main()
{
	for (int FailureAt = 0; FailureAt < 4; ++FailureAt)
	{
		auto S = Stream(); Check(S->Activate(), "consume allocation fixture");
		const auto Baseline = Reserve(*S, 0);
		Check(S->ConfirmPhysicalCommit(Baseline, true) && S->PublishCompleted(Baseline), "consume allocation baseline");
		AllocationFailureCountdown = FailureAt;
		const auto Failed = S->Consume(1);
		AllocationFailureCountdown = -1;
		Check(Failed.Status == EConsumeStatus::InvalidInput && !Failed.Reservation, "consume allocation fails closed");
		S->RequestStop();
		Check(S->GetState() == EStreamState::Stopped && !S->ReadRecorded(1), "no stranded reservation or partial history");
	}
	for (int FailureAt = 0; FailureAt < 12; ++FailureAt)
	{
		auto S = Stream(); auto Receiver = std::make_shared<FCountingReceiver>();
		FInputPresentationBindings Bindings(S);
		Check(Bindings.BindAction("edge", 0, EStateAction::Started, std::weak_ptr<FCountingReceiver>(Receiver), &FCountingReceiver::Event)
			&& Bindings.Seal() && S->Activate(), "dispatch allocation fixture");
		for (unsigned N = 0; N < 2; ++N)
		{
			const auto T = Reserve(*S, N);
			Check(S->ConfirmPhysicalCommit(T, true) && S->PublishCompleted(T), "dispatch allocation publication");
		}
		bool Threw = false;
		AllocationFailureCountdown = FailureAt;
		try { Bindings.HandleInputs(); } catch (const std::bad_alloc&) { Threw = true; }
		AllocationFailureCountdown = -1;
		Check(!Threw || Receiver->Events == 0, "allocation failure cannot occur after a delivered callback");
		Bindings.HandleInputs();
		Check(Receiver->Events == 1, "recovery or repeated tick never duplicates edge");
	}
	for (int FailureAt : {-1, 0, 1})
	{
		auto Prepared = Stream(); Check(Prepared->Activate(), "prepared activate");
		const auto Baseline = Reserve(*Prepared, 0);
		Check(Prepared->ConfirmPhysicalCommit(Baseline, true) && Prepared->PublishCompleted(Baseline), "prepared baseline");
		const auto Token = Reserve(*Prepared, 1);
		Check(Prepared->ConfirmPhysicalCommit(Token, true), "prepared physical confirmation");
		Check(!Prepared->FinalizePublication(Token), "unprepared finalization refused");
		AllocationFailureCountdown = FailureAt;
		const bool Ready = Prepared->PreparePublication(Token);
		AllocationFailureCountdown = -1;
		Check(!Prepared->ReadCompleted(1) && Prepared->ReadCompleted(0)->Serial == 1, "preparation never publishes");
		if (FailureAt >= 0)
		{
			Check(!Ready && Prepared->GetState() == EStreamState::Stopped, "either allocation failure aborts");
			Check(!Prepared->IsPublicationPrepared(Token) && !Prepared->FinalizePublication(Token), "failed preparation grants no finalize authority");
			Check(Prepared->ReadOutcome()->AbortReason == EAbortReason::SnapshotPublicationFailed, "allocation failure reason");
			continue;
		}
		Check(Ready && Prepared->IsPublicationPrepared(Token), "successful preparation ready");
		AllocationFailureCountdown = 0;
		const bool PreparedAgain = Prepared->PreparePublication(Token);
		AllocationFailureCountdown = -1;
		Check(PreparedAgain, "double preparation needs no new allocation");
		Prepared->RequestStop();
		AllocationFailureCountdown = 0;
		const bool Finalized = Prepared->FinalizePublication(Token);
		AllocationFailureCountdown = -1;
		Check(Finalized, "finalization after stop performs no allocation");
		Completed(*Prepared, 1);
		const auto Frame = Prepared->ReadCompleted(1)->Frame;
		const auto Identity = IdentifyEdge(Frame, 0);
		Check(Identity && Identity->Epoch == 1 && Identity->Frame == 1 && Identity->SourceSequence == 2
			&& Identity->Ordinal == 0 && Identity->Action == 0 && Identity->ProducerId == 1, "edge identity comes from immutable physical frame");
		auto OtherProducer = *Identity; OtherProducer.ProducerId = 2;
		Check(!(OtherProducer == *Identity), "different producer cannot alias edge identity");
		Check(!IdentifyEdge(Frame, 1), "invalid ordinal grants no identity");
		Check(!Prepared->FinalizePublication(Token) && !Prepared->IsPublicationPrepared(Token), "no double finalization");
	}
	{
		auto Prepared = Stream(); Check(Prepared->Activate(), "prepare abort activate");
		const auto Token = Reserve(*Prepared, 0);
		Check(Prepared->ConfirmPhysicalCommit(Token, true) && Prepared->PreparePublication(Token), "prepare before abort");
		Check(Prepared->Abort(Token, EAbortReason::Cancelled) && !Prepared->ReadCompleted(0)
			&& !Prepared->FinalizePublication(Token), "abort prepared frame cannot publish");
	}
	// Deterministic interleavings: no sleeps, threads or physical/UE claims.
	for (unsigned StopAt = 0; StopAt < 4; ++StopAt)
	{
		auto S = Stream(); Check(S->Activate(), "activate"); auto T = Reserve(*S, 0);
		Check(!S->ReadCompleted(0) && !S->ReadOutcome(), "reservation is not completion");
		if (StopAt == 0) S->RequestStop(); // Ready, before modeled application.
		unsigned Applied = 1; // Native modeled grouped application, not a physics step.
		if (StopAt == 1) S->RequestStop(); // Applied, before confirm.
		Check(S->ConfirmPhysicalCommit(T, true), "token owner confirms even while draining");
		if (StopAt == 2) S->RequestStop(); // Confirmed, before completion.
		Check(!S->ConfirmPhysicalCommit(T, true), "duplicate confirmation rejected");
		if (StopAt < 3)
		{
			Check(S->GetState() == EStreamState::Draining && !S->IsActive() && !S->ReadLatest(), "presentation detaches while owner drains");
			Check(S->Consume(1).Status == EConsumeStatus::Detached && !S->Activate(), "new work/replacement of same stream rejected");
			S->RequestStop(); Check(S->GetState() == EStreamState::Draining, "repeated stop preserves reservation");
		}
		Check(S->PublishCompleted(T), "canonical completion publishes reserved frame");
		if (StopAt == 3) S->RequestStop();
		Check(S->GetState() == EStreamState::Stopped && Applied == 1, "stop fence completes without duplicate apply");
		Completed(*S, 0);
		const auto W = *S->ReadCompleted(0);
		Check(!S->PublishCompleted(T) && !S->Abort(T, EAbortReason::Cancelled)
			&& !S->ConfirmPhysicalCommit(T, true) && S->ReadCompleted(0)->Serial == W.Serial, "finalized token cannot publish/abort/confirm twice");
	}
	for (unsigned Phase = 0; Phase < 3; ++Phase)
	{
		auto S = Stream(); Check(S->Activate(), "abort activate"); const auto T = Reserve(*S, 0);
		if (Phase) Check(S->ConfirmPhysicalCommit(T, true), "abort after modeled apply");
		S->RequestStop();
		if (Phase == 0) Check(!S->ConfirmPhysicalCommit(T, false), "failed apply uses terminal abort");
		else Check(S->Abort(T, Phase == 1 ? EAbortReason::SnapshotPublicationFailed : EAbortReason::Cancelled), "explicit owner abort");
		const auto O = S->ReadOutcome();
		Check(O && O->Outcome == ETransactionOutcome::Aborted && O->AbortReason && O->Frame == 0
			&& *O->AbortReason == (Phase == 0 ? EAbortReason::ApplicationFailed : Phase == 1 ? EAbortReason::SnapshotPublicationFailed : EAbortReason::Cancelled), "distinguishable abort reason");
		Check(S->GetState() == EStreamState::Stopped && !S->ReadCompleted(0) && !S->PublishCompleted(T)
			&& !S->Abort(T, EAbortReason::Cancelled) && !S->Activate(), "aborted frame never completed/retried");
	}
	auto Idle = Stream(); Idle->RequestStop(); Check(Idle->GetState() == EStreamState::Stopped && !Idle->Activate(), "stop before activation");
	auto Before = Stream(); Check(Before->Activate(), "idle activate"); Before->RequestStop();
	Check(Before->GetState() == EStreamState::Stopped && !Before->Consume(0).Reservation, "stop before acquisition grants no token");
	auto Reentrant = std::make_shared<FCancelDuringPoll>();
	auto During = std::make_shared<FInputStream>(Reentrant, Reentrant->C, FStreamEpoch{1}, Speed::Input::FProducerIdentity{EProducerKind::Device, 1});
	Reentrant->Target = During; Check(During->Activate(), "source stop activate"); const auto Cancelled = During->Consume(0);
	Check(Cancelled.Status == EConsumeStatus::InvalidInput && !Cancelled.Reservation && !Cancelled.Targets.Valid
		&& During->GetState() == EStreamState::Stopped && !During->ReadCompleted(0), "stop during source produces no applyable capability");

	auto A = Stream(); auto B = Stream(); auto OtherEpoch = Stream({2});
	Check(A->Activate() && B->Activate() && OtherEpoch->Activate(), "token ownership fixtures");
	const auto TA = Reserve(*A, 0); const auto TB = Reserve(*B, 0); const auto TE = Reserve(*OtherEpoch, 0);
	for (const auto& Bad : {FReservationToken{}, TB, TE})
		Check(!A->ConfirmPhysicalCommit(Bad, true) && !A->PublishCompleted(Bad) && !A->Abort(Bad, EAbortReason::Cancelled)
			&& A->GetState() == EStreamState::Reserved, "wrong stream/epoch/empty token cannot alter reservation");
	Check(A->ConfirmPhysicalCommit(TA, true) && A->PublishCompleted(TA), "valid owner completes");
	const auto Next = Reserve(*A, 1);
	Check(!A->ConfirmPhysicalCommit(TA, true) && !A->PublishCompleted(TA) && !A->Abort(TA, EAbortReason::Cancelled), "old frame token cannot finalize next reservation");
	Check(A->ConfirmPhysicalCommit(Next, true) && A->PublishCompleted(Next), "next owner unaffected by stale requests");
	B->Abort(TB, EAbortReason::Cancelled); OtherEpoch->Abort(TE, EAbortReason::Cancelled);

	// A throwing presentation callback for completed frame0 races logically with
	// the worker's reservation for frame1. Stop must preserve the latter.
	auto S = Stream(); FInputPresentationBindings Bindings(S);
	Check(Bindings.BindSnapshotAction("throw", 0, [](const FInputFrame&, Speed::Input::FActionId)
		{ throw std::runtime_error("receiver"); }) && Bindings.Seal() && S->Activate(), "throwing callback setup");
	const auto T0 = Reserve(*S, 0); Check(S->ConfirmPhysicalCommit(T0, true) && S->PublishCompleted(T0), "first frame complete");
	const auto T1 = Reserve(*S, 1);
	Check(Bindings.HandleInputs() == EDispatchStatus::Detached && S->GetState() == EStreamState::Draining, "callback exception requests deferred stop");
	Check(S->ConfirmPhysicalCommit(T1, true) && S->PublishCompleted(T1), "pending owner can finish after receiver throw");
	Completed(*S, 1); // Outcome is the last finalization; older completed witnesses remain readable.
	Check(S->ReadCompleted(0) && S->ReadCompleted(1) && S->GetState() == EStreamState::Stopped, "all successful canonical frames witnessed after stop");
	std::cout << "PASS InputTransactionProbe checks=" << Checks << '\n';
}
