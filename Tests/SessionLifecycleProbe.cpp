#include "IAmSpeed/World/Simulation/Lifecycle/SessionLifecycle.h"
#include <cstdlib>
#include <iostream>

using namespace Speed::Lifecycle;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{
    ++Checks;
    if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); }
}
static std::shared_ptr<const FCodeLease> Lease()
{ return std::make_shared<const FCodeLease>(FCodeLease{17}); }

template<std::size_t N>
static void Retire(TSessionLifecycle<N>& S, const FParticipantHandle& P)
{
    if (S.ParticipantState(P) == EParticipantState::Active)
        Check(S.RequestRetire(P), "request retirement");
    if (S.ParticipantState(P) == EParticipantState::RetireRequested)
        Check(S.BeginParticipantDrain(P), "begin participant drain");
    Check(S.AcknowledgeDetached(P), "detach after reservation resolution");
    Check(!S.AcknowledgeDetached(P), "duplicate detach rejected");
}
template<std::size_t N, std::size_t M>
static void Shutdown(TSessionLifecycle<N>& S, const std::array<FParticipantHandle, M>& Participants)
{
    S.RequestStop();
    const auto K = S.GetKey();
    TSessionLifecycle<1> Foreign(K.GetEpoch(), Lease());
    Check(S.GetState() == ESessionState::StopRequested, "stop request distinct from stopped");
    Check(!S.AcknowledgeStopped(K) && !S.AcknowledgeThreadExited(K) && !S.Reap(K), "no skipped lifecycle fences");
    Check(S.BeginStopDrain(K), "begin session drain");
    Check(!S.BeginStopDrain(K), "duplicate drain rejected");
    for (const auto& P : Participants)
        if (S.ParticipantState(P) != EParticipantState::Detached) Retire(S, P);
    Check(!S.AcknowledgeStopped(Foreign.GetKey()), "foreign stop ack rejected at valid phase");
    Check(S.AcknowledgeStopped(K), "all participants detached before stopped");
    Check(!S.AcknowledgeStopped(K) && !S.Reap(K), "stopped is neither duplicate ack nor exited");
    Check(S.HoldsCodeLease(), "stopped retains code lease");
    Check(!S.AcknowledgeThreadExited(Foreign.GetKey()), "foreign exit ack rejected at valid phase");
    Check(S.AcknowledgeThreadExited(K), "explicit thread exit witness");
    Check(!S.AcknowledgeThreadExited(K), "duplicate exit rejected");
    Check(S.HoldsCodeLease(), "exit retains code until reaping");
    Check(!S.Reap(Foreign.GetKey()), "foreign reap rejected at valid phase");
    Check(S.Reap(K), "reap after exit");
    Check(!S.Reap(K) && !S.HoldsCodeLease(), "lease released once");
    Check(!S.Start() && !S.TryAdmit() && !S.SetPaused(false), "terminal cannot reactivate");
}

int main()
{
    // Ordered traces model both sides of stop/admission without starting threads.
    for (unsigned StopPhase = 0; StopPhase != 4; ++StopPhase)
    {
        TSessionLifecycle<2> S(1, Lease());
        const auto A = *S.RegisterParticipant();
        const auto B = *S.RegisterParticipant();
        Check(S.Start(), "start");
        Check(!S.MarkCommitted({}) && !S.MarkPublished({}) && !S.Complete({}) && !S.Abort({}), "empty reservation capability rejected");
        const auto T = *S.TryAdmit();
        Check(!S.TryAdmit(), "one pending reservation");
        if (StopPhase == 0) Check(S.RequestStop(), "stop reserved");
        Check(S.MarkCommitted(T), "commit admitted work even after stop");
        Check(!S.MarkCommitted(T), "duplicate commit");
        if (StopPhase == 1) Check(S.RequestStop(), "stop committed");
        Check(S.MarkPublished(T), "global publication witness");
        Check(!S.Abort(T), "global publication cannot abort");
        if (StopPhase == 2) Check(S.RequestStop(), "stop published");
        Check(S.Complete(T), "complete admitted published reservation");
        if (StopPhase == 3) Check(S.RequestStop(), "stop resolved");
        Check(!S.RequestStop() && !S.TryAdmit(), "duplicate stop and closed admission");
        Check(S.LastOutcome() && S.LastOutcome()->Outcome == EOutcome::Completed, "completed outcome persists");
        Check(!S.Complete(T) && !S.Abort(T) && !S.MarkPublished(T), "resolved token rejected");
        Shutdown(S, std::array<FParticipantHandle, 2>{A, B});
        Check(S.LastOutcome()->Outcome == EOutcome::Completed, "reaping preserves value outcome");
    }
    for (bool Committed : {false, true})
    {
        TSessionLifecycle<1> S(2, Lease());
        const auto P = *S.RegisterParticipant();
        Check(S.Start(), "abort fixture start");
        const auto T = *S.TryAdmit();
        if (Committed) Check(S.MarkCommitted(T), "application before abort");
        Check(S.RequestStop() && S.BeginStopDrain(S.GetKey()), "draining retains pending work");
        Check(S.BeginParticipantDrain(P), "retire participant in admitted frame");
        Check(!S.AcknowledgeDetached(P) && !S.AcknowledgeStopped(S.GetKey()), "pending prevents detached and stopped");
        Check(!S.Complete(T), "completion requires global publication");
        Check(S.Abort(T), "explicit pre-publication abort");
        Check(S.LastOutcome()->Outcome == EOutcome::Aborted && !S.TryAdmit(), "abort outcome is terminal");
        Retire(S, P);
        Check(S.AcknowledgeStopped(S.GetKey()), "aborted reservation resolved before stop");
        Check(S.AcknowledgeThreadExited(S.GetKey()) && S.Reap(S.GetKey()), "abort reap");
    }
    {
        TSessionLifecycle<1> S(3, Lease());
        const auto P = *S.RegisterParticipant();
        Check(S.Start() && S.SetPaused(true) && S.IsPaused(), "pause request");
        Check(!S.TryAdmit() && S.GetState() == ESessionState::Running, "pause is not stopped");
        Check(S.RequestStop() && !S.SetPaused(false), "stop while paused cannot resume");
        Shutdown(S, std::array<FParticipantHandle, 1>{P});
    }
    {
        TSessionLifecycle<1> S(4, Lease());
        const auto P = *S.RegisterParticipant();
        Check(S.Start() && S.RequestStop() && !S.TryAdmit(), "stop wins before admission");
        Shutdown(S, std::array<FParticipantHandle, 1>{P});
    }
    {
        TSessionLifecycle<1> S(5, Lease());
        const auto Old = *S.RegisterParticipant();
        Check(!S.RegisterParticipant(), "bounded registration overflow");
        Retire(S, Old); // No mailbox capacity is required for retirement.
        const auto Current = *S.RegisterParticipant();
        Check(Current.Generation != Old.Generation, "reused slot advances generation");
        Check(!S.RequestRetire(Old) && !S.AcknowledgeDetached(Old), "stale generation rejected");
        auto BadSlot = Current; BadSlot.Slot = 1;
        Check(!S.RequestRetire(BadSlot) && !S.ParticipantState(BadSlot), "out of bounds handle rejected");
        TSessionLifecycle<1> Foreign(5, Lease());
        auto ForeignHandle = Current; ForeignHandle.Session = Foreign.GetKey();
        Check(!S.RequestRetire(ForeignHandle) && !S.BeginStopDrain(Foreign.GetKey()), "same epoch foreign session rejected");
        TSessionLifecycle<1> OldEpoch(4, Lease());
        Check(!S.AcknowledgeThreadExited(OldEpoch.GetKey()), "stale epoch rejected");
        Check(S.Start() && !S.RegisterParticipant(), "configuration sealed on start");
        Shutdown(S, std::array<FParticipantHandle, 1>{Current});
    }
    {
        TSessionLifecycle<2> S(6, Lease());
        const auto A = *S.RegisterParticipant();
        const auto B = *S.RegisterParticipant();
        Check(S.Start(), "participant retirement fixture start");
        const auto T = *S.TryAdmit();
        Check(S.RequestRetire(A) && !S.RequestRetire(A), "idempotent request fail closed");
        Check(S.BeginParticipantDrain(A) && !S.AcknowledgeDetached(A), "admitted participant stays alive");
        Check(S.MarkCommitted(T) && S.MarkPublished(T) && S.Complete(T), "resolve before detach");
        Retire(S, A);
        const auto Next = *S.TryAdmit();
        Check(!S.MarkCommitted(T), "old reservation cannot commit next frame");
        Check(S.MarkCommitted(Next) && S.MarkPublished(Next) && S.Complete(Next), "remaining participant can advance");
        Check(S.RequestRetire(B) && !S.TryAdmit(), "retirement immediately excludes the last active participant");
        Shutdown(S, std::array<FParticipantHandle, 2>{A, B});
    }
    {
        auto OwnedLease = Lease();
        std::weak_ptr<const FCodeLease> Witness = OwnedLease;
        TSessionLifecycle<1> S(7, OwnedLease);
        OwnedLease.reset();
        Check(!Witness.expired() && S.Start(), "protocol retains sole code lease");
        Check(S.RequestStop() && S.BeginStopDrain(S.GetKey()), "empty session drain");
        Check(!Witness.expired() && S.AcknowledgeStopped(S.GetKey()), "lease before stop");
        Check(!Witness.expired() && S.AcknowledgeThreadExited(S.GetKey()), "lease before exit");
        Check(!Witness.expired() && S.Reap(S.GetKey()) && Witness.expired(), "lease lives through exit until reap");
    }
    {
        TSessionLifecycle<1> S(8, Lease());
        Check(S.RequestStop() && !S.Start(), "cancel configuration before start");
        Shutdown(S, std::array<FParticipantHandle, 0>{});
    }
    {
        TSessionLifecycle<64> S(9, Lease());
        std::array<FParticipantHandle, 64> P;
        for (auto& H : P) { const auto R = S.RegisterParticipant(); Check(bool(R), "fill all participant slots"); H = *R; }
        Check(!S.RegisterParticipant() && S.Start(), "full capacity starts without overflow");
        const auto T = *S.TryAdmit();
        Check(S.RequestRetire(P[63]) && S.BeginParticipantDrain(P[63]), "top mask bit retirement");
        Check(!S.AcknowledgeDetached(P[63]), "top mask bit keeps pending participant alive");
        Check(S.Abort(T), "abort closes admission without explicit stop");
        Check(S.GetState() == ESessionState::StopRequested, "abort requests terminal stop");
        Shutdown(S, P);
    }
    std::cout << "SessionLifecycle checks=" << Checks << " PASS\n";
}
