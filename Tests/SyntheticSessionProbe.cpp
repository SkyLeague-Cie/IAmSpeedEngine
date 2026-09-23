#include "Support/SyntheticSessionService.h"
#include <cstdlib>
#include <iostream>

using namespace Speed::Lifecycle;
using namespace Speed::Lifecycle::Testing;
using FService = TSyntheticSessionService<2>;
static unsigned Checks = 0;
static void Check(bool Value, const char* Message)
{
    ++Checks;
    if (!Value) { std::cerr << "FAIL " << Message << '\n'; std::exit(1); }
}
static std::shared_ptr<const FCodeLease> Lease()
{ return std::make_shared<const FCodeLease>(FCodeLease{23}); }

static void StopAndReap(FService& Service, const FSessionAddress& Address)
{
    Service.RequestStop(Address);
    if (Service.Observe(Address)->Session == ESessionState::StopRequested)
        Check(Service.BeginDrain(Address), "begin requested drain");
    Check(!Service.ReturnFakeLane(Address) && Service.ReapReady() == 0, "draining cannot exit or reap");
    if (Service.Observe(Address)->Participant != EParticipantState::Detached)
        Check(Service.Detach(Address), "resolved participant detaches");
    Check(!Service.Detach(Address), "duplicate detach rejected");
    Check(Service.StopAtBoundary(Address), "all work resolved before stopped");
    Check(Service.Observe(Address)->Session == ESessionState::Stopped &&
        Service.Observe(Address)->HoldsLease && Service.ReapReady() == 0, "stopped is not exited or reaped");
    Check(Service.ReturnFakeLane(Address), "explicit fake lane return");
    Check(!Service.ReturnFakeLane(Address) && Service.Observe(Address)->HoldsLease, "duplicate exit rejected, lease retained");
    Check(Service.ReapReady() == 1 && Service.ReapReady() == 0, "one reaping only");
    Check(!Service.Observe(Address), "stale address after reap");
}

int main()
{
    // Client is actually destroyed at each explicit barrier; the service keeps
    // the frame and participant alive. No actual thread blocks at these points.
    for (unsigned Barrier = 0; Barrier != 5; ++Barrier)
    {
        FService Service;
        const auto Trace = std::make_shared<FTrace>();
        auto Code = Lease();
        const std::weak_ptr<const FCodeLease> CodeWitness = Code;
        auto Client = Service.Create(Trace, Code);
        Check(bool(Client), "create lifetime fixture");
        Code.reset();
        const auto Address = Client->GetAddress();
        const auto Witness = Client->LifetimeWitness();
        Check(Client->Submit(37), "submit scalar command");
        std::optional<FReservationToken> Token;
        if (Barrier >= 1) { Token = Service.Admit(Address); Check(bool(Token), "admit frame"); }
        if (Barrier >= 2) Check(Service.Apply(Address, *Token), "apply before client destruction");
        if (Barrier >= 3) Check(Service.Publish(Address, *Token), "publish before client destruction");
        if (Barrier >= 4) Check(Service.Complete(Address, *Token), "complete before client destruction");
        Client.reset();
        Check(Trace->ClientsDestroyed == 1 && Trace->SessionsDestroyed == 0 &&
            Trace->ParticipantsDestroyed == 0 && !Witness.expired() && !CodeWitness.expired(), "client cannot destroy retained frame state");
        Check(Service.Observe(Address)->Session == ESessionState::StopRequested &&
            !Service.Observe(Address)->ClientOpen && !Service.Admit(Address), "client destruction closes admission");
        Check(Service.BeginDrain(Address), "drain after client destruction");
        if (Barrier >= 1 && Barrier < 4)
        {
            Check(!Service.Detach(Address) && !Service.StopAtBoundary(Address) &&
                !Service.ReturnFakeLane(Address) && Service.ReapReady() == 0, "held frame prevents premature lifetime release");
            if (Barrier == 1) Check(Service.Apply(Address, *Token), "apply retained reservation");
            if (Barrier <= 2) Check(Service.Publish(Address, *Token), "publish retained reservation");
            Check(!Service.Abort(Address, *Token), "published retained frame cannot abort");
            Check(Service.Complete(Address, *Token), "resolve retained frame");
        }
        Check(Trace->Applied == (Barrier ? 1u : 0u) && Trace->Published == Trace->Applied &&
            Trace->Completed == Trace->Applied && Trace->Aborted == 0, "exactly once value application and publication");
        Check(Service.Observe(Address)->PublishedValue == (Barrier ? 37u : 0u), "retained frame preserves scalar output");
        StopAndReap(Service, Address);
        Check(Witness.expired() && CodeWitness.expired() && Trace->SessionsDestroyed == 1 &&
            Trace->ParticipantsDestroyed == 1 && Service.RetainedSessions() == 0, "lifetimes end only after reap");
    }
    for (bool ApplyBeforeAbort : {false, true})
    {
        FService Service;
        const auto Trace = std::make_shared<FTrace>();
        auto Client = Service.Create(Trace, Lease());
        const auto A = Client->GetAddress();
        Check(Client->Submit(10), "baseline input");
        const auto First = *Service.Admit(A);
        Check(Service.Apply(A, First) && Service.Publish(A, First) && Service.Complete(A, First), "complete baseline");
        Check(Client->Submit(99), "next input");
        const auto Second = *Service.Admit(A);
        if (ApplyBeforeAbort) Check(Service.Apply(A, Second), "abort after modeled application");
        Check(Service.Abort(A, Second) && !Service.Abort(A, Second), "explicit abort once");
        Check(Service.Observe(A)->PublishedValue == 10 && Service.Observe(A)->PublicationSerial == 1 &&
            Trace->Published == 1 && Trace->Completed == 1 && Trace->Aborted == 1, "abort preserves last visible snapshot");
        Check(!Service.Admit(A) && !Client->Submit(100) && !Service.Publish(A, Second), "aborted session never advances");
        Client.reset();
        StopAndReap(Service, A);
    }
    {
        FService Service;
        const auto Trace = std::make_shared<FTrace>();
        auto Client = Service.Create(Trace, Lease());
        const auto A = Client->GetAddress();
        Check(Service.SetPaused(A, true) && !Service.Admit(A), "pause closes frame admission");
        Client.reset();
        Check(Service.Observe(A)->Paused && !Service.SetPaused(A, false), "paused stop cannot resume");
        StopAndReap(Service, A);
        Check(Trace->Applied == 0 && Trace->SessionsDestroyed == 1, "paused stop needs no pacing tick");
    }
    {
        FService Service;
        const auto T = std::make_shared<FTrace>();
        auto First = Service.Create(T, Lease());
        auto Second = Service.Create(T, Lease());
        const auto A = First->GetAddress(), B = Second->GetAddress();
        const auto WA = First->LifetimeWitness(), WB = Second->LifetimeWitness();
        Check(!Service.Create(T, Lease()) && T->SessionsCreated == 2 && T->ParticipantsCreated == 2, "capacity rejects before activation");
        const auto Held = *Service.Admit(A);
        First.reset(); Second.reset();
        StopAndReap(Service, B);
        Check(!WA.expired() && WB.expired() && Service.RetainedSessions() == 1 &&
            T->SessionsDestroyed == 1, "held first slot does not starve exited second slot");
        Check(Service.Abort(A, Held), "held slot still owns abort authority");
        StopAndReap(Service, A);
        Check(WA.expired() && T->SessionsDestroyed == 2 && T->ParticipantsDestroyed == 2, "both slots retired independently");
    }
    {
        FService Service, ForeignService;
        const auto T = std::make_shared<FTrace>();
        auto Client = Service.Create(T, Lease());
        auto Foreign = ForeignService.Create(T, Lease());
        const auto A = Client->GetAddress(), F = Foreign->GetAddress();
        const auto Token = *Service.Admit(A);
        const auto ForeignToken = *ForeignService.Admit(F);
        Check(!Service.Observe(F) && !Service.Apply(A, ForeignToken), "same numeric epoch from foreign service rejected");
        auto BadGeneration = A; ++BadGeneration.Participant.Generation;
        auto BadEpoch = A; BadEpoch.Participant.Session = F.Participant.Session;
        auto BadSlot = A; BadSlot.Slot = 2;
        Check(!Service.RequestStop(BadGeneration) && !Service.RequestStop(BadEpoch) &&
            !Service.RequestStop(BadSlot), "stale generation and identity cannot stop live session");
        Check(!Service.Publish(A, Token) && !Service.Complete(A, Token) && T->Applied == 0 &&
            T->Published == 0, "wrong phase cannot publish or complete");
        Check(Service.Apply(A, Token) && !Service.Apply(A, Token), "application once");
        Check(Service.Publish(A, Token) && !Service.Publish(A, Token) && !Service.Abort(A, Token), "publication once and no late abort");
        Check(Service.Complete(A, Token) && !Service.Complete(A, Token), "completion once");
        Check(ForeignService.Abort(F, ForeignToken), "resolve foreign fixture");
        Client.reset(); Foreign.reset();
        StopAndReap(Service, A); StopAndReap(ForeignService, F);
        auto Replacement = Service.Create(T, Lease());
        const auto R = Replacement->GetAddress();
        Check(R.Generation != A.Generation && R.Participant.Session.GetEpoch() != A.Participant.Session.GetEpoch(), "replacement gets fresh slot generation and epoch");
        const auto NewToken = *Service.Admit(R);
        Check(!Service.RequestStop(A) && !Service.Apply(R, Token), "old endpoint and token cannot mutate replacement");
        Check(Service.Abort(R, NewToken), "replacement abort");
        Replacement.reset(); StopAndReap(Service, R);
    }
    {
        FService Service;
        const auto T = std::make_shared<FTrace>();
        auto Client = Service.Create(T, Lease());
        const auto A = Client->GetAddress();
        Check(Client->Submit(7), "first captured command");
        const auto Token = *Service.Admit(A);
        Check(Client->Submit(88), "next command while frame held");
        Check(Service.Apply(A, Token) && Service.Publish(A, Token) && Service.Complete(A, Token), "complete captured frame");
        Check(Service.Observe(A)->PublishedValue == 7, "later client write cannot alter admitted frame");
        const auto Next = *Service.Admit(A);
        Check(!Service.Apply(A, Token) && Service.Apply(A, Next) && Service.Publish(A, Next) &&
            Service.Complete(A, Next) && Service.Observe(A)->PublishedValue == 88, "new command belongs to next frame only");
        Client.reset(); StopAndReap(Service, A);
    }
    {
        FService Service;
        const auto T = std::make_shared<FTrace>();
        auto First = Service.Create(T, Lease());
        auto Second = Service.Create(T, Lease());
        const auto A = First->GetAddress(), B = Second->GetAddress();
        *First = std::move(*Second);
        Check(T->ClientsCreated == 2 && T->ClientsDestroyed == 1 && !Second->Submit(2), "move assignment closes old client exactly once");
        Second.reset();
        Check(T->ClientsDestroyed == 1 && First->Submit(4), "moved-from destruction does not close destination");
        First.reset();
        Check(T->ClientsDestroyed == 2, "destination eventually closes once");
        StopAndReap(Service, A); StopAndReap(Service, B);
    }
    {
        FService Service;
        const auto T = std::make_shared<FTrace>();
        auto Client = Service.Create(T, Lease(), false);
        const auto A = Client->GetAddress();
        Check(Service.Observe(A)->Lane == ELaneState::NeverStarted && !Service.Admit(A), "unstarted lane has no frame");
        Client.reset(); StopAndReap(Service, A);
        Check(T->Applied == 0 && T->Published == 0 && T->SessionsDestroyed == 1, "unstarted cancellation reaps without fictitious work");
    }
    std::cout << "SyntheticSession checks=" << Checks << " PASS\n";
}
