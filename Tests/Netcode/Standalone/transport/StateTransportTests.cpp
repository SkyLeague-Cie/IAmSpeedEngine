#include "StateFixture.h"
#include <iostream>
#include <stdexcept>
#include <string>
using namespace Speed::Netcode;
static int Checks = 0;
#define CHECK(E) do { ++Checks; if (!(E)) throw std::runtime_error(std::to_string(__LINE__) + " " #E); } while (false)
static FStateFragment WireCopy(const FStateFragment& F)
{ auto B = EncodeFragment(F); CHECK(B); auto D = DecodeFragment(B->data(), B->size()); CHECK(D); return *D; }
int main()
{
    try
    {
        const auto Binding = StateFixture::Binding();
        auto S = StateFixture::Snapshot(1); auto Bytes = EncodeState(S); CHECK(Bytes);
        CHECK(EncodeState(*DecodeState(Bytes->data(), Bytes->size())) == Bytes);
        for (std::size_t N = 0; N < Bytes->size(); ++N) CHECK(!DecodeState(Bytes->data(), N));
        auto F = FragmentState(S, Binding); CHECK(F && F->size() == 3);
        CHECK(EncodeFragment(F->front())->size() <= 1088);
        FStateReceiver Owner(Binding), Remote(Binding);
        CHECK(Owner.Receive(999, F->front()) == EResult::WrongPeer);
        auto Wrong = F->front(); ++Wrong.Epoch; CHECK(Owner.Receive(1, Wrong) == EResult::WrongStream);
        FStateOutbox ForOwner(Binding, 101), ForRemote(Binding, 202);
        CHECK(ForOwner.Queue(S) == EResult::Accepted && ForRemote.Queue(S) == EResult::Accepted);
        FReplicatedRoster OwnerRoster, RemoteRoster;
        for (U64 Sequence = 1; Sequence <= 3; ++Sequence)
        {
            if (Sequence > 1)
            {
                S = StateFixture::Snapshot(Sequence);
                CHECK(ForOwner.Queue(S) == EResult::Accepted && ForRemote.Queue(S) == EResult::Accepted);
            }
            F = FragmentState(S, Binding); CHECK(F);
            // Reorder, withhold middle fragment, duplicate first: no partial state.
            for (auto* R : {&Owner, &Remote})
            {
                CHECK(R->Receive(1, WireCopy(F->back())) == EResult::Accepted);
                CHECK(R->Receive(1, WireCopy(F->front())) == EResult::Accepted);
                CHECK(R->Receive(1, WireCopy(F->front())) == EResult::Duplicate);
                CHECK(!R->TakeReady());
                CHECK(R->Receive(1, WireCopy((*F)[1])) == EResult::Accepted);
            }
            auto A = Owner.TakeReady(), B = Remote.TakeReady(); CHECK(A && B);
            CHECK(EncodeState(*A) == EncodeState(S) && EncodeState(*B) == EncodeState(S));
            CHECK(OwnerRoster.Install(*A, StateFixture::ValidateActor)); CHECK(RemoteRoster.Install(*B, StateFixture::ValidateActor));
            // Lost final ACK: a retransmission is acknowledged without republish.
            CHECK(Owner.Receive(1, WireCopy(F->front())) == EResult::Duplicate); CHECK(!Owner.TakeReady());
            CHECK(!ForOwner.Acknowledge(202, *Owner.Acknowledgement()));
            CHECK(ForOwner.Acknowledge(101, WireCopy(*Owner.Acknowledgement())));
            CHECK(ForRemote.Acknowledge(202, WireCopy(*Remote.Acknowledgement())));
            CHECK(ForOwner.Empty() && ForRemote.Empty());
        }
        CHECK(OwnerRoster.Read()->Actors.back().Incarnation == 2);
        auto Before = EncodeState(*OwnerRoster.Read()); auto Invalid = StateFixture::Snapshot(3);
        Invalid.Sequence = 4; Invalid.Frame = 34; Invalid.RegistryGeneration = 4;
        Invalid.Actors.erase(Invalid.Actors.begin() + 1); // silently missing ball
        CHECK(!OwnerRoster.Install(Invalid, StateFixture::ValidateActor)); CHECK(EncodeState(*OwnerRoster.Read()) == Before);
        Invalid = StateFixture::Snapshot(3); Invalid.Sequence = 4; Invalid.Frame = 34; Invalid.Actors.back().Type = 99;
        CHECK(!OwnerRoster.Install(Invalid, StateFixture::ValidateActor)); CHECK(EncodeState(*OwnerRoster.Read()) == Before);
        FStateReceiver Corrupt(Binding); F = FragmentState(StateFixture::Snapshot(1), Binding);
        (*F)[1].Bytes[0] ^= 1;
        CHECK(Corrupt.Receive(1, (*F)[0]) == EResult::Accepted); CHECK(Corrupt.Receive(1, (*F)[2]) == EResult::Accepted);
        CHECK(Corrupt.Receive(1, (*F)[1]) == EResult::Invalid); CHECK(!Corrupt.TakeReady() && !Corrupt.Acknowledgement());
        auto TooBig = StateFixture::Snapshot(1); TooBig.Actors[0].Payload.resize(MaxActorState + 1); CHECK(!EncodeState(TooBig));
        auto Dup = StateFixture::Snapshot(1); Dup.Actors.push_back(Dup.Actors.back()); CHECK(!EncodeState(Dup));
        auto Physical = *DecodeCanonicalActor(StateFixture::Snapshot(1).Actors.front().Payload);
        Physical.Kinematic[0] = -0.0; Physical.Kinematic[1] = std::numeric_limits<double>::denorm_min();
        Physical.Kinematic[2] = std::numeric_limits<double>::max();
        auto Exact = EncodeCanonicalActor(Physical); CHECK(Exact);
        auto Decoded = DecodeCanonicalActor(*Exact); CHECK(Decoded && EncodeCanonicalActor(*Decoded) == Exact);
        CHECK(std::signbit(Decoded->Kinematic[0]));
        Physical.Kinematic[0] = std::numeric_limits<double>::quiet_NaN(); CHECK(!EncodeCanonicalActor(Physical));
        std::cout << "PASS state_checks=" << Checks << " snapshots=3 recipients=owner,remote actors=car,ball,pad lifecycle=destroy,respawn scope=synthetic_bytes_no_physics_no_UE\n";
        return 0;
    }
    catch (const std::exception& E) { std::cerr << "FAIL " << E.what() << '\n'; return 1; }
}
