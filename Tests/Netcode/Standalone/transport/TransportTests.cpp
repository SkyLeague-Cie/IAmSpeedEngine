#include "NetInputTransport.h"
#include "TransportFixture.h"
#include "RollbackIslandPlan.h"
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace Speed::Netcode;
static int Checks = 0, Tests = 0;
#define CHECK(E) do { ++Checks; if (!(E)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + " " #E); } while (false)
static FBinding Binding()
{
    FBinding B; B.Session = 11; B.Epoch = 7; B.Actor = 23; B.Incarnation = 2;
    B.OwnerPeer = 101; B.ServerPeer = 1; B.SourceProducer = 99; B.Contract[0] = 42;
    return B;
}
static bool ValidateGame(const FFrame& I)
{
    for (std::size_t N = 3; N < ActionCount; ++N) if (I.Values[N] < 0 || I.Values[N] > 1) return false;
    for (const auto& E : I.Edges) if (E.Action >= 3 && (E.Value < 0 || E.Value > 1)) return false;
    return true;
}
static FPacket Packet(U64 N, EKind Kind = EKind::OwnerInput)
{
    auto B = Binding(); FPacket P; P.Kind = Kind; P.Session = B.Session; P.Epoch = B.Epoch;
    P.Actor = B.Actor; P.Incarnation = B.Incarnation; P.Contract = B.Contract;
    P.Frame = N; P.Sequence = N + 1; P.Input.SourceSequence = N + 1;
    P.Input.DeviceGeneration = 1; P.Input.ProducerId = B.SourceProducer;
    P.Input.Reset = N == 0;
    return P;
}
static FPacket AcrossWire(const FPacket& P)
{
    auto B = Encode(P); CHECK(B); auto D = Decode(B->data(), B->size()); CHECK(D); return *D;
}
static void Codec()
{
    auto P = Packet(0); P.Input.Values[2] = -127;
    auto B = Encode(P); CHECK(B && B->size() == BaseBytes);
    CHECK((*B)[0] == 'I' && (*B)[1] == 'S' && (*B)[2] == 'N' && (*B)[3] == '1');
    CHECK((*B)[12] == 11 && (*B)[20] == 7); // stable LE field offsets
    CHECK(Same(P, AcrossWire(P)));
    auto Ack = Encode(AckFor(P)); CHECK(Ack && Ack->size() == HeaderBytes);
    CHECK(AcrossWire(AckFor(P)).Kind == EKind::Ack);
    for (std::size_t N = 0; N < B->size(); ++N) CHECK(!Decode(B->data(), N));
    auto Extra = *B; Extra.push_back(0); CHECK(!Decode(Extra.data(), Extra.size()));
    for (std::size_t At : {std::size_t(0), std::size_t(4), std::size_t(6), std::size_t(7), std::size_t(10), std::size_t(117), std::size_t(124)})
    {
        auto Bad = *B; Bad[At] = 255; CHECK(!Decode(Bad.data(), Bad.size()));
    }
    auto Many = Packet(1); Many.Input.ActiveMask = 0;
    for (std::size_t N = 0; N < MaxEdges; ++N)
        Many.Input.Edges.push_back({3, std::uint8_t(N % 2), std::int16_t(N % 2 ? 0 : 1), 2, std::uint16_t(N)});
    CHECK(Encode(Many)->size() == MaxDatagram); CHECK(Same(Many, AcrossWire(Many)));
    Many.Input.Edges.push_back({3, 0, 1, 2, 64}); CHECK(!Encode(Many));
}
static void RolesAndIdentity()
{
    auto B = Binding(); FExactInbox Server(B, EKind::OwnerInput, 0, ValidateGame);
    auto P = Packet(0);
    CHECK(Server.Receive(999, P) == EResult::WrongPeer);
    for (int Field = 0; Field < 8; ++Field)
    {
        auto Wrong = P;
        switch (Field) { case 0: ++Wrong.Session; break; case 1: ++Wrong.Epoch; break;
        case 2: ++Wrong.Actor; break; case 3: ++Wrong.Incarnation; break; case 4: ++Wrong.Contract[1]; break;
        case 5: ++Wrong.Input.ProducerId; break; case 6: Wrong.Kind = EKind::AuthorityInput; break;
        case 7: Wrong.Input.ProducerKind = 1; break; }
        CHECK(Server.Receive(B.OwnerPeer, Wrong) == EResult::WrongStream);
    }
    CHECK(Server.Receive(B.OwnerPeer, P) == EResult::Accepted);
    FExactInbox Remote(B, EKind::AuthorityInput, 0, ValidateGame);
    P.Kind = EKind::AuthorityInput;
    CHECK(Remote.Receive(B.OwnerPeer, P) == EResult::WrongPeer);
    CHECK(Remote.Receive(B.ServerPeer, P) == EResult::Accepted);
    auto R = Remote.Consume(0); CHECK(R && R->FirstConsume);
    FBinding Invalid = B; Invalid.ServerPeer = Invalid.OwnerPeer;
    FExactInbox Refused(Invalid, EKind::OwnerInput, 0, ValidateGame);
    CHECK(Refused.Receive(B.OwnerPeer, Packet(0)) == EResult::Invalid);
}
static void FramesEdgesAndAbsence()
{
    auto B = Binding(); FExactInbox Inbox(B, EKind::OwnerInput, 0, ValidateGame);
    CHECK(!Inbox.Consume(0)); CHECK(Inbox.NextFrame() == 0);
    auto P2 = Packet(2); P2.Input.Values[3] = 0;
    P2.Input.Edges = {{0, 1, 0, 3, 0}, {3, 0, 1, 3, 1}, {3, 1, 0, 3, 2}};
    auto P1 = Packet(1); P1.Input.Values[0] = 255; P1.Input.ActiveMask = 1;
    P1.Input.Edges = {{0, 0, 255, 2, 0}};
    CHECK(Inbox.Receive(B.OwnerPeer, AcrossWire(P2)) == EResult::Accepted);
    CHECK(!Inbox.Consume(1));
    CHECK(Inbox.Receive(B.OwnerPeer, Packet(0)) == EResult::Accepted);
    CHECK(Inbox.Receive(B.OwnerPeer, AcrossWire(P1)) == EResult::Accepted);
    CHECK(Inbox.Receive(B.OwnerPeer, P1) == EResult::Duplicate);
    auto Conflict = P1; Conflict.Input.Values[0] = 42;
    CHECK(Inbox.Receive(B.OwnerPeer, Conflict) == EResult::Conflict);
    auto First = Inbox.Consume(0); CHECK(First && First->FirstConsume && First->Packet.Input.Reset);
    auto Held = Inbox.Consume(1); CHECK(Held && Held->Packet.Input.Values[0] == 255);
    auto Edges = Inbox.Consume(2); CHECK(Edges && Edges->Packet.Input.Edges.size() == 3);
    auto Again = Inbox.Consume(2); CHECK(Again && !Again->FirstConsume);
    CHECK(Inbox.Receive(B.OwnerPeer, P2) == EResult::Duplicate);
    CHECK(!Inbox.Consume(3)); // do not repeat held throttle or release implicitly
    auto NoBaseline = Packet(0); NoBaseline.Input.Reset = false;
    FExactInbox MissingBaseline(B, EKind::OwnerInput, 0, ValidateGame);
    CHECK(MissingBaseline.Receive(B.OwnerPeer, NoBaseline) == EResult::Accepted);
    CHECK(!MissingBaseline.Consume(0));
    CHECK(MissingBaseline.RequiresResync());
}
static void ContinuityAndBounds()
{
    auto B = Binding(); FExactInbox I(B, EKind::OwnerInput, 0, ValidateGame);
    CHECK(I.Receive(B.OwnerPeer, Packet(Window)) == EResult::TooFar);
    CHECK(I.Receive(B.OwnerPeer, Packet(0)) == EResult::Accepted); CHECK(I.Consume(0));
    auto Bad = Packet(1); Bad.Input.DeviceGeneration = 2;
    CHECK(I.Receive(B.OwnerPeer, Bad) == EResult::Accepted); CHECK(!I.Consume(1));
    // Invalid continuity quarantines this stream; there is no packet replacement.
    auto Fixed = Bad; Fixed.Input.Reset = true;
    CHECK(I.RequiresResync()); CHECK(I.Receive(B.OwnerPeer, Fixed) == EResult::Invalid);
    FExactInbox Reset(B, EKind::OwnerInput, 0, ValidateGame);
    CHECK(Reset.Receive(B.OwnerPeer, Packet(0)) == EResult::Accepted); CHECK(Reset.Consume(0));
    CHECK(Reset.Receive(B.OwnerPeer, Fixed) == EResult::Accepted); CHECK(Reset.Consume(1));
    FExactInbox SharedSource(B, EKind::OwnerInput, 0, ValidateGame);
    CHECK(SharedSource.Receive(B.OwnerPeer, Packet(0)) == EResult::Accepted); CHECK(SharedSource.Consume(0));
    auto Held = Packet(1); Held.Input.SourceSequence = 1;
    CHECK(SharedSource.Receive(B.OwnerPeer, Held) == EResult::Accepted); CHECK(SharedSource.Consume(1));
    auto Replay = Packet(2); Replay.Input.SourceSequence = 1; Replay.Input.Edges = {{3, 0, 1, 1, 0}};
    Replay.Input.ActiveMask = 8;
    CHECK(SharedSource.Receive(B.OwnerPeer, Replay) == EResult::Accepted); CHECK(!SharedSource.Consume(2));
    FExactInbox Long(B, EKind::OwnerInput, 0, ValidateGame);
    for (U64 N = 0; N < 2 * Window + 3; ++N)
    { CHECK(Long.Receive(B.OwnerPeer, Packet(N)) == EResult::Accepted); CHECK(Long.Consume(N)); }
    CHECK(Long.Receive(B.OwnerPeer, Packet(0)) == EResult::Stale);
    auto Max = Packet(0); Max.Frame = std::numeric_limits<U64>::max();
    FExactInbox End(B, EKind::OwnerInput, Max.Frame, ValidateGame);
    CHECK(End.Receive(B.OwnerPeer, Max) == EResult::Accepted); CHECK(End.Consume(Max.Frame));
    CHECK(!End.Consume(0));
    FExactInbox Validator(B, EKind::OwnerInput, 0, ValidateGame);
    auto Range = Packet(0); Range.Input.Values[3] = 2;
    CHECK(Validator.Receive(B.OwnerPeer, Range) == EResult::Invalid);
}
static void Reliability()
{
    auto B = Binding(); FReliableOutbox Out(B, B.ServerPeer);
    for (U64 N = 0; N < Window; ++N) CHECK(Out.Queue(Packet(N)) == EResult::Accepted);
    CHECK(Out.Size() == Window); CHECK(Out.Queue(Packet(Window)) == EResult::Full);
    CHECK(Out.Queue(Packet(0)) == EResult::Duplicate);
    CHECK(!Out.Acknowledge(999, AckFor(Packet(0))));
    auto Wrong = AckFor(Packet(0)); ++Wrong.Frame;
    CHECK(!Out.Acknowledge(B.ServerPeer, Wrong));
    CHECK(Out.Acknowledge(B.ServerPeer, AcrossWire(AckFor(Packet(0)))));
    CHECK(!Out.Acknowledge(B.ServerPeer, AckFor(Packet(0))));
    CHECK(Out.Queue(Packet(Window)) == EResult::Accepted);
    CHECK(Out.Queue(Packet(0)) == EResult::Stale); // ACK cannot enable sequence reuse
}
static void EndToEndImpairments()
{
    auto B = Binding();
    FExactInbox Server(B, EKind::OwnerInput, 0, ValidateGame), Remote(B, EKind::AuthorityInput, 0, ValidateGame);
    FReliableOutbox Owner(B, B.ServerPeer), Relay(B, 202);
    constexpr U64 Count = 40;
    std::vector<FPacket> Expected;
    for (U64 N = 0; N < Count; ++N)
    {
        auto P = Packet(N); P.Input.Values[0] = std::int16_t(N % 256);
        P.Input.Values[1] = std::int16_t((N * 3) % 256); P.Input.Values[2] = std::int16_t(int(N) - 20);
        P.Input.ActiveMask = 7; // zero values may still be explicitly active
        if (N && N % 5 == 0) P.Input.Edges = {{3, 0, 1, N + 1, 0}, {3, 1, 0, N + 1, 1}};
        Expected.push_back(P); CHECK(Owner.Queue(P) == EResult::Accepted);
    }
    U64 ServerCount = 0, RemoteCount = 0, EdgeCount = 0;
    for (unsigned Tick = 0; Tick < 40; ++Tick)
    {
        std::vector<FPacket> Batch; Owner.Retry([&](auto P) { Batch.push_back(P); });
        for (auto It = Batch.rbegin(); It != Batch.rend(); ++It) // reorder
        {
            if ((It->Sequence + Tick) % 3 == 0) continue; // loss
            auto P = AcrossWire(*It); auto R = Server.Receive(B.OwnerPeer, P);
            CHECK(R == EResult::Accepted || R == EResult::Duplicate);
            CHECK(Server.Receive(B.OwnerPeer, P) == EResult::Duplicate); // duplication
            if ((P.Sequence + Tick) % 4) Owner.Acknowledge(B.ServerPeer, AckFor(P)); // ACK loss
        }
        while (ServerCount < Count)
        {
            auto R = Server.Consume(ServerCount); if (!R) break;
            CHECK(R->FirstConsume); CHECK(Same(R->Packet, Expected[ServerCount]));
            auto P = R->Packet; P.Kind = EKind::AuthorityInput;
            CHECK(Relay.Queue(P) == EResult::Accepted); ++ServerCount;
        }
        Batch.clear(); Relay.Retry([&](auto P) { Batch.push_back(P); });
        for (auto It = Batch.rbegin(); It != Batch.rend(); ++It)
        {
            if ((It->Sequence + Tick) % 4 == 0) continue;
            auto P = AcrossWire(*It); auto R = Remote.Receive(B.ServerPeer, P);
            CHECK(R == EResult::Accepted || R == EResult::Duplicate);
            if ((P.Sequence + Tick) % 5) Relay.Acknowledge(202, AckFor(P));
        }
        while (RemoteCount < Count)
        {
            auto R = Remote.Consume(RemoteCount); if (!R) break;
            CHECK(R->FirstConsume); auto E = Expected[RemoteCount]; E.Kind = EKind::AuthorityInput;
            CHECK(Same(R->Packet, E)); EdgeCount += R->Packet.Input.Edges.size(); ++RemoteCount;
        }
        if (ServerCount == Count && RemoteCount == Count && !Owner.Size() && !Relay.Size()) break;
    }
    CHECK(ServerCount == Count && RemoteCount == Count && Owner.Size() == 0 && Relay.Size() == 0);
    CHECK(EdgeCount == 14);
    std::cout << "SIMULATED_TRANSPORT frames=" << Count << " edges=" << EdgeCount << " loss/reorder/duplicate/ack_loss=covered\n";
}
static void VersionTwoCompatibility()
{
    const auto Contract = Fixture::Contract(); CHECK(Contract);
    for (U64 N = 0; N < 32; ++N)
    {
        auto P = Fixture::Packet(N); auto Frame = ToV2(AcrossWire(P), Contract); CHECK(Frame);
        CHECK(Frame->GetData().ActiveMask == P.Input.ActiveMask);
        auto Back = FromV2(*Frame, Fixture::Binding(), P.Kind, P.Sequence); CHECK(Back && Same(*Back, P));
    }
    auto LargeOrder = Fixture::Packet(5);
    LargeOrder.Input.Edges[0].Ordinal = 0xFFFFFFFEu;
    LargeOrder.Input.Edges[1].Ordinal = 0xFFFFFFFFu;
    auto Frame = ToV2(AcrossWire(LargeOrder), Contract); CHECK(Frame);
    auto Back = FromV2(*Frame, Fixture::Binding(), LargeOrder.Kind, LargeOrder.Sequence);
    CHECK(Back && Same(*Back, LargeOrder)); // exact 32-bit within-source order
    auto Unwired = Fixture::Packet(1); Unwired.Input.Values[31] = 1;
    CHECK(!ToV2(Unwired, Contract));
    auto InvalidBool = Fixture::Packet(1); InvalidBool.Input.Values[3] = 2;
    CHECK(!ToV2(InvalidBool, Contract));
    FExactInbox Inbox(Fixture::Binding(), EKind::OwnerInput, 0, Fixture::ValidateGame);
    auto Baseline = Fixture::Packet(0); CHECK(Inbox.Receive(101, Baseline) == EResult::Accepted); CHECK(Inbox.Consume(0));
    auto MissingEdges = Fixture::Packet(1); MissingEdges.Input.Edges.clear();
    CHECK(Inbox.Receive(101, MissingEdges) == EResult::Accepted); CHECK(!Inbox.Consume(1)); CHECK(Inbox.RequiresResync());
}
static void CausalIslandPlanning()
{
    std::vector<FRetainedActor> Actors;
    for (U64 Id = 1; Id <= 7; ++Id) Actors.push_back({Id, 1, 10, 11, 20, true, true, true, true});
    // Reverse enumeration deliberately requires fixed-point transitive closure.
    std::vector<FCausalDependency> Dependencies = {{3, 7, 14, 18}, {2, 3, 11, 16}, {1, 2, 12, 12}, {4, 5, 11, 20}};
    auto P = PlanRollbackIsland(Actors, Dependencies, {1}, 10, 20, 4, true);
    CHECK(P.Result == EPlanResult::Ready); CHECK(P.Actors == std::vector<U64>({1, 2, 3, 7}));
    auto Unknown = PlanRollbackIsland(Actors, Dependencies, {1}, 10, 20, 4, false);
    CHECK(Unknown.Result == EPlanResult::NeedFullWorldCoverage && Unknown.Actors.size() == 7);
    auto Missing = Actors; Missing[6].ContactState = false;
    CHECK(PlanRollbackIsland(Missing, Dependencies, {1}, 10, 20, 4, true).Result == EPlanResult::MissingHistory);
    CHECK(PlanRollbackIsland(Actors, Dependencies, {1}, 10, 20, 4, true, 3).Result == EPlanResult::BudgetExceeded);
    CHECK(PlanRollbackIsland(Actors, Dependencies, {1}, 10, 20, 4, true, 7, 9).Result == EPlanResult::BudgetExceeded);
    auto Bad = Dependencies; Bad.push_back({1, 99, 11, 20});
    CHECK(PlanRollbackIsland(Actors, Bad, {1}, 10, 20, 4, true).Result == EPlanResult::Invalid);
    auto Outside = Dependencies; Outside.push_back({1, 4, 21, 25});
    CHECK(PlanRollbackIsland(Actors, Outside, {1}, 10, 20, 4, true).Actors == P.Actors);
    // A new swept candidate discovered during a trial replay expands/restarts
    // from frame10; this planning fixture does not certify a real broadphase.
    Dependencies.push_back({7, 4, 18, 19});
    P = PlanRollbackIsland(Actors, Dependencies, {1}, 10, 20, 4, true);
    CHECK(P.Result == EPlanResult::Ready && P.Actors == std::vector<U64>({1, 2, 3, 4, 5, 7}));
    Missing = Actors; Missing[4].EventLedger = false;
    CHECK(PlanRollbackIsland(Missing, Dependencies, {1}, 10, 20, 4, true).Result == EPlanResult::MissingHistory);
    CHECK(PlanRollbackIsland(Actors, Dependencies, {1}, 20, 10, 4, true).Result == EPlanResult::Invalid);
}
static void DecoderMutationAndSequenceReuse()
{
    auto P = Fixture::Packet(5); auto Bytes = Encode(P); CHECK(Bytes);
    for (std::size_t I = 0; I < Bytes->size(); ++I)
        for (unsigned Bit = 0; Bit < 8; ++Bit)
        {
            auto Mutated = *Bytes; Mutated[I] ^= std::uint8_t(1u << Bit);
            auto Parsed = Decode(Mutated.data(), Mutated.size());
            if (Parsed) CHECK(Encode(*Parsed) == std::optional<std::vector<std::uint8_t>>(Mutated));
            else CHECK(!Parsed);
        }
    auto B = Binding(); FExactInbox Inbox(B, EKind::OwnerInput, 0, ValidateGame);
    CHECK(Inbox.Receive(B.OwnerPeer, Packet(0)) == EResult::Accepted); CHECK(Inbox.Consume(0));
    auto Reuse = Packet(1); Reuse.Sequence = 1;
    CHECK(Inbox.Receive(B.OwnerPeer, Reuse) == EResult::Conflict);
    auto P1 = Packet(1); CHECK(Inbox.Receive(B.OwnerPeer, P1) == EResult::Accepted); CHECK(Inbox.Consume(1));
    auto ReversedSource = Packet(2); ReversedSource.Input.SourceSequence = 1;
    CHECK(Inbox.Receive(B.OwnerPeer, ReversedSource) == EResult::Accepted); CHECK(!Inbox.Consume(2));
    CHECK(Inbox.RequiresResync());
}
int main()
{
    try
    {
        for (auto T : {Codec, RolesAndIdentity, FramesEdgesAndAbsence, ContinuityAndBounds, Reliability, EndToEndImpairments, VersionTwoCompatibility, CausalIslandPlanning, DecoderMutationAndSequenceReuse})
        { T(); ++Tests; }
        std::cout << "PASS tests=" << Tests << " checks=" << Checks << " scope=pure_cpp_no_physics_no_UE\n"; return 0;
    }
    catch (const std::exception& E) { std::cerr << "FAIL tests_completed=" << Tests << " checks=" << Checks << " " << E.what() << '\n'; return 1; }
}
