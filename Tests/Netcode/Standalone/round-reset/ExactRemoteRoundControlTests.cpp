#include "IAmSpeed/Netcode/round-reset/ExactRemoteRoundControl.h"
#include "IAmSpeed/World/Simulation/ExactRoundConfiguration.h"
#include "IAmSpeed/World/Simulation/ExactRoundNativeAddress.h"
#include <cstdlib>
#include <iostream>
#include <limits>

namespace N = Speed::Netcode;
namespace R = Speed::Netcode::RoundControl;
using R::EResult;
using namespace std::chrono_literals;

static unsigned Checks = 0;
#define CHECK(...) do { ++Checks; if (!(__VA_ARGS__)) { \
    std::cerr << "line " << __LINE__ << ": " << #__VA_ARGS__ << '\n'; std::exit(1); } } while (false)

static const R::FKey Key{5, 6, 20, 1, 2, 400};
static const R::FPublication Step{200, 201, 202, 203};
static const auto Now = R::FClock::time_point{} + 10s;

template<class T>
static T Require(std::optional<T> Value)
{
    CHECK(Value);
    return std::move(*Value);
}

static N::FBinding Binding(N::U64 Index)
{
    N::FBinding B;
    B.Session = 10 + Index;
    B.Epoch = 20 + Index;
    B.Actor = 30 + Index;
    B.Incarnation = 40 + Index;
    B.OwnerPeer = 50 + Index;
    B.ServerPeer = 60;
    B.SourceProducer = 70 + Index;
    B.Contract.front() = static_cast<std::uint8_t>(Index + 1);
    CHECK(B.Valid());
    return B;
}

static N::FPacket Capture(const N::FBinding& B)
{
    N::FPacket P;
    P.Session = B.Session;
    P.Epoch = B.Epoch;
    P.Actor = B.Actor;
    P.Incarnation = B.Incarnation;
    P.Contract = B.Contract;
    P.Sequence = 1;
    P.Frame = Key.Frame;
    P.Input.ProducerId = B.SourceProducer;
    P.Input.ProducerKind = B.SourceKind;
    P.Input.SourceSequence = 420;
    P.Input.DeviceGeneration = 2;
    P.Input.Reset = true;
    P.Input.ActiveMask = 7;
    P.Input.Values[0] = 123;
    P.Input.Values[1] = 17;
    P.Input.Values[2] = -38;
    CHECK(N::Valid(P));
    return P;
}

static R::FMessage Message(const N::FBinding& B, R::FPayload Payload, const R::FKey& Round = Key)
{ return {Round, R::FSessionAddress::From(B), std::move(Payload)}; }

static R::FMessage RoundTrip(const R::FMessage& M)
{
    const auto Wire = R::Encode(M);
    CHECK(Wire);
    const auto Decoded = R::Decode(Wire->data(), Wire->size());
    CHECK(Decoded && R::Kind(*Decoded) == R::Kind(M) && R::Encode(*Decoded) == Wire);
    return *Decoded;
}

static void TestWire()
{
    const auto B = Binding(0);
    const auto P = Capture(B);
    const R::FPayload Cases[]{R::FReserve{5000}, R::FReservationResult{false}, R::FReservationResult{true},
        R::FBaseline{P}, R::FPublished{Step}, R::FApplied{Step}, R::FReleased{Step}};
    for (const auto& Payload : Cases)
    {
        const auto M = Message(B, Payload);
        const auto Decoded = RoundTrip(M);
        CHECK(Decoded.Key == Key && Decoded.Session == M.Session);
        const auto Bytes = *R::Encode(M);
        for (std::size_t N = 0; N < Bytes.size(); ++N) CHECK(!R::Decode(Bytes.data(), N));
        CHECK(!N::Decode(Bytes.data(), Bytes.size()));

        auto Changed = Bytes;
        Changed[4] = 2; // unsupported version
        CHECK(!R::Decode(Changed.data(), Changed.size()));
        Changed = Bytes;
        Changed[6] = 0x7f; // unknown kind
        CHECK(!R::Decode(Changed.data(), Changed.size()));
        Changed = Bytes;
        Changed[7] = 1; // reserved flags
        CHECK(!R::Decode(Changed.data(), Changed.size()));
        Changed = Bytes;
        Changed[10] = 1;
        CHECK(!R::Decode(Changed.data(), Changed.size()));
        Changed = Bytes;
        Changed.push_back(0);
        CHECK(!R::Decode(Changed.data(), Changed.size()));
    }

    auto Encoded = *R::Encode(Message(B, R::FReservationResult{true}));
    Encoded.back() = 2;
    CHECK(!R::Decode(Encoded.data(), Encoded.size()));
    CHECK(!R::Encode(Message(B, R::FReserve{0})));
    CHECK(!R::Encode(Message(B, R::FReserve{R::MaxBudgetMilliseconds + 1})));
    CHECK(!R::Encode(Message(B, R::FApplied{{0, 1, 2, 3}})));
    CHECK(!R::Encode(Message(B, R::FApplied{{1, 0, 2, 3}})));
    CHECK(!R::Encode(Message(B, R::FApplied{{1, 2, 0, 3}})));
    CHECK(!R::Encode(Message(B, R::FApplied{{1, 2, 3, 0}})));
    auto InvalidKey = Key;
    InvalidKey.PreviousGeneration = 0;
    InvalidKey.Generation = 1;
    CHECK(!R::Encode(Message(B, R::FReserve{200}, InvalidKey)));
    InvalidKey = Key;
    ++InvalidKey.Generation;
    CHECK(!R::Encode(Message(B, R::FReserve{200}, InvalidKey)));

    auto BadInput = P;
    ++BadInput.Frame;
    CHECK(!R::Encode(Message(B, R::FBaseline{BadInput})));
    BadInput = P;
    ++BadInput.Incarnation;
    CHECK(!R::Encode(Message(B, R::FBaseline{BadInput})));
    BadInput = P;
    BadInput.Input.Reset = false;
    CHECK(!R::Encode(Message(B, R::FBaseline{BadInput})));
    BadInput = P;
    BadInput.Kind = N::EKind::AuthorityInput;
    CHECK(!R::Encode(Message(B, R::FBaseline{BadInput})));

    const auto Baked = std::get<R::FBaseline>(RoundTrip(Message(B, R::FBaseline{P})).Payload).Packet;
    CHECK(N::Same(P, Baked));
    CHECK(Baked.Input.Values == P.Input.Values && Baked.Input.ActiveMask == 7);
    const auto Plain = *N::Encode(P);
    CHECK(!R::Decode(Plain.data(), Plain.size()) && N::Decode(Plain.data(), Plain.size()));
}

struct FMatch
{
    const N::FBinding A = Binding(0), B = Binding(1);
    R::FHostRoundControl Host = MakeHost();

    R::FHostRoundControl MakeHost() const
    {
        auto Gate = R::FHostRoundControl::Create(Key, {B, A}, 5000, Now);
        CHECK(Gate);
        return std::move(*Gate);
    }

    R::FClientRoundControl AcceptOffer(const N::FBinding& Session)
    {
        const auto Offer = Host.ReserveFor(Session.Session, Now);
        CHECK(Offer);
        CHECK(!R::FClientRoundControl::OpenOffer(Session.OwnerPeer, Session, Key.Match, Key.MatchEpoch, 1, 0, *Offer, Now));
        CHECK(!R::FClientRoundControl::OpenOffer(Session.ServerPeer, Session, Key.Match, Key.MatchEpoch, 0, 0, *Offer, Now));
        CHECK(!R::FClientRoundControl::OpenOffer(Session.ServerPeer, Session, Key.Match, Key.MatchEpoch, 1, 20, *Offer, Now));
        auto Gate = R::FClientRoundControl::OpenOffer(Session.ServerPeer, Session, Key.Match, Key.MatchEpoch, 1, 19, *Offer, Now);
        CHECK(Gate && (Gate->Key() == Key));
        CHECK(Gate->ReceiveFromServer(Session.ServerPeer, *Offer, Now + 1s) == EResult::Duplicate);
        CHECK(Gate->Deadline() == Now + 5s);
        return std::move(*Gate);
    }

    EResult Take(const N::FBinding& Session, const R::FMessage& M, R::FClock::time_point At = Now)
    { return Host.ReceiveFromClient(Session.OwnerPeer, RoundTrip(M), At); }

    void ReserveAndSupplyBoth()
    {
        for (const auto& Session : {A, B})
        {
            CHECK(Take(Session, Message(Session, R::FReservationResult{true})) == EResult::Accepted);
            CHECK(Take(Session, Message(Session, R::FBaseline{Capture(Session)})) == EResult::Accepted);
        }
        CHECK(Host.ReadyForWorldCommit(Now));
    }
};

static void TestTwoClientCutover()
{
    FMatch M;
    auto A = M.AcceptOffer(M.A), B = M.AcceptOffer(M.B);
    const auto P1 = Capture(M.A), P2 = Capture(M.B);
    CHECK(!A.PublishBaseline(P1, Now));
    CHECK(!A.AcknowledgeInstalled(Step, Now));
    CHECK(M.Take(M.A, Message(M.A, R::FBaseline{P1})) == EResult::OutOfOrder);
    CHECK(M.Take(M.A, Require(A.ReservationResult(true, Now))) == EResult::Accepted);
    CHECK(M.Take(M.A, Require(A.ReservationResult(true, Now))) == EResult::Duplicate);
    CHECK(M.Take(M.A, Require(A.PublishBaseline(P1, Now))) == EResult::Accepted);
    CHECK(M.Take(M.A, Require(A.PublishBaseline(P1, Now))) == EResult::Duplicate);
    CHECK(!M.Host.AllReserved(Now) && !M.Host.BaselineFor(M.A.Session, Now));
    CHECK(M.Host.ObserveCompletedNeutral(Key, Step, Now) == EResult::OutOfOrder);
    CHECK(M.Take(M.B, Require(B.ReservationResult(true, Now))) == EResult::Accepted);
    CHECK(M.Host.AllReserved(Now) && !M.Host.ReadyForWorldCommit(Now));
    CHECK(M.Take(M.B, Require(B.PublishBaseline(P2, Now))) == EResult::Accepted);
    const auto* Retained = M.Host.BaselineFor(M.A.Session, Now);
    CHECK(M.Host.ReadyForWorldCommit(Now) && Retained && N::Same(*Retained, P1));
    CHECK(M.Host.ObserveCompletedNeutral(Key, Step, Now) == EResult::Accepted);
    CHECK(M.Host.ObserveCompletedNeutral(Key, Step, Now) == EResult::Duplicate);
    CHECK(!M.Host.ReleaseFor(M.A.Session, Now));

    const auto PublishedA = Require(M.Host.PublicationFor(M.A.Session, Now));
    const auto PublishedB = Require(M.Host.PublicationFor(M.B.Session, Now));
    CHECK(A.ReceiveFromServer(M.B.OwnerPeer, PublishedA, Now) == EResult::WrongPeer);
    CHECK(A.ReceiveFromServer(M.A.ServerPeer, PublishedB, Now) == EResult::WrongSession);
    CHECK(A.ReceiveFromServer(M.A.ServerPeer, PublishedA, Now) == EResult::Accepted);
    CHECK(A.ReceiveFromServer(M.A.ServerPeer, PublishedA, Now) == EResult::Duplicate);
    CHECK(B.ReceiveFromServer(M.B.ServerPeer, PublishedB, Now) == EResult::Accepted);
    CHECK(B.ReceiveFromServer(M.B.ServerPeer, Message(M.B, R::FReleased{Step}), Now) == EResult::OutOfOrder);

    const auto AppliedA = Require(A.AcknowledgeInstalled(Step, Now));
    CHECK(M.Take(M.B, AppliedA) == EResult::WrongPeer);
    CHECK(M.Take(M.A, AppliedA) == EResult::Accepted);
    CHECK(M.Take(M.A, AppliedA) == EResult::Duplicate);
    CHECK(M.Host.AuthorizeRelease(Key, Step, Now) == EResult::OutOfOrder);
    CHECK(!A.GameplayReleased(Now) && !B.GameplayReleased(Now) && !M.Host.GameplayReleased(Key, Step, Now));
    CHECK(M.Take(M.B, Require(B.AcknowledgeInstalled(Step, Now))) == EResult::Accepted);
    CHECK(M.Host.AuthorizeRelease(Key, Step, Now) == EResult::Accepted);
    CHECK(M.Host.AuthorizeRelease(Key, Step, Now) == EResult::Duplicate);
    CHECK(M.Host.GameplayReleased(Key, Step, Now));
    for (auto Pair : {std::make_pair(&A, &M.A), std::make_pair(&B, &M.B)})
    {
        const auto Release = Require(M.Host.ReleaseFor(Pair.second->Session, Now));
        CHECK(Pair.first->ReceiveFromServer(Pair.second->ServerPeer, RoundTrip(Release), Now) == EResult::Accepted);
        CHECK(Pair.first->ReceiveFromServer(Pair.second->ServerPeer, Release, Now) == EResult::Duplicate);
        CHECK(Pair.first->GameplayReleased(Now));
    }
}

static void TestFailClosed()
{
    {
        FMatch M;
        auto DuplicatePeer = M.B;
        DuplicatePeer.OwnerPeer = M.A.OwnerPeer;
        CHECK(!R::FHostRoundControl::Create(Key, {M.A, DuplicatePeer}, 5000, Now));
        CHECK(!R::FHostRoundControl::Create(Key, {}, 5000, Now));
        CHECK(!R::FHostRoundControl::Create(Key, {M.A}, 0, Now));
        auto Old = Key;
        --Old.Reservation;
        CHECK(M.Take(M.A, Message(M.A, R::FReservationResult{false}, Old)) == EResult::Stale);
        auto Future = Key;
        ++Future.Generation;
        ++Future.PreviousGeneration;
        CHECK(M.Take(M.A, Message(M.A, R::FReservationResult{false}, Future)) == EResult::WrongRound);
        auto WrongEpoch = Message(M.A, R::FReservationResult{false});
        ++WrongEpoch.Session.Epoch;
        CHECK(M.Take(M.A, WrongEpoch) == EResult::WrongSession);
        auto WrongActor = Message(M.A, R::FReservationResult{false});
        ++WrongActor.Session.Actor;
        CHECK(M.Take(M.A, WrongActor) == EResult::WrongSession);
        CHECK(M.Take(M.A, Message(M.A, R::FReservationResult{true})) == EResult::Accepted);
        CHECK(M.Take(M.B, Message(M.B, R::FReservationResult{false})) == EResult::Rejected);
        CHECK(!M.Host.AllReserved(Now));
        CHECK(!M.Host.ReleaseFor(M.A.Session, Now));
    }
    {
        FMatch M;
        M.ReserveAndSupplyBoth();
        auto Conflict = Capture(M.A);
        ++Conflict.Input.SourceSequence;
        CHECK(M.Take(M.A, Message(M.A, R::FBaseline{Conflict})) == EResult::Conflict);
        CHECK(!M.Host.ReadyForWorldCommit(Now));
    }
    for (auto Field : {&R::FPublication::Serial, &R::FPublication::StateHash,
             &R::FPublication::InputHash, &R::FPublication::NeutralBaselineHash})
    {
        FMatch M;
        M.ReserveAndSupplyBoth();
        CHECK(M.Host.ObserveCompletedNeutral(Key, Step, Now) == EResult::Accepted);
        auto WrongHash = Step;
        ++(WrongHash.*Field);
        CHECK(M.Take(M.B, Message(M.B, R::FApplied{WrongHash})) == EResult::Conflict);
        CHECK(M.Take(M.A, Message(M.A, R::FApplied{Step})) == EResult::Closed);
        CHECK(!M.Host.ReleaseFor(M.A.Session, Now));
    }
    {
        FMatch M;
        M.ReserveAndSupplyBoth();
        CHECK(M.Host.ObserveCompletedNeutral(Key, Step, Now) == EResult::Accepted);
        auto WrongFrame = Key;
        ++WrongFrame.Frame;
        CHECK(M.Take(M.A, Message(M.A, R::FApplied{Step}, WrongFrame)) == EResult::Conflict);
        CHECK(!M.Host.ReleaseFor(M.A.Session, Now));
    }
    {
        FMatch M;
        auto Client = M.AcceptOffer(M.A);
        CHECK(!Client.ReservationResult(true, Now + 5s));
        CHECK(M.Take(M.A, Message(M.A, R::FReservationResult{true}), Now + 5s) == EResult::Expired);
        CHECK(!M.Host.ReserveFor(M.A.Session, Now + 5s));
        CHECK(M.Host.AuthorizeRelease(Key, Step, Now + 5s) == EResult::Expired);
        M.Host.CancelBeforeRelease();
        CHECK(!M.Host.ReserveFor(M.A.Session, Now));
    }
    {
        FMatch M;
        auto Client = M.AcceptOffer(M.A);
        CHECK(Client.ReservationResult(true, Now));
        CHECK(Client.PublishBaseline(Capture(M.A), Now));
        CHECK(Client.ReceiveFromServer(M.A.ServerPeer, Message(M.A, R::FPublished{Step}), Now) == EResult::Accepted);
        auto WrongSerial = Step;
        ++WrongSerial.Serial;
        CHECK(!Client.AcknowledgeInstalled(WrongSerial, Now));
        CHECK(!Client.AcknowledgeInstalled(Step, Now));
        CHECK(!Client.GameplayReleased(Now));
    }
}

static void TestEffectiveConfigurationWriter()
{
    using FWriter = Speed::FExactRoundConfigurationWriter;
    FWriter A, Same, Diff;
    CHECK(!A.Hash());
    CHECK(A.Unsigned(1) && A.Float(6) && A.Boolean(true) && A.Double(2) && A.Signed(-1));
    CHECK(Same.Unsigned(1) && Same.Float(6) && Same.Boolean(true) && Same.Double(2) && Same.Signed(-1));
    CHECK(A.Hash() && A.Hash() == Same.Hash());
    CHECK(Diff.Unsigned(1) && Diff.Float(6) && Diff.Boolean(true) && Diff.Float(2) && Diff.Signed(-1));
    CHECK(A.Hash() != Diff.Hash());
    CHECK(!Diff.Double(std::numeric_limits<double>::quiet_NaN()) && !Diff.Hash());
    CHECK(!Diff.Unsigned(1) && !Diff.Hash());
    FWriter Limit;
    for (unsigned I = 0; I < FWriter::MaxValues; ++I) CHECK(Limit.Unsigned(I));
    CHECK(Limit.Hash() && !Limit.Unsigned(17) && !Limit.Hash());
}

static void TestNativeAddressFromCompletedState()
{
    using FNative = Speed::FExactRoundNativeAddress;
    const Speed::FExactRoundNativeOrigin Origin{13, 19, 23};
    CHECK(!FNative::FromCompleted({}, 67, 71, 37));
    CHECK(!FNative::FromCompleted(Origin, UINT32_MAX, 71, 37));
    CHECK(!FNative::FromCompleted(Origin, 67, 0, 37));
    CHECK(!FNative::FromCompleted(Origin, 67, 71, 0));
    CHECK(!FNative::FromCompleted(Origin, 67, 71, UINT64_MAX));
    const auto Native = Require(FNative::FromCompleted(Origin, 67, 71, 37));
    Speed::PreparedWorld::FAddress Address;
    Address.ResumeGeneration = 41;
    Address.StateHash = 43;
    Address.SupplementHash = 53;
    Native.Stamp(Address);
    CHECK(Address.Valid() && Native.Matches(Address));
    CHECK(Address.StateVersion == 37 && Address.PublicationSerial == 38 && Address.Frame == 67 && Address.Next == 68);
    CHECK(Address.ResumeGeneration == 41 && Address.StateHash == 43 && Address.SupplementHash == 53);
    for (auto Member : {&Speed::PreparedWorld::FAddress::Authority, &Speed::PreparedWorld::FAddress::WorldIncarnation,
        &Speed::PreparedWorld::FAddress::TopologyVersion, &Speed::PreparedWorld::FAddress::StateVersion,
        &Speed::PreparedWorld::FAddress::PublicationSerial, &Speed::PreparedWorld::FAddress::Frame,
        &Speed::PreparedWorld::FAddress::Next, &Speed::PreparedWorld::FAddress::InputHash})
    {
        auto Altered = Address;
        ++(Altered.*Member);
        CHECK(!Native.Matches(Altered));
    }
}

int main()
{
    TestWire();
    TestTwoClientCutover();
    TestFailClosed();
    TestEffectiveConfigurationWriter();
    TestNativeAddressFromCompletedState();
    std::cout << "PASS checks=" << Checks << " scope=portable_round_control_no_sockets_no_native_application\n";
}
