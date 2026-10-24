#include "IAmSpeed/World/Simulation/PreparedWorldCore.h"
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <new>

static std::atomic<long long> FailAfter{-1};
static std::atomic<unsigned long long> Allocations{0};
void* operator new(std::size_t Size)
{
    Allocations.fetch_add(1);
    const auto Fail = FailAfter.load();
    if (Fail >= 0 && FailAfter.fetch_sub(1) == 0) throw std::bad_alloc();
    if (auto* P = std::malloc(Size ? Size : 1)) return P;
    throw std::bad_alloc();
}
void* operator new[](std::size_t Size) { return ::operator new(Size); }
void operator delete(void* P) noexcept { std::free(P); }
void operator delete[](void* P) noexcept { std::free(P); }
void operator delete(void* P, std::size_t) noexcept { std::free(P); }
void operator delete[](void* P, std::size_t) noexcept { std::free(P); }
using namespace Speed::PreparedWorld;
static unsigned Checks = 0;
#define CHECK(X) do { ++Checks; if (!(X)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #X); std::exit(1); } } while (false)
static void Put(std::vector<Byte>& B, U64 V, std::size_t N)
{ for (std::size_t I = 0; I < N; ++I) { B.push_back(Byte(V & 255)); V >>= 8; } }
static void Set(std::vector<Byte>& B, std::size_t At, U64 V, std::size_t N)
{ for (std::size_t I = 0; I < N; ++I) { B[At + I] = Byte(V & 255); V >>= 8; } }
template<class T> static void PutFloat(std::vector<Byte>& B, T V)
{
    U64 Bits = 0;
    if constexpr (sizeof(T) == 4) { U32 Small = 0; std::memcpy(&Small, &V, 4); Bits = Small; }
    else std::memcpy(&Bits, &V, 8);
    Put(B, Bits, sizeof(T));
}
template<class T> static bool SameBits(T A, T B) { return std::memcmp(&A, &B, sizeof(T)) == 0; }
static FPair Pair(U64 A, U64 B, double Offset)
{
    FPair P; P.A = A; P.B = B;
    for (std::size_t I = 0; I < P.Geometry.size(); ++I) P.Geometry[I] = Offset + double(I) * 0.125;
    P.Geometry[3] = -0.0; P.Geometry[4] = std::numeric_limits<double>::denorm_min();
    P.History = {0, 42, UINT32_MAX, 13, 5, 8, UINT32_MAX - 1};
    P.Activity = {-0.0f, std::numeric_limits<float>::denorm_min(), 1.25f, -3.5f,
        0, 3, std::numeric_limits<float>::max(), std::numeric_limits<float>::lowest()};
    P.Rolling = true; return P;
}
struct FFixture
{
    FRestoreEnvelope Envelope;
    FParsedWorld Values;
    std::vector<Byte> Bytes;
    std::vector<std::size_t> BodyAt, PairAt;
    std::size_t DynamicCountAt = 0, PendingCountAt = 0;
    FFixture()
    {
        auto& A = Envelope.Address;
        A.Authority = 51; A.WorldIncarnation = 9; A.TopologyVersion = 4; A.StateVersion = 17;
        A.Frame = 42; A.Next = 43; A.InputHash = 0x332211; A.ResumeGeneration = 27; A.PublicationSerial = 19;
        Envelope.Actors = {{7, 1, 501, 11, 12}, {8, 4, 502, 13, 14}, {9, 2, 503, 15, 16}};
        Envelope.Endpoints = {{7001, 1, 7, 1, 88}, {7002, 3, 7, 1, 22},
            {8001, 1, 8, 4, 99}, {9001, 2, 9, 2, 11}};
        for (const auto& B : Envelope.Actors)
        {
            Envelope.ActorSupplements.push_back({B, {Byte(B.StableId), 0, 255}, {22, Byte(B.Incarnation)}});
            FBody Body; Body.StableId = B.StableId;
            for (std::size_t I = 0; I < Body.Kinematic.size(); ++I) Body.Kinematic[I] = double(I) * -1.125;
            Body.Kinematic[0] = -0.0; Body.Kinematic[1] = std::numeric_limits<double>::denorm_min();
            Body.Kinematic[12] = 1.0; Body.Frozen = B.StableId == 9;
            Body.MechanicPayload = {Byte(B.StableId), 0, 99, 255}; Values.Bodies.push_back(Body);
        }
        Envelope.WorldSchema = 92; Envelope.WorldPayload = {9, 11, 25};
        Values.Dynamic = {Pair(8001, 7001, 0.25), Pair(9001, 7002, 18.75)};
        Values.Pending = {Pair(8001, 9001, -2.25)}; Encode();
    }
    void Encode()
    {
        Bytes.clear(); BodyAt.clear(); PairAt.clear();
        Put(Bytes, Schema, 4); Put(Bytes, Envelope.Address.Frame, 8); Put(Bytes, Envelope.Address.InputHash, 8);
        Put(Bytes, Values.Bodies.size(), 4);
        for (const auto& B : Values.Bodies)
        {
            BodyAt.push_back(Bytes.size()); Put(Bytes, B.StableId, 8);
            for (auto V : B.Kinematic) PutFloat(Bytes, V);
            Put(Bytes, B.Frozen ? 1 : 0, 1); Put(Bytes, B.MechanicPayload.size(), 4);
            Bytes.insert(Bytes.end(), B.MechanicPayload.begin(), B.MechanicPayload.end());
        }
        for (const auto Item : {std::make_pair(&Values.Dynamic, true), std::make_pair(&Values.Pending, false)})
        {
            (Item.second ? DynamicCountAt : PendingCountAt) = Bytes.size(); Put(Bytes, Item.first->size(), 4);
            for (const auto& P : *Item.first)
            {
                PairAt.push_back(Bytes.size()); Put(Bytes, P.A, 8); Put(Bytes, P.B, 8);
                for (auto V : P.Geometry) PutFloat(Bytes, V);
                for (auto V : P.History) Put(Bytes, V, 4);
                for (auto V : P.Activity) PutFloat(Bytes, V);
                Put(Bytes, P.Rolling ? 1 : 0, 1); Put(Bytes, P.Active ? 1 : 0, 1);
            }
        }
        Reseal();
    }
    void Reseal()
    {
        Envelope.Address.StateHash = HashBytes(Bytes.data(), Bytes.size());
        Envelope.Address.SupplementHash = HashSupplement(Envelope);
    }
    auto ParseNow() const { return Parse(Bytes.data(), Bytes.size(), Envelope.Address, Envelope.Actors); }
};
static void TestRoundTripAndIndex()
{
    FFixture F; CHECK(ValidEnvelope(F.Envelope)); const auto P = F.ParseNow(); CHECK(P);
    CHECK(P->Frame == 42 && P->InputHash == 0x332211 && P->Bodies.size() == 3);
    for (std::size_t I = 0; I < P->Bodies.size(); ++I)
    {
        CHECK(P->Bodies[I].StableId == F.Values.Bodies[I].StableId);
        CHECK(P->Bodies[I].Frozen == F.Values.Bodies[I].Frozen);
        CHECK(P->Bodies[I].MechanicPayload == F.Values.Bodies[I].MechanicPayload);
        for (std::size_t J = 0; J < 19; ++J) CHECK(SameBits(P->Bodies[I].Kinematic[J], F.Values.Bodies[I].Kinematic[J]));
    }
    const std::vector<const std::vector<FPair>*> Before{&F.Values.Dynamic, &F.Values.Pending};
    const std::vector<const std::vector<FPair>*> After{&P->Dynamic, &P->Pending};
    for (std::size_t C = 0; C < 2; ++C) for (std::size_t I = 0; I < Before[C]->size(); ++I)
    {
        const auto& A = (*Before[C])[I]; const auto& B = (*After[C])[I]; CHECK(A.A == B.A && A.B == B.B);
        CHECK(A.History == B.History); CHECK(A.Rolling == B.Rolling && A.Active == B.Active);
        for (std::size_t J = 0; J < 15; ++J) CHECK(SameBits(A.Geometry[J], B.Geometry[J]));
        for (std::size_t J = 0; J < 8; ++J) CHECK(SameBits(A.Activity[J], B.Activity[J]));
    }
    auto I = BuildContactIndexes(*P, F.Envelope.Actors, F.Envelope.Endpoints); CHECK(I);
    const U64 K0 = (U64(88) << 32) | 99, K1 = (U64(11) << 32) | 22, KP = (U64(11) << 32) | 99;
    CHECK(I->DynamicKeys == std::vector<U64>({K0, K1})); CHECK(I->PendingKeys == std::vector<U64>({KP}));
    CHECK(I->Dynamic.at(K0) == 0 && I->Dynamic.at(K1) == 1 && I->Pending.at(KP) == 0);
    CHECK(I->DynamicByBody.at(88) == std::set<U64>{K0}); CHECK(I->DynamicByBody.at(11) == std::set<U64>{K1});
    CHECK(I->DynamicByBody.at(99).count(KP) == 0); // pending is NOT an active body-key entry
    F.Values.Dynamic.clear(); F.Values.Pending.clear(); F.Encode();
    const auto Empty = F.ParseNow(); CHECK(Empty && Empty->Dynamic.empty() && Empty->Pending.empty());
    const auto EI = BuildContactIndexes(*Empty, F.Envelope.Actors, F.Envelope.Endpoints);
    CHECK(EI && EI->Dynamic.empty() && EI->Pending.empty() && EI->DynamicByBody.empty());
}
static void TestMalformedBytes()
{
    const FFixture Original;
    for (std::size_t N = 0; N < Original.Bytes.size(); ++N)
    {
        auto F = Original; F.Bytes.resize(N); F.Reseal(); CHECK(!F.ParseNow());
    }
    for (std::size_t N = 0; N < Original.Bytes.size(); ++N)
    {
        auto F = Original; F.Bytes[N] ^= 0x40; CHECK(!F.ParseNow()); // digest mismatch, no partial result
    }
    auto Bad = [&](std::size_t At, U64 Value, std::size_t Width)
    { auto F = Original; Set(F.Bytes, At, Value, Width); F.Reseal(); CHECK(!F.ParseNow()); };
    Bad(0, 1, 4); Bad(4, 41, 8); Bad(12, 12, 8); Bad(20, 2, 4);
    Bad(Original.BodyAt[1], 7, 8); Bad(Original.BodyAt[1], 82, 8);
    Bad(Original.BodyAt[0] + 8, 0x7ff8000000000001ull, 8); // NaN kinematic
    Bad(Original.BodyAt[0] + 8 + 19 * 8, 2, 1);
    Bad(Original.BodyAt[0] + 8 + 19 * 8 + 1, UINT32_MAX, 4);
    Bad(Original.DynamicCountAt, UINT32_MAX, 4); Bad(Original.PendingCountAt, UINT32_MAX, 4);
    Bad(Original.PairAt[0], 0, 8); Bad(Original.PairAt[0], 7001, 8); // self pair with B=7001
    Bad(Original.PairAt[0] + 16, 0x7ff0000000000000ull, 8); // infinite geometry
    Bad(Original.PairAt[0] + 16 + 120 + 28, 0x7fc00001, 4); // NaN activity
    Bad(Original.PairAt[0] + PairBytes - 2, 3, 1); Bad(Original.PairAt[0] + PairBytes - 1, 3, 1);
    auto F = Original; F.Bytes.push_back(0); F.Reseal(); CHECK(!F.ParseNow());
    F = Original; F.Values.Pending[0] = F.Values.Dynamic[0]; F.Encode(); CHECK(!F.ParseNow());
    std::swap(F.Values.Pending[0].A, F.Values.Pending[0].B); F.Encode(); CHECK(!F.ParseNow());
    F = Original; F.Values.Dynamic[0].A = 12345; F.Encode(); const auto Foreign = F.ParseNow(); CHECK(Foreign);
    CHECK(!BuildContactIndexes(*Foreign, F.Envelope.Actors, F.Envelope.Endpoints));
    F = Original; F.Envelope.Address.Frame = UINT32_MAX; F.Envelope.Address.Next = U64(UINT32_MAX) + 1; F.Encode(); CHECK(!F.ParseNow());
    F.Envelope.Address.Frame = UINT32_MAX - 1; F.Envelope.Address.Next = UINT32_MAX; F.Encode(); CHECK(F.ParseNow());
}
static void TestEnvelopeAndLimits()
{
    const FFixture Original;
    auto Bad = [&](auto Mutate)
    { auto F = Original; Mutate(F.Envelope); CHECK(!ValidEnvelope(F.Envelope)); };
    Bad([](auto& E) { ++E.Address.Authority; }); Bad([](auto& E) { ++E.Address.WorldIncarnation; });
    Bad([](auto& E) { ++E.Address.TopologyVersion; }); Bad([](auto& E) { ++E.Address.StateVersion; });
    Bad([](auto& E) { ++E.Address.ResumeGeneration; }); Bad([](auto& E) { ++E.Address.PublicationSerial; });
    Bad([](auto& E) { ++E.ActorSupplements[1].GameplayPayload[0]; });
    Bad([](auto& E) { E.Actors[1].Incarnation++; }); Bad([](auto& E) { E.WorldPayload.push_back(25); });
    auto RehashBad = [&](auto Mutate)
    { auto F = Original; Mutate(F.Envelope); F.Envelope.Address.SupplementHash = HashSupplement(F.Envelope); CHECK(!ValidEnvelope(F.Envelope)); };
    RehashBad([](auto& E) { E.Actors[0].RequiredCoverage &= ~U64(Sleep); });
    RehashBad([](auto& E) { E.Actors[1].StableId = E.Actors[0].StableId; });
    RehashBad([](auto& E) { E.Endpoints[1].NativeObjectId = E.Endpoints[0].NativeObjectId; });
    RehashBad([](auto& E) { E.Endpoints[0].ActorIncarnation++; });
    RehashBad([](auto& E) { E.Endpoints[1].StableId = E.Endpoints[0].StableId; });
    RehashBad([](auto& E) { E.ActorSupplements.pop_back(); });
    RehashBad([](auto& E) { E.WorldSchema = 0; }); RehashBad([](auto& E) { E.WorldCoverage &= ~U64(ActorLifecycle); });
    RehashBad([](auto& E) { E.Address.Next++; });
    FLimits L; L.MaxPayload = Original.Bytes.size() - 1;
    CHECK(!Parse(Original.Bytes.data(), Original.Bytes.size(), Original.Envelope.Address, Original.Envelope.Actors, L));
    L.MaxPayload++; CHECK(Parse(Original.Bytes.data(), Original.Bytes.size(), Original.Envelope.Address, Original.Envelope.Actors, L));
    L = {}; L.MaxBodies = 2; CHECK(!ValidEnvelope(Original.Envelope, L));
    L = {}; L.MaxMechanicPayload = 3; CHECK(!Parse(Original.Bytes.data(), Original.Bytes.size(), Original.Envelope.Address, Original.Envelope.Actors, L));
    L = {}; L.MaxPairs = 2; CHECK(!Parse(Original.Bytes.data(), Original.Bytes.size(), Original.Envelope.Address, Original.Envelope.Actors, L));
    L.MaxPairs = 3; CHECK(Parse(Original.Bytes.data(), Original.Bytes.size(), Original.Envelope.Address, Original.Envelope.Actors, L));
}
static void TestAllocationFailureAndPreparedStorageTail()
{
    FFixture F; bool Success = false; unsigned Failures = 0;
    // This live fixture verifies the real parser/index builder never touches
    // live storage on a fallible path. It is NOT a native component/world lease.
    auto Live = std::make_unique<FContactIndexes>(); Live->Dynamic.emplace(123, 9);
    auto* const Before = Live.get();
    for (long long Budget = 0; Budget < 512; ++Budget)
    {
        FailAfter = Budget;
        try
        {
            auto Parsed = F.ParseNow(); if (!Parsed) std::terminate();
            auto Index = BuildContactIndexes(*Parsed, F.Envelope.Actors, F.Envelope.Endpoints);
            if (!Index) std::terminate();
            auto Prepared = std::make_unique<FContactIndexes>(std::move(*Index));
            FailAfter = -1;
            CHECK(Live.get() == Before && Live->Dynamic.at(123) == 9);
            const auto Count = Allocations.load(); FailAfter = 0;
            Live.swap(Prepared); // same standard no-throw storage install used by native candidate
            FailAfter = -1; CHECK(Allocations.load() == Count);
            CHECK(Live->Dynamic.size() == 2 && Prepared.get() == Before && Prepared->Dynamic.at(123) == 9);
            Success = true; break;
        }
        catch (const std::bad_alloc&)
        {
            FailAfter = -1; ++Failures; CHECK(Live.get() == Before && Live->Dynamic.at(123) == 9);
        }
    }
    CHECK(Success && Failures > 10);
    std::printf("allocation failure budgets exhausted cleanly: %u\n", Failures);
}
int main()
{
    const U32 Order = 0x04030201; CHECK(*reinterpret_cast<const Byte*>(&Order) == 1);
    static_assert(PairBytes == 198 && BodyFixedBytes == 165);
    static_assert(noexcept(std::declval<std::unique_ptr<FContactIndexes>&>().swap(std::declval<std::unique_ptr<FContactIndexes>&>())));
    TestRoundTripAndIndex(); TestMalformedBytes(); TestEnvelopeAndLimits(); TestAllocationFailureAndPreparedStorageTail();
    std::printf("PreparedWorldCore: %u checks PASS; real frozen2776-format parser/indexes, fixture bytes only; native NOT_COMPILED/RUN/ADMITTED\n", Checks);
    return 0;
}
