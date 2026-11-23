#pragma once

#include "PreparedWorldCore.h"

namespace Speed::PreparedWorld
{
struct FSnapshotView
{
    const Byte* Data = nullptr;
    std::size_t Size = 0;
    U64 Frame = 0, InputHash = 0, StateHash = 0, PublicationSerial = 0;
};

struct FBodyReplacement
{
    U64 StableId = 0;
    std::array<double, 19> Kinematic{};
    bool Frozen = false;
};

struct FContactRemoval { U64 A = 0, B = 0; };

struct FReplacementSnapshot
{
    std::vector<Byte> Payload;
    FRestoreEnvelope Envelope;
};

namespace ReplacementDetail
{
class FWriter
{
public:
    std::vector<Byte> Bytes;

    void UInt(U64 Value, std::size_t Width)
    {
        for (std::size_t I = 0; I < Width; ++I)
        {
            Bytes.push_back(Byte(Value & 255));
            Value >>= 8;
        }
    }

    template<class T> void Float(T Value)
    {
        static_assert(std::numeric_limits<T>::is_iec559 && (sizeof(T) == 4 || sizeof(T) == 8));
        if constexpr (sizeof(T) == 4)
        {
            U32 Bits = 0;
            std::memcpy(&Bits, &Value, sizeof(Bits));
            UInt(Bits, 4);
        }
        else
        {
            U64 Bits = 0;
            std::memcpy(&Bits, &Value, sizeof(Bits));
            UInt(Bits, 8);
        }
    }
};
}

// Local schema-2 edit only. Preserve canonical body order, opaque mechanical
// bytes and every contact except the exact requested endpoint pairs.
inline std::optional<FReplacementSnapshot> BuildReplacement(
    const FSnapshotView& Source, FRestoreEnvelope Envelope, const std::vector<FBodyReplacement>& Replacements,
    const std::vector<FContactRemoval>& ContactsToRemove)
{
    const FLimits Limits;
    auto& Address = Envelope.Address;
    if (!Source.PublicationSerial || Address.PublicationSerial <= Source.PublicationSerial
        || Replacements.size() > Limits.MaxBodies || ContactsToRemove.size() > Limits.MaxPairs
        || !ValidEnvelopeContents(Envelope, Limits)) return {};

    const auto Parsed = ParseSnapshot(Source.Data, Source.Size, Source.Frame, Source.InputHash,
        Source.StateHash, Envelope.Actors, Limits);
    if (!Parsed || !BuildContactIndexes(*Parsed, Envelope.Actors, Envelope.Endpoints, Limits)) return {};

    std::map<U64, const FBodyReplacement*> PendingReplacements;
    for (const auto& Body : Replacements)
    {
        if (!Body.StableId || !PendingReplacements.emplace(Body.StableId, &Body).second
            || !std::all_of(Body.Kinematic.begin(), Body.Kinematic.end(), [](double Value) { return std::isfinite(Value); }))
            return {};
    }
    std::set<std::pair<U64, U64>> PendingRemovals;
    for (const auto& Contact : ContactsToRemove)
    {
        const auto Key = std::minmax(Contact.A, Contact.B);
        if (!Key.first || Key.first == Key.second || !PendingRemovals.emplace(Key.first, Key.second).second) return {};
    }

    ReplacementDetail::FWriter W;
    W.Bytes.reserve(Source.Size); // This edit can only shrink the input.
    W.UInt(Schema, 4);
    W.UInt(Parsed->Frame, 8);
    W.UInt(Parsed->InputHash, 8);
    W.UInt(Parsed->Bodies.size(), 4);
    for (const auto& Body : Parsed->Bodies)
    {
        const auto I = PendingReplacements.find(Body.StableId);
        const auto* Replacement = I == PendingReplacements.end() ? nullptr : I->second;
        W.UInt(Body.StableId, 8);
        for (double Value : Replacement ? Replacement->Kinematic : Body.Kinematic) W.Float(Value);
        W.UInt(Replacement ? Replacement->Frozen : Body.Frozen, 1);
        W.UInt(Body.MechanicPayload.size(), 4);
        W.Bytes.insert(W.Bytes.end(), Body.MechanicPayload.begin(), Body.MechanicPayload.end());
        if (Replacement) PendingReplacements.erase(I);
    }
    if (!PendingReplacements.empty()) return {};

    for (const auto* Collection : {&Parsed->Dynamic, &Parsed->Pending})
    {
        std::vector<const FPair*> Kept;
        Kept.reserve(Collection->size());
        for (const auto& Pair : *Collection)
        {
            const auto Key = std::minmax(Pair.A, Pair.B);
            if (!PendingRemovals.erase({Key.first, Key.second})) Kept.push_back(&Pair);
        }
        W.UInt(Kept.size(), 4);
        for (const auto* Pair : Kept)
        {
            W.UInt(Pair->A, 8);
            W.UInt(Pair->B, 8);
            for (double Value : Pair->Geometry) W.Float(Value);
            for (U32 Value : Pair->History) W.UInt(Value, 4);
            for (float Value : Pair->Activity) W.Float(Value);
            W.UInt(Pair->Rolling, 1);
            W.UInt(Pair->Active, 1);
        }
    }
    if (!PendingRemovals.empty()) return {};

    Address.Frame = Parsed->Frame;
    Address.Next = Parsed->Frame + 1;
    Address.InputHash = Parsed->InputHash;
    Address.StateHash = HashBytes(W.Bytes.data(), W.Bytes.size());
    Address.SupplementHash = HashSupplement(Envelope);
    if (!Address.Valid()) return {};
    return FReplacementSnapshot{std::move(W.Bytes), std::move(Envelope)};
}
}
