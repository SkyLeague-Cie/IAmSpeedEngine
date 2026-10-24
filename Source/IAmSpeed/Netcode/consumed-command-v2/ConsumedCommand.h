#pragma once
#include "../transport/NetInputTransport.h"
#include <algorithm>
#include <memory>

namespace Speed::Netcode::Consumed
{
inline constexpr bool NativeEndpointQualified = false;
constexpr std::size_t MaxFingerprint = 64 * 1024, MaxTargets = 64, MaxPayload = 128 * 1024;
enum class EStage : std::uint8_t { Proposed = 1, Published = 2 };
// This describes the ORIGIN processing policy. It does not relabel a receiving
// ExactRemote producer as Device or authorize that producer's native admission.
enum class EOrigin : std::uint8_t { Device = 1, ExactScenario = 2, AI = 4 };
struct FTarget
{
    U64 Actor = 0, Incarnation = 0;
    bool operator==(const FTarget& R) const noexcept { return Actor == R.Actor && Incarnation == R.Incarnation; }
};
struct FProcessing
{
    EOrigin Origin = EOrigin::Device;
    std::array<std::uint16_t, ActionCount> Step{};
    std::optional<std::array<std::int16_t, ActionCount>> Before;
    std::vector<std::uint8_t> BindingFingerprint; // exact canonical origin owner fingerprint, including policy
    bool operator==(const FProcessing& R) const
    { return Origin == R.Origin && Step == R.Step && Before == R.Before && BindingFingerprint == R.BindingFingerprint; }
};
struct FPublication
{
    U64 AuthorityPeer = 0, AuthorityEpoch = 0, Serial = 0, WorldStateHash = 0;
    bool Empty() const noexcept { return !(AuthorityPeer || AuthorityEpoch || Serial || WorldStateHash); }
    bool Complete() const noexcept { return AuthorityPeer && AuthorityEpoch && Serial && WorldStateHash; }
};
struct FWorldIdentity
{
    U64 Id = 0, Incarnation = 0;
    bool Valid() const noexcept { return Id && Incarnation; }
    bool operator==(const FWorldIdentity& R) const noexcept { return Id == R.Id && Incarnation == R.Incarnation; }
    bool operator!=(const FWorldIdentity& R) const noexcept { return !(*this == R); }
};
struct FDecision
{
    EStage Stage = EStage::Proposed;
    FPacket Original; // Original REQUEST frame, unchanged values/mask/edges/Reset/source metadata.
    U64 RegistryVersion = 0;
    U64 AuthorityTimelineRevision = 0, BeforeWorldPublication = 0;
    FWorldIdentity World;
    std::vector<FTarget> Targets; // complete session target set, stable IDs/incarnations in canonical order
    FProcessing Processing;
    std::array<std::int16_t, ActionCount> Applied{}; // separate physical decision; never fed through a second slew
    FPublication Publication;
};
struct FActionPolicy
{
    bool Wired = false, Axis = false, Signed = false;
    std::int16_t Quantization = 0;
    bool Accepts(std::int16_t V) const noexcept
    { return Wired ? Quantization > 0 && V >= (Signed ? -int(Quantization) : 0) && V <= Quantization : V == 0; }
    bool operator==(const FActionPolicy& R) const noexcept
    { return Wired == R.Wired && Axis == R.Axis && Signed == R.Signed && Quantization == R.Quantization; }
};
// Obtained from the authoritative registry/checkpoint, never from a client's
// claimed Before/policy. BindingFingerprint must be checked there as well.
struct FTrustedContext
{
    FBinding Binding;
    U64 Frame = 0, RegistryVersion = 0, AuthorityEpoch = 0;
    U64 AuthorityTimelineRevision = 0, BeforeWorldPublication = 0;
    FWorldIdentity World;
    std::vector<FTarget> Targets;
    FProcessing Processing;
    std::array<FActionPolicy, ActionCount> Actions;
};
inline bool ValidTargets(const std::vector<FTarget>& T) noexcept
{
    if (T.empty() || T.size() > MaxTargets) return false;
    for (std::size_t I = 0; I < T.size(); ++I)
        if (!T[I].Actor || !T[I].Incarnation || (I && T[I - 1].Actor >= T[I].Actor)) return false;
    return true;
}
inline bool ValidProcessing(const FProcessing& P, const std::array<FActionPolicy, ActionCount>& Actions) noexcept
{
    if ((P.Origin != EOrigin::Device && P.Origin != EOrigin::ExactScenario && P.Origin != EOrigin::AI)
        || P.BindingFingerprint.empty() || P.BindingFingerprint.size() > MaxFingerprint
        || bool(P.Before) != (P.Origin == EOrigin::Device)) return false;
    for (std::size_t I = 0; I < ActionCount; ++I)
    {
        const auto& A = Actions[I];
        if (P.Step[I] && (P.Origin != EOrigin::Device || !A.Wired || !A.Axis || A.Quantization <= 0 || P.Step[I] > 2u * unsigned(A.Quantization))) return false;
        if (P.Before && !A.Accepts((*P.Before)[I])) return false;
    }
    return true;
}
inline std::optional<std::array<std::int16_t, ActionCount>> EvaluateRequested(const FPacket& Request, const FTrustedContext& C)
{
    if (!C.World.Valid() || !C.AuthorityTimelineRevision || !C.BeforeWorldPublication || !Valid(Request) || Request.Kind != EKind::OwnerInput || !C.Binding.Valid() || !C.Binding.Matches(Request)
        || Request.Input.ProducerId != C.Binding.SourceProducer || Request.Input.ProducerKind != C.Binding.SourceKind
        || C.Binding.SourceKind != (C.Processing.Origin == EOrigin::AI ? 1 : 0)
        || Request.Frame != C.Frame || !ValidTargets(C.Targets) || !ValidProcessing(C.Processing, C.Actions)) return {};
    if (std::find(C.Targets.begin(), C.Targets.end(), FTarget{Request.Actor, Request.Incarnation}) == C.Targets.end()) return {};
    auto Applied = Request.Input.Values;
    for (std::size_t I = 0; I < ActionCount; ++I)
    {
        if (!C.Actions[I].Accepts(Applied[I]) || (!C.Actions[I].Wired && (Request.Input.ActiveMask & (std::uint32_t{1} << I)))) return {};
        if (!Request.Input.Reset && C.Processing.Origin == EOrigin::Device && C.Processing.Step[I])
        {
            const int Before = (*C.Processing.Before)[I], Step = C.Processing.Step[I];
            Applied[I] = std::int16_t(Before + std::clamp(int(Applied[I]) - Before, -Step, Step));
        }
    }
    for (const auto& E : Request.Input.Edges)
        if (!C.Actions[E.Action].Wired || !C.Actions[E.Action].Accepts(E.Value)) return {};
    return Applied;
}
// A future server execution proposal. It is not a completed receipt or world
// publication, even if a UDP ACK/source cutoff has already been observed.
inline std::optional<FDecision> Propose(const FPacket& Request, const FTrustedContext& Context)
{
    const auto Applied = EvaluateRequested(Request, Context);
    if (!Applied) return {};
    return FDecision{EStage::Proposed, Request, Context.RegistryVersion, Context.AuthorityTimelineRevision, Context.BeforeWorldPublication, Context.World, Context.Targets, Context.Processing, *Applied, {}};
}
inline bool MatchesTrusted(const FDecision& D, const FTrustedContext& C)
{
    const auto Applied = EvaluateRequested(D.Original, C);
    return Applied && D.RegistryVersion == C.RegistryVersion && D.AuthorityTimelineRevision == C.AuthorityTimelineRevision
        && D.BeforeWorldPublication == C.BeforeWorldPublication && D.World == C.World && D.Targets == C.Targets
        && D.Processing == C.Processing && D.Applied == *Applied;
}
inline bool StructurallyValid(const FDecision& D)
{
    if (!D.World.Valid() || !D.AuthorityTimelineRevision || !D.BeforeWorldPublication || !Valid(D.Original) || D.Original.Kind != EKind::OwnerInput || !ValidTargets(D.Targets)
        || D.Processing.BindingFingerprint.empty() || D.Processing.BindingFingerprint.size() > MaxFingerprint
        || bool(D.Processing.Before) != (D.Processing.Origin == EOrigin::Device)
        || (D.Processing.Origin != EOrigin::Device && D.Processing.Origin != EOrigin::ExactScenario && D.Processing.Origin != EOrigin::AI)) return false;
    if (D.Processing.Origin != EOrigin::Device && std::any_of(D.Processing.Step.begin(), D.Processing.Step.end(), [](auto S) { return S != 0; })) return false;
    return D.Stage == EStage::Proposed ? D.Publication.Empty() : D.Stage == EStage::Published && D.Publication.Complete();
}
inline std::optional<std::vector<std::uint8_t>> EncodeDecision(const FDecision& D)
{
    if (!StructurallyValid(D)) return {};
    auto Original = Encode(D.Original); if (!Original) return {};
    std::vector<std::uint8_t> B; B.reserve(256 + Original->size() + D.Processing.BindingFingerprint.size() + 16 * D.Targets.size());
    Wire::Put(B, 0x31444349, 4); Wire::Put(B, 2, 2); Wire::Put(B, std::uint8_t(D.Stage), 1); Wire::Put(B, std::uint8_t(D.Processing.Origin), 1);
    Wire::Put(B, D.RegistryVersion, 8); Wire::Put(B, D.AuthorityTimelineRevision, 8); Wire::Put(B, D.BeforeWorldPublication, 8); Wire::Put(B, D.World.Id, 8); Wire::Put(B, D.World.Incarnation, 8); Wire::Put(B, Original->size(), 4); B.insert(B.end(), Original->begin(), Original->end());
    Wire::Put(B, D.Targets.size(), 2);
    for (const auto& T : D.Targets) { Wire::Put(B, T.Actor, 8); Wire::Put(B, T.Incarnation, 8); }
    Wire::Put(B, D.Processing.BindingFingerprint.size(), 4);
    B.insert(B.end(), D.Processing.BindingFingerprint.begin(), D.Processing.BindingFingerprint.end());
    for (auto V : D.Processing.Step) Wire::Put(B, V, 2);
    Wire::Put(B, D.Processing.Before.has_value(), 1);
    if (D.Processing.Before) for (auto V : *D.Processing.Before) Wire::Put(B, std::uint16_t(V), 2);
    for (auto V : D.Applied) Wire::Put(B, std::uint16_t(V), 2);
    for (auto V : {D.Publication.AuthorityPeer, D.Publication.AuthorityEpoch, D.Publication.Serial, D.Publication.WorldStateHash}) Wire::Put(B, V, 8);
    if (B.size() > MaxPayload) return {};
    return B;
}
inline std::optional<FDecision> DecodeDecision(const std::vector<std::uint8_t>& B)
{
    if (B.size() > MaxPayload || B.size() < 52) return {};
    Wire::FReader R{B.data(), B.size()};
    if (R.Get(4) != 0x31444349 || R.Get(2) != 2) return {};
    FDecision D; D.Stage = EStage(R.Get(1)); D.Processing.Origin = EOrigin(R.Get(1)); D.RegistryVersion = R.Get(8);
    D.AuthorityTimelineRevision = R.Get(8); D.BeforeWorldPublication = R.Get(8); D.World = {R.Get(8), R.Get(8)};
    const auto Size = R.Get(4); if (!R.Good || Size > MaxDatagram || Size > B.size() - R.At) return {};
    auto P = Decode(B.data() + R.At, std::size_t(Size)); if (!P) return {}; D.Original = std::move(*P); R.At += std::size_t(Size);
    const auto Targets = R.Get(2); if (!Targets || Targets > MaxTargets) return {};
    D.Targets.reserve(std::size_t(Targets));
    for (U64 I = 0; I < Targets; ++I) D.Targets.push_back({R.Get(8), R.Get(8)});
    const auto Fingerprint = R.Get(4); if (!R.Good || !Fingerprint || Fingerprint > MaxFingerprint || Fingerprint > B.size() - R.At) return {};
    D.Processing.BindingFingerprint.assign(B.begin() + R.At, B.begin() + R.At + std::size_t(Fingerprint)); R.At += std::size_t(Fingerprint);
    for (auto& S : D.Processing.Step) S = std::uint16_t(R.Get(2));
    const auto Before = R.Get(1); if (Before > 1) return {};
    const auto Signed = [&]() { const auto U = R.Get(2); return std::int16_t(U < 32768 ? int(U) : int(U) - 65536); };
    if (Before) { D.Processing.Before.emplace(); for (auto& V : *D.Processing.Before) V = Signed(); }
    for (auto& V : D.Applied) V = Signed();
    D.Publication = {R.Get(8), R.Get(8), R.Get(8), R.Get(8)};
    if (!R.Good || R.At != B.size() || !StructurallyValid(D)) return {};
    return D;
}
// Real world publisher/receiver must implement this against its actual complete
// publication ledger and trusted connection. The default never certifies a
// source Poll, proposal, UDP ACK or an input-only completed owner receipt.
class IPublishedWorldEvidence
{
public:
    virtual ~IPublishedWorldEvidence() = default;
    virtual bool Confirms(const FDecision&, const std::vector<std::uint8_t>&) const noexcept { return false; }
};
inline std::shared_ptr<const FDecision> AcceptPublished(U64 TrustedSender, const FDecision& D,
    const FTrustedContext& Context, const IPublishedWorldEvidence& Evidence)
{
    // The caller may legally own mutable aliases to its arguments. Retain our
    // private immutable copies BEFORE any virtual/reentrant evidence callback.
    const auto Pinned = std::make_shared<const FDecision>(D);
    const FTrustedContext Trusted = Context;
    if (Pinned->Stage != EStage::Published || TrustedSender != Trusted.Binding.ServerPeer
        || !Trusted.AuthorityEpoch || Pinned->Publication.AuthorityEpoch != Trusted.AuthorityEpoch
        || Pinned->Publication.AuthorityPeer != TrustedSender || !MatchesTrusted(*Pinned, Trusted)) return {};
    auto Bytes = EncodeDecision(*Pinned); if (!Bytes || !Evidence.Confirms(*Pinned, *Bytes)) return {};
    return Pinned; // never reread caller-owned D after the callback
}
}
