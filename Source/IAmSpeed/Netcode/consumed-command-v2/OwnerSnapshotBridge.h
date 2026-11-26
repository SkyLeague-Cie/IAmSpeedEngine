#pragma once
#include "ConsumedCommand.h"
#include "IAmSpeed/Input/SameFrameInputOwner.h"

namespace Speed::Netcode::Consumed
{
namespace V2 = Input::V2;
inline std::array<FActionPolicy, ActionCount> Policies(const V2::FInputActionContract& C)
{
    std::array<FActionPolicy, ActionCount> Result{};
    for (std::size_t I = 0; I < ActionCount; ++I)
        if (const auto* A = C.Find(Input::FActionId(I))) Result[I] = {A->Wiring == V2::EActionWiring::Wired,
            A->Type == V2::EActionType::Axis1D, A->Signed, A->Quantization};
    return Result;
}
// Use this with one coherent V2 source set. The bridge does NOT install the
// snapshot in a live owner, mint a completed receipt or open ExactRemote.
inline std::optional<V2::FInputFrame> OriginalFrameToV2(const FPacket& P,
    std::shared_ptr<const V2::FInputActionContract> Contract)
{
    if (!Contract || !Valid(P) || P.Kind != EKind::OwnerInput) return {};
    V2::FInputFrameData I;
    I.ConsumptionFrame = P.Frame; I.SourceSequence = P.Input.SourceSequence; I.StreamEpoch.Value = P.Epoch;
    I.DeviceGeneration.Value = P.Input.DeviceGeneration; I.Producer = {Input::EProducerKind(P.Input.ProducerKind), P.Input.ProducerId};
    I.Values = P.Input.Values; I.ActiveMask = P.Input.ActiveMask; I.Reset = P.Input.Reset;
    for (const auto& E : P.Input.Edges) I.Transitions.push_back({E.Action, V2::ETransition(E.State), E.Value, {E.SourceSequence, E.Ordinal}});
    V2::FInputFrame Frame(Contract, std::move(I));
    if (!Frame.IsValidFor(*Contract)) return {};
    return Frame;
}
inline std::optional<V2::FInputFrame> OriginalToV2(const FDecision& D, std::shared_ptr<const V2::FInputActionContract> Contract)
{ if (!StructurallyValid(D)) return {}; return OriginalFrameToV2(D.Original, std::move(Contract)); }
inline V2::FContractFingerprint OriginBindingFingerprintFor(const V2::FInputActionContract& Contract, const FTrustedContext& C)
{
    auto F = Contract.GetFingerprint();
    const auto Add = [&](U64 V) { for (unsigned I = 0; I < 8; ++I) F.push_back(std::uint8_t(V >> (8 * I))); };
    // Same canonical source-owner schema as frozen FingerprintFor. Registry
    // origin adapter ID is the session ID, not the destination receiver owner.
    Add(2); Add(C.Binding.Session); Add(C.Binding.SourceKind); Add(C.Binding.SourceProducer); Add(C.Binding.Epoch);
    Add(std::uint8_t(C.Processing.Origin)); for (auto Step : C.Processing.Step) Add(Step);
    return F;
}
struct FPreparedAuthoritativeInput
{
    std::shared_ptr<const FDecision> AcceptedDecision;
    std::shared_ptr<const FTrustedContext> OriginContext;
    std::shared_ptr<const V2::FOwnerInputSnapshot> LocalSnapshot;
    // This is a prepared value, never a completed local receipt. Keep the full
    // origin decision/Before/policy/publication alongside the receiver snapshot.
};
inline std::shared_ptr<const FPreparedAuthoritativeInput> PrepareLocalDecisionEntry(
    U64 TrustedSender, const FDecision& D, const FTrustedContext& Context,
    const IPublishedWorldEvidence& Evidence, std::shared_ptr<const V2::FInputActionContract> Contract,
    const V2::FContractFingerprint& DestinationBinding)
{
    const auto OriginContext = std::make_shared<const FTrustedContext>(Context);
    const V2::FContractFingerprint LocalBinding = DestinationBinding;
    if (LocalBinding.empty() || !Contract || OriginContext->Actions != Policies(*Contract)) return {};
    const auto Accepted = AcceptPublished(TrustedSender, D, *OriginContext, Evidence); if (!Accepted) return {};
    if (Accepted->Processing.BindingFingerprint != OriginBindingFingerprintFor(*Contract, *OriginContext)) return {};
    auto Original = OriginalToV2(*Accepted, std::move(Contract)); if (!Original) return {};
    auto Snapshot = std::make_shared<const V2::FOwnerInputSnapshot>(V2::FOwnerInputSnapshot{
        *Original, LocalBinding, Original->GetData().Values, Accepted->Applied, Original->GetData().SourceSequence});
    return std::make_shared<const FPreparedAuthoritativeInput>(FPreparedAuthoritativeInput{Accepted, OriginContext, std::move(Snapshot)});
    // Applied is COPIED ONCE, no Process/mapper/raw source or lifecycle call.
    // Native owner/registry/world must still prepare/install/step/publish this
    // local proposal before retaining a completed LOCAL receipt.
}
}
