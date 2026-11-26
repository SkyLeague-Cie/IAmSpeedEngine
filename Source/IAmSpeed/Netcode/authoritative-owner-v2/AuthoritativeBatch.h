#pragma once
#include "AuthoritativeDecisionOwner.h"
namespace Speed::Netcode::Authoritative
{
struct FAuthorityBatchAddress
{
    U64 AuthorityPeer = 0, AuthorityEpoch = 0, AuthorityTimelineRevision = 0;
    U64 Frame = 0, RegistryVersion = 0, BeforeWorldPublication = 0, PublicationSerial = 0, WorldStateHash = 0;
    C::FWorldIdentity World;
};
inline bool LocalEntryMatchesOrigin(const C::FPreparedAuthoritativeInput& E)
{
    if (!E.AcceptedDecision || !E.OriginContext || !E.LocalSnapshot) return false;
    const auto& S = *E.LocalSnapshot; const auto& D = *E.AcceptedDecision; const auto& P = D.Original;
    const auto& I = S.Input.GetData(); const auto Contract = S.Input.GetContract();
    if (!Contract || !S.Input.IsValidFor(*Contract) || S.BindingFingerprint.empty()
        || C::Policies(*Contract) != E.OriginContext->Actions
        || C::OriginBindingFingerprintFor(*Contract, *E.OriginContext) != D.Processing.BindingFingerprint
        || S.Requested != P.Input.Values || S.Applied != D.Applied || S.CutoffSequence != P.Input.SourceSequence
        || I.ConsumptionFrame != P.Frame || I.SourceSequence != P.Input.SourceSequence || I.StreamEpoch.Value != P.Epoch
        || I.DeviceGeneration.Value != P.Input.DeviceGeneration || I.Producer.Id != P.Input.ProducerId
        || std::uint8_t(I.Producer.Kind) != P.Input.ProducerKind || I.Values != P.Input.Values
        || I.ActiveMask != P.Input.ActiveMask || I.Reset != P.Input.Reset || I.Transitions.size() != P.Input.Edges.size()) return false;
    for (std::size_t N = 0; N < I.Transitions.size(); ++N)
    {
        const auto& A = I.Transitions[N]; const auto& B = P.Input.Edges[N];
        if (A.Action != B.Action || std::uint8_t(A.State) != B.State || A.ValueAtTransition != B.Value
            || A.Order.Sequence != B.SourceSequence || A.Order.WithinSequence != B.Ordinal) return false;
    }
    return true;
}
// Exact complete INPUT source set only, not a full actor roster or physical
// restore. A real coordinator supplies this from its complete registry/world.
inline bool ValidateAuthoritativeInputBatch(const FAuthorityBatchAddress& A, const std::vector<U64>& Sessions,
    const std::vector<std::shared_ptr<const C::FPreparedAuthoritativeInput>>& Entries)
{
    if (!A.World.Valid() || !A.AuthorityPeer || !A.AuthorityEpoch || !A.AuthorityTimelineRevision || !A.RegistryVersion || !A.BeforeWorldPublication
        || !A.PublicationSerial || A.PublicationSerial <= A.BeforeWorldPublication || !A.WorldStateHash
        || Sessions.empty() || Sessions.size() > C::MaxTargets || Entries.size() != Sessions.size()) return false;
    for (std::size_t I = 0; I < Sessions.size(); ++I)
    {
        if (!Sessions[I] || (I && Sessions[I-1] >= Sessions[I]) || !Entries[I] || !Entries[I]->AcceptedDecision || !Entries[I]->OriginContext || !Entries[I]->LocalSnapshot) return false;
        const auto& D = *Entries[I]->AcceptedDecision;
        if (Entries[I]->OriginContext->AuthorityEpoch != A.AuthorityEpoch
            || Entries[I]->OriginContext->Binding.ServerPeer != A.AuthorityPeer) return false;
        if (D.World != A.World || D.Stage != C::EStage::Published || D.Original.Session != Sessions[I] || D.Original.Frame != A.Frame || D.RegistryVersion != A.RegistryVersion
            || D.AuthorityTimelineRevision != A.AuthorityTimelineRevision || D.BeforeWorldPublication != A.BeforeWorldPublication
            || D.Publication.AuthorityPeer != A.AuthorityPeer || D.Publication.AuthorityEpoch != A.AuthorityEpoch
            || D.Publication.Serial != A.PublicationSerial || D.Publication.WorldStateHash != A.WorldStateHash
            || !C::MatchesTrusted(D, *Entries[I]->OriginContext) || !LocalEntryMatchesOrigin(*Entries[I])) return false;
    }
    return true;
}
}
