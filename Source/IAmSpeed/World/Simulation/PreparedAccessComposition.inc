// Distinct world2775 derivative; no provider, complete native config closure or
// admission is supplied. Default declarations/binding reject the new path.
namespace Speed
{
bool FSimulationWorld::CollectPreparedAccess(std::vector<PreparedAccess::FParticipantAccess>& Out) const
{
    if (!PreparedRestore || !PreparedRestore->Publication || !PreparedRestore->Storage || !PreparedRestore->LeaseOwner) return false;
    auto& P = *PreparedRestore; Out.clear();
    const auto Span = [](const void* V, std::size_t N)
    { return PreparedAccess::FSpan{reinterpret_cast<std::uintptr_t>(V), N}; };
    PreparedAccess::FParticipantAccess Own;
    Own.Participant = reinterpret_cast<std::uintptr_t>(&P); // process-local PLAN id, never actor/wire identity
    Own.Writes = {Span(&ContactStorage, sizeof(ContactStorage)), Span(&CurrentStepFrame, sizeof(CurrentStepFrame)),
        Span(&LastPreparedRestoreGeneration, sizeof(LastPreparedRestoreGeneration)), Span(&LastPreparedRestorePublication, sizeof(LastPreparedRestorePublication)),
        Span(&P.Storage, sizeof(P.Storage)), Span(&P.Ready, sizeof(P.Ready)), Span(&P.Finalized, sizeof(P.Finalized)),
        Span(&P.Released, sizeof(P.Released)), Span(&P.Calling, sizeof(P.Calling)),
        Span(&PreparedRestoreEntryActive, sizeof(PreparedRestoreEntryActive))};
    const auto Read = [&Own, &Span](const auto& Values)
    {
        if (!Values.empty()) Own.ReadOnly.push_back(Span(Values.data(), sizeof(Values[0]) * Values.size()));
    };
    const auto& Envelope = *P.Publication->Envelope; const auto& Physical = *P.Publication->Physical;
    Own.ReadOnly = {Span(&Envelope, sizeof(Envelope)), Span(&Physical, sizeof(Physical)), Span(P.Storage.get(), sizeof(*P.Storage))};
    Read(Envelope.Actors); Read(Envelope.Endpoints); Read(Envelope.ActorSupplements); Read(Envelope.WorldPayload);
    for (const auto& S : Envelope.ActorSupplements) { Read(S.MechanicalExtension); Read(S.GameplayPayload); }
    Read(Physical.Bodies); Read(Physical.Dynamic); Read(Physical.Pending);
    for (const auto& B : Physical.Bodies) Read(B.MechanicPayload);
    // New TArray values are immutable during the tail. Native TMap ownership/
    // backing allocation closure still requires actual native qualification;
    // this source does not infer that from a TMap header or from fixture maps.
    for (const auto* Values : {&P.Storage->DynamicContactPairs, &P.Storage->PendingRollingContactPairs})
        if (!Values->IsEmpty()) Own.ReadOnly.push_back(Span(Values->GetData(), sizeof(FDynamicContactPair) * static_cast<std::size_t>(Values->Num())));
    Out.push_back(std::move(Own));
    if (!P.LeaseOwner->AppendPreparedAccess(Out) || !P.Supplement->AppendPreparedAccess(Out)) return false;
    for (const auto& C : P.Components) if (!C->AppendPreparedAccess(Out)) return false;
    return PreparedAccess::ValidateGlobalAccess(Out);
}
}
