// Included by the DERIVED SimulationWorld.cpp only. Native source-only:
// NOT_COMPILED / NOT_RUN / NOT_ADMITTED. No production lease or complete
// component/global gameplay participant has been implemented or qualified.
#include <thread>
namespace Speed
{
// Worker-only transition gate covers argument cleanup and plan destruction as
// well as callbacks. It stays set after unique_ptr::reset clears its pointer.
struct FPreparedRestoreEntryScope final
{
    bool& Active;
    explicit FPreparedRestoreEntryScope(bool& A) noexcept : Active(A) { if (Active) std::terminate(); Active = true; }
    ~FPreparedRestoreEntryScope() noexcept { Active = false; }
    FPreparedRestoreEntryScope(const FPreparedRestoreEntryScope&) = delete;
    FPreparedRestoreEntryScope& operator=(const FPreparedRestoreEntryScope&) = delete;
};
struct FSimulationWorld::FPreparedWorldRestore
{
    std::shared_ptr<const uint8> Identity;
    std::shared_ptr<const FPreparedRestorePublication> Publication;
    std::unique_ptr<FPreparedRestoreLeaseOwner> LeaseOwner;
    std::vector<PreparedAccess::FParticipantAccess> Access;
    std::unique_ptr<IPreparedSimulationWorldSupplement> Supplement;
    std::vector<std::unique_ptr<IPreparedSimulationComponentRestore>> Components;
    std::unique_ptr<FContactStorage> Storage;
    std::thread::id Worker = std::this_thread::get_id();
    bool Ready = false, Finalized = false, Released = false, Calling = false;
    ~FPreparedWorldRestore() noexcept
    {
        if (Released) return;
        if (Finalized) std::terminate(); // no undo exists after the no-fail tail starts
        if (LeaseOwner) LeaseOwner->BeginAbort();
        for (auto I = Components.rbegin(); I != Components.rend(); ++I) (*I)->AbortBeforeFinalize();
        if (Supplement) Supplement->AbortBeforeFinalize();
        if (LeaseOwner) LeaseOwner->FinishAbort();
    }
};
FSimulationWorld::FSimulationWorld() : ContactStorage(std::make_unique<FContactStorage>()) {}
FSimulationWorld::~FSimulationWorld()
{
    // Native lifetime ownership must still prevent deleting the world inside
    // a callback; never silently destroy a plan whose lease query is active.
    if (PreparedRestoreEntryActive || (PreparedRestore && PreparedRestore->LeaseOwner
        && PreparedRestore->LeaseOwner->IsCallbackActive())) std::terminate();
    PreparedRestoreEntryActive = true;
    PreparedRestore.reset();
}
bool FSimulationWorld::MatchesPreparedRestore(const FPreparedRestoreToken& T) const noexcept
{ return !PreparedRestoreEntryActive && PreparedRestore && PreparedRestore->Worker == std::this_thread::get_id()
    && !PreparedRestore->Calling && (!PreparedRestore->LeaseOwner || !PreparedRestore->LeaseOwner->IsCallbackActive()) && T.Identity && T.Identity == PreparedRestore->Identity; }

std::optional<FSimulationWorld::FPreparedRestoreToken> FSimulationWorld::PrepareSnapshotRestore(
    const FSimulationSnapshot& Snapshot,
    std::shared_ptr<const PreparedWorld::FRestoreEnvelope> Envelope,
    std::unique_ptr<ISimulationRestoreBoundaryLease> Lease,
    std::unique_ptr<IPreparedSimulationWorldSupplement> Supplement)
{
    // Plans/leases must have idempotent aborting RAII destruction before commit;
    // even rejecting the arguments below must not leak a reservation or fence.
    if (IsRestoreBoundaryReserved()) return {};
    FPreparedRestoreEntryScope Entry(PreparedRestoreEntryActive);
    // Function parameters outlive ordinary locals: explicitly close their
    // incoming ownership while the entry gate is still active, even on reject.
    struct FArgumentCleanup final
    {
        decltype(Envelope)& E; decltype(Lease)& L; decltype(Supplement)& S;
        ~FArgumentCleanup() noexcept { S.reset(); L.reset(); E.reset(); }
    } Arguments{Envelope, Lease, Supplement};
    if (!Envelope || !Lease || !Supplement) return {};
    try
    {
        auto Plan = std::make_unique<FPreparedWorldRestore>();
        Plan->Supplement = std::move(Supplement);
        // Private immutable copy. Never retain a caller's mutable alias.
        auto Pinned = std::make_shared<const PreparedWorld::FRestoreEnvelope>(*Envelope);
        if (!PreparedWorld::ValidEnvelope(*Pinned)) return {};
        Plan->LeaseOwner = std::unique_ptr<FPreparedRestoreLeaseOwner>(
            new FPreparedRestoreLeaseOwner(*this, Pinned->Address, std::move(Lease)));
        // From this point the external lease already pins this world and ALL
        // native/config/lifecycle identities and excludes every mutation lane.
        // Install reservation BEFORE invoking component preparation callbacks.
        Plan->Identity = std::make_shared<const uint8>(uint8{0});
        PreparedRestore = std::move(Plan);
        PreparedRestore->Calling = true;
        auto Fail = [this]() -> std::optional<FPreparedRestoreToken> { PreparedRestore.reset(); return {}; };
        if (!PreparedRestore->LeaseOwner->View().IsExactAddress(Pinned->Address)) return Fail();
        if (bOrderDirty || Pinned->Address.ResumeGeneration <= LastPreparedRestoreGeneration
            || Pinned->Address.PublicationSerial <= LastPreparedRestorePublication
            || Snapshot.NumFrame != Pinned->Address.Frame
            || Snapshot.InputJournalHash != Pinned->Address.InputHash
            || Snapshot.StateHash != Pinned->Address.StateHash
            || Snapshot.Payload.IsEmpty() || Bodies.Num() != static_cast<int32>(Pinned->Actors.size())
            || SolidSubBodiesByStableId.Num() != static_cast<int32>(Pinned->Endpoints.size())
            || !PreparedRestore->Supplement->BindOwnerLease(PreparedRestore->LeaseOwner->View())
            || !(PreparedRestore->Supplement->Address() == Pinned->Address)
            || PreparedRestore->Supplement->Schema() != Pinned->WorldSchema
            || PreparedRestore->Supplement->CoveredDomains() != PreparedWorld::CompleteWorldCoverage) return Fail();
        auto Parsed = PreparedWorld::Parse(Snapshot.Payload.GetData(), static_cast<std::size_t>(Snapshot.Payload.Num()),
            Pinned->Address, Pinned->Actors);
        if (!Parsed) return Fail();
        for (const auto& Binding : Pinned->Actors)
        {
            ISpeedComponent* Adapter = AdaptersByStableId.FindRef(Binding.StableId);
            if (!Adapter || FindStableId(*Adapter) != Binding.StableId) return Fail();
            bool Found = false;
            for (const FSimulationBodyRecord& Body : Bodies)
                if (Body.StableId == Binding.StableId && Body.Adapter == Adapter) { Found = true; break; }
            if (!Found) return Fail();
        }
        for (const auto& Endpoint : Pinned->Endpoints)
        {
            USolidSubBody* Body = SolidSubBodiesByStableId.FindRef(Endpoint.StableId);
            if (!Body || Body->GetUniqueID() != Endpoint.NativeObjectId
                || FindStableSubBodyId(Body) != Endpoint.StableId || !Body->GetParentComponent()
                || FindStableId(*Body->GetParentComponent()) != Endpoint.Actor) return Fail();
        }
        auto Indexes = PreparedWorld::BuildContactIndexes(*Parsed, Pinned->Actors, Pinned->Endpoints);
        if (!Indexes) return Fail();
        PreparedRestore->Storage = std::make_unique<FContactStorage>();
        auto& Storage = *PreparedRestore->Storage;
        const auto StagePairs = [this](const std::vector<PreparedWorld::FPair>& Source,
            const std::vector<uint64>& Keys, TArray<FDynamicContactPair>& Target)
        {
            Target.Reserve(static_cast<int32>(Source.size()));
            for (std::size_t I = 0; I < Source.size(); ++I)
            {
                const auto& P = Source[I]; FDynamicContactPair Out;
                Out.BodyA = SolidSubBodiesByStableId.FindRef(P.A); Out.BodyB = SolidSubBodiesByStableId.FindRef(P.B);
                Out.PairKey = Keys[I];
                Out.LocalAnchorA = FVector(P.Geometry[0], P.Geometry[1], P.Geometry[2]);
                Out.LocalAnchorB = FVector(P.Geometry[3], P.Geometry[4], P.Geometry[5]);
                Out.LocalNormalB = FVector(P.Geometry[6], P.Geometry[7], P.Geometry[8]);
                Out.LastCOMA = FVector(P.Geometry[9], P.Geometry[10], P.Geometry[11]);
                Out.LastCOMB = FVector(P.Geometry[12], P.Geometry[13], P.Geometry[14]);
                Out.FirstSeenFrame = P.History[0]; Out.LastSeenFrame = P.History[1]; Out.LastSolvedFrame = P.History[2];
                Out.SupportedFrameCount = P.History[3]; Out.ActiveContactFrameCount = P.History[4];
                Out.ReactionFrameCount = P.History[5]; Out.FeatureTransitionCount = P.History[6];
                Out.AcquisitionNormalSpeed = P.Activity[0]; Out.AccumulatedNormalImpulse = P.Activity[1];
                Out.AccumulatedNormalActivity = P.Activity[2]; Out.AccumulatedNormalSupportActivity = P.Activity[3];
                Out.PeakNormalActivity = P.Activity[4]; Out.PeakNormalSupportActivity = P.Activity[5];
                Out.MinimumSeparation = P.Activity[6]; Out.MaximumSeparation = P.Activity[7];
                Out.bRollingManifoldReady = P.Rolling; Out.bActiveContactLastSolve = P.Active;
                Target.Add(std::move(Out));
            }
        };
        StagePairs(Parsed->Dynamic, Indexes->DynamicKeys, Storage.DynamicContactPairs);
        StagePairs(Parsed->Pending, Indexes->PendingKeys, Storage.PendingRollingContactPairs);
        // Every TMap/TSet insertion occurs HERE, before live component writes.
        for (const auto& P : Indexes->Dynamic) Storage.DynamicContactPairIndices.Add(P.first, static_cast<int32>(P.second));
        for (const auto& P : Indexes->Pending) Storage.PendingRollingContactPairIndices.Add(P.first, static_cast<int32>(P.second));
        for (const auto& Body : Indexes->DynamicByBody)
        {
            auto& Keys = Storage.DynamicContactPairKeysByBody.Add(Body.first);
            for (const auto Key : Body.second) Keys.Add(Key);
        }
        auto Publication = std::make_shared<FPreparedRestorePublication>();
        Publication->Envelope = std::move(Pinned);
        Publication->Physical = std::make_shared<const PreparedWorld::FParsedWorld>(std::move(*Parsed));
        PreparedRestore->Publication = Publication;
        PreparedRestore->Components.reserve(Publication->Envelope->Actors.size());
        // Fixed stable-id order, matching future owner/registry group locking.
        for (std::size_t I = 0; I < Publication->Envelope->Actors.size(); ++I)
        {
            const auto& Binding = Publication->Envelope->Actors[I];
            const PreparedWorld::FBody* State = nullptr;
            for (const auto& Body : Publication->Physical->Bodies) if (Body.StableId == Binding.StableId) { State = &Body; break; }
            if (!State) return Fail();
            SKinematic K;
            K.Location = FVector(State->Kinematic[0], State->Kinematic[1], State->Kinematic[2]);
            K.Velocity = FVector(State->Kinematic[3], State->Kinematic[4], State->Kinematic[5]);
            K.Acceleration = FVector(State->Kinematic[6], State->Kinematic[7], State->Kinematic[8]);
            K.Rotation = FQuat(State->Kinematic[9], State->Kinematic[10], State->Kinematic[11], State->Kinematic[12]);
            K.AngularVelocity = FVector(State->Kinematic[13], State->Kinematic[14], State->Kinematic[15]);
            K.AngularAcceleration = FVector(State->Kinematic[16], State->Kinematic[17], State->Kinematic[18]);
            const FSimulationComponentRestoreRequest Request{Publication->Envelope->Address,
                Publication->Envelope->ActorSupplements[I], K, State->Frozen,
                TConstArrayView<uint8>(State->MechanicPayload.data(), static_cast<int32>(State->MechanicPayload.size())), PreparedRestore->LeaseOwner->View()};
            auto Component = AdaptersByStableId.FindRef(Binding.StableId)->PrepareCompleteSimulationRestore(Request);
            if (!Component) return Fail(); // ALL frozen production adapters take this default rejection
            PreparedRestore->Components.push_back(std::move(Component));
            const auto& Stored = *PreparedRestore->Components.back();
            if (!(Stored.Address() == Publication->Envelope->Address) || !(Stored.Binding() == Binding)
                || Stored.CoveredDomains() != PreparedWorld::CompleteActorCoverage) return Fail();
        }
        if (!CollectPreparedAccess(PreparedRestore->Access)) return Fail();
        PreparedRestore->Calling = false;
        return FPreparedRestoreToken(PreparedRestore->Identity);
    }
    catch (...) { PreparedRestore.reset(); return {}; }
}
std::shared_ptr<const FSimulationWorld::FPreparedRestorePublication> FSimulationWorld::ReadPreparedRestore(
    const FPreparedRestoreToken& T) const noexcept
{ return MatchesPreparedRestore(T) ? PreparedRestore->Publication : nullptr; }
bool FSimulationWorld::PrepareRestoreFinalization(const FPreparedRestoreToken& T)
{
    if (!MatchesPreparedRestore(T) || PreparedRestore->Finalized) return false;
    if (PreparedRestore->Ready) return true;
    FPreparedRestoreEntryScope Entry(PreparedRestoreEntryActive);
    try
    {
        auto& P = *PreparedRestore;
        P.Calling = true; // BEFORE the first lease or provider callback
        if (!P.LeaseOwner->View().HoldsExactBoundary(*this, P.Publication->Envelope->Address)) { PreparedRestore.reset(); return false; }
        for (auto& C : P.Components) if (!C->PrepareFinalization()) { PreparedRestore.reset(); return false; }
        if (!P.Supplement->PrepareFinalization()) { PreparedRestore.reset(); return false; }
        for (std::size_t I = 0; I < P.Components.size(); ++I)
        {
            const auto& C = *P.Components[I];
            if (!(C.Address() == P.Publication->Envelope->Address)
                || !(C.Binding() == P.Publication->Envelope->Actors[I])
                || C.CoveredDomains() != PreparedWorld::CompleteActorCoverage) { PreparedRestore.reset(); return false; }
        }
        if (!(P.Supplement->Address() == P.Publication->Envelope->Address)
            || P.Supplement->Schema() != P.Publication->Envelope->WorldSchema
            || P.Supplement->CoveredDomains() != PreparedWorld::CompleteWorldCoverage
            || !P.LeaseOwner->View().HoldsExactBoundary(*this, P.Publication->Envelope->Address)) { PreparedRestore.reset(); return false; }
        std::vector<PreparedAccess::FParticipantAccess> Current;
        if (!CollectPreparedAccess(Current) || !PreparedAccess::SameDeclarations(P.Access, Current)) { PreparedRestore.reset(); return false; }
        P.LeaseOwner->SetReady(); P.Calling = false; P.Ready = true; return true;
    }
    catch (...) { PreparedRestore.reset(); return false; }
}
bool FSimulationWorld::AbortPreparedRestore(const FPreparedRestoreToken& T) noexcept
{
    if (!MatchesPreparedRestore(T) || PreparedRestore->Finalized) return false;
    FPreparedRestoreEntryScope Entry(PreparedRestoreEntryActive);
    PreparedRestore->Calling = true; PreparedRestore.reset(); return true;
}
void FSimulationWorld::FinalizePreparedRestore(const FPreparedRestoreToken& T) noexcept
{
    if (!MatchesPreparedRestore(T) || !PreparedRestore->Ready || PreparedRestore->Finalized) std::terminate();
    FPreparedRestoreEntryScope Entry(PreparedRestoreEntryActive);
    auto& P = *PreparedRestore; P.Calling = true;
    if (!P.LeaseOwner->View().HoldsExactBoundary(*this, P.Publication->Envelope->Address)) std::terminate();
    P.LeaseOwner->BeginFinalize();
    // Only opt-in prevalidated no-fail plans enter this tail. Legacy restore is
    // never called. The previous storage is retained, not destroyed, until release.
    for (auto& C : P.Components) C->Finalize();
    ContactStorage.swap(P.Storage);
    CurrentStepFrame = static_cast<unsigned int>(P.Publication->Envelope->Address.Frame);
    LastPreparedRestoreGeneration = P.Publication->Envelope->Address.ResumeGeneration;
    LastPreparedRestorePublication = P.Publication->Envelope->Address.PublicationSerial;
    P.Supplement->Finalize(); P.LeaseOwner->FinishFinalize(); P.Finalized = true; P.Calling = false;
}
void FSimulationWorld::ReleasePreparedRestoreAfterPublication(const FPreparedRestoreToken& T,
    const std::shared_ptr<const FPreparedRestorePublication>& Published) noexcept
{
    if (!MatchesPreparedRestore(T) || !PreparedRestore->Finalized || !Published
        || Published != PreparedRestore->Publication) std::terminate();
    FPreparedRestoreEntryScope Entry(PreparedRestoreEntryActive);
    auto& P = *PreparedRestore;
    P.Calling = true; P.LeaseOwner->BeginRelease();
    for (auto& C : P.Components) C->ReleaseAfterPublication();
    P.Supplement->ReleaseAfterPublication(); P.LeaseOwner->FinishRelease();
    P.Released = true; PreparedRestore.reset();
}
}
