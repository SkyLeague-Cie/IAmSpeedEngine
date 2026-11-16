#pragma once
#include "AIInputProducer.h"
#include "../Netcode/remote-input/ExactRemoteInputIngress.h"

#include "SameFrameInputOwner.h"
#include "RequestedNetworkSessionDescriptor.h"
#include "NativeRequestedLifecycleV2.h"
#include "IAmSpeed/Netcode/transport/NetworkRequestedInputSource.h"
#include "RawAcquisitionJournal.h"
#include "Testing/TestInputProducerV2.h"
#include <deque>
#include <map>
#include <set>
#include <functional>

class ASpeedPhysicsSimulation;
namespace Speed::PreparationPrefix {class FClosedSourceProducer;}
namespace Speed::Input::V2
{
struct FSessionActor
{
    std::uint64_t Id = 0, Generation = 0;
    bool operator==(const FSessionActor& R) const noexcept { return Id == R.Id && Generation == R.Generation; }
    bool operator<(const FSessionActor& R) const noexcept { return Id < R.Id; }
};
// Inert values only. No source, factory closure, UObject or platform handle.
struct FSessionDescriptor
{
    std::uint64_t Id = 0, Epoch = 0, Controller = 0, Producer = 0, Journal = 0;
    FFrameNumber First = 0;
    EProducerContract Kind = EProducerContract::Unknown;
    std::shared_ptr<const FInputActionContract> Contract;
    FInputProcessingPolicy Processing;
    std::uint32_t ResumeRearmMask = 0; // Menu-consumed Boolean actions wait for release after pause.
    std::vector<FSessionActor> Actors;
    std::vector<FInputFrame> Scenario;
    bool EnableDesktop=false;
    bool AllowScenarioAppend = false; // Test owner only, never a GT producer callback.
    std::optional<FRequestedNetworkSessionDescriptor> Network;
};
enum class EBoundaryOperation : std::uint8_t { Bind, PauseAll, Resume, Detach, RetireAll };
struct FBoundaryCommandDescriptor
{
    std::uint64_t Id = 0, WorkerGeneration = 0, RegistryVersion = 0, Session = 0, Epoch = 0, ResumeGeneration = 0;
    EBoundaryOperation Operation = EBoundaryOperation::Bind;
    FSessionDescriptor Binding;
    std::optional<FRequestedControlTargetV2> NativeV2Control;
};
enum class EBoundaryStatus : std::uint8_t { Pending, WaitingForBaseline, Applied, Rejected, TerminalFailure,ConstructedPendingClaim,ConstructedThenRetired,Retired };
// Only an explicit optimistic-version conflict is eligible for controller retry.
enum class EBoundaryRejectionReason : std::uint8_t { Unspecified, RegistryVersionMismatch };
enum class ESessionPhase : std::uint8_t { Active, Paused, WaitingBaseline, CancelPending, Retired };
struct FSessionOutcome { std::uint64_t Session = 0, Epoch = 0; ESessionPhase Phase = ESessionPhase::Active; };
// Worker-captured receipt metadata only; never part of command identity,
// admission decisions, frame hashes or replay. Bounded by the receipt limit.
struct FBoundaryBatchObservation
{
    bool Captured = false, HasWorkerFrame = false, PristineRegistry = false, InitialBindEligible = false;
    std::optional<FRawInitialOwnerAdmission> InitialJournal;
    FFrameNumber WorkerFrame = 0;
    std::uint64_t Ordinal = 0, Count = 0, Index = 0, InitialVersion = 0,
        VersionBefore = 0, PriorAppliedBinds = 0, SessionsBefore = 0;
};
struct FBoundaryReceipt
{
    std::uint64_t Id = 0, WorkerGeneration = 0, RegistryVersion = 0, ResumeGeneration = 0;
    EBoundaryStatus Status = EBoundaryStatus::Pending;
    std::vector<FSessionOutcome> Sessions;
    EBoundaryRejectionReason RejectionReason = EBoundaryRejectionReason::Unspecified;
    FBoundaryBatchObservation Boundary;
    std::optional<FRequestedLifecycleReceiptV2> NativeV2;
};
enum class ECommandAdmission : std::uint8_t { Enqueued, Duplicate, Rejected };
struct FRegistryLimits
{
    std::size_t Sessions = 16, Actors = 64, Commands = 64, Receipts = 256, Frames = HistoryCapacity;
    std::size_t DescriptorBytes = 4 * 1024 * 1024;
};

// GT-side command bridge; registry service is exclusively on its worker.
class FInputSessionCommands final
{
    friend class FInputSessionRegistry;
    friend class ::ASpeedPhysicsSimulation;
    friend class Speed::PreparationPrefix::FClosedSourceProducer;
public:
    explicit FInputSessionCommands(FRegistryLimits InLimits = {}) : Limits(InLimits), Publisher(std::this_thread::get_id()) {}
    ECommandAdmission Submit(const FBoundaryCommandDescriptor& C)
    {
        if (std::this_thread::get_id() != Publisher || !C.Id || !C.WorkerGeneration
            || C.Operation > EBoundaryOperation::RetireAll) return ECommandAdmission::Rejected;
        try
        {
            auto Bytes = Canonical(C, Limits);
            if (!Bytes) return ECommandAdmission::Rejected;
            std::lock_guard<std::mutex> Lock(Gate);
            const auto Old = Records.find(C.Id);
            if (Old != Records.end()) return Old->second.Digest == Digest(*Bytes) && Old->second.Bytes == *Bytes ? ECommandAdmission::Duplicate : ECommandAdmission::Rejected;
            const bool NativeV2=C.NativeV2Control||(C.Binding.Network&&C.Binding.Network->Version==2);
            if(NativeV2&&Queue.size()>=32)return ECommandAdmission::Rejected;
            if (C.Id <= HighId || Queue.size() >= Limits.Commands || Records.size() >= Limits.Receipts || StopRequested||EnqueueOrdinal==UINT64_MAX)
                return ECommandAdmission::Rejected;
            FRecord R{C, std::move(*Bytes), {C.Id, C.WorkerGeneration, C.RegistryVersion, C.ResumeGeneration, EBoundaryStatus::Pending, {}}};
            R.Digest = Digest(R.Bytes);
            R.EnqueueOrdinal=EnqueueOrdinal+1;
            R.Receipt.Sessions.reserve(Limits.Sessions);
            Queue.push_back(C.Id);
            try { Records.emplace(C.Id, std::move(R)); }
            catch (...) { Queue.pop_back(); throw; }
            HighId = C.Id;++EnqueueOrdinal; return ECommandAdmission::Enqueued;
        }
        catch (...) { return ECommandAdmission::Rejected; }
    }
    std::optional<FBoundaryReceipt> Read(std::uint64_t Id) const
    {
        std::lock_guard<std::mutex> Lock(Gate); const auto I = Records.find(Id);
        return I == Records.end() ? std::nullopt : std::optional<FBoundaryReceipt>(I->second.Receipt);
    }
    // The publisher explicitly releases a copied final receipt. HighId remains
    // monotone, so an acknowledged command can never be admitted a second time.
    bool Acknowledge(std::uint64_t Id)
    {
        if (std::this_thread::get_id()!=Publisher) return false;
        std::lock_guard<std::mutex> Lock(Gate);
        const auto I=Records.find(Id);
        if(I!=Records.end()&&(I->second.Command.NativeV2Control||(I->second.Command.Binding.Network&&I->second.Command.Binding.Network->Version==2)))return false;
        if (I==Records.end() || I->second.Receipt.Status==EBoundaryStatus::Pending
            || I->second.Receipt.Status==EBoundaryStatus::WaitingForBaseline) return false;
        Records.erase(I); return true;
    }
    // Caller has joined the worker and proved that no registry/producer was
    // constructed. This only cancels inert descriptors, never owner objects.
    bool CancelUnprocessedAfterJoin()
    {
        if (std::this_thread::get_id() != Publisher) return false;
        std::lock_guard<std::mutex> Lock(Gate);
        for (const auto& P : Records)
            if (P.second.Receipt.Status != EBoundaryStatus::Pending) return false;
        for (auto& P : Records) P.second.Receipt.Status = EBoundaryStatus::TerminalFailure;
        Queue.clear(); StopRequested = true; return true;
    }
    void RequestStop() { std::lock_guard<std::mutex> Lock(Gate); StopRequested = true; }
    static bool ExactPolicy(EProducerContract Kind, const FInputProcessingPolicy& P) noexcept
    {
        if (Kind != EProducerContract::ExactScenario && Kind != EProducerContract::ExactRemote && Kind != EProducerContract::AI) return false;
        return std::all_of(P.Step.begin(), P.Step.end(), [](auto S) { return S == 0; });
    }
private:
    static std::uint64_t Digest(const std::vector<std::uint8_t>& Bytes) noexcept
    { std::uint64_t H = 14695981039346656037ull; for (const auto B : Bytes) { H ^= B; H *= 1099511628211ull; } return H; }
    struct FRecord { FBoundaryCommandDescriptor Command; std::vector<std::uint8_t> Bytes; FBoundaryReceipt Receipt; std::uint64_t Digest = 0,EnqueueOrdinal=0; };
    static std::optional<std::vector<std::uint8_t>> Canonical(const FBoundaryCommandDescriptor& C, const FRegistryLimits& L)
    {
        if (!L.Sessions || !L.Actors || !L.Commands || !L.Receipts || !L.Frames || !L.DescriptorBytes) return {};
        if (C.Binding.Actors.size() > L.Actors || C.Binding.Scenario.size() > L.Frames) return {};
        std::vector<std::uint8_t> B;
        const auto Add = [&](std::uint64_t V) { if (B.size() > L.DescriptorBytes || L.DescriptorBytes - B.size() < 8) throw std::length_error("descriptor"); for (unsigned I = 0; I < 8; ++I) B.push_back(std::uint8_t(V >> (I * 8))); };
        Add(3); Add(C.Id); Add(C.WorkerGeneration); Add(C.RegistryVersion); Add(C.Session); Add(C.Epoch); Add(C.ResumeGeneration); Add(std::uint64_t(C.Operation));
        if(C.NativeV2Control)
        {
            const auto& T=*C.NativeV2Control;
            if(!T.IsValid()||C.Operation!=EBoundaryOperation::Detach||C.Binding.Network||C.Session!=T.Session||C.Epoch!=T.Epoch)return {};
            Add(0x4e455443544c32ULL);Add(2);Add(T.Session);Add(T.Epoch);Add(T.Service);Add(T.SourceGeneration);Add(T.SourceInstance);
        }
        const auto& D = C.Binding;
        if(D.Network&&(!D.Network->IsValid()||D.Kind!=EProducerContract::ExactRemote||!D.Contract
            ||D.Producer||D.Controller||D.Journal||D.EnableDesktop||D.AllowScenarioAppend||D.ResumeRearmMask
            ||!D.Scenario.empty()||D.Actors.size()!=1||D.Actors[0].Id!=D.Network->LocalComponent
            ||D.Actors[0].Generation!=D.Network->LocalComponentGeneration))return {};
        Add(D.Id); Add(D.Epoch); Add(D.Controller); Add(D.Producer); Add(D.Journal); Add(D.First); Add(std::uint64_t(D.Kind));
        Add(D.Contract ? D.Contract->GetFingerprint().size() : 0);
        if (D.Contract) for (const auto V : D.Contract->GetFingerprint()) Add(V);
        for (const auto V : D.Processing.Step) Add(V);
        Add(D.ResumeRearmMask);
        // Preserve legacy identities. Explicit Desktop adds its versioned
        // contract marker before the bounded actor count (never this value).
        if(D.EnableDesktop){Add(0x44534b544f505631ULL);Add(FDesktopPublication::ContractFingerprint);}
        // No presence byte for legacy descriptors: their canonical bytes stay
        // identical. The optional network extension has a distinct schema tag.
        if(D.Network)D.Network->AppendCanonical(Add);
        Add(D.Actors.size()); for (const auto& A : D.Actors) { Add(A.Id); Add(A.Generation); }
        Add(D.AllowScenarioAppend); Add(D.Scenario.size());
        for (const auto& F : D.Scenario)
        {
            if (!D.Contract || !F.IsValidFor(*D.Contract)) return {};
            const auto& V = F.GetData();
            Add(V.Version); Add(V.SourceSequence); Add(V.ConsumptionFrame); Add(V.Producer.Id); Add(std::uint64_t(V.Producer.Kind));
            Add(V.StreamEpoch.Value); Add(V.DeviceGeneration.Value); Add(V.ActiveMask); Add(V.Reset);
            for (const auto Value : V.Values) Add(std::uint16_t(Value));
            Add(V.Transitions.size()); for (const auto& E : V.Transitions) { Add(E.Action); Add(std::uint64_t(E.State)); Add(std::uint16_t(E.ValueAtTransition)); Add(E.Order.Sequence); Add(E.Order.WithinSequence); }
        }
        return B;
    }
    FRegistryLimits Limits;
    const std::thread::id Publisher;
    mutable std::mutex Gate;
    std::map<std::uint64_t, FRecord> Records;
    std::deque<std::uint64_t> Queue;
    std::uint64_t HighId = 0;
    std::uint64_t EnqueueOrdinal=0;
    bool StopRequested = false;
};

struct FInputRegistryView
{
    std::uint64_t Version = 0, WorkerGeneration = 0;
    bool Terminal = false;
    std::vector<FSessionDescriptor> Bindings;
    std::vector<FSessionOutcome> Phases; // Immutable boundary phase publication for independent AI controllers.
};
// Values copied from the actually bound immutable service on its native
// registry owner. Public values alone do not authenticate a native admission.
struct FRequestedNetworkBindingValues
{
    FSessionDescriptor Descriptor;
    Netcode::FNetworkInputWorkerConfiguration Configuration;
    FContractFingerprint OwnerBindingFingerprint;
};
struct FInstalledSessionInput
{
    std::uint64_t Session = 0, Epoch = 0;
    std::shared_ptr<const FOwnerInputSnapshot> Snapshot;
    std::vector<FSessionActor> Actors;
    bool AlreadyAppliedAtInitialBoundary = false;
    std::optional<FActionValues> FilterBefore;
    std::shared_ptr<const FRequestedInputConsumption> Requested;
    std::optional<FRequestedNetworkSessionDescriptor> RequestedMapping;
    std::shared_ptr<const FNativeRequestedBindingV2> NativeV2;
};
struct FRegistryFrame
{
    FFrameNumber Frame = 0;
    std::shared_ptr<const FInputRegistryView> Registry;
    std::vector<FInstalledSessionInput> Inputs;
    std::shared_ptr<const FNativeRequestedFrameAdmissionV2> NativeV2;
};
enum class ERegistryFrameFailure : std::uint8_t
{ None, PrepareAdmission, SessionInactive, OwnerPoll, PrepareException, InstallAdmission, TokenInstall, SnapshotRead, InputUnavailable };
struct FRegistryFrameFailure
{
    ERegistryFrameFailure Reason = ERegistryFrameFailure::None;
    FFrameNumber Frame = 0;
    std::uint64_t Session = 0, Epoch = 0, PollSerial = 0, RegistryVersion = 0;
    EOwnerInputStatus OwnerStatus = EOwnerInputStatus::InvalidBinding;
    EOwnerInputPhase OwnerPhase = EOwnerInputPhase::Quarantined;
    EOwnerPollFailure PollFailure = EOwnerPollFailure::None;
    bool OwnerPresent = false, OwnerStatusPresent = false;
};

// Portable owner endpoint. Its actor install/step hooks are explicit phases;
// no claim of actual engine actor writes or world restoration is made here.
class FInputSessionRegistry final
{
public:
    // Only the actual native simulation driver can issue this local admission.
    // Commands carry inert IDs, never this object, a factory, or a UObject.
    // Its identity is not a network identity or any permission to restore.
    class FNativeNetworkAdmission final
    {
        friend class FInputSessionRegistry;
        friend class ::ASpeedPhysicsSimulation;
        FNativeNetworkAdmission(std::shared_ptr<const std::uint8_t> Key,FBoundaryCommandDescriptor C,
            std::shared_ptr<Netcode::FNetworkRequestedInputService> S)
            :OwnerKey(std::move(Key)),Command(std::move(C)),Service(std::move(S)){}
        FNativeNetworkAdmission(std::shared_ptr<const std::uint8_t> Key,FBoundaryCommandDescriptor C,
            std::shared_ptr<Netcode::FNetworkRequestedInputService> S,std::shared_ptr<const FNativeRequestedAgreementSealV2> Seal)
            :OwnerKey(std::move(Key)),Command(std::move(C)),Service(std::move(S)),NativeV2(std::move(Seal)){}
        const std::shared_ptr<const std::uint8_t> OwnerKey;
        const FBoundaryCommandDescriptor Command;
        const std::shared_ptr<Netcode::FNetworkRequestedInputService> Service;
        const std::shared_ptr<const FNativeRequestedAgreementSealV2> NativeV2;
    };
    struct FJournalService { std::uint64_t Id = 0; std::shared_ptr<FRawAcquisitionJournal> Journal; std::shared_ptr<FAIInputCommands> AI; std::shared_ptr<Netcode::FExactRemoteInputIngress> Remote; };
    FInputSessionRegistry(std::shared_ptr<FInputSessionCommands> InCommands, std::uint64_t Generation,
        std::vector<FJournalService> InJournals = {}, EExactRemoteAdmission InRemoteAdmission = EExactRemoteAdmission::Closed,std::shared_ptr<const std::uint8_t> NativeOwnerKey={})
        : Commands(std::move(InCommands)), Worker(std::this_thread::get_id()), WorkerGeneration(Generation), RemoteAdmission(InRemoteAdmission), Journals(std::move(InJournals)), NativeAdmissionOwner(std::move(NativeOwnerKey))
    {
        if (!Commands || !Generation) throw std::invalid_argument("registry");
        PublishView(std::make_shared<FInputRegistryView>(FInputRegistryView{0, Generation, false, {}}));
        std::set<std::uint64_t> Ids;
        std::set<const void*> Raw;
        for (const auto& J : Journals)
            if (!J.Id || (unsigned(bool(J.Journal))+unsigned(bool(J.AI))+unsigned(bool(J.Remote)) != 1 || (J.Remote && !AdmitsOriginalDeviceRemote(RemoteAdmission))) || !Ids.insert(J.Id).second || !Raw.insert(J.Journal ? static_cast<const void*>(J.Journal.get()) : (J.AI ? static_cast<const void*>(J.AI.get()) : J.Remote.get())).second)
                throw std::invalid_argument("journal alias");
        if (!Commands->Limits.Frames) throw std::invalid_argument("history capacity");
        History.resize(Commands->Limits.Frames);
    }
    FInputSessionRegistry(const FInputSessionRegistry&) = delete;
    FInputSessionRegistry& operator=(const FInputSessionRegistry&) = delete;
    ~FInputSessionRegistry()
    {
        // Never destroy thread-affine producers implicitly on a foreign lane.
        // A coordinator may still use this registry in its no-fail tail. It
        // must explicitly release/abort before destroying the mutable endpoint;
        // retained immutable capture metadata alone is not lifetime ownership.
        if (!IsWorker() || BoundaryMutationBlocked() || !CloseWorkerLedger()) std::terminate();
    }
    #include "../RegistryResumeMethods.inc"
    #include "../GroupReplayStart.inc"
    bool CloseWorkerLedger()
    {
        if (!IsWorker() || BoundaryMutationBlocked()) return false;
        if (ReplayGroup && ReplayGroup->Finalized) return false;
        ReplayGroup.reset();
        if (ResumeGroup && ResumeGroup->Finalized) return false;
        ResumeGroup.reset();
        AbortFrame();
        // Native/game cancellation must have completed on the actual owner,
        // outside Commands->Gate, before this data-only final ledger drain.
        if(HasNativeV2Sessions())return false;
        std::lock_guard<std::mutex> Lock(Commands->Gate);
        Commands->StopRequested = true;
        const bool Retired = RetireAll();
        for (auto& P : Commands->Records)
            if (P.second.Receipt.Status == EBoundaryStatus::Pending || P.second.Receipt.Status == EBoundaryStatus::WaitingForBaseline)
            { P.second.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(P.second); }
        Commands->Queue.clear(); Waiting.reset(); Replaying = false; GroupExecutionReplay = false; ReplayPlan.clear(); GroupReplayPublished.reset(); GroupReplayReceipt.reset();
        return Retired;
    }
    // Thread-neutral acquisition journals cross the bridge, never producers or
    // native device handles. Called only at a worker boundary before Bind.
    bool RegisterJournalAtBoundary(FJournalService Service)
    {
        if (!IsWorker() || BoundaryMutationBlocked() || ResumeGroup || ReplayGroup || FrozenRequestedFrame || Pending || Terminal || !Service.Id || (unsigned(bool(Service.Journal))+unsigned(bool(Service.AI))+unsigned(bool(Service.Remote)) != 1 || (Service.Remote && !AdmitsOriginalDeviceRemote(RemoteAdmission)))) return false;
        for (const auto& J : Journals)
            if (J.Id == Service.Id || (J.Journal && J.Journal == Service.Journal) || (J.AI && J.AI == Service.AI) || (J.Remote && J.Remote == Service.Remote))
                return J.Id == Service.Id && J.Journal == Service.Journal && J.AI == Service.AI && J.Remote == Service.Remote;
        if (Journals.size() >= Commands->Limits.Sessions) return false;
        Journals.push_back(std::move(Service)); return true;
    }
    bool RegisterNetworkAdmissionAtBoundary(std::shared_ptr<const FNativeNetworkAdmission> Admission)
    {
        if(!IsWorker()||FPresentationInputScope::IsActive()||Pending||FrozenRequestedFrame||Terminal||Replaying
            ||!Admission||!NativeAdmissionOwner||Admission->OwnerKey!=NativeAdmissionOwner
            ||!Admission->Service||!ValidNetworkAdmission(*Admission))return false;
        const auto Id=Admission->Command.Id;const auto Old=NetworkAdmissions.find(Id);
        if(Old!=NetworkAdmissions.end())return Old->second==Admission;
        if(NetworkAdmissions.size()>=Commands->Limits.Sessions)return false;
        for(const auto& P:NetworkAdmissions)
            if(P.second->Service==Admission->Service
                ||P.second->Command.Binding.Network->Service==Admission->Command.Binding.Network->Service)return false;
        for(const auto& P:Sessions)
            if(P.second.NetworkAdmission&&(P.second.NetworkAdmission->Service==Admission->Service
                ||P.second.Description.Network->Service==Admission->Command.Binding.Network->Service))return false;
        NetworkAdmissions.emplace(Id,std::move(Admission));return true;
    }
    bool AppendExactScenarioFrame(std::uint64_t Session, std::uint64_t Epoch,
        std::uint64_t Controller, const FInputFrame& Frame)
    {
        if (!IsWorker() || BoundaryMutationBlocked() || ResumeGroup || ReplayGroup || Pending || Terminal || Replaying) return false;
        const auto It = Sessions.find(Session);
        if (It == Sessions.end()) return false;
        auto& S = It->second;
        return S.Description.AllowScenarioAppend && S.Description.Kind == EProducerContract::ExactScenario
            && S.Description.Epoch == Epoch && S.Description.Controller == Controller
            && S.Phase == ESessionPhase::Active && S.Owner && S.Owner->AppendExactScenarioFrame(Frame);
    }
    std::shared_ptr<const FInputRegistryView> ReadRegistry() const
    {
        if (!IsWorker()) return {};
        if (BoundaryReservation) return BoundaryReservation->Capture->CurrentBoundary();
        if (ReplayGroup) return ReplayGroup->Before;
        if (ResumeGroup) return ResumeGroup->Published ? ResumeGroup->Publication->Registry : ResumeGroup->Before;
        auto Published=std::make_shared<FInputRegistryView>(*View);
        Published->Phases.clear();
        for (const auto& P:Sessions) Published->Phases.push_back({P.first,P.second.Description.Epoch,P.second.Phase});
        return Published;
    }
    struct FFreshExactScenarioSeed
    {
        std::shared_ptr<const FInputRegistryView> Registry;
        FInputFrame Seed;
        FContractFingerprint BindingFingerprint;
    };
    // Initial exact-scenario route only. No producer lifecycle, Poll, token,
    // publication, boundary service or actor callback is performed here.
    std::optional<FFreshExactScenarioSeed> InspectFreshExactScenarioSeed(
        FFrameNumber N, const FInputRegistryView& Expected) const
    {
        if (!IsWorker() || FPresentationInputScope::IsActive() || Terminal || Pending || Waiting ||
            Replaying || Installed || Begun || Latest || InitialReceipt || HistoricalInitial ||
            Expected.Terminal || Expected.WorkerGeneration != WorkerGeneration ||
            Expected.Version != View->Version || View->Terminal ||
            Sessions.size() != 1 || View->Bindings.size() != 1 ||
            Expected.Bindings.size() != 1 || Expected.Phases.size() != 1)
            return {};
        try
        {
            std::lock_guard<std::mutex> Lock(Commands->Gate);
            if (Commands->StopRequested || !Commands->Queue.empty()) return {};
            for (const auto& H : History) if (H) return {};
            const auto& S = Sessions.begin()->second;
            const auto& D = S.Description;
            const auto& Phase = Expected.Phases.front();
            if (!S.Owner || S.Token || S.Journal || S.ResumeGeneration || S.AcquisitionFence ||
                S.Phase != ESessionPhase::Active || Phase.Phase != S.Phase ||
                Phase.Session != D.Id || Phase.Epoch != D.Epoch || D.First != N ||
                D.Kind != EProducerContract::ExactScenario || D.Actors.size() != 1 ||
                D.Scenario.size() != 1 || D.Journal || D.ResumeRearmMask ||
                !FInputSessionCommands::ExactPolicy(D.Kind, D.Processing)) return {};
            // Compare complete canonical values, not ReadRegistry's freshly
            // allocated shared_ptr or its possibly older private phase vector.
            const auto Encode = [&](const FSessionDescriptor& Binding)
            {
                FBoundaryCommandDescriptor C; C.Binding = Binding;
                return FInputSessionCommands::Canonical(C, Commands->Limits);
            };
            const auto Actual = Encode(D);
            const auto Published = Encode(View->Bindings.front());
            const auto Requested = Encode(Expected.Bindings.front());
            if (!Actual || !Published || !Requested || *Actual != *Published || *Actual != *Requested) return {};
            FOwnerInputBinding Binding{D.Id, {EProducerKind::Device, D.Producer},
                {D.Epoch}, D.Contract, D.Processing, D.Kind};
            const auto Seed = S.Owner->InspectFreshExactScenarioSeed(N, Binding);
            if (!Seed) return {};
            auto ActualSeed = D; ActualSeed.Scenario = {*Seed};
            const auto Inspected = Encode(ActualSeed);
            if (!Inspected || *Inspected != *Actual) return {};
            auto Observed = std::make_shared<FInputRegistryView>(*View);
            Observed->Phases = {{D.Id, D.Epoch, S.Phase}};
            return FFreshExactScenarioSeed{std::move(Observed), *Seed, S.Owner->GetBindingFingerprint()};
        }
        catch (...) { return {}; }
    }

    struct FPreparationSessionReadValues
    {
        std::uint64_t MapKey = 0, ResumeGeneration = 0, AcquisitionFence = 0;
        ESessionPhase Phase = ESessionPhase::CancelPending;
        bool JournalPresent = false, TokenPresent = false;
        std::vector<std::uint8_t> DescriptorCanonical;
        std::optional<FSameFrameInputOwner::FPreparationReadValues> Owner;
    };
    struct FPreparationCommandReadValues
    {
        std::uint64_t MapKey = 0, Digest = 0;
        std::vector<std::uint8_t> DescriptorCanonical;
        FBoundaryReceipt Receipt;
    };
    struct FPreparationReadValues
    {
        std::shared_ptr<const FInputRegistryView> Registry;
        std::shared_ptr<const FRegistryFrame> Latest, Initial, HistoricalInitial;
        std::vector<std::shared_ptr<const FRegistryFrame>> History;
        // Pending is mutable until InstallAll. Copy its vector of fanout
        // values while preserving the REAL immutable snapshot handles.
        // This copy's address is never a registry/frame identity.
        std::shared_ptr<const FRegistryFrame> PendingFrameCopy;
        std::vector<FPreparationSessionReadValues> Sessions;
        std::vector<std::vector<std::uint8_t>> PublishedDescriptorsCanonical;
        std::vector<FPreparationCommandReadValues> Commands;
        std::deque<std::uint64_t> Queue;
        std::map<std::uint64_t, std::uint64_t> LastEpoch;
        std::uint64_t WorkerGeneration = 0, Constructed = 0, Destroyed = 0;
        std::uint64_t CommandHighId = 0;
        std::size_t HistoryCount = 0, JournalsCount = 0;
        std::size_t ReplayPlanCount = 0, ReplayIndex = 0;
        std::optional<std::uint64_t> Waiting;
        bool Terminal = false, Installed = false, Begun = false;
        bool Replaying = false, StopRequested = false;
        FRegistryFrameFailure LastFailure;
    };
    // Pure inspection BETWEEN transitions, never from inside Commands->Gate.
    // No Poll/Install/Begin/Abort/getter on a producer, token copy, publication,
    // GT thread bypass, source authority or successful phase follows.
    std::optional<FPreparationReadValues> ReadPreparationValuesOnWorker(std::size_t MaxDescriptorBytes) const noexcept
    {
        if (!IsWorker() || FPresentationInputScope::IsActive() || !Commands) return {};
        try
        {
            std::unique_lock<std::mutex> Lock(Commands->Gate, std::try_to_lock);
            if (!Lock.owns_lock() || !View || !MaxDescriptorBytes ||
                Sessions.size() > 1 || View->Bindings.size() > 1 ||
                Commands->Records.size() > 1 || Commands->Queue.size() > 1 ||
                !Journals.empty() || !NetworkAdmissions.empty() || Replaying || !ReplayPlan.empty() ||
                History.size() > Commands->Limits.Frames) return {};
            for (const auto& H : History) if (H) return {};
            std::size_t RemainingDescriptorBytes = MaxDescriptorBytes;
            for (const auto& P : Commands->Records)
            {
                if (P.second.Bytes.size() > RemainingDescriptorBytes) return {};
                RemainingDescriptorBytes -= P.second.Bytes.size();
            }
            FPreparationReadValues R;
            R.Registry = View;
            R.Latest = Latest; R.Initial = InitialReceipt; R.HistoricalInitial = HistoricalInitial;
            R.History = History;
            for (const auto& H : History) if (H) ++R.HistoryCount;
            if (Pending) R.PendingFrameCopy = std::make_shared<const FRegistryFrame>(*Pending);
            R.WorkerGeneration = WorkerGeneration; R.Constructed = Constructed; R.Destroyed = Destroyed;
            R.Terminal = Terminal; R.Installed = Installed; R.Begun = Begun;
            R.Replaying = Replaying; R.Waiting = Waiting;
            R.ReplayPlanCount = ReplayPlan.size(); R.ReplayIndex = ReplayIndex;
            R.JournalsCount = Journals.size(); R.LastEpoch = LastEpoch; R.LastFailure = LastFrameFailure;
            R.StopRequested = Commands->StopRequested; R.CommandHighId = Commands->HighId;
            R.Queue = Commands->Queue;
            for (const auto& P : Commands->Records)
                R.Commands.push_back({P.first, P.second.Digest, P.second.Bytes, P.second.Receipt});
            const auto Encode = [&](const FSessionDescriptor& D)
            {
                FBoundaryCommandDescriptor C; C.Binding = D;
                FRegistryLimits L = Commands->Limits;
                L.DescriptorBytes = std::min(L.DescriptorBytes, RemainingDescriptorBytes);
                auto Bytes = FInputSessionCommands::Canonical(C, L);
                if (!Bytes || Bytes->size() > RemainingDescriptorBytes) return std::optional<std::vector<std::uint8_t>>{};
                RemainingDescriptorBytes -= Bytes->size();
                return Bytes;
            };
            for (const auto& D : View->Bindings)
            {
                auto Bytes = Encode(D); if (!Bytes) return {};
                R.PublishedDescriptorsCanonical.push_back(std::move(*Bytes));
            }
            for (const auto& P : Sessions)
            {
                const auto& S = P.second;
                if(S.RequestedOwner||S.NetworkAdmission)return {};
                FPreparationSessionReadValues V;
                V.MapKey = P.first; V.Phase = S.Phase;
                V.ResumeGeneration = S.ResumeGeneration; V.AcquisitionFence = S.AcquisitionFence;
                V.JournalPresent = bool(S.Journal); V.TokenPresent = bool(S.Token);
                auto Bytes = Encode(S.Description); if (!Bytes) return {};
                V.DescriptorCanonical = std::move(*Bytes);
                if (S.Owner)
                {
                    V.Owner = S.Owner->ReadPreparationValuesForRegistry(S.Token ? &*S.Token : nullptr);
                    if (!V.Owner) return {};
                }
                R.Sessions.push_back(std::move(V));
            }
            return R;
        }
        catch (...) { return {}; }
    }

    std::uint64_t ConstructionCount() const noexcept { return IsWorker() ? Constructed : 0; }
    std::uint64_t DestructionCount() const noexcept { return IsWorker() ? Destroyed : 0; }
    bool IsTerminal() const noexcept { return !IsWorker() || Terminal; }

private:
    std::uint64_t BoundaryObservationOrdinal = 0;
public:

    // Called between transactions, including while the simulation is paused.
    bool ServiceBoundary(std::optional<FFrameNumber> ActualWorkerFrame = std::nullopt,std::optional<std::uint64_t> CapturedCutoff=std::nullopt)
    {
        if (!IsWorker() || BoundaryMutationBlocked() || ResumeGroup || ReplayGroup || FrozenRequestedFrame || Pending) return false;
        std::lock_guard<std::mutex> Lock(Commands->Gate);
        if (Commands->StopRequested)
        {
            MakeTerminal();
            Replaying = false; ReplayPlan.clear();
            const bool Retired = RetireAll();
            for (auto& P : Commands->Records)
                if (P.second.Receipt.Status == EBoundaryStatus::Pending || P.second.Receipt.Status == EBoundaryStatus::WaitingForBaseline)
                { P.second.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(P.second); }
            Commands->Queue.clear(); Waiting.reset(); return Retired;
        }
        if (Replaying) return false;
        const auto Cutoff=CapturedCutoff.value_or(Commands->EnqueueOrdinal);
        std::size_t Count=0;
        for(const auto Id:Commands->Queue){if(Commands->Records.at(Id).EnqueueOrdinal>Cutoff)break;++Count;}
        const auto InitialVersion = View->Version;
        const bool Pristine = InitialVersion == 0 && Sessions.empty() && LastEpoch.empty()
            && !Latest && !InitialReceipt && !HistoricalInitial && !Waiting && !Terminal;
        // Ordinal identifies nonempty service lots; no unbounded trace buffer.
        if (Count && BoundaryObservationOrdinal != std::numeric_limits<std::uint64_t>::max()) ++BoundaryObservationOrdinal;
        std::uint64_t PriorAppliedBinds = 0;
        // Serialize only disjoint initial Device/AI Binds captured in THIS lot.
        // No DTO/digest/version/epoch/First rewrite and no command requeue.
        bool InitialBindLot = ActualWorkerFrame.has_value() && Pristine;
        for (std::size_t I = 0; I < Count; ++I)
        {
            const auto Id = Commands->Queue.front(); Commands->Queue.pop_front();
            auto& R = Commands->Records.at(Id);
            auto& O = R.Receipt.Boundary;
            O.Captured = true; O.HasWorkerFrame = ActualWorkerFrame.has_value(); O.WorkerFrame = ActualWorkerFrame.value_or(0);
            O.PristineRegistry = Pristine; O.Ordinal = BoundaryObservationOrdinal; O.Count = Count; O.Index = I;
            O.InitialVersion = InitialVersion; O.VersionBefore = View->Version;
            O.PriorAppliedBinds = PriorAppliedBinds; O.SessionsBefore = Sessions.size();
            bool InitialBind = InitialBindLot && R.Command.Operation == EBoundaryOperation::Bind
                && R.Command.RegistryVersion == InitialVersion && R.Command.WorkerGeneration == WorkerGeneration
                && (R.Command.Binding.Kind == EProducerContract::Device || R.Command.Binding.Kind == EProducerContract::AI)
                && R.Command.Binding.First == *ActualWorkerFrame;
            if (InitialBind && R.Command.Binding.Kind == EProducerContract::Device)
            {
                const auto& D = R.Command.Binding;
                const auto J = std::find_if(Journals.begin(), Journals.end(),
                    [&](const auto& Service) { return Service.Id == D.Journal && Service.Journal; });
                if (J == Journals.end()) InitialBind = false;
                else { O.InitialJournal = J->Journal->ReadInitialOwnerAdmission(); InitialBind = O.InitialJournal->CanAdmit(D.Epoch, D.First); }
            }
            O.InitialBindEligible = InitialBind;
            if (!InitialBind) InitialBindLot = false;
            if (Terminal && R.Command.Operation != EBoundaryOperation::RetireAll) { R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); ReleaseRejectedBindJournal(R); continue; }
            if (R.Command.WorkerGeneration != WorkerGeneration)
            { R.Receipt.Status = EBoundaryStatus::Rejected; ReleaseRejectedBindJournal(R); continue; }
            // Only prior successful initial Binds in this captured lot may
            // advance the version; Apply/Bind retains every identity check.
            if (R.Command.RegistryVersion != View->Version && !InitialBind)
            {
                R.Receipt.Status = EBoundaryStatus::Rejected;
                R.Receipt.RejectionReason = EBoundaryRejectionReason::RegistryVersionMismatch;
                ReleaseRejectedBindJournal(R); continue;
            }
            if (Waiting)
            {
                const auto Op = R.Command.Operation;
                const bool Supersedes = Op == EBoundaryOperation::PauseAll || Op == EBoundaryOperation::RetireAll
                    || (Op == EBoundaryOperation::Detach && R.Command.Session == Commands->Records.at(*Waiting).Command.Session
                        && R.Command.Epoch == Commands->Records.at(*Waiting).Command.Epoch);
                if (!Supersedes) { R.Receipt.Status = EBoundaryStatus::Rejected; ReleaseRejectedBindJournal(R); continue; }
                auto& Old = Commands->Records.at(*Waiting); Old.Receipt.Status = EBoundaryStatus::Rejected;
                Outcomes(Old); Waiting.reset();
            }
            try { Apply(R); }
            catch (...) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); }
            if (R.Command.Operation == EBoundaryOperation::Bind && R.Receipt.Status == EBoundaryStatus::Applied) ++PriorAppliedBinds;
            ReleaseRejectedBindJournal(R);
            if (R.Receipt.Status != EBoundaryStatus::Applied) InitialBindLot = false;
        }
        if (Waiting)
        {
            auto& R = Commands->Records.at(*Waiting);
            try { if (TryResume(R)) Waiting.reset(); }
            catch (...) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); Waiting.reset(); }
        }
        return !Terminal && !Waiting;
    }
    #include "../EmptyInputFrameReceipt.inc"
    // Complete cohort admission BEFORE any irreversible Poll. A missing exact
    // remote frame stays absent; callers may wait at the same physical boundary.
    EInputFrameReadiness QueryFrameReadiness(FFrameNumber N, std::size_t MinimumRemoteSessions = 0)
    {
        if (!IsWorker() || BoundaryMutationBlocked() || ResumeGroup || ReplayGroup || Pending || Terminal || Waiting || Replaying)
            return EInputFrameReadiness::Rejected;
        struct FGuard { bool& Flag; explicit FGuard(bool& F) : Flag(F) { Flag = true; } ~FGuard() { Flag = false; } } Guard(InspectingFrameReadiness);
        std::size_t RemoteCount = 0;
        bool Missing = false;
        for (auto& P : Sessions)
        {
            auto& S = P.second;
            if (S.RequestedOwner) return EInputFrameReadiness::Rejected;
            if (S.Description.Kind == EProducerContract::ExactRemote) ++RemoteCount;
            if (S.Phase != ESessionPhase::Active || !S.Owner) return EInputFrameReadiness::Rejected;
            const auto Ready = S.Owner->QueryFrameReadiness(N);
            if (Ready == EInputFrameReadiness::Rejected) return Ready;
            Missing = Missing || Ready == EInputFrameReadiness::Waiting;
        }
        return Missing || RemoteCount < MinimumRemoteSessions ? EInputFrameReadiness::Waiting : EInputFrameReadiness::Ready;
    }

    // Explicit pre-decision cutoff. Source selection/processing still occurs
    // in PrepareFrame after the native correction decision. No socket wait.
    bool FreezeRequestedSources(FFrameNumber N)
    {
        if(!IsWorker()||FPresentationInputScope::IsActive()||BoundaryMutationBlocked()||ResumeGroup||ReplayGroup||Terminal||Pending||Waiting||Replaying||FrozenRequestedFrame)return false;
        bool Any=false;
        for(const auto& P:Sessions)
            if(P.second.Phase!=ESessionPhase::Active||(!P.second.Owner&&!P.second.RequestedOwner))return false;
        for(auto& P:Sessions)if(auto& O=P.second.RequestedOwner)
        {
            Any=true;auto T=O->FreezeFrame(N,bool(P.second.NativeV2Binding));
            if(!T){AbortFrame();return false;}
            P.second.RequestedToken=std::move(T);
        }
        if(Any)FrozenRequestedFrame=N;return true;
    }
    // Read-only check: selection cannot create, reuse or repair a cutoff.
    bool RequestedSourcesFrozenFor(FFrameNumber N) const noexcept
    {
        if(!IsWorker()||FPresentationInputScope::IsActive()||BoundaryMutationBlocked()||ResumeGroup||ReplayGroup||Terminal||Pending||Waiting||Replaying)return false;
        bool Any=false;
        for(const auto& P:Sessions)
            if(P.second.Phase!=ESessionPhase::Active||(!P.second.Owner&&!P.second.RequestedOwner))return false;
        for(const auto& P:Sessions)if(const auto& O=P.second.RequestedOwner)
        {
            Any=true;
            if(P.second.Phase!=ESessionPhase::Active||!P.second.RequestedToken
                ||O->GetPhase()!=ERequestedOwnerPhase::Frozen||O->GetNextFrame()!=N)return false;
        }
        return Any?FrozenRequestedFrame==N:!FrozenRequestedFrame;
    }
    bool PrepareFrame(FFrameNumber N)
    {
        if (!IsWorker() || BoundaryMutationBlocked() || ResumeGroup || ReplayGroup) return false;
        LastFrameFailure = {};
        if (Terminal || Pending || Waiting || Replaying || Sessions.empty())
        { RecordFrameFailure(ERegistryFrameFailure::PrepareAdmission, N); return false; }
        if (RemoteAdmission != EExactRemoteAdmission::Closed && QueryFrameReadiness(N) != EInputFrameReadiness::Ready)
        { RecordFrameFailure(ERegistryFrameFailure::InputUnavailable, N); return false; }
        // Readiness is checked for ALL sessions before any source is touched.
        for (const auto& P : Sessions)
            if (P.second.Phase != ESessionPhase::Active || (!P.second.Owner&&!P.second.RequestedOwner))
            { RecordFrameFailure(ERegistryFrameFailure::SessionInactive, N, &P.second); return false; }
        try
        {
            auto Frame = std::make_shared<FRegistryFrame>(); Frame->Frame = N; Frame->Registry = View;
            Frame->NativeV2=FrozenNativeV2;
            Frame->Inputs.reserve(Sessions.size());
            for (auto& Pair : Sessions)
            {
                auto& S = Pair.second;
                if (S.Phase != ESessionPhase::Active || (!S.Owner&&!S.RequestedOwner))
                { RecordFrameFailure(ERegistryFrameFailure::SessionInactive, N, &S); AbortFrame(); return false; }
                if(S.RequestedOwner)
                {
                    if(FrozenRequestedFrame!=N||!S.RequestedToken||!S.RequestedOwner->PrepareFrozen(*S.RequestedToken))
                    {RecordFrameFailure(ERegistryFrameFailure::OwnerPoll,N,&S);AbortFrame();return false;}
                    Frame->Inputs.push_back({S.Description.Id,S.Description.Epoch,{},S.Description.Actors,false,{}, {},S.Description.Network,S.NativeV2Binding});
                    continue;
                }
                const bool Initial = S.Owner->HasInitialReceipt();
                auto P = S.Owner->Poll(N);
                if (!P.Token || P.Status != EOwnerInputStatus::Ready)
                { RecordFrameFailure(ERegistryFrameFailure::OwnerPoll, N, &S, P.Status); AbortFrame(); return false; }
                S.Token = std::move(P.Token);
                Frame->Inputs.push_back({S.Description.Id, S.Description.Epoch, {}, S.Description.Actors, Initial, S.Owner->ReadPreparedFilterBefore()});
            }
            Pending = std::move(Frame); return true;
        }
        catch (...) { RecordFrameFailure(ERegistryFrameFailure::PrepareException, N); AbortFrame(); return false; }
    }
    bool InstallAll()
    {
        if (!IsWorker() || BoundaryMutationBlocked()) return false;
        const auto N = Pending ? Pending->Frame : FFrameNumber{0};
        LastFrameFailure = {};
        if (!Pending || Installed || Begun || !EmptyReceiptStateValid())
        { RecordFrameFailure(ERegistryFrameFailure::InstallAdmission, N); return false; }
        std::size_t I = 0;
        for (auto& Pair : Sessions)
        {
            auto& S = Pair.second;
            if(S.RequestedOwner)
            {
                if(!S.RequestedToken||!S.RequestedOwner->Install(*S.RequestedToken))
                {RecordFrameFailure(ERegistryFrameFailure::TokenInstall,N,&S);AbortFrame();return false;}
                auto& Input=Pending->Inputs[I++];Input.Requested=S.RequestedOwner->ReadInstalled(*S.RequestedToken);
                if(!Input.Requested){RecordFrameFailure(ERegistryFrameFailure::SnapshotRead,N,&S);AbortFrame();return false;}
                Input.FilterBefore=Input.Requested->BeforeProcessing;continue;
            }
            if (!S.Token || !S.Owner->Install(*S.Token))
            { RecordFrameFailure(ERegistryFrameFailure::TokenInstall, N, &S); AbortFrame(); return false; }
            Pending->Inputs[I++].Snapshot = S.Owner->ReadInstalled(*S.Token);
            if (!Pending->Inputs[I - 1].Snapshot)
            { RecordFrameFailure(ERegistryFrameFailure::SnapshotRead, N, &S); AbortFrame(); return false; }
        }
        Installed = true; return true;
    }
    FRegistryFrameFailure ReadLastFrameFailure() const noexcept { return IsWorker() ? LastFrameFailure : FRegistryFrameFailure{}; }
    std::shared_ptr<const FRegistryFrame> ReadInstalled() const
    { return IsWorker() && Installed && !Terminal ? Pending : nullptr; }
    bool BeginAll()
    {
        if (!IsWorker() || BoundaryMutationBlocked() || !Pending || !Installed || Begun || Terminal || !EmptyReceiptStateValid()) return false;
        for (const auto& Pair : Sessions)
        {
            const auto& S=Pair.second;
            if(S.RequestedOwner?(!S.RequestedToken||S.RequestedOwner->GetPhase()!=ERequestedOwnerPhase::Installed)
                :(!S.Token||!S.Owner||S.Owner->GetPhase()!=EOwnerInputPhase::Installed)){AbortFrame();return false;}
        }
        for (auto& Pair : Sessions)
        {
            auto& S=Pair.second;
            if(!(S.RequestedOwner?S.RequestedOwner->BeginStep(*S.RequestedToken,Pending->Frame)
                :S.Owner->BeginStep(*S.Token,Pending->Frame))){AbortFrame();return false;}
        }
        Begun = true; return true;
    }
    bool CloseInitialBoundary()
    {
        // Empty steps never establish an owner Reset/baseline. Check worker
        // identity before reading this worker-owned transaction tag.
        if (!IsWorker() || BoundaryMutationBlocked() || !Pending || !Installed || Begun || Terminal || PendingEmptyReceipt || !EmptyReceiptStateValid()) return false;
        for (const auto& Pair : Sessions)
        {
            const auto& S = Pair.second;
            // A requested-network baseline must reach a real physics commit;
            // never relabel this special legacy initial-apply boundary.
            if(S.RequestedOwner)return false;
            const auto V = S.Owner->ReadInstalled(*S.Token);
            if (!V || !V->Input.GetData().Reset || S.Owner->HasInitialReceipt()) { AbortFrame(); return false; }
        }
        for (auto& Pair : Sessions)
        {
            auto& S = Pair.second;
            if (!S.Owner->CloseInitialBoundary(*S.Token)) { AbortFrame(); return false; }
            S.Token.reset();
        }
        InitialReceipt = Pending; HistoricalInitial = Pending; Pending.reset(); Installed = false; return true;
    }
    bool ValidateComplete() const noexcept
    {
        if (!IsWorker() || BoundaryMutationBlocked() || !Pending || !Begun || Terminal || !EmptyReceiptStateValid()) return false;
        for (const auto& Pair : Sessions)
        {
            const auto& S=Pair.second;
            if(S.RequestedOwner?(!S.RequestedToken||!S.RequestedOwner->CanComplete(*S.RequestedToken,Pending->Frame))
                :(!S.Token||!S.Owner||!S.Owner->CanComplete(*S.Token,Pending->Frame)))return false;
        }
        return true;
    }
    bool CompleteAll() noexcept
    {
        if (!ValidateComplete()) return false;
        for (auto& Pair : Sessions)
        {
            auto& S = Pair.second;
            if(S.RequestedOwner)
            {
                if(!S.RequestedOwner->CompleteStep(*S.RequestedToken,Pending->Frame)){MakeTerminal();return false;}
                S.RequestedToken.reset();
                if(S.NativeV2Binding)NativeV2Completed(S,Pending->Frame);
                continue;
            }
            if (!S.Owner->CompleteStep(*S.Token, Pending->Frame)) { MakeTerminal(); return false; }
            S.Token.reset();
        }
        if (GroupExecutionReplay) { GroupReplayReceipt = Pending; ++ReplayIndex; Pending.reset(); Installed = Begun = PendingEmptyReceipt = false; return true; }
        History[Pending->Frame % History.size()] = Pending;
        Latest = Pending; LastCompletedForCapture = Pending; InitialReceipt.reset(); Pending.reset(); FrozenRequestedFrame.reset(); FrozenNativeV2.reset(); Installed = Begun = PendingEmptyReceipt = false; return true;
    }
    void AbortFrame() noexcept
    {
        if (!IsWorker() || BoundaryMutationBlocked() || ResumeGroup || ReplayGroup) return;
        FrozenNativeV2.reset();
        for (auto& Pair : Sessions)
        {
            auto& S=Pair.second;if(S.Token)S.Owner->Abort(*S.Token);S.Token.reset();
            if(S.RequestedToken)S.RequestedOwner->Abort(*S.RequestedToken);S.RequestedToken.reset();
        }
        Pending.reset(); FrozenRequestedFrame.reset(); Installed = Begun = PendingEmptyReceipt = false; MakeTerminal();
    }
#include "../RegistryBoundaryCapture.inc"
#include "../RegistryBoundaryReservation.inc"
#include "../EmptyInputPeriodEvidence.inc"
    std::shared_ptr<const FRegistryFrame> ReadLatest() const { return IsWorker() && !Terminal ? Latest : nullptr; }
    std::shared_ptr<const FRegistryFrame> ReadInitial() const { return IsWorker() && !Terminal ? InitialReceipt : nullptr; }
    std::shared_ptr<const FRegistryFrame> ReadHistoricalInitial() const { return IsWorker() ? HistoricalInitial : nullptr; }
    std::uint64_t PollCount(std::uint64_t Id) const noexcept
    {
        if (!IsWorker()) return 0;
        const auto I = Sessions.find(Id); return I != Sessions.end() && I->second.Owner ? I->second.Owner->GetPollSerial() : 0;
    }
    bool HasFilterState(std::uint64_t Id) const noexcept
    {
        if (!IsWorker()) return false;
        const auto I = Sessions.find(Id); return I != Sessions.end() && I->second.Owner && I->second.Owner->HasFilterState();
    }
    std::optional<FRequestedInputFrame> ReadConfirmedRequestedInput(std::uint64_t Session,std::uint64_t Epoch,FFrameNumber N)
    {
        if(!IsWorker()||Terminal||Pending||Replaying)return {};
        const auto I=Sessions.find(Session);
        return I!=Sessions.end()&&I->second.Description.Epoch==Epoch&&I->second.RequestedOwner
            ?I->second.RequestedOwner->ReadConfirmedForReplay(N):std::nullopt;
    }
    // Historical registry and immutable input fanout only. A future world adapter
    // must restore/validate actor checkpoints before using these for real replay.
    std::shared_ptr<const FRegistryFrame> ReadHistorical(FFrameNumber N) const
    {
        if (!IsWorker()) return {};
        const auto& F = History[N % History.size()]; return F && F->Frame == N ? F : nullptr;
    }
    // Portable replay evidence cursor. No world restoration is claimed: callers
    // receive the historical registry/actor set to restore before engine replay.
    bool BeginRetainedReplay(FFrameNumber First, FFrameNumber Last, const std::vector<FSessionActor>& AvailableActors)
    {
        if (!IsWorker() || BoundaryMutationBlocked() || ResumeGroup || ReplayGroup || Terminal || Pending || FrozenRequestedFrame || Waiting || Replaying || First > Last || Last - First >= History.size()) return false;
        try
        {
            std::vector<std::shared_ptr<const FRegistryFrame>> Plan;
            for (auto N = First;; ++N)
            {
                auto F = ReadHistorical(N); if (!F) return false;
                for (const auto& Input : F->Inputs)
                {
                    for (const auto& A : Input.Actors)
                        if (std::find(AvailableActors.begin(), AvailableActors.end(), A) == AvailableActors.end()) return false;
                    const auto B = std::find_if(F->Registry->Bindings.begin(), F->Registry->Bindings.end(), [&](const auto& D) { return D.Id == Input.Session && D.Epoch == Input.Epoch; });
                    // This legacy replay cursor only certifies original-input
                    // history. Corrected requested replay needs a confirmed
                    // plan plus native processing-state restoration.
                    if (B == F->Registry->Bindings.end() || !Input.Snapshot || Input.Requested) return false;
                    auto Expected = Input.Snapshot->Requested;
                    if (UsesAuthoritativeDeviceProcessing(B->Kind, RemoteAdmission))
                    {
                        if (!Input.FilterBefore) return false;
                        Expected = ProcessAuthoritativeInputValues(B->Processing, Expected, *Input.FilterBefore, Input.Snapshot->Input.GetData().Reset);
                    }
                    else if (!FInputSessionCommands::ExactPolicy(B->Kind,B->Processing) || Input.FilterBefore) return false;
                    if (Expected != Input.Snapshot->Applied) return false;
                }
                Plan.push_back(std::move(F)); if (N == Last) break;
            }
            ReplayPlan = std::move(Plan); ReplayIndex = 0; Replaying = true; return true;
        }
        catch (...) { return false; }
    }
    std::shared_ptr<const FRegistryFrame> NextRetainedReplay()
    { return IsWorker() && !BoundaryMutationBlocked() && !GroupExecutionReplay && Replaying && ReplayIndex < ReplayPlan.size() ? ReplayPlan[ReplayIndex++] : nullptr; }
    bool EndRetainedReplay()
    {
        if (!IsWorker() || BoundaryMutationBlocked() || GroupExecutionReplay || !Replaying || ReplayIndex != ReplayPlan.size()) return false;
        Replaying = false; ReplayPlan.clear(); return true;
    }
private:
    friend class ::ASpeedPhysicsSimulation;
    struct FSession
    {
        FSessionDescriptor Description;
        std::unique_ptr<FSameFrameInputOwner> Owner;
        std::unique_ptr<FRequestedInputOwner> RequestedOwner;
        std::optional<FRequestedInputOwner::FToken> RequestedToken;
        std::shared_ptr<const FNativeNetworkAdmission> NetworkAdmission;
        std::shared_ptr<const FNativeRequestedBindingV2> NativeV2Binding;
        const void* NativeV2SourceAddress=nullptr;
        std::uint64_t NativeV2Command=0,NativeV2CompletedFrames=0;
        std::uint64_t NativeV2DetachCommand=0;
        ERequestedRetirementReasonV2 NativeV2Retire=ERequestedRetirementReasonV2::None;
        bool NativeV2Claimed=false;
        std::shared_ptr<FRawAcquisitionJournal> Journal;
        std::optional<FSameFrameInputOwner::FToken> Token;
        ESessionPhase Phase = ESessionPhase::Active;
        std::uint64_t ResumeGeneration = 0;
        std::uint64_t AcquisitionFence = 0;
    };
    bool CanObserveRequestedBindingsAtBoundary() const noexcept
    {
        if(!IsWorker()||FPresentationInputScope::IsActive()||Terminal||Pending||FrozenRequestedFrame||Waiting||Replaying)return false;
        bool Any=false;
        for(const auto& Pair:Sessions)if(Pair.second.Description.Network)
        {
            const auto& S=Pair.second;Any=true;
            if(S.Phase!=ESessionPhase::Active||!S.RequestedOwner||!S.NetworkAdmission
                ||S.RequestedOwner->GetPhase()!=ERequestedOwnerPhase::Idle||S.RequestedToken
                ||S.NetworkAdmission->OwnerKey!=NativeAdmissionOwner||!S.NetworkAdmission->Service)return false;
        }
        return Any;
    }
    std::vector<FRequestedNetworkBindingValues> CaptureRequestedBindingsAtBoundary() const
    {
        if(!CanObserveRequestedBindingsAtBoundary())throw std::runtime_error("requested binding observation outside stable boundary");
        std::vector<FRequestedNetworkBindingValues> Records;Records.reserve(Sessions.size());
        for(const auto& Pair:Sessions)if(Pair.second.Description.Network)
        {
            const auto& S=Pair.second;
            Records.push_back({S.Description,S.NetworkAdmission->Service->GetConfiguration(),S.RequestedOwner->GetBindingFingerprint()});
        }
        return Records;
    }
    void RecordFrameFailure(ERegistryFrameFailure Reason, FFrameNumber Frame,
        const FSession* Session = nullptr, EOwnerInputStatus Status = EOwnerInputStatus::InvalidBinding) noexcept
    {
        const auto* Owner = Session ? Session->Owner.get() : nullptr;
        LastFrameFailure = {Reason, Frame, Session ? Session->Description.Id : 0,
            Session ? Session->Description.Epoch : 0, Owner ? Owner->GetPollSerial() : 0,
            View ? View->Version : 0, Status, Owner ? Owner->GetPhase() : EOwnerInputPhase::Quarantined,
            Owner ? Owner->GetLastPollFailure() : EOwnerPollFailure::None,
            Owner != nullptr, Owner != nullptr && Reason == ERegistryFrameFailure::OwnerPoll};
    }
    bool IsWorker() const noexcept { return std::this_thread::get_id() == Worker; }
    bool ValidNetworkAdmission(const FNativeNetworkAdmission& A) const
    {
        const auto& C=A.Command;const auto& D=C.Binding;
        if(!A.OwnerKey||A.OwnerKey!=NativeAdmissionOwner||!A.Service||!C.Id||C.Operation!=EBoundaryOperation::Bind
            ||C.WorkerGeneration!=WorkerGeneration||C.RegistryVersion!=View->Version||!D.Network
            ||!D.Network->IsValid()||D.Network->WorldGeneration!=WorkerGeneration||!D.Contract
            ||!D.Id||!D.Epoch||D.First==UINT64_MAX||!FInputSessionCommands::Canonical(C,Commands->Limits))return false;
        const auto& N=*D.Network;const auto& Config=A.Service->GetConfiguration();
        if(N.Version==2)
        {
            if(!NativeRequestedV2RuntimeQualified()||!A.NativeV2||A.NativeV2->Owner()!=std::this_thread::get_id()
                ||!(A.NativeV2->Mapping()==N)||A.NativeV2->ServiceIdentity()!=A.Service->ReadServiceIdentity()
                ||A.NativeV2->Expired(std::chrono::steady_clock::now()))return false;
            const auto Closed=A.NativeV2->ReadClosure();if(Closed.Host||Closed.Slot||Closed.Socket)return false;
        }
        else if(N.Version!=1||A.NativeV2)return false;
        return Config.AllowLocalFallback&&Config.FirstFrame==D.First&&Config.Binding.Session==N.Session
            &&Config.Binding.Epoch==N.Epoch&&Config.Binding.Actor==N.Actor&&Config.Binding.Incarnation==N.Incarnation
            &&Config.Binding.ContractId==N.WireContract&&A.Service->GetContract()
            &&A.Service->GetContract()->GetFingerprint()==D.Contract->GetFingerprint();
    }
    bool RetireSession(FSession& S)
    {
        if(S.RequestedOwner)
        {
            if(S.RequestedToken){S.RequestedOwner->Abort(*S.RequestedToken);S.RequestedToken.reset();}
            if(S.RequestedOwner->GetPhase()!=ERequestedOwnerPhase::Closed&&!S.RequestedOwner->RetireAtBoundary())return false;
            S.RequestedOwner.reset();S.NetworkAdmission.reset();++Destroyed;return true;
        }
        if(!S.Owner)return true;
        if(S.Token){S.Owner->Abort(*S.Token);S.Token.reset();}
        if(!S.Owner->RetireAtBoundary())return false;
        S.Owner.reset();++Destroyed;return true;
    }
    void PublishView(std::shared_ptr<FInputRegistryView> Next)
    {
        auto Failed = std::make_shared<FInputRegistryView>(*Next); Failed->Terminal = true;
        TerminalView = std::move(Failed); View = std::move(Next);
    }
    void MakeTerminal() noexcept { Terminal = true; View = TerminalView; Latest.reset(); InitialReceipt.reset(); }
    void Outcomes(FInputSessionCommands::FRecord& R)
    {
        R.Receipt.Sessions.clear();
        for (const auto& P : Sessions) R.Receipt.Sessions.push_back({P.first, P.second.Description.Epoch, P.second.Phase});
        R.Receipt.RegistryVersion = View->Version;
    }
    bool RetireAll()
    {
        bool Done = true;
        for (auto& P : Sessions)
        {
            auto& S = P.second;
            S.Phase = ESessionPhase::CancelPending;
            if(S.NativeV2Binding){S.NativeV2Retire=ERequestedRetirementReasonV2::WorkerClose;continue;}
            if (!RetireSession(S)) { Done = false; continue; }
            S.Phase = ESessionPhase::Retired;
        }
        NetworkAdmissions.clear();
        return Done;
    }
    bool TryResume(FInputSessionCommands::FRecord& R)
    {
        auto It = Sessions.find(R.Command.Session);
        if (It == Sessions.end()) { R.Receipt.Status = EBoundaryStatus::Rejected; return true; }
        auto& S = It->second;
        if (S.Journal && S.Journal->BeginAcquisition().PhysicalFence != S.AcquisitionFence)
        { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return true; }
        if (S.Journal && !S.Journal->IsResumeReady()) return false;
        if (!S.Journal && S.Description.Kind != EProducerContract::AI && !S.Owner->InspectBaseline(S.Owner->GetNextFrame())) { R.Receipt.Status = EBoundaryStatus::Rejected; S.Phase = ESessionPhase::Paused; return true; }
        if (!S.Owner->ResumeAtBoundary()) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return true; }
        S.Phase = ESessionPhase::Active; R.Receipt.Status = EBoundaryStatus::Applied; Outcomes(R); return true;
    }
    // Also runs on admission failures before Apply. A later queued Bind may
    // reuse this journal, so retain it until that command has been processed.
    void ReleaseRejectedBindJournal(const FInputSessionCommands::FRecord& R)
    {
        if(R.Command.Operation==EBoundaryOperation::Bind&&R.Receipt.Status!=EBoundaryStatus::Pending
            &&R.Receipt.Status!=EBoundaryStatus::WaitingForBaseline)NetworkAdmissions.erase(R.Command.Id);
        if (R.Command.Operation != EBoundaryOperation::Bind
            || (R.Receipt.Status != EBoundaryStatus::Rejected && R.Receipt.Status != EBoundaryStatus::TerminalFailure)) return;
        const auto Id = R.Command.Binding.Journal;
        const bool Owned = std::any_of(Sessions.begin(), Sessions.end(), [&](const auto& S) { return S.second.Description.Journal == Id; });
        const bool Queued = std::any_of(Commands->Queue.begin(), Commands->Queue.end(), [&](auto Key)
        {
            const auto& C = Commands->Records.at(Key).Command;
            return C.Operation == EBoundaryOperation::Bind && C.Binding.Journal == Id;
        });
        if (!Owned && !Queued)
            Journals.erase(std::remove_if(Journals.begin(), Journals.end(), [&](const auto& J) { return J.Id == Id; }), Journals.end());
    }
    void Apply(FInputSessionCommands::FRecord& R)
    {
        const auto& C = R.Command;
        if (C.Operation == EBoundaryOperation::Bind) { Bind(R); return; }
        if (C.Operation == EBoundaryOperation::RetireAll)
        {
            const bool Native=HasNativeV2Sessions();
            if(Native)NativeV2GlobalControls.emplace(C.Id,C.Operation);
            MakeTerminal();const bool Legacy=RetireAll();
            if(Native)MarkNativeV2Retirements(ERequestedRetirementReasonV2::GlobalRetire);
            R.Receipt.Status=Native?EBoundaryStatus::Pending:(Legacy?EBoundaryStatus::Applied:EBoundaryStatus::TerminalFailure);Outcomes(R);return;
        }
        if (C.Operation == EBoundaryOperation::PauseAll)
        {
            const bool Native=HasNativeV2Sessions();
            if(Native)NativeV2GlobalControls.emplace(C.Id,C.Operation);
            for (auto& P : Sessions)
            {
                auto& S = P.second;
                if(S.NativeV2Binding){S.NativeV2Retire=ERequestedRetirementReasonV2::GlobalPause;S.Phase=ESessionPhase::CancelPending;continue;}
                if (S.Phase == ESessionPhase::Paused) continue;
                if(S.RequestedOwner)
                {
                    // A paused network stream is closed. Resume must use a
                    // fresh admitted epoch/source; never recycle its cursor.
                    if(!S.RequestedOwner->RetireAtBoundary()){MakeTerminal();R.Receipt.Status=EBoundaryStatus::TerminalFailure;Outcomes(R);return;}
                    S.Phase=ESessionPhase::Paused;continue;
                }
                if (!S.Owner->PauseAtBoundary()) { S.Phase = ESessionPhase::CancelPending; MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return; }
                S.Phase = ESessionPhase::Paused;
            }
            Latest.reset(); InitialReceipt.reset(); R.Receipt.Status = Native?EBoundaryStatus::Pending:EBoundaryStatus::Applied; Outcomes(R); return;
        }
        auto It = Sessions.find(C.Session);
        if (It == Sessions.end() || It->second.Description.Epoch != C.Epoch) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        auto& S = It->second;
        if(S.NativeV2Binding)
        {
            if(C.Operation!=EBoundaryOperation::Detach||!C.NativeV2Control
                ||!C.NativeV2Control->Matches(S.Description.Id,S.Description.Epoch,S.Description.Network->Service,
                    S.NativeV2Binding->Source()->Generation(),S.NativeV2Binding->Source()->Instance())
                ||S.NativeV2DetachCommand){R.Receipt.Status=EBoundaryStatus::Rejected;return;}
            S.NativeV2DetachCommand=C.Id;S.NativeV2Retire=ERequestedRetirementReasonV2::ExplicitDetach;
            S.Phase=ESessionPhase::CancelPending;R.Receipt.NativeV2=FRequestedLifecycleReceiptV2{};
            R.Receipt.Status=EBoundaryStatus::Pending;return;
        }
        if(C.NativeV2Control){R.Receipt.Status=EBoundaryStatus::Rejected;return;}
        if (C.Operation == EBoundaryOperation::Resume)
        {
            if(S.RequestedOwner||S.NetworkAdmission){R.Receipt.Status=EBoundaryStatus::Rejected;return;}
            if (S.Phase != ESessionPhase::Paused || C.ResumeGeneration <= S.ResumeGeneration)
            { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
            S.ResumeGeneration = C.ResumeGeneration; S.Phase = ESessionPhase::WaitingBaseline;
            if (S.Journal && !S.Journal->RequestFreshResume()) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return; }
            if (S.Journal) S.AcquisitionFence = S.Journal->BeginAcquisition().PhysicalFence;
            R.Receipt.Status = EBoundaryStatus::WaitingForBaseline;
            if (!TryResume(R)) Waiting = C.Id;
            return;
        }
        if (C.Operation == EBoundaryOperation::Detach)
        {
            auto Next = std::make_shared<FInputRegistryView>(*View);
            if (Next->Version == UINT64_MAX) throw std::overflow_error("registry");
            ++Next->Version;
            Next->Bindings.erase(std::remove_if(Next->Bindings.begin(), Next->Bindings.end(), [&](const auto& D) { return D.Id == C.Session; }), Next->Bindings.end());
            auto Failed = std::make_shared<FInputRegistryView>(*Next); Failed->Terminal = true;
            S.Phase = ESessionPhase::CancelPending;
            if (!RetireSession(S)) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return; }
            S.Phase = ESessionPhase::Retired;
            R.Receipt.Status = EBoundaryStatus::Applied; Outcomes(R);
            const auto JournalId = S.Description.Journal;
            Sessions.erase(It);
            Journals.erase(std::remove_if(Journals.begin(), Journals.end(), [&](const auto& J) { return J.Id == JournalId; }), Journals.end());
            TerminalView = std::move(Failed); View = std::move(Next); R.Receipt.RegistryVersion = View->Version; return;
        }
        R.Receipt.Status = EBoundaryStatus::Rejected;
    }
    void Bind(FInputSessionCommands::FRecord& R)
    {
        auto D = R.Command.Binding;
        const bool Requested=D.Network.has_value();
        std::shared_ptr<const FNativeNetworkAdmission> Admission;
        if(Requested)
        {
            const auto Found=NetworkAdmissions.find(R.Command.Id);
            if(Found==NetworkAdmissions.end()||!ValidNetworkAdmission(*Found->second)
                ||!R.Receipt.Boundary.HasWorkerFrame||D.First!=R.Receipt.Boundary.WorkerFrame)
            {R.Receipt.Status=EBoundaryStatus::Rejected;return;}
            auto Bytes=FInputSessionCommands::Canonical(Found->second->Command,Commands->Limits);
            if(!Bytes||*Bytes!=R.Bytes){R.Receipt.Status=EBoundaryStatus::Rejected;return;}
            Admission=Found->second;
        }
        if (!D.Id || !D.Epoch || (!Requested&&(!D.Controller||!D.Producer)) || !D.Contract || D.Actors.empty()
            || Sessions.size() >= Commands->Limits.Sessions || Sessions.count(D.Id)
            || (LastEpoch.count(D.Id) && D.Epoch <= LastEpoch.at(D.Id))
            || (!Requested && D.Kind != EProducerContract::Device && D.Kind != EProducerContract::ExactScenario && D.Kind != EProducerContract::AI
                && !(D.Kind == EProducerContract::ExactRemote && AdmitsOriginalDeviceRemote(RemoteAdmission)))
            || (D.Kind == EProducerContract::ExactRemote && !D.Scenario.empty())
            || (D.AllowScenarioAppend && D.Kind != EProducerContract::ExactScenario)
            || (D.ResumeRearmMask && D.Kind != EProducerContract::Device)
            || (D.EnableDesktop && D.Kind != EProducerContract::Device)
            || (!Requested && !ValidInputProcessingPolicy(D.Kind, D.Processing, *D.Contract, RemoteAdmission)))
        { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        std::sort(D.Actors.begin(), D.Actors.end());
        std::set<std::uint64_t> ActorIds;
        std::size_t ActorCount = D.Actors.size();
        for (const auto& P : Sessions)
        {
            if ((D.Producer&&P.second.Description.Producer == D.Producer) || (D.Controller&&P.second.Description.Controller == D.Controller))
            { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
            if(Requested&&P.second.NetworkAdmission)
            {
                const auto& Existing=*P.second.Description.Network;
                if(P.second.NetworkAdmission->Service==Admission->Service
                    ||(Existing.Session==D.Network->Session&&Existing.Epoch==D.Network->Epoch
                        &&Existing.Actor==D.Network->Actor&&Existing.Incarnation==D.Network->Incarnation
                        &&Existing.Component==D.Network->Component))
                {R.Receipt.Status=EBoundaryStatus::Rejected;return;}
            }
            for (const auto& A : P.second.Description.Actors) ActorIds.insert(A.Id);
            ActorCount += P.second.Description.Actors.size();
        }
        for (const auto& A : D.Actors)
            if (!A.Id || !A.Generation || !ActorIds.insert(A.Id).second) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        if (ActorCount > Commands->Limits.Actors) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        auto Next = std::make_shared<FInputRegistryView>(*View);
        if (Next->Version == UINT64_MAX) throw std::overflow_error("registry");
        ++Next->Version; Next->Bindings.push_back(D);
        std::sort(Next->Bindings.begin(), Next->Bindings.end(), [](const auto& A, const auto& B) { return A.Id < B.Id; });
        std::shared_ptr<FRawAcquisitionJournal> Journal;
        if (D.Kind == EProducerContract::Device)
        {
            for (const auto& J : Journals) if (J.Id == D.Journal) Journal = J.Journal;
            for (const auto& P : Sessions)
                if (Journal && P.second.Journal == Journal) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
            if (!Journal) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        }
        std::shared_ptr<FAIInputCommands> AI;
        if (D.Kind == EProducerContract::AI)
        {
            for (const auto& J : Journals) if (J.Id == D.Journal) AI = J.AI;
            if (!AI) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
            for (const auto& P : Sessions)
                if (P.second.Description.Journal == D.Journal) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        }
        std::shared_ptr<Netcode::FExactRemoteInputIngress> Remote;
        if (!Requested && D.Kind == EProducerContract::ExactRemote)
        {
            for (const auto& J : Journals) if (J.Id == D.Journal) Remote = J.Remote;
            if (!Remote || Remote->GetState() != Netcode::ERemoteIngressState::Unbound
                || !Remote->MatchesBinding(*D.Contract, {D.Epoch}, {EProducerKind::Device,D.Producer}, D.First))
            { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
            for (const auto& P : Sessions)
                if (P.second.Description.Journal == D.Journal) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        }
        // Allocate ledger/registry storage before constructing a mutable source.
        auto Failed = std::make_shared<FInputRegistryView>(*Next); Failed->Terminal = true;
        auto EpochSlot = LastEpoch.try_emplace(D.Id,0).first;
        auto Slot = Sessions.try_emplace(D.Id).first;
        auto& S = Slot->second;
        S.Description = D; S.Journal = Journal; S.Phase = ESessionPhase::CancelPending;
        if(Requested)
        {
            if(D.Network->Version==2)
            {
                R.Receipt.NativeV2=FRequestedLifecycleReceiptV2{};auto& Trace=*R.Receipt.NativeV2;
                Trace.Service=D.Network->Service;Trace.SourceGeneration=D.Network->SourceGeneration;
                auto Bound=Admission->Service->BindSourceWithIdentityOnCurrentThread(D.Network->SourceGeneration,true);
                Trace.WorkerSelectionBound=Bound.WorkerSelectionBound;
                Trace.BindFailure=Bound.Failure;
                if(Bound.WorkerSelectionBound)EpochSlot->second=D.Epoch;
                if(Bound.SourceConstructed){++Constructed;Trace.Construction=ERequestedConstructionV2::Constructed;}
                if(!Bound.Source||!Bound.Identity||!Bound.Identity->MatchesSource(Bound.Source.get())
                    ||!Bound.Identity->MatchesService(Admission->Service->ReadServiceIdentity())
                    ||Bound.Identity->Generation()!=D.Network->SourceGeneration||!Bound.Identity->Instance()
                    ||Bound.Identity->Owner()!=Worker||!Bound.Identity->IsOpen())
                {
                    if(Bound.Source){Bound.Source->Cancel();Bound.Source.reset();Bound.SourceRetired=true;}
                    if(Bound.SourceConstructed){++Destroyed;Trace.SourceDestroyed=true;Trace.Construction=ERequestedConstructionV2::ConstructedThenRetired;}
                    Trace.Retirement=ERequestedRetirementReasonV2::ConstructionFailed;
                    R.Receipt.Status=Bound.SourceConstructed?EBoundaryStatus::ConstructedThenRetired:EBoundaryStatus::Rejected;Sessions.erase(Slot);return;
                }
                Trace.SourceInstance=Bound.Identity->Instance();S.NativeV2SourceAddress=Bound.Source.get();S.NativeV2Command=R.Command.Id;
                try
                {
                    S.RequestedOwner=FRequestedInputOwner::Create(std::move(Bound.Source),{D.Network->SourceGeneration,D.Contract,D.Processing,D.Network->DigitalPolicy},D.First);
                    if(!S.RequestedOwner)throw std::runtime_error("native V2 owner creation failed");
                    S.NativeV2Binding=std::shared_ptr<const FNativeRequestedBindingV2>(new FNativeRequestedBindingV2(Admission->NativeV2,Bound.Identity,D.Id,D.Epoch));
                    S.NetworkAdmission=std::move(Admission);S.Phase=ESessionPhase::CancelPending;
                    // No adapter callback under Commands->Gate and no View
                    // publication before the real owner-side claim succeeds.
                    R.Receipt.Status=EBoundaryStatus::ConstructedPendingClaim;return;
                }
                catch(...)
                {
                    if(S.RequestedOwner){S.RequestedOwner->RetireAtBoundary();S.RequestedOwner.reset();}
                    ++Destroyed;Trace.SourceDestroyed=true;Trace.Construction=ERequestedConstructionV2::ConstructedThenRetired;
                    Trace.Retirement=ERequestedRetirementReasonV2::ConstructionFailed;R.Receipt.Status=EBoundaryStatus::ConstructedThenRetired;Sessions.erase(Slot);return;
                }
            }
            auto Source=Admission->Service->BindSourceOnCurrentThread(D.Network->SourceGeneration);
            if(!Source){Sessions.erase(Slot);R.Receipt.Status=EBoundaryStatus::Rejected;return;}
            ++Constructed;
            S.RequestedOwner=FRequestedInputOwner::Create(std::move(Source),{D.Network->SourceGeneration,D.Contract,D.Processing,D.Network->DigitalPolicy},D.First);
            if(!S.RequestedOwner){++Destroyed;Sessions.erase(Slot);R.Receipt.Status=EBoundaryStatus::Rejected;return;}
            S.NetworkAdmission=std::move(Admission);S.Phase=ESessionPhase::Active;
            EpochSlot->second=D.Epoch;TerminalView=std::move(Failed);View=std::move(Next);
            R.Receipt.Status=EBoundaryStatus::Applied;Outcomes(R);return;
        }
        std::unique_ptr<IInputProducer> Source;
        if (D.Kind == EProducerContract::ExactScenario)
            Source = FTestInputProducer::Create(D.Contract, {D.Epoch}, {EProducerKind::Device,D.Producer}, D.First, D.Scenario);
        else if (D.Kind == EProducerContract::AI)
            Source = FAIInputProducer::Create(AI, D.Contract, {D.Epoch}, {EProducerKind::AI,D.Producer}, D.First);
        else if (D.Kind == EProducerContract::ExactRemote)
            Source = Remote->CreateProducerForCurrentOwner();
        else Source = D.EnableDesktop
            ? FDeviceInputProducer::CreateDesktop(Journal,D.Contract,{D.Epoch},{EProducerKind::Device,D.Producer},D.First,D.ResumeRearmMask)
            : FDeviceInputProducer::Create(Journal,D.Contract,{D.Epoch},{EProducerKind::Device,D.Producer},D.First,D.ResumeRearmMask);
        if (!Source) { Sessions.erase(Slot); R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        ++Constructed;
        FOwnerInputBinding Binding{D.Id, {D.Kind == EProducerContract::AI ? EProducerKind::AI : EProducerKind::Device,D.Producer}, {D.Epoch}, D.Contract, D.Processing, D.Kind};
        S.Owner = FSameFrameInputOwner::Create(std::move(Source), Binding, D.First, RemoteAdmission);
        if (!S.Owner) { ++Destroyed; Sessions.erase(Slot); R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        if (Journal && !S.Owner->PauseAtBoundary())
        {
            MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return;
        }
        S.Phase = Journal ? ESessionPhase::Paused : ESessionPhase::Active;
        EpochSlot->second = D.Epoch;
        TerminalView = std::move(Failed); View = std::move(Next); R.Receipt.Status = EBoundaryStatus::Applied; Outcomes(R);
    }
    std::shared_ptr<FInputSessionCommands> Commands;
    const std::thread::id Worker;
    const std::uint64_t WorkerGeneration;
    const EExactRemoteAdmission RemoteAdmission;
    bool InspectingFrameReadiness = false;
    std::vector<FJournalService> Journals;
    const std::shared_ptr<const std::uint8_t> NativeAdmissionOwner;
    std::map<std::uint64_t,std::shared_ptr<const FNativeNetworkAdmission>> NetworkAdmissions;
    #include "NativeRequestedLifecycleRegistryV2.inl"
    std::optional<FFrameNumber> FrozenRequestedFrame;
    std::map<std::uint64_t, FSession> Sessions;
    std::map<std::uint64_t, std::uint64_t> LastEpoch;
    std::shared_ptr<const FInputRegistryView> View, TerminalView;
    std::shared_ptr<FRegistryFrame> Pending;
    std::shared_ptr<const FRegistryFrame> Latest, InitialReceipt, HistoricalInitial;
    std::vector<std::shared_ptr<const FRegistryFrame>> History;
    std::vector<std::shared_ptr<const FRegistryFrame>> ReplayPlan;
    std::size_t ReplayIndex = 0;
    std::optional<std::uint64_t> Waiting;
    bool Terminal = false, Installed = false, Begun = false, Replaying = false;
    bool PendingEmptyReceipt = false;
    FRegistryFrameFailure LastFrameFailure{};
    std::unique_ptr<FReplayGroupPlan> ReplayGroup;
    bool GroupExecutionReplay = false;
    std::shared_ptr<const FGroupReplayPublication> GroupReplayPublished;
    std::shared_ptr<const FRegistryFrame> GroupReplayReceipt;
    std::unique_ptr<FGroupResumePlan> ResumeGroup; // destroyed before Sessions

    const std::shared_ptr<const std::uint8_t> BoundaryCaptureOrigin=std::make_shared<const std::uint8_t>(std::uint8_t{0});
    std::shared_ptr<const FRegistryFrame> LastCompletedForCapture; // Completion history survives PauseAll clearing the public Latest observation.
    bool BoundaryReservationCalling = false;
    mutable bool BoundaryCaptureCalling = false; // worker-only, spans cleanup as well as allocations
    std::unique_ptr<FBoundaryReservationPlan> BoundaryReservation; // destroyed before the worker-owned sessions
    std::uint64_t Constructed = 0, Destroyed = 0;
};
}
