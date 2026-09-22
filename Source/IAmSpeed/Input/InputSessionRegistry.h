#pragma once

#include "SameFrameInputOwner.h"
#include "RawAcquisitionJournal.h"
#include "Testing/TestInputProducerV2.h"
#include <deque>
#include <map>
#include <set>

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
    std::vector<FSessionActor> Actors;
    std::vector<FInputFrame> Scenario;
};
enum class EBoundaryOperation : std::uint8_t { Bind, PauseAll, Resume, Detach, RetireAll };
struct FBoundaryCommandDescriptor
{
    std::uint64_t Id = 0, WorkerGeneration = 0, RegistryVersion = 0, Session = 0, Epoch = 0, ResumeGeneration = 0;
    EBoundaryOperation Operation = EBoundaryOperation::Bind;
    FSessionDescriptor Binding;
};
enum class EBoundaryStatus : std::uint8_t { Pending, WaitingForBaseline, Applied, Rejected, TerminalFailure };
enum class ESessionPhase : std::uint8_t { Active, Paused, WaitingBaseline, CancelPending, Retired };
struct FSessionOutcome { std::uint64_t Session = 0, Epoch = 0; ESessionPhase Phase = ESessionPhase::Active; };
struct FBoundaryReceipt
{
    std::uint64_t Id = 0, WorkerGeneration = 0, RegistryVersion = 0, ResumeGeneration = 0;
    EBoundaryStatus Status = EBoundaryStatus::Pending;
    std::vector<FSessionOutcome> Sessions;
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
            if (C.Id <= HighId || Queue.size() >= Limits.Commands || Records.size() >= Limits.Receipts || StopRequested)
                return ECommandAdmission::Rejected;
            FRecord R{C, std::move(*Bytes), {C.Id, C.WorkerGeneration, C.RegistryVersion, C.ResumeGeneration, EBoundaryStatus::Pending, {}}};
            R.Digest = Digest(R.Bytes);
            R.Receipt.Sessions.reserve(Limits.Sessions);
            Queue.push_back(C.Id);
            try { Records.emplace(C.Id, std::move(R)); }
            catch (...) { Queue.pop_back(); throw; }
            HighId = C.Id; return ECommandAdmission::Enqueued;
        }
        catch (...) { return ECommandAdmission::Rejected; }
    }
    std::optional<FBoundaryReceipt> Read(std::uint64_t Id) const
    {
        std::lock_guard<std::mutex> Lock(Gate); const auto I = Records.find(Id);
        return I == Records.end() ? std::nullopt : std::optional<FBoundaryReceipt>(I->second.Receipt);
    }
    void RequestStop() { std::lock_guard<std::mutex> Lock(Gate); StopRequested = true; }
    static bool ExactPolicy(EProducerContract Kind, const FInputProcessingPolicy& P) noexcept
    {
        if (Kind != EProducerContract::ExactScenario && Kind != EProducerContract::ExactRemote) return false;
        return std::all_of(P.Step.begin(), P.Step.end(), [](auto S) { return S == 0; });
    }
private:
    static std::uint64_t Digest(const std::vector<std::uint8_t>& Bytes) noexcept
    { std::uint64_t H = 14695981039346656037ull; for (const auto B : Bytes) { H ^= B; H *= 1099511628211ull; } return H; }
    struct FRecord { FBoundaryCommandDescriptor Command; std::vector<std::uint8_t> Bytes; FBoundaryReceipt Receipt; std::uint64_t Digest = 0; };
    static std::optional<std::vector<std::uint8_t>> Canonical(const FBoundaryCommandDescriptor& C, const FRegistryLimits& L)
    {
        if (!L.Sessions || !L.Actors || !L.Commands || !L.Receipts || !L.Frames || !L.DescriptorBytes) return {};
        if (C.Binding.Actors.size() > L.Actors || C.Binding.Scenario.size() > L.Frames) return {};
        std::vector<std::uint8_t> B;
        const auto Add = [&](std::uint64_t V) { if (B.size() > L.DescriptorBytes || L.DescriptorBytes - B.size() < 8) throw std::length_error("descriptor"); for (unsigned I = 0; I < 8; ++I) B.push_back(std::uint8_t(V >> (I * 8))); };
        Add(1); Add(C.Id); Add(C.WorkerGeneration); Add(C.RegistryVersion); Add(C.Session); Add(C.Epoch); Add(C.ResumeGeneration); Add(std::uint64_t(C.Operation));
        const auto& D = C.Binding;
        Add(D.Id); Add(D.Epoch); Add(D.Controller); Add(D.Producer); Add(D.Journal); Add(D.First); Add(std::uint64_t(D.Kind));
        Add(D.Contract ? D.Contract->GetFingerprint().size() : 0);
        if (D.Contract) for (const auto V : D.Contract->GetFingerprint()) Add(V);
        for (const auto V : D.Processing.Step) Add(V);
        Add(D.Actors.size()); for (const auto& A : D.Actors) { Add(A.Id); Add(A.Generation); }
        Add(D.Scenario.size());
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
    bool StopRequested = false;
};

struct FInputRegistryView
{
    std::uint64_t Version = 0, WorkerGeneration = 0;
    bool Terminal = false;
    std::vector<FSessionDescriptor> Bindings;
};
struct FInstalledSessionInput
{
    std::uint64_t Session = 0, Epoch = 0;
    std::shared_ptr<const FOwnerInputSnapshot> Snapshot;
    std::vector<FSessionActor> Actors;
    bool AlreadyAppliedAtInitialBoundary = false;
    std::optional<FActionValues> FilterBefore;
};
struct FRegistryFrame
{
    FFrameNumber Frame = 0;
    std::shared_ptr<const FInputRegistryView> Registry;
    std::vector<FInstalledSessionInput> Inputs;
};

// Portable owner endpoint. Its actor install/step hooks are explicit phases;
// no claim of actual engine actor writes or world restoration is made here.
class FInputSessionRegistry final
{
public:
    struct FJournalService { std::uint64_t Id = 0; std::shared_ptr<FRawAcquisitionJournal> Journal; };
    FInputSessionRegistry(std::shared_ptr<FInputSessionCommands> InCommands, std::uint64_t Generation,
        std::vector<FJournalService> InJournals = {})
        : Commands(std::move(InCommands)), Worker(std::this_thread::get_id()), WorkerGeneration(Generation), Journals(std::move(InJournals))
    {
        if (!Commands || !Generation) throw std::invalid_argument("registry");
        PublishView(std::make_shared<FInputRegistryView>(FInputRegistryView{0, Generation, false, {}}));
        std::set<std::uint64_t> Ids;
        std::set<const FRawAcquisitionJournal*> Raw;
        for (const auto& J : Journals)
            if (!J.Id || !J.Journal || !Ids.insert(J.Id).second || !Raw.insert(J.Journal.get()).second)
                throw std::invalid_argument("journal alias");
        if (!Commands->Limits.Frames) throw std::invalid_argument("history capacity");
        History.resize(Commands->Limits.Frames);
    }
    FInputSessionRegistry(const FInputSessionRegistry&) = delete;
    FInputSessionRegistry& operator=(const FInputSessionRegistry&) = delete;
    ~FInputSessionRegistry()
    {
        // Never destroy thread-affine producers implicitly on a foreign lane.
        if (!IsWorker() || !CloseWorkerLedger()) std::terminate();
    }
    bool CloseWorkerLedger()
    {
        if (!IsWorker()) return false;
        AbortFrame();
        std::lock_guard<std::mutex> Lock(Commands->Gate);
        Commands->StopRequested = true;
        const bool Retired = RetireAll();
        for (auto& P : Commands->Records)
            if (P.second.Receipt.Status == EBoundaryStatus::Pending || P.second.Receipt.Status == EBoundaryStatus::WaitingForBaseline)
            { P.second.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(P.second); }
        Commands->Queue.clear(); Waiting.reset(); Replaying = false; ReplayPlan.clear();
        return Retired;
    }
    std::shared_ptr<const FInputRegistryView> ReadRegistry() const { return IsWorker() ? View : nullptr; }
    std::uint64_t ConstructionCount() const noexcept { return IsWorker() ? Constructed : 0; }
    std::uint64_t DestructionCount() const noexcept { return IsWorker() ? Destroyed : 0; }
    bool IsTerminal() const noexcept { return !IsWorker() || Terminal; }

    // Called between transactions, including while the simulation is paused.
    bool ServiceBoundary()
    {
        if (!IsWorker() || Pending) return false;
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
        const auto Count = Commands->Queue.size();
        for (std::size_t I = 0; I < Count; ++I)
        {
            const auto Id = Commands->Queue.front(); Commands->Queue.pop_front();
            auto& R = Commands->Records.at(Id);
            if (Terminal && R.Command.Operation != EBoundaryOperation::RetireAll) { R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); continue; }
            if (R.Command.WorkerGeneration != WorkerGeneration || R.Command.RegistryVersion != View->Version)
            { R.Receipt.Status = EBoundaryStatus::Rejected; continue; }
            if (Waiting)
            {
                const auto Op = R.Command.Operation;
                const bool Supersedes = Op == EBoundaryOperation::PauseAll || Op == EBoundaryOperation::RetireAll
                    || (Op == EBoundaryOperation::Detach && R.Command.Session == Commands->Records.at(*Waiting).Command.Session
                        && R.Command.Epoch == Commands->Records.at(*Waiting).Command.Epoch);
                if (!Supersedes) { R.Receipt.Status = EBoundaryStatus::Rejected; continue; }
                auto& Old = Commands->Records.at(*Waiting); Old.Receipt.Status = EBoundaryStatus::Rejected;
                Outcomes(Old); Waiting.reset();
            }
            try { Apply(R); }
            catch (...) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); }
        }
        if (Waiting)
        {
            auto& R = Commands->Records.at(*Waiting);
            try { if (TryResume(R)) Waiting.reset(); }
            catch (...) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); Waiting.reset(); }
        }
        return !Terminal && !Waiting;
    }
    bool PrepareFrame(FFrameNumber N)
    {
        if (!IsWorker() || Terminal || Pending || Waiting || Replaying || Sessions.empty()) return false;
        // Readiness is checked for ALL sessions before any source is touched.
        for (const auto& P : Sessions)
            if (P.second.Phase != ESessionPhase::Active || !P.second.Owner) return false;
        try
        {
            auto Frame = std::make_shared<FRegistryFrame>(); Frame->Frame = N; Frame->Registry = View;
            Frame->Inputs.reserve(Sessions.size());
            for (auto& Pair : Sessions)
            {
                auto& S = Pair.second;
                if (S.Phase != ESessionPhase::Active || !S.Owner) { AbortFrame(); return false; }
                const bool Initial = S.Owner->HasInitialReceipt();
                auto P = S.Owner->Poll(N);
                if (!P.Token || P.Status != EOwnerInputStatus::Ready) { AbortFrame(); return false; }
                S.Token = std::move(P.Token);
                Frame->Inputs.push_back({S.Description.Id, S.Description.Epoch, {}, S.Description.Actors, Initial, S.Owner->ReadPreparedFilterBefore()});
            }
            Pending = std::move(Frame); return true;
        }
        catch (...) { AbortFrame(); return false; }
    }
    bool InstallAll()
    {
        if (!IsWorker() || !Pending || Installed || Begun) return false;
        std::size_t I = 0;
        for (auto& Pair : Sessions)
        {
            auto& S = Pair.second;
            if (!S.Token || !S.Owner->Install(*S.Token)) { AbortFrame(); return false; }
            Pending->Inputs[I++].Snapshot = S.Owner->ReadInstalled(*S.Token);
            if (!Pending->Inputs[I - 1].Snapshot) { AbortFrame(); return false; }
        }
        Installed = true; return true;
    }
    std::shared_ptr<const FRegistryFrame> ReadInstalled() const
    { return IsWorker() && Installed && !Terminal ? Pending : nullptr; }
    bool BeginAll()
    {
        if (!IsWorker() || !Pending || !Installed || Begun || Terminal) return false;
        for (const auto& Pair : Sessions)
            if (!Pair.second.Token || Pair.second.Owner->GetPhase() != EOwnerInputPhase::Installed) { AbortFrame(); return false; }
        for (auto& Pair : Sessions)
            if (!Pair.second.Owner->BeginStep(*Pair.second.Token, Pending->Frame)) { AbortFrame(); return false; }
        Begun = true; return true;
    }
    bool CloseInitialBoundary()
    {
        if (!IsWorker() || !Pending || !Installed || Begun || Terminal) return false;
        for (const auto& Pair : Sessions)
        {
            const auto& S = Pair.second;
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
        if (!IsWorker() || !Pending || !Begun || Terminal) return false;
        for (const auto& Pair : Sessions)
            if (!Pair.second.Token || !Pair.second.Owner->CanComplete(*Pair.second.Token, Pending->Frame)) return false;
        return true;
    }
    bool CompleteAll() noexcept
    {
        if (!ValidateComplete()) return false;
        for (auto& Pair : Sessions)
        {
            auto& S = Pair.second;
            if (!S.Owner->CompleteStep(*S.Token, Pending->Frame)) { MakeTerminal(); return false; }
            S.Token.reset();
        }
        History[Pending->Frame % History.size()] = Pending;
        Latest = Pending; InitialReceipt.reset(); Pending.reset(); Installed = Begun = false; return true;
    }
    void AbortFrame() noexcept
    {
        if (!IsWorker()) return;
        for (auto& Pair : Sessions) { auto& S = Pair.second; if (S.Token) S.Owner->Abort(*S.Token); S.Token.reset(); }
        Pending.reset(); Installed = Begun = false; MakeTerminal();
    }
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
        if (!IsWorker() || Terminal || Pending || Waiting || Replaying || First > Last || Last - First >= History.size()) return false;
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
                    if (B == F->Registry->Bindings.end() || !Input.Snapshot) return false;
                    auto Expected = Input.Snapshot->Requested;
                    if (B->Kind == EProducerContract::Device)
                    {
                        if (!Input.FilterBefore) return false;
                        if (!Input.Snapshot->Input.GetData().Reset)
                            for (std::size_t A=0; A<ActionCount; ++A)
                                if (const int Step = B->Processing.Step[A])
                                    Expected[A] = std::int16_t(int((*Input.FilterBefore)[A]) + std::clamp(int(Expected[A])-int((*Input.FilterBefore)[A]),-Step,Step));
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
    { return IsWorker() && Replaying && ReplayIndex < ReplayPlan.size() ? ReplayPlan[ReplayIndex++] : nullptr; }
    bool EndRetainedReplay()
    {
        if (!IsWorker() || !Replaying || ReplayIndex != ReplayPlan.size()) return false;
        Replaying = false; ReplayPlan.clear(); return true;
    }
private:
    struct FSession
    {
        FSessionDescriptor Description;
        std::unique_ptr<FSameFrameInputOwner> Owner;
        std::shared_ptr<FRawAcquisitionJournal> Journal;
        std::optional<FSameFrameInputOwner::FToken> Token;
        ESessionPhase Phase = ESessionPhase::Active;
        std::uint64_t ResumeGeneration = 0;
        std::uint64_t AcquisitionFence = 0;
    };
    bool IsWorker() const noexcept { return std::this_thread::get_id() == Worker; }
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
            if (!S.Owner) continue;
            if (S.Token) { S.Owner->Abort(*S.Token); S.Token.reset(); }
            S.Phase = ESessionPhase::CancelPending;
            if (!S.Owner->RetireAtBoundary()) { Done = false; continue; }
            S.Owner.reset(); ++Destroyed; S.Phase = ESessionPhase::Retired;
        }
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
        if (!S.Journal && !S.Owner->InspectBaseline(S.Owner->GetNextFrame())) { R.Receipt.Status = EBoundaryStatus::Rejected; S.Phase = ESessionPhase::Paused; return true; }
        if (!S.Owner->ResumeAtBoundary()) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return true; }
        S.Phase = ESessionPhase::Active; R.Receipt.Status = EBoundaryStatus::Applied; Outcomes(R); return true;
    }
    void Apply(FInputSessionCommands::FRecord& R)
    {
        const auto& C = R.Command;
        if (C.Operation == EBoundaryOperation::Bind) { Bind(R); return; }
        if (C.Operation == EBoundaryOperation::RetireAll)
        { MakeTerminal(); R.Receipt.Status = RetireAll() ? EBoundaryStatus::Applied : EBoundaryStatus::TerminalFailure; Outcomes(R); return; }
        if (C.Operation == EBoundaryOperation::PauseAll)
        {
            for (auto& P : Sessions)
            {
                auto& S = P.second;
                if (S.Phase == ESessionPhase::Paused) continue;
                if (!S.Owner->PauseAtBoundary()) { S.Phase = ESessionPhase::CancelPending; MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return; }
                S.Phase = ESessionPhase::Paused;
            }
            Latest.reset(); InitialReceipt.reset(); R.Receipt.Status = EBoundaryStatus::Applied; Outcomes(R); return;
        }
        auto It = Sessions.find(C.Session);
        if (It == Sessions.end() || It->second.Description.Epoch != C.Epoch) { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        auto& S = It->second;
        if (C.Operation == EBoundaryOperation::Resume)
        {
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
            if (!S.Owner->RetireAtBoundary()) { MakeTerminal(); R.Receipt.Status = EBoundaryStatus::TerminalFailure; Outcomes(R); return; }
            S.Owner.reset(); ++Destroyed; S.Phase = ESessionPhase::Retired;
            R.Receipt.Status = EBoundaryStatus::Applied; Outcomes(R);
            Sessions.erase(It); TerminalView = std::move(Failed); View = std::move(Next); R.Receipt.RegistryVersion = View->Version; return;
        }
        R.Receipt.Status = EBoundaryStatus::Rejected;
    }
    void Bind(FInputSessionCommands::FRecord& R)
    {
        auto D = R.Command.Binding;
        if (!D.Id || !D.Epoch || !D.Controller || !D.Producer || !D.Contract || D.Actors.empty()
            || Sessions.size() >= Commands->Limits.Sessions || Sessions.count(D.Id)
            || (LastEpoch.count(D.Id) && D.Epoch <= LastEpoch.at(D.Id))
            || (D.Kind != EProducerContract::Device && D.Kind != EProducerContract::ExactScenario)
            || (D.Kind == EProducerContract::ExactScenario && !FInputSessionCommands::ExactPolicy(D.Kind, D.Processing)))
        { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        std::sort(D.Actors.begin(), D.Actors.end());
        std::set<std::uint64_t> ActorIds;
        std::size_t ActorCount = D.Actors.size();
        for (const auto& P : Sessions)
        {
            if (P.second.Description.Producer == D.Producer || P.second.Description.Controller == D.Controller)
            { R.Receipt.Status = EBoundaryStatus::Rejected; return; }
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
        // Allocate ledger/registry storage before constructing a mutable source.
        auto Failed = std::make_shared<FInputRegistryView>(*Next); Failed->Terminal = true;
        auto EpochSlot = LastEpoch.try_emplace(D.Id,0).first;
        auto Slot = Sessions.try_emplace(D.Id).first;
        auto& S = Slot->second;
        S.Description = D; S.Journal = Journal; S.Phase = ESessionPhase::CancelPending;
        std::unique_ptr<IInputProducer> Source;
        if (D.Kind == EProducerContract::ExactScenario)
            Source = FTestInputProducer::Create(D.Contract, {D.Epoch}, {EProducerKind::Device,D.Producer}, D.First, D.Scenario);
        else Source = FDeviceInputProducer::Create(Journal, D.Contract, {D.Epoch}, {EProducerKind::Device,D.Producer}, D.First);
        if (!Source) { Sessions.erase(Slot); R.Receipt.Status = EBoundaryStatus::Rejected; return; }
        ++Constructed;
        FOwnerInputBinding Binding{D.Id, {EProducerKind::Device,D.Producer}, {D.Epoch}, D.Contract, D.Processing, D.Kind};
        S.Owner = FSameFrameInputOwner::Create(std::move(Source), Binding, D.First);
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
    std::vector<FJournalService> Journals;
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
    std::uint64_t Constructed = 0, Destroyed = 0;
};
}
