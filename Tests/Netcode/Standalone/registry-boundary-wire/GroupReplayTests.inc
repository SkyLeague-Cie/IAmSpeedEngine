static void ReadyRegistryForReplay(FFixture& F)
{
    const auto Resume = F.Registry.PrepareGroupResumeAtBoundary(F.Request()); CHECK(Resume);
    CHECK(F.Registry.PrepareGroupResumeFinalization(*Resume)); F.Registry.FinalizePreparedGroupResume(*Resume);
    F.Registry.PublishPreparedGroupResume(*Resume); F.Registry.ReleasePreparedGroupResume(*Resume);
    F.Complete(1); F.Complete(2);
}
static V::FInputSessionRegistry::FGroupReplayRequest ReplayRequest(FFixture& F)
{
    auto View = F.Registry.ReadRegistry(); std::vector<V::FSessionActor> Actors;
    for (const auto& B : View->Bindings) Actors.insert(Actors.end(), B.Actors.begin(), B.Actors.end());
    std::sort(Actors.begin(), Actors.end());
    return {23, View->Version, 1, 2, 3, View->Phases, std::move(Actors)};
}
static void ActualRegistryOwnerGroupReplay()
{
    FFixture F; ReadyRegistryForReplay(F); const auto Latest = F.Registry.ReadLatest();
    const auto H1 = F.Registry.ReadHistorical(1), H2 = F.Registry.ReadHistorical(2);
    const auto R = ReplayRequest(F);
    auto T = F.Registry.PrepareGroupRetainedReplayStart(R); CHECK(T);
    const auto Pub = F.Registry.ReadPreparedGroupReplay(*T); CHECK(Pub && Pub->Frames.size() == 2 && Pub->Frames[0] == H1);
    CHECK(!F.Registry.ReadGroupReplayPublication() && !F.Registry.PrepareFrame(3));
    CHECK(!F.Registry.BeginRetainedReplay(1, 2, R.Actors));
    CHECK(!F.Registry.PrepareGroupResumeAtBoundary(F.Request(2, 3)));
    CHECK(F.Registry.PrepareGroupReplayFinalization(*T));
    Allocations = 0; Budget = 0;
    F.Registry.FinalizePreparedGroupReplay(*T); F.Registry.PublishPreparedGroupReplay(*T);
    const auto Published = F.Registry.ReadGroupReplayPublication(); F.Registry.ReleasePreparedGroupReplay(*T);
    Budget = -1;
    CHECK(Allocations == 0 && Published == Pub && !F.Registry.AbortGroupReplay(*T));
    CHECK(!F.Registry.PrepareGroupReplayFrame(2) && !F.Registry.PrepareFrame(1));
    CHECK(!F.Registry.NextRetainedReplay() && !F.Registry.EndRetainedReplay() && !F.Registry.EndGroupRetainedReplay());
    for (I::FFrameNumber N = 1; N <= 2; ++N)
    {
        CHECK(F.Registry.PrepareGroupReplayFrame(N)); CHECK(F.Registry.InstallAll());
        const auto Installed = F.Registry.ReadInstalled(); CHECK(Installed && Installed->Frame == N);
        const auto Old = N == 1 ? H1 : H2;
        for (std::size_t I = 0; I < Installed->Inputs.size(); ++I)
        {
            const auto& A = Installed->Inputs[I]; const auto& B = Old->Inputs[I];
            CHECK(A.Session == B.Session && A.Epoch == B.Epoch && A.Actors == B.Actors);
            CHECK(A.Snapshot->Requested == B.Snapshot->Requested && A.Snapshot->Applied == B.Snapshot->Applied);
            CHECK(A.Snapshot->Input.GetData().SourceSequence == B.Snapshot->Input.GetData().SourceSequence);
        }
        CHECK(F.Registry.BeginAll() && F.Registry.ValidateComplete()); CHECK(F.Registry.CompleteAll());
        auto Receipt = F.Registry.ReadGroupReplayReceipt(); CHECK(Receipt && Receipt->Frame == N);
        CHECK(F.Registry.ReadLatest() == Latest && F.Registry.ReadHistorical(1) == H1 && F.Registry.ReadHistorical(2) == H2);
        for (std::uint64_t S = 1; S <= F.Count; ++S) CHECK(F.Registry.PollCount(S) == 3);
    }
    CHECK(F.Registry.EndGroupRetainedReplay()); CHECK(!F.Registry.ReadGroupReplayReceipt());
    CHECK(!F.Registry.ReadGroupReplayPublication() && Published->Frames[0] == H1);
    F.Complete(3); for (std::uint64_t S = 1; S <= F.Count; ++S) CHECK(F.Registry.PollCount(S) == 4);
}
static void GroupReplayAbortFailuresAndAdmission()
{
    for (int Limit = 0; Limit < 70; ++Limit)
    {
        FFixture F; ReadyRegistryForReplay(F); const auto R = ReplayRequest(F); auto Latest = F.Registry.ReadLatest();
        Budget = Limit; auto T = F.Registry.PrepareGroupRetainedReplayStart(R); Budget = -1;
        if (T) CHECK(F.Registry.AbortGroupReplay(*T));
        CHECK(F.Registry.ReadLatest() == Latest && F.Registry.ReadHistorical(1));
        auto Retry = F.Registry.PrepareGroupRetainedReplayStart(R); CHECK(Retry && F.Registry.PrepareGroupReplayFinalization(*Retry));
        CHECK(F.Registry.AbortGroupReplay(*Retry)); CHECK(!F.Registry.AbortGroupReplay(*Retry));
        for (std::uint64_t S = 1; S <= F.Count; ++S) CHECK(F.Registry.PollCount(S) == 3);
    }
    FFixture F; ReadyRegistryForReplay(F); auto R = ReplayRequest(F);
    const auto Reject = [&](const V::FInputSessionRegistry::FGroupReplayRequest& Bad) { CHECK(!F.Registry.PrepareGroupRetainedReplayStart(Bad)); };
    auto Bad = R; ++Bad.WorkerGeneration; Reject(Bad); Bad = R; ++Bad.RegistryVersion; Reject(Bad);
    Bad = R; Bad.Sessions.pop_back(); Reject(Bad); Bad = R; ++Bad.Sessions[0].Epoch; Reject(Bad);
    Bad = R; Bad.Sessions[0].Phase = V::ESessionPhase::Paused; Reject(Bad);
    Bad = R; std::reverse(Bad.Sessions.begin(), Bad.Sessions.end()); Reject(Bad);
    Bad = R; Bad.Actors.pop_back(); Reject(Bad); Bad = R; ++Bad.Actors[0].Generation; Reject(Bad);
    Bad = R; Bad.Actors.push_back({9999, 1}); Reject(Bad); Bad = R; std::reverse(Bad.Actors.begin(), Bad.Actors.end()); Reject(Bad);
    Bad = R; Bad.First = 3; Reject(Bad); Bad = R; Bad.Last = 3; Reject(Bad); Bad = R; ++Bad.LiveNext; Reject(Bad);
    { I::FPresentationInputScope Scope; Reject(R); }
    auto T = F.Registry.PrepareGroupRetainedReplayStart(R); CHECK(T);
    bool Ready = true, Aborted = true;
    std::thread Other([&] { Ready = F.Registry.PrepareGroupReplayFinalization(*T); Aborted = F.Registry.AbortGroupReplay(*T); });
    Other.join(); CHECK(!Ready && !Aborted); CHECK(F.Registry.AbortGroupReplay(*T));
    auto Command = F.Command(V::EBoundaryOperation::PauseAll);
    T = F.Registry.PrepareGroupRetainedReplayStart(R); CHECK(T);
    CHECK(F.Commands->Submit(Command) == V::ECommandAdmission::Enqueued);
    CHECK(!F.Registry.PrepareGroupReplayFinalization(*T)); CHECK(!F.Registry.ReadPreparedGroupReplay(*T));
    CHECK(F.Registry.ServiceBoundary());
}
static void GroupReplayCommandFenceAndTeardown()
{
    FFixture F; ReadyRegistryForReplay(F); auto T = F.Registry.PrepareGroupRetainedReplayStart(ReplayRequest(F)); CHECK(T);
    CHECK(F.Registry.PrepareGroupReplayFinalization(*T));
    std::promise<void> Started; auto Begin = Started.get_future();
    auto Stop = std::async(std::launch::async, [&] { Started.set_value(); F.Commands->RequestStop(); });
    Begin.wait(); CHECK(Stop.wait_for(std::chrono::milliseconds(5)) == std::future_status::timeout);
    F.Registry.FinalizePreparedGroupReplay(*T); CHECK(!F.Registry.CloseWorkerLedger());
    F.Registry.PublishPreparedGroupReplay(*T); F.Registry.ReleasePreparedGroupReplay(*T);
    CHECK(Stop.wait_for(std::chrono::seconds(1)) == std::future_status::ready); Stop.get();
    // Stop safely retires our live owners; no source polling or synthetic reset.
    CHECK(F.Registry.ServiceBoundary()); CHECK(F.Registry.CloseWorkerLedger());
    { FFixture Dropped; ReadyRegistryForReplay(Dropped); auto X = Dropped.Registry.PrepareGroupRetainedReplayStart(ReplayRequest(Dropped)); CHECK(X); }
}
static void RealJournalDeviceRegistryReplaysAppliedWithoutAcquisition()
{
    auto Commands = std::make_shared<V::FInputSessionCommands>();
    auto Journal = std::make_shared<V::FRawAcquisitionJournal>(7);
    V::FInputSessionRegistry Registry(Commands, 23, {{1, Journal, {}}});
    V::FInputActionContractDescription CD; CD.Revision.Value = 23; CD.Actions = V::FInputActionContract::BaseActions();
    CD.Mapping.push_back({{V::ERawControlKind::PadAxis, std::uint16_t(V::EPadAxis::LeftTrigger)}, 0, 1, {}});
    CD.Physical = {{0, V::EPhysicalDestination::Throttle}, {1, V::EPhysicalDestination::Brake}, {2, V::EPhysicalDestination::Steering}};
    auto Contract = V::FInputActionContract::Create(std::move(CD)); CHECK(Contract);
    V::FSessionDescriptor D; D.Id = 1; D.Epoch = 7; D.Controller = 200; D.Producer = 99; D.Journal = 1;
    D.Contract = Contract; D.Kind = V::EProducerContract::Device; D.Actors = {{100, 2}}; D.Processing.Step[0] = 10;
    V::FBoundaryCommandDescriptor Bind; Bind.Id = 1; Bind.WorkerGeneration = 23; Bind.Binding = D;
    CHECK(Commands->Submit(Bind) == V::ECommandAdmission::Enqueued); CHECK(Registry.ServiceBoundary(0));
    CHECK(Commands->Read(1)->Status == V::EBoundaryStatus::Applied);
    V::FBoundaryCommandDescriptor Resume; Resume.Id = 2; Resume.WorkerGeneration = 23; Resume.RegistryVersion = 1;
    Resume.Operation = V::EBoundaryOperation::Resume; Resume.Session = 1; Resume.Epoch = 7; Resume.ResumeGeneration = 1;
    CHECK(Commands->Submit(Resume) == V::ECommandAdmission::Enqueued); CHECK(!Registry.ServiceBoundary());
    const auto Publish = [&](std::uint64_t N, float Value, bool Fresh)
    {
        V::FRawAcquisitionBatch Batch; Batch.Count = 1; auto& R = Batch.Readings[0];
        R.DeviceId = 73; R.Sequence = N + 1; R.Generation = {1}; R.Kind = V::ERawDeviceKind::Gamepad;
        R.TimestampMicroseconds = N; R.FreshBaseline = Fresh; R.State.Count = 1;
        R.State.Values[0] = {{V::ERawControlKind::PadAxis, std::uint16_t(V::EPadAxis::LeftTrigger)}, Value};
        CHECK(Journal->Publish(Journal->BeginAcquisition(), Batch));
    };
    const auto Step = [&](I::FFrameNumber N, bool Replay)
    {
        CHECK(Replay ? Registry.PrepareGroupReplayFrame(N) : Registry.PrepareFrame(N)); CHECK(Registry.InstallAll());
        auto S = Registry.ReadInstalled()->Inputs[0].Snapshot; CHECK(S);
        CHECK(Registry.BeginAll() && Registry.ValidateComplete()); CHECK(Registry.CompleteAll()); return S;
    };
    Publish(0, 0.25f, true); CHECK(Registry.ServiceBoundary()); auto S0 = Step(0, false); CHECK(S0->Applied[0] == 64);
    Publish(1, 1.0f, false); auto S1 = Step(1, false); CHECK(S1->Requested[0] == 255 && S1->Applied[0] == 74);
    Publish(2, 0.5f, false); auto S2 = Step(2, false); CHECK(S2->Requested[0] == 128 && S2->Applied[0] == 84);
    const auto Before = Journal->ReadInitialOwnerAdmission(); const auto Latest = Registry.ReadLatest();
    const auto BoundaryCapture=Registry.CaptureCompletedBoundary(2);CHECK(BoundaryCapture);
    const auto BoundaryToken=Registry.ReserveCompletedBoundary(BoundaryCapture);CHECK(BoundaryToken);
    const auto BoundaryWire=Speed::Netcode::BoundaryMetadata::PrepareRegistryMetadata(Registry,*BoundaryToken);CHECK(BoundaryWire);
    SaveBoundaryWireSample("real-device-filter-policy.irb",BoundaryWire->CanonicalBytes());
    const auto DeviceMetadata=Speed::Netcode::BoundaryMetadata::Decode(BoundaryWire->CanonicalBytes());CHECK(DeviceMetadata);
    CHECK(DeviceMetadata->Consumed[0].Origin==Speed::Netcode::BoundaryMetadata::EOrigin::Device);
    CHECK(DeviceMetadata->Consumed[0].Step[0]==10 && DeviceMetadata->Current[0].Step[0]==10);
    CHECK(DeviceMetadata->Closed[0].SourceSequence==3 && DeviceMetadata->Closed[0].CutoffSequence==3);
    CHECK(BoundaryWire->OriginCapture()->Completed()->Inputs[0].Snapshot->Requested[0]==128);
    CHECK(BoundaryWire->OriginCapture()->Completed()->Inputs[0].Snapshot->Applied[0]==84);
    Registry.ReleaseCompletedBoundaryReservation(*BoundaryToken);
    const auto AfterMetadata=Journal->ReadInitialOwnerAdmission();
    CHECK(Before.NextFrame==AfterMetadata.NextFrame && Before.DeliverySequence==AfterMetadata.DeliverySequence
        && Before.PhysicalFence==AfterMetadata.PhysicalFence && Before.Serial==AfterMetadata.Serial && Registry.PollCount(1)==3);
    V::FInputSessionRegistry::FGroupReplayRequest R{23, 1, 1, 2, 3, Registry.ReadRegistry()->Phases, {{100, 2}}};
    auto T = Registry.PrepareGroupRetainedReplayStart(R); CHECK(T && Registry.PrepareGroupReplayFinalization(*T));
    Registry.FinalizePreparedGroupReplay(*T); Registry.PublishPreparedGroupReplay(*T); Registry.ReleasePreparedGroupReplay(*T);
    CHECK(Step(1, true)->Applied == S1->Applied); CHECK(Step(2, true)->Applied == S2->Applied);
    const auto After = Journal->ReadInitialOwnerAdmission();
    CHECK(Before.NextFrame == After.NextFrame && Before.DeliverySequence == After.DeliverySequence
        && Before.PhysicalFence == After.PhysicalFence && Before.Serial == After.Serial && Before.ControlBarrier == After.ControlBarrier);
    CHECK(!After.HasFrozenCutoff && !After.Paused && Registry.PollCount(1) == 3 && Registry.ReadLatest() == Latest);
    CHECK(Registry.EndGroupRetainedReplay());
    Publish(3, 0.5f, false); CHECK(Step(3, false)->Applied[0] == 94 && Registry.PollCount(1) == 4);
}
#include "BoundaryCaptureTests.inc"
#include "BoundaryReservationTests.inc"
#include "BoundaryWireTests.inc"
int main(int Argc,char* Argv[])
{
    if(Argc==2 && std::string(Argv[1])=="--boundary-reservation-live-destruction")return DestroyWhileBoundaryReservationIsLive();
    if(Argc==2)BoundaryWireRunDirectory=Argv[1];
    if (ResumeRegressionMain()) return 1;
    try
    {
        BoundaryWireActualRegistryViews(); BoundaryWireReplacement(); BoundaryWireMalformedAndBounds(); BoundaryWireFailureAndReentrancy();
        BoundaryReservationPinsAndBlocks(); BoundaryReservationCommandFence(); BoundaryReservationFailuresAndTeardown(); BoundaryReservationRejectsStale();
        CompletedBoundaryViewsSurviveDetach(); BoundaryCapturePendingGroupsAndHistory(); BoundaryCaptureFailureAndLifetimes(); InitialBoundaryIsNotCompletedConsumption();
        ActualRegistryOwnerGroupReplay(); GroupReplayAbortFailuresAndAdmission(); GroupReplayCommandFenceAndTeardown();
        RealJournalDeviceRegistryReplaysAppliedWithoutAcquisition();
        std::cout << "PASS " << Checks << " checks; real registry+owner group retained replay; INPUT-only; native guards closed\n"; return 0;
    }
    catch (const std::exception& E) { Budget = -1; std::cerr << "FAIL " << Checks << " " << E.what() << '\n'; return 1; }
}
