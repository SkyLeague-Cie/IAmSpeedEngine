// Authored raw gamepad samples exercise the real mapper, values-only wire codec,
// ingress and authoritative owner. No platform hardware or native physics claim.
class FRawSlewFixture final : public V2::IRawInputSource, public V2::IInputProducerPollFence
{
public:
    std::uint64_t Next = 0;
    V2::IInputProducerPollFence* PollFence() noexcept override { return this; }
    std::optional<V2::FInputPollCutoff> FreezeForOwner(std::uint64_t N) override
    {
        if (Frozen || N != Next || N >= 8) return {};
        Frozen = true; Delivered = false; return V2::FInputPollCutoff{N + 1, N < 4 ? 1u : 2u, N + 1};
    }
    std::optional<V2::FRawInputSample> Poll(std::uint64_t N) override
    {
        if (!Frozen || Delivered || N != Next) return {};
        Delivered = true;
        V2::FRawInputSample S; S.DeviceId = 73; S.Generation = {N < 4 ? 1u : 2u};
        S.Kind = V2::ERawDeviceKind::Gamepad; S.Sequence = N + 1;
        S.Status = N == 0 || N == 4 ? V2::ERawSampleStatus::Resync : V2::ERawSampleStatus::Valid;
        const float Steering[] = {0.0f, 1.0f, -1.0f, -1.0f, -0.5f, 1.0f, 0.0f, -1.0f};
        const float Throttle[] = {0.25f, 1.0f, 0.0f, 0.0f, 0.75f, 0.0f, 1.0f, 0.0f};
        S.FinalState = {{{V2::ERawControlKind::PadAxis, std::uint16_t(V2::EPadAxis::LeftX)}, Steering[N]},
            {{V2::ERawControlKind::PadAxis, std::uint16_t(V2::EPadAxis::LeftTrigger)}, Throttle[N]},
            {{V2::ERawControlKind::PadAxis, std::uint16_t(V2::EPadAxis::RightTrigger)}, 0.0f}};
        if (S.Status == V2::ERawSampleStatus::Valid)
            for (std::size_t A = 0; A < S.FinalState.size(); ++A)
                if (S.FinalState[A].Value != Previous[A])
                    S.Changes.push_back({S.FinalState[A], {N + 1, std::uint32_t(S.Changes.size())}, 0});
        for (std::size_t A = 0; A < S.FinalState.size(); ++A) Previous[A] = S.FinalState[A].Value;
        return S;
    }
    bool CloseFrozenCutoff(const V2::FInputPollCutoff& C) noexcept override
    {
        if (!Frozen || !Delivered || C.Sequence != Next + 1 || C.Generation != (Next < 4 ? 1u : 2u) || C.LifecycleFence != Next + 1) return false;
        ++Next; Frozen = Delivered = false; return true;
    }
private:
    std::array<float, 3> Previous{};
    bool Frozen = false, Delivered = false;
};
static std::shared_ptr<const V2::FInputActionContract> SlewContract()
{
    V2::FInputActionContractDescription D; D.Revision.Value = 73; D.Actions = V2::FInputActionContract::BaseActions();
    D.Mapping = {{{V2::ERawControlKind::PadAxis, std::uint16_t(V2::EPadAxis::LeftX)}, 2, 1, {}},
        {{V2::ERawControlKind::PadAxis, std::uint16_t(V2::EPadAxis::LeftTrigger)}, 0, 1, {}},
        {{V2::ERawControlKind::PadAxis, std::uint16_t(V2::EPadAxis::RightTrigger)}, 1, 1, {}}};
    D.Physical = {{0, V2::EPhysicalDestination::Throttle}, {1, V2::EPhysicalDestination::Brake}, {2, V2::EPhysicalDestination::Steering}};
    return V2::FInputActionContract::Create(std::move(D));
}
static void CompleteSlewOwner(V2::FSameFrameInputOwner& O, std::uint64_t N)
{
    auto P = O.Poll(N); CHECK(P.Status == V2::EOwnerInputStatus::Ready && P.Token);
    CHECK(O.Install(*P.Token) && O.BeginStep(*P.Token, N)); CHECK(O.CompleteStep(*P.Token, N));
}
static void OriginalRawFramesHaveLocalAndRemoteProcessingParity()
{
    const auto Contract = SlewContract(); CHECK(Contract);
    auto D = RemoteDescriptor(1); D.Contract = Contract;
    D.Processing.Step[0] = D.Processing.Step[1] = D.Processing.Step[2] = 16;
    auto Raw = std::make_shared<FRawSlewFixture>();
    V2::FOwnerInputBinding LocalBinding{D.Id, {Speed::Input::EProducerKind::Device, D.Producer}, {D.Epoch}, Contract, D.Processing, V2::EProducerContract::Device};
    auto Local = V2::FSameFrameInputOwner::Create(V2::FDeviceInputProducer::Create(Raw, Contract, {D.Epoch}, LocalBinding.Producer, 0), LocalBinding);
    CHECK(Local && Local->HasFilterState());
    auto Ingress = RemoteService(D), OwnerIngress = RemoteService(D);
    auto Commands = std::make_shared<V2::FInputSessionCommands>();
    const auto Admission = V2::EExactRemoteAdmission::OriginalDeviceAuthoritativeProcessing;
    V2::FInputSessionRegistry Registry(Commands, 7, {{D.Journal, {}, {}, Ingress}}, Admission);
    CHECK(Commands->Submit(RemoteBind(D, 1, 0)) == V2::ECommandAdmission::Enqueued);
    CHECK(Registry.ServiceBoundary(0) && Commands->Read(1)->Status == V2::EBoundaryStatus::Applied);
    CHECK(Registry.HasFilterState(D.Id));
    auto RemoteBinding = LocalBinding; RemoteBinding.ProducerContract = V2::EProducerContract::ExactRemote;
    auto Remote = V2::FSameFrameInputOwner::Create(OwnerIngress->CreateProducerForCurrentOwner(), RemoteBinding, 0, Admission);
    CHECK(Remote && Remote->HasFilterState() && Remote->GetProducerContract() == V2::EProducerContract::ExactRemote);
    CHECK(Local->GetBindingFingerprint() != Remote->GetBindingFingerprint());
    auto ChangedBinding = RemoteBinding; ++ChangedBinding.Processing.Step[0];
    auto ExtraIngress = RemoteService(D);
    auto ChangedOwner = V2::FSameFrameInputOwner::Create(ExtraIngress->CreateProducerForCurrentOwner(), ChangedBinding, 0, Admission);
    CHECK(ChangedOwner && ChangedOwner->GetBindingFingerprint() != Remote->GetBindingFingerprint());
    CHECK(ChangedOwner->RetireAtBoundary());
    auto WireBinding = Fixture::Binding(); WireBinding.Session = D.Id; WireBinding.SourceProducer = D.Producer;
    std::vector<V2::FInputFrame> Originals;
    for (std::uint64_t N = 0; N < 8; ++N)
    {
        const auto Before = Local->ReadBoundaryApplied(); CompleteSlewOwner(*Local, N);
        const auto L = Local->ReadCompleted(N)->Snapshot; Originals.push_back(L->Input);
        auto Packet = FromV2(L->Input, WireBinding, EKind::OwnerInput, N + 1); CHECK(Packet);
        auto Bytes = Encode(*Packet); CHECK(Bytes); auto Decoded = Decode(Bytes->data(), Bytes->size()); CHECK(Decoded);
        auto Received = ToV2(*Decoded, Contract); CHECK(Received);
        CHECK(FromV2(*Received, WireBinding, EKind::OwnerInput, N + 1)->Input.Values == L->Requested);
        CHECK(Ingress->SubmitFrame(*Received) == ERemoteSubmission::Accepted);
        CHECK(Ingress->SubmitFrame(*Received) == ERemoteSubmission::Duplicate);
        CHECK(OwnerIngress->SubmitFrame(*Received) == ERemoteSubmission::Accepted);
        CHECK(Registry.QueryFrameReadiness(N, 1) == V2::EInputFrameReadiness::Ready);
        CHECK(Registry.PrepareFrame(N) && Registry.InstallAll() && Registry.BeginAll());
        const auto Installed = Registry.ReadInstalled(); CHECK(Installed && Installed->Inputs.size() == 1);
        const auto& R = Installed->Inputs[0]; CHECK(R.FilterBefore && *R.FilterBefore == Before);
        CHECK(R.Snapshot->Requested == L->Requested && R.Snapshot->Applied == L->Applied);
        CHECK(R.Snapshot->Input.GetData().Producer.Id == D.Producer && R.Snapshot->Input.GetData().DeviceGeneration.Value == (N < 4 ? 1u : 2u));
        CHECK(R.Snapshot->CutoffSequence == L->CutoffSequence);
        CHECK(Encode(*FromV2(R.Snapshot->Input, WireBinding, EKind::OwnerInput, N + 1)) == Bytes);
        auto Expected = L->Requested;
        if (!L->Input.GetData().Reset)
            for (std::size_t A = 0; A < 3; ++A) Expected[A] = std::int16_t(int(Before[A]) + std::clamp(int(Expected[A]) - int(Before[A]), -16, 16));
        CHECK(Expected == L->Applied);
        CHECK(V2::ProcessAuthoritativeInputValues(D.Processing, L->Requested, Before, L->Input.GetData().Reset) == Expected);
        if (N == 1 || N == 2 || N == 5) CHECK(L->Requested != L->Applied);
        if (N == 0 || N == 4) CHECK(L->Input.GetData().Reset && L->Requested == L->Applied);
        CHECK(Registry.ValidateComplete() && Registry.CompleteAll());
        CompleteSlewOwner(*Remote, N); CHECK(Remote->ReadCompleted(N)->Snapshot->Applied == L->Applied);
    }
    CHECK(Registry.BeginRetainedReplay(0, 7, D.Actors));
    for (std::uint64_t N = 0; N < 8; ++N) CHECK(Registry.NextRetainedReplay() == Registry.ReadHistorical(N));
    CHECK(Registry.EndRetainedReplay());
    const auto Serial = Remote->GetPollSerial(); CHECK(Remote->BeginReplay(1, 7));
    for (std::uint64_t N = 1; N < 8; ++N)
    {
        CompleteSlewOwner(*Remote, N); const auto Receipt = Remote->ReadReplayReceipt();
        CHECK(Receipt && Receipt->Replay && Receipt->Snapshot->Applied == Local->ReadCompleted(N)->Snapshot->Applied);
        CHECK(Encode(*FromV2(Receipt->Snapshot->Input, WireBinding, EKind::OwnerInput, N + 1)) == Encode(*FromV2(Originals[N], WireBinding, EKind::OwnerInput, N + 1)));
    }
    CHECK(Remote->EndReplay() && Remote->GetPollSerial() == Serial && Raw->Next == 8);
    // Rebinding must reconstruct filter state under the same explicit admission.
    CHECK(Remote->PauseAtBoundary());
    auto NewBinding = RemoteBinding; ++NewBinding.Epoch.Value;
    auto NewIngress = FExactRemoteInputIngress::Create(Contract, NewBinding.Epoch, NewBinding.Producer, 8); CHECK(NewIngress);
    const auto PriorFingerprint = Remote->GetBindingFingerprint();
    CHECK(Remote->RebindAtBoundary(NewIngress->CreateProducerForCurrentOwner(), NewBinding, 8));
    CHECK(Remote->HasFilterState() && Remote->GetBindingFingerprint() != PriorFingerprint);
    auto FreshData = Originals[7].GetData(); FreshData.ConsumptionFrame = 8; FreshData.SourceSequence = 9;
    FreshData.StreamEpoch = NewBinding.Epoch; FreshData.Reset = true; FreshData.Transitions.clear();
    FreshData.Values[0] = 255; FreshData.Values[2] = -127; FreshData.ActiveMask |= 5;
    CHECK(NewIngress->SubmitFrame(V2::FInputFrame(Contract, FreshData)) == ERemoteSubmission::Accepted);
    CompleteSlewOwner(*Remote, 8); CHECK(Remote->ReadCompleted(8)->Snapshot->Applied == FreshData.Values);
    auto PendingData = FreshData; PendingData.ConsumptionFrame = 9; PendingData.SourceSequence = 10; PendingData.Reset = false;
    CHECK(NewIngress->SubmitFrame(V2::FInputFrame(Contract, PendingData)) == ERemoteSubmission::Accepted);
    const auto Last = Remote->ReadLatest(); auto Pending = Remote->Poll(9); CHECK(Pending.Token);
    CHECK(Remote->Abort(*Pending.Token) && Remote->ReadCompleted(8) == Last && !Remote->ReadCompleted(9));
    CHECK(Remote->GetPhase() == V2::EOwnerInputPhase::Quarantined);
    CHECK(Remote->RetireAtBoundary()); CHECK(Registry.CloseWorkerLedger());
    CHECK(Ingress->GetState() == ERemoteIngressState::Retired && OwnerIngress->GetState() == ERemoteIngressState::Retired);
}
static void RemoteProcessingRequiresExplicitOptInAndValidPolicy()
{
    for (int Case = 0; Case < 6; ++Case)
    {
        auto D = RemoteDescriptor(1); D.Processing.Step[0] = 16;
        auto Admission = V2::EExactRemoteAdmission::OriginalDeviceAuthoritativeProcessing;
        if (Case == 0) Admission = V2::EExactRemoteAdmission::Closed;
        if (Case == 1) Admission = V2::EExactRemoteAdmission::OriginalDeviceExact;
        if (Case == 2) D.Processing.Step[0] = 65535;
        if (Case == 3) D.Processing.Step[3] = 1; // digital action
        if (Case == 4) D.Processing.Step[31] = 1; // unwired/absent action
        if (Case == 5) Admission = static_cast<V2::EExactRemoteAdmission>(255);
        auto Ingress = RemoteService(D); auto Commands = std::make_shared<V2::FInputSessionCommands>();
        V2::FInputSessionRegistry Registry(Commands, 7, {}, Admission);
        const bool Registered = Registry.RegisterJournalAtBoundary({D.Journal, {}, {}, Ingress});
        CHECK(Registered == (Case >= 1 && Case <= 4));
        CHECK(Commands->Submit(RemoteBind(D, 1, 0)) == V2::ECommandAdmission::Enqueued);
        CHECK(Registry.ServiceBoundary(0) && Commands->Read(1)->Status == V2::EBoundaryStatus::Rejected);
        CHECK(Registry.ConstructionCount() == 0 && Ingress->GetState() == ERemoteIngressState::Unbound);
    }
}

static void ZeroStepRemoteProcessingHasDistinctFingerprintAndState()
{
    const auto D = RemoteDescriptor(1);
    V2::FOwnerInputBinding B{D.Id, {Speed::Input::EProducerKind::Device, D.Producer}, {D.Epoch}, D.Contract, {}, V2::EProducerContract::ExactRemote};
    auto A = RemoteService(D), C = RemoteService(D);
    auto Exact = V2::FSameFrameInputOwner::Create(A->CreateProducerForCurrentOwner(), B, 0, V2::EExactRemoteAdmission::OriginalDeviceExact);
    auto Processed = V2::FSameFrameInputOwner::Create(C->CreateProducerForCurrentOwner(), B, 0, V2::EExactRemoteAdmission::OriginalDeviceAuthoritativeProcessing);
    CHECK(Exact && Processed && !Exact->HasFilterState() && Processed->HasFilterState());
    CHECK(Exact->GetBindingFingerprint() != Processed->GetBindingFingerprint());
    for (std::uint64_t N = 0; N < 2; ++N)
    {
        CHECK(A->SubmitFrame(RemoteFrame(1, N)) == ERemoteSubmission::Accepted);
        CHECK(C->SubmitFrame(RemoteFrame(1, N)) == ERemoteSubmission::Accepted);
        CompleteSlewOwner(*Exact, N); CompleteSlewOwner(*Processed, N);
        CHECK(Exact->ReadCompleted(N)->Snapshot->Applied == Processed->ReadCompleted(N)->Snapshot->Applied);
    }
    CHECK(Exact->RetireAtBoundary() && Processed->RetireAtBoundary());
}
