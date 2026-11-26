static void ReadyForHistoricalReplay(FFixture& F)
{
    CHECK(F.Owner->ResumeAtBoundary()); Complete(*F.Owner, 1); Complete(*F.Owner, 2);
    CHECK(F.Source->Next == 3 && F.Owner->GetNextFrame() == 3);
}
static void PreparedReplayIsNotLifecycleResume()
{
    FFixture F; ReadyForHistoricalReplay(F);
    const auto Latest = F.Owner->ReadLatest(); const auto History = F.Owner->ReadCompleted(1);
    const int Freeze = F.Source->Freeze, Produce = F.Source->Produce, Close = F.Source->Close, Lifecycle = F.Source->LegacyLifecycle;
    const auto Poll = F.Owner->GetPollSerial();
    auto Token = F.Owner->PrepareRetainedReplayStart(1, 2); CHECK(Token);
    const auto View = F.Owner->ReadPreparedReplayStart(*Token);
    CHECK(View && View->First == 1 && View->Last == 2 && View->LiveNext == 3 && View->ReplayPass == 1 && View->Frames.size() == 2);
    CHECK(View->Frames[0] == History && View->Frames[1] == F.Owner->ReadCompleted(2));
    CHECK(F.Owner->Poll(1).Status == V::EOwnerInputStatus::Busy);
    CHECK(!F.Owner->PauseAtBoundary() && !F.Owner->ResumeAtBoundary() && !F.Owner->RetireAtBoundary());
    CHECK(!F.Owner->BeginReplay(1, 2)); CHECK(!F.Owner->PrepareRetainedReplayStart(0, 2));
    CHECK(!F.Owner->AppendExactScenarioFrame(MakeFrame(4)));
    CHECK(F.Owner->PrepareRetainedReplayFinalization(*Token));
    Allocations = 0; Budget = 0;
    F.Owner->FinalizeRetainedReplayStart(*Token);
    const auto Blocked = F.Owner->Poll(1).Status;
    std::shared_ptr<const V::FSameFrameInputOwner::FPreparedReplayView> CommonInputMarker = View;
    F.Owner->ReleaseRetainedReplayStart(*Token);
    Budget = -1;
    CHECK(!Allocations && Blocked == V::EOwnerInputStatus::Busy && CommonInputMarker == View);
    CHECK(!F.Owner->AbortRetainedReplayStart(*Token));
    for (I::FFrameNumber N = 1; N <= 2; ++N)
    {
        auto P = F.Owner->Poll(N); CHECK(P.Status == V::EOwnerInputStatus::Ready && P.Token);
        CHECK(F.Owner->Install(*P.Token)); auto S = F.Owner->ReadInstalled(*P.Token); CHECK(S);
        CHECK(S->Input.GetData().ConsumptionFrame == N && S->Input.GetData().SourceSequence == 2);
        CHECK(F.Owner->BeginStep(*P.Token, N)); CHECK(F.Owner->CompleteStep(*P.Token, N));
        const auto Receipt = F.Owner->ReadReplayReceipt(); CHECK(Receipt && Receipt->Replay && Receipt->ReplayPass == 1);
    }
    CHECK(F.Owner->Poll(2).Status == V::EOwnerInputStatus::ReplayComplete);
    CHECK(F.Owner->EndReplay());
    CHECK(F.Source->Freeze == Freeze && F.Source->Produce == Produce && F.Source->Close == Close && F.Source->LegacyLifecycle == Lifecycle);
    CHECK(F.Source->Next == 3 && F.Owner->GetNextFrame() == 3 && F.Owner->GetPollSerial() == Poll);
    CHECK(F.Owner->ReadLatest() == Latest && F.Owner->ReadCompleted(1) == History);
    Complete(*F.Owner, 3); // exact non-Reset live successor, no synthetic baseline
    CHECK(F.Source->Next == 4 && F.Owner->GetNextFrame() == 4 && F.Owner->GetPollSerial() == Poll + 1);
}
static void ReplayPreparationAbortsWithoutConsumingHistoryOrSource()
{
    for (int Limit = 0; Limit < 24; ++Limit)
    {
        FFixture F; ReadyForHistoricalReplay(F);
        const auto Latest = F.Owner->ReadLatest();
        Budget = Limit; auto Token = F.Owner->PrepareRetainedReplayStart(0, 2); Budget = -1;
        if (Token) CHECK(F.Owner->AbortRetainedReplayStart(*Token));
        CHECK(F.Source->Next == 3 && F.Source->Close == 3 && F.Owner->GetPollSerial() == 3 && F.Owner->ReadLatest() == Latest);
        auto Retry = F.Owner->PrepareRetainedReplayStart(0, 2); CHECK(Retry);
        const auto View = F.Owner->ReadPreparedReplayStart(*Retry); CHECK(View && View->ReplayPass == 1);
        CHECK(F.Owner->PrepareRetainedReplayFinalization(*Retry)); CHECK(F.Owner->AbortRetainedReplayStart(*Retry));
        CHECK(!F.Owner->ReadPreparedReplayStart(*Retry));
        auto Again = F.Owner->PrepareRetainedReplayStart(0, 2); CHECK(Again);
        CHECK(!F.Owner->AbortRetainedReplayStart(*Retry)); CHECK(F.Owner->AbortRetainedReplayStart(*Again));
    }
    FFixture F; ReadyForHistoricalReplay(F);
    CHECK(!F.Owner->PrepareRetainedReplayStart(2, 1)); CHECK(!F.Owner->PrepareRetainedReplayStart(0, 99));
    CHECK(!F.Owner->PrepareRetainedReplayStart(1, 3));
    { I::FPresentationInputScope Scope; CHECK(!F.Owner->PrepareRetainedReplayStart(0, 2)); }
    auto Token = F.Owner->PrepareRetainedReplayStart(0, 2); CHECK(Token);
    bool OtherPrepared = true, OtherAborted = true;
    std::thread Other([&] { OtherPrepared = F.Owner->PrepareRetainedReplayFinalization(*Token); OtherAborted = F.Owner->AbortRetainedReplayStart(*Token); });
    Other.join(); CHECK(!OtherPrepared && !OtherAborted);
    { I::FPresentationInputScope Scope; CHECK(!F.Owner->PrepareRetainedReplayFinalization(*Token) && !F.Owner->AbortRetainedReplayStart(*Token)); }
    CHECK(F.Owner->AbortRetainedReplayStart(*Token));
}
// Authored raw GAMEPAD DTO fixture -> real frozen FDeviceInputProducer/mapper.
// No platform API/physical device and no source-family relabeling is involved.
class FRawReplayFixture final : public V::IRawInputSource, public V::IInputProducerPollFence
{
public:
    I::FFrameNumber Next = 0;
    int Freezes = 0, Polls = 0, Closes = 0;
    V::IInputProducerPollFence* PollFence() noexcept override { return this; }
    std::optional<V::FInputPollCutoff> FreezeForOwner(I::FFrameNumber N) override
    {
        if (Frozen || N != Next || N > 3) return {};
        ++Freezes; Frozen = true; Delivered = false; return V::FInputPollCutoff{N + 1, 1, N + 1};
    }
    std::optional<V::FRawInputSample> Poll(I::FFrameNumber N) override
    {
        if (!Frozen || Delivered || N != Next) return {};
        ++Polls; Delivered = true;
        V::FRawInputSample S; S.DeviceId = 73; S.Generation = {1}; S.Kind = V::ERawDeviceKind::Gamepad; S.Sequence = N + 1;
        S.Status = N ? V::ERawSampleStatus::Valid : V::ERawSampleStatus::Resync;
        const float Value = N == 0 ? 0.25f : N == 1 ? 1.0f : 0.5f;
        S.FinalState.push_back({{V::ERawControlKind::PadAxis, std::uint16_t(V::EPadAxis::LeftTrigger)}, Value});
        if (N == 1 || N == 2) S.Changes.push_back({S.FinalState[0], {N + 1, 0}, 0});
        return S;
    }
    bool CloseFrozenCutoff(const V::FInputPollCutoff& C) noexcept override
    {
        if (!Frozen || !Delivered || C.Sequence != Next + 1 || C.Generation != 1 || C.LifecycleFence != Next + 1) return false;
        ++Closes; ++Next; Frozen = Delivered = false; return true;
    }
private:
    bool Frozen = false, Delivered = false;
};
static void RealDeviceMapperFilterReplayedWithoutRawAcquisition()
{
    V::FInputActionContractDescription D; D.Revision.Value = 23; D.Actions = V::FInputActionContract::BaseActions();
    D.Mapping.push_back({{V::ERawControlKind::PadAxis, std::uint16_t(V::EPadAxis::LeftTrigger)}, 0, 1, {}});
    D.Physical = {{0, V::EPhysicalDestination::Throttle}, {1, V::EPhysicalDestination::Brake}, {2, V::EPhysicalDestination::Steering}};
    auto C = V::FInputActionContract::Create(std::move(D)); CHECK(C);
    auto Raw = std::make_shared<FRawReplayFixture>();
    auto Device = V::FDeviceInputProducer::Create(Raw, C, {7}, {I::EProducerKind::Device, 99}, 0); CHECK(Device);
    auto B = Binding(); B.Contract = C; B.ProducerContract = V::EProducerContract::Device; B.Processing.Step[0] = 10;
    auto O = V::FSameFrameInputOwner::Create(std::move(Device), B, 0); CHECK(O && O->HasFilterState());
    Complete(*O, 0); Complete(*O, 1); Complete(*O, 2);
    const auto At1 = O->ReadCompleted(1), At2 = O->ReadCompleted(2);
    CHECK(At1->Snapshot->Requested[0] == 255 && At1->Snapshot->Applied[0] == 74);
    CHECK(At2->Snapshot->Requested[0] == 128 && At2->Snapshot->Applied[0] == 84);
    const auto LiveApplied = O->ReadBoundaryApplied();
    auto Token = O->PrepareRetainedReplayStart(1, 2); CHECK(Token && O->PrepareRetainedReplayFinalization(*Token));
    O->FinalizeRetainedReplayStart(*Token); O->ReleaseRetainedReplayStart(*Token);
    for (I::FFrameNumber N = 1; N <= 2; ++N)
    {
        auto P = O->Poll(N); CHECK(P.Token && P.Status == V::EOwnerInputStatus::Ready); CHECK(O->Install(*P.Token));
        const auto S = O->ReadInstalled(*P.Token); CHECK(S && S->Applied == (N == 1 ? At1 : At2)->Snapshot->Applied);
        CHECK(O->BeginStep(*P.Token, N)); CHECK(O->CompleteStep(*P.Token, N));
    }
    CHECK(O->EndReplay() && O->ReadBoundaryApplied() == LiveApplied);
    CHECK(Raw->Next == 3 && Raw->Freezes == 3 && Raw->Polls == 3 && Raw->Closes == 3 && O->GetPollSerial() == 3);
    CHECK(O->ReadCompleted(1) == At1 && O->ReadCompleted(2) == At2);
    Complete(*O, 3);
    CHECK(Raw->Next == 4 && O->ReadCompleted(3)->Snapshot->Applied[0] == 94);
}
