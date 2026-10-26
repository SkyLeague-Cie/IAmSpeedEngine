#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/World/Simulation/FastSimulation.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Configs/SubBodyConfig.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"

// Test-only access to the existing virtual entry points. No new runtime driver.
struct FLocalFastBatchFixtureAccess
{
    static void Initialize(ASpeedPhysicsSimulation& Driver, USpeedWorldSubsystem& World)
    {
        Driver.SpeedWorldSubsystem = &World;
        Driver.ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
        Driver.InitializeCanonicalFrame(0.f);
    }
    static ESimulationWorkerResult Drive(ASpeedPhysicsSimulation& Driver, bool Batch)
    {
        return Batch ? Driver.DriveSimulation(1.f / 300.f, 0.f)
                     : Driver.DriveOwnedWorkerPulse(1.f / 300.f, 0.f);
    }
    static uint64 Frame(const ASpeedPhysicsSimulation& Driver) { return Driver.CanonicalNumFrame; }
};

namespace
{
struct FBatchCVar
{
    IConsoleVariable* Variable;
    int32 Previous;
    FBatchCVar(const TCHAR* Name, int32 Value)
        : Variable(IConsoleManager::Get().FindConsoleVariable(Name)),
          Previous(Variable ? Variable->GetInt() : 0)
    { if (Variable) Variable->Set(Value, ECVF_SetByCode); }
    ~FBatchCVar() { if (Variable) Variable->Set(Previous, ECVF_SetByCode); }
};

// Scheduler fixture only: no fabricated body integration or collision oracle.
class FSealedBatchControl final : public ISpeedComponent
{
public:
    bool Ready = false, Frozen = false;
    uint64 Published = 0;
    uint32 CurrentFrame = 0;
    uint8 Latched = 0;
    uint32 Applied = 0;
    SKinematic State;
    FVector Local = FVector::ZeroVector;
    TArray<USSubBody*> Bodies;
    TArray<uint64> Prepared, Committed;
    TArray<float> Deltas;
    TArray<uint8> Values;
    FSealedBatchControl()
    { Prepared.Reserve(90); Committed.Reserve(90); Deltas.Reserve(90); Values.Reserve(90); }
    bool IsCanonicalRunController() const override { return true; }
    bool IsCanonicalRunReady() const override { return Ready; }
    bool IsCanonicalRunComplete() const override { return Published == 90; }
    void PrepareCanonicalFrame(const FCanonicalFrameContext& C) override
    { CurrentFrame = uint32(C.NumFrame) + 1; Prepared.Add(C.NumFrame); Deltas.Add(C.PhysicalDeltaTime); }
    bool ApplySimulationInput(TConstArrayView<uint8> Input) override
    { if (Input.Num() != 1) return false; Latched = Input[0]; ++Applied; return true; }
    void AppendSimulationSnapshot(TArray<uint8>& Out) const override
    {
        using W = Speed::FEpisodeStateBytesForTesting;
        W::Add(Out, CurrentFrame); W::Add(Out, Published); W::Add(Out, Latched); W::Add(Out, Applied);
    }
    void OnCanonicalFramePublished(uint64 N) override
    { Published = N + 1; Committed.Add(N); Values.Add(Latched); }
    unsigned int NumFrame() const override { return CurrentFrame; }
    float GetPhysMass() const override { return 1.f; }
    const FVector& GetPhysCOM() const override { return State.Location; }
    const FVector& GetPhysCenterOfMassLocal() const override { return Local; }
    const TArray<USSubBody*>& GetSubBodies() const override { return Bodies; }
    TArray<USSubBody*> CreateSubBodies() override { return Bodies; }
    SubBodyConfig GetSubBodyConfig(const USSubBody&) const override { return {}; }
    bool IsFrozen() const override { return Frozen; }
    void SetIsFrozen(bool Value) override { Frozen = Value; }
    SKinematic GetKinematicsOfSubBody(const USSubBody&, const unsigned int&) const override { return State; }
    FMatrix ComputeWorldInvInertiaTensor() const override { return FMatrix::Identity; }
    FMatrix ComputeWorldInvInertiaTensorOfSubBody(const USSubBody&) const override { return FMatrix::Identity; }
    const SKinematic& GetKinematicState() const override { return State; }
    const SKinematic& GetKinematicStateForFrame(const unsigned int&) const override { return State; }
    void SetKinematicState(const SKinematic& Value) override { State = Value; }
    float GetPhysMaxSpeed() const override { return 10000.f; }
    float GetPhysMaxAngularSpeed() const override { return 100.f; }
    void SetIsUpsideDown(bool) override {}
    bool IsUpsideDown() const override { return false; }
    bool IsInAutoRecover() const override { return false; }
    bool IsSubBodyInAutoRecoverMode() const override { return false; }
    void RcvImpactOnSubBody(const USSubBody&, const FVector&) override {}
    void PostPhysicsUpdatePrv(const float&) override {}
};

struct FBatchResult
{
    Speed::SimulationBoundary::FFrameHashJournal Hashes;
    TArray<uint64> Frames;
    TArray<uint8> Values;
};

bool RunBatchFixture(FAutomationTestBase& Test, bool Batch, bool Exhaust, FBatchResult& Out)
{
    FBatchCVar Canonical(TEXT("p.IAmSpeed.CanonicalFrame.Enabled"), 1);
    FBatchCVar Fast(TEXT("p.IAmSpeed.FastSimulation.Enabled"), 1);
    FBatchCVar Mode(TEXT("p.IAmSpeed.Simulation.ExecutionMode"), 2);
    FBatchCVar Delay(TEXT("p.IAmSpeed.FastSimulation.ReadyDelayCallbacks"), 0);
    FBatchCVar Budget(TEXT("p.IAmSpeed.FastSimulation.MaxFramesPerRun"), Exhaust ? 16 : 90);
    FBatchCVar Backend(TEXT("p.IAmSpeed.StaticCollision.Backend"), 0);
    if (!Test.TestTrue(TEXT("all explicit fixture policies exist"),
        Canonical.Variable && Fast.Variable && Mode.Variable && Delay.Variable && Budget.Variable && Backend.Variable)) return false;
    const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false).SetTransactional(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true, ERHIFeatureLevel::Num, &Options);
    if (!Test.TestNotNull(TEXT("isolated scheduler world"), World)) return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
    auto* Driver = World->SpawnActor<AFastSimulation>();
    auto* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
    if (!Test.TestNotNull(TEXT("real Fast driver"), Driver) || !Test.TestNotNull(TEXT("real registry"), Bridge)) return false;
    FSealedBatchControl Control;
    Bridge->RegisterSpeedComponent(&Control); Bridge->ApplyPendingOps();
    ON_SCOPE_EXIT { Bridge->UnregisterSpeedComponent(&Control); Bridge->ApplyPendingOps(); };
    FLocalFastBatchFixtureAccess::Initialize(*Driver, *Bridge);
    const uint64 Id = Bridge->GetSimulationStableId(Control);
    if (!Test.TestTrue(TEXT("registered controller has an identity"), Id != 0)) return false;
    for (const auto Command : {TPair<uint64, uint8>(0, 3), TPair<uint64, uint8>(17, 7), TPair<uint64, uint8>(47, 11)})
    {
        const TArray<uint8> Payload = {Command.Value};
        if (!Test.TestTrue(TEXT("frame-addressed input accepted before sealing"),
            Driver->QueueSimulationInput(Command.Key, Id, Payload))) return false;
    }
    Driver->SealSimulationInputs();
    bool SawIdle = false, Invalid = false;
    uint32 AdvancingCalls = 0;
    ESimulationWorkerResult Final = ESimulationWorkerResult::Idle;
    FSimulationWorker Owner([&]
    {
        if (!SawIdle)
        {
            const auto Idle = FLocalFastBatchFixtureAccess::Drive(*Driver, Batch);
            Invalid |= Idle != ESimulationWorkerResult::Idle || FLocalFastBatchFixtureAccess::Frame(*Driver) != 0 || !Control.Prepared.IsEmpty();
            SawIdle = true; Control.Ready = true;
            return Invalid ? ESimulationWorkerResult::Failed : ESimulationWorkerResult::Idle;
        }
        const uint64 Before = FLocalFastBatchFixtureAccess::Frame(*Driver);
        Final = FLocalFastBatchFixtureAccess::Drive(*Driver, Batch); ++AdvancingCalls;
        const uint64 After = FLocalFastBatchFixtureAccess::Frame(*Driver);
        if (!Batch) Invalid |= After != Before + 1;
        if (Final == ESimulationWorkerResult::Complete || Final == ESimulationWorkerResult::Failed)
        {
            const auto Again = FLocalFastBatchFixtureAccess::Drive(*Driver, Batch);
            Invalid |= Again != Final || FLocalFastBatchFixtureAccess::Frame(*Driver) != After;
            // Expected budget refusal is captured as Final, never converted into
            // a successful driver result. Complete only closes this test worker.
            return ESimulationWorkerResult::Complete;
        }
        return Invalid ? ESimulationWorkerResult::Failed : Final;
    }, [](FSimulationWorkerWaitContext&) {});
    if (!Test.TestTrue(TEXT("owned native worker starts"), Owner.Start())) return false;
    const double Deadline = FPlatformTime::Seconds() + 60.;
    while (Owner.IsRunning() && FPlatformTime::Seconds() < Deadline) FPlatformProcess::SleepNoStats(.001f);
    const bool Finished = !Owner.IsRunning(); Owner.StopAndJoin();
    if (!Test.TestTrue(TEXT("bounded joined owner"), Finished && SawIdle && !Invalid)) return false;
    const int32 Count = Exhaust ? 16 : 90;
    bool Valid = Control.Prepared.Num() == Count && Control.Committed.Num() == Count && Control.Values.Num() == Count
        && Control.Deltas.Num() == Count && Driver->GetFrameHashes().Num() == Count
        && Final == (Exhaust ? ESimulationWorkerResult::Failed : ESimulationWorkerResult::Complete)
        && AdvancingCalls == (Batch ? 1u : uint32(Count)) && Control.Applied == (Exhaust ? 1u : 3u)
        && !Driver->AllowsCameraPresentation();
    for (int32 N = 0; N < Count && Valid; ++N)
    {
        const float Delta = 1.f / 300.f;
        const uint8 Expected = N < 17 ? 3 : (N < 47 ? 7 : 11);
        Valid &= Control.Prepared[N] == uint64(N) && Control.Committed[N] == uint64(N)
            && Control.Values[N] == Expected && FMemory::Memcmp(&Control.Deltas[N], &Delta, sizeof(float)) == 0;
    }
    if (!Test.TestTrue(TEXT("all frames, inputs, exact delta and terminal policy retained"), Valid)) return false;
    Out.Hashes = Driver->GetFrameHashes(); Out.Frames = Control.Committed; Out.Values = Control.Values;
    return true;
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFastBatchSealedRun,
    "IAmSpeed.Simulation.FastBatch.SealedRun", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFastBatchSealedRun::RunTest(const FString&)
{
    FBatchResult Pulses, Batch;
    if (!RunBatchFixture(*this, false, false, Pulses) || !RunBatchFixture(*this, true, false, Batch)) return false;
    return TestTrue(TEXT("complete batch equals every individual pulse publication"),
        Pulses.Frames == Batch.Frames && Pulses.Values == Batch.Values && !Pulses.Hashes.FirstDivergence(Batch.Hashes).IsSet());
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedFastBatchFrameBudget,
    "IAmSpeed.Simulation.FastBatch.FrameBudget", EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FIAmSpeedFastBatchFrameBudget::RunTest(const FString&)
{
    AddExpectedError(TEXT("FastSimulation exhausted its frame budget"), EAutomationExpectedErrorFlags::Contains, 1);
    FBatchResult Prefix;
    return RunBatchFixture(*this, true, true, Prefix);
}
#endif
