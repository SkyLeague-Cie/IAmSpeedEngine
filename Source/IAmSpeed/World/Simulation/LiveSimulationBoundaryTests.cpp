#if WITH_DEV_AUTOMATION_TESTS
#include "LiveSimulationBoundary.h"
#include "RealTimeSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/AutomationTest.h"
#include "Misc/ScopeExit.h"
#include <atomic>
#include <type_traits>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedLiveBoundary01621Test,
    "IAmSpeed.Simulation.LiveBoundary01621",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedLiveBoundary01621Test::RunTest(const FString&)
{
    using namespace Speed::SimulationBoundary;
    static_assert(!std::is_default_constructible_v<FLiveBoundaryLease>);
    static_assert(!std::is_constructible_v<FLiveBoundaryLease, uint64>);
    static_assert(!std::is_copy_constructible_v<FLiveBoundaryLease>);
    static_assert(!std::is_move_constructible_v<FLiveBoundaryLease>);
    const auto Options = UWorld::InitializationValues().AllowAudioPlayback(false).CreatePhysicsScene(false)
        .RequiresHitProxies(false).CreateNavigation(false).CreateAISystem(false)
        .ShouldSimulatePhysics(false).SetTransactional(false);
    auto* World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr, true, ERHIFeatureLevel::Num, &Options);
    if (!TestNotNull(TEXT("live boundary fixture world"), World)) return false;
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ON_SCOPE_EXIT { World->DestroyWorld(false); GEngine->DestroyWorldContext(World); };
    World->InitializeActorsForPlay(FURL());
    auto* Driver = World->SpawnActor<ARealTimeSimulation>();
    auto* Bridge = World->GetSubsystem<USpeedWorldSubsystem>();
    if (!TestNotNull(TEXT("live boundary driver"), Driver) || !TestNotNull(TEXT("real world bridge"), Bridge)) return false;
    FString Failure;
    TestFalse(TEXT("unstarted host cannot manufacture a live capability"), bool(Driver->RequestLiveBoundary(Failure)));
    // Empty immutable collision data isolates ownership without substituting a
    // fake physical frame: RunCanonicalFrames still executes the normal core.
    auto Data = MakeShared<Speed::Analytic::FAnalyticWorldData>();
    if (!TestTrue(TEXT("empty analytical world validates"), Data->FinalizeAndValidate())) return false;
    Bridge->AnalyticWorldData = Data;
    Bridge->SimulationWorld.SetStaticCollisionWorld(MakeUnique<Speed::FAnalyticStaticCollisionWorld>(*Data));
    Driver->SpeedWorldSubsystem = Bridge;
    Driver->DispatchBeginPlay();
    Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::IAmSpeedThread));
    Driver->bExecutionModeInitialized = true;
    Driver->InitializeCanonicalFrame(0);
    std::atomic<uint64> Steps{0}, Budget{0}, LastPacedStep{0};
    FRealTimeFramePacer Pacer;
    std::atomic<double> LastPacingInterval{0.};
    std::atomic<bool> Failed{false}, BlockNext{false};
    std::atomic<uint32> Services{0}, PacingResets{0};
    FEvent* BlockEntered = FPlatformProcess::GetSynchEventFromPool(true);
    FEvent* BlockRelease = FPlatformProcess::GetSynchEventFromPool(true);
    Driver->SimulationWorker = MakeUnique<FSimulationWorker>([&]()
    {
        if (BlockNext.exchange(false))
        {
            BlockEntered->Trigger();
            if (!BlockRelease->Wait(5000)) { Failed.store(true); return ESimulationWorkerResult::Failed; }
        }
        if (Steps.load() >= Budget.load()) return ESimulationWorkerResult::Idle;
        const double StartedAt = FPlatformTime::Seconds();
        const auto Pace = Pacer.BeginFrame(StartedAt, .001);
        LastPacingInterval.store(Pace.NextDeadlineSeconds - StartedAt);
        if (!Driver->RunCanonicalFrames(1)) { Failed.store(true); return ESimulationWorkerResult::Failed; }
        ++Steps;
        return ESimulationWorkerResult::Advanced;
        }, [&](FSimulationWorkerWaitContext& Wait)
    {
        if (Wait.WaitUntil(Pacer.GetNextDeadlineSeconds()) == ESimulationWaitResult::Interrupted) Pacer.Reset();
        LastPacedStep.store(Steps.load());
    },
    [&](bool) { ++Services; return ESimulationBoundaryResult::Ready; },
    [&]() { Driver->CloseInputSessionsOnWorker(); }, [&]() { Pacer.Reset(); ++PacingResets; });
    ON_SCOPE_EXIT
    {
        BlockRelease->Trigger(); Driver->StopOwnedWorker();
        FPlatformProcess::ReturnSynchEventToPool(BlockEntered);
        FPlatformProcess::ReturnSynchEventToPool(BlockRelease);
    };
    if (!TestTrue(TEXT("real canonical owner starts"), Driver->SimulationWorker->Start())) return false;
    const auto WaitFor = [&](TFunctionRef<bool()> Predicate)
    {
        const double Deadline = FPlatformTime::Seconds() + 2.;
        while (!Predicate() && FPlatformTime::Seconds() < Deadline) FPlatformProcess::SleepNoStats(.001f);
        return Predicate();
    };
    TestFalse(TEXT("missing real publication does not park or pause"), bool(Driver->RequestLiveBoundary(Failure)));
    TestFalse(TEXT("refusal preserves running user intent"), Driver->IsOwnedSimulationPaused());
    Budget.store(1);
    if (!TestTrue(TEXT("first real canonical frame publishes"), WaitFor([&]() { return Steps.load() == 1 || Failed.load(); })) || Failed.load()) return false;
    FSimulationSnapshot FrameZero;
    if (!TestTrue(TEXT("real frame zero has a publication"), Driver->ReadLatestSimulationSnapshot(FrameZero) && FrameZero.PublicationSerial && FrameZero.NumFrame == 0)) return false;
    TestFalse(TEXT("frame-zero publication cannot be relabeled as a live N>0 boundary"), bool(Driver->RequestLiveBoundary(Failure)));
    Budget.store(3);
    if (!TestTrue(TEXT("canonical owner advances to a real N>0 publication"), WaitFor([&]() { return (Steps.load() == 3 && LastPacedStep.load() == 3) || Failed.load(); })) || Failed.load()) return false;
    const uint64 Generation = Driver->GetInputWorkerGeneration();
    uint32 CallbackCalls = 0;
    uint64 BoundNextFrame = 0, BoundPublicationSerial = 0;
    auto AtBoundary = [&](const FLiveBoundaryLease& Lease, FString&)
    {
        ++CallbackCalls;
        TestTrue(TEXT("capability proves the actual driver/world/generation"), Lease.IsCurrent(*Driver)
            && Lease.Driver() == Driver && Lease.World() == World && Lease.WorkerGeneration() == Generation);
        TestTrue(TEXT("capability addresses the real completed and next frames"), Lease.CompletedFrame() == 2
            && Lease.NextFrame() == 3 && Lease.Snapshot().NumFrame == 2 && Lease.PublicationSerial() != 0);
        BoundNextFrame = Lease.NextFrame(); BoundPublicationSerial = Lease.PublicationSerial();
        // Park longer than the pacing recovery window; release must not carry
        // this wall-clock delay into shortened future physical intervals.
        FPlatformProcess::SleepNoStats(.02f);
        const uint32 FrozenServices = Services.load();
        auto* Unregistered = NewObject<USpeedMovementComponent>(World->SpawnActor<AActor>());
        uint32 Factories = 0;
        for (uint32 Index = 0; Index < 2; ++Index)
        {
            const auto Result = Driver->BindPresentationAtFrameBoundary(*Unregistered, static_cast<ISpeedComponent*>(nullptr),
                [&](const FSimulationPresentationBinding&) -> TSharedPtr<ISimulationPresentationProducer, ESPMode::ThreadSafe>
                { ++Factories; return {}; });
            TestFalse(TEXT("nested actual binder refuses an unregistered adapter"), Result.IsValid());
            TestTrue(TEXT("nested refusal retains the outer real lease"), Lease.IsCurrent(*Driver) && Driver->IsOwnedBoundaryServiceSuspended());
        }
        TestEqual(TEXT("invalid nested binding never enters its factory"), Factories, 0u);
        TestEqual(TEXT("nested bindings release only their own service depth"), Driver->OwnedBoundarySuspendDepth, 0u);
        TestEqual(TEXT("exclusive callback never overlaps lifecycle service"), Services.load(), FrozenServices);
        FSimulationSnapshot Still;
        TestTrue(TEXT("callback does not fabricate or advance physics"), Driver->ReadLatestSimulationSnapshot(Still)
            && Still.NumFrame == Lease.CompletedFrame() && Still.PublicationSerial == Lease.PublicationSerial());
        return true;
    };
    auto Request = Driver->RequestLiveBoundary(Failure);
    if (!TestTrue(TEXT("live request accepted after genuine canonical publication"), Request.IsValid())) return false;
    ELiveBoundaryResult Result = ELiveBoundaryResult::Waiting;
    if (!TestTrue(TEXT("callback and release reach proved disposition"), WaitFor([&]()
        { Result = Driver->PollLiveBoundary(*Request, AtBoundary, Failure); return Result != ELiveBoundaryResult::Waiting; }))) return false;
    TestTrue(TEXT("live callback completed"), Result == ELiveBoundaryResult::Completed);
    TestEqual(TEXT("callback runs once"), CallbackCalls, 1u);
    TestFalse(TEXT("external lease does not change user pause intent"), Driver->IsOwnedSimulationPaused());
    TestEqual(TEXT("real park resets pacing once"), PacingResets.load(), 1u);
    TestTrue(TEXT("completed request cannot rerun its callback"), Driver->PollLiveBoundary(*Request, AtBoundary, Failure) == ELiveBoundaryResult::Completed);
    TestEqual(TEXT("completed callback remains single-use"), CallbackCalls, 1u);
    Budget.store(4);
    if (!TestTrue(TEXT("physical owner continues after release"), WaitFor([&]() { return (Steps.load() == 4 && LastPacedStep.load() == 4) || Failed.load(); })) || Failed.load()) return false;
    TestTrue(TEXT("structural release discards real pacing recovery debt"), LastPacingInterval.load() >= .000999);
    FSimulationSnapshot After;
    TestTrue(TEXT("next real frame is consecutive without rebase or reset"), Driver->ReadLatestSimulationSnapshot(After)
        && After.NumFrame == BoundNextFrame && After.PublicationSerial == BoundPublicationSerial + 1
        && Driver->GetInputWorkerGeneration() == Generation);

    // A legacy pending suspension owns its original fail-closed semantics.
    BlockEntered->Reset(); BlockRelease->Reset(); BlockNext.store(true);
    if (!TestTrue(TEXT("real owner is in flight for legacy overlap"), BlockEntered->Wait(1000))) return false;
    TestTrue(TEXT("legacy suspension times out without an ACK"), Driver->TrySuspendOwnedBoundaryService(0) == ESimulationQuiescence::TimedOut);
    TestFalse(TEXT("live acquisition refuses a pending legacy suspension"), bool(Driver->RequestLiveBoundary(Failure)));
    BlockRelease->Trigger();
    if (!TestTrue(TEXT("legacy owner later claims its own ACK"), Driver->TrySuspendOwnedBoundaryService(1000) == ESimulationQuiescence::BoundaryAcknowledged)) return false;
    TestFalse(TEXT("live acquisition refuses an owned legacy suspension"), bool(Driver->RequestLiveBoundary(Failure)));
    Driver->ResumeOwnedBoundaryService(); Driver->ResumeOwnedSimulation();

    // A later actor/binding cleanup can take the original legacy barrier. Its
    // request cancels the pending live owner before the worker parks legacy.
    auto Superseded = Driver->RequestLiveBoundary(Failure);
    if (!TestTrue(TEXT("live request exists before later legacy cleanup"), Superseded.IsValid())) return false;
    if (!TestTrue(TEXT("later legacy cleanup obtains its own real ACK"), Driver->TrySuspendOwnedBoundaryService(1000) == ESimulationQuiescence::BoundaryAcknowledged)) return false;
    TestTrue(TEXT("superseded live request reports real cancellation"), Driver->PollLiveBoundary(*Superseded, AtBoundary, Failure) == ELiveBoundaryResult::Cancelled);
    TestTrue(TEXT("disposing live request does not release legacy cleanup"), Driver->IsOwnedBoundaryServiceSuspended());
    TestEqual(TEXT("superseded live callback never executes"), CallbackCalls, 1u);
    Driver->ResumeOwnedBoundaryService(); Driver->ResumeOwnedSimulation();

    // Cancel before ACK: no callback and no borrowed acknowledgment or pause.
    const uint32 ResetsBeforeCancel = PacingResets.load();
    BlockEntered->Reset(); BlockRelease->Reset(); BlockNext.store(true);
    if (!TestTrue(TEXT("real owner is in flight before cancellation"), BlockEntered->Wait(1000))) return false;
    auto Cancelled = Driver->RequestLiveBoundary(Failure);
    if (!TestTrue(TEXT("pending cancellable request exists"), Cancelled.IsValid())) return false;
    TestTrue(TEXT("pending poll has no old ACK to borrow"), Driver->PollLiveBoundary(*Cancelled, AtBoundary, Failure) == ELiveBoundaryResult::Waiting);
    TestTrue(TEXT("cancellation waits for real disposition"), Driver->CancelLiveBoundary(*Cancelled, Failure) == ELiveBoundaryResult::Waiting);
    TestFalse(TEXT("cancellation never changes user pause"), Driver->IsOwnedSimulationPaused());
    BlockRelease->Trigger();
    if (!TestTrue(TEXT("cancelled request eventually disposes"), WaitFor([&]()
        { Result = Driver->PollLiveBoundary(*Cancelled, AtBoundary, Failure); return Result != ELiveBoundaryResult::Waiting; }))) return false;
    TestTrue(TEXT("cancelled request is explicitly cancelled"), Result == ELiveBoundaryResult::Cancelled);
    TestEqual(TEXT("pre-ACK cancellation never invokes callback"), CallbackCalls, 1u);
    TestEqual(TEXT("pre-ACK cancellation does not pretend it parked"), PacingResets.load(), ResetsBeforeCancel);

    // New user pause while waiting survives structural cancellation/disposal.
    BlockEntered->Reset(); BlockRelease->Reset(); BlockNext.store(true);
    if (!TestTrue(TEXT("owner blocks before user pause"), BlockEntered->Wait(1000))) return false;
    auto UserPaused = Driver->RequestLiveBoundary(Failure);
    if (!TestTrue(TEXT("request waiting before pause intention"), UserPaused.IsValid())) return false;
    TestTrue(TEXT("user pause remains a real pending pause"), Driver->TryPauseOwnedSimulation(0) == ESimulationQuiescence::TimedOut);
    BlockRelease->Trigger();
    if (!TestTrue(TEXT("structural park acknowledges user pause"), Driver->SimulationWorker->TryPause(1000))) return false;
    if (!TestTrue(TEXT("user-paused callback disposes"), WaitFor([&]()
        { Result = Driver->PollLiveBoundary(*UserPaused, [](const FLiveBoundaryLease&, FString&) { return false; }, Failure); return Result != ELiveBoundaryResult::Waiting; }))) return false;
    TestTrue(TEXT("callback refusal is explicit after disposition"), Result == ELiveBoundaryResult::Rejected);
    TestTrue(TEXT("new pause intention survives callback refusal"), Driver->IsOwnedSimulationPaused() && Driver->SimulationWorker->IsPaused());
    Driver->ResumeOwnedSimulation();
    TestFalse(TEXT("only explicit user resume clears intention"), Driver->IsOwnedSimulationPaused());

    // A failed callback cannot retain its subordinate borrow and freeze play.
    auto LeakedBorrow = Driver->RequestLiveBoundary(Failure);
    if (!TestTrue(TEXT("borrow-refusal request exists"), LeakedBorrow.IsValid())) return false;
    if (!TestTrue(TEXT("leaked callback borrow reaches disposal"), WaitFor([&]()
        {
            Result = Driver->PollLiveBoundary(*LeakedBorrow, [&](const FLiveBoundaryLease&, FString&)
            {
                TestTrue(TEXT("callback borrows the existing ACK"), Driver->TrySuspendOwnedBoundaryService(0) == ESimulationQuiescence::BoundaryAcknowledged);
                return false;
            }, Failure);
            return Result != ELiveBoundaryResult::Waiting;
        }))) return false;
    TestTrue(TEXT("expired borrow is an explicit refusal"), Result == ELiveBoundaryResult::Rejected);
    TestEqual(TEXT("only subordinate borrow depth expires"), Driver->OwnedBoundarySuspendDepth, 0u);
    TestFalse(TEXT("refused callback leaves no pause intention"), Driver->IsOwnedSimulationPaused());
    TestFalse(TEXT("refused callback leaves no legacy worker request"), Driver->SimulationWorker->IsBoundaryServiceSuspendRequested());

    // Dropped handles still cancel under retained driver/worker ownership.
    auto Abandoned = Driver->RequestLiveBoundary(Failure);
    if (!TestTrue(TEXT("abandonment request exists"), Abandoned.IsValid())) return false;
    Abandoned.Reset();
    if (!TestTrue(TEXT("abandoned request drains without caller polling"), WaitFor([&]()
        { auto Next = Driver->RequestLiveBoundary(Failure); if (!Next) return false; Request = MoveTemp(Next); return true; }))) return false;
    const uint32 BeforeRetire = CallbackCalls;
    if (!TestTrue(TEXT("controlled generation replacement joins old owner"), Driver->PrepareControlledInputRun())) return false;
    TestTrue(TEXT("generation actually changed through original API"), Driver->GetInputWorkerGeneration() != Generation);
    TestTrue(TEXT("retired old request can never report Completed"), Driver->PollLiveBoundary(*Request, AtBoundary, Failure) == ELiveBoundaryResult::Retired);
    TestEqual(TEXT("retirement never invokes old callback"), CallbackCalls, BeforeRetire);
    TestFalse(TEXT("retired/stopped driver refuses another live request"), bool(Driver->RequestLiveBoundary(Failure)));
    Driver->ActiveExecutionModeValue.Store(static_cast<uint8>(ESimulationExecutionMode::UnrealAsyncCallback));
    TestFalse(TEXT("legacy execution never fabricates a live lease"), bool(Driver->RequestLiveBoundary(Failure)));
    TestFalse(TEXT("canonical worker did not fail"), Failed.load());
    return !HasAnyErrors();
}
#endif
