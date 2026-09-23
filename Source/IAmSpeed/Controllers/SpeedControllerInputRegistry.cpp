#include "SpeedController.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"

namespace
{
ASpeedSimulation* InputDriver(UWorld* World)
{
    const auto* Mode = World ? World->GetAuthGameMode<ASpeedGameMode>() : nullptr;
    return Mode ? Mode->GetSpeedSimulation() : nullptr;
}
}

bool ASpeedController::BeginRegistryInputSession()
{
    check(IsInGameThread());
    const auto Session = InputSessionV2;
    auto* Driver = InputDriver(GetWorld());
    auto* Component = IsValid(SpeedCar) ? Cast<USpeedWheeledComponent>(SpeedCar->GetVehicleMovement()) : nullptr;
    auto* World = GetWorld() ? GetWorld()->GetSubsystem<USpeedWorldSubsystem>() : nullptr;
    if (!Session || !Session->Descriptor || !Driver || !Component || !World || Session->PendingCommand) return false;
    const auto Boundary = Driver->TryPauseOwnedSimulation();
    if (Boundary != ESimulationQuiescence::BoundaryAcknowledged && Boundary != ESimulationQuiescence::AlreadyStopped) return false;
    const uint64 Id = World->GetSimulationStableId(*Component);
    if (!Id || !SpeedCar->SetFrameInputStreamV2(nullptr)) return false;
    Session->Descriptor->Actors = {{Id, Id}};
    Session->Descriptor->Id = uint64(GetUniqueID()) + 1;
    Session->Descriptor->Controller = Session->Descriptor->Id;
    Session->WorkerGeneration = Driver->GetInputWorkerGeneration();
    return QueueRegistryInputCommand(Speed::Input::V2::EBoundaryOperation::Bind);
}

bool ASpeedController::QueueRegistryInputCommand(Speed::Input::V2::EBoundaryOperation Operation)
{
    using namespace Speed::Input::V2;
    check(IsInGameThread());
    auto* Driver = InputDriver(GetWorld());
    const auto Session = InputSessionV2;
    if (!Driver || !Session || !Session->Descriptor) return false;
    FBoundaryCommandDescriptor Command;
    Command.Id = Driver->AllocateInputCommandId(); Command.WorkerGeneration = Driver->GetInputWorkerGeneration();
    const auto View = Driver->ReadInputRegistryView();
    Command.RegistryVersion = View ? FMath::Max(View->Version, Session->RegistryVersion) : Session->RegistryVersion;
    Command.Operation = Operation; Command.Session = Session->Descriptor->Id; Command.Epoch = Session->Descriptor->Epoch;
    if (Operation == EBoundaryOperation::Bind) Command.Binding = *Session->Descriptor;
    if (Operation == EBoundaryOperation::Resume)
    {
        if (Session->ResumeGeneration == MAX_uint64) return false;
        Command.ResumeGeneration = ++Session->ResumeGeneration;
    }
    const auto Result = Driver->SubmitInputSessionCommand(Command,
        Operation == EBoundaryOperation::Bind ? Session->Journal : nullptr,
        Operation == EBoundaryOperation::Bind ? Session->Observation : nullptr);
    if (Result != ECommandAdmission::Enqueued) return false;
    if (Session->PendingCommand) Session->SupersededCommands.push_back(Session->PendingCommand);
    Session->PendingCommand = Command.Id; Session->PendingOperation = Operation;
    Session->ResumePending = Operation == EBoundaryOperation::Resume;
    return true;
}

bool ASpeedController::ServiceRegistryInputSession()
{
    using namespace Speed::Input::V2;
    check(IsInGameThread());
    auto* Driver = InputDriver(GetWorld());
    const auto Session = InputSessionV2;
    if (!Driver || !Session || !Session->Descriptor || Session->Closed || bInputLifecycleFault) return false;
    for (auto I=Session->SupersededCommands.begin(); I!=Session->SupersededCommands.end();)
    {
        if (Driver->AcknowledgeInputSessionReceipt(*I)) I=Session->SupersededCommands.erase(I);
        else ++I;
    }
    if (Session->PendingCommand)
    {
        const auto Receipt = Driver->ReadInputSessionReceipt(Session->PendingCommand);
        if (!Receipt) { bInputLifecycleFault = true; return false; }
        if (Receipt->Status == EBoundaryStatus::Pending || Receipt->Status == EBoundaryStatus::WaitingForBaseline)
        {
            // Pause supersedes a resume waiting on an unplugged device; it
            // never waits for that device to return merely to neutralize.
            if (IsPaused() && Session->PendingOperation == EBoundaryOperation::Resume)
            {
                if (!QueueRegistryInputCommand(EBoundaryOperation::PauseAll)) { bInputLifecycleFault = true; return false; }
            }
            return true;
        }
        if (Receipt->Status != EBoundaryStatus::Applied) { bInputLifecycleFault = true; return false; }
        Session->RegistryVersion = Receipt->RegistryVersion;
        if (Session->PendingOperation == EBoundaryOperation::Bind)
        {
            Session->RegistryBound = true;
            Session->RegistryPaused = Session->Descriptor->Kind == EProducerContract::Device;
        }
        else if (Session->PendingOperation == EBoundaryOperation::PauseAll) Session->RegistryPaused = true;
        else if (Session->PendingOperation == EBoundaryOperation::Resume) Session->RegistryPaused = false;
        Driver->AcknowledgeInputSessionReceipt(Session->PendingCommand);
        Session->PendingCommand = 0; Session->ResumePending = false;
    }
    if (!Session->RegistryBound) return true;
    if (IsPaused() && !Session->RegistryPaused)
    {
        if (!QueueRegistryInputCommand(EBoundaryOperation::PauseAll)) { bInputLifecycleFault = true; return false; }
    }
    else if (!IsPaused() && Session->RegistryPaused)
    {
        if (!QueueRegistryInputCommand(EBoundaryOperation::Resume)) { bInputLifecycleFault = true; return false; }
    }
    else if (!IsPaused() && !Session->RegistryPaused) SetStandaloneSimulationPaused(false);
    return true;
}

bool ASpeedController::ReleaseRegistryInputSession()
{
    using namespace Speed::Input::V2;
    check(IsInGameThread());
    const auto Session = InputSessionV2;
    if (!Session || !Session->Descriptor) return false;
    Session->RequestStopObservation();
    if (Speed::Input::FPresentationInputScope::IsActive()) return false;
    auto* Driver = InputDriver(GetWorld());
    if (!Driver)
    {
        if (Session->RegistryBound || Session->PendingCommand || !Session->CloseAtBoundary()) return false;
        InputSessionV2.reset(); InputReceiversV2.clear(); return true;
    }
    if (Session->WorkerGeneration != Driver->GetInputWorkerGeneration()) return false;
    const auto RetireJoined = [&]()
    {
        if (!Driver->JoinOwnedSimulationForInputTeardown() || !Driver->InputOwnersRetiredAfterJoin(Session->WorkerGeneration)) return false;
#if !UE_BUILD_SHIPPING
        UE_LOG(LogTemp, Display, TEXT("[InputSessionTeardownAck] Controller=%u Session=%llu Epoch=%llu Route=JoinRetired WorkerGeneration=%llu"),
            GetUniqueID(), Session->Descriptor->Id, Session->Descriptor->Epoch, Session->WorkerGeneration);
#endif
        Session->PendingCommand = 0; Session->RegistryBound = false;
        if (!Session->CloseAtBoundary()) return false;
        InputSessionV2.reset(); InputReceiversV2.clear(); return true;
    };
    bool DetachQueued = Session->PendingCommand && Session->PendingOperation == EBoundaryOperation::Detach;
    if (Session->RegistryBound && !DetachQueued)
    {
        if (!QueueRegistryInputCommand(EBoundaryOperation::Detach)) return RetireJoined();
        DetachQueued = true;
    }
    const auto Boundary = Driver->TryPauseOwnedSimulation();
    if (Boundary == ESimulationQuiescence::AlreadyStopped) return RetireJoined();
    if (Boundary != ESimulationQuiescence::BoundaryAcknowledged) return false;
    const double Deadline = FPlatformTime::Seconds() + 1.0;
    if (Session->PendingCommand && Session->PendingOperation == EBoundaryOperation::Bind)
    {
        for (;;)
        {
            const auto Receipt = Driver->ReadInputSessionReceipt(Session->PendingCommand);
            if (!Receipt) return false;
            if (Receipt->Status != EBoundaryStatus::Pending)
            {
                if (Receipt->Status == EBoundaryStatus::Applied) Session->RegistryBound = true;
                else if (Receipt->Status != EBoundaryStatus::Rejected) return false;
                Driver->AcknowledgeInputSessionReceipt(Session->PendingCommand);
                Session->RegistryVersion = Receipt->RegistryVersion; Session->PendingCommand = 0; break;
            }
            if (FPlatformTime::Seconds() >= Deadline) return false;
            FPlatformProcess::SleepNoStats(.001f);
        }
    }
    if (Session->RegistryBound)
    {
        // Detach supersedes a pending fresh-resume command for this epoch.
        if (!DetachQueued && !QueueRegistryInputCommand(EBoundaryOperation::Detach)) return false;
        for (;;)
        {
            const auto Receipt = Driver->ReadInputSessionReceipt(Session->PendingCommand);
            if (!Receipt) return false;
            if (Receipt->Status != EBoundaryStatus::Pending && Receipt->Status != EBoundaryStatus::WaitingForBaseline)
            {
                if (Receipt->Status != EBoundaryStatus::Applied) return false;
#if !UE_BUILD_SHIPPING
                UE_LOG(LogTemp, Display, TEXT("[InputSessionTeardownAck] Controller=%u Session=%llu Epoch=%llu Route=DetachApplied Command=%llu RegistryVersion=%llu"),
                    GetUniqueID(), Session->Descriptor->Id, Session->Descriptor->Epoch,
                    Session->PendingCommand, Receipt->RegistryVersion);
#endif
                Driver->AcknowledgeInputSessionReceipt(Session->PendingCommand);
                Session->PendingCommand = 0; Session->RegistryBound = false; break;
            }
            if (FPlatformTime::Seconds() >= Deadline) return false;
            FPlatformProcess::SleepNoStats(.001f);
        }
    }
    if (!Session->CloseAtBoundary()) return false;
    for (const auto Id : Session->SupersededCommands) Driver->AcknowledgeInputSessionReceipt(Id);
    if (IsValid(SpeedCar) && !SpeedCar->SetFrameInputStreamV2(nullptr)) return false;
    InputSessionV2.reset(); InputReceiversV2.clear();
    return true;
}
