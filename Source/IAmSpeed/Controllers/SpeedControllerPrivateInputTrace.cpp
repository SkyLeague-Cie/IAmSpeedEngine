#include "SpeedController.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

bool ASpeedController::ObservePrivateInputAdmission(bool Result, const TCHAR* Site)
{
#if !UE_BUILD_SHIPPING
    if (!Result) TracePrivateInputState(Site);
#endif
    return Result;
}

void ASpeedController::MarkInputLifecycleFault(const TCHAR* Site,
    const Speed::Input::V2::FBoundaryReceipt* Receipt)
{
    const bool WasFaulted = bInputLifecycleFault;
    bInputLifecycleFault = true;
#if !UE_BUILD_SHIPPING
    if (!WasFaulted) TracePrivateInputState(Site, Receipt, true);
#endif
}

void ASpeedController::TracePrivateInputState(const TCHAR* Site,
    const Speed::Input::V2::FBoundaryReceipt* CheckedReceipt, bool FirstFault)
{
#if !UE_BUILD_SHIPPING
    // GT-owned bounded diagnostic sidecar only. No polling, writer, authority,
    // retry, ACK, canonical state/hash, or receipt mutation is added here.
    static const bool Enabled = FParse::Param(FCommandLine::Get(), TEXT("SLBumpPrivateLocalInputTrace"));
    static uint32 ProcessLines = 0;
    if (!Enabled || !IsInGameThread() || HasAnyFlags(RF_ClassDefaultObject) || !GetWorld()
        || !GetWorld()->IsGameWorld() || PrivateInputTraceLines >= 32 || ProcessLines >= 128) return;
    const auto Session = InputSessionV2;
    const auto* Mode = GetWorld()->GetAuthGameMode<ASpeedGameMode>();
    const auto* Driver = Mode ? Mode->GetSpeedSimulation() : nullptr;
    const auto View = Driver ? Driver->ReadInputRegistryView() : nullptr;
    const auto ObservedReceipt = !CheckedReceipt && Driver && Session && Session->PendingCommand
        ? Driver->ReadInputSessionReceipt(Session->PendingCommand)
        : std::optional<Speed::Input::V2::FBoundaryReceipt>{};
    const auto* Receipt = CheckedReceipt ? CheckedReceipt : (ObservedReceipt ? &*ObservedReceipt : nullptr);
    if (FCString::Strcmp(Site, TEXT("Registry.ReceiptBeforeAck")) == 0)
    {
        const uint64 Command = Session ? Session->PendingCommand : 0;
        const int32 Status = Receipt ? int32(Receipt->Status) : -1;
        if (PrivateInputTraceReceiptCommand == Command && PrivateInputTraceReceiptStatus == Status) return;
        PrivateInputTraceReceiptCommand = Command; PrivateInputTraceReceiptStatus = Status;
    }
    ++PrivateInputTraceLines; ++ProcessLines;
    const auto* Descriptor = Session && Session->Descriptor ? &*Session->Descriptor : nullptr;
    const auto* Component = IsValid(SpeedCar) ? Cast<USpeedWheeledComponent>(SpeedCar->GetVehicleMovement()) : nullptr;
    int32 OwnerPhase = -1;
    if (View && Descriptor)
        for (const auto& Phase : View->Phases)
            if (Phase.Session == Descriptor->Id && Phase.Epoch == Descriptor->Epoch) OwnerPhase = int32(Phase.Phase);
    UE_LOG(LogTemp, Display, TEXT("[BumpPrivateLocalInput] site=%s first_fault=%d first_cause_captured=%d controller=%u pawn=%s stable_actor=%llu fault=%d required=%d pending_factory=%d session_present=%d descriptor=%d session=%llu owner=%llu epoch=%llu first_frame=%llu bound=%d registry_paused=%d closed=%d command=%llu op=%d submitted_worker=%llu current_worker=%llu submitted_version=%llu receipt_checked=%d receipt_present=%d receipt_id=%llu receipt_worker=%llu receipt_status=%d receipt_reason=%d receipt_version=%llu view_present=%d view_version=%llu view_terminal=%d owner_phase=%d driver_present=%d paused=%d"),
        Site, int32(FirstFault), int32(FirstFault), GetUniqueID(), *GetNameSafe(SpeedCar),
        Component ? Component->GetPublishedSimulationStableId() : 0,
        int32(bInputLifecycleFault), int32(bInputSessionRequiredV2), int32(bInputSessionPendingV2), int32(bool(Session)), int32(Descriptor != nullptr),
        Session ? Session->Session : 0, Descriptor ? Descriptor->Id : 0, Descriptor ? Descriptor->Epoch : 0,
        Descriptor ? Descriptor->First : 0, int32(Session && Session->RegistryBound), int32(Session && Session->RegistryPaused), int32(Session && Session->Closed),
        Session ? Session->PendingCommand : 0, Session ? int32(Session->PendingOperation) : -1,
        Session ? Session->WorkerGeneration : 0, Driver ? Driver->GetInputWorkerGeneration() : 0,
        Session ? Session->RegistryVersion : 0, int32(CheckedReceipt != nullptr), int32(Receipt != nullptr),
        Receipt ? Receipt->Id : 0, Receipt ? Receipt->WorkerGeneration : 0, Receipt ? int32(Receipt->Status) : -1,
        Receipt ? int32(Receipt->RejectionReason) : -1, Receipt ? Receipt->RegistryVersion : 0,
        int32(bool(View)), View ? View->Version : 0, int32(View && View->Terminal), OwnerPhase,
        int32(Driver != nullptr), int32(IsPaused()));
#endif
}
