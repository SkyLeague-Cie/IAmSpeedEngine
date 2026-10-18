#include "SafeNetworkPhysicsComponent.h"

#include "Physics/PhysScene.h"
#include "SafeNetworkPhysicsRPCObservation.h"
#include "GameFramework/Actor.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY(SafeNetworkPhysicsLog);

bool USafeNetworkPhysicsComponent::ShouldBypassNetworkPhysicsLifecycle() const
{
	if (IsTemplate() || HasAnyFlags(RF_ClassDefaultObject))
	{
		return true;
	}

	const UWorld* World = GetWorld();
	if (!World || !World->IsGameWorld())
	{
		return true;
	}

	const FPhysScene* PhysScene = World->GetPhysicsScene();
	if (!PhysScene || !PhysScene->GetSolver())
	{
		return true;
	}

	return false;
}

void USafeNetworkPhysicsComponent::LogBypass(const TCHAR* Stage) const
{
	const UWorld* World = GetWorld();
	const FString WorldName = World ? World->GetName() : TEXT("<null>");
	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("<null>");

	UE_LOG(
		SafeNetworkPhysicsLog,
		Warning,
		TEXT("[SafeNetworkPhysicsComponent] Bypassing %s on %s (Owner=%s, World=%s, IsTemplate=%d)"),
		Stage,
		*GetName(),
		*OwnerName,
		*WorldName,
		IsTemplate() || HasAnyFlags(RF_ClassDefaultObject));
}

void USafeNetworkPhysicsComponent::LogLifecycle(const TCHAR* Stage) const
{
	const UWorld* World = GetWorld();
	const FString WorldName = World ? World->GetName() : TEXT("<null>");
	const FString OwnerName = GetOwner() ? GetOwner()->GetName() : TEXT("<null>");

#if !(UE_BUILD_SHIPPING)
	UE_LOG(
		SafeNetworkPhysicsLog,
		VeryVerbose,
		TEXT("[SafeNetworkPhysicsComponent] %s on %s (Owner=%s, World=%s, IsRegistered=%d)"),
		Stage,
		*GetName(),
		*OwnerName,
		*WorldName,
		IsRegistered());
#endif
}

void USafeNetworkPhysicsComponent::PrepareAsyncShutdown(const TCHAR* Stage)
{
	if (FAsyncNetworkPhysicsComponent* AsyncNetworkPhysicsComponent = GetNetworkPhysicsComponent_Internal())
	{
		if (FAsyncNetworkPhysicsComponentInput* AsyncInput = AsyncNetworkPhysicsComponent->GetProducerInputData_External())
		{
			AsyncInput->bIsLocallyControlled = false;
			AsyncInput->NetMode = ENetMode::NM_Standalone;
			AsyncInput->NetRole = ENetRole::ROLE_None;
			AsyncInput->ActorComponent = nullptr;
			AsyncInput->ImplementationInterface_Internal = nullptr;
			AsyncInput->PhysicsObject = nullptr;
			AsyncInput->bCompareStateToTriggerRewind = false;
			AsyncInput->bCompareStateToTriggerRewindIncludeSimProxies = false;
			AsyncInput->bCompareInputToTriggerRewind = false;
			AsyncInput->bUnregisterDataHistoryFromRewindData = true;
#if !(UE_BUILD_SHIPPING)
			UE_LOG(
				SafeNetworkPhysicsLog,
				VeryVerbose,
				TEXT("[SafeNetworkPhysicsComponent] Prepared async shutdown during %s on %s"),
				Stage,
				*GetName());
#endif
		}
	}
}

void USafeNetworkPhysicsComponent::InitializeComponent()
{
	if (ShouldBypassNetworkPhysicsLifecycle())
	{
		LogBypass(TEXT("InitializeComponent"));
		UActorComponent::InitializeComponent();
		return;
	}

	LogLifecycle(TEXT("InitializeComponent"));
	Super::InitializeComponent();
	bInitializedNetworkPhysicsLifecycle = true;
}

void USafeNetworkPhysicsComponent::UninitializeComponent()
{
	if (!bInitializedNetworkPhysicsLifecycle)
	{
		LogBypass(TEXT("UninitializeComponent"));
		UActorComponent::UninitializeComponent();
		return;
	}

	PrepareAsyncShutdown(TEXT("UninitializeComponent"));
	LogLifecycle(TEXT("UninitializeComponent"));
	Super::UninitializeComponent();
	bInitializedNetworkPhysicsLifecycle = false;
}

void USafeNetworkPhysicsComponent::BeginDestroy()
{
	PrepareAsyncShutdown(TEXT("BeginDestroy"));
	Super::BeginDestroy();
}


bool USafeNetworkPhysicsComponent::CallRemoteFunction(UFunction* Function, void* Parameters, FOutParmRec* OutParms, FFrame* Stack)
{
#if !(UE_BUILD_SHIPPING)
    static const bool bPrivateTrace = FParse::Param(FCommandLine::Get(), TEXT("SLBumpPrivateRPCEnvelopeTrace"));
    if (bPrivateTrace && IsInGameThread() && GetWorld() && GetWorld()->IsGameWorld() &&
        Function && Function->GetFName() == FName(TEXT("ServerReceiveInputData")))
    {
        const auto Seen = ObserveSpeedInputRPC(this, Function, Parameters);
        ++PrivateRPCTraceCallSerial;
        // One game thread owns these diagnostics. Fixed process/component caps;
        // record the first calls, count transitions, and sparse count16 repeats.
        static uint32 ProcessTraceLines = 0;
        const bool bRecord = PrivateRPCTraceCallSerial <= 8 || Seen.HistoryCount != PrivateRPCTraceLastCount ||
            (Seen.HistoryCount >= 16 && PrivateRPCTraceCallSerial % 64 == 0);
        if (bRecord && PrivateRPCTraceLines < 128 && ProcessTraceLines < 256)
        {
            ++PrivateRPCTraceLines;
            ++ProcessTraceLines;
            UE_LOG(SafeNetworkPhysicsLog, Display,
                TEXT("[BumpPrivateActualRPCEnvelope] stage=before_Super_CallRemoteFunction seq=%u component=%s outer_class=%s actor=%s world=%s parameter_matched=%d owner_matched=%d history_present=%d history_count=%d has_data=%d earliest_frame=%d latest_frame=%d delta_issue=%d scope=ACTUAL_REFLECTED_PROXY_ONLY_NOT_SERIALIZED_PACKET_OR_DELIVERY"),
                PrivateRPCTraceCallSerial, *GetPathName(), *GetPathNameSafe(GetOuter() ? GetOuter()->GetClass() : nullptr),
                *GetPathNameSafe(GetOwner()), *GetPathNameSafe(GetWorld()), int32(Seen.bParameterMatched), int32(Seen.bOwnerMatched),
                int32(Seen.bHistoryPresent), Seen.HistoryCount, int32(Seen.bHistoryHasData), Seen.EarliestFrame, Seen.LatestFrame,
                int32(Seen.bDeltaSerializationIssue));
        }
        PrivateRPCTraceLastCount = Seen.HistoryCount;
    }
#endif
    // Preserve the real producer, parameter object, routing and authority.
    // No synthetic serialization, history resize or packet-validation bypass.
    return Super::CallRemoteFunction(Function, Parameters, OutParms, Stack);
}
