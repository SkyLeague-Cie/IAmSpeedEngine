#include "SafeNetworkPhysicsComponent.h"

#include "Physics/PhysScene.h"

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

	UE_LOG(
		SafeNetworkPhysicsLog,
		Log,
		TEXT("[SafeNetworkPhysicsComponent] %s on %s (Owner=%s, World=%s, IsRegistered=%d)"),
		Stage,
		*GetName(),
		*OwnerName,
		*WorldName,
		IsRegistered());
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

			UE_LOG(
				SafeNetworkPhysicsLog,
				Log,
				TEXT("[SafeNetworkPhysicsComponent] Prepared async shutdown during %s on %s"),
				Stage,
				*GetName());
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
