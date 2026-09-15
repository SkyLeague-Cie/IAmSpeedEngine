#include "SpeedArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/SphereComponent.h"
#include "IAmSpeed/Actors/SpeedCar.h"
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/World/Simulation/SpeedGameMode.h"
#include "IAmSpeed/World/Simulation/SpeedSimulation.h"

USpeedArmComponent::USpeedArmComponent(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	TargetArmLength = 350.0f;
	TargetOffset.Z = 150.0f;
	ProbeSize = 25.0f;
	bDoCollisionTest = true;
	bInheritPitch = bInheritRoll = bInheritYaw = false;
	bEnableCameraRotationLag = true;
	CameraRotationLagSpeed = 37.0f;
	bEnableCameraLag = false;
	CameraLagMaxDistance = 1000.0f;
	CameraLagMaxTimeStep = 0.005f;
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(this, FName("SpringEndpoint"));
	Camera->SetRelativeRotation(FRotator(-2.0, 0.0, 0.0));
	Camera->SetFieldOfView(110.0f);
	Camera->SetAspectRatioAxisConstraint(EAspectRatioAxisConstraint::AspectRatio_MaintainXFOV);
	Camera->bOverrideAspectRatioAxisConstraint = true;
	Camera->PostProcessSettings.bOverride_MotionBlurAmount = true;
	Camera->PostProcessSettings.MotionBlurAmount = 0.0;
	Camera->PostProcessSettings.bOverride_MotionBlurTargetFPS = true;
	Camera->PostProcessSettings.MotionBlurTargetFPS = 0.0;
	HitSphere = CreateDefaultSubobject<USphereComponent>(TEXT("Sphere"));
	HitSphere->SetupAttachment(Camera);
	HitSphere->SetSphereRadius(1.0);
	HitSphere->BodyInstance.bUseCCD = true;
}

void USpeedArmComponent::InitializeForCar(ASpeedCar& Owner, USpeedWheeledComponent& Movement)
{
	check(IsInGameThread());
	if (bGenericSnapshotInitialized) return;
	InitialConfiguration.Arm.ArmLocalPosition = GetRelativeLocation();
	InitialConfiguration.Arm.LocalCameraPosition = Camera->GetRelativeLocation();
	InitialConfiguration.Arm.LocalCameraRotation = Camera->GetRelativeRotation();
	InitialConfiguration.Arm.bRotationLag = bEnableCameraRotationLag;
	InitialConfiguration.FieldOfView = Camera->FieldOfView;
	InitialConfiguration.SocketOffset = SocketOffset;
	InitialConfiguration.ProbeSize = ProbeSize;
	InitialConfiguration.RotationLagSpeed = CameraRotationLagSpeed;
	InitialConfiguration.LagSpeedCoeff = LagSpeedCoeff;
	InitialState.BaseDistance = TargetArmLength;
	InitialState.ConfiguredCameraHeight = TargetOffset.Z;
	InitialState.Stiffness = .55f;
	InitialState.SwivelSpeed = 2.9f;
	InitialState.TransitionSpeed = 1.3f;
	InitialState.DesiredRegularArmLength = TargetArmLength;
	InitialState.TargetArmLength = TargetArmLength;
	InitialState.TargetOffset = TargetOffset;
	InitialState.SocketOffset = SocketOffset;
	InitialState.CameraRotationLagSpeed = CameraRotationLagSpeed;
	InitialState.ProbeSize = ProbeSize;
	InitialState.LagSpeedCoeff = LagSpeedCoeff;
	InitialState.CachedAirCameraHorizontalTarget = FVector::ForwardVector;
	InitialState.CachedGroundCameraHorizontalForward = FVector::ForwardVector;
	InitialState.CarTarget = FVector::ForwardVector;
	InitialState.PhysRotator = Camera->GetRelativeRotation();
	bGenericSettingsUnsupported = bEnableCameraLag || Camera->bUsePawnControlRotation ||
		Camera->ProjectionMode != ECameraProjectionMode::Perspective;
	bGenericSnapshotInitialized = true;
	GenericMovement = &Movement;
	Movement.EnableGenericCameraInput(true);
	SetComponentTickEnabled(false);
	const FTransform InitialWorld = Camera->GetComponentTransform();
	Camera->SetAbsolute(true, true, true);
	Camera->SetWorldTransform(InitialWorld);
	Camera->bUsePawnControlRotation = false;
	Camera->bLockToHmd = false;
	Camera->ClearAdditiveOffset();
	BindGenericAtWorldBeginPlay();
}

void USpeedArmComponent::BindGenericAtWorldBeginPlay()
{
	check(IsInGameThread());
	UWorld* World = GetWorld();
	if (!World || !GenericMovement.IsValid() || BoundEndpoint) return;
	if (!World->HasBegunPlay())
	{
		if (!WorldBeginPlayHandle.IsValid())
			WorldBeginPlayHandle = World->OnWorldBeginPlay.AddUObject(this, &USpeedArmComponent::BindGenericAtWorldBeginPlay);
		return;
	}
	World->OnWorldBeginPlay.Remove(WorldBeginPlayHandle);
	WorldBeginPlayHandle.Reset();
	const auto* Mode = Cast<ASpeedGameMode>(World->GetAuthGameMode());
	auto* Simulation = Mode ? Mode->GetSpeedSimulation() : nullptr;
	if (!Simulation || !BindEndpointAtLifecycle(*Simulation, *GenericMovement.Get(), nullptr))
		UE_LOG(LogTemp, Warning, TEXT("[GenericCameraLifecycleUnavailable] No owner binding; no live fallback."));
}

USpeedArmComponent::FEndpointPtr USpeedArmComponent::CreateCameraEndpoint(const FSimulationPresentationBinding& Binding)
{
	if (bGenericSettingsUnsupported) return nullptr;
	return MakeShared<FSpeedCarCameraPresentation, ESPMode::ThreadSafe>(
		Binding.OwnerStableId, Binding.FirstFrame, 1, InitialConfiguration, InitialState);
}

USpeedArmComponent::FEndpointPtr USpeedArmComponent::BindEndpointAtLifecycle(
	ASpeedSimulation& Simulation, ISpeedComponent& Owner, ISpeedComponent* Target)
{
	check(IsInGameThread());
	if (BoundEndpoint && BoundSimulation.Get() != &Simulation) return nullptr;
	auto Bound = Simulation.BindPresentationAtFrameBoundary(Owner, Target,
		[this](const FSimulationPresentationBinding& Binding) { return CreateCameraEndpoint(Binding); }, BoundEndpoint);
	if (!Bound) return nullptr;
	BoundEndpoint = Bound;
	BoundSimulation = &Simulation;
	GenericSnapshot.Reset();
	LastPublicationSerial = 0;
	return Bound;
}

bool USpeedArmComponent::ReadGenericOwnerSnapshot(FSimulationPoseConsumption& Out) const
{
	check(IsInGameThread());
	return bGenericSnapshotInitialized && BoundEndpoint && BoundSimulation.IsValid() &&
		BoundSimulation->ReadPresentationPose(BoundEndpoint->OwnerStableId(), Out);
}

ASpeedSimulation* USpeedArmComponent::GetBoundSimulation() const
{
	return BoundSimulation.Get();
}

bool USpeedArmComponent::ApplyGenericCameraSnapshot(const FSimulationPoseConsumption& Owner)
{
	check(IsInGameThread());
	if (!bGenericSnapshotInitialized || !BoundEndpoint || !BoundSimulation.IsValid() || !Camera ||
		!Camera->IsUsingAbsoluteLocation() || !Camera->IsUsingAbsoluteRotation() || !Camera->IsUsingAbsoluteScale()) return false;
	FSimulationPresentationOutput Output;
	if (!BoundSimulation->ReadPresentationOutput(BoundEndpoint->OwnerStableId(), BoundEndpoint->Channel(), Output)) return false;
	return UpdateCameraFromSnapshot(Output, Owner);
}

bool USpeedArmComponent::UpdateCameraFromSnapshot(const FSimulationPresentationOutput& Output,
	const FSimulationPoseConsumption& Owner)
{
	check(IsInGameThread());
	if (!bGenericSnapshotInitialized || !BoundEndpoint || !Camera || bGenericSettingsUnsupported ||
		!Camera->IsUsingAbsoluteLocation() || !Camera->IsUsingAbsoluteRotation() || !Camera->IsUsingAbsoluteScale() ||
		Camera->bUsePawnControlRotation || Camera->bLockToHmd) return false;
	const auto Endpoint = StaticCastSharedPtr<FSpeedCarCameraPresentation>(BoundEndpoint);
	FSpeedCarCameraPose Pose;
	FTransform World;
	if (!Endpoint->Accepts(Output) || !FSpeedCarCameraPose::Read(Output.Payload, Pose) || !Pose.Compose(Output, Owner, World)) return false;
	if (GenericSnapshot && (Pose.Epoch < GenericSnapshot->Epoch || Output.PublicationSerial < LastPublicationSerial ||
		(Pose.Epoch == GenericSnapshot->Epoch && (Pose.Frame < GenericSnapshot->Frame ||
		((Pose.Frame == GenericSnapshot->Frame) != (Output.PublicationSerial == LastPublicationSerial)))))) return false;
	if (GenericSnapshot && !Camera->GetComponentTransform().Equals(ExpectedWorld, 0.0)) return false;
	FRotationConversionCache PhysicalRotationCache;
	const FRotator PhysicalRotation = PhysicalRotationCache.NormalizedQuatToRotator(World.GetRotation());
	FRotationConversionCache DisplacedRotationCache;
	DisplacedRotationCache.RotatorToQuat(PhysicalRotation + FRotator(0, 180, 0));
	// SetRelativeRotationCache compares only rotators. Displace it first so an
	// equivalent rotator with different quaternion bits cannot retain stale data.
	Camera->SetRelativeRotationCache(DisplacedRotationCache);
	Camera->SetRelativeRotationCache(PhysicalRotationCache);
	Camera->SetRelativeLocation_Direct(World.GetLocation());
	Camera->SetRelativeRotation_Direct(PhysicalRotation);
	Camera->SetRelativeScale3D_Direct(World.GetScale3D());
	Camera->UpdateComponentToWorld(EUpdateTransformFlags::SkipPhysicsUpdate, ETeleportType::TeleportPhysics);
	Camera->SetFieldOfView(Pose.FieldOfView);
	ExpectedWorld = World;
	LastPublicationSerial = Output.PublicationSerial;
	GenericSnapshot = MakeShared<const FSpeedCarCameraPose, ESPMode::ThreadSafe>(Pose);
	return Camera->GetComponentTransform().Equals(World, 0.0);
}

void USpeedArmComponent::UpdateDesiredArmLocation(bool bTrace, bool bLocationLag, bool bRotationLag, float Delta)
{
	if (bGenericSnapshotInitialized) return;
	Super::UpdateDesiredArmLocation(bTrace, bLocationLag, bRotationLag, Delta);
}

void USpeedArmComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	if (UWorld* World = GetWorld()) World->OnWorldBeginPlay.Remove(WorldBeginPlayHandle);
	WorldBeginPlayHandle.Reset();
	if (BoundEndpoint && BoundSimulation.IsValid()) BoundSimulation->UnregisterPresentationProducer(BoundEndpoint.ToSharedRef());
	BoundEndpoint.Reset();
	BoundSimulation.Reset();
	GenericSnapshot.Reset();
	if (GenericMovement.IsValid()) GenericMovement->EnableGenericCameraInput(false);
	GenericMovement.Reset();
	Super::EndPlay(EndPlayReason);
}
