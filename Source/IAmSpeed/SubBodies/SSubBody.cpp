// Fill out your copyright notice in the Description page of Project Settings.


#include "SSubBody.h"
#include "Solid/CylinderSubBody.h"
#include "IAmSpeed/World/Simulation/JoinedActorKinematics1213.h"
#include "IAmSpeed/Base/ExactRotatorQuaternionCache.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "Configs/SubBodyConfig.h"
#include "GameFramework/Actor.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"

USSubBody::USSubBody(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
{
    PrimaryComponentTick.bCanEverTick = false;
    ResponseParamsBodyDomainStorage().CollisionResponse.SetAllChannels(ECollisionResponse::ECR_Ignore);
    ResponseParamsBodyDomainStorage().CollisionResponse.SetResponse(ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
	SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Ignore); // For UE
	SetCollisionObjectType(ECollisionChannel::ECC_WorldDynamic); // For UE
    SetSimulatePhysics(false); // For UE
    SetGenerateOverlapEvents(false); // For UE
    SetCollisionEnabled(ECollisionEnabled::NoCollision);
}

void USSubBody::OnRegister()
{
    // A nested native component can inherit a Blueprint-template parent
    // pointer. Resolve that exact default-subobject identity on this actor
    // before SceneComponent attempts attachment. Preserve every authored
    // relative transform, socket and dimension; never fit geometry to a mesh.
    if(!IsTemplate())
    {
        USceneComponent* Parent=GetAttachParent();
        AActor* Actor=GetOwner();
        if(Parent && Parent->IsTemplate() && Actor)
        {
            auto* Instance=Cast<USceneComponent>(Actor->GetDefaultSubobjectByName(Parent->GetFName()));
            if(Instance && !Instance->IsTemplate() && Instance->GetOwner()==Actor &&
                Instance->IsA(Parent->GetClass()))
                SetupAttachment(Instance,GetAttachSocketName());
        }
    }
    Super::OnRegister();
}

void USSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	ParentComponentBodyDomainStorage() = InParentComponent;
    if (ParentComponentBodyDomainStorage())
    {
		SubBodyConfig Config = ParentComponentBodyDomainStorage()->GetSubBodyConfig(*this);
        if (Config.bValid)
        {
            SetRelativeLocation(Config.LocalOffset);
            SetRelativeRotation(Config.LocalRotation);
            CollisionChannelBodyDomainStorage() = Config.CollisionChannel;
        }
        
        for (USSubBody* SubBody: ParentComponentBodyDomainStorage()->GetSubBodies())
        {
            if (SubBody)
            {
                AlwaysIgnoredComponentsBodyDomainStorage().AddUnique(SubBody);
            }
		}
    }
}

void USSubBody::ResetForFrame(const float& Delta)
{
    FutureHitStorage() = SHitResult();
    CurrentHitStorage() = SHitResult();
    ResetIgnoredComponents();
}

void USSubBody::ResetIgnoredComponents()
{
    // Reuse the ordinary frame's buffer instead of freeing/reallocating it at
    // 300 Hz. An exceptional contact burst must not retain unbounded slack:
    // keep at most 1 KiB or the permanent registry's existing allocation.
    const SIZE_T RetainedBytes = FMath::Max<SIZE_T>(1024, AlwaysIgnoredComponentsBodyDomainStorage().GetAllocatedSize());
    if (IgnoredComponentsBodyDomainStorage().GetAllocatedSize() > RetainedBytes)
        IgnoredComponentsBodyDomainStorage().Empty(AlwaysIgnoredComponentsBodyDomainStorage().Num());
    else
        IgnoredComponentsBodyDomainStorage().Reset(AlwaysIgnoredComponentsBodyDomainStorage().Num());
    IgnoredComponentsBodyDomainStorage().Append(AlwaysIgnoredComponentsBodyDomainStorage());
}

void USSubBody::AddExternalSubBodies(const TArray<USSubBody*>& SubBodies)
{
    for (USSubBody* SubBody : SubBodies)
    {
        if (SubBody && SubBody->GetParentComponent() != ParentComponentBodyDomainStorage() &&
            !AlwaysIgnoredComponentsBodyDomainStorage().Contains(SubBody) && !SubBody->IsSensor() &&
            !(ParentComponentBodyDomainStorage() && ParentComponentBodyDomainStorage()->IsPrescribedMotion() && SubBody->GetParentComponent() && SubBody->GetParentComponent()->IsPrescribedMotion()))
        {
            switch (SubBody->GetSubBodyType())
            {
            case ESubBodyType::Cylinder:
                if(!IgnoredSubBodyTypesBodyDomainStorage().Contains(ESubBodyType::Cylinder))ExternalCylinderSubBodiesBodyDomainStorage().AddUnique(Cast<UCylinderSubBody>(SubBody));
                break;
			case ESubBodyType::Hitbox:
				if (!IgnoredSubBodyTypesBodyDomainStorage().Contains(ESubBodyType::Hitbox))
				    ExternalBoxSubBodiesBodyDomainStorage().AddUnique(Cast<UBoxSubBody>(SubBody));
				break;
			case ESubBodyType::Sphere:
				if (!IgnoredSubBodyTypesBodyDomainStorage().Contains(ESubBodyType::Sphere))
				    ExternalSphereSubBodiesBodyDomainStorage().AddUnique(Cast<USphereSubBody>(SubBody));
				break;
			case ESubBodyType::Wheel:
				if (!IgnoredSubBodyTypesBodyDomainStorage().Contains(ESubBodyType::Wheel))
				    ExternalWheelSubBodiesBodyDomainStorage().AddUnique(Cast<USWheelSubBody>(SubBody));
				break;
            }
        }
    }
}

void USSubBody::RemoveExternalSubBodies(const TArray<USSubBody*>& SubBodies)
{
    for (USSubBody* SubBody : SubBodies)
    {
        if (SubBody)
        {
            switch (SubBody->GetSubBodyType())
            {
            case ESubBodyType::Cylinder:
                ExternalCylinderSubBodiesBodyDomainStorage().Remove(Cast<UCylinderSubBody>(SubBody));break;
            case ESubBodyType::Hitbox:
                ExternalBoxSubBodiesBodyDomainStorage().Remove(Cast<UBoxSubBody>(SubBody));
                break;
            case ESubBodyType::Sphere:
                ExternalSphereSubBodiesBodyDomainStorage().Remove(Cast<USphereSubBody>(SubBody));
                break;
            case ESubBodyType::Wheel:
                ExternalWheelSubBodiesBodyDomainStorage().Remove(Cast<USWheelSubBody>(SubBody));
                break;
            }
        }
    }
}

void USSubBody::UpdateKinematicsFromOwner(const SKinematic& ParentKinematic)
{
	// Reuse only pure local Euler conversion, independently on each calling lane.
	// Do not share USceneComponent's mutable rotation cache with the game thread.
	static thread_local Speed::FExactRotatorQuaternionCache LocalRotationCache;
	auto RelativeLocationVar = GetRelativeLocation();
	auto RelativeRotationVar = LocalRotationCache.Get(GetRelativeRotation());
 // Preserve this exact native cache access even when completed base arithmetic
 // can be reused. Failed value/FP validation leaves the original path intact.
 if(Speed::FrameParallel::TryConsumeJoinedActorBase1430(*this,ParentKinematic,
     RelativeLocationVar,RelativeRotationVar,KinematicsStorage()))return;
	const FVector WorldOffset = ParentKinematic.Rotation.RotateVector(RelativeLocationVar);
	KinematicsStorage().Location = ParentKinematic.Location + WorldOffset;
	KinematicsStorage().Velocity = ParentKinematic.Velocity + FVector::CrossProduct(ParentKinematic.AngularVelocity, WorldOffset);
	KinematicsStorage().Acceleration = ParentKinematic.Acceleration +
        FVector::CrossProduct(ParentKinematic.AngularAcceleration, WorldOffset) +
        FVector::CrossProduct(ParentKinematic.AngularVelocity, FVector::CrossProduct(ParentKinematic.AngularVelocity, WorldOffset));
    KinematicsStorage().Rotation = ParentKinematic.Rotation * RelativeRotationVar;
	KinematicsStorage().AngularVelocity = ParentKinematic.AngularVelocity;
	KinematicsStorage().AngularAcceleration = ParentKinematic.AngularAcceleration;
}

bool USSubBody::SweepTOI(const float& RemainingDelta, float& OutTOI)
{
    OutTOI = RemainingDelta;
    FutureHitStorage() = SHitResult();

    const FVector Start = KinematicsStorage().Location;
    const FVector End =
        Start +
        KinematicsStorage().Velocity * RemainingDelta +
        0.5f * KinematicsStorage().Acceleration * RemainingDelta * RemainingDelta;

    if (!InternalSweep(Start, End, FutureHitStorage(), RemainingDelta))
        return false;

    // Convert normalized Hit.Time (0..1) to true TOI (seconds)
    OutTOI = FutureHitStorage().TOI;
    OutTOI = FMath::Clamp(OutTOI, 0.f, RemainingDelta);

    return true;
}

void USSubBody::AdvanceToTOI(const float& t)
{
    // Integrate Kinematics to time t
    if (t > 0.f)
        KinematicsStorage() = KinematicsStorage().Integrate(t);
}

void USSubBody::AcceptHit()
{
    CurrentHitStorage() = FutureHitStorage();
    FutureHitStorage() = SHitResult();

    if (CurrentHitStorage().SubBody.IsValid())
    {
        USSubBody* OtherSubBody = CurrentHitStorage().SubBody.Get();
        IgnoredComponentsBodyDomainStorage().AddUnique(OtherSubBody);

        // Do NOT blindly accept OtherSubBody->FutureHit.
        // It may be an unrelated hit, e.g. Hitbox vs Stadium while Sensor vs Hitbox is being resolved.

        if (OtherSubBody->WillHit())
        {
            const SHitResult& OtherFutureHit = OtherSubBody->GetFutureHit();

            const bool bIsReciprocal =
                OtherFutureHit.SubBody.Get() == this ||
                OtherFutureHit.Component.Get() == this;

            const bool bSensorInvolved =
                IsSensor() || OtherSubBody->IsSensor();

            // For sensors, never consume the other's FutureHit.
            // Sensors are gameplay-only and must not steal physical hits.
            if (bIsReciprocal && !bSensorInvolved)
            {
                OtherSubBody->AcceptHit();
            }
        }
    }
    else if (CurrentHitStorage().Component.IsValid())
    {
        IgnoredComponentsBodyDomainStorage().AddUnique(CurrentHitStorage().Component.Get());
    }
}

bool USSubBody::ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const
{
	return IgnoredComponentsBodyDomainStorage().Contains(&OtherComp);
}

void USSubBody::ResolveCurrentHit(const float& delta, const float& SimTime)
{
    if (CurrentHitStorage().bBlockingHit)
        ResolveCurrentHitPrv(delta, SimTime);
}

bool USSubBody::InternalSweep(const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
{
    UWorld* World = GetWorld();
    if (!World) return false;

	FHitResult Hit;
	bool bHit = false;
	Speed::Analytic::FWorldHit AnalyticHit;
	const bool bUsedAnalyticAuthority =
		Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), Hit, bHit,
		&AnalyticHit);
	if (!bUsedAnalyticAuthority)
	{
		const FCollisionQueryParams Params = BuildTraceParams();
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->SweepSingleByChannel(Hit, Start, End,
			KinematicsStorage().Rotation, GetCollisionChannel(), GetCollisionShape(),
			Params, GetResponseParams());
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::SubBodySweep,
		Start, End, KinematicsStorage().Rotation, GetCollisionShape(),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, Hit);

	OutHit = SHitResult();
	OutHit.bHit = bHit;
	OutHit.bBlockingHit = bHit;
    if (bHit)
    {
		OutHit = bUsedAnalyticAuthority
			? SHitResult::FromAnalyticHit(AnalyticHit, delta, Hit.Component)
			: SHitResult::FromUnrealHit(Hit, delta);
    }
	return bHit;
}

FCollisionQueryParams USSubBody::BuildTraceParams() const
{
    FCollisionQueryParams Params(NAME_None, false);
    Params.bReturnFaceIndex = true;
    Params.bReturnPhysicalMaterial = true;
	Params.AddIgnoredComponents(IgnoredComponentsBodyDomainStorage());
    return Params;
}

uint64 USSubBody::MakePairKey(const USSubBody* A, const UPrimitiveComponent* OtherComp)
{
    const uint32 IdA = A ? (uint32)A->GetUniqueID() : 0u;
    const uint32 IdB = OtherComp ? (uint32)OtherComp->GetUniqueID() : 0u;

    const uint32 Lo = FMath::Min(IdA, IdB);
    const uint32 Hi = FMath::Max(IdA, IdB);
    return (uint64(Hi) << 32) | uint64(Lo);
}

int32 USSubBody::SubBodyPriority(USSubBody::ESubBodyType T)
{
    switch (T)
    {
    case USSubBody::ESubBodyType::Cylinder:return 400;
    case USSubBody::ESubBodyType::Hitbox: return 300;
    case USSubBody::ESubBodyType::Sphere: return 200;
    case USSubBody::ESubBodyType::Wheel:  return 100;
    default: return 0;
    }
}

USSubBody* USSubBody::PickResolver(USSubBody* A, USSubBody* B)
{
    if (!A) return B;
    if (!B) return A; // world/static

    // Sensor always resolves (gameplay first, no physical response)
    if (A->IsSensor()) return A;
    if (B->IsSensor()) return B;

    const int32 PA = SubBodyPriority(A->GetSubBodyType());
    const int32 PB = SubBodyPriority(B->GetSubBodyType());
    if (PA != PB) return (PA > PB) ? A : B;

    // tie-break deterministic
    return (A->GetUniqueID() < B->GetUniqueID()) ? A : B;
}

const TArray<TWeakObjectPtr<UBoxSubBody>>& USSubBody::GetExternalBoxSubBodies() const
{
    return ExternalBoxSubBodiesBodyDomainStorage();
}

const TArray<TWeakObjectPtr<USphereSubBody>>& USSubBody::GetExternalSphereSubBodies() const
{
    return ExternalSphereSubBodiesBodyDomainStorage();
}

const TArray<TWeakObjectPtr<USWheelSubBody>>& USSubBody::GetExternalWheelSubBodies() const
{
    return ExternalWheelSubBodiesBodyDomainStorage();
}


