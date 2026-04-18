// Fill out your copyright notice in the Description page of Project Settings.


#include "SSubBody.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "Configs/SubBodyConfig.h"

USSubBody::USSubBody(const FObjectInitializer& ObjectInitializer):
	Super(ObjectInitializer)
{
    PrimaryComponentTick.bCanEverTick = false;
    ResponseParams.CollisionResponse.SetAllChannels(ECollisionResponse::ECR_Ignore);
    ResponseParams.CollisionResponse.SetResponse(ECollisionChannel::ECC_WorldStatic, ECollisionResponse::ECR_Block);
}

void USSubBody::Initialize(ISpeedComponent* InParentComponent)
{
	ParentComponent = InParentComponent;
    if (ParentComponent)
    {
		SubBodyConfig Config = ParentComponent->GetSubBodyConfig(*this);
        if (Config.bValid)
        {
            SetRelativeLocation(Config.LocalOffset);
            SetRelativeRotation(Config.LocalRotation);
            CollisionChannel = Config.CollisionChannel;
        }
        
        for (USSubBody* SubBody: ParentComponent->GetSubBodies())
        {
            if (SubBody)
            {
                AlwaysIgnoredComponents.AddUnique(SubBody);
            }
		}
    }
}

void USSubBody::ResetForFrame(const float& Delta)
{
    FutureHit = SHitResult();
    CurrentHit = SHitResult();
    IgnoredComponents.Empty();
	IgnoredComponents.Append(AlwaysIgnoredComponents);
}

void USSubBody::AddExternalSubBodies(const TArray<USSubBody*>& SubBodies)
{
    for (USSubBody* SubBody : SubBodies)
    {
        if (SubBody && !AlwaysIgnoredComponents.Contains(SubBody) && !SubBody->IsSensor())
        {
            switch (SubBody->GetSubBodyType())
            {
			case ESubBodyType::Hitbox:
				if (!IgnoredSubBodyTypes.Contains(ESubBodyType::Hitbox))
				    ExternalBoxSubBodies.AddUnique(Cast<UBoxSubBody>(SubBody));
				break;
			case ESubBodyType::Sphere:
				if (!IgnoredSubBodyTypes.Contains(ESubBodyType::Sphere))
				    ExternalSphereSubBodies.AddUnique(Cast<USphereSubBody>(SubBody));
				break;
			case ESubBodyType::Wheel:
				if (!IgnoredSubBodyTypes.Contains(ESubBodyType::Wheel))
				    ExternalWheelSubBodies.AddUnique(Cast<USWheelSubBody>(SubBody));
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
            case ESubBodyType::Hitbox:
                ExternalBoxSubBodies.Remove(Cast<UBoxSubBody>(SubBody));
                break;
            case ESubBodyType::Sphere:
                ExternalSphereSubBodies.Remove(Cast<USphereSubBody>(SubBody));
                break;
            case ESubBodyType::Wheel:
                ExternalWheelSubBodies.Remove(Cast<USWheelSubBody>(SubBody));
                break;
            }
        }
    }
}

void USSubBody::UpdateKinematicsFromOwner(const SKinematic& ParentKinematic)
{
	auto RelativeLocationVar = GetRelativeLocation();
	auto RelativeRotationVar = GetRelativeRotation().Quaternion();
	Kinematics.Location = ParentKinematic.Location + ParentKinematic.Rotation.RotateVector(RelativeLocationVar);
	Kinematics.Velocity = ParentKinematic.Velocity + FVector::CrossProduct(ParentKinematic.AngularVelocity, ParentKinematic.Rotation.RotateVector(RelativeLocationVar));
	Kinematics.Acceleration = ParentKinematic.Acceleration + 
        FVector::CrossProduct(ParentKinematic.AngularAcceleration, ParentKinematic.Rotation.RotateVector(RelativeLocationVar)) + 
        FVector::CrossProduct(ParentKinematic.AngularVelocity, FVector::CrossProduct(ParentKinematic.AngularVelocity, ParentKinematic.Rotation.RotateVector(RelativeLocationVar)));
    Kinematics.Rotation = ParentKinematic.Rotation * RelativeRotationVar;
	Kinematics.AngularVelocity = ParentKinematic.AngularVelocity;
	Kinematics.AngularAcceleration = ParentKinematic.AngularAcceleration;
}

bool USSubBody::SweepTOI(const float& RemainingDelta, float& OutTOI)
{
    OutTOI = RemainingDelta;
    FutureHit = SHitResult();

    const FVector Start = Kinematics.Location;
    const FVector End =
        Start +
        Kinematics.Velocity * RemainingDelta +
        0.5f * Kinematics.Acceleration * RemainingDelta * RemainingDelta;

    if (!InternalSweep(Start, End, FutureHit, RemainingDelta))
        return false;

    // Convert normalized Hit.Time (0..1) to true TOI (seconds)
    OutTOI = FutureHit.TOI;
    OutTOI = FMath::Clamp(OutTOI, 0.f, RemainingDelta);

    return true;
}

void USSubBody::AdvanceToTOI(const float& t)
{
    // Integrate Kinematics to time t
    if (t > 0.f)
        Kinematics = Kinematics.Integrate(t);
}

void USSubBody::AcceptHit()
{
    CurrentHit = FutureHit;
	FutureHit = SHitResult();

    if (CurrentHit.SubBody.IsValid())
    {
        IgnoredComponents.AddUnique(CurrentHit.SubBody.Get());
		// if Other sub-body will hit, accept the hit for it too to ensure both sides are in sync and won't try to resolve the same hit again
        auto OtherSubBody = CurrentHit.SubBody.Get();
        if (OtherSubBody->WillHit())
        {
            OtherSubBody->AcceptHit();
        }
    }
    else if (CurrentHit.Component.IsValid())
    {
        IgnoredComponents.AddUnique(CurrentHit.Component.Get());
    }
}

bool USSubBody::ComponentHasBeenIgnored(const UPrimitiveComponent& OtherComp) const
{
	return IgnoredComponents.Contains(&OtherComp);
}

void USSubBody::ResolveCurrentHit(const float& delta, const float& SimTime)
{
    if (CurrentHit.bBlockingHit)
        ResolveCurrentHitPrv(delta, SimTime);
}

bool USSubBody::InternalSweep(const FVector& Start, const FVector& End, SHitResult& OutHit, const float& delta)
{
    UWorld* World = GetWorld();
    if (!World) return false;

    FCollisionQueryParams Params = BuildTraceParams();
	FHitResult Hit;
    bool bHit = World->SweepSingleByChannel(
        Hit,
        Start,
        End,
        Kinematics.Rotation,
        GetCollisionChannel(),
        GetCollisionShape(),
        Params,
        GetResponseParams()
    );

	OutHit = SHitResult();
	OutHit.bHit = bHit;
	OutHit.bBlockingHit = bHit;
    if (bHit)
    {
		OutHit = SHitResult::FromUnrealHit(Hit, delta);
    }
	return bHit;
}

FCollisionQueryParams USSubBody::BuildTraceParams() const
{
    FCollisionQueryParams Params(NAME_None, false);
    Params.bReturnFaceIndex = true;
    Params.bReturnPhysicalMaterial = true;
	Params.AddIgnoredComponents(IgnoredComponents);
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

const TArray<TWeakObjectPtr<UBoxSubBody>> USSubBody::GetExternalBoxSubBodies() const
{
    return ExternalBoxSubBodies;
}

const TArray<TWeakObjectPtr<USphereSubBody>> USSubBody::GetExternalSphereSubBodies() const
{
    return ExternalSphereSubBodies;
}

const TArray<TWeakObjectPtr<USWheelSubBody>> USSubBody::GetExternalWheelSubBodies() const
{
    return ExternalWheelSubBodies;
}


