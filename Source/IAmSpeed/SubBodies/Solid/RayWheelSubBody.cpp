#include "RayWheelSubBody.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"

#include "IAmSpeed/Base/SUtils.h"
#include "IAmSpeed/Components/ISpeedWheeledComponent.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "Engine/World.h"

URayWheelSubBody::URayWheelSubBody(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

bool URayWheelSubBody::BuildSuspensionRay(const float Delta,
	FVector& OutStart, FVector& OutEnd, FVector& OutSuspensionUp) const
{
	if (!ParentComponent || Delta <= KINDA_SMALL_NUMBER)
	{
		return false;
	}

	const FTransform ChassisTransform(
		ParentComponent->GetPhysRotation(), ParentComponent->GetPhysLocation());
	OutSuspensionUp = ChassisTransform.GetUnitAxis(EAxis::Z);
	const FVector WorldRest = ChassisTransform.TransformPosition(GetLocalOffset());
	OutStart = WorldRest + MaxLength() * OutSuspensionUp;
	const float NextDisplacement = PredictNextDisplacement(Delta);
	const FVector PredictedWheelCenter = WorldPos()
		+ (NextDisplacement - SpringDisplacement() - CollisionMargin())
			* OutSuspensionUp;
	OutEnd = PredictedWheelCenter - Radius() * OutSuspensionUp;
	return !OutSuspensionUp.IsNearlyZero();
}

void URayWheelSubBody::AcceptSuspensionRayHit(
	SHitResult& Hit, const FVector& SuspensionUp) const
{
	Hit.ImpactNormal = Speed::QuantizeUnitNormal(Hit.ImpactNormal);
	Hit.Location = Hit.ImpactPoint + Radius() * SuspensionUp;
}

bool URayWheelSubBody::SweepSuspensionOnGround(
	SHitResult& OutHit, const float& Delta)
{
	UWorld* World = GetWorld();
	FVector Start;
	FVector End;
	FVector SuspensionUp;
	if (!World || !BuildSuspensionRay(Delta, Start, End, SuspensionUp))
	{
		return false;
	}

	FCollisionQueryParams Params(NAME_None, false);
	Params.bReturnFaceIndex = true;
	Params.bReturnPhysicalMaterial = true;
	if (const AActor* VehicleOwner = GetOwner())
	{
		Params.AddIgnoredActor(VehicleOwner);
	}

	FHitResult UnrealHit;
	bool bHit = false;
	if (!Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, FQuat::Identity, FCollisionShape::MakeSphere(0.0f),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), UnrealHit, bHit,
		nullptr))
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->LineTraceSingleByChannel(UnrealHit, Start, End,
			GetCollisionChannel(), Params, GetResponseParams());
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::RayWheelSuspensionProbe,
		Start, End, FQuat::Identity, FCollisionShape::MakeSphere(0.0f),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, UnrealHit);
	if (!bHit)
	{
		SetOnGround(false);
		if (WheelComponent)
		{
			WheelComponent->UpdateWheelOnGroundStates();
		}
		return false;
	}

	OutHit = SHitResult::FromUnrealHit(UnrealHit, Delta);
	AcceptSuspensionRayHit(OutHit, SuspensionUp);
	SetOnGround(true);
	if (WheelComponent)
	{
		WheelComponent->UpdateWheelOnGroundStates();
	}
	return true;
}

bool URayWheelSubBody::SweepSuspensionOnSpheres(
	SHitResult& OutHit, const float& Delta)
{
	FVector Start;
	FVector End;
	FVector SuspensionUp;
	if (!BuildSuspensionRay(Delta, Start, End, SuspensionUp))
	{
		return false;
	}

	const SSRay Ray;
	bool bHit = false;
	float BestTOI = Delta;
	SHitResult BestHit;
	for (const TWeakObjectPtr<USphereSubBody>& OtherSphere : ExternalSphereSubBodies)
	{
		if (!OtherSphere.IsValid() || OtherSphere->GetOwner() == GetOwner()
			|| IgnoredComponents.Contains(OtherSphere.Get()))
		{
			continue;
		}
		const SHitResult Hit = Ray.IntersectDuringMovement(
			OtherSphere->MakeSphere(), Start, End, Delta);
		if (Hit.bHit && Hit.TOI <= BestTOI)
		{
			BestTOI = Hit.TOI;
			BestHit = Hit;
			BestHit.SubBody = OtherSphere.Get();
			BestHit.Component = OtherSphere.Get();
			bHit = true;
		}
	}
	if (!bHit)
	{
		return false;
	}

	OutHit = BestHit;
	AcceptSuspensionRayHit(OutHit, SuspensionUp);
	SetOnGround(true);
	if (WheelComponent)
	{
		WheelComponent->UpdateWheelOnGroundStates();
	}
	return true;
}

bool URayWheelSubBody::SweepSuspensionOnBoxes(
	SHitResult& OutHit, const float& Delta)
{
	FVector Start;
	FVector End;
	FVector SuspensionUp;
	if (!BuildSuspensionRay(Delta, Start, End, SuspensionUp))
	{
		return false;
	}

	const SSRay Ray;
	bool bHit = false;
	float BestTOI = Delta;
	SHitResult BestHit;
	for (const TWeakObjectPtr<UBoxSubBody>& OtherBox : ExternalBoxSubBodies)
	{
		if (!OtherBox.IsValid() || OtherBox->GetOwner() == GetOwner()
			|| IgnoredComponents.Contains(OtherBox.Get()))
		{
			continue;
		}
		const SHitResult Hit = Ray.IntersectDuringMovement(
			OtherBox->MakeBox(), Start, End, Delta);
		if (Hit.bHit && Hit.TOI <= BestTOI)
		{
			BestTOI = Hit.TOI;
			BestHit = Hit;
			BestHit.SubBody = OtherBox.Get();
			BestHit.Component = OtherBox.Get();
			bHit = true;
		}
	}
	if (!bHit)
	{
		return false;
	}

	OutHit = BestHit;
	AcceptSuspensionRayHit(OutHit, SuspensionUp);
	SetOnGround(true);
	if (WheelComponent)
	{
		WheelComponent->UpdateWheelOnGroundStates();
	}
	return true;
}

bool URayWheelSubBody::SweepSuspensionAlongNormal(
	const FVector& Normal, const float SearchDistance, const float Delta,
	SHitResult& OutHit) const
{
	UWorld* World = GetWorld();
	const FVector SearchNormal = Normal.GetSafeNormal();
	const FVector SuspensionUp = GetSuspensionDirectionWS().GetSafeNormal();
	if (!World || SearchNormal.IsNearlyZero() || SuspensionUp.IsNearlyZero()
		|| SearchDistance <= 0.0f)
	{
		return false;
	}

	FCollisionQueryParams Params(NAME_None, false);
	Params.bReturnFaceIndex = true;
	Params.bReturnPhysicalMaterial = true;
	if (const AActor* VehicleOwner = GetOwner())
	{
		Params.AddIgnoredActor(VehicleOwner);
	}
	const FVector CurrentRayPoint = WorldPos() - Radius() * SuspensionUp;
	FHitResult UnrealHit;
	const FVector Start = CurrentRayPoint + SearchDistance * SearchNormal;
	const FVector End = CurrentRayPoint - SearchDistance * SearchNormal;
	bool bHit = false;
	if (!Speed::Analytic::FStaticWorldQueryAudit::TryCompactAuthoritySingle(
		World,
		Start, End, FQuat::Identity, FCollisionShape::MakeSphere(0.0f),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), UnrealHit, bHit,
		nullptr))
	{
		Speed::Analytic::FStaticWorldQueryAudit::RecordLegacySweep();
		bHit = World->LineTraceSingleByChannel(UnrealHit, Start, End,
			GetCollisionChannel(), Params, GetResponseParams());
	}
	Speed::Analytic::FStaticWorldQueryAudit::RecordSingle(
		Speed::Analytic::EStaticQuerySite::RayWheelEstablishedSupportProbe,
		Start, End, FQuat::Identity, FCollisionShape::MakeSphere(0.0f),
		static_cast<uint8>(GetCollisionChannel()), GetResponseParams(), bHit, UnrealHit);
	if (!bHit)
	{
		return false;
	}

	OutHit = SHitResult::FromUnrealHit(UnrealHit, Delta);
	AcceptSuspensionRayHit(OutHit, SuspensionUp);
	return true;
}
