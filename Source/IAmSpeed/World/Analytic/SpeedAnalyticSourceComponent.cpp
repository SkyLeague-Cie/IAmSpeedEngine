#include "SpeedAnalyticSourceComponent.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"

USpeedAnalyticSourceComponent::USpeedAnalyticSourceComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

bool USpeedAnalyticSourceComponent::IsAutomaticallyEligible(
	const UStaticMeshComponent& Component) const
{
	if (!bIncludeOwnedStaticMeshes || !Component.GetStaticMesh() ||
		Component.GetMobility() != EComponentMobility::Static ||
		Component.GetCollisionEnabled() == ECollisionEnabled::NoCollision)
	{
		return false;
	}

	// A mesh that blocks only the camera is presentation/support geometry, not
	// part of the authoritative physical world. This automatically excludes the
	// Hidden camera-only or presentation-only geometry.
	for (int32 Channel = 0; Channel < UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray); ++Channel)
	{
		if (Channel != static_cast<int32>(ECC_Camera) &&
			Component.GetCollisionResponseToChannel(
				static_cast<ECollisionChannel>(Channel)) == ECR_Block)
		{
			return true;
		}
	}
	return false;
}

void USpeedAnalyticSourceComponent::GatherOwnedStaticMeshes(
	TArray<UStaticMeshComponent*>& OutComponents) const
{
	OutComponents.Reset();
	const AActor* Owner = GetOwner();
	if (!Owner)
	{
		return;
	}

	TArray<UStaticMeshComponent*> Candidates;
	Owner->GetComponents<UStaticMeshComponent>(Candidates);
	for (UStaticMeshComponent* Candidate : Candidates)
	{
		if (Candidate && IsAutomaticallyEligible(*Candidate))
		{
			OutComponents.Add(Candidate);
		}
	}
	OutComponents.Sort([](const UStaticMeshComponent& A, const UStaticMeshComponent& B)
	{
		return A.GetPathName() < B.GetPathName();
	});
}
