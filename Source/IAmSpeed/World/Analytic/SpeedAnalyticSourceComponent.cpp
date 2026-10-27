#include "SpeedAnalyticSourceComponent.h"

#include "SpeedAnalyticCollisionAsset.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/Actor.h"

USpeedAnalyticSourceComponent::USpeedAnalyticSourceComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

UPrimitiveComponent* USpeedAnalyticSourceComponent::ResolveAuthoredContact(
	const FSpeedAnalyticAuthoredSourceRecord& Record,
	const USpeedAnalyticCollisionAsset& Asset, FString* OutReason) const
{
	auto Fail = [OutReason](const TCHAR* Reason) -> UPrimitiveComponent*
	{
		if (OutReason) *OutReason = Reason;
		return nullptr;
	};
	if (!Record.IsValid(OutReason)) return nullptr;
	const AActor* Owner = GetOwner();
	if (!Owner || !IsRegistered() || AuthoredSourceId != Record.SourceId ||
		bIncludeOwnedStaticMeshes || AnalyticCollisionAsset.Get() != &Asset ||
		GetPathName(Owner) != Record.SourceComponentPath ||
		AuthoredContactComponentPath != Record.ContactComponentPath ||
		!Owner->GetActorTransform().IsValid() ||
		!Owner->GetActorTransform().Equals(Record.OwnerWorldTransform, 0.0))
		return Fail(TEXT("Authored source declaration, asset or owner transform mismatch."));
	TArray<UPrimitiveComponent*> Candidates;
	Owner->GetComponents<UPrimitiveComponent>(Candidates);
	UPrimitiveComponent* Contact = nullptr;
	for (UPrimitiveComponent* Candidate : Candidates)
	{
		if (!Candidate || Candidate->GetOwner() != Owner ||
			Candidate->GetPathName(Owner) != Record.ContactComponentPath) continue;
		if (Contact) return Fail(TEXT("Authored contact path is ambiguous."));
		Contact = Candidate;
	}
	if (!Contact || !Contact->IsRegistered() || Contact->GetMobility() != EComponentMobility::Static ||
		!Contact->GetComponentTransform().IsValid() ||
		!Contact->GetComponentTransform().Equals(Record.ContactWorldTransform, 0.0))
		return Fail(TEXT("Authored contact is missing, nonstatic, unregistered or moved."));
	uint64 Blocking = 0, Overlap = 0;
	for (uint32 Channel = 0; Channel < UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray); ++Channel)
	{
		const ECollisionResponse Response = Contact->GetCollisionResponseToChannel(
			static_cast<ECollisionChannel>(Channel));
		if (Response == ECR_Block) Blocking |= uint64(1) << Channel;
		if (Response == ECR_Overlap) Overlap |= uint64(1) << Channel;
	}
	if (Record.ObjectType != static_cast<uint32>(Contact->GetCollisionObjectType()) ||
		Record.CollisionEnabled != static_cast<uint8>(Contact->GetCollisionEnabled()) ||
		Record.bQueryCollisionEnabled != Contact->IsQueryCollisionEnabled() ||
		Record.BlockingChannels != Blocking || Record.OverlapChannels != Overlap)
		return Fail(TEXT("Authored contact collision policy differs from the immutable source record."));
	return Contact;
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
