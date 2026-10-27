#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpeedAnalyticSourceComponent.generated.h"

class UStaticMeshComponent;
class UPrimitiveComponent;
class USpeedAnalyticCollisionAsset;
struct FSpeedAnalyticAuthoredSourceRecord;

/**
 * Unreal-side authoring source. It discovers eligible static meshes owned by
 * one actor; generated analytical data never retains this component.
 */
UCLASS(ClassGroup = (IAmSpeed), meta = (BlueprintSpawnableComponent))
class IAMSPEED_API USpeedAnalyticSourceComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	USpeedAnalyticSourceComponent();

	void GatherOwnedStaticMeshes(TArray<UStaticMeshComponent*>& OutComponents) const;
	bool IsAutomaticallyEligible(const UStaticMeshComponent& Component) const;
	UPrimitiveComponent* ResolveAuthoredContact(
		const FSpeedAnalyticAuthoredSourceRecord& Record,
		const USpeedAnalyticCollisionAsset& Asset, FString* OutReason = nullptr) const;

	// Zero selects legacy mesh discovery. A nonzero declaration requires the
	// explicit asset, disabled mesh discovery and an exact owned contact path.
	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	uint64 AuthoredSourceId = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	FString AuthoredContactComponentPath;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Analytic Collision")
	bool bIncludeOwnedStaticMeshes = true;

	/**
	 * Optional immutable provider set authored in the world transform recorded
	 * by its source manifest. Mesh and authored analytic sources have separate
	 * provenance; every declared source must resolve before runtime admission.
	 */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Analytic Collision")
	TSoftObjectPtr<USpeedAnalyticCollisionAsset> AnalyticCollisionAsset;
};
