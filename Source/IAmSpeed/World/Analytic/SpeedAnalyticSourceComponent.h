#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpeedAnalyticSourceComponent.generated.h"

class UStaticMeshComponent;
class USpeedAnalyticCollisionAsset;

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

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Analytic Collision")
	bool bIncludeOwnedStaticMeshes = true;

	/**
	 * Optional immutable provider set authored in the world transform recorded
	 * by its mesh-source manifest. Every live owned mesh must match that
	 * manifest before the runtime is allowed to consume the asset.
	 */
	UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Analytic Collision")
	TSoftObjectPtr<USpeedAnalyticCollisionAsset> AnalyticCollisionAsset;
};
