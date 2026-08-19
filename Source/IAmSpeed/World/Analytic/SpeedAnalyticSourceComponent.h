#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SpeedAnalyticSourceComponent.generated.h"

class UStaticMeshComponent;

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
};
