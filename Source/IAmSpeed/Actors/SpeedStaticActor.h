#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpeedStaticActor.generated.h"

class UStaticMeshComponent;
class USpeedAnalyticSourceComponent;

/**
 * Opt-in authoring root for deterministic analytical static-world baking.
 * Subclasses remain ordinary Unreal actors; runtime physics consumes only the
 * generated immutable payload.
 */
UCLASS(Abstract, Blueprintable)
class IAMSPEED_API ASpeedStaticActor : public AActor
{
	GENERATED_BODY()

public:
	ASpeedStaticActor();

	void GatherAnalyticStaticMeshes(TArray<UStaticMeshComponent*>& OutComponents) const;

protected:
	virtual bool ShouldIncludeAnalyticStaticMesh(
		const UStaticMeshComponent& Component) const;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Analytic Collision")
	TObjectPtr<USpeedAnalyticSourceComponent> AnalyticSource = nullptr;
};
