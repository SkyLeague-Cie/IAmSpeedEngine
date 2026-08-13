#pragma once

#include "CoreMinimal.h"
#include "SWheelSubBody.h"
#include "RayWheelSubBody.generated.h"

/**
 * Generic wheel whose suspension support is a zero-radius ray. The configured
 * wheel radius is represented as a length offset along the suspension axis.
 */
UCLASS()
class IAMSPEED_API URayWheelSubBody : public USWheelSubBody
{
	GENERATED_BODY()

public:
	URayWheelSubBody(const FObjectInitializer& ObjectInitializer);

	bool SweepSuspensionAlongNormal(const FVector& Normal, float SearchDistance,
		float Delta, SHitResult& OutHit) const override;

protected:
	bool SweepSuspensionOnGround(SHitResult& OutHit, const float& Delta) override;
	bool SweepSuspensionOnSpheres(SHitResult& OutHit, const float& Delta) override;
	bool SweepSuspensionOnBoxes(SHitResult& OutHit, const float& Delta) override;

private:
	bool BuildSuspensionRay(float Delta, FVector& OutStart, FVector& OutEnd,
		FVector& OutSuspensionUp) const;
	void AcceptSuspensionRayHit(SHitResult& Hit, const FVector& SuspensionUp) const;
};
