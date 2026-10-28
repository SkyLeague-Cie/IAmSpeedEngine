#pragma once

#include "CoreMinimal.h"
#include "Components/PrimitiveComponent.h"
#include "SpeedAnalyticContactComponent.generated.h"

/** Identity/filter anchor for an authored analytic source. No mesh, render proxy,
 * Chaos body or replacement collision shape. Geometry stays in the source asset. */
UCLASS(ClassGroup=(IAmSpeed), meta=(BlueprintSpawnableComponent))
class IAMSPEED_API USpeedAnalyticContactComponent : public UPrimitiveComponent
{
    GENERATED_BODY()
public:
    USpeedAnalyticContactComponent();
    bool ShouldCreatePhysicsState() const override { return false; }
    UBodySetup* GetBodySetup() override { return nullptr; }
    FPrimitiveSceneProxy* CreateSceneProxy() override { return nullptr; }
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;

    /** Explicit authoring migration only. Copies the existing source policy,
     * never creates a profile, changes geometry or grants query eligibility. */
    UFUNCTION(BlueprintCallable, Category="IAmSpeed|Analytic Collision")
    void ApplyAuthoredSourcePolicy(bool& OutSuccess, FString& OutReason);

    UFUNCTION(BlueprintCallable, Category="IAmSpeed|Analytic Collision")
    void RefreshAuthoredBounds(bool& OutSuccess, FString& OutReason);
protected:
    void OnRegister() override;
private:
    FBox LocalAnalyticBounds = FBox(EForceInit::ForceInit);
};
