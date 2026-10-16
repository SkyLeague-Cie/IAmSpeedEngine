#pragma once
#include "CoreMinimal.h"
#include "SolidSubBody.h"
#include "IAmSpeed/World/Analytic/TorusSurface.h"
#include "RingSubBody.generated.h"

/** Static torus authoring body; canonical physics consumes its immutable value copy.
 * Dynamic/moving rings are deliberately not supported by this admission path.
 */
UCLASS(ClassGroup="Collision",editinlinenew,meta=(DisplayName="Ring Collision",BlueprintSpawnableComponent))
class IAMSPEED_API URingSubBody : public USolidSubBody
{
    GENERATED_UCLASS_BODY()
public:
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Shape") double MajorRadius=640.;
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Shape") double TubeRadius=64.;
    /** Procedural builders provide a stable unique identity; placed actors may use their authored path. */
    UPROPERTY(EditAnywhere,BlueprintReadOnly,Category="Shape") FName AnalyticIdentity;
    bool BuildStaticTorus(uint64 StableSourceId,Speed::Analytic::FTorusSurface& Out,FString& Error) const;
    FPrimitiveSceneProxy* CreateSceneProxy() override { return nullptr; }
    FBoxSphereBounds CalcBounds(const FTransform& LocalToWorld) const override;
};
