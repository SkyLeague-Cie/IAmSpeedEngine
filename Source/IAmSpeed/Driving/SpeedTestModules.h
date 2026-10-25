#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Actors/SpeedStaticActor.h"
#include "SpeedTestModules.generated.h"
class URingSubBody;
class UStaticMeshComponent;

/** Full torus solid, matching the Rings/Aurora analytic provider. The mesh is
 * presentation only and is authored explicitly with the same radii. */
UCLASS(Blueprintable)
class IAMSPEED_API ASpeedRingObstacle : public AActor
{
    GENERATED_BODY()
public:
    ASpeedRingObstacle();
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Ring") TObjectPtr<URingSubBody> Solid;
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Ring") TObjectPtr<UStaticMeshComponent> Visual;
};

/** Explicit closed mesh module admitted by the ordinary analytic bake. No
 * mesh-to-subbody auto link or shape preset is created. Vertex positions in cm:
 * entrance x=0,z=0; run rises to Rise; optional table, then symmetric descent.
 */
UCLASS(Blueprintable)
class IAMSPEED_API ASpeedRampModule : public ASpeedStaticActor
{
    GENERATED_BODY()
public:
    ASpeedRampModule();
    UPROPERTY(VisibleAnywhere,BlueprintReadOnly,Category="Ramp") TObjectPtr<UStaticMeshComponent> SolidMesh;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Ramp") double Run=3000;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Ramp") double Rise=420;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Ramp") double Width=1600;
    UPROPERTY(EditAnywhere,BlueprintReadWrite,Category="Ramp") double TableLength=0;
    /** Values-only convex prism definition; the editor authoring tool consumes it. */
    UFUNCTION(BlueprintCallable,Category="Ramp") bool BuildClosedPrism(TArray<FVector>& Vertices,TArray<int32>& Triangles) const;
};
