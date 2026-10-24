#pragma once
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SpeedTestModuleAuthoring.generated.h"
class UStaticMesh;

/** Editor-only explicit mesh authoring. Runtime components never generate or
 * auto-link collision shapes. Inputs use mathematical outward winding. */
UCLASS()
class IAMSPEEDEDITOR_API USpeedTestModuleAuthoring : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable,Category="IAmSpeed|Test modules")
    static UStaticMesh* CreateModuleMesh(const FString& PackageName,const TArray<FVector>& Vertices,const TArray<int32>& Triangles);
};
