#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SpeedAnalyticAuthoredImport.generated.h"

class USpeedAnalyticCollisionAsset;
class USpeedAnalyticSourceComponent;
class UPrimitiveComponent;
class UStaticMesh;
class UStaticMeshComponent;
class UBlueprint;
class UWorld;
class AActor;

/** Explicit, editor-only import of the acquired stadium coefficient payload.
 * Staging is transient; persistence is an explicit separate operation.
 * No fitting, fabricated mesh provenance or physical authority admission.
 */
UCLASS()
class IAMSPEEDEDITOR_API USpeedAnalyticAuthoredImport : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
public:
	/** Temporary, identity-transformed Editor probe of the real compiled Blueprint. Never PIE/Game. */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static AActor* SpawnStadiumEditorProbe(UBlueprint* Blueprint, UWorld* World, FString& OutReason);

	/** Destroy only a transient probe created by the companion authoring helper. */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static bool DestroyStadiumEditorProbe(AActor* Actor);

	/** Persist only the analytic payload; the already-installed render asset is untouched. */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static void PersistStadiumAnalyticV1(USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact,
		const FString& ExpectedProjectDirectory, const FString& PackageRoot, bool& OutSuccess,
		USpeedAnalyticCollisionAsset*& OutAnalyticAsset, FString& OutManifestJson, FString& OutReason);

	/** Snapshot the effective policy, including every stored (also Python-hidden) channel. */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static FString DescribeContactPolicy(UPrimitiveComponent* Contact);

	/** Explicit Editor authoring: copy actual policy onto its template, without a BodyInstance/handle copy. */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static bool CopyExplicitContactPolicy(UPrimitiveComponent* Actual, UPrimitiveComponent* ContactTemplate);
	/** Reads the three pinned v1 inputs under this project's Saved/RLDimension.
	 * Returns a transient diagnostic candidate and its identity/domain manifest.
	 * Retain the returned object explicitly; nothing is installed or saved.
	 * Source and Contact must be actual registered components in an editor world.
	 * Coordinates are already world coordinates and are copied without transforms.
	 */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static USpeedAnalyticCollisionAsset* StageStadiumV1(
		USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact,
		FString& OutManifestJson, FString& OutReason);

	/** Builds a transient render-only mesh from the pinned derived Wavefront.
	 * Keeps shared position IDs and supplied analytic corner normals. No world,
	 * component, material asset, physics source, package or saved asset is changed.
	 * UVs are explicitly unbound; retain the returned candidate explicitly.
	 */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static UStaticMesh* StageStadiumRenderV1(FString& OutManifestJson, FString& OutReason);

	/** Save fresh persistent copies of both staged candidates in this project.
	 * ExpectedProjectDirectory prevents accidentally writing a different checkout.
	 * PackageRoot must be below /Game/Generated/Analytic/RLDimension/ and absent.
	 * No Blueprint is changed here and no analytic authority flag is promoted.
	 * OutManifestJson records partial saves on failure; never retry over outputs.
	 */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static void PersistStadiumV1(USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact,
		const FString& ExpectedProjectDirectory, const FString& PackageRoot,
		bool& OutSuccess, UStaticMesh*& OutRenderMesh, USpeedAnalyticCollisionAsset*& OutAnalyticAsset,
		FString& OutManifestJson, FString& OutReason);

	/** Inspect actual buffers and persisted section maps; never promotes physics. */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static void ValidateStadiumRenderV2(UStaticMesh* Mesh, bool& OutSuccess, FString& OutManifestJson, FString& OutReason);

	/** Revalidate the saved source/contact record after Blueprint reconstruction.
	 * This checks binding only, never physical primitive eligibility/admission.
	 */
	UFUNCTION(BlueprintCallable, Category = "IAmSpeed|Analytic Import")
	static void ValidateStadiumBinding(USpeedAnalyticSourceComponent* Source, UPrimitiveComponent* Contact,
		USpeedAnalyticCollisionAsset* Asset, bool& OutMatches, FString& OutReason);
};
