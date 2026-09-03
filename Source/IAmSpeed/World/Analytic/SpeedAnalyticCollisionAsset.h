#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "SpeedAnalyticCollisionAsset.generated.h"

class FDataValidationContext;

namespace Speed::Analytic
{
struct FAnalyticWorldData;
}

USTRUCT(BlueprintType)
struct IAMSPEED_API FSpeedAnalyticMeshSourceRecord
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 SourceId = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 MeshId = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FString ActorPath;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FString ComponentPath;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FString MeshPath;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FTransform WorldTransform = FTransform::Identity;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	int32 LodIndex = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	int32 TriangleCount = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	int32 DegenerateTriangleCount = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FBox WorldBounds = FBox(EForceInit::ForceInit);

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 TriangleHash = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 NormalHash = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 MaterialHash = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 CollisionPolicyHash = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint32 ObjectType = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint8 CollisionEnabled = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	bool bQueryCollisionEnabled = false;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 BlockingChannels = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 OverlapChannels = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint8 CollisionTraceFlag = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	TArray<uint64> MaterialIds;
};

USTRUCT(BlueprintType)
struct IAMSPEED_API FSpeedAnalyticTriangleRecord
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 SourceId = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 SurfaceId = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 FeatureId = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 PrimitiveId = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint32 MaterialId = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FVector Vertex0 = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FVector Vertex1 = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FVector Vertex2 = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FVector Normal0 = FVector::UpVector;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FVector Normal1 = FVector::UpVector;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	FVector Normal2 = FVector::UpVector;
};

USTRUCT(BlueprintType)
struct IAMSPEED_API FSpeedAnalyticIndexedTriangleRecord
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 SourceId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 SurfaceId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 FeatureId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 PrimitiveId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint32 MaterialId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") FIntVector VertexIndices = FIntVector::ZeroValue;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") FIntVector NormalIndices = FIntVector::ZeroValue;
	// Exact finite-plane fallback for a source whose complete indexed topology
	// was certified closed and two-manifold by the bake. It is considered only
	// when no higher-level compact primitive answers the authority query.
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") bool bResidualAuthorityEligible = false;
};

USTRUCT(BlueprintType)
struct IAMSPEED_API FSpeedAnalyticBoundedPlaneRecord
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	uint64 SourceId = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	uint64 SurfaceId = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	uint64 FeatureId = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	uint64 PrimitiveId = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	int32 MaterialId = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	int32 ObjectType = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	uint64 BlockingChannels = 0;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	FVector Origin = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	FVector Normal = FVector::UpVector;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	FVector AxisU = FVector::ForwardVector;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	FVector AxisV = FVector::RightVector;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	FVector2D HalfExtents = FVector2D::ZeroVector;

	// Generated records remain immutable at runtime, but deterministic editor
	// authoring utilities must be able to replace an exact finite domain.
	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	TArray<FVector2D> DomainVertices;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	bool bQueryCollisionEnabled = true;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	bool bRequiresCompactOptIn = false;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	bool bAuthorityEligible = false;
};

USTRUCT()
struct IAMSPEED_API FSpeedAnalyticExtrudedQuinticPatchRecord
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 SourceId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 SurfaceId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 FeatureId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 PrimitiveId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 CanonicalGroupId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint8 CanonicalSymmetryAxisMask = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint32 MaterialId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint32 ObjectType = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 BlockingChannels = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") TArray<FVector> SectionControlPoints;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") TArray<FVector> InteriorCorrectionControlPoints;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") double BaseRootMeanSquareResidualCm = 0.0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") double BaseMaximumResidualCm = 0.0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") double CorrectedRootMeanSquareResidualCm = 0.0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") double CorrectedMaximumResidualCm = 0.0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") double AdditionalResidualAgreementAllowanceCm = 0.0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") FVector ExtrusionAxis = FVector::ForwardVector;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") double MinimumExtrusionCoordinate = 0.0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") double MaximumExtrusionCoordinate = 0.0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") FBox Bounds = FBox(EForceInit::ForceInit);
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") bool bQueryCollisionEnabled = false;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") bool bCanonicalC2ByConstruction = false;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") bool bCanonicalSymmetryByConstruction = false;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") bool bAuthorityEligible = false;
};

USTRUCT()
struct IAMSPEED_API FSpeedAnalyticTensorBezierPatchRecord
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 SourceId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 SurfaceId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 FeatureId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 PrimitiveId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 CanonicalGroupId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint32 MaterialId = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint32 ObjectType = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") uint64 BlockingChannels = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") int32 DegreeU = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") int32 DegreeV = 0;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") TArray<FVector> ControlPoints;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") bool bQueryCollisionEnabled = false;
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision") bool bAuthorityEligible = false;
};

USTRUCT()
struct IAMSPEED_API FSpeedAnalyticPiecewiseTensorBezierCellRecord
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint64 FeatureId = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint64 PrimitiveId = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") double MinimumU = 0.0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") double MaximumU = 1.0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") double MinimumV = 0.0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") double MaximumV = 1.0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") double LongitudinalParameterScale = 1.0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") bool bTerminalClosure = false;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") int32 DegreeU = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") int32 DegreeV = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") TArray<FVector> ControlPoints;
};

USTRUCT()
struct IAMSPEED_API FSpeedAnalyticPiecewiseTensorBezierPatchRecord
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint64 SourceId = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint64 SurfaceId = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint64 PrimitiveId = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint64 CanonicalGroupId = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint32 MaterialId = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint32 ObjectType = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") uint64 BlockingChannels = 0;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") TArray<FSpeedAnalyticPiecewiseTensorBezierCellRecord> Cells;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") bool bQueryCollisionEnabled = false;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") bool bSourceResidualCertified = false;
	UPROPERTY(EditAnywhere, Category = "Analytic Collision") bool bAuthorityEligible = false;
};

/**
 * Versioned generated artifact. Authoring adapters populate this asset; the
 * simulation consumes only the validated, immutable runtime copy returned by
 * BuildRuntimeData.
 */
UCLASS(BlueprintType)
class IAMSPEED_API USpeedAnalyticCollisionAsset : public UDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint32 BakeSchemaVersion = 9;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint32 SchemaVersion = 6;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	uint64 SourceHash = 0;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	TArray<FSpeedAnalyticMeshSourceRecord> MeshSources;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	TArray<FSpeedAnalyticTriangleRecord> Triangles;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	TArray<FVector> VertexPositions;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	TArray<FVector> VertexNormals;

	UPROPERTY(VisibleAnywhere, Category = "Analytic Collision")
	TArray<FSpeedAnalyticIndexedTriangleRecord> IndexedTriangles;

	// Exposed for explicit editor/commandlet authoring only. BuildRuntimeData
	// still validates and copies these records into an immutable runtime world.
	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	TArray<FSpeedAnalyticBoundedPlaneRecord> BoundedPlanes;

	// Generated records remain immutable at runtime, but deterministic editor
	// authoring utilities must be able to refresh an exact finite domain.
	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	TArray<FSpeedAnalyticExtrudedQuinticPatchRecord> ExtrudedQuinticPatches;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	TArray<FSpeedAnalyticTensorBezierPatchRecord> TensorBezierPatches;

	UPROPERTY(EditAnywhere, Category = "Analytic Collision")
	TArray<FSpeedAnalyticPiecewiseTensorBezierPatchRecord> PiecewiseTensorBezierPatches;

	TSharedPtr<const Speed::Analytic::FAnalyticWorldData> BuildRuntimeData(
		FString* OutReason = nullptr,
		bool bBuildRecognitionDiagnostics = false,
		bool bAuthorityProvidersOnly = false) const;

	bool ValidateGeneratedData(
		FString* OutReason = nullptr,
		bool bAuthorityProvidersOnly = false) const;

#if WITH_EDITOR
	virtual void PreSave(FObjectPreSaveContext SaveContext) override;
	virtual EDataValidationResult IsDataValid(FDataValidationContext& Context) const override;
#endif
};
