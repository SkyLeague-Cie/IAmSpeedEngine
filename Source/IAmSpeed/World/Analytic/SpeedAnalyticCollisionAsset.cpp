#include "SpeedAnalyticCollisionAsset.h"

#include "AnalyticWorldData.h"

#if WITH_EDITOR
#include "Misc/DataValidation.h"
#include "UObject/ObjectSaveContext.h"

DEFINE_LOG_CATEGORY_STATIC(LogSpeedAnalyticCollisionAsset, Log, All);
#endif

bool USpeedAnalyticCollisionAsset::ValidateGeneratedData(FString* OutReason) const
{
	auto Fail = [OutReason](const FString& Reason)
	{
		if (OutReason)
		{
			*OutReason = Reason;
		}
		return false;
	};
	if (BakeSchemaVersion != 4)
	{
		return Fail(FString::Printf(
			TEXT("Unsupported analytic bake schema %u."), BakeSchemaVersion));
	}
	if (SchemaVersion != Speed::Analytic::AnalyticWorldSchemaVersion)
	{
		return Fail(FString::Printf(
			TEXT("Unsupported runtime schema %u."), SchemaVersion));
	}
	if ((!MeshSources.IsEmpty() || !BoundedPlanes.IsEmpty()) && SourceHash == 0)
	{
		return Fail(TEXT("Generated analytical data has no source hash."));
	}
	uint64 PreviousSourceId = 0;
	int64 ExpectedTriangleCount = 0;
	TSet<uint64> SourceIds;
	for (int32 Index = 0; Index < MeshSources.Num(); ++Index)
	{
		const FSpeedAnalyticMeshSourceRecord& Source = MeshSources[Index];
		if (Source.SourceId == 0 || Source.MeshId == 0 || Source.TriangleCount <= 0 ||
			Source.TriangleHash == 0 || Source.NormalHash == 0 ||
			Source.CollisionPolicyHash == 0 || !Source.WorldBounds.IsValid)
		{
			return Fail(FString::Printf(
				TEXT("Mesh source record %d is incomplete."), Index));
		}
		if (Index > 0 && Source.SourceId <= PreviousSourceId)
		{
			return Fail(TEXT("Mesh source records are not uniquely sorted."));
		}
		PreviousSourceId = Source.SourceId;
		ExpectedTriangleCount += Source.TriangleCount;
		SourceIds.Add(Source.SourceId);
	}
	if (!Triangles.IsEmpty())
	{
		return Fail(TEXT("Schema-4 asset still contains expanded schema-2 triangles."));
	}
	if (ExpectedTriangleCount != IndexedTriangles.Num())
	{
		return Fail(FString::Printf(
			TEXT("Triangle payload count %d does not match source total %lld."),
			IndexedTriangles.Num(), ExpectedTriangleCount));
	}
	if (ExpectedTriangleCount > 0 &&
		(VertexPositions.IsEmpty() || VertexNormals.IsEmpty()))
	{
		return Fail(TEXT("Indexed triangle payload has an empty vertex or normal pool."));
	}
	uint64 PreviousPrimitiveId = 0;
	for (int32 Index = 0; Index < IndexedTriangles.Num(); ++Index)
	{
		const FSpeedAnalyticIndexedTriangleRecord& Triangle = IndexedTriangles[Index];
		if (!SourceIds.Contains(Triangle.SourceId) || Triangle.SurfaceId == 0 ||
			Triangle.FeatureId == 0 || Triangle.PrimitiveId == 0)
		{
			return Fail(FString::Printf(
				TEXT("Triangle record %d has incomplete identifiers."), Index));
		}
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			if (!VertexPositions.IsValidIndex(Triangle.VertexIndices[Corner]) ||
				!VertexNormals.IsValidIndex(Triangle.NormalIndices[Corner]))
			{
				return Fail(FString::Printf(
					TEXT("Indexed triangle record %d references an invalid pool index."),
					Index));
			}
		}
		if (Index > 0 && Triangle.PrimitiveId <= PreviousPrimitiveId)
		{
			return Fail(TEXT("Triangle records are not uniquely sorted."));
		}
		PreviousPrimitiveId = Triangle.PrimitiveId;
	}
	TSet<uint64> PlanePrimitiveIds;
	for (int32 Index = 0; Index < BoundedPlanes.Num(); ++Index)
	{
		const FSpeedAnalyticBoundedPlaneRecord& Record = BoundedPlanes[Index];
		Speed::Analytic::FBoundedPlane Plane;
		Plane.SourceId = Record.SourceId;
		Plane.SurfaceId = Record.SurfaceId;
		Plane.FeatureId = Record.FeatureId;
		Plane.PrimitiveId = Record.PrimitiveId;
		Plane.MaterialId = static_cast<uint32>(FMath::Max(0, Record.MaterialId));
		Plane.ObjectType = static_cast<uint32>(FMath::Max(0, Record.ObjectType));
		Plane.BlockingChannels = Record.BlockingChannels;
		Plane.Origin = FVector3d(Record.Origin);
		Plane.Normal = FVector3d(Record.Normal);
		Plane.AxisU = FVector3d(Record.AxisU);
		Plane.AxisV = FVector3d(Record.AxisV);
		Plane.HalfExtents = FVector2d(Record.HalfExtents);
		Plane.bQueryCollisionEnabled = Record.bQueryCollisionEnabled;
		Plane.bRequiresCompactOptIn = Record.bRequiresCompactOptIn;
		Plane.bAuthorityEligible = Record.bAuthorityEligible;
		FString PlaneReason;
		if (!Plane.IsValid(&PlaneReason))
		{
			return Fail(FString::Printf(
				TEXT("Bounded plane record %d is invalid: %s"), Index, *PlaneReason));
		}
		if (PlanePrimitiveIds.Contains(Record.PrimitiveId))
		{
			return Fail(FString::Printf(
				TEXT("Bounded plane record %d duplicates primitive %016llX."),
				Index, Record.PrimitiveId));
		}
		PlanePrimitiveIds.Add(Record.PrimitiveId);
	}
	PreviousPrimitiveId = 0;
	for (int32 Index = 0; Index < ExtrudedQuinticPatches.Num(); ++Index)
	{
		const FSpeedAnalyticExtrudedQuinticPatchRecord& Patch =
			ExtrudedQuinticPatches[Index];
		if (!SourceIds.Contains(Patch.SourceId) || Patch.SurfaceId == 0 ||
			Patch.FeatureId == 0 || Patch.PrimitiveId == 0 ||
			Patch.SectionControlPoints.Num() != 6 ||
			Patch.InteriorCorrectionControlPoints.Num() != 2 ||
			!Patch.Bounds.IsValid ||
			!FMath::IsNearlyEqual(Patch.ExtrusionAxis.SquaredLength(), 1.0, 1.0e-6) ||
			!FMath::IsFinite(Patch.MinimumExtrusionCoordinate) ||
			!FMath::IsFinite(Patch.MaximumExtrusionCoordinate) ||
			!FMath::IsFinite(Patch.BaseRootMeanSquareResidualCm) ||
			!FMath::IsFinite(Patch.BaseMaximumResidualCm) ||
			!FMath::IsFinite(Patch.CorrectedRootMeanSquareResidualCm) ||
			!FMath::IsFinite(Patch.CorrectedMaximumResidualCm) ||
			Patch.BaseRootMeanSquareResidualCm < 0.0 ||
			Patch.BaseMaximumResidualCm < 0.0 ||
			Patch.CorrectedRootMeanSquareResidualCm < 0.0 ||
			Patch.CorrectedMaximumResidualCm < 0.0 ||
			Patch.CorrectedRootMeanSquareResidualCm >
				Patch.BaseRootMeanSquareResidualCm + 1.0e-9 ||
			Patch.CorrectedMaximumResidualCm >
				Patch.BaseMaximumResidualCm + 1.0e-9 ||
			(Patch.CanonicalGroupId == 0) !=
				(!Patch.bCanonicalC2ByConstruction &&
					!Patch.bCanonicalSymmetryByConstruction) ||
			(Patch.bCanonicalSymmetryByConstruction &&
				!Patch.bCanonicalC2ByConstruction) ||
			Patch.MaximumExtrusionCoordinate <= Patch.MinimumExtrusionCoordinate)
		{
			return Fail(FString::Printf(
				TEXT("Extruded quintic patch record %d is incomplete."), Index));
		}
		for (const FVector& ControlPoint : Patch.SectionControlPoints)
		{
			if (!FMath::IsFinite(ControlPoint.X) ||
				!FMath::IsFinite(ControlPoint.Y) ||
				!FMath::IsFinite(ControlPoint.Z))
			{
				return Fail(FString::Printf(
					TEXT("Extruded quintic patch record %d has a non-finite control point."),
					Index));
			}
		}
		for (const FVector& Correction : Patch.InteriorCorrectionControlPoints)
		{
			if (!FMath::IsFinite(Correction.X) ||
				!FMath::IsFinite(Correction.Y) ||
				!FMath::IsFinite(Correction.Z))
			{
				return Fail(FString::Printf(
					TEXT("Extruded quintic patch record %d has a non-finite correction."),
					Index));
			}
		}
		if (Index > 0 && Patch.PrimitiveId <= PreviousPrimitiveId)
		{
			return Fail(TEXT("Extruded quintic patches are not uniquely sorted."));
		}
		PreviousPrimitiveId = Patch.PrimitiveId;
	}
	return true;
}

TSharedPtr<const Speed::Analytic::FAnalyticWorldData>
USpeedAnalyticCollisionAsset::BuildRuntimeData(
	FString* OutReason, const bool bBuildRecognitionDiagnostics) const
{
	using namespace Speed::Analytic;
	if (!ValidateGeneratedData(OutReason))
	{
		return nullptr;
	}
	TSharedPtr<FAnalyticWorldData> Runtime = MakeShared<FAnalyticWorldData>();
	Runtime->SchemaVersion = SchemaVersion;
	Runtime->SourceHash = SourceHash;
	TMap<uint64, const FSpeedAnalyticMeshSourceRecord*> SourcesById;
	for (const FSpeedAnalyticMeshSourceRecord& Source : MeshSources)
	{
		SourcesById.Add(Source.SourceId, &Source);
	}
	Runtime->Triangles.Reserve(IndexedTriangles.Num());
	for (const FSpeedAnalyticIndexedTriangleRecord& Record : IndexedTriangles)
	{
		const FSpeedAnalyticMeshSourceRecord* const* SourcePtr =
			SourcesById.Find(Record.SourceId);
		if (!SourcePtr || !*SourcePtr)
		{
			if (OutReason)
			{
				*OutReason = TEXT("Triangle references an unknown mesh source.");
			}
			return nullptr;
		}
		const FSpeedAnalyticMeshSourceRecord& Source = **SourcePtr;
		FTriangleSurface& Triangle = Runtime->Triangles.AddDefaulted_GetRef();
		Triangle.SourceId = Record.SourceId;
		Triangle.SurfaceId = Record.SurfaceId;
		Triangle.FeatureId = Record.FeatureId;
		Triangle.PrimitiveId = Record.PrimitiveId;
		Triangle.MaterialId = Record.MaterialId;
		Triangle.ObjectType = Source.ObjectType;
		Triangle.BlockingChannels = Source.BlockingChannels;
		for (int32 Corner = 0; Corner < 3; ++Corner)
		{
			Triangle.Vertices[Corner] = FVector3d(
				VertexPositions[Record.VertexIndices[Corner]]);
			Triangle.VertexNormals[Corner] = FVector3d(
				VertexNormals[Record.NormalIndices[Corner]]).GetSafeNormal();
		}
		Triangle.FaceNormal = FVector3d::CrossProduct(
			Triangle.Vertices[1] - Triangle.Vertices[0],
			Triangle.Vertices[2] - Triangle.Vertices[0]).GetSafeNormal();
		Triangle.Bounds = FBox3d(EForceInit::ForceInit);
		Triangle.Bounds += Triangle.Vertices[0];
		Triangle.Bounds += Triangle.Vertices[1];
		Triangle.Bounds += Triangle.Vertices[2];
		Triangle.bQueryCollisionEnabled = Source.bQueryCollisionEnabled;
		Triangle.bAuthorityEligible = false;
	}
	Runtime->Planes.Reserve(BoundedPlanes.Num());
	for (const FSpeedAnalyticBoundedPlaneRecord& Record : BoundedPlanes)
	{
		FBoundedPlane& Plane = Runtime->Planes.AddDefaulted_GetRef();
		Plane.SourceId = Record.SourceId;
		Plane.SurfaceId = Record.SurfaceId;
		Plane.FeatureId = Record.FeatureId;
		Plane.PrimitiveId = Record.PrimitiveId;
		Plane.MaterialId = static_cast<uint32>(FMath::Max(0, Record.MaterialId));
		Plane.ObjectType = static_cast<uint32>(FMath::Max(0, Record.ObjectType));
		Plane.BlockingChannels = Record.BlockingChannels;
		Plane.Origin = FVector3d(Record.Origin);
		Plane.Normal = FVector3d(Record.Normal);
		Plane.AxisU = FVector3d(Record.AxisU);
		Plane.AxisV = FVector3d(Record.AxisV);
		Plane.HalfExtents = FVector2d(Record.HalfExtents);
		Plane.bQueryCollisionEnabled = Record.bQueryCollisionEnabled;
		Plane.bRequiresCompactOptIn = Record.bRequiresCompactOptIn;
		Plane.bAuthorityEligible = Record.bAuthorityEligible;
	}
	Runtime->ExtrudedQuinticPatches.Reserve(ExtrudedQuinticPatches.Num());
	for (const FSpeedAnalyticExtrudedQuinticPatchRecord& Record :
		ExtrudedQuinticPatches)
	{
		FExtrudedQuinticPatch& Patch =
			Runtime->ExtrudedQuinticPatches.AddDefaulted_GetRef();
		Patch.SourceId = Record.SourceId;
		Patch.SurfaceId = Record.SurfaceId;
		Patch.FeatureId = Record.FeatureId;
		Patch.PrimitiveId = Record.PrimitiveId;
		Patch.CanonicalGroupId = Record.CanonicalGroupId;
		Patch.CanonicalSymmetryAxisMask = Record.CanonicalSymmetryAxisMask;
		Patch.MaterialId = Record.MaterialId;
		Patch.ObjectType = Record.ObjectType;
		Patch.BlockingChannels = Record.BlockingChannels;
		for (int32 ControlIndex = 0; ControlIndex < 6; ++ControlIndex)
		{
			Patch.SectionControlPoints[ControlIndex] =
				FVector3d(Record.SectionControlPoints[ControlIndex]);
		}
		for (int32 CorrectionIndex = 0; CorrectionIndex < 2; ++CorrectionIndex)
		{
			Patch.InteriorCorrectionControlPoints[CorrectionIndex] =
				FVector3d(Record.InteriorCorrectionControlPoints[CorrectionIndex]);
		}
		Patch.BaseRootMeanSquareResidualCm = Record.BaseRootMeanSquareResidualCm;
		Patch.BaseMaximumResidualCm = Record.BaseMaximumResidualCm;
		Patch.CorrectedRootMeanSquareResidualCm =
			Record.CorrectedRootMeanSquareResidualCm;
		Patch.CorrectedMaximumResidualCm = Record.CorrectedMaximumResidualCm;
		Patch.ExtrusionAxis = FVector3d(Record.ExtrusionAxis);
		Patch.MinimumExtrusionCoordinate = Record.MinimumExtrusionCoordinate;
		Patch.MaximumExtrusionCoordinate = Record.MaximumExtrusionCoordinate;
		Patch.Bounds = FBox3d(Record.Bounds);
		Patch.bQueryCollisionEnabled = Record.bQueryCollisionEnabled;
		Patch.bCanonicalC2ByConstruction = Record.bCanonicalC2ByConstruction;
		Patch.bCanonicalSymmetryByConstruction =
			Record.bCanonicalSymmetryByConstruction;
		Patch.bAuthorityEligible = Record.bAuthorityEligible;
	}
	if (!Runtime->FinalizeAndValidate(OutReason))
	{
		return nullptr;
	}
	if (bBuildRecognitionDiagnostics)
	{
		Runtime->BuildRecognitionDiagnostics();
	}
	return Runtime;
}

#if WITH_EDITOR
void USpeedAnalyticCollisionAsset::PreSave(FObjectPreSaveContext SaveContext)
{
	Super::PreSave(SaveContext);
	if (!SaveContext.IsCooking())
	{
		return;
	}

	FString Reason;
	if (!ValidateGeneratedData(&Reason))
	{
		UE_LOG(
			LogSpeedAnalyticCollisionAsset,
			Error,
			TEXT("Analytical collision asset %s is invalid for cook: %s"),
			*GetPathName(),
			*Reason);
	}
}

EDataValidationResult USpeedAnalyticCollisionAsset::IsDataValid(
	FDataValidationContext& Context) const
{
	FString Reason;
	if (!ValidateGeneratedData(&Reason))
	{
		Context.AddError(FText::FromString(Reason));
		return EDataValidationResult::Invalid;
	}
	return EDataValidationResult::Valid;
}
#endif
