#include "AnalyticLandscapeAdapter.h"

#include "Engine/Texture2D.h"
#include "LandscapeComponent.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "LandscapeProxy.h"

namespace Speed::Analytic
{
namespace
{
	int32 AnalyticLandscapeVertexToTexel(
		const int32 Vertex, const int32 SubsectionSizeVerts)
	{
		int32 Subsection = (Vertex - 1) / (SubsectionSizeVerts - 1);
		int32 LocalVertex = (Vertex - 1) % (SubsectionSizeVerts - 1) + 1;
		if (Subsection < 0)
		{
			Subsection = 0;
			LocalVertex = 0;
		}
		return Subsection * SubsectionSizeVerts + LocalVertex;
	}

	bool AppendCookedHeightmapSamples(
		const ULandscapeComponent& Component,
		FFlatLandscapeSource& Source)
	{
		UTexture2D* Texture = Component.GetHeightmap(false);
		FTexturePlatformData* PlatformData = Texture ? Texture->GetPlatformData() : nullptr;
		if (!PlatformData || PlatformData->Mips.IsEmpty() ||
			PlatformData->PixelFormat != PF_B8G8R8A8)
		{
			return false;
		}

		FTexture2DMipMap& Mip = PlatformData->Mips[0];
		const int64 ExpectedBytes =
			static_cast<int64>(Mip.SizeX) * static_cast<int64>(Mip.SizeY) * sizeof(FColor);
		if (Mip.BulkData.GetBulkDataSize() < ExpectedBytes)
		{
			return false;
		}
		const FColor* Pixels = static_cast<const FColor*>(Mip.BulkData.LockReadOnly());
		if (!Pixels)
		{
			Mip.BulkData.Unlock();
			return false;
		}

		const int32 OffsetX = FMath::RoundToInt(Mip.SizeX * Component.HeightmapScaleBias.Z);
		const int32 OffsetY = FMath::RoundToInt(Mip.SizeY * Component.HeightmapScaleBias.W);
		const int32 SubsectionSizeVerts = Component.SubsectionSizeQuads + 1;
		const int32 ComponentSizeVerts = Component.ComponentSizeQuads + 1;
		const FTransform ComponentToWorld = Component.GetComponentTransform();
		bool bValid = true;
		for (int32 Y = 0; Y < ComponentSizeVerts && bValid; ++Y)
		{
			const int32 TexelY = OffsetY +
				AnalyticLandscapeVertexToTexel(Y, SubsectionSizeVerts);
			for (int32 X = 0; X < ComponentSizeVerts; ++X)
			{
				const int32 TexelX = OffsetX +
					AnalyticLandscapeVertexToTexel(X, SubsectionSizeVerts);
				if (TexelX < 0 || TexelX >= Mip.SizeX ||
					TexelY < 0 || TexelY >= Mip.SizeY)
				{
					bValid = false;
					break;
				}
				const FColor& Sample = Pixels[TexelY * Mip.SizeX + TexelX];
				const uint16 PackedHeight =
					(static_cast<uint16>(Sample.R) << 8) | Sample.G;
				const double LocalHeight =
					(static_cast<double>(PackedHeight) - 32768.0) / 128.0;
				const FVector WorldVertex = ComponentToWorld.TransformPosition(
					FVector(X, Y, LocalHeight));
				Source.WorldBounds += FVector3d(WorldVertex);
				Source.WorldHeights.Add(WorldVertex.Z);
			}
		}
		Mip.BulkData.Unlock();
		return bValid;
	}

	bool AppendCookedCollisionMetadataSamples(
		const ALandscapeProxy& Landscape,
		FFlatLandscapeSource& Source)
	{
		if (Landscape.CollisionComponents.IsEmpty())
		{
			return false;
		}
		Source.bHoleCoverageValidated = true;
		for (const ULandscapeHeightfieldCollisionComponent* Component :
			Landscape.CollisionComponents)
		{
			if (!Component || !Component->CachedLocalBox.IsValid)
			{
				return false;
			}
			for (const uint8 QuadFlags : Component->CollisionQuadFlags)
			{
				if ((QuadFlags &
					ULandscapeHeightfieldCollisionComponent::QF_NoCollision) != 0)
				{
					Source.bContainsHoles = true;
				}
			}

			const FBox& LocalBounds = Component->CachedLocalBox;
			const FTransform ComponentToWorld = Component->GetComponentTransform();
			const double LocalHalfHeight = LocalBounds.GetExtent().Z;
			const double LocalMidHeight = LocalBounds.GetCenter().Z;
			// ULandscapeComponent::UpdateCachedBounds expands an exactly-flat
			// component by precisely one local unit above and below its real
			// height so that its render bounds have non-zero volume. Preserve the
			// midpoint as the height sample only for that exact cooked signature.
			// Any other Z extent represents actual relief (or conservative invalid
			// metadata), so retain both extrema and let the pure adapter reject it.
			const bool bHasFlatCookedBoundsSignature =
				FMath::IsNearlyEqual(LocalHalfHeight, 1.0, 1.0e-9);
			const int32 HeightSampleCount = bHasFlatCookedBoundsSignature ? 1 : 2;
			for (int32 ZIndex = 0; ZIndex < HeightSampleCount; ++ZIndex)
			{
				const double Z = bHasFlatCookedBoundsSignature
					? LocalMidHeight
					: (ZIndex == 0 ? LocalBounds.Min.Z : LocalBounds.Max.Z);
				for (int32 YIndex = 0; YIndex < 2; ++YIndex)
				{
					const double Y = YIndex == 0 ? LocalBounds.Min.Y : LocalBounds.Max.Y;
					for (int32 XIndex = 0; XIndex < 2; ++XIndex)
					{
						const double X = XIndex == 0 ? LocalBounds.Min.X : LocalBounds.Max.X;
						const FVector WorldVertex = ComponentToWorld.TransformPosition(
							FVector(X, Y, Z));
						Source.WorldBounds += FVector3d(WorldVertex);
						Source.WorldHeights.Add(WorldVertex.Z);
					}
				}
			}
		}
		return !Source.WorldHeights.IsEmpty();
	}
}

FFlatLandscapeAdapterOutput BuildFlatLandscapePlane(
	const FFlatLandscapeSource& Source,
	const double FlatnessToleranceCm)
{
	FFlatLandscapeAdapterOutput Output;
	if (Source.SourceId == 0 || Source.WorldHeights.IsEmpty())
	{
		Output.Diagnostic = TEXT("Landscape source has no stable id or height samples.");
		return Output;
	}
	if (!Source.WorldBounds.IsValid ||
		Source.WorldBounds.Max.X <= Source.WorldBounds.Min.X ||
		Source.WorldBounds.Max.Y <= Source.WorldBounds.Min.Y)
	{
		Output.Result = EFlatLandscapeAdapterResult::InvalidBounds;
		Output.Diagnostic = TEXT("Landscape source has invalid XY bounds.");
		return Output;
	}
	if (Source.bContainsHoles)
	{
		Output.Result = EFlatLandscapeAdapterResult::UnsupportedHoles;
		Output.Diagnostic = TEXT(
			"Flat Landscape contains collision holes; the bounded-plane milestone refuses it.");
		return Output;
	}

	const double ReferenceHeight = Source.WorldHeights[0];
	if (!FMath::IsFinite(ReferenceHeight))
	{
		Output.Result = EFlatLandscapeAdapterResult::NonFiniteHeight;
		Output.Diagnostic = TEXT("Landscape source contains a non-finite height.");
		return Output;
	}
	for (const double Height : Source.WorldHeights)
	{
		if (!FMath::IsFinite(Height))
		{
			Output.Result = EFlatLandscapeAdapterResult::NonFiniteHeight;
			Output.Diagnostic = TEXT("Landscape source contains a non-finite height.");
			return Output;
		}
		Output.MaximumHeightResidual = FMath::Max(
			Output.MaximumHeightResidual, FMath::Abs(Height - ReferenceHeight));
	}
	if (Output.MaximumHeightResidual > FMath::Max(0.0, FlatnessToleranceCm))
	{
		Output.Result = EFlatLandscapeAdapterResult::NonFlat;
		Output.Diagnostic = FString::Printf(
			TEXT("Landscape residual %.9g cm exceeds flatness tolerance %.9g cm."),
			Output.MaximumHeightResidual, FlatnessToleranceCm);
		return Output;
	}

	Output.Plane.SurfaceId = Source.SourceId;
	Output.Plane.FeatureId = CombineStableIds(
		Source.SourceId, static_cast<uint64>(EFeatureKind::Interior) + 1ull);
	Output.Plane.Origin = FVector3d(
		0.5 * (Source.WorldBounds.Min.X + Source.WorldBounds.Max.X),
		0.5 * (Source.WorldBounds.Min.Y + Source.WorldBounds.Max.Y),
		ReferenceHeight);
	Output.Plane.Normal = FVector3d::UpVector;
	Output.Plane.AxisU = FVector3d::ForwardVector;
	Output.Plane.AxisV = FVector3d::RightVector;
	Output.Plane.HalfExtents = FVector2d(
		0.5 * (Source.WorldBounds.Max.X - Source.WorldBounds.Min.X),
		0.5 * (Source.WorldBounds.Max.Y - Source.WorldBounds.Min.Y));
	// Material ownership is not yet imported. Even when hole flags were fully
	// validated, this first adapter is safe only for shadow comparison.
	Output.Plane.bAuthorityEligible = false;
	Output.Result = EFlatLandscapeAdapterResult::SuccessShadowOnly;
	Output.Diagnostic = TEXT(
		"Flat Landscape imported for shadow mode; holes/material ownership remain unvalidated.");
	return Output;
}

FFlatLandscapeAdapterOutput BuildFlatLandscapePlane(
	const ALandscapeProxy& Landscape,
	const double FlatnessToleranceCm)
{
	const FVector3d LandscapeUp = FVector3d(
		Landscape.GetActorTransform().TransformVectorNoScale(FVector::UpVector));
	if (!LandscapeUp.GetSafeNormal().Equals(FVector3d::UpVector, 1.0e-9))
	{
		FFlatLandscapeAdapterOutput Output;
		Output.Result = EFlatLandscapeAdapterResult::UnsupportedTransform;
		Output.Diagnostic = TEXT(
			"The first flat-Landscape adapter requires a gravity-aligned Landscape.");
		return Output;
	}

	FFlatLandscapeSource Source;
	Source.SourceId = StableStringId(Landscape.GetPathName());
	bool bReadRenderHeightmap = false;
	for (ULandscapeComponent* Component : Landscape.LandscapeComponents)
	{
		if (!Component)
		{
			continue;
		}
		if (!AppendCookedHeightmapSamples(*Component, Source))
		{
			bReadRenderHeightmap = false;
			Source.WorldBounds = FBox3d(EForceInit::ForceInit);
			Source.WorldHeights.Reset();
			break;
		}
		bReadRenderHeightmap = true;
	}
	if (!bReadRenderHeightmap)
	{
		if (!AppendCookedCollisionMetadataSamples(Landscape, Source))
		{
			FFlatLandscapeAdapterOutput Output;
			Output.Diagnostic = TEXT(
				"Landscape cook metadata is unavailable at adapter time.");
			return Output;
		}
	}

	FFlatLandscapeAdapterOutput Output =
		BuildFlatLandscapePlane(Source, FlatnessToleranceCm);
	return Output;
}

} // namespace Speed::Analytic
