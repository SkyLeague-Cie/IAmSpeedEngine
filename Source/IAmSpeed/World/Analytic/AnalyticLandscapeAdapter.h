#pragma once

#include "AnalyticWorldData.h"

class ALandscapeProxy;

namespace Speed::Analytic
{

enum class EFlatLandscapeAdapterResult : uint8
{
	SuccessShadowOnly = 0,
	NoSourceData,
	NonFiniteHeight,
	NonFlat,
	InvalidBounds,
	UnsupportedTransform,
	UnsupportedHoles,
};

struct IAMSPEED_API FFlatLandscapeSource
{
	uint64 SourceId = 0;
	FBox3d WorldBounds = FBox3d(EForceInit::ForceInit);
	TArray<double> WorldHeights;
	bool bHoleCoverageValidated = false;
	bool bContainsHoles = false;
};

struct IAMSPEED_API FFlatLandscapeAdapterOutput
{
	EFlatLandscapeAdapterResult Result = EFlatLandscapeAdapterResult::NoSourceData;
	FBoundedPlane Plane;
	double MaximumHeightResidual = 0.0;
	FString Diagnostic;
};

IAMSPEED_API FFlatLandscapeAdapterOutput BuildFlatLandscapePlane(
	const FFlatLandscapeSource& Source,
	double FlatnessToleranceCm);

// Unreal is an import adapter only. Its output is plain immutable runtime data;
// analytical queries never retain or dereference the Landscape actor.
IAMSPEED_API FFlatLandscapeAdapterOutput BuildFlatLandscapePlane(
	const ALandscapeProxy& Landscape,
	double FlatnessToleranceCm);

} // namespace Speed::Analytic
