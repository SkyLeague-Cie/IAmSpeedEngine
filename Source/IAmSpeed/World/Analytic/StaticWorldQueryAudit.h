#pragma once

#include "CoreMinimal.h"
#include "CollisionShape.h"

class UWorld;
struct FHitResult;

namespace Speed::Analytic
{
struct FAnalyticWorldData;

enum class EStaticQuerySite : uint8
{
	SolidSweep = 0,
	SensorSweep,
	SubBodySweep,
	WheelSuspensionProbe,
	WheelEstablishedSupportProbe,
	RayWheelSuspensionProbe,
	RayWheelEstablishedSupportProbe,
	BoxPenetrationProjection,
	SpherePenetrationProjection,
	SpherePenetrationResidual,
};

class IAMSPEED_API FStaticWorldQueryAudit
{
public:
	static bool IsEnabled();
	static bool IsShadowEnabled();
	static bool ShouldBuildAnalyticWorld();

	static void BeginFrame(uint64 Frame, const FAnalyticWorldData* WorldData);
	static void EndFrame();

	static void RecordSingle(
		EStaticQuerySite Site,
		const FVector& Start,
		const FVector& End,
		const FQuat& Rotation,
		const FCollisionShape& Shape,
		uint8 TraceChannel,
		bool bHit,
		const FHitResult& UnrealHit);

	static void RecordMulti(
		EStaticQuerySite Site,
		const FVector& Start,
		const FVector& End,
		const FQuat& Rotation,
		const FCollisionShape& Shape,
		uint64 ObjectTypes,
		const TArray<FHitResult>& UnrealHits);
};

} // namespace Speed::Analytic
