#pragma once

#include "CoreMinimal.h"
#include "CollisionShape.h"

class UWorld;
class USpeedWorldSubsystem;
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
	static bool IsCompactAuthorityEnabled();
	static bool ShouldBuildAnalyticWorld();

	static bool TryCompactAuthoritySingle(
		const FVector& Start, const FVector& End, const FQuat& Rotation,
		const FCollisionShape& Shape, uint8 TraceChannel,
		const FCollisionResponseParams& ResponseParams,
		FHitResult& OutHit, bool& bOutHit);
	static bool TryCompactAuthorityMulti(
		const FVector& Start, const FVector& End, const FQuat& Rotation,
		const FCollisionShape& Shape, uint64 ObjectTypes,
		TArray<FHitResult>& OutHits);

	static void BeginFrame(uint64 Frame, const FAnalyticWorldData* WorldData,
		const USpeedWorldSubsystem* RuntimeBridge);
	static void EndFrame();
	static void RecordLegacySweep();

	static void RecordSingle(
		EStaticQuerySite Site,
		const FVector& Start,
		const FVector& End,
		const FQuat& Rotation,
		const FCollisionShape& Shape,
		uint8 TraceChannel,
		const FCollisionResponseParams& ResponseParams,
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
