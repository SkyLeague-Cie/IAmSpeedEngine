#pragma once

#include "CoreMinimal.h"
#include "CollisionShape.h"

class UWorld;
class USpeedWorldSubsystem;
struct FHitResult;

namespace Speed::Analytic
{
struct FAnalyticWorldData;
struct FWorldHit;

struct FStaticWorldQueryCounters
{
	uint32 QueryCount = 0;
	uint32 LegacySweepCount = 0;
	uint32 AuthorityAttemptCount = 0;
	uint32 AuthorityCoveredCount = 0;
	uint32 AuthorityFallbackCount = 0;
	uint32 StrictMissingWorldCount = 0;
	// Wall-clock time consumed by the generic analytical authority path.  This
	// includes a Hybrid coverage comparison when one is required, so a fast
	// simulation report can distinguish solver work from static-world queries.
	double AuthorityMilliseconds = 0.0;
	double MaximumAuthorityMilliseconds = 0.0;
};

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
	BoxPersistentSupportProbe,
};

class IAMSPEED_API FStaticWorldQueryAudit
{
public:
	static bool IsEnabled();
	static bool IsShadowEnabled();
	static bool IsCompactAuthorityEnabled();
	static bool IsSurfaceAnalyticBackend();
	static bool IsAuthorityChaosShadowEnabled();
	static bool ShouldBuildAnalyticWorld();
	/** Records a proven empty single query without constructing geometry. Returns false
	 * when the ordinary authority/audit path is required; the caller must then use it.
	 * On true, reset the caller's miss output and do not call RecordSingle again. */
	static bool TryRecordEmptySingle(
		UWorld* World, uint8 TraceChannel, const FCollisionResponseParams& ResponseParams);

	static bool TryCompactAuthoritySingle(
		UWorld* World,
		const FVector& Start, const FVector& End, const FQuat& Rotation,
		const FCollisionShape& Shape, uint8 TraceChannel,
		const FCollisionResponseParams& ResponseParams,
		FHitResult& OutHit, bool& bOutHit,
		FWorldHit* OutAnalyticHit = nullptr,
		uint64 RequiredSourceId = 0,
		uint64 RequiredSurfaceId = 0,
		uint64 RequiredCanonicalGroupId = 0,
		const FVector& ReferenceNormal = FVector::ZeroVector,
		float MinimumReferenceNormalDot = -1.0f,
		bool bAllowEstablishedFaceContactAtBoundary = false);
	static bool TryCompactAuthorityMulti(
		UWorld* World,
		const FVector& Start, const FVector& End, const FQuat& Rotation,
		const FCollisionShape& Shape, uint64 ObjectTypes,
		TArray<FHitResult>& OutHits,
		FWorldHit* OutAnalyticHit = nullptr);

	static void BeginFrame(uint64 Frame, const FAnalyticWorldData* WorldData,
		const USpeedWorldSubsystem* RuntimeBridge);
	static FStaticWorldQueryCounters GetCurrentFrameCounters();
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
		const FHitResult& UnrealHit,
		uint64 RequiredSourceId = 0,
		uint64 RequiredSurfaceId = 0,
		uint64 RequiredCanonicalGroupId = 0);

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
