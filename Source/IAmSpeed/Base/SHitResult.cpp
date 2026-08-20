#include "IAmSpeed/Base/SHitResult.h"

#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"

namespace Speed
{

SHitResult SHitResult::FromAnalyticHit(
	const Analytic::FWorldHit& Hit, const float Delta,
	UPrimitiveComponent* InComponent, const uint32 InFrameTag)
{
	SHitResult Result;
	if (!Hit.bHit) return Result;
	Result.bHit = true;
	Result.bBlockingHit = true;
	Result.Location = FVector(Hit.Location);
	Result.ImpactPoint = FVector(Hit.Point);
	Result.ImpactNormal = FVector(Hit.Normal);
	Result.TOI = static_cast<float>(Hit.Time * Delta);
	Result.FaceIndex = static_cast<int32>(Hit.PrimitiveId & 0x7fffffffull);
	Result.Component = InComponent;
	Result.bStartPenetrating = Hit.bStartPenetrating;
	Result.PenetrationDepth = static_cast<float>(Hit.PenetrationDepth);
	Result.GeometricErrorBoundCm =
		static_cast<float>(Hit.GeometricErrorBoundCm);
	Result.ContactPointThis = FVector(Hit.QueryPoint);
	Result.ContactPointOther = FVector(Hit.Point);
	Result.ContactFeatureThis = Hit.QueryFeatureKind;
	Result.ContactFeatureOther = Hit.SurfaceFeatureKind;
	Result.ContactFeatureIndexThis = Hit.QueryFeatureIndex;
	Result.ContactFeatureIndexOther = Hit.SurfaceFeatureIndex;
	Result.SourceId = Hit.SourceId;
	Result.SurfaceId = Hit.SurfaceId;
	Result.FeatureId = Hit.FeatureId;
	Result.PrimitiveId = Hit.PrimitiveId;
	Result.MaterialId = Hit.MaterialId;
	Result.FrameTag = InFrameTag;
	return Result;
}

} // namespace Speed
