#include "IAmSpeed/Base/SHitResult.h"

#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"

namespace
{
	// A geometric tolerance is an upper bound. A narrowing cast may round down;
	// positive subnormals must also survive hosts using FTZ/DAZ.
	float UpperGeometricError(const double Value)
	{
		uint64 DoubleBits = 0;
		FMemory::Memcpy(&DoubleBits, &Value, sizeof(DoubleBits));
		constexpr uint64 MagnitudeMask = 0x7fffffffffffffffull;
		constexpr uint64 ExponentMask = 0x7ff0000000000000ull;
		const uint64 Magnitude = DoubleBits & MagnitudeMask;
		if ((DoubleBits >> 63) != 0 || Magnitude == 0 ||
			(Magnitude & ExponentMask) == ExponentMask)
			return static_cast<float>(Value); // Preserve existing invalid/zero semantics.
		if (Magnitude < 0x3810000000000000ull) // double(float minimum NORMAL)
		{
			const uint32 MinimumNormalBits = 0x00800000u;
			float MinimumNormal = 0.0f;
			FMemory::Memcpy(&MinimumNormal, &MinimumNormalBits, sizeof(MinimumNormal));
			return MinimumNormal;
		}
		float Bound = static_cast<float>(Value);
		if (static_cast<double>(Bound) < Value)
		{
			uint32 FloatBits = 0;
			FMemory::Memcpy(&FloatBits, &Bound, sizeof(FloatBits));
			++FloatBits; // Next positive float, or +infinity on finite overflow.
			FMemory::Memcpy(&Bound, &FloatBits, sizeof(Bound));
		}
		return Bound;
	}
}

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
		UpperGeometricError(Hit.GeometricErrorBoundCm);
	Result.bSurfaceNormalMayVary = Hit.bSurfaceNormalMayVary;
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
	Result.CanonicalGroupId = Hit.CanonicalGroupId;
	Result.MaterialId = Hit.MaterialId;
	Result.FrameTag = InFrameTag;
	return Result;
}

SHitResult SHitResult::FromAnalyticHitAtSeconds(const Analytic::FWorldHit& Hit,float Seconds,uint32 InFrameTag)
{
    SHitResult Result;if(!Hit.bHit)return Result;
    Result.bHit=true;Result.bBlockingHit=true;Result.Location=FVector(Hit.Location);Result.ImpactPoint=FVector(Hit.Point);
    Result.ImpactNormal=FVector(Hit.Normal);Result.TOI=Seconds; // exact float copy, no normalized-time round trip
    Result.bStartPenetrating=Hit.bStartPenetrating;Result.PenetrationDepth=static_cast<float>(Hit.PenetrationDepth);
    Result.GeometricErrorBoundCm=UpperGeometricError(Hit.GeometricErrorBoundCm);Result.bSurfaceNormalMayVary=Hit.bSurfaceNormalMayVary;
    Result.ContactPointThis=FVector(Hit.QueryPoint);Result.ContactPointOther=FVector(Hit.Point);
    Result.SourceId=Hit.SourceId;Result.SurfaceId=Hit.SurfaceId;Result.FeatureId=Hit.FeatureId;Result.PrimitiveId=Hit.PrimitiveId;
    Result.CanonicalGroupId=Hit.CanonicalGroupId;Result.MaterialId=Hit.MaterialId;Result.FrameTag=InFrameTag;
    // No fabricated triangle index, feature type, component, subbody or native ticket.
    return Result;
}
} // namespace Speed
