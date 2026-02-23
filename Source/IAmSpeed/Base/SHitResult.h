#pragma once
#include "CoreMinimal.h"

class UWorld;
class USSubBody;

namespace Speed
{

	struct SHitResult
	{
		SHitResult() = default;
		SHitResult(bool bInBlocking, const FVector& InImpactPoint, const FVector& InImpactNormal, const float& InTOI, UPrimitiveComponent* InComponent = nullptr) :
			bHit(true), bBlockingHit(bInBlocking), ImpactPoint(InImpactPoint), ImpactNormal(InImpactNormal), TOI(InTOI), Component(InComponent)
		{
		}

		bool bHit = false; // true if there is a hit between the two shapes, false otherwise
		bool bBlockingHit = false; // true if the impact point is a blocking hit, false if it's an overlapping hit (for example, if the sphere starts inside the box and goes outside, we want to report a hit but not a blocking hit)
		FVector Location = FVector::ZeroVector; // location of the hit in absolute space, only relevant if bHit is true. For a blocking hit, this is the point where the two shapes would touch. For an overlapping hit, this is the point where the two shapes are the most overlapped (for example, if the sphere starts inside the box and goes outside, this would be the point where the sphere is the deepest inside the box). Note that for an overlapping hit, this point may not be on the surface of either shape.
		FVector ImpactPoint = FVector::ZeroVector; // point of impact in absolute space, only relevant if bHit is true. For a blocking hit, this is the point where the two shapes would touch. For an overlapping hit, this is the point where the two shapes are the most overlapped (for example, if the sphere starts inside the box and goes outside, this would be the point where the sphere is the deepest inside the box). Note that for an overlapping hit, this point may not be on the surface of either shape.
		FVector ImpactNormal = FVector::ZeroVector; // normal at the impact point, pointing from other into THIS object (box, sphere) that asked this SHitResult
		float TOI = 0.0; // time of impact

		TWeakObjectPtr<UPrimitiveComponent> Component = nullptr; // The component that was hit, useful for applying damage or other effects. Only relevant if bHit is true and bBlocking is true (blocking hit).
		TWeakObjectPtr<USSubBody> SubBody = nullptr; // The subbody that was hit, useful for applying damage or other effects. Only relevant if bHit is true and bBlocking is true (blocking hit).

		bool bStartPenetrating = false; // true if the two shapes are already penetrating at the start of the movement, false otherwise. Only relevant if bHit is true and bBlocking is false (overlapping hit).
		float PenetrationDepth = 0.0; // depth of penetration at the impact point, only relevant if bHit is true and bBlocking is false (overlapping hit)
		FVector ContactPointThis = FVector::ZeroVector; // contact point on THIS object (box or sphere) at the impact point, only relevant if bHit is true and bBlocking is false (overlapping hit)
		FVector ContactPointOther = FVector::ZeroVector; // contact point on the OTHER object (box or sphere) at the impact point, only relevant if bHit is true and bBlocking is false (overlapping hit)
		uint32 FrameTag = 0; // Tag to identify the frame in which this hit result was generated, useful for avoiding processing the same hit multiple times in the same frame. Only relevant if bHit is true.

		static SHitResult FromUnrealHit(const FHitResult& UnrealHit, const float& Delta, const uint32& InFrameTag = 0)
		{
			SHitResult ret = SHitResult(UnrealHit.bBlockingHit, UnrealHit.ImpactPoint, UnrealHit.ImpactNormal, UnrealHit.Time * Delta, UnrealHit.Component.Get());
			ret.Location = UnrealHit.Location;
			ret.bStartPenetrating = UnrealHit.bStartPenetrating;
			ret.PenetrationDepth = UnrealHit.PenetrationDepth;
			ret.FrameTag = InFrameTag;
			return ret;
		}
	};
}

using SHitResult = Speed::SHitResult;