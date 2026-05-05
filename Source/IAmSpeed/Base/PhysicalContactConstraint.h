#pragma once

#include "CoreMinimal.h"

class UPrimitiveComponent;
class USSubBody;

struct IAMSPEED_API FPhysicalContactConstraint
{
	// Contact Normal in world space, points out of the obstacle. For example, if a sphere hits the ground, the normal would point up
	FVector Normal = FVector::ZeroVector;
	// Contact Point in world space, the point on the surface of the obstacle that is in contact with the sub-body.
	FVector ContactPoint = FVector::ZeroVector;
	// The component that is in contact with the sub-body. For example, if a sphere hits the ground, the other component would be the ground component
	TWeakObjectPtr<UPrimitiveComponent> OtherComponent = nullptr;
	// The sub-body that is in contact with the other component. For example, if a sphere hits the ground, the source sub-body would be the sphere sub-body
	TWeakObjectPtr<USSubBody> SourceSubBody = nullptr;
	// Penetration depth at the contact point. For example, if a sphere is penetrating the ground by 5 cm, the penetration depth would be 5 cm.
	float PenetrationDepth = 0.0f;
	// Time of impact, between 0 and 1, where 0 means the beginning of the frame and 1 means the end of the frame.
	// This is useful for determining the order of contacts and for performing sub-stepping if necessary.
	float TOI = 0.0f;
	uint64 PairKey = 0;
	// Whether this contact is persistent, meaning it was already present in the previous frame. This is useful for stability and filtering, as we can treat new contacts differently from persistent contacts.
	uint8 FramesSinceSeen = 0;
	// Whether this contact should be treated as a persistent support constraint.
	// This is useful for implementing the “keep rear edge down” constraint, where we want to maintain a stable contact
	// with the ground even when the normal velocity is zero or slightly positive.
	bool bPersistent = false;

	bool IsValid() const
	{
		return !Normal.IsNearlyZero() && OtherComponent.IsValid();
	}
};
