#pragma once

#include "CoreMinimal.h"


struct SubBodyConfig
{
	bool bValid = false; // Whether this sub-body config is valid and should be used to create a sub-body
	int Index = -1; // Index of the sub-body relevant for the parent component
	float Mass = 0.f; // Mass of the sub-body, used for calculating impulse and forces

	// --- Geometry ---
	float Radius = 0.f; // Radius of the sub-body, used for sphere collision
	FVector BoxExtent = FVector::ZeroVector; // Box extent of the sub-body, used for box collision
	FVector LocalOffset = FVector::ZeroVector; // Local offset of the sub-body from the component's center, used for calculating the sub-body's position in world space
	FQuat LocalRotation = FQuat::Identity; // Local rotation of the sub-body from the component's rotation, used for calculating the sub-body's orientation in world space

	// --- Collision ---
	float Restitution = 0.f; // Restitution coefficient of the sub-body, used for calculating bounce
	float StaticFriction = 0.0f; // Static friction coefficient of the sub-body, used for calculating friction when the sub-body is not moving relative to the other object
	float DynamicFriction = 0.0f; // Dynamic friction coefficient of the sub-body, used for calculating friction when the sub-body is moving relative to the other object
	float ImpactThreshold = 5.0f; // Minimum impact speed required to generate a hit result with non-zero impulse, used to avoid generating hit results for very small impacts that would not be felt by the player
	bool bIsMainSubBody = false; // Whether this sub-body is the main sub-body of the component (e.g. for a car body, the main sub-body would be the hitbox). This is used to determine which sub-body should be used for certain calculations (e.g. applying damage to the car body should use the hitbox sub-body, not the wheel sub-bodies).
	bool bEnableFakePhysics = false; // Whether this sub-body should use fake physics (bump between cars for example) instead of real physics. If true, this sub-body will not generate hit results and will not be affected by forces or impulses, but it will still be able to detect overlaps and trigger events.
	float HitDamping = 0.f; // Damping factor applied to the sub-body's velocity when a hit is detected, used to reduce the sub-body's velocity after a hit and make the impact feel more satisfying. This is only applied if the sub-body is not using fake physics.
	ECollisionChannel CollisionChannel = ECC_WorldDynamic; // Collision channel of the sub-body, used for collision filtering
};
