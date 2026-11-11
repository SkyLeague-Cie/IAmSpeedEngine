#pragma once
#include "CoreMinimal.h"

namespace Speed::BoxWallResponseGeometry
{
// Shared arithmetic of the actual wall/gutter resolver. These operations
// consume an existing hit; they do not select a collision point/time/normal,
// certify its accuracy, or grant a prepared integration/response permission.
IAMSPEED_API FVector SelectNormal(bool bAnalyticWitness, const FVector& ImpactNormal);
IAMSPEED_API FVector SelectPoint(bool bAnalyticWitness, const FVector& ImpactPoint);

struct FResponsePointValues
{
    float FaceAlignment = 0;
    bool FaceOnPlaneContact = false;
    FVector SolvePoint = FVector::ZeroVector;
};

// BodyRotation is the actual subbody rotation; COM is the actual GetCOM()
// value. Preserve the existing float alignment and >=0.98f decision exactly.
IAMSPEED_API FResponsePointValues BuildResponsePoint(const FVector& Normal,
    const FVector& Point, const FVector& COM, const FQuat& BodyRotation);
}
