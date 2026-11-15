#include "BoxWallResponseGeometry.h"
#include "IAmSpeed/Base/SUtils.h"

namespace Speed::BoxWallResponseGeometry
{
FVector SelectNormal(const bool bAnalyticWitness, const FVector& ImpactNormal)
{
    return bAnalyticWitness ? ImpactNormal.GetSafeNormal()
        : Speed::QuantizeUnitNormal(ImpactNormal.GetSafeNormal());
}

FVector SelectPoint(const bool bAnalyticWitness, const FVector& ImpactPoint)
{
    return bAnalyticWitness ? ImpactPoint
        : Speed::QuantizeVectorCm(ImpactPoint, 0.1f);
}

FResponsePointValues BuildResponsePoint(const FVector& Normal, const FVector& Point,
    const FVector& COM, const FQuat& BodyRotation)
{
    FResponsePointValues Result;
    Result.FaceAlignment = FMath::Max(
        FMath::Abs(FVector::DotProduct(BodyRotation.GetAxisX(), Normal)),
        FMath::Max(
            FMath::Abs(FVector::DotProduct(BodyRotation.GetAxisY(), Normal)),
            FMath::Abs(FVector::DotProduct(BodyRotation.GetAxisZ(), Normal))));
    Result.FaceOnPlaneContact = Result.FaceAlignment >= 0.98f;

    // The original face-center branch prevents a sampled corner from turning
    // most of a face-on wall/ceiling rebound into angular motion.
    Result.SolvePoint = Result.FaceOnPlaneContact
        ? COM + Normal * FVector::DotProduct(Point - COM, Normal)
        : Point;
    return Result;
}
}
