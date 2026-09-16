#pragma once
#include "CoreMinimal.h"

namespace Speed
{
// Rigid correction is needed only when suspension travel cannot separate the
// spherical wheel from the support plane. Force bump-stop onset is not the
// geometric travel limit. This deliberately does not consume suspension state.
inline double WheelSupportTravelPenetration(const FVector& RestCenter,
    const FVector& SuspensionUp, double MaxRaise, double MaxDrop,
    const FVector& ContactPoint, const FVector& UnitNormal, double Radius)
{
    const double Travel = FVector::DotProduct(SuspensionUp, UnitNormal) >= 0
        ? MaxRaise : -MaxDrop;
    const FVector AdmissibleCenter = RestCenter + Travel * SuspensionUp;
    return FVector::DotProduct(ContactPoint + Radius * UnitNormal - AdmissibleCenter, UnitNormal);
}
}
