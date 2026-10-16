#pragma once

#include "IAmSpeed/Base/SHitResult.h"

namespace Speed::Collision
{
// Re-express one box/box contact from the other participant's perspective.
// The shared world point, time and source identity still describe the same event.
inline SHitResult MakeReciprocalBoxContact(const SHitResult& Contact,
    UPrimitiveComponent* SelfComponent, USSubBody* SelfSubBody)
{
    SHitResult InverseHit = Contact;
    InverseHit.ImpactNormal = -Contact.ImpactNormal;
    InverseHit.Component = SelfComponent;
    InverseHit.SubBody = SelfSubBody;
    const auto Point = InverseHit.ContactPointThis;
    InverseHit.ContactPointThis = InverseHit.ContactPointOther;
    InverseHit.ContactPointOther = Point;
    const auto Feature = InverseHit.ContactFeatureThis;
    InverseHit.ContactFeatureThis = InverseHit.ContactFeatureOther;
    InverseHit.ContactFeatureOther = Feature;
    const auto Index = InverseHit.ContactFeatureIndexThis;
    InverseHit.ContactFeatureIndexThis = InverseHit.ContactFeatureIndexOther;
    InverseHit.ContactFeatureIndexOther = Index;
    return InverseHit;
}
}
