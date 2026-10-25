#pragma once

#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/Base/Kinematic.h"

class ISpeedComponent;
class USSubBody;
namespace Speed::CollisionMatrix042
{
/** Values copied at the real selected CCD resolution, never a second query. */
struct FResolution
{
    uint64 Frame = 0;
    float TimeWithinFrame = 0;
    SHitResult Hit;
    const ISpeedComponent* Owner = nullptr;
    const ISpeedComponent* Other = nullptr;
    SKinematic Before, After, OtherBefore, OtherAfter;
};

/** Observer is installed only on the joined matrix worker's lane. */
class IAMSPEED_API FObserver
{
public:
    virtual ~FObserver() = default;
    virtual void Resolved(const FResolution& Resolution) = 0;
    static FObserver* Exchange(FObserver* Observer);
    static FResolution Before(uint64 Frame, float TimeWithinFrame,
        const USSubBody& Resolver, const SHitResult& Hit);
    static void After(FResolution& Resolution);
};
}
#endif
