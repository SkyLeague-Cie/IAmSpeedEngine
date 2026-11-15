#pragma once
#include "CoreMinimal.h"

#if WITH_EDITOR && !UE_BUILD_SHIPPING
class USpeedAnalyticCollisionAsset;
namespace Speed::Analytic::Prepared { class FScene; }
namespace Speed
{
// Only values cross the module boundary. The game factory runs synchronously
// on GT during construction. Published worlds retain no callback or UObject.
using FOrdinaryC2ScenePtr=TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe>;
using FOrdinaryC2SceneFactory=FOrdinaryC2ScenePtr(*)(const USpeedAnalyticCollisionAsset&,FString&);
IAMSPEED_API bool RegisterOrdinaryC2SceneFactory(FOrdinaryC2SceneFactory Factory);
IAMSPEED_API void UnregisterOrdinaryC2SceneFactory(FOrdinaryC2SceneFactory Factory);

struct FOrdinaryC2DiagnosticScene
{
    FOrdinaryC2ScenePtr Nominal;
    uint64 SceneGeneration=0;
    uint64 FinalSourceHash=0;
    // This plane remains in the ordinary world. A future consumer must
    // separately prove it clear. No query is authorized by this sidecar.
    uint64 ExternalLandscapePrimitive=0;
};
}
#endif
