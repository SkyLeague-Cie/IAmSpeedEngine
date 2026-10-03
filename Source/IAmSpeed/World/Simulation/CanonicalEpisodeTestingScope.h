#pragma once
#include "CoreMinimal.h"
class ASpeedSimulation;
struct FCanonicalFrameContext;
#if !UE_BUILD_SHIPPING
namespace Speed
{
// Only the canonical driver can establish this thread-local prepare scope.
// This is diagnostic admission metadata, never a scheduling/physics input.
class IAMSPEED_API FCanonicalEpisodeTestingScope
{
 friend class ::ASpeedSimulation;
 FCanonicalEpisodeTestingScope(const FCanonicalFrameContext& Context);
 ~FCanonicalEpisodeTestingScope();
 FCanonicalEpisodeTestingScope(const FCanonicalEpisodeTestingScope&)=delete;
 FCanonicalEpisodeTestingScope& operator=(const FCanonicalEpisodeTestingScope&)=delete;
 uint64 PreviousFrame=0;
 bool PreviousActive=false,PreviousResimulation=false;
public:
 static bool IsOrdinaryPreparation(uint64 Frame);
};
}
#endif
