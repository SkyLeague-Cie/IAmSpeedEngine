#pragma once
#include "CoreMinimal.h"
class ASpeedPhysicsSimulation;
namespace Speed::Replay
{
// Observes a physical owner call only. It is not a published/committed-frame
// certificate: the recorded ring must later contain this exact endpoint.
class IAMSPEED_API FPhysicalFrameScope
{
    const ASpeedPhysicsSimulation* PreviousDriver;
    uint64 PreviousFrame;
    bool PreviousActive;
public:
    FPhysicalFrameScope(const ASpeedPhysicsSimulation& Driver,uint64 Frame);
    ~FPhysicalFrameScope();
    FPhysicalFrameScope(const FPhysicalFrameScope&)=delete;
    FPhysicalFrameScope& operator=(const FPhysicalFrameScope&)=delete;
};
// Exported function shares the one IAS TLS across Module/Core DLLs. An inline
// header TLS would silently create separate per-module state on Windows.
IAMSPEED_API bool ReadPhysicalOwnerFrame(const ASpeedPhysicsSimulation* Driver,uint64& Frame) noexcept;
}
