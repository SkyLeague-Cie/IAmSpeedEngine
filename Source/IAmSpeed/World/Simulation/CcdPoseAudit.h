#pragma once
#include "CoreMinimal.h"
#if WITH_DEV_AUTOMATION_TESTS
class USpeedWorldSubsystem;
namespace Speed::Testing
{
// Development-only observation; never canonical state or a collision authority.
// These are actual solver endpoints, not a certificate between endpoints.
enum class ECcdPoseAuditStage : uint8
{
    FrameBegin, AfterIntegrate, BeforeResolve, AfterResolve, AfterProjection, AfterPost
};
struct FCcdPoseAuditSample
{
    const USpeedWorldSubsystem* World;
    uint32 Frame;
    ECcdPoseAuditStage Stage;
    float TimePassed, SubDelta;
};
using FCcdPoseAuditObserver = void (*)(void*, const FCcdPoseAuditSample&) noexcept;
IAMSPEED_API bool SetCcdPoseAuditObserverForCurrentThread(FCcdPoseAuditObserver Observer, void* Context) noexcept;
IAMSPEED_API bool ClearCcdPoseAuditObserverForCurrentThread(FCcdPoseAuditObserver Observer, void* Context) noexcept;
}
#endif
