#include "IAmSpeed.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"

#if PLATFORM_LINUX && WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "Linux/LinuxPlatformApplicationMisc.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#endif

DEFINE_LOG_CATEGORY(SpeedPhysicsLog);

void IAmSpeed::StartupModule()
{
#if PLATFORM_LINUX && WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
    // NullRHI skips ApplicationCore's normal SDL initialization. Slate automation
    // notifications still query the video backend, so establish its real lifecycle
    // before those tests run. ApplicationCore retains ownership of SDL teardown.
    if (FParse::Param(FCommandLine::Get(), TEXT("SLPortableHeadlessSDL")))
    {
        FString VideoDriver;
        if (!FParse::Param(FCommandLine::Get(), TEXT("NullRHI")) ||
            !FParse::Value(FCommandLine::Get(), TEXT("-sdlvideodriver="), VideoDriver) ||
            !VideoDriver.Equals(TEXT("dummy"), ESearchCase::IgnoreCase))
        {
            UE_LOG(SpeedPhysicsLog, Fatal,
                TEXT("SLPortableHeadlessSDL requires -NullRHI -sdlvideodriver=dummy"));
            return;
        }
        if (!FLinuxPlatformApplicationMisc::InitSDL())
        {
            UE_LOG(SpeedPhysicsLog, Fatal, TEXT("Headless automation SDL initialization failed"));
            return;
        }
        UE_LOG(SpeedPhysicsLog, Log, TEXT("Headless automation SDL lifecycle initialized"));
    }
#endif
}

void IAmSpeed::ShutdownModule()
{
    Speed::ShutdownFrameParallelExecutor();
}

IMPLEMENT_MODULE(IAmSpeed, IAmSpeed)
