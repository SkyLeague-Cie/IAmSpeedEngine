#include "CanonicalFrameDriver.h"

#include "HAL/IConsoleManager.h"

namespace Speed::CanonicalFrameDriver
{
	static constexpr int32 MaxFastFramesPerRun = 100000000;

	static TAutoConsoleVariable<int32> CVarEnabled(
		TEXT("p.IAmSpeed.CanonicalFrame.Enabled"),
		0,
		TEXT("When enabled, ASpeedSimulation owns component preparation and the world step."));

	static TAutoConsoleVariable<int32> CVarFastEnabled(
		TEXT("p.IAmSpeed.FastSimulation.Enabled"),
		0,
		TEXT("Runs one sealed, frame-addressed scenario without per-frame real-time pacing. ")
		TEXT("Requires p.IAmSpeed.CanonicalFrame.Enabled=1 and a canonical run controller."));

	static TAutoConsoleVariable<int32> CVarFastMaxFramesPerRun(
		TEXT("p.IAmSpeed.FastSimulation.MaxFramesPerRun"),
		1000000,
		TEXT("Safety bound for one sealed FastSimulation run; clamped to [1, 100000000]."));

	static TAutoConsoleVariable<int32> CVarFastReadyDelayCallbacks(
		TEXT("p.IAmSpeed.FastSimulation.ReadyDelayCallbacks"),
		2,
		TEXT("Number of Unreal async callbacks yielded without simulating after a scenario is sealed. ")
		TEXT("This lets the current world-query bridge finish scene initialization before Fast owns the run."));

	bool IsEnabled()
	{
		return CVarEnabled.GetValueOnAnyThread() != 0;
	}

	bool IsFastSimulationEnabled()
	{
		return CVarFastEnabled.GetValueOnAnyThread() != 0;
	}

	uint32 GetFastMaxFramesPerRun()
	{
		return static_cast<uint32>(FMath::Clamp(
			CVarFastMaxFramesPerRun.GetValueOnAnyThread(),
			1,
			MaxFastFramesPerRun));
	}

	uint32 GetFastReadyDelayCallbacks()
	{
		return static_cast<uint32>(FMath::Clamp(
			CVarFastReadyDelayCallbacks.GetValueOnAnyThread(),
			0,
			1000));
	}
}
