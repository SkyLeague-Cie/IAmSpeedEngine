#include "ExactRemoteDedicatedHost.h"
#include "Engine/World.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameDriver.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"

Speed::Netcode::EExactRemoteHostPolicy Speed::Netcode::QueryExactRemoteHostPolicy(const UWorld* World)
{
    check(IsInGameThread());
    if (!FParse::Param(FCommandLine::Get(), TEXT("SpeedExactRemoteDedicatedHost"))) return EExactRemoteHostPolicy::Legacy;
    if (!World || !World->IsGameWorld() || World->GetNetMode() != NM_DedicatedServer
        || !CanonicalFrameDriver::IsEnabled()
        || CanonicalFrameDriver::GetExecutionMode() != ESimulationExecutionMode::IAmSpeedThread
        || CanonicalFrameDriver::IsFastSimulationEnabled() || ASpeedPhysicsSimulation::GetEngineFPS() != 300)
        return EExactRemoteHostPolicy::Invalid;
    return EExactRemoteHostPolicy::IsolatedDedicated;
}
