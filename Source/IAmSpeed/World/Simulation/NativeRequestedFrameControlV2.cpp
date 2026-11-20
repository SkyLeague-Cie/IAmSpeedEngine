#include "SpeedPhysicsSimulation.h"
#include "CanonicalFrameContext.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"

bool ASpeedPhysicsSimulation::ValidateRequestedV2AtCutoff(const Speed::Input::V2::FSessionDescriptor& D,
    const std::shared_ptr<const Speed::Input::V2::FNativeRequestedBindingV2>& B,uint64 N)const
{
    if(!SimulationWorker||!SimulationWorker->OwnsCurrentCanonicalWork()||!bCanonicalFrameInitialized
        ||!B||!B->OnOwner()||!D.Network||D.Network->Version!=2||!(*D.Network==B->Mapping())
        ||!SpeedWorldSubsystem||!SpeedWorldSubsystem->ValidateRequestedNetworkBindingForFrameV2(D,B))return false;
    const auto Clock=MakeCanonicalFrameContext(N);const auto& Prepared=*B->Seal()->Context();const auto& AtBind=Prepared.Boundary();
    return Clock.NumFrame==N&&N>=D.First&&Clock.LogicalRateNumerator==AtBind.LogicalRateNumerator
        &&Clock.LogicalRateDenominator==AtBind.LogicalRateDenominator&&Clock.FrameOrigin==AtBind.FrameOrigin
        &&Clock.PhysicalDeltaTime==AtBind.PhysicalDeltaTime;
}
bool ASpeedPhysicsSimulation::ValidateRequestedBindingsForFrozenFrame(uint64 N)const
{
    if(!InputSessionRegistry||!SpeedWorldSubsystem)return false;
    const auto V=InputSessionRegistry->ReadRegistry();if(!V||V->Terminal)return false;
    const auto& Token=InputSessionRegistry->FrozenNativeV2;
    if(!InputSessionRegistry->ValidFrozenNativeV2(N,Token))return false;
    for(const auto& D:V->Bindings)if(D.Network)
    {
        if(D.Network->Version==2)
        {if(!Token||!SpeedWorldSubsystem->ValidateRequestedNetworkBindingForFrameV2(D,Token->Find(D.Id,D.Epoch)))return false;}
        else if(!SpeedWorldSubsystem->ValidateRequestedNetworkBinding(D))return false;
    }
    return true;
}
bool ASpeedPhysicsSimulation::ValidateRequestedBindingsAtBoundary()const
{
    if(!SimulationWorker||!SimulationWorker->OwnsCurrentInputBoundary()||!InputSessionRegistry||!SpeedWorldSubsystem)return false;
    const auto V=InputSessionRegistry->ReadRegistry();if(!V||V->Terminal)return false;
    for(const auto& D:V->Bindings)if(D.Network)
    {
        if(D.Network->Version==2)
        {
            const auto I=InputSessionRegistry->Sessions.find(D.Id);
            if(I==InputSessionRegistry->Sessions.end()||!I->second.NativeV2Claimed
                ||!SpeedWorldSubsystem->ValidateRequestedNetworkBindingForFrameV2(D,I->second.NativeV2Binding))return false;
        }
        else if(!SpeedWorldSubsystem->ValidateRequestedNetworkBinding(D))return false;
    }
    return true;
}
