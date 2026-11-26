#pragma once
#include "CompletedSimulationPublication.h"

class ASpeedPhysicsSimulation;
namespace Speed
{
// Historical observation of a local native admission. This is NOT host approval,
// current readiness, a completed frame, producer registration or a restore token.
// Only the driver's actual owned input boundary may construct this object.
class FNativeRequestedInputObservation final
{
public:
    FNativeRequestedInputObservation(const FNativeRequestedInputObservation&)=delete;
    FNativeRequestedInputObservation& operator=(const FNativeRequestedInputObservation&)=delete;
    const std::shared_ptr<const FCompletedSimulationSource>& Source() const noexcept {return OwnerSource;}
    const std::shared_ptr<const Input::V2::FInputRegistryView>& RegistryAtBoundary() const noexcept {return Registry;}
    const std::vector<Input::V2::FRequestedNetworkBindingValues>& Bindings() const noexcept {return Records;}
    uint64 BoundaryFrame() const noexcept {return NextFrame;}
    float PhysicalDeltaTime() const noexcept {return FrameDelta;}
private:
    friend class ::ASpeedPhysicsSimulation;
    FNativeRequestedInputObservation(std::shared_ptr<const FCompletedSimulationSource> S,
        std::shared_ptr<const Input::V2::FInputRegistryView> V,
        std::vector<Input::V2::FRequestedNetworkBindingValues> R,uint64 N,float Dt)
        :OwnerSource(std::move(S)),Registry(std::move(V)),Records(std::move(R)),NextFrame(N),FrameDelta(Dt){}
    const std::shared_ptr<const FCompletedSimulationSource> OwnerSource;
    const std::shared_ptr<const Input::V2::FInputRegistryView> Registry;
    const std::vector<Input::V2::FRequestedNetworkBindingValues> Records;
    const uint64 NextFrame;
    const float FrameDelta;
};
}
