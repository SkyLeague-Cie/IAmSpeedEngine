#pragma once
#include "IAmSpeed/Netcode/client-prediction/NativeProcessCodePins.h"
#include "IAmSpeed/World/Simulation/FrameParallelExecutor.h"
#include <memory>
#include <string>

namespace Speed::Netcode
{
// Internal finite-host participant, NOT an ISimulationRestoreBoundaryLease.
// Declare this BEFORE all dependent fields, so its real release runs last.
class FNativeFiniteProcessRetention final
{
    const std::string ReportBase;
    const std::shared_ptr<const Prediction::FNativeProcessCodePins> Code;
    std::unique_ptr<FFrameExecutorServiceRetention> Service;
    FFrameExecutorServiceObservation Before,Held;
    FNativeFiniteProcessRetention(std::string,std::shared_ptr<const Prediction::FNativeProcessCodePins>);
    bool Write(const char*,const FFrameExecutorServiceObservation&) const;
public:
    ~FNativeFiniteProcessRetention();
    FNativeFiniteProcessRetention(const FNativeFiniteProcessRetention&)=delete;
    FNativeFiniteProcessRetention& operator=(const FNativeFiniteProcessRetention&)=delete;
    static std::unique_ptr<FNativeFiniteProcessRetention> Create(const std::string&,std::uint32_t,std::string&);
};
}
