#pragma once
#include <atomic>
#include <cstdint>
namespace Speed::Netcode::Prediction
{
// Actual C++ constructor/destructor return-site observations only. These
// counters do not certify an Engine/source worker join or a restore lease.
struct FNativeConfigurationLifetimeValues
{
    std::uint64_t NativeConstructionReturns=0,NativeBaseDestructionReturns=0,ConfigurationDestructionEntries=0;
    bool ConfigurationDestructionBeforeNativeReturn=false;
};
struct FNativeConfigurationLifetimeTrace final
{
    std::atomic<std::uint64_t> NativeConstructionReturns{0},NativeBaseDestructionReturns{0},ConfigurationDestructionEntries{0};
    std::atomic<bool> ConfigurationDestructionBeforeNativeReturn{false};
    FNativeConfigurationLifetimeValues Read() const
    {return {NativeConstructionReturns.load(),NativeBaseDestructionReturns.load(),ConfigurationDestructionEntries.load(),ConfigurationDestructionBeforeNativeReturn.load()};}
};
}
