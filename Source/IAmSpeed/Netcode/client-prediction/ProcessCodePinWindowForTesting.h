#pragma once
#if WITH_DEV_AUTOMATION_TESTS
#include <atomic>
#include <cstdint>
namespace Speed::Netcode::Prediction::Testing
{
// Finite Windows-loader experiment only. Real event handles synchronize an
// actual LoadLibrary/FreeLibrary thread before the next actual PIN attempt.
// No API result, module roster, owner receipt or lifetime proof is supplied.
struct FProcessPinWindowControl
{
    std::uintptr_t InputCapturedEvent=0,ContinueEvent=0;
    std::uint32_t WaitMilliseconds=2000;
    std::atomic<std::uint32_t> ActualPass{0};
};
class IAMSPEED_API FScopedProcessPinWindowForTesting final
{
    FProcessPinWindowControl* Bound=nullptr;
    bool Installed=false;
public:
    explicit FScopedProcessPinWindowForTesting(FProcessPinWindowControl&) noexcept;
    ~FScopedProcessPinWindowForTesting();
    bool IsInstalled() const noexcept {return Installed;} // diagnostic, NOT permission
    FScopedProcessPinWindowForTesting(const FScopedProcessPinWindowForTesting&)=delete;
    FScopedProcessPinWindowForTesting& operator=(const FScopedProcessPinWindowForTesting&)=delete;
};
}
#endif
