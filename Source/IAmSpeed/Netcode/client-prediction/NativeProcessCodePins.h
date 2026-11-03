#pragma once
#include <cstdint>
#include <memory>
#include <vector>

namespace Speed::Netcode::Prediction
{
enum class EProcessCodePinStatus : std::uint8_t
{
    UnsupportedPlatform, SelfModuleUnavailable, EnumerationFailed,
    InvalidEnumeration, BoundExceeded, DuplicateModule, SelfModuleMissing,
    PinFailed, RosterChanged, PinnedBaseSetAtPinTimes
};
struct FProcessCodePinRecord
{
    std::uintptr_t EnumeratedBase=0, ReturnedBase=0;
    std::uint32_t Error=0;
    bool Attempted=false, PinReturned=false;
};
// Reports REAL Windows process-lifetime loader pins. The report owns no
// unpin operation: FLAG_PIN is held by Windows until this actual process exits.
// Enumeration provides unretained bases. Even two equal rosters and equal
// bases do NOT establish the incarnation seen at the first enumeration. Pins
// retain the actual modules returned at their PIN calls only. Pre-existing
// callback/object provenance and initial incarnations remain UNKNOWN.
// This is only code retention at the actual PIN calls, NOT source,
// config, escaped-view exclusion, executor/worker join or restoration authority.
// Use only in explicitly selected finite private processes, never a shared Editor.
class IAMSPEED_API FNativeProcessCodePins final
{
    FNativeProcessCodePins()=default;
    EProcessCodePinStatus Outcome=EProcessCodePinStatus::UnsupportedPlatform;
    std::uint32_t Process=0, FailureError=0;
    std::uintptr_t Self=0;
    std::vector<FProcessCodePinRecord> Observations;
public:
    FNativeProcessCodePins(const FNativeProcessCodePins&)=delete;
    FNativeProcessCodePins& operator=(const FNativeProcessCodePins&)=delete;
    // No caller-provided handle, module address, receipt or permission boolean.
    // Failure can leave some genuine pins held; observations preserve attempts.
    static std::shared_ptr<const FNativeProcessCodePins> PinObservedProcessModules();
    EProcessCodePinStatus Status() const noexcept {return Outcome;}
    std::uint32_t OriginalProcess() const noexcept {return Process;}
    std::uint32_t OriginalError() const noexcept {return FailureError;}
    std::uintptr_t ActualSelfModule() const noexcept {return Self;}
    const std::vector<FProcessCodePinRecord>& Records() const noexcept {return Observations;}
};
}
