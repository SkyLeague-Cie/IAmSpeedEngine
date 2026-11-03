#pragma once
#include <cstdint>
#include <cstddef>
#include <array>
#include <memory>
#include <vector>

namespace Speed::Netcode::Prediction
{
enum class EProcessCodePinStatus : std::uint8_t
{
    UnsupportedPlatform, SelfModuleUnavailable, EnumerationFailed,
    InvalidEnumeration, BoundExceeded, DuplicateModule, SelfModuleMissing,
    PinFailed, RosterChanged, PinnedBaseSetAtPinTimes, WindowControlFailed
};
struct FProcessCodePinRecord
{
    std::uintptr_t EnumeratedBase=0, ReturnedBase=0;
    std::uint32_t Error=0;
    bool Attempted=false, PinReturned=false;
};
// Diagnostic bounds, not source/lifetime permission. All slots are allocated
// before the first irreversible PIN; each actual module can be PINned once.
inline constexpr std::size_t MaximumProcessPinModules=1024;
inline constexpr std::size_t MaximumProcessPinPasses=4;
enum class EProcessPinTransition : std::uint8_t
{ NotObserved, Equal, StrictExpansion, MissingPinnedBase, BudgetExhausted, InvalidEnumeration };
struct FProcessCodePinPass
{
    bool InputIsInitial=true, InputSucceeded=false, AfterSucceeded=false;
    std::uint32_t InputAfterPass=0,InputBytes=0,AfterBytes=0;
    std::uint32_t InputAvailable=0,AfterAvailable=0,InputError=0,AfterError=0;
    std::uint32_t NewPinAttempts=0;
    // Test-only kernel event observations, never a source/restore capability.
    bool WindowInstalled=false,WindowSignaled=false,WindowWaitReturned=false;
    std::uint32_t WindowWaitResult=0,WindowSignalError=0,WindowWaitError=0;
    EProcessPinTransition Transition=EProcessPinTransition::NotObserved;
    std::array<std::uintptr_t,MaximumProcessPinModules> InputBases{},AfterBases{};
};
// Reports REAL Windows process-lifetime loader pins. The report owns no
// unpin operation: FLAG_PIN is held by Windows until this actual process exits.
// Strict expansion may be acquired through at most four actual PIN passes.
// Removal/substitution, any failed PIN and exhausted budgets still refuse.
// The successful final observed numeric base set must equal the actual PIN set.
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
    std::array<FProcessCodePinPass,MaximumProcessPinPasses> PassObservations{};
    std::uint32_t CompletedPassRecords=0;
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
    std::uint32_t PassCount() const noexcept {return CompletedPassRecords;}
    const std::array<FProcessCodePinPass,MaximumProcessPinPasses>& Passes() const noexcept {return PassObservations;}
};
}
