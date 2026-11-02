#pragma once
#include "NativeLocalSession.h"
#include <string>
#include <optional>
namespace Speed::Netcode::Prediction {struct FPrivateRoadPublication;}
namespace Speed::Netcode
{
enum class ENativeProbeScenario : std::uint8_t {SeparatedInputFixture,InteractingInputFixture};
struct FNativePredictionProbeOptions
{
    FLocalSessionConfig Launcher;
    ENativeProbeScenario Scenario=ENativeProbeScenario::SeparatedInputFixture; // finite unit source only
    std::string ReportPath,ReleasePath;
    // Controlled same-host DATA fragment loss only (owner input, authority
    // input and observable kinematics). Startup/ACK/close controls unaffected.
    std::uint32_t ControlledDatagramDropEvery=0;
    // Zero preserves the current fixture batching. Nonzero limits each retry
    // to the oldest N unacknowledged ORIGINAL records; never relabel/skip holes.
    std::uint32_t InputRetryRecordLimit=0; // supported 1/2/4/8/16
    std::uint32_t MaxReplayFramesPerPulse=16; // work budget; physical delta remains1/300s
};
// Finite commandlet/process probe, NOT game-client admission. Uses unit-authored
// local input frames and the real private native road kernel. Reconciliation is
// from original birth, not localized fullstate restore. No hardware/UI/real-Demo
// or complete hidden-state coverage may be inferred from a successful probe.
enum class ENativeProbePumpStep { Running,Succeeded,Failed,WrongOwner,Reentrant,Cancelled };
struct FNativeProbePumpDiagnostics
{
    std::uint64_t ClientDoneSendAttempts=0,ClientDoneDatagrams=0;
};
// Cooperative finite, unit-authored native probe. Tick returns at a completed
// publication or transport wait boundary, retaining actual native ownership.
// A game host can keep rendering between Tick calls. This API does not admit
// live device input, a game-client profile, complete state or localized replay.
class IAMSPEED_API FNativePredictionProbePump final
{
    struct FImpl;std::unique_ptr<FImpl> Impl;
    explicit FNativePredictionProbePump(FNativePredictionProbeOptions);
public:
    ~FNativePredictionProbePump();
    FNativePredictionProbePump(const FNativePredictionProbePump&)=delete;
    FNativePredictionProbePump& operator=(const FNativePredictionProbePump&)=delete;
    static std::unique_ptr<FNativePredictionProbePump> Create(FNativePredictionProbeOptions,std::string& Why);
    ENativeProbePumpStep Tick(); // original GT only; one bounded protocol pulse
    // Synchronously unwind the genuinely suspended native owners on their GT.
    // Callback Tick sees Reentrant, ReadLatest sees no publication; cancellation
    // is a distinct terminal result and cannot manufacture protocol success.
    ENativeProbePumpStep Cancel();
    std::optional<FNativeProbePumpDiagnostics> Diagnostics() const; // actual send counters, owner only
    std::shared_ptr<const Prediction::FPrivateRoadPublication> ReadLatest() const noexcept;
    std::string FailureReason() const; // values only, original GT only
    // GT destruction unwinds the suspended native stack and closes its real
    // graph/socket. It never reports ClientDone, success or Job drain on cancel.
};
IAMSPEED_API bool RunNativePredictionProcessProbe(const FNativePredictionProbeOptions& Options);
}
