#pragma once
#include "NativeLocalSession.h"
#include <string>
namespace Speed::Netcode
{
struct FNativePredictionProbeOptions
{
    FLocalSessionConfig Launcher;
    std::string ReportPath,ReleasePath;
};
// Finite commandlet/process probe, NOT game-client admission. Uses unit-authored
// local input frames and the real private native road kernel. Reconciliation is
// from original birth, not localized fullstate restore. No hardware/UI/real-Demo
// or complete hidden-state coverage may be inferred from a successful probe.
IAMSPEED_API bool RunNativePredictionProcessProbe(const FNativePredictionProbeOptions& Options);
}
