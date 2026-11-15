#pragma once

#include "CoreMinimal.h"
#include "Math/VectorRegister.h"
#include <stdexcept>
#include <exception>
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC)
#include <immintrin.h>
#endif

namespace Speed::Analytic
{
// A native execution requirement, never serialized eligibility or a proof ticket.
// Unspecified preserves the pre-existing mesh/diagnostic execution contract.
enum class EAnalyticNumericContract : uint8 { Unspecified, Binary64NearestAllowFlush };
enum class EAnalyticNumericStatus : uint8 { NotApplicable, Complete, Unknown };
enum class EAnalyticNumericFailure : uint8
{
    None, UnsupportedContract, HardwareUnavailable, InconsistentVectorRead,
    UnmaskedExceptions, VectorRounding, ScalarRounding, ContextApplication, ContextRestoration
};
struct FAnalyticNumericEvidence
{
    EAnalyticNumericStatus Status = EAnalyticNumericStatus::Unknown;
    EAnalyticNumericFailure Failure = EAnalyticNumericFailure::HardwareUnavailable;
    uint32 VectorControl = 0, ScalarControl = 0, SnapshotVectorControl = 0;
    bool bHardwareAvailable = false;
    uint64 ControlFingerprint() const noexcept
    {
        // Sticky exception flags do not select arithmetic. FTZ/DAZ do.
        return uint64(VectorControl & ~uint32(0x3f)) | (uint64(ScalarControl) << 32);
    }
};
// Integer-only classifier, also used by the independent hardware-mode tests.
inline FAnalyticNumericEvidence ClassifyAnalyticNumericContext(
    EAnalyticNumericContract Contract, FAnalyticNumericEvidence E) noexcept
{
    if (Contract == EAnalyticNumericContract::Unspecified)
    { E.Status=EAnalyticNumericStatus::NotApplicable; E.Failure=EAnalyticNumericFailure::None; return E; }
    E.Status=EAnalyticNumericStatus::Unknown;
    if (Contract != EAnalyticNumericContract::Binary64NearestAllowFlush)
        E.Failure=EAnalyticNumericFailure::UnsupportedContract;
    else if (!E.bHardwareAvailable) E.Failure=EAnalyticNumericFailure::HardwareUnavailable;
    else if (E.VectorControl != E.SnapshotVectorControl) E.Failure=EAnalyticNumericFailure::InconsistentVectorRead;
    else if ((E.VectorControl & 0x1f80u) != 0x1f80u || (E.ScalarControl & 0x003fu) != 0x003fu)
        E.Failure=EAnalyticNumericFailure::UnmaskedExceptions;
    else if ((E.VectorControl & 0x6000u) != 0) E.Failure=EAnalyticNumericFailure::VectorRounding;
    else if ((E.ScalarControl & 0x0c00u) != 0) E.Failure=EAnalyticNumericFailure::ScalarRounding;
    else { E.Status=EAnalyticNumericStatus::Complete; E.Failure=EAnalyticNumericFailure::None; }
    return E;
}
inline FAnalyticNumericEvidence InspectCurrentAnalyticNumericContext(EAnalyticNumericContract Contract) noexcept
{
    FAnalyticNumericEvidence E;
    if (Contract == EAnalyticNumericContract::Unspecified) return ClassifyAnalyticNumericContext(Contract,E);
    E.VectorControl=VectorGetControlRegister();
#if defined(_MSC_VER) && defined(_M_X64) && !defined(_M_ARM64EC)
    // Same read-only FXSAVE64 pattern as FrameSpeculationSafety1213. No FXRSTOR,
    // arithmetic probe, fenv mutation, allocation or logging occurs here.
    alignas(16) uint8 Saved[512]{};
    _fxsave64(Saved);
    E.ScalarControl=uint32(Saved[0]) | (uint32(Saved[1]) << 8);
    E.SnapshotVectorControl=uint32(Saved[24]) | (uint32(Saved[25]) << 8)
        | (uint32(Saved[26]) << 16) | (uint32(Saved[27]) << 24);
    E.bHardwareAvailable=true;
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
    unsigned short Control=0;
    __asm__ __volatile__("fnstcw %0" : "=m"(Control));
    E.ScalarControl=uint32(Control);
    E.SnapshotVectorControl=VectorGetControlRegister();
    E.bHardwareAvailable=true;
#endif
    return ClassifyAnalyticNumericContext(Contract,E);
}
class FAnalyticNumericUnknown final : public std::runtime_error
{
public:
    explicit FAnalyticNumericUnknown(const FAnalyticNumericEvidence& InEvidence)
        : std::runtime_error("IAmSpeed authored analytic numeric context UNKNOWN"), Evidence(InEvidence) {}
    FAnalyticNumericEvidence Evidence;
};
inline FAnalyticNumericEvidence RequireCurrentAnalyticNumericContext(EAnalyticNumericContract Contract)
{
    const auto E=InspectCurrentAnalyticNumericContext(Contract);
    if (E.Status == EAnalyticNumericStatus::Unknown) throw FAnalyticNumericUnknown(E);
    return E;
}
inline FAnalyticNumericEvidence AnalyticNumericTransferFailure(EAnalyticNumericContract Contract,
    EAnalyticNumericFailure Failure) noexcept
{
    auto E=InspectCurrentAnalyticNumericContext(Contract);
    E.Status=EAnalyticNumericStatus::Unknown; E.Failure=Failure;
    return E;
}
inline void RequireAnalyticNumericTransfer(EAnalyticNumericContract Contract, bool bAvailable,
    EAnalyticNumericFailure Failure=EAnalyticNumericFailure::ContextApplication)
{
    if (Contract == EAnalyticNumericContract::Unspecified) return;
    if (!bAvailable) throw FAnalyticNumericUnknown(AnalyticNumericTransferFailure(Contract,Failure));
    RequireCurrentAnalyticNumericContext(Contract);
}
[[noreturn]] inline void ThrowAnalyticNumericTransferFailure(EAnalyticNumericContract Contract,
    EAnalyticNumericFailure Failure, const char* LegacyMessage)
{
    if (Contract == EAnalyticNumericContract::Unspecified) throw std::runtime_error(LegacyMessage);
    throw FAnalyticNumericUnknown(AnalyticNumericTransferFailure(Contract,Failure));
}
inline std::exception_ptr AnalyticNumericTransferException(EAnalyticNumericContract Contract,
    EAnalyticNumericFailure Failure, const char* LegacyMessage)
{
    if (Contract == EAnalyticNumericContract::Unspecified) return std::make_exception_ptr(std::runtime_error(LegacyMessage));
    return std::make_exception_ptr(FAnalyticNumericUnknown(AnalyticNumericTransferFailure(Contract,Failure)));
}
inline bool ValidateCurrentAnalyticNumericContext(EAnalyticNumericContract Contract, FString* OutReason)
{
    const auto E=InspectCurrentAnalyticNumericContext(Contract);
    if (E.Status != EAnalyticNumericStatus::Unknown) return true;
    if (OutReason) *OutReason=FString::Printf(TEXT("AnalyticNumericUnknown reason=%u MXCSR=%08X FCW=%04X SnapshotMXCSR=%08X"),
        uint32(E.Failure), E.VectorControl, E.ScalarControl, E.SnapshotVectorControl);
    return false;
}
}
