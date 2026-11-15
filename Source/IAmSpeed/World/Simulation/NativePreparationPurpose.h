#pragma once
#include "CoreMinimal.h"
namespace Speed::OnePausedWork
{
enum class EPurpose : uint8 { FullFrame, PreparationPrefixStop, PostPrepareInitialPositionStop, TerminalStepSyncStop, TerminalSphereRsCaptureStop, TerminalConsumedPrefixAdvanceStop, TerminalRsContactAdvanceStop, TerminalRsResponseStop, TerminalRsResponseSyncStop, TerminalPostSyncRsCaptureStop };
static_assert(uint8(EPurpose::TerminalSphereRsCaptureStop)==4&&uint8(EPurpose::TerminalConsumedPrefixAdvanceStop)==5,"Terminal purpose wire values must stay stable");
static_assert(uint8(EPurpose::TerminalRsContactAdvanceStop)==6,"Rs contact-advance wire purpose must be6");
static_assert(uint8(EPurpose::TerminalRsResponseStop)==7,"Rs response wire purpose must be7");
static_assert(uint8(EPurpose::TerminalRsResponseSyncStop)==8,"Rs response-sync wire purpose must be8");
static_assert(uint8(EPurpose::TerminalPostSyncRsCaptureStop)==9,"Post-sync Rs capture purpose must be9");
constexpr bool HasFirstRsResponseSyncPrefix(EPurpose p) noexcept
{return p==EPurpose::TerminalRsResponseSyncStop||p==EPurpose::TerminalPostSyncRsCaptureStop;}
constexpr bool HasFirstRsResponsePrefix(EPurpose p) noexcept
{return p==EPurpose::TerminalRsResponseStop||HasFirstRsResponseSyncPrefix(p);}
constexpr bool HasBoundedRsContactPrefix(EPurpose p) noexcept
{return p==EPurpose::TerminalRsContactAdvanceStop||HasFirstRsResponsePrefix(p);}
constexpr bool HasConsumedSphereAdvancePrefix(EPurpose p) noexcept
{return p==EPurpose::TerminalConsumedPrefixAdvanceStop||HasBoundedRsContactPrefix(p);}
// Position is still its own one-use effect, before the native Step serial changes.
constexpr bool HasInitialPosition(EPurpose purpose) noexcept
{return purpose==EPurpose::PostPrepareInitialPositionStop||purpose==EPurpose::TerminalStepSyncStop||(purpose==EPurpose::TerminalSphereRsCaptureStop||HasConsumedSphereAdvancePrefix(purpose));}
constexpr bool HasNativeStepPrefix(EPurpose purpose) noexcept
{return purpose==EPurpose::TerminalStepSyncStop||(purpose==EPurpose::TerminalSphereRsCaptureStop||HasConsumedSphereAdvancePrefix(purpose));}
constexpr bool HasSphereRsCapturePrefix(EPurpose purpose) noexcept
{return purpose==EPurpose::TerminalSphereRsCaptureStop||HasConsumedSphereAdvancePrefix(purpose);}
constexpr int32 ExpectedNativeCallCount(EPurpose purpose) noexcept
{
    return purpose==EPurpose::PreparationPrefixStop?33:purpose==EPurpose::PostPrepareInitialPositionStop?34:
        purpose==EPurpose::TerminalPostSyncRsCaptureStop?53:purpose==EPurpose::TerminalStepSyncStop?38:purpose==EPurpose::TerminalSphereRsCaptureStop?41:purpose==EPurpose::TerminalRsResponseSyncStop?50:purpose==EPurpose::TerminalRsResponseStop?49:HasConsumedSphereAdvancePrefix(purpose)?45:0;
}
}
namespace Speed::PreparationPrefix
{
class FContract;
using FContractPtr = TSharedPtr<const FContract, ESPMode::ThreadSafe>;
}
