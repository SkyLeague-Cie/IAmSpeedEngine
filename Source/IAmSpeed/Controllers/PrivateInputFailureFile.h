#pragma once
#include "CoreMinimal.h"
#include <array>

namespace Speed::Input::Diagnostics
{
enum class EPrivateInputChannel : uint8 { FirstFault, Retirement, Startup };
// One GT owner. Failed/dropped writes consume an attempt too; never retry disk IO.
class FPrivateInputRecordBudgets final
{
public:
    static constexpr uint32 PerChannel = 8, Channels = 3, ProcessMaximum = PerChannel * Channels;
    uint32 Take(EPrivateInputChannel Channel)
    {
        const uint32 Index = uint32(Channel);
        return Index < Channels && Attempts[Index] < PerChannel ? ++Attempts[Index] : 0;
    }
    bool HasCapacity(EPrivateInputChannel Channel) const
    {
        const uint32 Index = uint32(Channel);
        return Index < Channels && Attempts[Index] < PerChannel;
    }
private:
    std::array<uint32, Channels> Attempts{};
};
static_assert(FPrivateInputRecordBudgets::ProcessMaximum == 24);
}

// Shipping-capable, GT-only and default-off under SLPrivateInputFailureFile.
// Independent eight-attempt channels: InputFirstFault, InputRetirement, InputStartup.
// Total <=24 attempts/process; each complete escaped UTF8 JSONL record <=4096 bytes.
IAMSPEED_API void RecordPrivateInputFailure(const TCHAR* Site, const FString& Evidence);
IAMSPEED_API void RecordPrivateInputStartup(const TCHAR* Site, const FString& Evidence);
IAMSPEED_API bool CanRecordPrivateInputStartup();
