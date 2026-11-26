#pragma once
#include "CoreMinimal.h"
class UWorld;
namespace Speed::Netcode
{
// GT-only read-only shared policy. Callers still own and validate each history object.
// Legacy preserves the default path; Invalid must never select a fallback.
enum class EExactRemoteHostPolicy : uint8 { Legacy, IsolatedDedicated, Invalid };
IAMSPEED_API EExactRemoteHostPolicy QueryExactRemoteHostPolicy(const UWorld* World);
}
