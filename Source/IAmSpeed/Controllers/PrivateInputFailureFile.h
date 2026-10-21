#pragma once
#include "CoreMinimal.h"
// GT-only opt-in private evidence. A single process-wide eight-record budget
// covers every caller. Evidence is JSON-escaped plain text, never executable JSON.
IAMSPEED_API void RecordPrivateInputFailure(const TCHAR* Site, const FString& Evidence);
