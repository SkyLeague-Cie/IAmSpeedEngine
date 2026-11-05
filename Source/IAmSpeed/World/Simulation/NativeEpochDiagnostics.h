#pragma once
#include "CoreMinimal.h"
#include "Logging/LogMacros.h"
#include <vector>
namespace Speed::Netcode::Prediction {class FPrivateRoadPhysicsWorld;}
namespace Speed::NativeEpoch
{
// Values only. No UObject, mutable native reference or arbitrary callback is
// retained in the physical phase. This is diagnostic routing, NOT a lease.
IAMSPEED_API bool HasOriginalDiagnosticOutbox() noexcept;
// Read-only real output-dispatch context, never an exclusion/authority token.
IAMSPEED_API bool IsOriginalDiagnosticFlush() noexcept;
IAMSPEED_API void QueueOriginalDiagnostic(const ANSICHAR* File,int32 Line,
    const FLogCategoryName& Category,ELogVerbosity::Type Verbosity,FString Message);
class IAMSPEED_API FDiagnosticOutbox final
{
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    struct FRecord
    {
        const ANSICHAR* File;int32 Line;FLogCategoryName Category;
        ELogVerbosity::Type Verbosity;FString Message;
    };
    std::vector<FRecord> Records;
    bool Active=false,Overflow=false;
    uint64 Characters=0;
    FDiagnosticOutbox();
    bool Begin() noexcept;
    void End() noexcept;
    bool Flush(); // only AFTER physical scope/GC exclusion have ended
    friend void QueueOriginalDiagnostic(const ANSICHAR*,int32,const FLogCategoryName&,ELogVerbosity::Type,FString);
public:
    ~FDiagnosticOutbox();
    FDiagnosticOutbox(const FDiagnosticOutbox&)=delete;
    FDiagnosticOutbox& operator=(const FDiagnosticOutbox&)=delete;
};
}
// The ordinary legacy path is exactly the original UE_LOG call. All original
// categories and severities remain; Fatal remains immediate and terminal.
#if NO_LOGGING
#define SPEED_NATIVE_LOG(Category,Verbosity,...) UE_LOG(Category,Verbosity,__VA_ARGS__)
#else
#define SPEED_NATIVE_LOG(Category,Verbosity,...) do { \
    if constexpr(ELogVerbosity::Verbosity==ELogVerbosity::Fatal) {UE_LOG(Category,Verbosity,__VA_ARGS__);} \
    else if(Speed::NativeEpoch::HasOriginalDiagnosticOutbox()) { \
        if(UE_LOG_ACTIVE(Category,Verbosity)) Speed::NativeEpoch::QueueOriginalDiagnostic(__FILE__,__LINE__,Category.GetCategoryName(),ELogVerbosity::Verbosity,FString::Printf(__VA_ARGS__)); \
    } else {UE_LOG(Category,Verbosity,__VA_ARGS__);} \
} while(false)
#endif
