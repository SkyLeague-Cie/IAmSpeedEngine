#pragma once

#if !UE_BUILD_SHIPPING
#include "CoreMinimal.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformAtomics.h"
#include "IAmSpeed/World/Analytic/StaticWorldQueryAudit.h"
#include "IAmSpeed/World/Collision/PreparedPhysicalExecution.h"

// Observe existing native queries and pose writes only. No diagnostic sweep,
// displacement prediction, setter or prepared-query serial allocation.
namespace Speed::BackboardNativeTrace
{
inline volatile int32 Records = 0;

inline int32 Take(const FVector& COM)
{
    static const auto* Enabled = IConsoleManager::Get().FindTConsoleVariableDataInt(
        TEXT("p.IAmSpeed.WheelSupport.FrontBumpTrace"));
    if (!Enabled || Enabled->GetValueOnAnyThread() == 0 ||
        FMath::Abs(COM.X) < 4800.0 || FMath::Abs(COM.X) > 5300.0 ||
        COM.Z < -100.0 || COM.Z > 250.0)
        return 0;
    const int32 Sequence = FPlatformAtomics::InterlockedIncrement(&Records);
    if (Sequence == 16385)
        UE_LOG(LogTemp, Display, TEXT("[BackboardNativeTraceLimit] Records=16384"));
    return Sequence <= 16384 ? Sequence : 0;
}

inline FString Vector(const FVector& V)
{
    return FString::Printf(TEXT("(%.17g,%.17g,%.17g)"), V.X, V.Y, V.Z);
}

inline FString Quat(const FQuat& Q)
{
    return FString::Printf(TEXT("(%.17g,%.17g,%.17g,%.17g)"), Q.X, Q.Y, Q.Z, Q.W);
}

inline FString Identity(int32 Sequence, uint32 Frame, const void* Owner, const void* Body,
    uint64 OwnerId, int32 SubBody, UWorld* World)
{
    PreparedExecution::FStepIdentity Step;
    const bool Valid = PreparedExecution::ReadCurrentStep(Step);
    uint64 Canonical = 0, Scene = 0;
    const bool CanonicalValid = Analytic::FStaticWorldQueryAudit::TryObserveStrictFrame(World, Canonical, Scene);
    return FString::Printf(
        TEXT("Seq=%d Frame=%u CanonicalValid=%d Canonical=%llu Scene=%llu StepValid=%d StepFrame=%llu Iteration=%d Step=%llu Query=%llu Thread=%u OwnerPtr=%p BodyPtr=%p Owner=%llu SubBody=%d"),
        Sequence, Frame, int(CanonicalValid), static_cast<unsigned long long>(Canonical),
        static_cast<unsigned long long>(Scene), int(Valid), static_cast<unsigned long long>(Step.Frame), Step.Iteration,
        static_cast<unsigned long long>(Step.StepSerial), static_cast<unsigned long long>(Step.QuerySerial),
        Step.ThreadId, Owner, Body, static_cast<unsigned long long>(OwnerId), SubBody);
}
}
#endif
