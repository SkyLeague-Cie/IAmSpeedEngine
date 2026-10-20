#include "SubBodyKinematicsTasks.h"
#include "JoinedActorKinematics1213.h"
#include "IAmSpeed/World/Simulation/FrameSpeculationSafety1213.h"
#include "FrameParallelExecutor.h"
#include "HAL/PlatformTLS.h"
#include <exception>
#include <stdexcept>
#include "IAmSpeed/Base/ExactRotatorQuaternionCache.h"
#include "IAmSpeed/Components/ISpeedComponent.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SolidSubBody.h"
#include "IAmSpeed/SubBodies/Solid/BoxSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "IAmSpeed/SubBodies/Solid/SWheelSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/SensorSubBody.h"
#include "IAmSpeed/SubBodies/Sensor/BoxSensor.h"
#include "IAmSpeed/SubBodies/Sensor/SphereSensor.h"
#include "IAmSpeed/World/Simulation/FrameParallelPolicy.h"
#include "IAmSpeed/World/Simulation/FrameParallelSettings.h"

namespace Speed::FrameParallel
{
namespace
{
constexpr int32 MaxKinematicItems = 256;
bool IsSupportedNativeClass(const USSubBody& Body)
{
    const UClass* Type = Body.GetClass();
    return Type == USSubBody::StaticClass() || Type == USolidSubBody::StaticClass()
        || Type == UBoxSubBody::StaticClass() || Type == USphereSubBody::StaticClass()
        || Type == USWheelSubBody::StaticClass() || Type == USensorSubBody::StaticClass()
        || Type == UBoxSensor::StaticClass() || Type == USphereSensor::StaticClass();
}
}

bool CaptureNativeSubBodyKinematicInput(const USSubBody& Body, FSubBodyKinematicInput& Out)
{
    if (!IsSupportedNativeClass(Body)) return false;
    FSubBodyKinematicInput Captured;
    Captured.RelativeLocation = Body.GetRelativeLocation();
    // Pure conversion, deliberately avoiding USceneComponent's mutable cache.
    static thread_local Speed::FExactRotatorQuaternionCache LocalRotationCache;
    Captured.RelativeRotation = LocalRotationCache.Get(Body.GetRelativeRotation());
    Captured.bWheel = Body.GetClass() == USWheelSubBody::StaticClass();
    if (Captured.bWheel)
        Captured.WheelSpringDisplacement = static_cast<const USWheelSubBody&>(Body).SpringDisplacement();
    Out = Captured;
    return true;
}

SKinematic CalculateSubBodyKinematics(const SKinematic& Parent, const FSubBodyKinematicInput& Input)
{
    SKinematic Result;
    const FVector WorldOffset = Parent.Rotation.RotateVector(Input.RelativeLocation);
    Result.Location = Parent.Location + WorldOffset;
    Result.Velocity = Parent.Velocity + FVector::CrossProduct(Parent.AngularVelocity, WorldOffset);
    Result.Acceleration = Parent.Acceleration
        + FVector::CrossProduct(Parent.AngularAcceleration, WorldOffset)
        + FVector::CrossProduct(Parent.AngularVelocity, FVector::CrossProduct(Parent.AngularVelocity, WorldOffset));
    Result.Rotation = Parent.Rotation * Input.RelativeRotation;
    Result.AngularVelocity = Parent.AngularVelocity;
    Result.AngularAcceleration = Parent.AngularAcceleration;
    if (Input.bWheel)
    {
        const FVector SuspensionOffset = Parent.Rotation.RotateVector(
            Input.WheelSpringDisplacement * FVector::UpVector);
        Result.Location += SuspensionOffset;
        Result.Velocity = Parent.Velocity + FVector::CrossProduct(
            Parent.AngularVelocity, Result.Location - Parent.Location);
    }
    return Result;
}

bool CalculateSubBodyKinematicsBatch(const SKinematic& Parent,
    TConstArrayView<FSubBodyKinematicInput> Inputs, TArrayView<SKinematic> Outputs, const int32 Lanes)
{
    if (Inputs.Num() != Outputs.Num() || Inputs.Num() < 1 || Inputs.Num() > MaxKinematicItems
        || Lanes < 1 || Lanes > FFrameParallelPolicy::MaximumLanes || Lanes > Inputs.Num()) return false;
    const bool bNested = Speed::IsInsideFrameParallelTask();
    const bool bMasksSafe1213 = Lanes <= 1 || bNested || Speed::AreCurrentFrameFloatingPointExceptionsMasked1213();
    const FFrameFloatingPointContext OwnerNumericContext = CaptureFrameFloatingPointContext();
    const int32 EffectiveLanes = bNested || !OwnerNumericContext.bValid || !bMasksSafe1213 ? 1 : Lanes;
    const EFrameParallelFallback Fallback = !OwnerNumericContext.bValid || !bMasksSafe1213
        ? EFrameParallelFallback::NumericEnvironmentUnavailable : bNested
        ? EFrameParallelFallback::NestedTask : Lanes == 1
        ? EFrameParallelFallback::SerialPolicy : EFrameParallelFallback::None;
    if (EffectiveLanes == 1)
    {
        for (int32 Index = 0; Index < Inputs.Num(); ++Index)
            Outputs[Index] = CalculateSubBodyKinematics(Parent, Inputs[Index]);
        RecordFrameParallelStage(EFrameParallelStage::Prepare, Inputs.Num(), 1, 1, Fallback);
        return true;
    }
    uint32 NativeThreadIds[FFrameParallelPolicy::MaximumLanes] = {};
    std::exception_ptr Failures[FFrameParallelPolicy::MaximumLanes];
    const auto Work = [&Parent, Inputs, Outputs, EffectiveLanes, OwnerNumericContext,
        &NativeThreadIds, &Failures](const int32 Lane)
    {
        Speed::FScopedFrameParallelTask TaskScope(OwnerNumericContext);
        NativeThreadIds[Lane] = FPlatformTLS::GetCurrentThreadId();
        try
        {
            if (!TaskScope.IsNumericContextApplied())
                throw std::runtime_error("IAmSpeed preparation numeric context unavailable");
            for (int32 Index = Lane; Index < Inputs.Num(); Index += EffectiveLanes)
                Outputs[Index] = CalculateSubBodyKinematics(Parent, Inputs[Index]);
        }
        catch (...) { Failures[Lane] = std::current_exception(); }
        if (!TaskScope.RestoreNumericContext())
            Failures[Lane] = std::make_exception_ptr(std::runtime_error(
                "IAmSpeed preparation numeric restoration failed"));
    };
    Speed::DispatchFrameParallelLanes(EffectiveLanes, Work);
    int32 DistinctThreads = 0;
    for (int32 Lane = 0; Lane < EffectiveLanes; ++Lane)
    {
        bool bSeen = false;
        for (int32 Prior = 0; Prior < Lane; ++Prior) bSeen |= NativeThreadIds[Prior] == NativeThreadIds[Lane];
        if (!bSeen) ++DistinctThreads;
    }
    RecordFrameParallelStage(EFrameParallelStage::Prepare, Inputs.Num(), EffectiveLanes,
        DistinctThreads, EFrameParallelFallback::None);
    // No partial values are installed into actors: failures propagate at the joined barrier.
    for (int32 Lane = 0; Lane < EffectiveLanes; ++Lane)
        if (Failures[Lane]) std::rethrow_exception(Failures[Lane]);
    return true;
}

bool TryUpdateSubBodyKinematicsInParallel(ISpeedComponent& Component,
    const int32 LaneLimit, const int32 MinItemsPerLane)
{
    const auto& Bodies = Component.GetSubBodies();
    const auto SerialFallback = [&](EFrameParallelFallback Reason)
    {
        RecordFrameParallelStage(EFrameParallelStage::Prepare, Bodies.Num(),
            Bodies.IsEmpty() ? 0 : 1, Bodies.IsEmpty() ? 0 : 1, Reason);
        return false;
    };
    // Joined-base reuse must pass through the exact original native virtual
    // method and its per-body local rotation cache. Do not nest another batch.
    if (IsJoinedActorBaseOwnerScope1430()) return SerialFallback(EFrameParallelFallback::SerialPolicy);
    if (Speed::IsInsideFrameParallelTask()) return SerialFallback(EFrameParallelFallback::NestedTask);
    if (LaneLimit <= 1) return SerialFallback(EFrameParallelFallback::SerialPolicy);
    if (Bodies.Num() < 2) return SerialFallback(EFrameParallelFallback::TooFewItems);
    if (Bodies.Num() > MaxKinematicItems) return SerialFallback(EFrameParallelFallback::BatchCapacityExceeded);
    const int32 Lanes = Speed::FFrameParallelPolicy::ResolveDispatchLanes(
        LaneLimit, Bodies.Num(), MinItemsPerLane);
    if (Lanes <= 1) return SerialFallback(EFrameParallelFallback::TooFewItems);
    if (!Speed::AreCurrentFrameFloatingPointExceptionsMasked1213())
        return SerialFallback(EFrameParallelFallback::NumericEnvironmentUnavailable);

    // The legacy method derives the parent first, before any local body reads.
    const SKinematic OriginState = Component.GetOriginKinematicState();
    TArray<FSubBodyKinematicInput, TInlineAllocator<8>> Inputs;
    Inputs.SetNum(Bodies.Num());
    for (int32 Index = 0; Index < Bodies.Num(); ++Index)
        if (!Bodies[Index] || !CaptureNativeSubBodyKinematicInput(*Bodies[Index], Inputs[Index]))
            return SerialFallback(EFrameParallelFallback::UnsupportedWork);

    // Capture at this actor's original point, before this actor's original Reset.
    TArray<SKinematic, TInlineAllocator<8>> Outputs;
    Outputs.SetNum(Bodies.Num());
    if (!CalculateSubBodyKinematicsBatch(OriginState, MakeArrayView(Inputs), MakeArrayView(Outputs), Lanes)) return false;
    for (int32 Index = 0; Index < Bodies.Num(); ++Index)
        Bodies[Index]->SetKinematicState(Outputs[Index]);
    return true;
}
}
