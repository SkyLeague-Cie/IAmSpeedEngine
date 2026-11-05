#pragma once
#include "PrivateRoadPhysicsWorld.h"
namespace Speed::Netcode::Prediction
{
enum class EPrivateRoadPartialComparison:uint8 {Unknown,Equal,Different};
inline EPrivateRoadPartialComparison ComparePrivateRoadPartialValues(const FPrivateRoadActorFrame& A,const FPrivateRoadActorFrame& B)
{
    if(!A.NativeSolidInitializationWitness||!B.NativeSolidInitializationWitness||!A.NativePartialValuesAvailable||!B.NativePartialValuesAvailable||A.NativeWheelSuspensionValues.empty()||B.NativeWheelSuspensionValues.empty()
        ||A.NativeWheelRuntimeValues.empty()||B.NativeWheelRuntimeValues.empty()||A.NativeSolidValues.empty()||B.NativeSolidValues.empty())return EPrivateRoadPartialComparison::Unknown;
    const auto& X=A.NativeSleep;const auto& Y=B.NativeSleep;const auto& P=A.NativeQuantization;const auto& Q=B.NativeQuantization;
    return X.Location==Y.Location&&X.Rotation==Y.Rotation&&X.HasCompletedPose==Y.HasCompletedPose
        &&P.NoOpInput==Q.NoOpInput&&P.Valid==Q.Valid&&A.NativeWheelSuspensionValues==B.NativeWheelSuspensionValues&&A.NativeWheelRuntimeValues==B.NativeWheelRuntimeValues&&A.NativeSolidValues==B.NativeSolidValues
        ?EPrivateRoadPartialComparison::Equal:EPrivateRoadPartialComparison::Different;
}
}
