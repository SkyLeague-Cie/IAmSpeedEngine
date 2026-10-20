#pragma once
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "WheelSystem.h"
#include "SuspensionSystem.h"
namespace Speed::Testing1534
{
// Capture actual live Setup() values, not only the semantic setup pointer.
// FGraph's stored points/bounds are private on the pinned immutable Engine.
// This bounded fixture admits only an actually empty graph; no public presence
// query is presented as a complete serialization of an active private graph.
inline bool AppendWheelConfig(TArray<uint8>& O,const Chaos::FSimpleWheelConfig& C)
{
 using W=FEpisodeStateBytesForTesting;
 W::Add(O,C.Offset);W::Add(O,C.WheelMass);W::Add(O,C.WheelRadius);W::Add(O,C.WheelWidth);
 W::Add(O,C.MaxSteeringAngle);W::Add(O,C.MaxBrakeTorque);W::Add(O,C.HandbrakeTorque);W::Add(O,C.ABSEnabled);
 W::Add(O,C.BrakeEnabled);W::Add(O,C.HandbrakeEnabled);W::Add(O,C.SteeringEnabled);W::Add(O,C.EngineEnabled);W::Add(O,C.TractionControlEnabled);
 W::Add(O,C.TorqueRatio);W::Add(O,C.AxleType);W::Add(O,C.FrictionCombineMethod);W::Add(O,C.ExternalTorqueCombineMethod);
 W::Add(O,C.FrictionMultiplier);W::Add(O,C.LateralSlipGraphMultiplier);W::Add(O,C.CorneringStiffness);W::Add(O,C.SideSlipModifier);
 W::Add(O,C.SlipThreshold);W::Add(O,C.SkidThreshold);W::Add(O,C.MaxSpinRotation);
 const bool Empty=C.LateralSlipGraph.IsEmpty();W::Add(O,Empty);return Empty;
}
inline void AppendSuspensionConfig(TArray<uint8>& O,const Chaos::FSimpleSuspensionConfig& C)
{
 using W=FEpisodeStateBytesForTesting;
 W::Add(O,C.SuspensionAxis);W::Add(O,C.SuspensionForceOffset);W::Add(O,C.SuspensionMaxRaise);W::Add(O,C.SuspensionMaxDrop);W::Add(O,C.MaxLength);
 W::Add(O,C.SpringRate);W::Add(O,C.SpringPreload);W::Add(O,C.CompressionDamping);W::Add(O,C.ReboundDamping);W::Add(O,C.RestingForce);
 W::Add(O,C.Swaybar);W::Add(O,C.DampingRatio);W::Add(O,C.WheelLoadRatio);W::Add(O,C.RaycastSafetyMargin);W::Add(O,C.SuspensionSmoothing);
}
}
#endif
