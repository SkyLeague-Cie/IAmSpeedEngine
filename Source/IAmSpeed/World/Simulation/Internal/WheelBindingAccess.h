#pragma once
// Private live binding reads/writes only. No public issuer or restore lease.
class USWheelSubBody;
class UChaosVehicleWheel;
class USpeedWheeledComponent;
namespace Chaos { class FSimpleWheelSim; class FSimpleSuspensionSim; }
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; namespace PartialObservation { class FNativeWheelCapture; } }
namespace Speed::NativeKinematics
{
class FWheelBindingAccess final
{
    friend class ::USpeedWheeledComponent;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    friend class Speed::Netcode::Prediction::PartialObservation::FNativeWheelCapture;
    static const UChaosVehicleWheel* Configuration(const USWheelSubBody& Body);
    static const UChaosVehicleWheel* Configuration(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body);
    static const Chaos::FSimpleWheelSim* Wheel(const USWheelSubBody& Body);
    static const Chaos::FSimpleWheelSim* Wheel(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body);
    static const Chaos::FSimpleSuspensionSim* Suspension(const USWheelSubBody& Body);
    static const Chaos::FSimpleSuspensionSim* Suspension(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body);
    static bool HasSuspension(const USWheelSubBody& Body);
    static bool HasSuspension(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USWheelSubBody& Body);
    static void BindChaosWheel(USWheelSubBody& Body,UChaosVehicleWheel* Value);
    static void BindChaosWheel(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USWheelSubBody& Body,UChaosVehicleWheel* Value);
    static void BindWheel(USWheelSubBody& Body,Chaos::FSimpleWheelSim* Value);
    static void BindWheel(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USWheelSubBody& Body,Chaos::FSimpleWheelSim* Value);
    static void BindSuspension(USWheelSubBody& Body,Chaos::FSimpleSuspensionSim* Value);
    static void BindSuspension(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USWheelSubBody& Body,Chaos::FSimpleSuspensionSim* Value);
};
}
