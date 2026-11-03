#pragma once
// Private original solid mass accesses. No public issuer or restore lease.
class USolidSubBody;
class UBoxSubBody;
class USphereSubBody;
class USWheelSubBody;
class USpeedWheeledComponent;
namespace Speed::Netcode::Prediction { class FPrivateRoadPhysicsWorld; class FPrivateNativeOperation; }
namespace Speed::NativeKinematics
{
class FSolidMassAccess final
{
    friend class ::UBoxSubBody;
    friend class ::USphereSubBody;
    friend class ::USWheelSubBody;
    friend class ::USpeedWheeledComponent;
    friend class Speed::Netcode::Prediction::FPrivateRoadPhysicsWorld;
    static float Read(const USolidSubBody& Body);
    static float Read(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body);
    static void Write(USolidSubBody& Body,float Value);
    static void Write(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,USolidSubBody& Body,float Value);
};
}
