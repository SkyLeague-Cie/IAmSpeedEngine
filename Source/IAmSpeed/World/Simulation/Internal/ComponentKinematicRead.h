#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"

// Module-internal live reads. No exported entry point or public issuer.
// This access control is NOT an ownership, lifetime or restoration lease.
class ISpeedComponent;
class ISpeedWheeledComponent;
class URayWheelSubBody;
class ISolidSweeper;
class USpeedWheeledComponent;
class USolidSubBody;
class UBoxSubBody;
class USphereSubBody;
class USWheelSubBody;
class USpeedWorldSubsystem;
namespace Speed { struct FKinematicState; class FSimulationWorld; }

namespace Speed::NativeKinematics
{
struct FCylinderRigidTrajectory;
struct FDynamicBodyWitness;
struct FGutterSlideDamping;
class FComponentRead final
{
    friend class ::ISpeedComponent;
    friend class ::ISpeedWheeledComponent;
    friend class ::URayWheelSubBody;
    friend class ::ISolidSweeper;
    friend class ::USpeedWheeledComponent;
    friend class ::USolidSubBody;
    friend class ::UBoxSubBody;
    friend class ::USphereSubBody;
    friend class ::USWheelSubBody;
    friend class ::USpeedWorldSubsystem;
    friend class Speed::FSimulationWorld;
    friend struct FCylinderRigidTrajectory;
    friend struct FDynamicBodyWitness;
    friend struct FGutterSlideDamping;
    static const FQuat& Rotation(const ISpeedComponent& Component);
    static const FVector& COMVelocity(const ISpeedComponent& Component);
    static const FVector& AngularVelocity(const ISpeedComponent& Component);
    static const FVector& Acceleration(const ISpeedComponent& Component);
    static const FVector& AngularAcceleration(const ISpeedComponent& Component);
    static const FVector& COM(const ISpeedComponent& Component);
    static const FVector& LocalCOM(const ISpeedComponent& Component);
    static FVector Location(const ISpeedComponent& Component);
    static FVector Velocity(const ISpeedComponent& Component);
    static FVector VelocityAtPoint(const ISpeedComponent& Component, const FVector& Point);
    static FVector AccelerationAtPoint(const ISpeedComponent& Component, const FVector& Point);
    static FVector Forward(const ISpeedComponent& Component);
    static FVector Right(const ISpeedComponent& Component);
    static FVector Up(const ISpeedComponent& Component);
    static float ForwardSpeed(const ISpeedComponent& Component);
    static bool GoingForward(const ISpeedComponent& Component);
    static Speed::FKinematicState Origin(const ISpeedComponent& Component);
    static Speed::FKinematicState OriginForFrame(const ISpeedComponent& Component, const unsigned int& NumFrameToRead);
    static const Speed::FKinematicState& Current(const ISpeedComponent& Component);
    static const Speed::FKinematicState& ForFrame(const ISpeedComponent& Component, const unsigned int& Frame);
    class FView final
    {
        const ISpeedComponent& Component;
        friend class FComponentRead;
        explicit FView(const ISpeedComponent& Value) noexcept : Component(Value) {}
    public:
        const FQuat& Rotation() const { return FComponentRead::Rotation(Component); }
        const FVector& COMVelocity() const { return FComponentRead::COMVelocity(Component); }
        const FVector& AngularVelocity() const { return FComponentRead::AngularVelocity(Component); }
        const FVector& Acceleration() const { return FComponentRead::Acceleration(Component); }
        const FVector& AngularAcceleration() const { return FComponentRead::AngularAcceleration(Component); }
        const FVector& COM() const { return FComponentRead::COM(Component); }
        const FVector& LocalCOM() const { return FComponentRead::LocalCOM(Component); }
        FVector Location() const { return FComponentRead::Location(Component); }
        FVector Velocity() const { return FComponentRead::Velocity(Component); }
        FVector VelocityAtPoint(const FVector& Point) const { return FComponentRead::VelocityAtPoint(Component, Point); }
        FVector AccelerationAtPoint(const FVector& Point) const { return FComponentRead::AccelerationAtPoint(Component, Point); }
        FVector Forward() const { return FComponentRead::Forward(Component); }
        FVector Right() const { return FComponentRead::Right(Component); }
        FVector Up() const { return FComponentRead::Up(Component); }
        float ForwardSpeed() const { return FComponentRead::ForwardSpeed(Component); }
        bool GoingForward() const { return FComponentRead::GoingForward(Component); }
        Speed::FKinematicState Origin() const { return FComponentRead::Origin(Component); }
        Speed::FKinematicState OriginForFrame(const unsigned int& NumFrameToRead) const { return FComponentRead::OriginForFrame(Component, NumFrameToRead); }
    };
    static FView Of(const ISpeedComponent& Component) noexcept { return FView(Component); }

};
}
