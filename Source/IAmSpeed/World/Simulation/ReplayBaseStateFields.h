#pragma once
// Explicit replay schema1. Field order is a versioned storage contract.
namespace Speed::Replay::Fields
{
// FKinematicState
template<class Archive,class State> void Kinematic(Archive& A,State& S)
{
    A.Vector(S.Location);
    A.Vector(S.Velocity);
    A.Vector(S.Acceleration);
    A.Quat(S.Rotation);
    A.Vector(S.AngularVelocity);
    A.Vector(S.AngularAcceleration);
}
// SBaseGameState
template<class Archive,class State> void BaseGame(Archive& A,State& S)
{
    A.Scalar(S.NumFrame);
    A.Vector(S.TestVelocity);
    A.Vector(S.TestAngularVelocity);
    A.Scalar(S.bHasTestVelocity);
}
// FBasePhysicsState
template<class Archive,class State> void BasePhysics(Archive& A,State& S)
{
    Kinematic(A,S.Kinematic);
    A.Scalar(S.bIsFrozen);
    A.Scalar(S.nbFramesbeforeCanMove);
    A.Scalar(S.bStartCountdown);
}
// FWheeledGameState
template<class Archive,class State> void WheeledGame(Archive& A,State& S)
{
    A.Scalar(S.bGroundState);
    A.Scalar(S.bIsUpsideDown);
    A.Array(S.WheelsOmega);
    A.Array(S.WheelsAngularPosition);
    A.Scalar(S.NbWheelsOnGround);
    A.Scalar(S.NbFrameSinceInAir);
}
// FWheeledPhysicsState
template<class Archive,class State> void WheeledPhysics(Archive& A,State& S)
{
    A.Scalar(S.bStartCountdown);
    A.Scalar(S.nbFramesbeforeCanMove);
    A.Scalar(S.FramesSinceLastImpact);
    A.Vector(S.AllowedSideVelocity);
    A.Vector(S.AllowedAngularVelocity);
    A.Array(S.SuspensionLastDisplacement);
    A.Scalar(S.PhysicalThrottleInput);
    A.Scalar(S.PhysicalBrakeInput);
    A.Scalar(S.PhysicalSteerInput);
    A.Scalar(S.NbFramesSinceGroundContact);
}
// FSpeedCarCameraPhysicalInput
template<class Archive,class State> void CameraInput(Archive& A,State& S)
{
    A.Scalar(S.Version);
    A.Scalar(S.Flags);
    A.Scalar(S.Yaw);
    A.Scalar(S.Pitch);
}
// FWheeledInputState
template<class Archive,class State> void WheeledInput(Archive& A,State& S)
{
    A.Scalar(S.bCanMove);
    A.Scalar(S.Throttle);
    A.Scalar(S.Brake);
    A.Scalar(S.Steer);
    CameraInput(A,S.Camera);
}
}
