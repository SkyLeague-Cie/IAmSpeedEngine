#include "BodyKinematicRead.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateNativeStorageMap.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "IAmSpeed/SubBodies/Common/ISweeper.h"
namespace Speed::NativeKinematics
{
const SHitResult& FBodyRead::CurrentHit(const USSubBody& Body) { return Body.CurrentHitStorage(); }
const SHitResult& FBodyRead::FutureHit(const USSubBody& Body) { return Body.FutureHitStorage(); }
bool FBodyRead::HasHit(const USSubBody& Body) { return Body.CurrentHitStorage().bBlockingHit; }
bool FBodyRead::WillHit(const USSubBody& Body) { return Body.FutureHitStorage().bBlockingHit; }
float FBodyRead::FutureTOI(const USSubBody& Body) { return Body.FutureHitStorage().TOI; }
const Speed::FKinematicState& FBodyRead::Body(const USSubBody& Body)
{
    return Body.KinematicsStorage();
}
const Speed::FKinematicState& FBodyRead::Sweeper(const ISweeper& Sweeper)
{
    return Sweeper.ReadPhysicsSweeperKinematics();
}

// Actual owner operation overloads. Legacy/default/custom entries above remain unchanged.
const SHitResult& FBodyRead::CurrentHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).CurrentHit; }
const SHitResult& FBodyRead::FutureHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).FutureHit; }
bool FBodyRead::HasHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).CurrentHit.bBlockingHit; }
bool FBodyRead::WillHit(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).FutureHit.bBlockingHit; }
float FBodyRead::FutureTOI(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body)
{ return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).FutureHit.TOI; }
const Speed::FKinematicState& FBodyRead::Body(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).Kinematics;
}
const Speed::FKinematicState& FBodyRead::Sweeper(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const ISweeper& Sweeper)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Sweeper).Kinematics;
}

ISpeedComponent* FBodyRead::Parent(const USSubBody& Body)
{return Body.GetParentComponent();}
ISpeedComponent* FBodyRead::Parent(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USSubBody& Body)
{
    return Speed::Netcode::Prediction::FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body).ParentComponent;
}

FMatrix FBodyRead::WorldInvInertia(const USolidSubBody& Body)
{return Body.ComputeWorldInvInertiaTensor();}
FMatrix FBodyRead::WorldInvInertia(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody& Body)
{
    using Speed::Netcode::Prediction::FPrivateNativeStorageAccess;
    // Actual adjusted USolidSubBody subject, same genuine operation/map/root,
    // and real retained alias owners checked BEFORE any virtual/object read.
    (void)FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,&Body);
    (void)FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,&Body);
    return Body.ComputeNativeWorldInvInertiaTensor(Operation);
}

FMatrix FBodyRead::WorldInvInertia(const Speed::Netcode::Prediction::FPrivateNativeOperation& Operation,const USolidSubBody* Body)
{
    using Speed::Netcode::Prediction::FPrivateNativeStorageAccess;
    (void)FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSubBodyValues>(Operation,Body);
    (void)FPrivateNativeStorageAccess::Part<const Speed::Netcode::Prediction::FPrivateRoadSolidValues>(Operation,Body);
    return Body->ComputeNativeWorldInvInertiaTensor(Operation);
}

}
