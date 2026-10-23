#pragma once
#include "IAmSpeed/Components/Netcode/SpeedPhysicsComponent.h"
#include "IAmSpeed/Components/Netcode/SpeedWheeledPhysicsComponent.h"
#include <type_traits>
#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
namespace Speed::SelectedStateBytesForTesting
{
namespace Private
{
// Exact selected-field bytes only. Capacity is a compile-time sum of scalar
// widths; no UE object/padding is copied and no floating-point math is done.
template<class... V> void AppendNamedScalars(TArray<uint8>& Out,const V&... Fields)
{
 static_assert(sizeof...(V)>0 && (std::is_arithmetic_v<V> && ...));
 constexpr SIZE_T BytesCount=(sizeof(V)+...);
 static_assert(BytesCount<=256); // The two selected-field bounded records below.
 uint8 Bytes[BytesCount];uint8* Cursor=Bytes;
 const auto Copy=[&Cursor](const auto& Value)
 {
  using S=std::remove_cvref_t<decltype(Value)>;
  const S Scalar=Value;FMemory::Memcpy(Cursor,&Scalar,sizeof(Scalar));Cursor+=sizeof(Scalar);
 };
 (Copy(Fields),...);
 Out.Append(Bytes,int32(BytesCount));
}
}
inline void AppendBaseState(TArray<uint8>& Out,const FBasePhysicsState& X)
{
 const auto& K=X.Kinematic;
 Private::AppendNamedScalars(Out,
  K.Location.X,K.Location.Y,K.Location.Z,K.Velocity.X,K.Velocity.Y,K.Velocity.Z,
  K.Acceleration.X,K.Acceleration.Y,K.Acceleration.Z,
  K.Rotation.X,K.Rotation.Y,K.Rotation.Z,K.Rotation.W,
  K.AngularVelocity.X,K.AngularVelocity.Y,K.AngularVelocity.Z,
  K.AngularAcceleration.X,K.AngularAcceleration.Y,K.AngularAcceleration.Z,
  X.bIsFrozen,X.nbFramesbeforeCanMove,X.bStartCountdown);
}
inline void AppendWheeledStatePrefix(TArray<uint8>& Out,const FWheeledPhysicsState& X)
{
 static_assert(UE_ARRAY_COUNT(X.SuspensionLastDisplacement)==4);
 // The fixed-array uint64 length prefix remains exactly where legacy Add put it.
 Private::AppendNamedScalars(Out,X.bStartCountdown,X.nbFramesbeforeCanMove,X.FramesSinceLastImpact,
  X.AllowedSideVelocity.X,X.AllowedSideVelocity.Y,X.AllowedSideVelocity.Z,
  X.AllowedAngularVelocity.X,X.AllowedAngularVelocity.Y,X.AllowedAngularVelocity.Z,
  uint64(4),X.SuspensionLastDisplacement[0],X.SuspensionLastDisplacement[1],
  X.SuspensionLastDisplacement[2],X.SuspensionLastDisplacement[3],
  X.PhysicalThrottleInput,X.PhysicalBrakeInput,X.PhysicalSteerInput,X.NbFramesSinceGroundContact);
}
inline void AppendWheeledStateWithBump1564(TArray<uint8>& Out,const FWheeledPhysicsState& X,uint32 Schema,uint8 Kind,uint64 StableId)
{
 static_assert(UE_ARRAY_COUNT(X.SuspensionLastDisplacement)==4);
 // Same 80-byte selected prefix followed by the validated 15-byte suffix.
 Private::AppendNamedScalars(Out,X.bStartCountdown,X.nbFramesbeforeCanMove,X.FramesSinceLastImpact,
  X.AllowedSideVelocity.X,X.AllowedSideVelocity.Y,X.AllowedSideVelocity.Z,
  X.AllowedAngularVelocity.X,X.AllowedAngularVelocity.Y,X.AllowedAngularVelocity.Z,
  uint64(4),X.SuspensionLastDisplacement[0],X.SuspensionLastDisplacement[1],
  X.SuspensionLastDisplacement[2],X.SuspensionLastDisplacement[3],
  X.PhysicalThrottleInput,X.PhysicalBrakeInput,X.PhysicalSteerInput,X.NbFramesSinceGroundContact,
  Schema,Kind,StableId,X.CarBumpCooldownFrames);
}
inline void AppendWheeledBumpSuffix1564(TArray<uint8>& Out,uint32 Schema,uint8 Kind,uint64 StableId,uint16 Cooldown)
{
 static_assert(sizeof(Schema)==4&&sizeof(Kind)==1&&sizeof(StableId)==8&&sizeof(Cooldown)==2);
 uint8 Bytes[15];
 FMemory::Memcpy(Bytes,&Schema,4);FMemory::Memcpy(Bytes+4,&Kind,1);
 FMemory::Memcpy(Bytes+5,&StableId,8);FMemory::Memcpy(Bytes+13,&Cooldown,2);
 Out.Append(Bytes,15);
}
}
#endif
