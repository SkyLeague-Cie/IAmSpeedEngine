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
// Test-only history ranges. The caller retains count framing and owns both
// source histories and destination separately; no source object bytes/padding.
namespace Private
{
inline bool HistoryRangeFits1564(int32 Start,int32 Count,int32 RecordBytes) noexcept
{
 return Start>=0 && Count>=0 && RecordBytes>0
  && uint64(Count)*uint64(RecordBytes)<=uint64(MAX_int32-Start);
}
inline bool HistoryStorageOverlaps1564(const TArray<uint8>& Out,const void* Input,SIZE_T InputBytes) noexcept
{
 if(!InputBytes || !Out.GetAllocatedSize())return false;
 const UPTRINT A=reinterpret_cast<UPTRINT>(Out.GetData());
 const UPTRINT B=reinterpret_cast<UPTRINT>(Input);
 return A<=B ? B-A<Out.GetAllocatedSize() : A-B<InputBytes;
}
class FHistoryAppendRange1564 final
{
 TArray<uint8>& Out;const int32 Start;int32 Committed=0;bool Valid=false;
public:
 FHistoryAppendRange1564(TArray<uint8>& Bytes,int32 Count,int32 Width,const void* Input,SIZE_T InputBytes)
  :Out(Bytes),Start(Bytes.Num())
 {
  if(!HistoryRangeFits1564(Start,Count,Width) || (Count && !Input)
   || HistoryStorageOverlaps1564(Out,Input,InputBytes))return;
  // One append extent per history. Only a fully encoded record is committed.
  // The nested guard trims any unwritten tail before outer witness scopes run.
  Out.AddUninitialized(Count*Width);Valid=true;
 }
 ~FHistoryAppendRange1564()
 {if(Valid)Out.SetNum(Start+Committed,EAllowShrinking::No);}
 FHistoryAppendRange1564(const FHistoryAppendRange1564&)=delete;
 FHistoryAppendRange1564& operator=(const FHistoryAppendRange1564&)=delete;
 bool IsValid()const noexcept{return Valid;}
 uint8* Cursor()noexcept{return Out.GetData()+Start+Committed;}
 void Commit(int32 Bytes)noexcept{Committed+=Bytes;}
};
template<int32 Expected,class... V> void WriteHistoryScalars1564(uint8*& Cursor,const V&... Fields)
{
 static_assert(sizeof...(V)>0 && (std::is_arithmetic_v<V> && ...));
 static_assert((sizeof(V)+...)==Expected && Expected<=256);
 uint8 Bytes[Expected];uint8* LocalCursor=Bytes;
 const auto Copy=[&LocalCursor](const auto& Value)
 {
  using S=std::remove_cvref_t<decltype(Value)>;
  const S Scalar=Value;FMemory::Memcpy(LocalCursor,&Scalar,sizeof(Scalar));LocalCursor+=sizeof(Scalar);
 };
 (Copy(Fields),...);
 FMemory::Memcpy(Cursor,Bytes,Expected);Cursor+=Expected;
}
inline void WriteBaseHistory1564(uint8*& Cursor,const FBasePhysicsState& X)
{
 const auto& K=X.Kinematic;
 WriteHistoryScalars1564<156>(Cursor,
  K.Location.X,K.Location.Y,K.Location.Z,K.Velocity.X,K.Velocity.Y,K.Velocity.Z,
  K.Acceleration.X,K.Acceleration.Y,K.Acceleration.Z,
  K.Rotation.X,K.Rotation.Y,K.Rotation.Z,K.Rotation.W,
  K.AngularVelocity.X,K.AngularVelocity.Y,K.AngularVelocity.Z,
  K.AngularAcceleration.X,K.AngularAcceleration.Y,K.AngularAcceleration.Z,
  X.bIsFrozen,X.nbFramesbeforeCanMove,X.bStartCountdown);
}
inline void WriteWheelPrefixHistory1564(uint8*& Cursor,const FWheeledPhysicsState& X)
{
 static_assert(UE_ARRAY_COUNT(X.SuspensionLastDisplacement)==4);
 WriteHistoryScalars1564<80>(Cursor,X.bStartCountdown,X.nbFramesbeforeCanMove,X.FramesSinceLastImpact,
  X.AllowedSideVelocity.X,X.AllowedSideVelocity.Y,X.AllowedSideVelocity.Z,
  X.AllowedAngularVelocity.X,X.AllowedAngularVelocity.Y,X.AllowedAngularVelocity.Z,
  uint64(4),X.SuspensionLastDisplacement[0],X.SuspensionLastDisplacement[1],
  X.SuspensionLastDisplacement[2],X.SuspensionLastDisplacement[3],
  X.PhysicalThrottleInput,X.PhysicalBrakeInput,X.PhysicalSteerInput,X.NbFramesSinceGroundContact);
}
inline void WriteWheelWithBumpHistory1564(uint8*& Cursor,const FWheeledPhysicsState& X,uint32 Schema,uint8 Kind,uint64 StableId)
{
 static_assert(UE_ARRAY_COUNT(X.SuspensionLastDisplacement)==4);
 WriteHistoryScalars1564<95>(Cursor,X.bStartCountdown,X.nbFramesbeforeCanMove,X.FramesSinceLastImpact,
  X.AllowedSideVelocity.X,X.AllowedSideVelocity.Y,X.AllowedSideVelocity.Z,
  X.AllowedAngularVelocity.X,X.AllowedAngularVelocity.Y,X.AllowedAngularVelocity.Z,
  uint64(4),X.SuspensionLastDisplacement[0],X.SuspensionLastDisplacement[1],
  X.SuspensionLastDisplacement[2],X.SuspensionLastDisplacement[3],
  X.PhysicalThrottleInput,X.PhysicalBrakeInput,X.PhysicalSteerInput,X.NbFramesSinceGroundContact,
  Schema,Kind,StableId,X.CarBumpCooldownFrames);
}
}
// All source arrays are read-only; rejection checks precede destination growth.
inline bool AppendFrameHistory1564(TArray<uint8>& Out,TConstArrayView<int32> Values)
{
 static_assert(sizeof(int32)==4);
 Private::FHistoryAppendRange1564 Range(Out,Values.Num(),sizeof(int32),Values.GetData(),SIZE_T(Values.Num())*sizeof(int32));
 if(!Range.IsValid())return false;
 if(!Values.IsEmpty())
 {
  // int32 array has contiguous scalar representations and no object padding.
  FMemory::Memcpy(Range.Cursor(),Values.GetData(),SIZE_T(Values.Num())*sizeof(int32));
  Range.Commit(Values.Num()*sizeof(int32));
 }
 return true;
}
inline bool AppendBaseHistory1564(TArray<uint8>& Out,TConstArrayView<FBasePhysicsState> Values)
{
 Private::FHistoryAppendRange1564 Range(Out,Values.Num(),156,Values.GetData(),SIZE_T(Values.Num())*sizeof(FBasePhysicsState));
 if(!Range.IsValid())return false;
 for(const auto& X:Values){uint8* Cursor=Range.Cursor();Private::WriteBaseHistory1564(Cursor,X);Range.Commit(156);}
 return true;
}
inline bool AppendWheelPrefixHistory1564(TArray<uint8>& Out,TConstArrayView<FWheeledPhysicsState> Values)
{
 Private::FHistoryAppendRange1564 Range(Out,Values.Num(),80,Values.GetData(),SIZE_T(Values.Num())*sizeof(FWheeledPhysicsState));
 if(!Range.IsValid())return false;
 for(const auto& X:Values){uint8* Cursor=Range.Cursor();Private::WriteWheelPrefixHistory1564(Cursor,X);Range.Commit(80);}
 return true;
}
template<class Validate> bool AppendWheelBumpHistory1564(TArray<uint8>& Out,TConstArrayView<FWheeledPhysicsState> Values,
 uint32 Schema,Validate&& Identity,int32& FailureIndex)
{
 FailureIndex=INDEX_NONE;
 Private::FHistoryAppendRange1564 Range(Out,Values.Num(),95,Values.GetData(),SIZE_T(Values.Num())*sizeof(FWheeledPhysicsState));
 if(!Range.IsValid())return false;
 for(int32 Index=0;Index<Values.Num();++Index)
 {
  uint8 Kind=0;uint64 StableId=0;
  // Exactly one LIVE identity resolution immediately before this record.
  if(!Identity(Values[Index],Kind,StableId)){FailureIndex=Index;return false;}
  uint8* Cursor=Range.Cursor();Private::WriteWheelWithBumpHistory1564(Cursor,Values[Index],Schema,Kind,StableId);Range.Commit(95);
 }
 return true;
}

}
#endif
