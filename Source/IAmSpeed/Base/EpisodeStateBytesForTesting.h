#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "UObject/WeakObjectPtr.h"
#include <type_traits>
#include "IAmSpeed/World/Simulation/FramePointerWitness.h"
#if !UE_BUILD_SHIPPING
namespace Speed {
template<class V> struct FEpisodeFieldCodec; // no arbitrary-composite fallback
struct FEpisodeStateBytesForTesting {
 template<class V> static void Add(TArray<uint8>& O,const V& X) {
  if constexpr(std::is_integral_v<V> || std::is_same_v<V,float> || std::is_same_v<V,double>) {const V Scalar=X;O.Append(reinterpret_cast<const uint8*>(&Scalar),sizeof(Scalar));}
  else if constexpr(std::is_enum_v<V>) {Add(O,static_cast<std::underlying_type_t<V>>(X));}
  else if constexpr(std::is_pointer_v<V>) {FFramePointerCaptureScope::Observe(O,reinterpret_cast<UPTRINT>(X));Add(O,reinterpret_cast<UPTRINT>(X));}
  else {FEpisodeFieldCodec<V>::Write(O,X);}
 }
 template<class V> static void Add(TArray<uint8>& O,const TObjectPtr<V>& X){Add(O,X.Get());}
 // Standalone owners keep referenced actors alive. Stale weak references are
 // outside this certificate and stop rather than hash an unobservable serial.
 template<class V> static void Add(TArray<uint8>& O,const TWeakObjectPtr<V>& X){checkf(!X.IsStale(true,true),TEXT("Episode witness does not admit stale weak references"));Add(O,X.GetEvenIfUnreachable());Add(O,X.IsExplicitlyNull());}
 static void Add(TArray<uint8>& O,const FVector& X){AddXYZ(O,X);}
 static void Add(TArray<uint8>& O,const FQuat& X){AddNumericFields(O,X.X,X.Y,X.Z,X.W);}
 static void Add(TArray<uint8>& O,const FTransform& X){Add(O,X.GetTranslation());Add(O,X.GetRotation());Add(O,X.GetScale3D());}
 static void Add(TArray<uint8>& O,const FMatrix& X){
  using S=std::remove_cvref_t<decltype(X.M[0][0])>;
  static_assert(std::is_same_v<S,float> || std::is_same_v<S,double>);
  S Values[16];for(int32 I=0;I<4;++I)for(int32 J=0;J<4;++J)Values[I*4+J]=X.M[I][J];
  static_assert(sizeof(Values)==16*sizeof(S));
  O.Append(reinterpret_cast<const uint8*>(Values),sizeof(Values));
 }
 static void Add(TArray<uint8>& O,const FVector_NetQuantize& X){AddXYZ(O,X);}
 static void Add(TArray<uint8>& O,const FVector_NetQuantize10& X){AddXYZ(O,X);}
 static void Add(TArray<uint8>& O,const FVector_NetQuantize100& X){AddXYZ(O,X);}
 static void Add(TArray<uint8>& O,const FVector_NetQuantizeNormal& X){AddXYZ(O,X);}
 template<class V,SIZE_T N> static void Add(TArray<uint8>& O,const V (&X)[N]){Add(O,uint64(N));for(const auto& E:X)Add(O,E);}
 template<class V> static void Array(TArray<uint8>& O,const TArray<V>& X){Add(O,X.Num());for(const auto& E:X)Add(O,E);}
private:
 // Only the explicit numeric overloads above use this helper. The local scalar
 // array has no UE object padding and adds no array-length prefix to the wire.
 template<class First,class... Rest> static void AddNumericFields(TArray<uint8>& O,const First& A,const Rest&... B){
  using S=std::remove_cvref_t<First>;
  static_assert(std::is_same_v<S,float> || std::is_same_v<S,double>);
  static_assert((std::is_same_v<S,std::remove_cvref_t<Rest>> && ...));
  const S Values[]={A,B...};static_assert(sizeof(Values)==(1+sizeof...(Rest))*sizeof(S));
  O.Append(reinterpret_cast<const uint8*>(Values),sizeof(Values));
 }
 template<class V> static void AddXYZ(TArray<uint8>& O,const V& X){AddNumericFields(O,X.X,X.Y,X.Z);}
}; }
#endif
