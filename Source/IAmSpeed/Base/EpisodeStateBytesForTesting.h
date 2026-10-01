#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"
#include "UObject/WeakObjectPtr.h"
#include <type_traits>
#if !UE_BUILD_SHIPPING
namespace Speed {
template<class V> struct FEpisodeFieldCodec; // no arbitrary-composite fallback
struct FEpisodeStateBytesForTesting {
 template<class V> static void Add(TArray<uint8>& O,const V& X) {
  if constexpr(std::is_integral_v<V> || std::is_same_v<V,float> || std::is_same_v<V,double>) {const V Scalar=X;O.Append(reinterpret_cast<const uint8*>(&Scalar),sizeof(Scalar));}
  else if constexpr(std::is_enum_v<V>) {Add(O,static_cast<std::underlying_type_t<V>>(X));}
  else if constexpr(std::is_pointer_v<V>) {Add(O,reinterpret_cast<UPTRINT>(X));}
  else {FEpisodeFieldCodec<V>::Write(O,X);}
 }
 template<class V> static void Add(TArray<uint8>& O,const TObjectPtr<V>& X){Add(O,X.Get());}
 // Standalone owners keep referenced actors alive. Stale weak references are
 // outside this certificate and stop rather than hash an unobservable serial.
 template<class V> static void Add(TArray<uint8>& O,const TWeakObjectPtr<V>& X){checkf(!X.IsStale(true,true),TEXT("Episode witness does not admit stale weak references"));Add(O,X.GetEvenIfUnreachable());Add(O,X.IsExplicitlyNull());}
 static void Add(TArray<uint8>& O,const FVector& X){Add(O,X.X);Add(O,X.Y);Add(O,X.Z);}
 static void Add(TArray<uint8>& O,const FQuat& X){Add(O,X.X);Add(O,X.Y);Add(O,X.Z);Add(O,X.W);}
 static void Add(TArray<uint8>& O,const FTransform& X){Add(O,X.GetTranslation());Add(O,X.GetRotation());Add(O,X.GetScale3D());}
 static void Add(TArray<uint8>& O,const FMatrix& X){for(int32 I=0;I<4;++I)for(int32 J=0;J<4;++J)Add(O,X.M[I][J]);}
 static void Add(TArray<uint8>& O,const FVector_NetQuantize& X){Add(O,X.X);Add(O,X.Y);Add(O,X.Z);}
 static void Add(TArray<uint8>& O,const FVector_NetQuantize10& X){Add(O,X.X);Add(O,X.Y);Add(O,X.Z);}
 static void Add(TArray<uint8>& O,const FVector_NetQuantize100& X){Add(O,X.X);Add(O,X.Y);Add(O,X.Z);}
 static void Add(TArray<uint8>& O,const FVector_NetQuantizeNormal& X){Add(O,X.X);Add(O,X.Y);Add(O,X.Z);}
 template<class V,SIZE_T N> static void Add(TArray<uint8>& O,const V (&X)[N]){Add(O,uint64(N));for(const auto& E:X)Add(O,E);}
 template<class V> static void Array(TArray<uint8>& O,const TArray<V>& X){Add(O,X.Num());for(const auto& E:X)Add(O,E);}
}; }
#endif
