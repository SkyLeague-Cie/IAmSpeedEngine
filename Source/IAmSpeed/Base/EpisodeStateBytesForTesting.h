#pragma once
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
namespace Speed {
struct FEpisodeStateBytesForTesting {
 template<class V> static void Add(TArray<uint8>& Out,const V& Value)
 {Out.Append(reinterpret_cast<const uint8*>(&Value),sizeof(V));}
 template<class V> static void Array(TArray<uint8>& Out,const TArray<V>& Values)
 {Add(Out,Values.Num());if(Values.Num())Out.Append(reinterpret_cast<const uint8*>(Values.GetData()),Values.Num()*sizeof(V));}
}; }
#endif
