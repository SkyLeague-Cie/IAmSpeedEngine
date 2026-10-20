#pragma once
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
#include "FramePointerWitness.h"
namespace Speed
{
inline bool DeclareAuxiliaryFramePointer(FFramePointerRegistry& R,const void* Pointer,uint64 Actor,uint64 Role)
{
 if(!Pointer)return true;
 const UPTRINT Address=reinterpret_cast<UPTRINT>(Pointer);
 // Retain an existing semantic alias; never derive a key from its address.
 return R.Find(Address)!=nullptr || R.Register(Address,FFramePointerKey{102,Actor,0,Role,0});
}
inline bool FrameWitnessKeyLess(const FFramePointerKey& A,const FFramePointerKey& B)
{
 if(A.Domain!=B.Domain)return A.Domain<B.Domain;if(A.Owner!=B.Owner)return A.Owner<B.Owner;
 if(A.Generation!=B.Generation)return A.Generation<B.Generation;if(A.Role!=B.Role)return A.Role<B.Role;
 if(A.Element!=B.Element)return A.Element<B.Element;
 for(int32 I=0;I<FMath::Min(A.Membership.Num(),B.Membership.Num());++I)if(A.Membership[I]!=B.Membership[I])return A.Membership[I]<B.Membership[I];
 return A.Membership.Num()<B.Membership.Num();
}
}
#endif
