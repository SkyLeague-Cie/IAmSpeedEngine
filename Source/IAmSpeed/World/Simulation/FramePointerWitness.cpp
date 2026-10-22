#include "FramePointerWitness.h"
#include "IAmSpeed/SubBodies/SSubBody.h"
#include "Components/PrimitiveComponent.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldData.h"
#if !UE_BUILD_SHIPPING
namespace Speed
{
namespace { thread_local FFramePointerCaptureScope* ActivePointerCapture=nullptr; }
FFramePointerCaptureScope::FFramePointerCaptureScope(TArray<uint8>& Bytes,
    const FFramePointerRegistry& Registry,const int32 InLimit)
    : Target(Bytes),Identities(Registry),Limit(FMath::Clamp(InLimit,1,16384))
{ Previous=ActivePointerCapture;ActivePointerCapture=this; }
FFramePointerCaptureScope::~FFramePointerCaptureScope()
{ check(ActivePointerCapture==this);ActivePointerCapture=Previous; }
void FFramePointerCaptureScope::Observe(TArray<uint8>& Bytes,const UPTRINT Address)
{
    FFramePointerCaptureScope* Scope=ActivePointerCapture;
    while(Scope && &Scope->Target!=&Bytes) Scope=Scope->Previous;
    if (!Scope) return;
    if (Scope->Events.Num()>=Scope->Limit) { Scope->bOverflow=true;return; }
    FFramePointerObservation E;
    E.Offset=Bytes.Num();E.RawAddress=Address;
    if (!Address) E.bKnown=true;
    else if (const FFramePointerKey* K=Scope->Identities.Find(Address))
    { E.Key=*K;E.bKnown=true; }
    else Scope->bUnknown=true;
    Scope->Events.Add(E);
}
void FFramePointerCaptureScope::ObserveObjectId(TArray<uint8>& Bytes,uint32 Id,const UObject* Object)
{
 FFramePointerCaptureScope* Scope=ActivePointerCapture;
 while(Scope && &Scope->Target!=&Bytes)Scope=Scope->Previous;
 if(!Scope)return;
 if(Scope->Events.Num()>=Scope->Limit){Scope->bOverflow=true;return;}
 FFramePointerObservation E;E.Offset=Bytes.Num();E.RawAddress=UPTRINT(Id);E.bObjectId=true;
 if(Object && Object->GetUniqueID()==Id)
  if(const auto* Key=Scope->Identities.Find(reinterpret_cast<UPTRINT>(Object))){E.Key=*Key;E.bKnown=true;}
 if(!E.bKnown)Scope->bUnknown=true;
 Scope->Events.Add(E);
}
void FFramePointerCaptureScope::ObservePairToken(TArray<uint8>& Bytes,uint64 Token,const USSubBody* Source,const UPrimitiveComponent* Other)
{
 static_assert(sizeof(UPTRINT)==sizeof(uint64));
 FFramePointerCaptureScope* Scope=ActivePointerCapture;
 while(Scope && &Scope->Target!=&Bytes)Scope=Scope->Previous;
 if(!Scope)return;
 if(Scope->Events.Num()>=Scope->Limit){Scope->bOverflow=true;return;}
 FFramePointerObservation E;E.Offset=Bytes.Num();E.RawAddress=UPTRINT(Token);E.bPairToken=true;
 if(!Token)E.bKnown=true;
 else if(Source && Other && Token==USSubBody::MakePairKey(Source,Other))
 {
  const auto* A=Scope->Identities.Find(reinterpret_cast<UPTRINT>(Source));
  const auto* B=Scope->Identities.Find(reinterpret_cast<UPTRINT>(Other));
  if(A && B)
  {
   E.Key=*A;E.SecondKey=*B;
   const auto Less=[](const FFramePointerKey& X,const FFramePointerKey& Y)
   {if(X.Domain!=Y.Domain)return X.Domain<Y.Domain;if(X.Owner!=Y.Owner)return X.Owner<Y.Owner;
    if(X.Generation!=Y.Generation)return X.Generation<Y.Generation;if(X.Role!=Y.Role)return X.Role<Y.Role;if(X.Element!=Y.Element)return X.Element<Y.Element;for(int32 I=0;I<FMath::Min(X.Membership.Num(),Y.Membership.Num());++I)if(X.Membership[I]!=Y.Membership[I])return X.Membership[I]<Y.Membership[I];return X.Membership.Num()<Y.Membership.Num();};
   if(Less(E.SecondKey,E.Key))Swap(E.Key,E.SecondKey);E.bKnown=true;
  }
 }
 if(!E.bKnown)Scope->bUnknown=true;
 Scope->Events.Add(E);
}
void FFramePointerCaptureScope::ObserveSolverPairToken(TArray<uint8>& Bytes,uint64 Token,
 const USSubBody* Source,const SHitResult& Hit,bool bStaticRestingWorld)
{
 const uint64 Provider=Hit.CanonicalGroupId!=0?Hit.CanonicalGroupId:Hit.SurfaceId;
 const uint64 Feature=bStaticRestingWorld && Hit.SurfaceId!=0 && !Hit.bSurfaceNormalMayVary
  && Hit.ContactFeatureThis==Speed::EContactFeatureKind::Vertex && Hit.ContactFeatureIndexThis>=0
  ? (0x504c414e00000000ull | static_cast<uint64>(Hit.ContactFeatureIndexThis+1)):0;
 if(!Provider && !Feature){ObservePairToken(Bytes,Token,Source,Hit.Component.Get());return;}
 FFramePointerCaptureScope* S=ActivePointerCapture;
 while(S && &S->Target!=&Bytes)S=S->Previous;
 if(!S)return;
 if(S->Events.Num()>=S->Limit){S->bOverflow=true;return;}
 FFramePointerObservation E;E.Offset=Bytes.Num();E.RawAddress=UPTRINT(Token);
 E.bPairToken=true;E.bSolverPairToken=true;E.PairProviderId=Provider;E.PairFeatureSalt=Feature;
 const auto* Other=Hit.Component.Get();
 if(Source && Other && Token)
 {
  uint64 Expected=USSubBody::MakePairKey(Source,Other);
  if(Provider)Expected=Speed::Analytic::CombineStableIds(Expected,Provider);
  if(Feature)Expected=Speed::Analytic::CombineStableIds(Expected,Feature);
  const auto* A=S->Identities.Find(reinterpret_cast<UPTRINT>(Source));
  const auto* B=S->Identities.Find(reinterpret_cast<UPTRINT>(Other));
  if(Expected==Token && A && B){E.Key=*A;E.SecondKey=*B;E.bKnown=true;}
 }
 if(!E.bKnown)S->bUnknown=true;
 S->Events.Add(E);
}
void FFramePointerCaptureScope::ObserveDynamicPairToken(TArray<uint8>& Bytes,uint64 Token,const USSubBody* Source,const USSubBody* Other)
{
 static_assert(sizeof(UPTRINT)==sizeof(uint64));
 FFramePointerCaptureScope* Scope=ActivePointerCapture;
 while(Scope && &Scope->Target!=&Bytes)Scope=Scope->Previous;
 if(!Scope)return;
 if(Scope->Events.Num()>=Scope->Limit){Scope->bOverflow=true;return;}
 FFramePointerObservation E;E.Offset=Bytes.Num();E.RawAddress=UPTRINT(Token);E.bPairToken=true;
 if(!Token)E.bKnown=true;
 else if(Source && Other && Token==((uint64(FMath::Min(Source->GetUniqueID(),Other->GetUniqueID()))<<32) | uint64(FMath::Max(Source->GetUniqueID(),Other->GetUniqueID()))))
 {
  const auto* A=Scope->Identities.Find(reinterpret_cast<UPTRINT>(Source));
  const auto* B=Scope->Identities.Find(reinterpret_cast<UPTRINT>(Other));
  if(A && B)
  {
   E.Key=*A;E.SecondKey=*B;
   const auto Less=[](const FFramePointerKey& X,const FFramePointerKey& Y)
   {if(X.Domain!=Y.Domain)return X.Domain<Y.Domain;if(X.Owner!=Y.Owner)return X.Owner<Y.Owner;
    if(X.Generation!=Y.Generation)return X.Generation<Y.Generation;if(X.Role!=Y.Role)return X.Role<Y.Role;if(X.Element!=Y.Element)return X.Element<Y.Element;for(int32 I=0;I<FMath::Min(X.Membership.Num(),Y.Membership.Num());++I)if(X.Membership[I]!=Y.Membership[I])return X.Membership[I]<Y.Membership[I];return X.Membership.Num()<Y.Membership.Num();};
   if(Less(E.SecondKey,E.Key))Swap(E.Key,E.SecondKey);E.bKnown=true;
  }
 }
 if(!E.bKnown)Scope->bUnknown=true;
 Scope->Events.Add(E);
}
namespace
{
bool MakeCapturedPointerCanonical1456(const TArray<uint8>& Target,
    const TArray<FFramePointerObservation>& Events,bool bValidSealed1456,
    bool bUnknown1456,bool bOverflow1456,TArray<uint8>& Canonical,
    TConstArrayView<int32> RawBoundaries={},TArray<int32>* ProjectedBoundaries=nullptr)
{
    if (!bValidSealed1456 || bUnknown1456 || bOverflow1456) return false;
    TArray<uint8> Out;
    int32 Next=0,BoundaryIndex=0;TArray<int32> Mapped;
    if(ProjectedBoundaries)
    {
        int32 Previous=-1;for(const int32 B:RawBoundaries){if(B<Previous||B<0||B>Target.Num())return false;Previous=B;}
        Mapped.Reserve(RawBoundaries.Num());
    }
    const auto Add=[&Out](const auto& V)
    { Out.Append(reinterpret_cast<const uint8*>(&V),sizeof(V)); };
    for (const FFramePointerObservation& E:Events)
    {
        const int32 Width=E.bObjectId?int32(sizeof(uint32)):int32(sizeof(UPTRINT));
        if (!E.bKnown || E.Offset<Next || E.Offset>Target.Num()-Width) return false;
        UPTRINT Actual=0;FMemory::Memcpy(&Actual,Target.GetData()+E.Offset,Width);
        if (Actual!=E.RawAddress) return false;
        if(ProjectedBoundaries)
        {
            while(BoundaryIndex<RawBoundaries.Num()&&RawBoundaries[BoundaryIndex]<=E.Offset)
                Mapped.Add(Out.Num()+RawBoundaries[BoundaryIndex++]-Next);
            if(BoundaryIndex<RawBoundaries.Num()&&RawBoundaries[BoundaryIndex]<E.Offset+Width)return false;
        }
        Out.Append(Target.GetData()+Next,E.Offset-Next);
        if(E.bObjectId)Add(uint32(0xFFFFFFFDu));
        if(E.bSolverPairToken)Add(uint32(0xFFFFFFFCu));
        else if(E.bPairToken)Add(uint32(0xFFFFFFFEu));
        Add(E.Key.Domain);Add(E.Key.Owner);Add(E.Key.Generation);Add(E.Key.Role);Add(E.Key.Element);Add(uint32(E.Key.Membership.Num()));for(uint64 Id:E.Key.Membership)Add(Id);
        if(E.bPairToken){Add(E.SecondKey.Domain);Add(E.SecondKey.Owner);Add(E.SecondKey.Generation);Add(E.SecondKey.Role);Add(E.SecondKey.Element);Add(uint32(E.SecondKey.Membership.Num()));for(uint64 Id:E.SecondKey.Membership)Add(Id);}
        if(E.bSolverPairToken){Add(E.PairProviderId);Add(E.PairFeatureSalt);}
        Next=E.Offset+Width;
    }
    if(ProjectedBoundaries)while(BoundaryIndex<RawBoundaries.Num())Mapped.Add(Out.Num()+RawBoundaries[BoundaryIndex++]-Next);
    if(Target.Num()>Next)Out.Append(Target.GetData()+Next,Target.Num()-Next);
    Canonical=MoveTemp(Out);if(ProjectedBoundaries)*ProjectedBoundaries=MoveTemp(Mapped);return true;
}
}
bool FFramePointerCaptureScope::MakeCanonical(TArray<uint8>& Canonical) const
{
    return MakeCapturedPointerCanonical1456(Target,Events,Identities.IsValidSealed(),bUnknown,bOverflow,Canonical);
}
#if WITH_DEV_AUTOMATION_TESTS
bool FFramePointerCaptureScope::MakeCanonicalWithBoundaries1534(TArray<uint8>& Out,TConstArrayView<int32> RawBoundaries,TArray<int32>& CanonicalBoundaries) const
{
 return MakeCapturedPointerCanonical1456(Target,Events,Identities.IsValidSealed(),bUnknown,bOverflow,Out,RawBoundaries,&CanonicalBoundaries);
}
bool FFramePointerCanonicalSnapshot1456::MakeCanonicalWithBoundaries1534(TArray<uint8>& Out,TConstArrayView<int32> RawBoundaries,TArray<int32>& CanonicalBoundaries) const
{
 return bCaptured1456&&MakeCapturedPointerCanonical1456(Raw1456,Events1456,bValidSealed1456,bUnknown1456,bOverflow1456,Out,RawBoundaries,&CanonicalBoundaries);
}
void FFramePointerCanonicalSnapshot1456::Reserve1456(int32 Bytes,int32 Count)
{
    Raw1456.Reserve(FMath::Clamp(Bytes,0,16*1024*1024));
    Events1456.Reserve(FMath::Clamp(Count,0,16384));
}
bool FFramePointerCanonicalSnapshot1456::ReserveAndPreTouch1456(int32 Bytes,int32 Count)
{
    if(bCaptured1456||!Raw1456.IsEmpty()||!Events1456.IsEmpty())return false;
    Raw1456.SetNumZeroed(FMath::Clamp(Bytes,0,16*1024*1024));
    Raw1456.Reset();
    Events1456.Reserve(FMath::Clamp(Count,0,16384));
    return true;
}
bool FFramePointerCaptureScope::CaptureCanonicalSnapshot1456(FFramePointerCanonicalSnapshot1456& Out) const
{
    if(!Identities.IsValidSealed()||bUnknown||bOverflow)return false;
    Out.Raw1456=Target;Out.Events1456=Events;
    Out.bValidSealed1456=true;Out.bUnknown1456=bUnknown;Out.bOverflow1456=bOverflow;Out.bCaptured1456=true;
    return true;
}
bool FFramePointerCanonicalSnapshot1456::MakeCanonical(TArray<uint8>& Out) const
{
    return bCaptured1456&&MakeCapturedPointerCanonical1456(Raw1456,Events1456,bValidSealed1456,bUnknown1456,bOverflow1456,Out);
}
#endif
}
#endif
