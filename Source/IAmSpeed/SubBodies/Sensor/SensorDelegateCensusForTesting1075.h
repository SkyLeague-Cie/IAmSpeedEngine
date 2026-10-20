#pragma once
#include "CoreMinimal.h"
#if !UE_BUILD_SHIPPING
namespace Speed
{
struct FSensorDelegateSlot1075
{
 bool HandleValid=false, Bound=false, Compactable=false;
 bool ExpectedHandle=false, ExpectedObject=false;
};
struct FSensorDelegateCensus1075
{
 static constexpr int32 Limit=64;
 // -1 means unavailable, never an observed empty list or zero broadcast depth.
 int32 Stored=-1, Captured=0, BroadcastDepth=-1;
 bool Complete=false;
 bool PublicBound=false, PublicExpectedObjectBound=false;
 FSensorDelegateSlot1075 Slots[Limit];
 bool ExactEmpty() const { return Complete && BroadcastDepth==0 && Stored==0 && Captured==0; }
 bool ExactSingleExpected() const
 {
  if(!Complete || BroadcastDepth!=0 || Stored!=1 || Captured!=1)return false;
  const auto& S=Slots[0];
  return S.HandleValid && S.Bound && !S.Compactable && S.ExpectedHandle && S.ExpectedObject;
 }
};
// This immutable Engine makes its stored invocation list and lock count private.
// Public presence queries cannot prove multiplicity, exact methods, tombstones,
// broadcast depth or lifetime. Keep the full census UNKNOWN on this host.
// No access bypass, layout cast, cloned invocation list, Engine modification,
// binding mutation, compaction or Broadcast override is permitted here.
template<class NativeDelegate>
class TSensorDelegateCensus1075 : public NativeDelegate
{
public:
 FSensorDelegateCensus1075 ObserveForFrameWitness1075(
  FDelegateHandle Expected={},const UObject* ExpectedObject=nullptr) const
 {
  static_cast<void>(Expected); // Public API cannot inspect the expected handle.
  FSensorDelegateCensus1075 Out;
  Out.PublicBound=this->IsBound();
  Out.PublicExpectedObjectBound=ExpectedObject && this->IsBoundToObject(ExpectedObject);
  return Out;
 }
};
}
#endif
