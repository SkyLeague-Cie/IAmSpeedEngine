#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/SubBodies/Sensor/SensorDelegateCensusForTesting1075.h"
#include "IAmSpeed/SubBodies/Sensor/SphereSensor.h"
#include "Misc/AutomationTest.h"
#include "UObject/UObjectGlobals.h"
#include "UObject/StrongObjectPtr.h"
namespace
{
DECLARE_MULTICAST_DELEGATE(FCensusNative1075);
using FCensusObserved1075=Speed::TSensorDelegateCensus1075<FCensusNative1075>;
static_assert(sizeof(FCensusObserved1075)==sizeof(FCensusNative1075));
static_assert(alignof(FCensusObserved1075)==alignof(FCensusNative1075));
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCensusSingle1075,
 "IAmSpeed.Physics.FrameWitness.SensorCensus1075.SingleAndEmpty",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FSensorCensusSingle1075::RunTest(const FString&)
{
 TStrongObjectPtr<UObject> Object(NewObject<USphereSensor>());FCensusObserved1075 D;
 TestFalse(TEXT("public emptiness cannot expose stored tombstones"),D.ObserveForFrameWitness1075().ExactEmpty());
 TestFalse(TEXT("new native delegate has no public binding"),D.IsBound());
 TStrongObjectPtr<USphereSensor> Sensor(NewObject<USphereSensor>());
 TestFalse(TEXT("actual sensor census remains unavailable"),Sensor->OnBeginOverlap.ObserveForFrameWitness1075().Complete);
 const auto ForeignSensor=Sensor->OnBeginOverlap.AddLambda([](USensorSubBody*,UPrimitiveComponent*){});
 const auto ActualSensor=Sensor->OnBeginOverlap.ObserveForFrameWitness1075();
 TestTrue(TEXT("real sensor lambda is publicly present"),ActualSensor.PublicBound);
 TestEqual(TEXT("private stored count is explicitly unknown"),ActualSensor.Stored,-1);
 TestFalse(TEXT("real sensor unknown lambda cannot qualify"),ActualSensor.ExactSingleExpected());
 Sensor->OnBeginOverlap.Remove(ForeignSensor);
 const auto H=D.AddUObject(Object.Get(),&UObject::ConditionalPostLoad);
 const auto C=D.ObserveForFrameWitness1075(H,Object.Get());
 TestTrue(TEXT("native public query sees the owner"),C.PublicExpectedObjectBound);
 TestFalse(TEXT("public owner presence is not an exact method certificate"),C.ExactSingleExpected());
 TestEqual(TEXT("private list is not fabricated from presence"),C.Stored,-1);
 TestFalse(TEXT("one binding is never empty"),C.ExactEmpty());
 D.Remove(H);TestFalse(TEXT("native removal clears public binding"),D.IsBound());
 TestFalse(TEXT("removal still cannot certify private storage"),D.ObserveForFrameWitness1075().ExactEmpty());
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCensusCopied1075,
 "IAmSpeed.Physics.FrameWitness.SensorCensus1075.CopiedHandleDuplicate",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FSensorCensusCopied1075::RunTest(const FString&)
{
 TStrongObjectPtr<UObject> Object(NewObject<USphereSensor>());FCensusObserved1075 D;
 auto Bound=FCensusNative1075::FDelegate::CreateUObject(Object.Get(),&UObject::ConditionalPostLoad);
 const auto H=Bound.GetHandle();D.Add(Bound);D.Add(Bound);
 const auto C=D.ObserveForFrameWitness1075(H,Object.Get());
 TestEqual(TEXT("copied duplicate count remains unavailable"),C.Stored,-1);
 TestFalse(TEXT("IsBoundToObject cannot hide a duplicate"),C.ExactSingleExpected());
 TestTrue(TEXT("native public presence is still true"),D.IsBoundToObject(Object.Get()));
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCensusWrong1075,
 "IAmSpeed.Physics.FrameWitness.SensorCensus1075.WrongMethodSameOwner",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FSensorCensusWrong1075::RunTest(const FString&)
{
 TStrongObjectPtr<UObject> Object(NewObject<USphereSensor>());FCensusObserved1075 D;
 const auto H=D.AddUObject(Object.Get(),&UObject::ConditionalPostLoad);D.Remove(H);
 D.AddUObject(Object.Get(),&UObject::ConditionalPreload);
 const auto C=D.ObserveForFrameWitness1075(H,Object.Get());
 TestTrue(TEXT("same object presence passes the inadequate old predicate"),D.IsBoundToObject(Object.Get()));
 TestEqual(TEXT("wrong method count remains unavailable"),C.Stored,-1);
 TestFalse(TEXT("private source-owned handle rejects substitution"),C.ExactSingleExpected());
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCensusForeign1075,
 "IAmSpeed.Physics.FrameWitness.SensorCensus1075.ForeignAndExpired",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FSensorCensusForeign1075::RunTest(const FString&)
{
 TStrongObjectPtr<UObject> Object(NewObject<USphereSensor>());FCensusObserved1075 D;
 const auto H=D.AddUObject(Object.Get(),&UObject::ConditionalPostLoad);
 const auto Foreign=D.AddLambda([]{});
 TestFalse(TEXT("foreign lambda cannot be omitted"),D.ObserveForFrameWitness1075(H,Object.Get()).ExactSingleExpected());
 D.Remove(Foreign);Object->MarkAsGarbage();
 TestFalse(TEXT("an expired/garbage real weak binding cannot certify live callback"),D.ObserveForFrameWitness1075(H,Object.Get()).ExactSingleExpected());
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCensusBroadcast1075,
 "IAmSpeed.Physics.FrameWitness.SensorCensus1075.BroadcastAndRemoval",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FSensorCensusBroadcast1075::RunTest(const FString&)
{
 FCensusObserved1075 D;bool Called=false;Speed::FSensorDelegateCensus1075 During;
 FDelegateHandle H;H=D.AddLambda([&]{Called=true;D.Remove(H);During=D.ObserveForFrameWitness1075();});
 D.Broadcast();
 TestTrue(TEXT("test exercised actual native Broadcast"),Called);
 TestEqual(TEXT("native broadcast lock is explicitly unavailable"),During.BroadcastDepth,-1);
 TestFalse(TEXT("mid-broadcast/tombstone census refuses completeness"),During.Complete);
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSensorCensusBound1075,
 "IAmSpeed.Physics.FrameWitness.SensorCensus1075.BoundAndObserverPurity",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FSensorCensusBound1075::RunTest(const FString&)
{
 FCensusObserved1075 D;int32 Calls=0;
 const auto H=D.AddLambda([&]{++Calls;});
 const auto Before=D.ObserveForFrameWitness1075(H);const auto Again=D.ObserveForFrameWitness1075(H);
 TestEqual(TEXT("read-only census does not invoke a callback"),Calls,0);
 TestEqual(TEXT("read-only census leaves native slot count"),Before.Stored,Again.Stored);
 D.Broadcast();TestEqual(TEXT("inherited Broadcast still invokes the native callback once"),Calls,1);
 for(int32 I=1;I<=Speed::FSensorDelegateCensus1075::Limit;++I)D.AddLambda([]{});
 const auto Overflow=D.ObserveForFrameWitness1075();
 TestEqual(TEXT("overflow cannot fabricate a private count"),Overflow.Stored,-1);
 TestTrue(TEXT("overflow fixture really has native bindings"),Overflow.PublicBound);
 TestFalse(TEXT("bounded observation refuses truncation PASS"),Overflow.Complete);
 TestEqual(TEXT("overflow captures no misleading prefix"),Overflow.Captured,0);
 return !HasAnyErrors();
}
#endif
