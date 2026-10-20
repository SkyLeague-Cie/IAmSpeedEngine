#include "FrameParallelExecutor.h"
#include "FrameParallelSettings.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "HAL/PlatformTLS.h"
#include "Math/VectorRegister.h"
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFramePrivateWorkersTest,
 "IAmSpeed.Simulation.PrivateExecutor.BoundedWorkers",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFramePrivateWorkersTest::RunTest(const FString&)
{
 using namespace Speed;
 const int32 Workers=EnsurePrivateFrameWorkers(FFrameParallelPolicy::MaximumLanes-1);
 if(!TestTrue(TEXT("bounded private worker capacity is available"),Workers>0&&Workers<12))return false;
 const int32 Width=Workers+1;std::array<uint32,12> Ids{};std::array<int32,12> Nested{};
 std::mutex Mutex;std::condition_variable Arrived;int32 Count=0;std::atomic<bool> TimedOut{false};
 DispatchPrivateFrameParallelLanes(Width,[&](int32 Lane)
 {
  Ids[Lane]=FPlatformTLS::GetCurrentThreadId();
  {
   std::unique_lock<std::mutex> Lock(Mutex);
   if(++Count==Width)Arrived.notify_all();
   if(!Arrived.wait_for(Lock,std::chrono::seconds(10),[&]{return Count==Width;}))TimedOut=true;
  }
  // A nested call must stay on this lane rather than exhaust/wait on its pool.
  const uint32 Before=FPlatformTLS::GetCurrentThreadId();
  DispatchPrivateFrameParallelLanes(3,[&](int32){if(FPlatformTLS::GetCurrentThreadId()==Before)++Nested[Lane];});
 });
 TestFalse(TEXT("all admitted native workers reached the live barrier"),TimedOut.load());
 int32 Distinct=0;
 for(int32 I=0;I<Width;++I){bool Seen=false;for(int32 J=0;J<I;++J)Seen|=Ids[J]==Ids[I];Distinct+=!Seen;TestEqual(TEXT("nested joined work stays on its existing lane"),Nested[I],3);}
 TestEqual(TEXT("independent native threads, including caller, actually observed"),Distinct,Width);
 AddInfo(FString::Printf(TEXT("[IAmSpeedPrivateExecutorUnit] ConfiguredWorkers=%d LogicalLanes=%d ActualNative=%d EnginePoolResized=0"),Workers,Width,Distinct));
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFramePrivateExceptionTest,
 "IAmSpeed.Simulation.PrivateExecutor.JoinedExceptions",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFramePrivateExceptionTest::RunTest(const FString&)
{
 std::atomic<int32> Visited{0};bool Caught=false;
 try{Speed::DispatchPrivateFrameParallelLanes(12,[&](int32 Lane){++Visited;if(Lane==7)throw std::runtime_error("native joined exception fixture");});}
 catch(const std::runtime_error&){Caught=true;}
 TestTrue(TEXT("worker exception reaches owner after barrier"),Caught);
 TestEqual(TEXT("all accepted work finishes before exception leaves owner"),Visited.load(),12);
 std::atomic<int32> Subsequent{0};Speed::DispatchPrivateFrameParallelLanes(4,[&](int32){++Subsequent;});
 TestEqual(TEXT("joined failed group leaves no stale callbacks in next group"),Subsequent.load(),4);
 bool Invalid=false;try{Speed::DispatchPrivateFrameParallelLanes(13,[&](int32){++Subsequent;});}catch(const std::invalid_argument&){Invalid=true;}
 TestTrue(TEXT("capacity overflow rejected before work"),Invalid);TestEqual(TEXT("overflow writes nothing"),Subsequent.load(),4);
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFramePrivateNumericTest,
 "IAmSpeed.Simulation.PrivateExecutor.NumericEnvironment",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFramePrivateNumericTest::RunTest(const FString&)
{
 using namespace Speed;
 const auto Original=CaptureFrameFloatingPointContext();
 if(!TestTrue(TEXT("owner numeric context available"),Original.bValid))return false;
 FScopedFrameFloatingPointContext RestoreOwner(Original);
 std::fenv_t Held;if(std::feholdexcept(&Held)!=0||std::fesetround(FE_TONEAREST)!=0){AddError(TEXT("fixture numeric control unavailable"));return false;}
 auto Owner=CaptureFrameFloatingPointContext();Owner.VectorControl&=~uint32(VECTOR_DENORMALS_FLUSH_TO_ZERO);
#if PLATFORM_CPU_X86_FAMILY
 Owner.VectorControl&=~uint32(0x0040u);
#endif
 if(!ApplyFrameFloatingPointContext(Owner)){AddError(TEXT("owner fixture context unavailable"));return false;}
 std::array<uint32,12> Bits{};std::array<bool,12> Applied{},Restored{};
 DispatchPrivateFrameParallelLanes(12,[&](int32 Lane)
 {
  const auto Before=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext RestoreWorker(Before);
  std::fenv_t WorkerHeld;std::feholdexcept(&WorkerHeld);std::fesetround(FE_UPWARD);
  auto Poison=CaptureFrameFloatingPointContext();Poison.VectorControl|=VECTOR_DENORMALS_FLUSH_TO_ZERO;
#if PLATFORM_CPU_X86_FAMILY
  Poison.VectorControl|=0x0040u;
#endif
  ApplyFrameFloatingPointContext(Poison);const uint32 PoisonWord=VectorGetControlRegister();const int32 PoisonRound=std::fegetround();
  {
   FScopedFrameParallelTask Scope(Owner);Applied[Lane]=Scope.IsNumericContextApplied()&&std::fegetround()==FE_TONEAREST;
   volatile float Tiny=1.1754943508222875e-38f,Half=.5f;float Product;
#if PLATFORM_CPU_X86_FAMILY
   _mm_store_ss(&Product,_mm_mul_ss(_mm_set_ss(Tiny),_mm_set_ss(Half)));
#else
   Product=Tiny*Half;
#endif
   FMemory::Memcpy(&Bits[Lane],&Product,sizeof(Product));Restored[Lane]=Scope.RestoreNumericContext();
  }
  Restored[Lane]=Restored[Lane]&&VectorGetControlRegister()==PoisonWord&&std::fegetround()==PoisonRound;
 });
 for(int32 I=0;I<12;++I){TestTrue(TEXT("owner context applied on private lane"),Applied[I]);TestEqual(TEXT("real gradual arithmetic is preserved"),Bits[I],uint32(0x00400000u));TestTrue(TEXT("private lane's prior numeric environment restored"),Restored[I]);}
 return !HasAnyErrors();
}
#endif

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameOwnedBackendTest,
 "IAmSpeed.Simulation.PrivateExecutor.BackendOwnership",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameOwnedBackendTest::RunTest(const FString&)
{
 const auto Available=Speed::ConfigureFrameParallelExecutor(12,2);
 TestTrue(TEXT("automatic and explicit supported settings own their workers"),Available.bPrivateSelected&&Available.bReady&&Available.PrivateWorkers>0);
 TestEqual(TEXT("shared scheduler is not queried or used for capacity"),Available.TaskGraphWorkers,0);
 TestEqual(TEXT("only IAmSpeed capacity is exposed to frame policy"),Available.SelectedWorkers,Available.PrivateWorkers);
 std::atomic<int32> Count{0};
 const auto Backend=Speed::DispatchFrameParallelLanes(2,[&](int32){++Count;});
 TestTrue(TEXT("generic frame dispatch selects the owned executor"),Backend==Speed::EFrameExecutorBackend::PrivatePool);
 TestEqual(TEXT("generic joined dispatch executes each lane once"),Count.load(),2);
 return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFrameOwnedPairTest,
 "IAmSpeed.Simulation.PrivateExecutor.OwnedPair",
 EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FFrameOwnedPairTest::RunTest(const FString&)
{
 using namespace Speed;
 const uint32 Caller=FPlatformTLS::GetCurrentThreadId();uint32 Producer=0,Owner=0;
 std::mutex Mutex;std::condition_variable Ready;bool P=false,O=false,TimedOut=false;
 const auto Backend=DispatchPrivateFramePair([&]
 {
  Producer=FPlatformTLS::GetCurrentThreadId();std::unique_lock<std::mutex> Lock(Mutex);
  P=true;Ready.notify_all();if(!Ready.wait_for(Lock,std::chrono::seconds(10),[&]{return O;}))TimedOut=true;
 },[&]
 {
  Owner=FPlatformTLS::GetCurrentThreadId();std::unique_lock<std::mutex> Lock(Mutex);
  O=true;Ready.notify_all();if(!Ready.wait_for(Lock,std::chrono::seconds(10),[&]{return P;}))TimedOut=true;
 });
 TestTrue(TEXT("certified pair uses owned worker"),Backend==EFrameExecutorBackend::PrivatePool);
 TestTrue(TEXT("producer and owner actually overlap on distinct native threads"),!TimedOut&&Producer!=Caller&&Owner==Caller);
 std::atomic<int32> Finished{0};bool ProducerFirst=false;
 try
 {
  DispatchPrivateFramePair([&]{++Finished;throw std::runtime_error("producer first");},
    [&]{++Finished;throw std::runtime_error("owner second");});
 }
 catch(const std::runtime_error& Error){ProducerFirst=FCStringAnsi::Strcmp(Error.what(),"producer first")==0;}
 TestTrue(TEXT("producer exception retains canonical precedence"),ProducerFirst);
 TestEqual(TEXT("both accepted callbacks join before either error escapes"),Finished.load(),2);
 std::array<int32,2> NestedOrder{};std::array<bool,2> NestedError{},Skipped{};
 DispatchPrivateFrameParallelLanes(2,[&](int32 Lane)
 {
  int32 Order=0;
  const auto Nested=DispatchPrivateFramePair([&]{NestedOrder[Lane]=++Order;},[&]{NestedOrder[Lane]=NestedOrder[Lane]*10+(++Order);});
  NestedError[Lane]=Nested==EFrameExecutorBackend::SerialQueueFallback;
  bool OwnerCalled=false;
  try{DispatchPrivateFramePair([]{throw std::runtime_error("nested producer");},[&]{OwnerCalled=true;});}
  catch(const std::runtime_error&){Skipped[Lane]=!OwnerCalled;}
 });
 for(int32 Lane=0;Lane<2;++Lane)
 {
  TestEqual(TEXT("nested pair preserves original producer-before-owner order"),NestedOrder[Lane],12);
  TestTrue(TEXT("nested pair stays serial without queuing or deadlock"),NestedError[Lane]);
  TestTrue(TEXT("serial producer failure never invokes owner validation"),Skipped[Lane]);
 }
 AddInfo(FString::Printf(TEXT("[IAmSpeedOwnedPairUnit] Caller=%u Producer=%u Owner=%u ActualNative=2 Joined=1 UnrealScheduler=0"),Caller,Producer,Owner));
 return !HasAnyErrors();
}
#endif
