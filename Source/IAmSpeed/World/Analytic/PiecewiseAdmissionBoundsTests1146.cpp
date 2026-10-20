#if WITH_DEV_AUTOMATION_TESTS
#include "JoinedPiecewiseProviderWave1075.h"
#include "IAmSpeed/World/Collision/StaticCollisionWorld.h"
#include "HAL/IConsoleManager.h"
#include "Misc/AutomationTest.h"
namespace Speed::Analytic::ParallelTests1075
{ bool SameHit(const FWorldHit&,const FWorldHit&);bool PiecewiseWorld(FAnalyticWorldData&,FString&); }
namespace Speed::Analytic::AdmissionTests1146
{
struct FInt1146
{
 IConsoleVariable* V;int32 Old;
 FInt1146(const TCHAR* Name,int32 Value):V(IConsoleManager::Get().FindConsoleVariable(Name)),Old(V?V->GetInt():0){if(V)V->Set(Value,ECVF_SetByCode);}
 ~FInt1146(){if(V)V->Set(Old,ECVF_SetByCode);}
 bool Is(int32 Value)const{return V&&V->GetInt()==Value;}
};
bool SameNumeric1146(const Speed::FFrameFloatingPointContext& A,const Speed::FFrameFloatingPointContext& B)
{return A.bValid&&B.bValid&&A.VectorControl==B.VectorControl&&FMemory::Memcmp(&A.Environment,&B.Environment,sizeof(std::fenv_t))==0;}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedPiecewiseAdmissionBounds1146,
 "IAmSpeed.Parallel.Analytic1146.ProviderAdmissionBounds",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FIAmSpeedPiecewiseAdmissionBounds1146::RunTest(const FString& Parameters)
{
 using namespace Speed;using namespace Analytic;using namespace ParallelTests1075;using namespace AdmissionTests1146;
 FInt1146 Enabled(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderWave1075"),1);
 FInt1146 Admission(TEXT("p.IAmSpeed.Parallel.PiecewiseAdmissionBounds1146"),1);
 FInt1146 Observe(TEXT("p.IAmSpeed.Parallel.PiecewiseProviderObserve1075"),1);
 FInt1146 Causal(TEXT("p.IAmSpeed.Parallel.AnalyticCausal1075"),0);
 if(!Enabled.Is(1)||!Admission.Is(1)||!Observe.Is(1)||!Causal.Is(0))return false;
 const auto Entry=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext Restore(Entry);
 if(!Restore.IsApplied())return false;
 FAnalyticWorldData Data;FString Reason;if(!PiecewiseWorld(Data,Reason)){AddError(Reason);return false;}
 FAnalyticStaticCollisionWorld Serial(Data,1,2,2);
 for(int32 Lanes:{2,4,8,12})
 {
  FAnalyticStaticCollisionWorld Parallel(Data,Lanes,2,2);
  for(int32 Index=0;Index<48;++Index)
  {
   FWorldQuery Q;Q.Shape=EQueryShape(Index%3);Q.Radius=3;Q.HalfExtent=FVector3d(5,7,4);
   Q.Start=FVector3d((Index%5-2)*10,0,30);Q.End=FVector3d(Q.Start.X,0,-30);
   Q.bAuthorityOnly=true;Q.bIncludeCompactPatches=true;Q.DomainTolerance=0;Q.InitialOverlapTolerance=0;
   if(Index%4==0){Q.Start.Z=33;Q.End.Z=3;}
   if(Index%5==0){Q.Start=Q.End=FVector3d(0,0,1);}
   if(Index%7==0){Q.Start.X=-0.0;Q.End.X=-0.0;}
   if(!ApplyFrameFloatingPointContext(Entry))return false;
   const FWorldHit Expected=Serial.SweepSingle(Q);const auto ExpectedNumeric=CaptureFrameFloatingPointContext();
   if(!ApplyFrameFloatingPointContext(Entry))return false;
   const FWorldHit Actual=Parallel.SweepSingle(Q);const auto ActualNumeric=CaptureFrameFloatingPointContext();
   if(!SameHit(Expected,Actual)||!SameNumeric1146(ExpectedNumeric,ActualNumeric))
   {AddError(FString::Printf(TEXT("admission1146 full hit/FP divergence lanes=%d query=%d"),Lanes,Index));return false;}
  }
 }
 ResetPiecewiseProviderWaveStats1075();return TestTrue(TEXT("entry numeric context restored"),Restore.Restore());
}
#endif
