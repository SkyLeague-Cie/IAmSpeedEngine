#include "IAmSpeed/World/Collision/PreparedSphereRequest.h"
#include "IAmSpeed/World/Collision/PreparedNativeSolidRequest.h"
#include "IAmSpeed/World/Collision/PreparedSweepEvidence.h"
#include <stdexcept>
#include "IAmSpeed/World/Simulation/SimulationWorker.h"
#include "HAL/PlatformProcess.h"
#include "HAL/Event.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPreparedPhysicalExecutionTest,
 "IAmSpeed.AnalyticWorld.Prepared.NativeExecutionAndSphereCaptureRefusals",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FPreparedPhysicalExecutionTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::PreparedExecution;
    FStepIdentity empty;empty.Frame=MAX_uint64;
    TestFalse(TEXT("unowned caller has no physical Step"),ReadCurrentStep(empty));
    TestTrue(TEXT("refusal clears caller output"),empty.Host==nullptr&&empty.Frame==0&&empty.StepSerial==0);
    empty.QuerySerial=MAX_uint64;TestFalse(TEXT("unhosted query cannot allocate an authenticated serial"),NextSphereQuery(empty));
    TestTrue(TEXT("serial refusal clears its output"),empty.QuerySerial==0&&empty.Lease==nullptr);
    FKinematicState a,b;TestTrue(TEXT("equal complete kinematic values retain exact bits"),SameKinematicBits(a,b));
    b.AngularVelocity.X=1;TestFalse(TEXT("angular velocity mutation cannot hide behind matching center PVA"),SameKinematicBits(a,b));
    b=a;b.Rotation.X=-0.;TestFalse(TEXT("signed zero in a quaternion is also a changed input bit"),SameKinematicBits(a,b));
    b=a;b.AngularAcceleration.Z=1;TestFalse(TEXT("angular acceleration is bound independently"),SameKinematicBits(a,b));
    FNativeSolidStaticValues solidA,solidB;
    TestTrue(TEXT("common solid values retain all initial bits"),SameNativeSolidStaticValues(solidA,solidB));
    solidB.PhysicalHalfExtent.X=1;TestFalse(TEXT("double physical Box extent is distinct from rounded query extent"),SameNativeSolidStaticValues(solidA,solidB));
    solidB=solidA;solidB.LocalCOM.Y=-0.;TestFalse(TEXT("Box COM signed bits are retained"),SameNativeSolidStaticValues(solidA,solidB));
    solidA.Query.DomainTolerance=0;solidB=solidA;solidB.Query.DomainTolerance=-0.;
    TestFalse(TEXT("native query comparison is bitwise, not cache-value equality"),SameNativeSolidStaticValues(solidA,solidB));
    solidB=solidA;solidB.Trace.IgnoredComponents.Add(123);
    TestFalse(TEXT("trace ignore membership cannot vanish behind matching masks"),SameNativeSolidStaticValues(solidA,solidB));
    solidB=solidA;solidB.Shape=ENativeSolidQueryShape::Box;
    TestFalse(TEXT("shape family cannot alias"),SameNativeSolidStaticValues(solidA,solidB));
    solidB=solidA;solidB.SphereMarginCaptured=true;
    TestFalse(TEXT("Sphere actual-member margin premise is explicit"),SameNativeSolidStaticValues(solidA,solidB));
    // Independent body/parent masses are both native inputs: an equal sum is
    // insufficient because the normal impulse and the parent update divide
    // by different values. Tensor off-diagonals must not be reconstructed.
    solidA.ResponseConfig.BodyMass=2;solidA.ResponseConfig.OwnerPhysMass=5;
    solidB=solidA;solidB.ResponseConfig.BodyMass=3;solidB.ResponseConfig.OwnerPhysMass=4;
    TestFalse(TEXT("equal total mass cannot alias distinct body and parent response inputs"),SameNativeSolidStaticValues(solidA,solidB));
    solidB=solidA;solidB.ResponseConfig.CachedBodyLocalInvInertia.M[0][1]=-0.;
    TestFalse(TEXT("cached off-diagonal signed bits survive without mass/extent reconstruction"),SameNativeSolidStaticValues(solidA,solidB));
    solidB=solidA;solidB.ResponseConfig.SphereProjectionPolicyCaptured=true;
    TestFalse(TEXT("uncaptured projection policy cannot stand in for a captured disabled policy"),SameNativeSolidStaticValues(solidA,solidB));
    FStepIdentity noSolid;noSolid.QuerySerial=MAX_uint64;
    TestFalse(TEXT("ordinary solid serial cannot be allocated outside FullFrame scope"),NextNativeSolidQuery(noSolid));
    TestTrue(TEXT("ordinary solid serial refusal clears output"),noSolid.QuerySerial==0);
    FNativeSolidStaticRequestPtr noSolidRequest;
    TestFalse(TEXT("unhosted common solid capture refuses before dereferencing interface/body"),CaptureNativeSolidCCD(
        nullptr,nullptr,nullptr,FVector::ZeroVector,FVector::ZeroVector,FCollisionShape::MakeBox(FVector(1,1,1)),FCollisionQueryParams(),1.f,noSolidRequest));
    TestFalse(TEXT("unhosted common solid capture emits no request"),noSolidRequest.IsValid());
    SHitResult rejectedWorld;rejectedWorld.bHit=true;rejectedWorld.bBlockingHit=true;rejectedWorld.TOI=1.f;
    bool ordinaryUnknown=false;
    try{(void)FComponentSweepScope::EvaluateOrdinaryNativeGround(nullptr,nullptr,nullptr,FVector::ZeroVector,FVector::ZeroVector,
        FCollisionShape::MakeBox(FVector(1,1,1)),FCollisionQueryParams(),1.f,rejectedWorld);}
    catch(const std::runtime_error&){ordinaryUnknown=true;}
    TestTrue(TEXT("unauthenticated ordinary World callback is Unknown, never a false-clear fallback"),ordinaryUnknown);
    TestFalse(TEXT("rejected callback clears a previously retained hit"),rejectedWorld.bHit||rejectedWorld.bBlockingHit);
    FNativeSphereRequestPtr request;
    TestFalse(TEXT("sphere reference cannot be published without authentic native scopes"),CaptureSphereCCD(nullptr,nullptr,FVector::ZeroVector,FVector::ZeroVector,FCollisionShape::MakeSphere(1),1.f,request));
    TestFalse(TEXT("unhosted capture leaves no stale request"),request.IsValid());
    TSharedPtr<const Analytic::Prepared::FScene,ESPMode::ThreadSafe> scene;TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe> filters;
    TestFalse(TEXT("unowned input access refuses"),ReadCurrentInputs(scene,filters));
    TestTrue(TEXT("unowned input access publishes neither scene nor filter"),!scene&&!filters);
    const USphereSubBody* memberBody=nullptr;const void* memberOwner=this;
    TestFalse(TEXT("raw body cannot be dereferenced outside an authentic member scope"),FSphereMemberSweepScope::ReadBody(nullptr,memberBody,memberOwner));
    TestTrue(TEXT("body identity refusal clears borrowed outputs"),memberBody==nullptr&&memberOwner==nullptr);
    uint64 memberSerial=17;float memberMargin=1;
    TestFalse(TEXT("unowned member margin read refuses"),ReadSphereMemberSweep(nullptr,nullptr,memberSerial,memberMargin));
    TestTrue(TEXT("unowned member margin read clears both outputs"),memberSerial==0&&memberMargin==0);
    for(float supported:{0.f,-0.f,0.01f})
        TestTrue(TEXT("finite nonnegative normal or zero response margin accepted as values only"),IsSupportedSphereResponseMargin(supported));
    for(uint32 bits:{0x80000001u,0x00000001u,0x80000000u|0x00800000u,0x7f800000u,0x7fc00000u})
    {
        float invalid=0;FMemory::Memcpy(&invalid,&bits,4);
        TestFalse(TEXT("subnormal negative nonfinite margin is never flushed into an accepted budget"),IsSupportedSphereResponseMargin(invalid));
    }
    FString primitiveReason;
    if(!TestTrue(TEXT("actual primitive value closure refuses altered returns and incomplete or stale pairs"),TestOrdinaryPrimitiveValueClosure(primitiveReason)))
    {AddError(primitiveReason);return false;}
    FEvent* completed=FPlatformProcess::GetSynchEventFromPool(true);
    if(!TestTrue(TEXT("real worker completion event available"),completed!=nullptr))return false;
    TAtomic<bool> laneMatched=false,reentrantRejected=false,reentrantPreserved=false,hostAbsent=false,closedOutsideWork=false,memberScopeAbsent=false;
    FSimulationWorker* expected=nullptr;
    {
        FSimulationWorker worker([&]()
        {
            laneMatched.Store(IsCurrentWorker(expected));
            uint64 serial=19;float margin=1;
            const bool refused=!ReadSphereMemberSweep(nullptr,nullptr,serial,margin)&&serial==0&&margin==0;
            memberScopeAbsent.Store(refused);
            try{FWorkerCallScope nested(*expected);}catch(const std::exception&){reentrantRejected.Store(true);}
            reentrantPreserved.Store(IsCurrentWorker(expected));
            FStepIdentity noHost;noHost.Frame=MAX_uint64;
            hostAbsent.Store(!ReadCurrentStep(noHost)&&noHost.Lease==nullptr&&noHost.Frame==0);
            return ESimulationWorkerResult::Complete;
        },[](FSimulationWorkerWaitContext&){}, {},[&]()
        {
            closedOutsideWork.Store(!IsCurrentWorker(expected));completed->Trigger();
        });
        expected=&worker;
        TestFalse(TEXT("game thread cannot borrow the worker pointer"),IsCurrentWorker(expected));
        const bool started=worker.Start();TestTrue(TEXT("real native worker starts"),started);
        const bool ended=started&&completed->Wait(10000);TestTrue(TEXT("bounded native worker closes"),ended);
        worker.StopAndJoin();
        TestTrue(TEXT("actual Work callback owns its exact native lane"),laneMatched.Load());
        TestTrue(TEXT("nested work scope is refused atomically"),reentrantRejected.Load()&&reentrantPreserved.Load());
        TestTrue(TEXT("a real worker alone cannot invent Host or Step authority"),hostAbsent.Load());
        TestTrue(TEXT("a real worker alone cannot invent a sphere-member margin scope"),memberScopeAbsent.Load());
        TestTrue(TEXT("boundary/close callback cannot retain the Work scope"),closedOutsideWork.Load());
        TestFalse(TEXT("joining never transfers worker TLS to game thread"),IsCurrentWorker(expected));
    }
    FPlatformProcess::ReturnSynchEventToPool(completed);
    return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand PreparedPhysicalExecutionUnit(TEXT("Speed.Analytic.RunPreparedPhysicalExecutionUnit"),
 TEXT("One native Work scope and unhosted sphere refusal unit; no World, actor, save or physical admission."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool used=false;auto& framework=FAutomationTestFramework::Get();const TCHAR* type=TEXT("FPreparedPhysicalExecutionTest");const TCHAR* name=TEXT("IAmSpeed.AnalyticWorld.Prepared.NativeExecutionAndSphereCaptureRefusals");
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticPreparedExecutionUnit"))||used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty()||!framework.ContainsTest(type))
    {UE_LOG(LogTemp,Error,TEXT("Prepared native execution unit refused"));return;}
    used=true;framework.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);framework.StartTestByName(type,0,name);
    if(!GIsAutomationTesting||!framework.GetCurrentTest()||framework.GetCurrentTestFullPath()!=name||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Prepared native synchronous test ownership refused"));return;}
    FAutomationTestExecutionInfo info;const bool stopped=framework.StopTest(info);
    if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||framework.GetCurrentTest()||!framework.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("Prepared native final test ownership refused"));return;}
    const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
    UE_LOG(LogTemp,Display,TEXT("ANALYTIC_PREPARED_EXECUTION_UNIT name=%s passed=%d errors=%d warnings=%d"),name,passed,info.GetErrorTotal(),info.GetWarningTotal());
 }));
}
