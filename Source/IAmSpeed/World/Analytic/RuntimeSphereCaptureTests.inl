#include "IAmSpeed/World/Simulation/CcdPoseAudit.h"
#include "IAmSpeed/Components/SpeedMovementComponent.h"
#include "IAmSpeed/SubBodies/Solid/SphereSubBody.h"
#include "UObject/StrongObjectPtr.h"
namespace
{
// Test-only selector lease, restored exactly; no runtime policy is weakened.
struct FAnalyticBackendFixtureScope
{
    struct FValue { IConsoleVariable* Variable=nullptr;int32 Value=0;EConsoleVariableFlags Flags=ECVF_Default; };
    TArray<FValue> Values;bool SetsSucceeded=true;
    static bool Set(IConsoleVariable* v,int32 value){v->ClearFlags(ECVF_SetByMask);v->Set(value,ECVF_SetByCode);return v->GetInt()==value;}
    FAnalyticBackendFixtureScope()
    {
        if(!IsInGameThread()){SetsSucceeded=false;return;}
        const TCHAR* names[]={TEXT("p.IAmSpeed.StaticCollision.Backend"),TEXT("p.IAmSpeed.AnalyticWorld.CompactAuthority"),TEXT("p.IAmSpeed.AnalyticWorld.Shadow")};
        for(const auto* name:names)
        {
            auto* v=IConsoleManager::Get().FindConsoleVariable(name);
            if(!v)continue;Values.Add({v,v->GetInt(),v->GetFlags()});SetsSucceeded=Set(v,0)&&SetsSucceeded;
        }
    }
    bool Ready() const{return Values.Num()==3&&SetsSucceeded&&IsInGameThread();}
    bool Backend(int32 backend,int32 shadow=0){return Ready()&&Set(Values[0].Variable,backend)&&Set(Values[2].Variable,shadow);}
    ~FAnalyticBackendFixtureScope()
    {
        for(const auto& v:Values){Set(v.Variable,v.Value);v.Variable->ClearFlags(EConsoleVariableFlags(MAX_uint32));v.Variable->SetFlags(v.Flags);}
    }
};
struct FPrimitiveCaptureFixture
{
    Speed::Testing::FCcdPrimitiveCapture Values[16];
    Speed::Testing::ECcdPoseAuditStage Stages[16];
    int32 Count=0;bool Overflow=false,Unexpected=false;
    USpeedMovementComponent* MutateOwnerOnBefore=nullptr;
    static void Observe(void* opaque,const Speed::Testing::FCcdPoseAuditSample& sample) noexcept
    {
        auto& self=*static_cast<FPrimitiveCaptureFixture*>(opaque);
        if(!sample.Primitive){self.Unexpected=true;return;}
        if(self.Count>=16){self.Overflow=true;return;}
        self.Values[self.Count]=*sample.Primitive;self.Stages[self.Count]=sample.Stage;++self.Count;
        if(self.MutateOwnerOnBefore && sample.Stage==Speed::Testing::ECcdPoseAuditStage::BeforePrimitiveIntegrate)
        {
            // Deliberately invalid observer on this OWN transient fixture only.
            // The producer must mark incomplete, never restore its old physics.
            auto state=self.MutateOwnerOnBefore->GetKinematicState();state.Acceleration.Z+=1.;
            self.MutateOwnerOnBefore->SetKinematicState(state);self.MutateOwnerOnBefore=nullptr;
        }
    }
};
struct FOwnedPrimitiveObserver
{
    FPrimitiveCaptureFixture& Fixture;bool Installed=false;
    explicit FOwnedPrimitiveObserver(FPrimitiveCaptureFixture& fixture):Fixture(fixture)
    {Installed=Speed::Testing::SetCcdPoseAuditObserverForCurrentThread(FPrimitiveCaptureFixture::Observe,&Fixture);}
    bool Close()
    {
        if(!Installed)return true;
        if(!Speed::Testing::ClearCcdPoseAuditObserverForCurrentThread(FPrimitiveCaptureFixture::Observe,&Fixture))return false;
        Installed=false;return true;
    }
    ~FOwnedPrimitiveObserver(){if(Installed)Close();}
};
bool PrimitiveStateBitsEqual(const SKinematic& a,const SKinematic& b)
{
    return FMemory::Memcmp(&a.Location,&b.Location,sizeof(a.Location))==0&&
        FMemory::Memcmp(&a.Velocity,&b.Velocity,sizeof(a.Velocity))==0&&
        FMemory::Memcmp(&a.Acceleration,&b.Acceleration,sizeof(a.Acceleration))==0&&
        FMemory::Memcmp(&a.Rotation,&b.Rotation,sizeof(a.Rotation))==0&&
        FMemory::Memcmp(&a.AngularVelocity,&b.AngularVelocity,sizeof(a.AngularVelocity))==0&&
        FMemory::Memcmp(&a.AngularAcceleration,&b.AngularAcceleration,sizeof(a.AngularAcceleration))==0;
}
bool PrimitivePVABitsEqual(const Speed::Testing::FCcdLinearStateValue& a,const Speed::Testing::FCcdLinearStateValue& b)
{
    for(int32 d=0;d<3;++d)if(AnalyticCertified::Bits(a.Position[d])!=AnalyticCertified::Bits(b.Position[d])||
        AnalyticCertified::Bits(a.Velocity[d])!=AnalyticCertified::Bits(b.Velocity[d])||AnalyticCertified::Bits(a.Acceleration[d])!=AnalyticCertified::Bits(b.Acceleration[d]))return false;
    return true;
}
SKinematic PrimitiveUnitState()
{
    // Mathematical integrator fixture only. No gameplay pose/profile, World,
    // Actor, ball class, contact/TOI, admission ticket or geometry is installed.
    SKinematic state;state.Location=FVector(10,-4,1);state.Velocity=FVector(128,3,-5);state.Acceleration=FVector(4,-8,0.125);return state;
}
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticPrimitiveCaptureLifecycleTest,
 "IAmSpeed.AnalyticWorld.RuntimeCapture.OptInOwnershipAndNoninterference",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticPrimitiveCaptureLifecycleTest::RunTest(const FString&)
{
    FTensorBoundsTestContext numeric;if(!TestTrue(TEXT("RNE fixture established"),numeric.Ready))return false;
    FAnalyticBackendFixtureScope selector;
    if(!TestTrue(TEXT("Explicit Legacy fixture for missing-World capture only"),selector.Ready()&&selector.Backend(0)))return false;
    using namespace Speed::Testing;
    TStrongObjectPtr<USpeedMovementComponent> observed(NewObject<USpeedMovementComponent>()),control(NewObject<USpeedMovementComponent>());
    observed->SetIsFrozen(false);control->SetIsFrozen(false);const auto initial=PrimitiveUnitState();observed->SetKinematicState(initial);control->SetKinematicState(initial);
    FPrimitiveCaptureFixture fixture,other;FOwnedPrimitiveObserver scope(fixture);
    if(!TestTrue(TEXT("same existing TLS observer installed"),scope.Installed))return false;
    TestFalse(TEXT("primitive capture disabled by default"),IsCcdPrimitiveCaptureEnabledForCurrentThread(observed.Get()));
    TestFalse(TEXT("conflicting observer registration refused"),SetCcdPoseAuditObserverForCurrentThread(FPrimitiveCaptureFixture::Observe,&other));
    TestFalse(TEXT("wrong context cannot enable capture"),EnableCcdPrimitiveCaptureForCurrentThread(FPrimitiveCaptureFixture::Observe,&other,observed.Get()));
    TestTrue(TEXT("owner opt-in uses adjusted ISpeedComponent identity"),EnableCcdPrimitiveCaptureForCurrentThread(FPrimitiveCaptureFixture::Observe,&fixture,observed.Get()));
    TestFalse(TEXT("another owner is not observed"),IsCcdPrimitiveCaptureEnabledForCurrentThread(control.Get()));
    const float delta=1.f/300.f;control->IntegrateKinematics(delta);TestEqual(TEXT("unselected actual integrator emits no added stages"),fixture.Count,0);
    observed->IntegrateKinematics(delta);TestEqual(TEXT("selected actual primitive emits exactly before/after"),fixture.Count,2);
    TestTrue(TEXT("opt-in observation preserves ALL actual kinematic output bits"),PrimitiveStateBitsEqual(observed->GetKinematicState(),control->GetKinematicState()));
    TestTrue(TEXT("no observer overflow or unrelated stages"),!fixture.Overflow&&!fixture.Unexpected);
    TestFalse(TEXT("wrong context cannot clear the owner observer"),ClearCcdPoseAuditObserverForCurrentThread(FPrimitiveCaptureFixture::Observe,&other));
    TestTrue(TEXT("original observer closes"),scope.Close());TestFalse(TEXT("primitive capture cleared with owner lifecycle"),IsCcdPrimitiveCaptureEnabledForCurrentThread(observed.Get()));
    TestTrue(TEXT("floating context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticPrimitiveCaptureActualDAGTest,
 "IAmSpeed.AnalyticWorld.RuntimeCapture.ActualPrimitivePairAndConsumedDelta",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticPrimitiveCaptureActualDAGTest::RunTest(const FString&)
{
    FTensorBoundsTestContext numeric;if(!TestTrue(TEXT("RNE fixture established"),numeric.Ready))return false;
    FAnalyticBackendFixtureScope selector;
    if(!TestTrue(TEXT("Explicit Legacy fixture for missing-World capture only"),selector.Ready()&&selector.Backend(0)))return false;
    using namespace Speed::Testing;using namespace Speed::Analytic::Certified;
    TStrongObjectPtr<USpeedMovementComponent> owner(NewObject<USpeedMovementComponent>());TStrongObjectPtr<USphereSubBody> sphere(NewObject<USphereSubBody>());
    owner->SetIsFrozen(false);owner->SetKinematicState(PrimitiveUnitState());sphere->Initialize(owner.Get());sphere->SetRelativeLocation(FVector::ZeroVector);sphere->SetKinematicState(owner->GetKinematicState());
    FPrimitiveCaptureFixture fixture;FOwnedPrimitiveObserver scope(fixture);if(!TestTrue(TEXT("observer installed"),scope.Installed))return false;
    TestTrue(TEXT("owner capture selected"),EnableCcdPrimitiveCaptureForCurrentThread(FPrimitiveCaptureFixture::Observe,&fixture,owner.Get()));
    const float queried=1.f/300.f,consumed=queried/2.f;const auto initial=owner->GetKinematicState();float ignoredTOI=queried;
    TestFalse(TEXT("actual public sphere sweep refuses null World; no contact authority from instrumentation"),sphere->SweepTOI(queried,ignoredTOI));
    owner->IntegrateKinematics(consumed);
    if(!TestEqual(TEXT("one actual sweep and one actual primitive pair"),fixture.Count,3))return false;
    const auto sweep=fixture.Values[0],before=fixture.Values[1],after=fixture.Values[2];
    TestTrue(TEXT("events occur exactly at requested producer and primitive sites"),fixture.Stages[0]==ECcdPoseAuditStage::SphereSweepCapture&&fixture.Stages[1]==ECcdPoseAuditStage::BeforePrimitiveIntegrate&&fixture.Stages[2]==ECcdPoseAuditStage::AfterPrimitiveIntegrate);
    TestTrue(TEXT("generation/owner/frame and positive serial preserved"),sweep.RunGeneration&&sweep.RunGeneration==before.RunGeneration&&before.RunGeneration==after.RunGeneration&&sweep.OwnerIdentity==before.OwnerIdentity&&before.OwnerIdentity==after.OwnerIdentity&&sweep.OwnerFrame==before.OwnerFrame&&before.OwnerFrame==after.OwnerFrame&&sweep.Serial<before.Serial&&before.Serial==after.Serial);
    TestTrue(TEXT("production sweep Start is actual captured subbody position"),FMemory::Memcmp(&sweep.QueryStart,&sweep.SubBody.Position,sizeof(FVector3d))==0);
    TestTrue(TEXT("sphere identity/radius actual; missing World explicit"),sweep.BodyIdentity==sphere.Get()&&sweep.IsSphereSubBody&&!sweep.IsWheelSubBody&&sweep.WorldWasObserved&&!sweep.HasWorld&&!before.WorldWasObserved&&!after.WorldWasObserved&&sweep.EffectiveSphereRadius==double(static_cast<const USSubBody*>(sphere.Get())->GetCollisionShape().GetSphereRadius()));
    TestTrue(TEXT("query and pre-consume PVA copied exactly"),PrimitivePVABitsEqual(sweep.SubBody,sweep.Owner)&&PrimitivePVABitsEqual(sweep.Owner,before.Owner));
    FZeroOffsetKinematicCenterReference model;TestTrue(TEXT("actual captured zero-offset inputs fit checked model"),BuildZeroOffsetKinematicCenterReference(sweep.SubBody.Position,sweep.SubBody.Velocity,sweep.SubBody.Acceleration,sweep.Delta,sweep.LocalBodyOffset,sweep.LocalCenterOfMass,model));
    const auto expected=BoundConsumedCenterAtFloatSeconds(model,before.Delta);
    TestTrue(TEXT("actual consumed delta is smaller; never rounded ratio substitution"),before.Delta==consumed&&before.Delta<sweep.Delta);
    for(int32 d=0;d<3;++d)TestTrue(TEXT("actual production primitive position is inside captured consumed DAG bound"),expected.Axis[d].Known&&expected.Axis[d].Lower<=after.Owner.Position[d]&&expected.Axis[d].Upper>=after.Owner.Position[d]);
    TestTrue(TEXT("actual primitive post position matches live owner output bits"),FMemory::Memcmp(&after.Owner.Position,&owner->GetKinematicState().Location,sizeof(FVector3d))==0);
    TestFalse(TEXT("readonly observer kept PVA/shape/FP unchanged"),CcdPrimitiveCaptureHadViolationForCurrentThread());
    TestTrue(TEXT("observer closure"),scope.Close());TestTrue(TEXT("floating context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FAnalyticPrimitiveCaptureMutationTest,
 "IAmSpeed.AnalyticWorld.RuntimeCapture.ChangedInputsAndCopiedLifetime",
 EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FAnalyticPrimitiveCaptureMutationTest::RunTest(const FString&)
{
    FTensorBoundsTestContext numeric;if(!TestTrue(TEXT("RNE fixture established"),numeric.Ready))return false;
    FAnalyticBackendFixtureScope selector;
    if(!TestTrue(TEXT("Explicit Legacy fixture for missing-World capture only"),selector.Ready()&&selector.Backend(0)))return false;
    using namespace Speed::Testing;
    TStrongObjectPtr<USpeedMovementComponent> owner(NewObject<USpeedMovementComponent>());TStrongObjectPtr<USphereSubBody> sphere(NewObject<USphereSubBody>());
    owner->SetIsFrozen(false);const auto initial=PrimitiveUnitState();owner->SetKinematicState(initial);sphere->Initialize(owner.Get());sphere->SetRelativeLocation(FVector::ZeroVector);sphere->SetKinematicState(initial);
    FPrimitiveCaptureFixture fixture;FOwnedPrimitiveObserver scope(fixture);if(!TestTrue(TEXT("observer installed"),scope.Installed))return false;EnableCcdPrimitiveCaptureForCurrentThread(FPrimitiveCaptureFixture::Observe,&fixture,owner.Get());
    const float delta=1.f/300.f;float ignoredTOI=delta;sphere->SweepTOI(delta,ignoredTOI);
    auto changed=initial;changed.Acceleration.Z+=1.;owner->SetKinematicState(changed);owner->IntegrateKinematics(delta);
    if(!TestEqual(TEXT("actual observation pair remains available for diagnosis"),fixture.Count,3))return false;
    TestFalse(TEXT("query inputs never falsely match changed effective pre-integration acceleration"),PrimitivePVABitsEqual(fixture.Values[0].Owner,fixture.Values[1].Owner));
    TestTrue(TEXT("saved sweep values did not alias mutable owner state"),AnalyticCertified::Bits(fixture.Values[0].Owner.Acceleration.Z)==AnalyticCertified::Bits(initial.Acceleration.Z));
    TestTrue(TEXT("observation records the effective changed acceleration"),AnalyticCertified::Bits(fixture.Values[1].Owner.Acceleration.Z)==AnalyticCertified::Bits(changed.Acceleration.Z));
    const uint64 oldGeneration=fixture.Values[0].RunGeneration;TestTrue(TEXT("first scope closes"),scope.Close());fixture.Count=0;
    FOwnedPrimitiveObserver next(fixture);TestTrue(TEXT("new run registration"),next.Installed);EnableCcdPrimitiveCaptureForCurrentThread(FPrimitiveCaptureFixture::Observe,&fixture,owner.Get());
    const double beforeInvalidCallback=owner->GetKinematicState().Acceleration.Z;fixture.MutateOwnerOnBefore=owner.Get();owner->IntegrateKinematics(delta);
    TestTrue(TEXT("mutating callback invalidates diagnostic without a false After pair"),fixture.Count==1&&CcdPrimitiveCaptureHadViolationForCurrentThread());
    TestEqual(TEXT("diagnostic never restores altered physics to forge a pass"),owner->GetKinematicState().Acceleration.Z,beforeInvalidCallback+1.);
    TestTrue(TEXT("invalid observer still closes safely"),next.Close());fixture.Count=0;
    FOwnedPrimitiveObserver finalScope(fixture);TestTrue(TEXT("fresh run after invalid observer"),finalScope.Installed);EnableCcdPrimitiveCaptureForCurrentThread(FPrimitiveCaptureFixture::Observe,&fixture,owner.Get());owner->IntegrateKinematics(delta);
    TestTrue(TEXT("serial reuse cannot alias an earlier run"),fixture.Count==2&&fixture.Values[0].RunGeneration>oldGeneration&&fixture.Values[0].Serial==fixture.Values[1].Serial&&!CcdPrimitiveCaptureHadViolationForCurrentThread());
    TestTrue(TEXT("final scope closes"),finalScope.Close());TestTrue(TEXT("floating context restored"),numeric.Restore.Restore());return !HasAnyErrors();
}
namespace
{
FAutoConsoleCommand RuntimeSphereCaptureUnits(TEXT("Speed.Analytic.RunRuntimeSphereCaptureUnits"),TEXT("Run three continuous-reference/consumed-DAG units; no World authority."),
 FConsoleCommandDelegate::CreateLambda([]()
 {
    static bool Used=false;auto& f=FAutomationTestFramework::Get();
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("AnalyticRuntimeSphereCaptureUnits"))||Used||GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Runtime sphere capture units refused"));return;}Used=true;
    const TCHAR* classes[]={TEXT("FAnalyticPrimitiveCaptureLifecycleTest"),TEXT("FAnalyticPrimitiveCaptureActualDAGTest"),TEXT("FAnalyticPrimitiveCaptureMutationTest")};
    const TCHAR* names[]={TEXT("IAmSpeed.AnalyticWorld.RuntimeCapture.OptInOwnershipAndNoninterference"),TEXT("IAmSpeed.AnalyticWorld.RuntimeCapture.ActualPrimitivePairAndConsumedDelta"),TEXT("IAmSpeed.AnalyticWorld.RuntimeCapture.ChangedInputsAndCopiedLifetime")};
    for(int32 i=0;i<3;++i)
    {
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()||!f.ContainsTest(classes[i])){UE_LOG(LogTemp,Error,TEXT("Runtime sphere capture unit ownership refused"));return;}
        f.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);f.StartTestByName(classes[i],0,names[i]);
        if(!GIsAutomationTesting||!f.GetCurrentTest()||f.GetCurrentTestFullPath()!=names[i]||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Runtime sphere capture unit synchronous ownership refused"));return;}
        FAutomationTestExecutionInfo info;const bool stopped=f.StopTest(info);
        if(GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty()){UE_LOG(LogTemp,Error,TEXT("Runtime sphere capture unit final ownership refused"));return;}
        const bool passed=stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0;
        UE_LOG(LogTemp,Display,TEXT("ANALYTIC_RUNTIME_SPHERE_CAPTURE_UNIT name=%s passed=%d errors=%d warnings=%d"),names[i],passed,info.GetErrorTotal(),info.GetWarningTotal());
    }
 }));
}
