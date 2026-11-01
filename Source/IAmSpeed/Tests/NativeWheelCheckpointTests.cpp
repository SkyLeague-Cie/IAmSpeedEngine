#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/RestorableWheel.h"
#include "Misc/AutomationTest.h"
#include <new>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeWheelCheckpoint,
    "IAmSpeed.Netcode.Prediction.NativeWheelCheckpoint",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeWheelCheckpoint::RunTest(const FString&)
{
    using Speed::Netcode::Prediction::FRestorableWheel;
    Chaos::FSimpleWheelConfig Config;Config.ABSEnabled=true;Config.TractionControlEnabled=true;
    Chaos::FSimpleWheelSim Native(&Config);
    // These base members have no initializer in the pinned Engine constructor.
    // This fixture owns their explicit setup before copy/capture/simulation.
    Native.MaxOmega=1000;Native.AppliedLinearDriveForce=0;Native.AppliedLinearBrakeForce=0;
    Native.LongitudinalAdhesiveLimit=0;Native.LateralAdhesiveLimit=0;Native.SideSlipModifier=Config.SideSlipModifier;
    auto Step=[](Chaos::FSimpleWheelSim& W,int32 I)
    {
        W.SetDriveTorque(I%90<45?1500.f:-700.f);W.SetBrakeTorque(I%120>85?2000.f:0.f);
        W.SetVehicleGroundSpeed(FVector(float((I*17)%550)-200.f,float((I*13)%71)-35.f,0));
        W.SetWheelLoadForce(I%50<37?2700.f:0.f);W.SetSurfaceFriction(I%70<45?1.f:.3f);
        W.SetSteeringAngle(float(I%80)-40.f);W.Simulate(1.f/300.f);
    };
    for(int32 I=0;I<37;++I)Step(Native,I);
    FRestorableWheel Reference(Native),Restored(Native);
    const auto Origin=Reference.CaptureCheckpoint();if(!TestTrue(TEXT("full native wheel captured"),bool(Origin)))return false;
    Step(Restored,101);const auto Changed=Restored.CaptureCheckpoint();
    TestTrue(TEXT("wheel physically changed"),Changed!=Origin);
    const auto Plan=Restored.PrepareRestore(*Origin);if(!Plan)return false;
    TestTrue(TEXT("prepare does not modify wheel"),Restored.CaptureCheckpoint()==Changed);
    std::vector<Speed::PreparedAccess::FParticipantAccess> Access;
    TestTrue(TEXT("native child access ranges captured"),Restored.AppendPreparedAccess(*Plan,91,Access));
    TestTrue(TEXT("complete ranges exclude immutable configuration"),Speed::PreparedAccess::ValidateGlobalAccess(Access));
    TestTrue(TEXT("all preconditions validated before write"),Restored.CommitPrepared(*Plan));
    TestTrue(TEXT("all 44 dynamic fields restored exactly"),Restored.CaptureCheckpoint()==Origin);
    TestFalse(TEXT("single use plan"),Restored.CanCommit(*Plan));
    for(int32 I=37;I<637;++I)
    {
        Step(Native,I);Step(Reference,I);Step(Restored,I);
        TestEqual(TEXT("restored original native angular velocity"),Restored.Omega,Native.Omega);
        TestEqual(TEXT("restored original native longitudinal force"),Restored.ForceFromFriction.X,Native.ForceFromFriction.X);
        TestEqual(TEXT("restored original native lateral force"),Restored.ForceFromFriction.Y,Native.ForceFromFriction.Y);
        const FRestorableWheel NativeRead(Native);
        TestTrue(TEXT("all fields equal independently stepped original Engine"),NativeRead.CaptureCheckpoint()==Restored.CaptureCheckpoint());
        TestTrue(TEXT("all wheel state remains exact after drive/brake/slip/air"),Reference.CaptureCheckpoint()==Restored.CaptureCheckpoint());
    }
    const auto Final=Restored.CaptureCheckpoint();if(!Final)return false;
    for(size_t N=0;N<Final->size();++N)
        TestFalse(TEXT("every truncated wheel checkpoint refused"),bool(Restored.PrepareRestore(Speed::Replay::FBytes(Final->begin(),Final->begin()+N))));
    auto Bad=*Final;Bad.push_back(0);TestFalse(TEXT("trailing bytes refused"),bool(Restored.PrepareRestore(Bad)));
    Bad=*Final;Bad[4]=2;TestFalse(TEXT("unknown schema refused"),bool(Restored.PrepareRestore(Bad)));
    const auto Stale=Restored.PrepareRestore(*Final);if(!Stale)return false;
    Restored.ExternalBrakeTorque+=1;TestFalse(TEXT("native mutation invalidates staged plan"),Restored.CanCommit(*Stale));
    TestFalse(TEXT("wrong target refused"),Reference.CanCommit(*Stale));
    auto Nonempty=Config;Nonempty.LateralSlipGraph.Add(Chaos::FVec2(1,1));
    Chaos::FSimpleWheelSim GraphNative(Native);GraphNative.SetupPtr=&Nonempty;FRestorableWheel Unsupported(GraphNative);
    TestFalse(TEXT("opaque nonempty graph refused without approximation"),bool(Unsupported.CaptureCheckpoint()));
    Restored.AccessSetup().WheelMass+=1;TestFalse(TEXT("configuration mutation refused"),bool(Restored.CaptureCheckpoint()));
    alignas(FRestorableWheel) uint8 Storage[sizeof(FRestorableWheel)];auto* First=new(Storage)FRestorableWheel(Native);
    const auto Bytes=First->CaptureCheckpoint();if(!Bytes)return false;const auto Old=First->PrepareRestore(*Bytes);if(!Old)return false;
    First->~FRestorableWheel();auto* Second=new(Storage)FRestorableWheel(Native);
    TestFalse(TEXT("same-address new lifetime cannot accept old plan"),Second->CanCommit(*Old));Second->~FRestorableWheel();
    AddInfo(TEXT("Native wheel child checkpoint only. Actual Engine force algorithm; no predicted-world admission, native lease or network rollback claim."));
    return !HasAnyErrors();
}
#endif
