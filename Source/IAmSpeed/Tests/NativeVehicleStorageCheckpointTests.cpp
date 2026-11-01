#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/NativeVehicleStorageCheckpoint.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeVehicleStorageCheckpoint,
    "IAmSpeed.Netcode.Prediction.NativeVehicleStorageCheckpoint",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeVehicleStorageCheckpoint::RunTest(const FString&)
{
    using Codec=Speed::Netcode::Prediction::FNativeVehicleStorageCheckpoint;
    Chaos::FSimpleWheelConfig WC;Chaos::FSimpleSuspensionConfig SC;
    SC.SetSuspensionMaxRaise(10);SC.SetSuspensionMaxDrop(25);SC.SuspensionSmoothing=7;
    SC.SpringRate=5000;SC.CompressionDamping=1000;SC.ReboundDamping=1200;
    Chaos::FSimpleWheeledVehicle Native,Reference;
    for(int32 I=0;I<4;++I)
    {
        auto& W=Native.Wheels.Emplace_GetRef(&WC);
        W.MaxOmega=1000;W.AppliedLinearDriveForce=0;W.AppliedLinearBrakeForce=0;
        W.LongitudinalAdhesiveLimit=0;W.LateralAdhesiveLimit=0;W.SideSlipModifier=WC.SideSlipModifier;W.WheelIndex=I;
        auto& S=Native.Suspension.Emplace_GetRef(&SC);S.SetSpringIndex(I);S.SetLocalRestingPosition(FVector(70-I*35,I%2?45:-45,-15));
        Reference.Wheels.Add(W);Reference.Suspension.Add(S);
    }
    auto Step=[](Chaos::FSimpleWheeledVehicle& V,int32 N)
    {
        for(int32 I=0;I<4;++I)
        {
            auto& S=V.Suspension[I];auto& W=V.Wheels[I];
            S.SetSuspensionLength(40+float((N*7+I*3)%23),30);S.SetLocalVelocity(FVector(N%17,I*3,-float(N%11)));
            S.Simulate(1.f/300.f);S.GetSpringLength();
            W.SetDriveTorque(N%90<50?1300.f:-500.f);W.SetBrakeTorque(N%70>50?1800.f:0.f);
            W.SetWheelLoadForce(N%60<47?S.GetSuspensionForce():0.f);
            W.SetVehicleGroundSpeed(FVector(float((N*13)%800)-300.f,float((N*3+I)%71)-35.f,0));
            W.SetSurfaceFriction(N%80<60?1.f:.25f);W.SetSteeringAngle(float(N%90)-45);W.Simulate(1.f/300.f);
        }
    };
    for(int32 N=0;N<41;++N){Step(Native,N);Step(Reference,N);}
    const auto Historical=Codec::Capture(Native);if(!TestTrue(TEXT("actual base-container state captured"),bool(Historical)))return false;
    TestTrue(TEXT("independent original native storage equal"),Codec::Capture(Reference)==Historical);
    auto* const WheelPointer=&Native.Wheels[2];auto* const SuspensionPointer=&Native.Suspension[2];
    const auto* const OriginalWheelConfig=WheelPointer->SetupPtr;const auto* const OriginalSuspensionConfig=SuspensionPointer->SetupPtr;
    for(int32 N=41;N<64;++N)Step(Native,N);
    const auto Changed=Codec::Capture(Native);TestTrue(TEXT("real physical target diverged"),Changed!=Historical);
    auto Plan=Codec::Prepare(Native,*Historical);if(!TestTrue(TEXT("all eight native child restores staged"),bool(Plan)))return false;
    TestTrue(TEXT("preparation leaves live native storage untouched"),Codec::Capture(Native)==Changed);
    std::vector<Speed::PreparedAccess::FParticipantAccess> Access;
    TestTrue(TEXT("actual base storage access declarations"),Codec::AppendPreparedAccess(Native,*Plan,51,Access));
    TestTrue(TEXT("no target/config or target/plan alias"),Speed::PreparedAccess::ValidateGlobalAccess(Access));
    auto Alias=Access.front();Alias.Participant=52;Alias.ReadOnly.clear();Alias.Writes={Access.front().ReadOnly.front()};Access.push_back(Alias);
    TestFalse(TEXT("configuration alias prevents transaction"),Speed::PreparedAccess::ValidateGlobalAccess(Access));
    TestFalse(TEXT("different live vehicle rejects plan"),Codec::CanCommit(Reference,*Plan));
    TestTrue(TEXT("live native base storage restored"),Codec::Commit(Native,*Plan));
    TestFalse(TEXT("plan single use"),Codec::CanCommit(Native,*Plan));
    TestTrue(TEXT("every native wheel and protected suspension field restored"),Codec::Capture(Native)==Historical);
    TestTrue(TEXT("wheel users retain original native address"),WheelPointer==&Native.Wheels[2]);
    TestTrue(TEXT("suspension users retain original native address"),SuspensionPointer==&Native.Suspension[2]);
    TestTrue(TEXT("wheel retains original configuration backing"),WheelPointer->SetupPtr==OriginalWheelConfig);
    TestTrue(TEXT("suspension retains original configuration backing"),SuspensionPointer->SetupPtr==OriginalSuspensionConfig);
    Plan.reset(); // The live native base objects must survive all prepared clones.
    for(int32 N=41;N<341;++N)
    {
        Step(Native,N);Step(Reference,N);
        TestTrue(TEXT("future native force and smoothing remain exact after plan retirement"),Codec::Capture(Native)==Codec::Capture(Reference));
    }
    const auto Final=Codec::Capture(Native);if(!Final)return false;
    auto Stale=Codec::Prepare(Native,*Final);if(!Stale)return false;
    Native.Wheels[0].ExternalBrakeTorque+=1;
    const auto BeforeRejection=Codec::Capture(Native);
    TestFalse(TEXT("changed native input rejects stale plan"),Codec::Commit(Native,*Stale));
    TestTrue(TEXT("failed precondition changes no native field"),Codec::Capture(Native)==BeforeRejection);
    auto Bad=*Final;Bad[4]=2;TestFalse(TEXT("version mismatch refused"),bool(Codec::Prepare(Native,Bad)));
    Bad=*Final;Bad.push_back(0);TestFalse(TEXT("trailing bytes refused"),bool(Codec::Prepare(Native,Bad)));
    for(size_t N=0;N<Final->size();N+=17)
        TestFalse(TEXT("truncated native storage checkpoint refused"),bool(Codec::Prepare(Native,Speed::Replay::FBytes(Final->begin(),Final->begin()+N))));
    Chaos::FSimpleEngineConfig EC;Native.Engine.Emplace(&EC);
    TestFalse(TEXT("additional live subsystem cannot be silently omitted"),bool(Codec::Capture(Native)));
    AddInfo(TEXT("Actual base native storage only; original wheel/suspension addresses/configuration survive correction and plan destruction. Component/subbody/contact/input/world restoration and real lifecycle lease remain required; no whole-world or network claim."));
    return !HasAnyErrors();
}
#endif
