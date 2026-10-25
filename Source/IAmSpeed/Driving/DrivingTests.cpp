#include "DrivingInput.h"
#include "DrivingMenuInput.h"
#include "SpeedTestModules.h"
#include "SpeedDrivingSettings.h"
#include "IAmSpeed/SubBodies/Solid/RingSubBody.h"
#include "Engine/World.h"
#include "IAmSpeed/Input/ActionMapping.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDrivingRingEditorRegistration,"IAmSpeed.Driving.RingEditorRegistration",EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FDrivingRingEditorRegistration::RunTest(const FString&)
{
    const auto Init=UWorld::InitializationValues().AllowAudioPlayback(false).RequiresHitProxies(false).CreatePhysicsScene(true).ShouldSimulatePhysics(false).EnableTraceCollision(true).CreateNavigation(false).CreateAISystem(false);
    UWorld* World=UWorld::CreateWorld(EWorldType::Editor,false,NAME_None,nullptr,true,ERHIFeatureLevel::Num,&Init);
    if(!TestNotNull(TEXT("transient editor world"),World))return false;
    World->bEnableTraceCollision=true;
    ASpeedRingObstacle* Ring=World->SpawnActor<ASpeedRingObstacle>();
    if(TestNotNull(TEXT("ring registers with editor trace collision enabled"),Ring))
    {
        TestTrue(TEXT("canonical ring component registered"),Ring->Solid->IsRegistered());
        TestFalse(TEXT("no Chaos physics state required"),Ring->Solid->ShouldCreatePhysicsState());
        TestNull(TEXT("no approximate Chaos body substituted"),Ring->Solid->GetBodySetup());
        Speed::Analytic::FTorusSurface Torus;FString Error;
        TestTrue(TEXT("canonical torus remains admitted"),Ring->Solid->BuildStaticTorus(2737,Torus,Error));
        TestTrue(TEXT("authored full torus bounds remain finite"),Ring->Solid->Bounds.SphereRadius>1400);
    }
    World->DestroyWorld(false);
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDrivingPreferencesRoundTrip,"IAmSpeed.Driving.PreferencesRoundTrip",EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FDrivingPreferencesRoundTrip::RunTest(const FString&)
{
    auto* Before=NewObject<USpeedDrivingSettings>();Before->CameraDistance=925;Before->CameraHeight=230;Before->CameraFov=95;
    Before->CameraAngle=-9;Before->CameraStiffness=.8f;Before->CameraSwivelSpeed=4.1f;Before->bInvertCameraPitch=false;
    Before->WindowWidth=2560;Before->WindowHeight=1440;Before->MaximumFps=165;
    Before->Keyboard[0].Code=0x2C;Before->Gamepad[0].Code=uint16(Speed::Input::V2::EPadButton::RightShoulder);
    Before->Gamepad[0].Kind=int32(Speed::Input::V2::ERawControlKind::PadButton);++Before->MappingRevision;
    TArray<uint8> Bytes;TestTrue(TEXT("serialize actual save object"),UGameplayStatics::SaveGameToMemory(Before,Bytes));
    auto* After=Cast<USpeedDrivingSettings>(UGameplayStatics::LoadGameFromMemory(Bytes));
    TestTrue(TEXT("fresh loaded valid preferences"),After&&After!=Before&&After->IsValid());if(!After)return false;
    TestEqual(TEXT("camera distance survives restart data"),After->CameraDistance,Before->CameraDistance);
    TestEqual(TEXT("camera height survives"),After->CameraHeight,Before->CameraHeight);TestEqual(TEXT("FOV survives"),After->CameraFov,Before->CameraFov);
    TestEqual(TEXT("camera angle survives"),After->CameraAngle,Before->CameraAngle);TestEqual(TEXT("stiffness survives"),After->CameraStiffness,Before->CameraStiffness);
    TestEqual(TEXT("swivel speed survives"),After->CameraSwivelSpeed,Before->CameraSwivelSpeed);TestEqual(TEXT("pitch inversion survives"),After->bInvertCameraPitch,Before->bInvertCameraPitch);
    TestEqual(TEXT("window width survives"),After->WindowWidth,Before->WindowWidth);TestEqual(TEXT("window height survives"),After->WindowHeight,Before->WindowHeight);
    TestEqual(TEXT("FPS limit survives"),After->MaximumFps,165);TestEqual(TEXT("mapping revision survives"),After->MappingRevision,Before->MappingRevision);
    Speed::Driving::FBindings A,B;TestTrue(TEXT("old binding valid"),Before->ReadBindings(A));TestTrue(TEXT("loaded binding valid"),After->ReadBindings(B));
    TestTrue(TEXT("entire canonical mapping survives"),Speed::Driving::MakeContract(A)->GetFingerprint()==Speed::Driving::MakeContract(B)->GetFingerprint());
    After->MaximumFps=77;TestFalse(TEXT("corrupt or unsupported limit rejected"),After->IsValid());
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FStockDrivingContractTest,"IAmSpeed.Driving.StockContractAndHalfAxes",EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FStockDrivingContractTest::RunTest(const FString&)
{
    using namespace Speed::Driving;using namespace Speed::Input::V2;
    const auto Contract=MakeContract(FBindings{});TestTrue(TEXT("stock contract valid"),bool(Contract));if(!Contract)return false;
    const auto Evaluate=[&](float X,float Y)
    {
        FRawInputSample S;S.DeviceId=1;S.Generation={1};S.Kind=ERawDeviceKind::Gamepad;S.Sequence=1;S.Status=ERawSampleStatus::Resync;
        for(uint16 I=0;I<uint16(EPadButton::Count);++I)S.FinalState.push_back({{ERawControlKind::PadButton,I},0});
        for(uint16 I=0;I<uint16(EPadAxis::Count);++I)S.FinalState.push_back({{ERawControlKind::PadAxis,I},I==0?X:I==3?Y:0});
        const auto M=FActionMapper::Create(Contract,{1},{Speed::Input::EProducerKind::Device,1});return M->Map(S,0);
    };
    for(float X:{-1.f,-.5f,0.f,.5f,1.f})
    {
        const auto R=Evaluate(X,1);TestTrue(TEXT("mapped gamepad state"),R.Status==EMappingStatus::Mapped&&bool(R.Frame));
        if(R.Frame)
        {
            const auto V=R.Frame->GetData().Values;
            TestTrue(TEXT("left/right halves preserve steering sign"),X==0?V[Speed::Input::Steering]==0:X<0?V[Speed::Input::Steering]<0:V[Speed::Input::Steering]>0);
            TestEqual(TEXT("camera up has signed pitch"),int32(V[CameraPitch]),-127);
            if(FMath::Abs(X)==1)TestEqual(TEXT("full stick is not doubled"),FMath::Abs(int32(V[Speed::Input::Steering])),127);
        }
    }
    FRawInputSample Key;Key.DeviceId=2;Key.Generation={1};Key.Kind=ERawDeviceKind::Keyboard;
    Key.Sequence=1;Key.Status=ERawSampleStatus::Resync;
    // A Resync baseline declares every supported keyboard control, including
    // neutral driving keys. A P-only sample is not a complete mapper baseline.
    for(uint16 Code=4;Code<=0xE7;++Code)
    {
        const FRawControl Control{ERawControlKind::KeyboardUsage,Code};
        if(Control.IsValid())Key.FinalState.push_back({Control,Code==PauseKeyUsage?1.f:0.f});
    }
    TestTrue(TEXT("complete canonical keyboard baseline"),Key.IsValid());
    const auto KeyboardMapper=FActionMapper::Create(Contract,{2},{Speed::Input::EProducerKind::Device,2});
    const auto Pause=KeyboardMapper->Map(Key,0);
    TestTrue(TEXT("PIE pause key uses the same deterministic Pause action"),Pause.Frame&&Pause.Frame->GetData().Values[PauseAction]==1);
    auto Reserved=FBindings{};Reserved.Keyboard[0]={{ERawControlKind::KeyboardUsage,PauseKeyUsage},1};
    TestFalse(TEXT("pause key cannot also accelerate"),bool(MakeContract(Reserved)));
    auto Remap=FBindings{};Remap.Keyboard[0]={{ERawControlKind::KeyboardUsage,0x2C},1};++Remap.Revision;
    const auto Changed=MakeContract(Remap);TestTrue(TEXT("new immutable remap contract"),bool(Changed));
    TestTrue(TEXT("fingerprint differs after remap"),Changed&&Changed->GetFingerprint()!=Contract->GetFingerprint());
    Remap.Gamepad[4]={{ERawControlKind::PadAxis,0},1};TestFalse(TEXT("reset axis refused"),bool(MakeContract(Remap)));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDrivingPausedMenuTest,"IAmSpeed.Driving.PausedMenuJournal",EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FDrivingPausedMenuTest::RunTest(const FString&)
{
    using namespace Speed::Driving;using namespace Speed::Input::V2;
    auto J=std::make_shared<FRawAcquisitionJournal>(71);FMenuInput Reader(J);uint64 Seq=0;
    const auto Publish=[&](float Up,float Enter,bool Fresh=false,float PauseKey=0.f)
    {
        FRawAcquisitionBatch B;B.Count=1;auto& R=B.Readings[0];R.DeviceId=3;R.Sequence=++Seq;R.TimestampMicroseconds=Seq*1000;
        R.Generation={1};R.Kind=ERawDeviceKind::Keyboard;R.FreshBaseline=Fresh;R.State.Count=3;
        // Raw journal capabilities are strictly ordered by HID usage.
        R.State.Values[0]={{ERawControlKind::KeyboardUsage,PauseKeyUsage},PauseKey};
        R.State.Values[1]={{ERawControlKind::KeyboardUsage,0x28},Enter};R.State.Values[2]={{ERawControlKind::KeyboardUsage,0x52},Up};
        return J->Publish(J->BeginAcquisition(),B);
    };
    std::optional<FMenuInput::FCapture> Capture;
    TestTrue(TEXT("baseline"),Publish(1,0,true));TestTrue(TEXT("physical pause"),J->SetLifecyclePaused(true)==ELifecycleResult::Applied);
    TestTrue(TEXT("held on menu entry ignored"),Reader.Read(1,false,Capture).empty());
    TestTrue(TEXT("release during pause"),Publish(0,0));Reader.Read(1,false,Capture);
    TestTrue(TEXT("press while physics paused"),Publish(1,0));const auto First=Reader.Read(1,false,Capture);
    TestEqual(TEXT("one up command"),int32(First.size()),1);if(!First.empty())TestTrue(TEXT("up direction"),First[0].Action==EMenuAction::Up);
    TestFalse(TEXT("menu did not consume a physical frame"),bool(J->Poll(0)));
    TestTrue(TEXT("same history never replayed"),Reader.Read(1,false,Capture).empty());
    TestTrue(TEXT("new context suppresses held key"),Reader.Read(2,true,Capture).empty());TestFalse(TEXT("held key not captured"),bool(Capture));
    Publish(0,0);Reader.Read(2,true,Capture);Publish(0,1);Reader.Read(2,true,Capture);
    TestTrue(TEXT("capture carries real acquired key"),Capture&&Capture->Control.Code==0x28);
    for(int32 I=0;I<270;++I)TestTrue(TEXT("bounded journal feed"),Publish(0,0));
    TestTrue(TEXT("overflow discards edges"),Reader.Read(2,false,Capture).empty());
    Reader.Read(3,false,Capture);Publish(0,0,false,1);
    const auto Back=Reader.Read(3,false,Capture);
    TestTrue(TEXT("P returns through acquired paused journal"),Back.size()==1&&Back[0].Action==EMenuAction::Back);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FDrivingClosedRampTest,"IAmSpeed.Driving.ClosedRampPrisms",EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FDrivingClosedRampTest::RunTest(const FString&)
{
    auto* Ramp=GetMutableDefault<ASpeedRampModule>();const double Before=Ramp->TableLength;
    for(double Table:{0.,2200.})
    {
        Ramp->TableLength=Table;TArray<FVector> V;TArray<int32> T;TestTrue(TEXT("prism definition"),Ramp->BuildClosedPrism(V,T));
        TMap<uint64,int32> Edges;double Volume=0;
        for(int32 I=0;I<T.Num();I+=3)
        {
            Volume+=FVector::DotProduct(V[T[I]],FVector::CrossProduct(V[T[I+1]],V[T[I+2]]))/6.;
            for(int32 J=0;J<3;++J){const uint32 A=T[I+J],B=T[I+(J+1)%3];++Edges.FindOrAdd((uint64(FMath::Min(A,B))<<32)|FMath::Max(A,B));}
        }
        for(const auto& E:Edges)TestEqual(TEXT("watertight edge incidence"),E.Value,2);
        const double Expected=Ramp->Width*Ramp->Rise*(Table==0?Ramp->Run*.5:Ramp->Run+Table);
        TestTrue(TEXT("outward volume equals full solid prism"),FMath::IsNearlyEqual(Volume,Expected,.001));
        for(const auto& P:V)TestTrue(TEXT("module rests on ground"),P.Z>=0);
    }
    Ramp->TableLength=Before;return true;
}
#endif
