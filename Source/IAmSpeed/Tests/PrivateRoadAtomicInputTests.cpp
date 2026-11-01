#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPartialStateComparison.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"

namespace IASAtomicRoadCampaign
{
namespace P=Speed::Netcode::Prediction;
namespace V=Speed::Input::V2;
using namespace Speed::Netcode;
struct FCase { const TCHAR* Name; unsigned Action; int16 Value; uint16 LocalCountdown=0,AuthorityCountdown=0; bool ExpectMismatch=false; bool FromRest=false; uint16 HalfPeriod=0,FrameCount=121; };
struct FMaxima
{
    double Position=0,Velocity=0,Acceleration=0,Rotation=0,AngularVelocity=0,AngularAcceleration=0;
    int64 FirstDifferent=-1; uint32 DifferentFrames=0;
    bool Add(const SKinematic& A,const SKinematic& B,U64 Frame)
    {
        const auto D=P::MeasurePrivateRoadKinematics(A,B);if(!D.Valid)return false;
        Position=FMath::Max(Position,D.Position);Velocity=FMath::Max(Velocity,D.Velocity);
        Acceleration=FMath::Max(Acceleration,D.Acceleration);Rotation=FMath::Max(Rotation,D.RotationRadians);
        AngularVelocity=FMath::Max(AngularVelocity,D.AngularVelocity);AngularAcceleration=FMath::Max(AngularAcceleration,D.AngularAcceleration);
        if(D.Different){if(FirstDifferent<0)FirstDifferent=int64(Frame);++DifferentFrames;}return true;
    }
};
bool RunCase(FAutomationTestBase& Test,const FCase& Case)
{
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    P::FPrivateRoadCarSeed Seed;Seed.StartPolicy=P::EPrivateRoadStartPolicy::ExplicitUnitRecipe;Seed.Actor=101;Seed.Incarnation=3;Seed.Contract=Contract;
    Seed.Local={Speed::Input::EProducerKind::Device,42};Seed.LocalEpoch={3};
    Seed.Remote.Session=1;Seed.Remote.Epoch=2;Seed.Remote.Actor=101;Seed.Remote.Incarnation=3;
    Seed.Remote.OwnerPeer=4;Seed.Remote.ServerPeer=5;Seed.Remote.SourceProducer=42;Seed.Remote.SourceKind=0;Seed.Remote.Contract[0]=7;
    Seed.Initial.Location=FVector(0,0,45);Seed.Initial.Velocity=FVector(Case.FromRest?0:500,0,0);
    // Identical processing, including real slew state, on each genuine role.
    Seed.Processing.Step[0]=17;Seed.Processing.Step[1]=17;Seed.Processing.Step[2]=9;
    Seed.Role=P::EBranchFilterRole::LocalDevice;Seed.CountdownFrames=Case.LocalCountdown;
    FString Why;auto Local=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    Seed.Role=P::EBranchFilterRole::RemoteRequests;Seed.CountdownFrames=Case.AuthorityCountdown;
    auto Authority=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    auto Neutral=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    if(!Test.TestTrue(*FString::Printf(TEXT("%s native three-world creation: %s"),Case.Name,*Why),bool(Local)&&bool(Authority)&&bool(Neutral)))return false;
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    Actions[5]={true,true,true,127};Actions[6]={true,true,true,127};
    // Retain the complete finite scenario: oscillations run613frames, unlike
    // the original121-frame campaign. Never evict history or relax its guards.
    const size_t Retention=FMath::Max<size_t>(512,size_t(Case.FrameCount));
    P::FRemoteRequestHistory H(Seed.Remote,Actions,256,Retention),Z(Seed.Remote,Actions,256,Retention);
    uint32 PartialMissing=0,PartialDifferent=0,PartialResponse=0;
    FMaxima Difference,Response;uint32 FilterDifferences=0,StartupDifferences=0;int64 FirstLocalCanMove=-1,FirstAuthorityCanMove=-1;
    const auto Body=[](const P::FPrivateRoadPublication& Pub)->const FSimulationPresentationBody*
    {return Pub.Actors.size()==1?Pub.Physics.PresentationBodies.FindByPredicate([&](const auto& X){return X.StableId==Pub.Actors[0].NativeStableId;}):nullptr;};
    const auto ValueAt=[&](U64 N)->int16
    {
        if(!Case.HalfPeriod)return N>=6&&N<90?Case.Value:0;
        if(N<6||N>=U64(Case.FrameCount-6))return 0;
        const int16 Minimum=Case.Action==0||Case.Action==1?0:int16(-Case.Value);
        return ((N-6)/Case.HalfPeriod)%2?Minimum:Case.Value;
    };
    for(U64 N=0;N<Case.FrameCount;++N)
    {
        // Six neutral frames, onset, held analog level, release: no mixing axes.
        const int16 Value=ValueAt(N);const uint32 Mask=Value?(uint32(1)<<Case.Action):0;
        V::FInputFrameData D;D.ConsumptionFrame=N;D.SourceSequence=N+1;D.DeviceGeneration={1};D.Producer=Seed.Local;
        D.StreamEpoch=Seed.LocalEpoch;D.Reset=N==0;D.Values[Case.Action]=Value;D.ActiveMask=Mask;
        if(Case.HalfPeriod&&N&&bool(ValueAt(N-1))!=bool(Value))
            D.Transitions.push_back({Speed::Input::FActionId(Case.Action),Value?V::ETransition::Started:V::ETransition::Completed,Value,{N+1,0}});
        auto Input=std::make_shared<const V::FInputFrame>(Contract,D);
        const auto Packet=[&](bool Zero)
        {
            const auto& B=Seed.Remote;FPacket Q;Q.Kind=EKind::AuthorityInput;Q.Session=B.Session;Q.Epoch=B.Epoch;
            Q.Actor=B.Actor;Q.Incarnation=B.Incarnation;Q.Contract=B.Contract;Q.Frame=N;Q.Sequence=N+1;
            Q.Input.SourceSequence=N+1;Q.Input.DeviceGeneration=1;Q.Input.ProducerId=B.SourceProducer;Q.Input.ProducerKind=B.SourceKind;
            Q.Input.Reset=N==0;Q.Input.Values[Case.Action]=Zero?0:Value;Q.Input.ActiveMask=Zero?0:Mask;
            if(Case.HalfPeriod&&!Zero&&N&&bool(ValueAt(N-1))!=bool(Value))
                Q.Input.Edges.push_back({uint8(Case.Action),uint8(Value?0:1),Value,N+1,0});
            return Q;
        };
        const auto HC=H.Confirm(5,Packet(false)).Status,ZC=Z.Confirm(5,Packet(true)).Status;
        if(HC!=EResult::Accepted||ZC!=EResult::Accepted)
        {
            Test.AddError(FString::Printf(TEXT("%s frame=%llu retained=%llu remoteConfirm=%d neutralConfirm=%d: no physical comparison inferred"),
                Case.Name,N,uint64(Retention),int(HC),int(ZC)));return false;
        }
        if(!Local->Step({{101,3,Input}},false,Why)||!Authority->Step({{101,3,H.Predict(N)}},false,Why)
            ||!Neutral->Step({{101,3,Z.Predict(N)}},false,Why))
        {Test.AddError(FString::Printf(TEXT("%s native step %llu: %s"),Case.Name,N,*Why));return false;}
        auto L=Local->ReadLatest(),A=Authority->ReadLatest(),B=Neutral->ReadLatest();
        if(!L||!A||!B||L->Frame!=N||A->Frame!=N||B->Frame!=N||!Body(*L)||!Body(*A)||!Body(*B))return false;
        if(!Difference.Add(Body(*L)->COMState,Body(*A)->COMState,N)||!Response.Add(Body(*A)->COMState,Body(*B)->COMState,N))return false;
        const auto& LF=L->Actors[0];const auto& AF=A->Actors[0];
        const auto Partial=P::ComparePrivateRoadPartialValues(LF,AF);
        const auto Control=P::ComparePrivateRoadPartialValues(AF,B->Actors[0]);
        if(Partial==P::EPrivateRoadPartialComparison::Unknown||Control==P::EPrivateRoadPartialComparison::Unknown)++PartialMissing;
        else {if(Partial==P::EPrivateRoadPartialComparison::Different)++PartialDifferent;if(Control==P::EPrivateRoadPartialComparison::Different)++PartialResponse;}
        if(!(LF.Filter==AF.Filter)||LF.Input->Requested!=AF.Input->Requested||LF.Input->Applied!=AF.Input->Applied)++FilterDifferences;
        if(LF.CanMove!=AF.CanMove||LF.Frozen!=AF.Frozen||LF.CountdownFrames!=AF.CountdownFrames)++StartupDifferences;
        if(LF.CanMove&&FirstLocalCanMove<0)FirstLocalCanMove=int64(N);
        if(AF.CanMove&&FirstAuthorityCanMove<0)FirstAuthorityCanMove=int64(N);
    }
    Test.AddInfo(FString::Printf(TEXT("%s case=%s action=%u quantized=%d frames=%u halfPeriod=%u frequencyHz=%.17g localCountdown=%u authorityCountdown=%u firstLocalCanMove=%lld firstAuthorityCanMove=%lld firstDifferent=%lld differentFrames=%u filterDifferentFrames=%u startupDifferentFrames=%u maxPositionCm=%.17g maxVelocityCmS=%.17g maxAccelerationCmSS=%.17g maxRotationRadians=%.17g maxAngularVelocity=%.17g maxAngularAcceleration=%.17g responsePositionCm=%.17g responseVelocityCmS=%.17g responseRotationRadians=%.17g sockets=0 completeHiddenState=0"),
        Case.HalfPeriod?TEXT("[OscillatingInputMeasure]"):TEXT("[AtomicInputMeasure]"),Case.Name,Case.Action,int(Case.Value),uint32(Case.FrameCount),uint32(Case.HalfPeriod),Case.HalfPeriod?300.0/(2.0*Case.HalfPeriod):0.0,Case.LocalCountdown,Case.AuthorityCountdown,FirstLocalCanMove,FirstAuthorityCanMove,
        Difference.FirstDifferent,Difference.DifferentFrames,FilterDifferences,StartupDifferences,Difference.Position,Difference.Velocity,Difference.Acceleration,
        Difference.Rotation,Difference.AngularVelocity,Difference.AngularAcceleration,Response.Position,Response.Velocity,Response.Rotation));
    Test.AddInfo(FString::Printf(TEXT("[NativePartialStateMeasure] case=%s action=%u frames=%u unknownFrames=%u differingFrames=%u neutralResponseFrames=%u complete_causal_state=0 sockets=0 hardware=0"),Case.Name,Case.Action,uint32(Case.FrameCount),PartialMissing,PartialDifferent,PartialResponse));
    bool OK=Test.TestEqual(TEXT("actual declared native cache/wheel subset observed; missing is unknown"),PartialMissing,uint32(0));
    OK=Test.TestEqual(TEXT("same requested input has identical processed state for both roles"),FilterDifferences,uint32(0))&&OK;
    if(Case.ExpectMismatch)
    {
        OK=Test.TestTrue(TEXT("countdown mismatch positive control produces actual physical difference"),Difference.DifferentFrames>0)&&OK;
        OK=Test.TestTrue(TEXT("countdown mismatch is observed in actual published native startup values"),StartupDifferences>0)&&OK;
    }
    else
    {
        OK=Test.TestEqual(TEXT("matched configuration has exact six-field COM kinematics every frame"),Difference.DifferentFrames,uint32(0))&&OK;
        OK=Test.TestEqual(TEXT("matched native startup counters, freeze and CanMove"),StartupDifferences,uint32(0))&&OK;
    }
    if(Case.Action<3)
        OK=Test.TestTrue(TEXT("isolated physical input has measurable response against neutral control"),Response.DifferentFrames>0)&&OK;
    else
        OK=Test.TestEqual(TEXT("camera input does not alter COM trajectory"),Response.DifferentFrames,uint32(0))&&OK;
    // Always retire all worlds, including a failed comparison; no short-circuit.
    const bool LC=Local->Close(),AC=Authority->Close(),NC=Neutral->Close();
    return Test.TestTrue(TEXT("all three actual native worlds retired"),LC&&AC&&NC)&&OK;
}
bool RunOscillating(FAutomationTestBase& Test,unsigned Action,const TCHAR* Name,int16 Maximum)
{
    bool OK=true;
    // At the actual canonical300Hz: 1,2,5,10,25,50,150Hz. Includes slew
    // saturation, rapid reversals and release, with no simultaneous axis mixing.
    for(const uint16 HalfPeriod:{uint16(150),uint16(75),uint16(30),uint16(15),uint16(6),uint16(3),uint16(1)})
    {
        const FString CaseName=FString::Printf(TEXT("%s-halfperiod-%u"),Name,uint32(HalfPeriod));
        FCase C{*CaseName,Action,Maximum};C.HalfPeriod=HalfPeriod;C.FrameCount=613;
        if(!RunCase(Test,C))OK=false;
    }
    Test.AddInfo(TEXT("Oscillation compares real native local/confirmed-remote COM and processing per frame, with a third neutral native control. Unit-authored inputs, no sockets/hardware or full-state equality. Pause/Reset remain separate lifecycle domain, not silently covered."));
    return OK&&!Test.HasAnyErrors();
}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASAtomicNativeInputs,"IAmSpeed.Netcode.Prediction.AtomicNativeInputs",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASAtomicNativeInputs::RunTest(const FString&)
{
    using namespace IASAtomicRoadCampaign;bool OK=true;
    const FCase Cases[]={
        {TEXT("throttle-quarter"),0,64},{TEXT("throttle-half"),0,128},{TEXT("throttle-full"),0,255},
        {TEXT("brake-quarter"),1,64},{TEXT("brake-half"),1,128},{TEXT("brake-full"),1,255},
        {TEXT("steer-left-quarter"),2,-32},{TEXT("steer-left-half"),2,-64},{TEXT("steer-left-full"),2,-127},
        {TEXT("steer-right-quarter"),2,32},{TEXT("steer-right-half"),2,64},{TEXT("steer-right-full"),2,127},
        {TEXT("camera-yaw-left"),5,-127},{TEXT("camera-yaw-right"),5,127},
        {TEXT("camera-pitch-down"),6,-127},{TEXT("camera-pitch-up"),6,127}};
    for(const auto& C:Cases)if(!RunCase(*this,C))OK=false;
    AddInfo(TEXT("Pause/ResetWorld are lifecycle commands, unsupported by this finite road graph: NOT_QUALIFIED. All inputs here are unit-authored receipts, not hardware or authenticated sockets. No complete hidden-state equality inferred."));
    return OK&&!HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeStartupCountdown,"IAmSpeed.Netcode.Prediction.NativeStartupCountdown",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeStartupCountdown::RunTest(const FString&)
{
    using namespace IASAtomicRoadCampaign;bool OK=true;
    const FCase Cases[]={
        {TEXT("startup-no-freeze"),0,255,0,0,false,true},
        {TEXT("startup-one-frame"),0,255,1,1,false,true},
        {TEXT("startup-five-frames"),0,255,5,5,false,true},
        {TEXT("startup-fifteen-frames"),0,255,15,15,false,true},
        {TEXT("startup-thirty-frames"),0,255,30,30,false,true},
        {TEXT("startup-mismatch-positive-control"),0,255,0,15,true,true}};
    for(const auto& C:Cases)if(!RunCase(*this,C))OK=false;
    AddInfo(TEXT("Actual native CanMove/countdown/freeze comparison. Same initial native graph/config except explicitly mismatched countdown control. No tuning and no conclusion about actual Demo/network initialization."));
    return OK&&!HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASOscillatingThrottle,"IAmSpeed.Netcode.Prediction.OscillatingInputs.Throttle",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASOscillatingThrottle::RunTest(const FString&)
{return IASAtomicRoadCampaign::RunOscillating(*this,0,TEXT("throttle"),255);}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASOscillatingBrake,"IAmSpeed.Netcode.Prediction.OscillatingInputs.Brake",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASOscillatingBrake::RunTest(const FString&)
{return IASAtomicRoadCampaign::RunOscillating(*this,1,TEXT("brake"),255);}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASOscillatingSteering,"IAmSpeed.Netcode.Prediction.OscillatingInputs.Steering",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASOscillatingSteering::RunTest(const FString&)
{return IASAtomicRoadCampaign::RunOscillating(*this,2,TEXT("steering"),127);}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASOscillatingCameraYaw,"IAmSpeed.Netcode.Prediction.OscillatingInputs.CameraYaw",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASOscillatingCameraYaw::RunTest(const FString&)
{return IASAtomicRoadCampaign::RunOscillating(*this,5,TEXT("camera-yaw"),127);}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASOscillatingCameraPitch,"IAmSpeed.Netcode.Prediction.OscillatingInputs.CameraPitch",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASOscillatingCameraPitch::RunTest(const FString&)
{return IASAtomicRoadCampaign::RunOscillating(*this,6,TEXT("camera-pitch"),127);}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASNativeInitialPublicationGate,"IAmSpeed.Netcode.Prediction.InitialPublicationCanMoveGate",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASNativeInitialPublicationGate::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;namespace V=Speed::Input::V2;
    using namespace Speed::Netcode;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    P::FPrivateRoadCarSeed Seed; // Default policy MUST remain closed.
    Seed.Actor=101;Seed.Incarnation=3;Seed.Contract=Contract;Seed.Role=P::EBranchFilterRole::LocalDevice;
    Seed.Local={Speed::Input::EProducerKind::Device,42};Seed.LocalEpoch={3};
    Seed.Remote.Session=1;Seed.Remote.Epoch=2;Seed.Remote.Actor=101;Seed.Remote.Incarnation=3;
    Seed.Remote.OwnerPeer=4;Seed.Remote.ServerPeer=5;Seed.Remote.SourceProducer=42;Seed.Remote.SourceKind=0;Seed.Remote.Contract[0]=7;
    Seed.Initial.Location=FVector(0,0,45);Seed.Initial.Velocity=FVector(500,0,0);
    FString Why;auto World=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    if(!TestTrue(*FString::Printf(TEXT("actual default-closed native graph: %s"),*Why),bool(World)))return false;
    const auto Input=[&](U64 Frame)
    {
        V::FInputFrameData D;D.ConsumptionFrame=Frame;D.SourceSequence=Frame+1;D.DeviceGeneration={1};
        D.Producer=Seed.Local;D.StreamEpoch=Seed.LocalEpoch;D.Reset=Frame==0;
        if(Frame){D.Values[0]=255;D.ActiveMask=1;D.Transitions.push_back({0,V::ETransition::Started,255,{Frame+1,0}});}
        return std::make_shared<const V::FInputFrame>(Contract,D);
    };
    bool OK=TestFalse(TEXT("no release before actual baseline publication"),World->ReleaseInitialPublication({},Why));
    OK=TestTrue(TEXT("actual frozen initial canonical frame"),World->Step({{101,3,Input(0)}},false,Why))&&OK;
    const auto Baseline=World->ReadLatest();
    if(!Baseline||Baseline->Actors.size()!=1){World->Close();AddError(TEXT("actual baseline unavailable"));return false;}
    OK=TestTrue(TEXT("default awaiting actual startup publication"),Baseline->AwaitingInitialPublication&&!Baseline->Actors[0].CanMove
        &&Baseline->Actors[0].Frozen&&Baseline->Actors[0].CountdownFrames==1)&&OK;
    auto Copy=std::make_shared<const P::FPrivateRoadPublication>(*Baseline);
    OK=TestFalse(TEXT("value-identical caller copy is not the retained original publication"),World->ReleaseInitialPublication(Copy,Why))&&OK;
    const auto Next=Input(1);bool AllBlocked=true;
    for(unsigned Attempt=0;Attempt<512;++Attempt)
        if(World->Step({{101,3,Next}},false,Why)||World->ReadLatest()!=Baseline)AllBlocked=false;
    OK=TestTrue(TEXT("512 attempted frames cannot expire the gate or mutate the physical publication"),AllBlocked)&&OK;
    OK=TestTrue(TEXT("explicit owner releases the actual retained initial publication"),World->ReleaseInitialPublication(Baseline,Why))&&OK;
    OK=TestFalse(TEXT("duplicate release is refused"),World->ReleaseInitialPublication(Baseline,Why))&&OK;
    OK=TestTrue(TEXT("common activation advances the next actual physical frame"),World->Step({{101,3,Next}},false,Why))&&OK;
    const auto Activated=World->ReadLatest();
    OK=TestTrue(TEXT("actual CanMove and freeze state reflect explicit activation"),Activated&&Activated->Frame==1
        &&!Activated->AwaitingInitialPublication&&Activated->Actors[0].CanMove&&!Activated->Actors[0].Frozen
        &&Activated->Actors[0].CountdownFrames==0)&&OK;
    OK=TestTrue(TEXT("retained old baseline remains immutable and closed"),Baseline->Frame==0&&Baseline->AwaitingInitialPublication
        &&!Baseline->Actors[0].CanMove)&&OK;
    const bool Closed=World->Close();
    AddInfo(TEXT("[InitialPublicationGate] actualNative=1 blockedAttempts=512 initialFrame=0 activationFrame=1 timerRelease=0 fullHiddenStateCertificate=0 networkAuthenticationQualified=0. Tests physical local-owner gate only; original-process session comparison is separately required by the process probe."));
    return TestTrue(TEXT("actual native startup graph retired"),Closed)&&OK&&!HasAnyErrors();
}
#endif
