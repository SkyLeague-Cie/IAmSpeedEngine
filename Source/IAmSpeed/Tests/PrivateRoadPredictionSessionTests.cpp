#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPredictionSession.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
#include "UObject/GarbageCollection.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadLateInputReconciliation,
    "IAmSpeed.Netcode.Prediction.PrivateRoadLateInputReconciliation",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadLateInputReconciliation::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;namespace V=Speed::Input::V2;using namespace Speed::Netcode;
    // Numerical regressions exercise the SAME production metric/verdict helper
    // used below on real retained native publications. No physical coverage is
    // inferred from these arithmetic counterexamples alone.
    {
        SKinematic A,B;auto D=P::MeasurePrivateRoadKinematics(A,B);
        TestTrue(TEXT("identical kinematics compare equal"),D.Valid&&!D.Different&&D.RotationRadians==0);
        B.Rotation=FQuat(0,0,0,-1);D=P::MeasurePrivateRoadKinematics(A,B);
        TestTrue(TEXT("antipodal quaternion has the same orientation"),D.Valid&&!D.Different&&D.RotationRadians==0);
        B=A;B.Rotation=FQuat(1e-9,0,0,1);
        TestTrue(TEXT("tiny-angle counterexample satisfies native quaternion normalization"),B.Rotation.IsNormalized());
        D=P::MeasurePrivateRoadKinematics(A,B);
        TestTrue(TEXT("tiny real rotation remains Different with nonzero stable angle"),D.Valid&&D.Different&&D.RotationRadians>0&&D.RotationRadians<3e-9);
        B=A;B.Location.X=1e-200;D=P::MeasurePrivateRoadKinematics(A,B);
        TestTrue(TEXT("vector difference cannot vanish through squared-norm underflow"),D.Valid&&D.Different&&D.Position>0);
        B=A;B.Acceleration.Y=1e-200;B.AngularAcceleration.Z=1e-200;D=P::MeasurePrivateRoadKinematics(A,B);
        TestTrue(TEXT("small acceleration field differences are independently retained"),D.Valid&&D.Different&&D.Acceleration>0&&D.AngularAcceleration>0);
    }
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    std::vector<P::FPrivateRoadCarSeed> Seeds;
    for(U64 A=0;A<2;++A)
    {
        P::FPrivateRoadCarSeed S;S.StartPolicy=P::EPrivateRoadStartPolicy::ExplicitUnitRecipe;S.Actor=101+A;S.Incarnation=3;S.Contract=Contract;
        S.Role=A?P::EBranchFilterRole::RemoteRequests:P::EBranchFilterRole::LocalDevice;
        S.Local={Speed::Input::EProducerKind::Device,42+A};S.LocalEpoch={3};
        S.Remote.Session=1;S.Remote.Epoch=2;S.Remote.Actor=S.Actor;S.Remote.Incarnation=3;
        S.Remote.OwnerPeer=4;S.Remote.ServerPeer=5;S.Remote.SourceProducer=6+A;S.Remote.SourceKind=0;S.Remote.Contract[0]=7;
        S.Initial.Location=FVector(0,A?1000:-1000,45);S.Initial.Velocity=FVector(500,0,0);
        S.Processing.Step[0]=17;S.Processing.Step[1]=17;Seeds.push_back(S);
    }
    const auto Packet=[&](U64 N,int16 Throttle,int16 Brake,int16 Steer)
    {
        const auto& B=Seeds[1].Remote;FPacket Q;Q.Kind=EKind::AuthorityInput;
        Q.Session=B.Session;Q.Epoch=B.Epoch;Q.Actor=B.Actor;Q.Incarnation=B.Incarnation;Q.Contract=B.Contract;
        Q.Frame=N;Q.Sequence=N+1;Q.Input.SourceSequence=N+1;Q.Input.DeviceGeneration=1;
        Q.Input.ProducerId=B.SourceProducer;Q.Input.ProducerKind=B.SourceKind;Q.Input.Reset=N==0;
        Q.Input.Values[0]=Throttle;Q.Input.Values[1]=Brake;Q.Input.Values[2]=Steer;
        Q.Input.ActiveMask=(Throttle?1u:0u)|(Brake?2u:0u)|(Steer?4u:0u);
        if(N==30){Q.Input.Edges.push_back({0,1,0,N+1,0});Q.Input.Edges.push_back({1,0,Brake,N+1,1});}
        if(N==60){Q.Input.Edges.push_back({1,1,0,N+1,0});Q.Input.Edges.push_back({0,0,Throttle,N+1,1});Q.Input.Edges.push_back({2,0,Steer,N+1,2});}
        return Q;
    };
    const auto Local=[&](U64 N,U64 Generation=1,bool FirstReset=true)
    {
        V::FInputFrameData D;D.ConsumptionFrame=N;D.SourceSequence=N+1;D.DeviceGeneration={Generation};
        D.Producer=Seeds[0].Local;D.StreamEpoch=Seeds[0].LocalEpoch;D.Reset=N==0&&FirstReset;
        D.Values[0]=128;D.ActiveMask=1|(1u<<5)|(1u<<6);
        D.Values[5]=static_cast<int16>((N%2?1:-1)*int(Contract->Find(Speed::Input::FActionId(5))->Quantization));
        D.Values[6]=static_cast<int16>((N%3?1:-1)*int(Contract->Find(Speed::Input::FActionId(6))->Quantization));
        return std::make_shared<const V::FInputFrame>(Contract,D);
    };
    const auto* YawContract=Contract->Find(Speed::Input::FActionId(5));
    const auto* PitchContract=Contract->Find(Speed::Input::FActionId(6));
    if(!YawContract||!PitchContract||!YawContract->Quantization||!PitchContract->Quantization)return false;
    FString Why;auto Client=P::FPrivateRoadPredictionSession::Create(Seeds,512,Why);
    if(!TestTrue(*FString::Printf(TEXT("real native prediction owner: %s"),*Why),bool(Client)))return false;
    auto Reference=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);if(!Reference)return false;
    const auto UnstartedPrefix=Client->CaptureConsumedInputPrefixForDiagnostics(512);
    const bool UnstartedUnknown=UnstartedPrefix.Status==P::EPrivateRoadInputPrefixCopy::Unavailable
        &&!UnstartedPrefix.Publication&&UnstartedPrefix.Prefix.empty();
    TestTrue(TEXT("unstarted input history is UNKNOWN with no numeric prefix"),UnstartedUnknown);
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory Known(Seeds[1].Remote,Actions,256,512);
    const auto Late8=Packet(8,192,0,0);
    const auto Initial=Packet(0,255,0,0),Late30=Packet(30,0,255,0),Late60=Packet(60,128,0,32);
    for(const auto& Q:{Initial,Late8,Late30,Late60})if(Known.Confirm(5,Q).Status!=EResult::Accepted)return false;
    // Only the independent reference knows the authored future. Client receives
    // each actual packet below, at its deliberately late delivery point.
    if(Client->ConfirmRemote(5,Initial).Status!=EResult::Accepted)return false;
    TestTrue(TEXT("wrong original peer rejected"),Client->ConfirmRemote(4,Late30).Status==EResult::WrongPeer);
    TestFalse(TEXT("first local retained frame requires real reset baseline"),Client->AdvanceLocal(Local(0,1,false),Why));
    const auto Pose=[](const P::FPrivateRoadPublication& Pub,U64 Actor)->const FSimulationPresentationBody*
    {
        const auto A=std::find_if(Pub.Actors.begin(),Pub.Actors.end(),[&](const auto& X){return X.Actor==Actor;});
        return A==Pub.Actors.end()?nullptr:Pub.Physics.PresentationBodies.FindByPredicate([&](const auto& X){return X.StableId==A->NativeStableId;});
    };
    std::shared_ptr<const P::FPrivateRoadPublication> Reference60;
    const auto Observe=[](const P::FPrivateRoadPublication& Actual)
    {
        P::FPrivateRoadObservedFrame Out;Out.Frame=Actual.Frame;
        for(const auto& A:Actual.Actors)
        {
            const auto* Body=Actual.Physics.PresentationBodies.FindByPredicate([&](const auto& B){return B.StableId==A.NativeStableId;});
            if(!Body)return P::FPrivateRoadObservedFrame{};
            Out.Actors.push_back({A.Actor,A.Incarnation,Body->COMState});
        }
        return Out;
    };
    const auto Step=[&](U64 N)
    {
        auto F=Local(N);
        if(!Client->AdvanceLocal(F,Why))return false;
        std::vector<P::FPrivateRoadRequest> Requests={{101,3,F},{102,3,Known.Predict(N)}};
        if(!Reference->Step(Requests,false,Why))return false;
        if(N==60)Reference60=Reference->ReadLatest();return true;
    };
    for(U64 N=0;N<=90;++N)
    {
        if(N==1)TestFalse(TEXT("generation change without reset rejected without advancing native frame"),Client->AdvanceLocal(Local(1,2),Why));
        if(!TestTrue(*FString::Printf(TEXT("native forward step %llu: %s"),N,*Why),Step(N)))return false;
    }
    const auto Original=Client->ReadLatest(),Authority90=Reference->ReadLatest();
    const auto OriginalPrefix=Client->CaptureConsumedInputPrefixForDiagnostics(512);
    if(!TestTrue(TEXT("actual first-forward graph and full input prefix captured together"),
        OriginalPrefix.Status==P::EPrivateRoadInputPrefixCopy::CompleteObservedPrefix
        &&OriginalPrefix.Publication==Original&&OriginalPrefix.Prefix.size()==91
        &&OriginalPrefix.CompletedGraphAdoptions==0))return false;
    const auto BoundedOut=Client->CaptureConsumedInputPrefixForDiagnostics(90);
    const bool OverflowUnknown=BoundedOut.Status==P::EPrivateRoadInputPrefixCopy::Overflow
        &&BoundedOut.Publication==Original&&BoundedOut.Prefix.empty();
    TestTrue(TEXT("bounded overflow is UNKNOWN, never empty-equal or zero divergence"),OverflowUnknown);
    const bool CameraSlotsObserved=OriginalPrefix.Prefix[0].Actors[0].Requested[5]!=0
        &&OriginalPrefix.Prefix[0].Actors[0].Requested[6]!=0;
    TestTrue(TEXT("actual camera slots beyond the old three-axis trace are retained"),CameraSlotsObserved);
    auto EditedValueCopy=OriginalPrefix;EditedValueCopy.Prefix[0].Actors[0].Requested[5]=0;
    TestTrue(TEXT("editing copied values cannot mutate retained native input history"),
        Client->CaptureConsumedInputPrefixForDiagnostics(512).Prefix==OriginalPrefix.Prefix
        &&Client->ReadLatest()==Original);
    const bool HoldResetUnknown=!OriginalPrefix.Prefix[30].Actors[1].ResetEvidenceKnown;
    const bool ActualResetCaptured=OriginalPrefix.Prefix[0].Actors[0].ResetEvidenceKnown
        &&OriginalPrefix.Prefix[0].Actors[0].ActualReset;
    TestTrue(TEXT("actual raw reset is captured and speculative reset remains UNKNOWN"),ActualResetCaptured&&HoldResetUnknown);
    for(const auto& F:OriginalPrefix.Prefix)for(const auto& X:F.Actors)
        TestTrue(TEXT("actual role, origin and evidence agree without normalizing them"),P::PrivateRoadObservedRoleOriginConsistent(X));
    if(!Original||!Authority90||!Pose(*Original,102)||!Pose(*Authority90,102))return false;
    const double BeforeError=FVector::Dist(Pose(*Original,102)->COMState.Location,Pose(*Authority90,102)->COMState.Location);
    if(!TestTrue(TEXT("wrong held remote request creates measured same-frame native divergence"),BeforeError>1))return false;
    if(!Reference60)return false;
    const auto Historical=Client->CompareObservedKinematics(Observe(*Reference60));
    TestTrue(TEXT("late observation compares exact retained native frame60, not latest90"),Historical.Frame==60&&Historical.Status==P::EPrivateRoadPoseComparison::Different&&Historical.Actors.size()==2&&Historical.Actors[1].Position>1);
    auto Future=Observe(*Authority90);Future.Frame=91;
    TestTrue(TEXT("unpredicted future has its own result"),Client->CompareObservedKinematics(Future).Status==P::EPrivateRoadPoseComparison::Future);
    auto WrongRoster=Observe(*Reference60);WrongRoster.Actors[1].Incarnation=4;
    TestTrue(TEXT("wrong incarnation cannot be compared to another history"),Client->CompareObservedKinematics(WrongRoster).Status==P::EPrivateRoadPoseComparison::RosterMismatch);
    const auto Receipt=Client->ConfirmRemote(5,Late30);
    TestTrue(TEXT("late original request invalidates earliest actual predicted frame"),Receipt.Status==EResult::Accepted&&Receipt.EarliestInvalidatedFrame==30);
    auto Status=Client->AdvanceCorrection(7,Why);
    if(!TestTrue(TEXT("bounded correction stages a genuine new physical graph"),Status==P::EPrivateRoadCorrection::Replaying))return false;
    TestTrue(TEXT("staged replay is invisible; old immutable publication remains live"),Client->ReadLatest()==Original);
    const auto DuringStagedPrefix=Client->CaptureConsumedInputPrefixForDiagnostics(512);
    TestTrue(TEXT("copy never observes an incomplete staged graph in place of the live prefix"),
        DuringStagedPrefix.Status==P::EPrivateRoadInputPrefixCopy::CompleteObservedPrefix
        &&DuringStagedPrefix.Publication==Original&&DuringStagedPrefix.Prefix==OriginalPrefix.Prefix
        &&DuringStagedPrefix.CompletedGraphAdoptions==0);
    U64 Last=90;
    for(U64 Pulse=1;Pulse<=64&&Status==P::EPrivateRoadCorrection::Replaying;++Pulse)
    {
        if(Pulse==2)
            TestTrue(TEXT("late receipt for an already replayed transition remains accepted"),Client->ConfirmRemote(5,Late8).Status==EResult::Accepted);
        if(Pulse==3)
            TestTrue(TEXT("future staged receipt joins the journal without starving replay"),Client->ConfirmRemote(5,Late60).Status==EResult::Accepted);
        if(Pulse<=30){++Last;if(!Step(Last))return false;}
        Status=Client->AdvanceCorrection(7,Why);
        if(!TestTrue(TEXT("no partially replayed past frame replaces live tail"),Client->ReadLatest()&&Client->ReadLatest()->Frame==Last))return false;
    }
    if(!TestTrue(*FString::Printf(TEXT("actual native correction adopts after tail catchup: %s"),*Why),Status==P::EPrivateRoadCorrection::Adopted))return false;
    const auto Corrected=Client->ReadLatest(),Expected=Reference->ReadLatest();
    if(!Corrected||!Expected)return false;
    TestTrue(TEXT("adoption changes real local graph lifetime, not semantic actor incarnation"),Corrected->LocalKernelLifetime!=Original->LocalKernelLifetime&&Corrected->Actors[0].Incarnation==3);
    for(const auto& A:Corrected->Actors)
    {
        const auto* X=Pose(*Corrected,A.Actor);const auto* Y=Pose(*Expected,A.Actor);if(!X||!Y)return false;
        TestTrue(TEXT("corrected native position exactly matches independent authority-input reference"),X->COMState.Location==Y->COMState.Location);
        TestTrue(TEXT("corrected native velocity exactly matches independent authority-input reference"),X->COMState.Velocity==Y->COMState.Velocity);
        TestTrue(TEXT("corrected native rotation and angular velocity match"),X->COMState.Rotation==Y->COMState.Rotation&&X->COMState.AngularVelocity==Y->COMState.AngularVelocity);
    }
    const auto HistoricalCorrected=Client->CompareObservedKinematics(Observe(*Reference60));
    TestTrue(TEXT("adoption replaces coherent native history; original frame60 observation now matches"),HistoricalCorrected.Frame==60&&HistoricalCorrected.Status==P::EPrivateRoadPoseComparison::Equal);
    TestTrue(TEXT("latest actual authority-input frame also matches without frame retiming"),Client->CompareObservedKinematics(Observe(*Expected)).Status==P::EPrivateRoadPoseComparison::Equal);
    const auto Witness=Client->ReadLastCorrection();
    const auto AdoptedPrefix=Client->CaptureConsumedInputPrefixForDiagnostics(512);
    const bool ActualAdoptedPrefix=AdoptedPrefix.Status==P::EPrivateRoadInputPrefixCopy::CompleteObservedPrefix
        &&AdoptedPrefix.Publication==Corrected&&AdoptedPrefix.Prefix.size()==Last+1
        &&AdoptedPrefix.CompletedGraphAdoptions==1
        &&AdoptedPrefix.Prefix[30].Actors[1].Requested[0]==0
        &&AdoptedPrefix.Prefix[30].Actors[1].Requested[1]==255
        &&OriginalPrefix.Prefix[30].Actors[1].Requested[0]==255
        &&OriginalPrefix.Prefix[30].Actors[1].Requested[1]==0;
    TestTrue(TEXT("actual adoption replaces the full prefix, without concatenating first-forward history"),ActualAdoptedPrefix);
    for(const auto& F:AdoptedPrefix.Prefix)
        TestTrue(TEXT("every captured historical frame belongs to the adopted graph"),F.LocalKernelLifetime==Corrected->LocalKernelLifetime);
    TestTrue(TEXT("read-only input capture keeps actual adopted publication unchanged"),Client->ReadLatest()==Corrected);
    TestTrue(TEXT("actual reconciliation records native change and complete replay count"),Witness&&Witness->FirstInvalidated==8&&Witness->ReplayedFrames==Last+1&&Witness->ThroughFrame==Last&&Witness->MaximumPositionChange>1&&Witness->Corrections==1);
    TestTrue(TEXT("received authority is retained across adoption"),Client->ConfirmRemote(5,Late60).Status==EResult::Duplicate);
    TestTrue(TEXT("old immutable historical prediction remains unchanged"),Original->Frame==90&&Original->LocalKernelLifetime!=Corrected->LocalKernelLifetime);
    TestTrue(TEXT("real graphs close on owner"),Client->Close()&&Reference->Close());CollectGarbage(RF_NoFlags,true);
    TestTrue(TEXT("corrected values survive actual graph retirement"),Corrected->Frame==Last);
    const auto ClosedPrefix=Client->CaptureConsumedInputPrefixForDiagnostics(512);
    const bool ClosedUnknown=ClosedPrefix.Status==P::EPrivateRoadInputPrefixCopy::Unavailable
        &&!ClosedPrefix.Publication&&ClosedPrefix.Prefix.empty();
    TestTrue(TEXT("closed graph cannot claim an observed input prefix"),ClosedUnknown);
    AddInfo(FString::Printf(TEXT("[NativeObservedInputPrefix] before=%llu after=%llu actionSlots=%u actualAdoption=%d oldCopyImmutable=%d cameraSlots=%d actualReset=%d speculativeResetUnknown=%d overflowUnknown=%d unstartedUnknown=%d closedUnknown=%d fullCausalState=0 restoreK=0"),
        uint64(OriginalPrefix.Prefix.size()),uint64(AdoptedPrefix.Prefix.size()),unsigned(ActionCount),ActualAdoptedPrefix,
        OriginalPrefix.Prefix[30].Actors[1].Requested[0]==255,CameraSlotsObserved,ActualResetCaptured,HoldResetUnknown,OverflowUnknown,UnstartedUnknown,ClosedUnknown));
    AddInfo(FString::Printf(TEXT("[NativeLateInputReconciliation] BeforePositionError=%.9g CorrectedFrame=%llu ReplayFrames=%llu ExactKinematics=1 NativeGraphs=REAL Correction=FRESH_ORIGIN_RECONSTRUCTION NetworkAuthentication=NOT_QUALIFIED CompleteSnapshotRestore=NOT_QUALIFIED"),BeforeError,Last,Witness?Witness->ReplayedFrames:0));
    return !HasAnyErrors();
}
#endif
