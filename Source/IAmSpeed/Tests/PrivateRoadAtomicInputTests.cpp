#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadKinematicDiagnostics.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPartialStateComparison.h"
#include "IAmSpeed/Netcode/client-prediction/OriginalRoadBodyValueCapture.h"
#include <array>
#include <cstddef>
#include <type_traits>
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeExit.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "EngineUtils.h"
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadNativeObjects.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "IAmSpeed/World/Simulation/NativeEpochDiagnostics.h"
#include "Misc/OutputDevice.h"
#include "Misc/OutputDeviceRedirector.h"
#include "UObject/GarbageCollection.h"
#include "UObject/UObjectHash.h"
#include "UObject/UObjectArray.h"
#include "UObject/ObjectHandleTracking.h"
#include "IAmSpeed/World/Simulation/RoadSettings/NativeRoadSettingsRead.h"
#include <atomic>

namespace
{
struct FActualNativeBoundaryLogProbe final : FOutputDevice
{
    enum class ECase {BirthGC,DestroyBirthDriver,ActivationFacade};
    ECase Case=ECase::BirthGC;
    TWeakObjectPtr<UWorld> World;
    int32 Calls=0,ActualGCReturns=0,LivePreparedWheelObjects=0;
    bool OutsideRecipe=false,OutsideCollection=false,ActualDriverDestroyReturned=false;
    bool FacadeFreezeTransition=false,ActualLegacyQueueReturned=false;
    bool CanBeUsedOnAnyThread() const override {return true;}
    bool CanBeUsedOnMultipleThreads() const override {return true;}
    void Serialize(const TCHAR* Text,ELogVerbosity::Type,const FName&) override
    {
        if(!IsInGameThread()||Calls||!World.IsValid()||!Speed::NativeEpoch::IsOriginalDiagnosticFlush())return;
        const TCHAR* Marker=Case==ECase::ActivationFacade?TEXT("[IASOriginalActivationCompleted]"):TEXT("[WheelSuspensionBinding]");
        if(!FCString::Strstr(Text,Marker))return;
        ++Calls;
        OutsideRecipe=!Speed::RoadSettings::SelectedOriginalRoadSettings();
        OutsideCollection=!Speed::NativeEpoch::HasOriginalDiagnosticOutbox();
        if(Case==ECase::DestroyBirthDriver)
        {
            auto* ActualWorld=World.Get();
            if(!GEngine||!ActualWorld)return;
            const bool AddedContext=GEngine->GetWorldContextFromWorld(ActualWorld)==nullptr;
            if(AddedContext)GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(ActualWorld);
            ON_SCOPE_EXIT{if(AddedContext)GEngine->DestroyWorldContext(ActualWorld);};
            // Real DestroyActor requires a real Engine world context. Supply
            // that test precondition and remove it BEFORE the genuine GC.
            for(TActorIterator<ASpeedBranchSimulation> It(ActualWorld);It;++It)
            {ActualDriverDestroyReturned=It->Destroy();break;}
        }
        if(Case!=ECase::ActivationFacade)
        {
            CollectGarbage(RF_NoFlags);++ActualGCReturns;
            // Reacquire after genuine GC; never use a pre-callback raw actor.
            if(auto* W=World.Get())for(TActorIterator<ASpeedBranchCar> It(W);It;++It)
                if(auto* C=Cast<USpeedBranchWheeledComponent>(It->GetVehicleMovement()))
                    ForEachObjectWithOuter(C,[&](UObject* Child)
                    {if(Child->IsA<UChaosVehicleWheel>())++LivePreparedWheelObjects;},EGetObjectsFlags::IncludeNestedObjects);
        }
        else if(auto* W=World.Get())for(TActorIterator<ASpeedBranchCar> It(W);It;++It)
        {
            auto* C=Cast<USpeedBranchWheeledComponent>(It->GetVehicleMovement());if(!C)continue;
            C->UnFreezeMovement();const bool WasUnfrozen=!C->IsFrozen();C->FreezeMovement();
            FacadeFreezeTransition=WasUnfrozen&&C->IsFrozen();
            if(auto* Bridge=W->GetSubsystem<USpeedWorldSubsystem>())
            {Bridge->UnregisterSpeedComponent(C);Bridge->ApplyPendingOps();ActualLegacyQueueReturned=true;}
        }
    }
};
// Actual Engine read callback witness only. It does NOT authorize restoration
// or claim that original mutable views cannot escape through this path.
struct FActualHandleReadWitness
{
    struct FTarget {TWeakObjectPtr<USSubBody> Body;const Chaos::FSimpleWheelSim* Observed=nullptr;uint64 OutsideReads=0;};
    std::vector<FTarget> Targets;
    uint64 ScopedTargetReads=0,OutsideTargetReads=0;
    void Observe(TArrayView<const UObject* const> Objects)
    {
        if(!IsInGameThread())return;
        for(const auto* Object:Objects)for(auto& T:Targets)
        {
            auto* Body=T.Body.Get();if(!Body||Body!=Object)continue;
            if(!Speed::RoadSettings::SelectedOriginalRoadSettings()){++OutsideTargetReads;++T.OutsideReads;continue;}
            ++ScopedTargetReads;
            if(auto* Wheel=Cast<USWheelSubBody>(Body);Wheel&&!T.Observed)T.Observed=Wheel->GetWheelSim(); // retain actual view identity, never mutate it
        }
    }
};
// Actual Engine read census of ALL objects, including factory and the native
// step/capture. This is diagnostic evidence, never callback exclusion authority.
// Do not resolve/dereference class/name/physical views inside the callback.
struct FActualRecipeObjectReadCensus
{
    struct FEntry
    {
        const UObject* OriginalAddress=nullptr; // identity only; never dereference without serial/pointer read-back
        int32 ObjectIndex=INDEX_NONE,ObjectSerial=0;
        uint64 BirthReads=0,StepReads=0,ControlReads=0;
    };
    std::array<FEntry,256> Entries{};
    uint32 Used=0,ObserverDepth=0,MaxObserverDepth=0;bool DuringBirth=true,DuringControl=false,Overflow=false;
    uint64 BirthObjects=0,StepObjects=0,ControlObjects=0,NullObjects=0,NativeSerialAllocationCalls=0,UnknownIdentities=0;
    void Observe(TArrayView<const UObject* const> Objects)
    {
        if(!IsInGameThread()||!Speed::RoadSettings::SelectedOriginalRoadSettings())return;
        ++ObserverDepth;if(ObserverDepth>MaxObserverDepth)MaxObserverDepth=ObserverDepth;
        ON_SCOPE_EXIT{--ObserverDepth;}; // observe depth; never skip a callback
        for(const auto* Object:Objects)
        {
            if(!Object){++NullObjects;continue;}
            if(DuringBirth)++BirthObjects;else if(DuringControl)++ControlObjects;else ++StepObjects;
            // Weak assignment itself calls FObjectPtr.Get in this exact Engine,
            // recursively entering the real read callback. Use the actual object
            // array identity operations that Weak assignment ultimately uses,
            // without constructing/resolving a tracked pointer in this observer.
            const int32 ActualIndex=GUObjectArray.ObjectToIndex(Object);
            if(ActualIndex<0){++UnknownIdentities;continue;}
            auto* Item=GUObjectArray.IndexToObject(ActualIndex,false);
            if(!Item||Item->IsUnreachable()||Item->GetObject()!=static_cast<const UObjectBase*>(Object))
            {++UnknownIdentities;continue;}
            int32 ActualSerial=Item->GetSerialNumber();
            if(!ActualSerial)
            {
                ++NativeSerialAllocationCalls; // real Engine metadata operation, not a fabricated generation
                ActualSerial=GUObjectArray.AllocateSerialNumber(ActualIndex);
            }
            if(ActualSerial<=0||Item->GetSerialNumber()!=ActualSerial||Item->GetObject()!=static_cast<const UObjectBase*>(Object))
            {++UnknownIdentities;continue;}
            uint32 Index=0;
            for(;Index<Used;++Index)
                if(Entries[Index].ObjectIndex==ActualIndex&&Entries[Index].ObjectSerial==ActualSerial
                    &&Entries[Index].OriginalAddress==Object)break;
            if(Index==Used)
            {
                if(Used==Entries.size()){Overflow=true;continue;}
                Entries[Index].OriginalAddress=Object;Entries[Index].ObjectIndex=ActualIndex;
                Entries[Index].ObjectSerial=ActualSerial;++Used;
            }
            if(DuringBirth)++Entries[Index].BirthReads;else if(DuringControl)++Entries[Index].ControlReads;else ++Entries[Index].StepReads;
        }
    }
};
struct FActualBirthReadScopeObserver final : FUObjectArray::FUObjectCreateListener
{
    std::atomic<bool> InsideOriginalCreate{false};
    std::atomic<uint64> ActualObjectCreationCalls{0},RecipeScopeCreationCalls{0};
    void NotifyUObjectCreated(const UObjectBase*,int32) override
    {
        // Actual Engine allocation callback, no access to a partially
        // constructed UObject. Observe only this thread's immutable scope.
        if(!IsInGameThread()||!InsideOriginalCreate.load(std::memory_order_acquire))return;
        ActualObjectCreationCalls.fetch_add(1,std::memory_order_relaxed);
        if(Speed::RoadSettings::SelectedOriginalRoadSettings())
            RecipeScopeCreationCalls.fetch_add(1,std::memory_order_relaxed);
    }
    void OnUObjectArrayShutdown() override {}
};
}

#include "IAmSpeed/World/Simulation/RoadSettings/DeclaredRoadConformanceProfile.h"

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
bool TestResponseChannelCapture(FAutomationTestBase& Test)
{
    namespace C=Speed::Netcode::Prediction::OriginalBodyCapture;
    struct FAdjacentResponse
    {
        FCollisionResponseParams Params{ECR_Block};
        std::array<uint8,16> After{};
    };
    static_assert(std::is_standard_layout_v<FAdjacentResponse>);
    static_assert(offsetof(FAdjacentResponse,After)==sizeof(FCollisionResponseParams));
    FAdjacentResponse A,B;A.After.fill(0x17);B.After.fill(0xe3);
    C::FReferences NoObjectReferences; // this scalar-only value has no object references
    C::FWriter X(NoObjectReferences),Y(NoObjectReferences);X.Value(A.Params);Y.Value(B.Params);
    bool OK=Test.TestEqual(TEXT("native stored collision response channel count"),C::StoredResponseChannels,size_t(64));
    OK=Test.TestTrue(TEXT("adjacent sentinel bytes cannot affect response encoding"),X.Bytes()==Y.Bytes())&&OK;
    uint32 Changed=0;
    for(size_t I=0;I<C::StoredResponseChannels;++I)
    {
        B.Params=A.Params;B.Params.CollisionResponse.SetResponse(ECollisionChannel(I),ECR_Ignore);
        C::FWriter Z(NoObjectReferences);Z.Value(B.Params);
        if(Z.Bytes()!=X.Bytes())++Changed;
    }
    OK=Test.TestEqual(TEXT("EVERY actual collision response channel including63 changes the encoding"),Changed,uint32(C::StoredResponseChannels))&&OK;
    Test.AddInfo(FString::Printf(TEXT("[ActualResponseChannelCodec] nativeStoredChannels=%llu independentlyChanged=%u adjacentSentinelsExcluded=%d deprecatedMarkerSerialized=0 completeEpoch=0"),
        uint64(C::StoredResponseChannels),Changed,int(X.Bytes()==Y.Bytes())));
    return OK;
}
bool RunCase(FAutomationTestBase& Test,const FCase& Case)
{
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    P::FPrivateRoadCarSeed Seed;Seed.DeclaredSettings=Speed::RoadSettings::MakeDeclaredRoadConformanceProfile();Seed.StartPolicy=P::EPrivateRoadStartPolicy::ExplicitUnitRecipe;Seed.Actor=101;Seed.Incarnation=3;Seed.Contract=Contract;
    Seed.Local={Speed::Input::EProducerKind::Device,42};Seed.LocalEpoch={3};
    Seed.Remote.Session=1;Seed.Remote.Epoch=2;Seed.Remote.Actor=101;Seed.Remote.Incarnation=3;
    Seed.Remote.OwnerPeer=4;Seed.Remote.ServerPeer=5;Seed.Remote.SourceProducer=42;Seed.Remote.SourceKind=0;Seed.Remote.Contract[0]=7;
    Seed.Initial.Location=FVector(0,0,45);Seed.Initial.Velocity=FVector(Case.FromRest?0:500,0,0);
    // Identical processing, including real slew state, on each genuine role.
    Seed.Processing.Step[0]=17;Seed.Processing.Step[1]=17;Seed.Processing.Step[2]=9;
    Seed.Role=P::EBranchFilterRole::LocalDevice;Seed.CountdownFrames=Case.LocalCountdown;
    auto* LiveStiffness=IConsoleManager::Get().FindConsoleVariable(TEXT("p.SkyLeague.Suspension.StiffnessScale"));
    if(!LiveStiffness||!LiveStiffness->IsVariableFloat())return false;
    const float OriginalLiveStiffness=LiveStiffness->GetFloat();
    if(!FMath::IsFinite(OriginalLiveStiffness))return false;
    ON_SCOPE_EXIT{LiveStiffness->Set(OriginalLiveStiffness,ECVF_SetByCode);};
    FString Why;auto Local=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    Seed.Role=P::EBranchFilterRole::RemoteRequests;Seed.CountdownFrames=Case.AuthorityCountdown;
    auto Authority=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    auto Neutral=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    std::vector<Speed::RoadSettings::FDeclaredValue> SensitiveValues;
    for(const auto& Key:Speed::RoadSettings::Catalog)
    {
        if(Key.Kind==Speed::RoadSettings::EKind::Integer)
        {
            const auto Value=Seed.DeclaredSettings->Read<int32>(Key);if(!Value)return false;
            SensitiveValues.push_back({Key,*Value});
        }
        else
        {
            const auto Value=Seed.DeclaredSettings->Read<float>(Key);if(!Value)return false;
            SensitiveValues.push_back({Key,Key.Index==177?*Value+17.0f:*Value});
        }
    }
    Speed::RoadSettings::ERefusal SensitiveWhy;
    auto SensitiveSeed=Seed;
    SensitiveSeed.DeclaredSettings=Speed::RoadSettings::FDeclaredRecipe::Create("declared-road-stiffness-sensitive-control.1",SensitiveValues,SensitiveWhy);
    if(!SensitiveSeed.DeclaredSettings)return false;
    auto StiffnessControl=P::FPrivateRoadPhysicsWorld::Create({SensitiveSeed},Why);
    if(!StiffnessControl)return false;
    if(!Test.TestTrue(*FString::Printf(TEXT("%s native three-world creation: %s"),Case.Name,*Why),bool(Local)&&bool(Authority)&&bool(Neutral)))return false;
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    Actions[5]={true,true,true,127};Actions[6]={true,true,true,127};
    // Retain the complete finite scenario: oscillations run613frames, unlike
    // the original121-frame campaign. Never evict history or relax its guards.
    const size_t Retention=FMath::Max<size_t>(512,size_t(Case.FrameCount));
    P::FRemoteRequestHistory H(Seed.Remote,Actions,256,Retention),Z(Seed.Remote,Actions,256,Retention);
    uint32 PartialMissing=0,PartialDifferent=0,PartialResponse=0;
    uint32 BodyMissing=0,BodyDifferent=0,BodyNeutralResponse=0;
    FMaxima StiffnessResponse;uint32 StiffnessPartialResponse=0;
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
        // Real native read-path test: only the local physical step sees a
        // deliberately different LIVE CVar. Its immutable original profile
        // must retain the chosen stiffness; the real authority/control graphs
        // run after the original registration value has been restored.
        const float ChangedLiveStiffness=OriginalLiveStiffness+17.0f;
        LiveStiffness->Set(ChangedLiveStiffness,ECVF_SetByCode);
        if(LiveStiffness->GetFloat()!=ChangedLiveStiffness)return false;
        const bool LocalAdvanced=Local->Step({{101,3,Input}},false,Why);
        const bool RegistrationUnchanged=LiveStiffness->GetFloat()==ChangedLiveStiffness;
        LiveStiffness->Set(OriginalLiveStiffness,ECVF_SetByCode);
        if(!RegistrationUnchanged||LiveStiffness->GetFloat()!=OriginalLiveStiffness)return false;
        // Predict advances the original ingress cursor. Read ONCE and share
        // that immutable received evidence between independent physical controls.
        const auto AuthorityEvidence=H.Predict(N),NeutralEvidence=Z.Predict(N);
        if(!LocalAdvanced||!Authority->Step({{101,3,AuthorityEvidence}},false,Why)
            ||!Neutral->Step({{101,3,NeutralEvidence}},false,Why)
            ||!StiffnessControl->Step({{101,3,NeutralEvidence}},false,Why))
        {Test.AddError(FString::Printf(TEXT("%s native step %llu: %s"),Case.Name,N,*Why));return false;}
        auto L=Local->ReadLatest(),A=Authority->ReadLatest(),B=Neutral->ReadLatest();
        if(!L||!A||!B||L->Frame!=N||A->Frame!=N||B->Frame!=N||!Body(*L)||!Body(*A)||!Body(*B))return false;
        if(!Difference.Add(Body(*L)->COMState,Body(*A)->COMState,N)||!Response.Add(Body(*A)->COMState,Body(*B)->COMState,N))return false;
        const auto SensitiveCut=StiffnessControl->ReadLatest();
        if(!SensitiveCut||!Body(*SensitiveCut)||!StiffnessResponse.Add(Body(*B)->COMState,Body(*SensitiveCut)->COMState,N))return false;
        if(P::ComparePrivateRoadPartialValues(B->Actors[0],SensitiveCut->Actors[0])==P::EPrivateRoadPartialComparison::Different)++StiffnessPartialResponse;
        const auto& LF=L->Actors[0];const auto& AF=A->Actors[0];
        if(!LF.OriginalBodyDomainsAvailable||!AF.OriginalBodyDomainsAvailable||!B->Actors[0].OriginalBodyDomainsAvailable
            ||LF.OriginalBodyDomainValues.empty()||AF.OriginalBodyDomainValues.empty()||B->Actors[0].OriginalBodyDomainValues.empty())++BodyMissing;
        else
        {
            if(LF.OriginalBodyDomainValues!=AF.OriginalBodyDomainValues)++BodyDifferent;
            if(AF.OriginalBodyDomainValues!=B->Actors[0].OriginalBodyDomainValues)++BodyNeutralResponse;
        }
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
    Test.AddInfo(FString::Printf(TEXT("[OriginalBodyDomainMeasure] case=%s fieldsPerStructSet=125 frames=%u unknownFrames=%u differingFrames=%u neutralResponseFrames=%u completeEpoch=0 restore=0"),
        Case.Name,uint32(Case.FrameCount),BodyMissing,BodyDifferent,BodyNeutralResponse));
    bool OK=Test.TestEqual(TEXT("actual declared native cache/wheel subset observed; missing is unknown"),PartialMissing,uint32(0));
    OK=Test.TestEqual(TEXT("actual original six body domains captured without unknown references"),BodyMissing,uint32(0))&&OK;
    if(!Case.ExpectMismatch)
        OK=Test.TestEqual(TEXT("matched actual original body domains equal at every paired frame"),BodyDifferent,uint32(0))&&OK;
    if(Case.Action<3)
        OK=Test.TestTrue(TEXT("physical input changes actual original body domains against independent neutral graph"),BodyNeutralResponse>0)&&OK;

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
    Test.AddInfo(FString::Printf(TEXT("[OriginalSettingsStiffnessSensitivity] case=%s frames=%u changedKinematicFrames=%u changedDeclaredNativeFrames=%u maxPositionCm=%.17g maxVelocityCmS=%.17g fullReaderCoverage=0"),
        Case.Name,uint32(Case.FrameCount),StiffnessResponse.DifferentFrames,StiffnessPartialResponse,StiffnessResponse.Position,StiffnessResponse.Velocity));
    OK=Test.TestTrue(TEXT("actual selected stiffness changes native physical state in a separately configured control"),StiffnessResponse.DifferentFrames>0||StiffnessPartialResponse>0)&&OK;
    // Always retire all worlds, including a failed comparison; no short-circuit.
    const bool LC=Local->Close(),AC=Authority->Close(),NC=Neutral->Close(),SC=StiffnessControl->Close();
    return Test.TestTrue(TEXT("all four actual native worlds retired"),LC&&AC&&NC&&SC)&&OK;
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
    if(!IASAtomicRoadCampaign::TestResponseChannelCapture(*this))return false;
    namespace P=Speed::Netcode::Prediction;namespace V=Speed::Input::V2;
    using namespace Speed::Netcode;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    P::FPrivateRoadCarSeed Seed; // Default policy MUST remain closed.
    Seed.DeclaredSettings=Speed::RoadSettings::MakeDeclaredRoadConformanceProfile();
    if(!TestTrue(TEXT("actual host explicitly selected all declared settings before birth"),bool(Seed.DeclaredSettings)))return false;
    Seed.Actor=101;Seed.Incarnation=3;Seed.Contract=Contract;Seed.Role=P::EBranchFilterRole::LocalDevice;
    Seed.Local={Speed::Input::EProducerKind::Device,42};Seed.LocalEpoch={3};
    Seed.Remote.Session=1;Seed.Remote.Epoch=2;Seed.Remote.Actor=101;Seed.Remote.Incarnation=3;
    Seed.Remote.OwnerPeer=4;Seed.Remote.ServerPeer=5;Seed.Remote.SourceProducer=42;Seed.Remote.SourceKind=0;Seed.Remote.Contract[0]=7;
    Seed.Initial.Location=FVector(0,0,45);Seed.Initial.Velocity=FVector(500,0,0);
    auto* BirthLiveStiffness=IConsoleManager::Get().FindConsoleVariable(TEXT("p.SkyLeague.Suspension.StiffnessScale"));
    if(!BirthLiveStiffness||!BirthLiveStiffness->IsVariableFloat())return false;
    const float OriginalBirthLive=BirthLiveStiffness->GetFloat();
    ON_SCOPE_EXIT{BirthLiveStiffness->Set(OriginalBirthLive,ECVF_SetByCode);};
    const float ForeignLive=OriginalBirthLive+17.0f;
    if(!FMath::IsFinite(OriginalBirthLive)||!FMath::IsFinite(ForeignLive)||ForeignLive==OriginalBirthLive)return false;
    BirthLiveStiffness->Set(ForeignLive,ECVF_SetByCode);
    if(BirthLiveStiffness->GetFloat()!=ForeignLive)return false;
    FActualRecipeObjectReadCensus AllRecipeReads;
#if UE_WITH_OBJECT_HANDLE_TRACKING
    const auto AllRecipeReadHandle=UE::CoreUObject::AddObjectHandleReadCallback(
        [&](TArrayView<const UObject* const> Objects){AllRecipeReads.Observe(Objects);});
    ON_SCOPE_EXIT{UE::CoreUObject::RemoveObjectHandleReadCallback(AllRecipeReadHandle);};
#endif
    FActualBirthReadScopeObserver BirthObserver;
    if(!GLog)return false;
    GUObjectArray.AddUObjectCreateListener(&BirthObserver);
    ON_SCOPE_EXIT{GUObjectArray.RemoveUObjectCreateListener(&BirthObserver);};
    FActualNativeBoundaryLogProbe BirthGCProbe;
    GLog->AddOutputDevice(&BirthGCProbe);
    ON_SCOPE_EXIT{GLog->RemoveOutputDevice(&BirthGCProbe);};
    uint32 ActualWorldCallbacks=0;bool LegacyWorldReadStayedLive=true;
    const auto CreationHandle=FWorldDelegates::OnPostWorldCreation.AddLambda([&](UWorld* Created)
    {
        if(!IsInGameThread()||!BirthObserver.InsideOriginalCreate.load())return;
        ++ActualWorldCallbacks;
        BirthGCProbe.World=Created;
        const float Read=Speed::RoadSettings::ReadOriginalOrLegacy<177>([&](){return BirthLiveStiffness->GetFloat();});
        LegacyWorldReadStayedLive=LegacyWorldReadStayedLive&&!Speed::RoadSettings::SelectedOriginalRoadSettings()&&Read==ForeignLive;
    });
    ON_SCOPE_EXIT
    {
        BirthObserver.InsideOriginalCreate.store(false,std::memory_order_release);
        FWorldDelegates::OnPostWorldCreation.Remove(CreationHandle);
    };
    BirthObserver.InsideOriginalCreate.store(true,std::memory_order_release);
    FString Why;auto World=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    BirthObserver.InsideOriginalCreate.store(false,std::memory_order_release);
    BirthLiveStiffness->Set(OriginalBirthLive,ECVF_SetByCode);
    bool BirthOK=TestEqual(TEXT("actual original world creation callback returned"),ActualWorldCallbacks,uint32(1));
    BirthOK=TestTrue(TEXT("real foreign legacy work in birth callback retains distinct live settings"),LegacyWorldReadStayedLive)&&BirthOK;
    BirthOK=TestTrue(TEXT("real Engine object creation callbacks executed"),BirthObserver.ActualObjectCreationCalls.load()>0)&&BirthOK;
    BirthOK=TestEqual(TEXT("no actual UObject constructor callback inherits original settings scope"),BirthObserver.RecipeScopeCreationCalls.load(),uint64(0))&&BirthOK;
    AddInfo(FString::Printf(TEXT("[OriginalSettingsBirthCallback] actualWorldCallbacks=%u actualObjectCallbacks=%llu objectCallbacksInsideRecipe=%llu legacyLiveRead=%d fullCallbackCensus=0"),
        ActualWorldCallbacks,BirthObserver.ActualObjectCreationCalls.load(),BirthObserver.RecipeScopeCreationCalls.load(),int(LegacyWorldReadStayedLive)));
    BirthOK=TestEqual(TEXT("actual birth output dispatch reached"),BirthGCProbe.Calls,1)&&BirthOK;
    BirthOK=TestEqual(TEXT("genuine birth diagnostic GC returned"),BirthGCProbe.ActualGCReturns,1)&&BirthOK;
    BirthOK=TestEqual(TEXT("all eight actual prepared native/reference wheel UObjects remain reachable after GC"),BirthGCProbe.LivePreparedWheelObjects,8)&&BirthOK;
    BirthOK=TestTrue(TEXT("birth output callback has no native recipe or physical collection scope"),BirthGCProbe.OutsideRecipe&&BirthGCProbe.OutsideCollection)&&BirthOK;
    if(!BirthOK){if(World)World->Close();return false;}
    if(!TestTrue(*FString::Printf(TEXT("actual default-closed native graph: %s"),*Why),bool(World)))return false;
    AllRecipeReads.DuringBirth=false;
    FActualHandleReadWitness HandleReads;HandleReads.Targets.reserve(5);
    if(auto* W=BirthGCProbe.World.Get())for(TActorIterator<ASpeedBranchCar> It(W);It;++It)
        if(auto* C=Cast<USpeedBranchWheeledComponent>(It->GetVehicleMovement()))
        {
            for(int32 Index=0;Index<4;++Index)
                if(auto* Body=Cast<USpeedBranchWheelSubBody>(C->GetDefaultSubobjectByName(USpeedWheeledComponent::WheelNames[Index])))
                    HandleReads.Targets.push_back({Body,nullptr});
            if(auto* Box=Cast<USpeedBranchBoxSubBody>(C->GetDefaultSubobjectByName(USpeedWheeledComponent::HitboxName)))
                HandleReads.Targets.push_back({Box,nullptr});
        }
    if(!TestEqual(TEXT("real original fixture wheel targets retained before read callback registration"),HandleReads.Targets.size(),size_t(5)))return false;
#if UE_WITH_OBJECT_HANDLE_TRACKING
    const auto ActualReadHandle=UE::CoreUObject::AddObjectHandleReadCallback(
        [&](TArrayView<const UObject* const> Objects){HandleReads.Observe(Objects);});
    ON_SCOPE_EXIT{UE::CoreUObject::RemoveObjectHandleReadCallback(ActualReadHandle);};
#endif
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
    OK=TestTrue(TEXT("actual initial publication retains full selected declared settings bytes"),
        Baseline->DeclaredSettingsIdentity==Seed.DeclaredSettings->CanonicalIdentity())&&OK;
    auto Copy=std::make_shared<const P::FPrivateRoadPublication>(*Baseline);
    OK=TestFalse(TEXT("value-identical caller copy is not the retained original publication"),World->ReleaseInitialPublication(Copy,Why))&&OK;
    const auto Next=Input(1);bool AllBlocked=true;
    for(unsigned Attempt=0;Attempt<512;++Attempt)
        if(World->Step({{101,3,Next}},false,Why)||World->ReadLatest()!=Baseline)AllBlocked=false;
    OK=TestTrue(TEXT("512 attempted frames cannot expire the gate or mutate the physical publication"),AllBlocked)&&OK;
    FActualNativeBoundaryLogProbe ActivationProbe;ActivationProbe.Case=FActualNativeBoundaryLogProbe::ECase::ActivationFacade;ActivationProbe.World=BirthGCProbe.World;
    const auto OldLogTempVerbosity=LogTemp.GetVerbosity();LogTemp.SetVerbosity(ELogVerbosity::Verbose);
    GLog->AddOutputDevice(&ActivationProbe);
    ON_SCOPE_EXIT{GLog->RemoveOutputDevice(&ActivationProbe);LogTemp.SetVerbosity(OldLogTempVerbosity);};
    OK=TestTrue(TEXT("explicit owner releases the actual retained initial publication"),World->ReleaseInitialPublication(Baseline,Why))&&OK;
    OK=TestEqual(TEXT("real optional activation diagnostic dispatched before commit read-back"),ActivationProbe.Calls,1)&&OK;
    OK=TestTrue(TEXT("genuine activation callback changes facade freeze and executes legacy queue path"),ActivationProbe.FacadeFreezeTransition&&ActivationProbe.ActualLegacyQueueReturned)&&OK;
    OK=TestTrue(TEXT("activation callback has no original recipe/physical collection scope"),ActivationProbe.OutsideRecipe&&ActivationProbe.OutsideCollection)&&OK;
    OK=TestFalse(TEXT("duplicate release is refused"),World->ReleaseInitialPublication(Baseline,Why))&&OK;
    OK=TestTrue(TEXT("common activation advances the next actual physical frame"),World->Step({{101,3,Next}},false,Why))&&OK;
    const auto Activated=World->ReadLatest();
    OK=TestTrue(TEXT("actual CanMove and freeze state reflect explicit activation"),Activated&&Activated->Frame==1
        &&!Activated->AwaitingInitialPublication&&Activated->Actors[0].CanMove&&!Activated->Actors[0].Frozen
        &&Activated->Actors[0].CountdownFrames==0)&&OK;
    OK=TestTrue(TEXT("retained old baseline remains immutable and closed"),Baseline->Frame==0&&Baseline->AwaitingInitialPublication
        &&!Baseline->Actors[0].CanMove)&&OK;
    // Genuine tracked reads of the same five live targets, outside the native
    // route. No callback is removed, suppressed, filtered or counter-adjusted.
    uint64 DeliberatePositiveReads=0;
    for(auto& T:HandleReads.Targets)
    {
        auto* Body=T.Body.Get();
        OK=TestTrue(TEXT("ordinary positive-control target retains original birth identity"),Body!=nullptr)&&OK;
        if(!Body)continue;
        const uint64 Before=T.OutsideReads;
        TObjectPtr<USSubBody> Positive=Body;
        OK=TestTrue(TEXT("ordinary same-target tracked read returns actual object"),Positive.Get()==Body)&&OK;
#if UE_WITH_OBJECT_HANDLE_TRACKING
        OK=TestTrue(TEXT("this deliberate Get invokes the actual callback for this specific target"),T.OutsideReads>Before)&&OK;
#endif
        DeliberatePositiveReads+=T.OutsideReads-Before;
    }
#if UE_WITH_OBJECT_HANDLE_TRACKING
    OK=TestEqual(TEXT("no actual wheel/hitbox read callback enters scoped native step/capture"),HandleReads.ScopedTargetReads,uint64(0))&&OK;
    OK=TestTrue(TEXT("five deliberate tracked reads still invoke the actual callback"),DeliberatePositiveReads>=5)&&OK;
#endif
    AddInfo(FString::Printf(TEXT("[ActualObjectHandleReadWitness] trackingCompiled=%d scopedTargetReads=%llu ordinaryPositiveReads=%llu physicalMutationAttempted=0 targetSet=wheel4_hitbox1 birthCallbackCensus=0 escapedViewExclusionCertified=0 completeRestoreLease=0"),
        int(UE_WITH_OBJECT_HANDLE_TRACKING),HandleReads.ScopedTargetReads,DeliberatePositiveReads));
    {
        const uint64 Before=AllRecipeReads.ControlObjects;
        P::FPrivateRoadPhysicsWorld::FOriginalWorldReadProbe Probe;
        {
            AllRecipeReads.DuringControl=true;ON_SCOPE_EXIT{AllRecipeReads.DuringControl=false;};
            Probe=World->ObserveOriginalWorldReadForDiagnostics();
        }
        const uint64 DeliberateReadDelta=AllRecipeReads.ControlObjects-Before;
        OK=TestTrue(TEXT("actual owner admitted the read-only known-world control"),Probe.Admitted)&&OK;
        if(Probe.Admitted)
        {
            OK=TestTrue(TEXT("owner tracked world read returned the actual retained original"),Probe.SameOriginal)&&OK;
#if UE_WITH_OBJECT_HANDLE_TRACKING
            OK=TestEqual(TEXT("one deliberate tracked world read is observed once without self-amplification"),DeliberateReadDelta,uint64(1))&&OK;
            OK=TestEqual(TEXT("actual observer never reenters itself while recording native identity"),AllRecipeReads.MaxObserverDepth,uint32(1))&&OK;
#endif
        }
    }
    AddInfo(FString::Printf(TEXT("[ActualRecipeObjectReadCensus] trackingCompiled=%d birthObjectReads=%llu stepObjectReads=%llu controlObjectReads=%llu nullReads=%llu uniqueObjects=%u overflow=%d unknownIdentities=%llu nativeSerialAllocationCalls=%llu maxObserverDepth=%u completeCallbackClosure=0 completeRestoreLease=0"),
        int(UE_WITH_OBJECT_HANDLE_TRACKING),AllRecipeReads.BirthObjects,AllRecipeReads.StepObjects,AllRecipeReads.ControlObjects,AllRecipeReads.NullObjects,AllRecipeReads.Used,int(AllRecipeReads.Overflow),AllRecipeReads.UnknownIdentities,AllRecipeReads.NativeSerialAllocationCalls,AllRecipeReads.MaxObserverDepth));
    for(uint32 Index=0;Index<AllRecipeReads.Used;++Index)
    {
        const auto& Entry=AllRecipeReads.Entries[Index];bool OriginalLive=false;
        FString Name=TEXT("<original identity no longer live>"),Type=TEXT("<unknown>");
        {
            // Actual short GC exclusion for read-back; never infer a pin/lease
            // from the saved pair. All names/classes stay outside recipe/math.
            FGCScopeGuard DiagnosticIdentityReadBack;
            const auto* Item=Entry.ObjectIndex>=0?GUObjectArray.IndexToObject(Entry.ObjectIndex,false):nullptr;
            if(Item&&!Item->IsUnreachable()&&Entry.ObjectSerial>0&&Item->GetSerialNumber()==Entry.ObjectSerial
                &&Item->GetObject()==static_cast<const UObjectBase*>(Entry.OriginalAddress))
            {
                const auto* Object=static_cast<const UObject*>(Item->GetObject());
                if(Object&&!Object->HasAnyFlags(RF_BeginDestroyed|RF_FinishDestroyed))
                {Name=Object->GetPathName();Type=Object->GetClass()->GetName();OriginalLive=true;}
            }
        }
        AddInfo(FString::Printf(TEXT("[ActualRecipeObjectReadTarget] ordinal=%u birthReads=%llu stepReads=%llu controlReads=%llu originalLive=%d class=%s path=%s"),
            Index,Entry.BirthReads,Entry.StepReads,Entry.ControlReads,int(OriginalLive),*Type,*Name));
    }
    const bool Closed=World->Close();
    FActualNativeBoundaryLogProbe InvalidBirth;InvalidBirth.Case=FActualNativeBoundaryLogProbe::ECase::DestroyBirthDriver;
    const auto InvalidBirthHandle=FWorldDelegates::OnPostWorldCreation.AddLambda([&](UWorld* Created){if(IsInGameThread())InvalidBirth.World=Created;});
    GLog->AddOutputDevice(&InvalidBirth);
    ON_SCOPE_EXIT{GLog->RemoveOutputDevice(&InvalidBirth);FWorldDelegates::OnPostWorldCreation.Remove(InvalidBirthHandle);};
    auto RefusedBirth=P::FPrivateRoadPhysicsWorld::Create({Seed},Why);
    OK=TestTrue(TEXT("genuine driver destruction in birth callback refuses original construction"),!RefusedBirth)&&OK;
    OK=TestTrue(TEXT("actual invalidating callback destroyed driver and genuine GC returned"),InvalidBirth.Calls==1&&InvalidBirth.ActualDriverDestroyReturned&&InvalidBirth.ActualGCReturns==1)&&OK;
    if(RefusedBirth)OK=TestTrue(TEXT("unexpected accepted negative world still retires"),RefusedBirth->Close())&&OK;
    AddInfo(FString::Printf(TEXT("[ActualNativeBoundaryCallbacks] birthGC=%d livePreparedWheels=%d activationFacade=%d invalidBirthDestroy=%d invalidBirthGC=%d fullCallbackCensus=0"),
        BirthGCProbe.ActualGCReturns,BirthGCProbe.LivePreparedWheelObjects,ActivationProbe.Calls,int(InvalidBirth.ActualDriverDestroyReturned),InvalidBirth.ActualGCReturns));
    AddInfo(TEXT("[InitialPublicationGate] actualNative=1 blockedAttempts=512 initialFrame=0 activationFrame=1 timerRelease=0 fullHiddenStateCertificate=0 networkAuthenticationQualified=0. Tests physical local-owner gate only; original-process session comparison is separately required by the process probe."));
    return TestTrue(TEXT("actual native startup graph retired"),Closed)&&OK&&!HasAnyErrors();
}
#endif
