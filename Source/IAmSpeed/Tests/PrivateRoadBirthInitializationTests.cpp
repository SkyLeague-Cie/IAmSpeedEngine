#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadBirthInitialization,
    "IAmSpeed.Netcode.Prediction.PrivateRoadBirthInitialization",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadBirthInitialization::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    if(!TestTrue(TEXT("actual birth controls run on GT"),IsInGameThread()))return false;
    auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});if(!Contract)return false;
    std::vector<P::FPrivateRoadCarSeed> Seeds;
    for(U64 Index=0;Index<2;++Index)
    {
        P::FPrivateRoadCarSeed Seed;Seed.Actor=101+Index;Seed.Incarnation=3;Seed.Contract=Contract;
        Seed.StartPolicy=P::EPrivateRoadStartPolicy::ExplicitUnitRecipe;
        Seed.Local={Speed::Input::EProducerKind::Device,42+Index};Seed.LocalEpoch={3};
        Seed.Remote.Session=1;Seed.Remote.Epoch=2;Seed.Remote.Actor=Seed.Actor;Seed.Remote.Incarnation=3;
        Seed.Remote.OwnerPeer=4;Seed.Remote.ServerPeer=5;Seed.Remote.SourceProducer=6+Index;Seed.Remote.SourceKind=0;Seed.Remote.Contract[0]=7;
        Seed.Initial.Location=FVector(0,Index?1000:-1000,45);Seeds.push_back(Seed);
    }
    FString Why;auto Foreign=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    if(!TestTrue(*FString::Printf(TEXT("actual foreign graph: %s"),*Why),bool(Foreign)))return false;
    P::FPrivateRoadBirthInitializationProbe ObservedProbe;
    auto Observed=P::FPrivateRoadPhysicsWorld::CreateWithBirthInitializationProbeForDiagnostics(
        Seeds,*Foreign,P::EPrivateRoadBirthProbeMode::Observe,ObservedProbe,Why);
    if(!TestTrue(*FString::Printf(TEXT("actual private native initialization: %s"),*Why),bool(Observed)))return false;
    const auto CheckBirth=[&](const TCHAR* Name,const P::FPrivateRoadBirthInitializationProbe& Probe)
    {
        TestTrue(*(FString(Name)+TEXT(" genuine Birth operation")),Probe.ActualBirthOperation);
        TestTrue(*(FString(Name)+TEXT(" foreign body refused")),Probe.ForeignBodyRefused);
        TestTrue(*(FString(Name)+TEXT(" foreign parent refused")),Probe.ForeignParentRefused);
        TestTrue(*(FString(Name)+TEXT(" foreign inverse subject refused")),Probe.ForeignInverseRefused);
        TestTrue(*(FString(Name)+TEXT(" Step-only hit route refuses actual Birth")),Probe.StepHitRefusedInBirth);
        TestTrue(*(FString(Name)+TEXT(" Step-only reset refuses actual Birth")),Probe.StepResetRefusedInBirth);
        TestTrue(*(FString(Name)+TEXT(" Step-only response refuses actual Birth")),Probe.StepResolveRefusedInBirth);
        TestTrue(*(FString(Name)+TEXT(" original named body values unchanged by refusals")),Probe.OwnBodyUnchangedAfterRefusals);
    };
    CheckBirth(TEXT("observed"),ObservedProbe);
    TestTrue(TEXT("actual Birth append refusal keeps nonempty output unchanged"),ObservedProbe.CanonicalMassAppendBirthRefused
        &&ObservedProbe.CanonicalMassAppendBirthOutUnchanged&&ObservedProbe.CanonicalMassAppendBirthSubjectSolid);
    AddInfo(FString::Printf(TEXT("[PrivateRoadCanonicalMassAppendBirthRefusal] refused=%d unchanged=%d actualBirth=%d subjectSolid=%d. Genuine original Birth call and real solid subject; no capability constructed or retained."),
        int(ObservedProbe.CanonicalMassAppendBirthRefused),int(ObservedProbe.CanonicalMassAppendBirthOutUnchanged),int(ObservedProbe.ActualBirthOperation),int(ObservedProbe.CanonicalMassAppendBirthSubjectSolid)));

    TestTrue(TEXT("actual Birth refuses Retirement clear before named original/reference effects"),ObservedProbe.RetirementClearRefusedInBirth&&ObservedProbe.RetirementBirthFootprintUnchanged);
    AddInfo(FString::Printf(TEXT("[PrivateRoadMassPhaseBoundary] resetRefusedInBirth=%d resolveRefusedInBirth=%d beforeWrite=%d. Genuine Birth operation; no minted diagnostic authority."),int(ObservedProbe.StepResetRefusedInBirth),int(ObservedProbe.StepResolveRefusedInBirth),int(ObservedProbe.OwnBodyUnchangedAfterRefusals)));
    TestEqual(TEXT("ten actual initializer returns for two stock cars"),ObservedProbe.InitializersReturned,uint32(10));
    TestFalse(TEXT("observation does not force factory failure"),ObservedProbe.FaultAfterFirstInitializerReached||ObservedProbe.CreateReturnedNull);
    std::array<Consumed::FActionPolicy,ActionCount> Actions{};
    Actions[0]={true,true,false,255};Actions[1]={true,true,false,255};Actions[2]={true,true,true,127};
    P::FRemoteRequestHistory HA(Seeds[0].Remote,Actions,256,512),HB(Seeds[1].Remote,Actions,256,512);
    const auto Packet=[&](size_t Actor)
    {
        const auto& Bind=Seeds[Actor].Remote;FPacket Q;Q.Kind=EKind::AuthorityInput;
        Q.Session=Bind.Session;Q.Epoch=Bind.Epoch;Q.Actor=Bind.Actor;Q.Incarnation=Bind.Incarnation;Q.Contract=Bind.Contract;
        Q.Frame=0;Q.Sequence=1;Q.Input.SourceSequence=1;Q.Input.DeviceGeneration=1;Q.Input.Reset=true;
        Q.Input.ProducerId=Bind.SourceProducer;Q.Input.ProducerKind=Bind.SourceKind;return Q;
    };
    if(HA.Confirm(5,Packet(0)).Status!=EResult::Accepted||HB.Confirm(5,Packet(1)).Status!=EResult::Accepted)return false;
    std::vector<P::FPrivateRoadRequest> Requests={{101,3,HA.Predict(0)},{102,3,HB.Predict(0)}};
    P::FPrivateRoadHitAcceptanceProbe StepProbe;
    if(!TestTrue(*FString::Printf(TEXT("real Step after actual birth: %s"),*Why),
        Observed->StepWithHitAcceptanceProbeForDiagnostics(Requests,false,*Foreign,StepProbe,Why)))return false;
    TestTrue(TEXT("Birth-only Initialize refuses actual Step before writing"),StepProbe.BirthInitializationRefusedDuringStep);
    if(!TestTrue(TEXT("independent native reference still advances"),Foreign->Step(Requests,false,Why)))return false;
    auto O=Observed->ReadLatest(),F=Foreign->ReadLatest();
    if(!TestTrue(TEXT("actual frame-zero publications"),O&&F&&O->Frame==0&&F->Frame==0&&O->Actors.size()==2&&F->Actors.size()==2&&O->Physics.PresentationBodies.Num()==2&&F->Physics.PresentationBodies.Num()==2))return false;
    for(size_t Index=0;Index<2;++Index)
    {
        TestTrue(TEXT("real original solid recipe and non-solid initialization witness"),O->Actors[Index].NativeSolidInitializationWitness&&F->Actors[Index].NativeSolidInitializationWitness);
        const auto* XBody=O->Physics.PresentationBodies.FindByPredicate([&](const auto& Body){return Body.StableId==O->Actors[Index].NativeStableId;});
        const auto* YBody=F->Physics.PresentationBodies.FindByPredicate([&](const auto& Body){return Body.StableId==F->Actors[Index].NativeStableId;});
        if(!TestTrue(TEXT("actual original actor-to-presentation bindings present"),XBody&&YBody))return false;
        const auto& X=XBody->COMState;const auto& Y=YBody->COMState;
        TestTrue(TEXT("same-frame six-field COM values equal after initialization"),X.Location==Y.Location&&X.Velocity==Y.Velocity&&X.Acceleration==Y.Acceleration
            &&X.Rotation==Y.Rotation&&X.AngularVelocity==Y.AngularVelocity&&X.AngularAcceleration==Y.AngularAcceleration);
    }
    P::FPrivateRoadBirthInitializationProbe AbortedProbe;
    auto Aborted=P::FPrivateRoadPhysicsWorld::CreateWithBirthInitializationProbeForDiagnostics(
        Seeds,*Foreign,P::EPrivateRoadBirthProbeMode::FailAfterFirstInitializer,AbortedProbe,Why);
    TestFalse(TEXT("forced late factory abort actually returns no owner"),bool(Aborted));
    CheckBirth(TEXT("late abort"),AbortedProbe);
    TestTrue(TEXT("aborted Birth still refuses Retirement clear before effects"),AbortedProbe.RetirementClearRefusedInBirth&&AbortedProbe.RetirementBirthFootprintUnchanged);
    TestTrue(TEXT("failure point follows one actual returned initializer"),AbortedProbe.FaultAfterFirstInitializerReached&&AbortedProbe.CreateReturnedNull);
    TestEqual(TEXT("one of five first-car initializers actually returned"),AbortedProbe.InitializersReturned,uint32(1));
    TestEqual(TEXT("only genuine preexisting native/config trace objects retained"),AbortedProbe.LifetimeObservers.size(),size_t(4));
    uint64 ActualConstructors=0;
    for(const auto& Trace:AbortedProbe.LifetimeObservers)
    {
        if(!TestTrue(TEXT("actual standalone counter allocation"),bool(Trace)))return false;
        const auto Values=Trace->Read();ActualConstructors+=Values.NativeConstructionReturns;
        TestEqual(TEXT("every constructed native object actually returned its base destructor"),Values.NativeBaseDestructionReturns,Values.NativeConstructionReturns);
        TestEqual(TEXT("configuration destructor actually entered after factory unwind"),Values.ConfigurationDestructionEntries,uint64(1));
        TestFalse(TEXT("configuration never destroyed before native base return"),Values.ConfigurationDestructionBeforeNativeReturn);
    }
    TestTrue(TEXT("late abort followed real native construction"),ActualConstructors>0);
    Requests={{101,3,HA.Predict(1)},{102,3,HB.Predict(1)}};
    TestTrue(TEXT("foreign graph remains usable after other graph abort"),Foreign->Step(Requests,false,Why));
    const auto CheckActualRetirement=[&](const TCHAR* Scenario,const std::shared_ptr<const P::FPrivateRoadRetirementBindingTrace>& Trace) {
        if(!TestTrue(TEXT("independent actual retirement counter allocation"),bool(Trace)))return;
        TestTrue(TEXT("real Close returned after actual world callback"),Trace->CloseReturns==1&&Trace->WorldDestroyReturns==1);
        TestTrue(TEXT("actual live survivors cleared; allskip is not positive"),Trace->LiveResolved>0&&Trace->FacadeClearReturns==Trace->LiveResolved);
        TestTrue(TEXT("positive has a real facade pointer present before clear"),Trace->AnyFacadeBindingPresentBefore>0);
        TestTrue(TEXT("three actual facade cells are null after each returned clear"),Trace->FacadeWheelNullAfter==Trace->FacadeClearReturns&&Trace->FacadeSuspensionNullAfter==Trace->FacadeClearReturns&&Trace->FacadeConfigurationNullAfter==Trace->FacadeClearReturns);
        TestTrue(TEXT("actual nonnull to null transitions match per-field observed presence"),Trace->FacadeWheelChangedFromPresent==Trace->FacadeWheelPresentBefore&&Trace->FacadeSuspensionChangedFromPresent==Trace->FacadeSuspensionPresentBefore&&Trace->FacadeConfigurationChangedFromPresent==Trace->FacadeConfigurationPresentBefore);
        AddInfo(FString::Printf(TEXT("[PrivateRoadRetirementFacadeEffects] scenario=%s anyBefore=%u wheelBefore=%u suspensionBefore=%u configurationBefore=%u wheelNullAfter=%u suspensionNullAfter=%u configurationNullAfter=%u wheelChanged=%u suspensionChanged=%u configurationChanged=%u"),Scenario,Trace->AnyFacadeBindingPresentBefore,Trace->FacadeWheelPresentBefore,Trace->FacadeSuspensionPresentBefore,Trace->FacadeConfigurationPresentBefore,Trace->FacadeWheelNullAfter,Trace->FacadeSuspensionNullAfter,Trace->FacadeConfigurationNullAfter,Trace->FacadeWheelChangedFromPresent,Trace->FacadeSuspensionChangedFromPresent,Trace->FacadeConfigurationChangedFromPresent));
        TestEqual(TEXT("actual weak population partitions"),Trace->WeakVisited,Trace->WeakUnresolved+Trace->GarbageFlagSkips+Trace->LiveResolved);
        TestTrue(TEXT("facade clears kept named original triples before later disconnection"),Trace->OriginalPointerTriplesCompared>0&&Trace->FacadeClearsKeptOriginalPointers);
        TestEqual(TEXT("separate retained original wheel roots actually disconnected"),Trace->OriginalWheelDisconnectReturns,Trace->OriginalPointerTriplesCompared);
        AddInfo(FString::Printf(TEXT("[PrivateRoadRetirementBindings] scenario=%s visited=%u unresolved=%u flagged=%u live=%u cleared=%u originalCompared=%u originalKept=%d disconnected=%u worldReturned=%u closeReturned=%u"),Scenario,Trace->WeakVisited,Trace->WeakUnresolved,Trace->GarbageFlagSkips,Trace->LiveResolved,Trace->FacadeClearReturns,Trace->OriginalPointerTriplesCompared,int(Trace->FacadeClearsKeptOriginalPointers),Trace->OriginalWheelDisconnectReturns,Trace->WorldDestroyReturns,Trace->CloseReturns));
    };
    CheckActualRetirement(TEXT("partial-birth"),AbortedProbe.RetirementObserver);
    const auto NormalObserver=Observed->CaptureRetirementBindingObserverForDiagnostics();
    P::FPrivateRoadRetirementForeignProbe ForeignRetirement;
    TestTrue(TEXT("observed owner closes with actual Retirement foreign-root refusal"),Observed->CloseWithForeignRetirementBindingProbeForDiagnostics(*Foreign,ForeignRetirement));
    TestTrue(TEXT("actual Retirement Root validator accepts own and refuses actual foreign allocation before named pointer effects"),ForeignRetirement.OwnRootValidationReturns==1&&ForeignRetirement.ActualRetirementCalls==1&&ForeignRetirement.ForeignRootValidatorRefused&&ForeignRetirement.NamedFootprintUnchanged);
    CheckActualRetirement(TEXT("normal"),NormalObserver);
    TestTrue(TEXT("independent foreign original owner closes"),Foreign->Close());
    AddInfo(FString::Printf(TEXT("[PrivateRoadRetirementBoundary] birthRefused=%d birthBeforeWrite=%d ownRootValidationReturns=%u foreignRetirementCalls=%u foreignRootValidatorRefused=%d foreignBeforeWrite=%d namedFacadeObserved=%u namedUnresolved=%u namedFlagged=%u foreignLeafProof=0"),int(ObservedProbe.RetirementClearRefusedInBirth),int(ObservedProbe.RetirementBirthFootprintUnchanged),ForeignRetirement.OwnRootValidationReturns,ForeignRetirement.ActualRetirementCalls,int(ForeignRetirement.ForeignRootValidatorRefused),int(ForeignRetirement.NamedFootprintUnchanged),ForeignRetirement.NamedFacadeSubjectsObserved,ForeignRetirement.NamedWeakUnresolved,ForeignRetirement.NamedGarbageFlagSkips));
    AddInfo(FString::Printf(TEXT("[PrivateRoadBirthInitialization] observedInitializers=%u abortedInitializers=%u actualNativeCtorReturns=%llu birthForeignGuards=4 actualStepRejectsBirth=1 lateAbortReturned=1. Partial reinitialization on an allocated first-car graph only; not earlier allocation failures, complete causal state/source/lifetime coverage, restoration or a connected match."),ObservedProbe.InitializersReturned,AbortedProbe.InitializersReturned,ActualConstructors));
    return !HasAnyErrors();
}
#endif
