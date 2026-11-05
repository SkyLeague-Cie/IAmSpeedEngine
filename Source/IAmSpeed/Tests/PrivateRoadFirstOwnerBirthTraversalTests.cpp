#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadFirstOwnerBirthTraversal,
    "IAmSpeed.Netcode.Prediction.PrivateRoadFirstOwnerBirthTraversal",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadFirstOwnerBirthTraversal::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    if(!TestTrue(TEXT("real first owner controls use GT"),IsInGameThread()))return false;
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
    const auto CheckTypedInitializers=[&](const TCHAR* Run,const P::FPrivateRoadBirthInitializationProbe& Probe,size_t Expected)
    {
        for(const auto& Row:Probe.ReferenceInitializerCases)
        {
            AddInfo(FString::Printf(TEXT("[PrivateRoadTypedReferenceInitializerCase] run=%s actor=%llu entry=%s subject=%s parent=%s positiveBefore=%d attempted=%d refused=%d positiveAfter=%d ownOriginal=%d ownReference=%d foreignOriginal=%d foreignReference=%d ownBodies=%u foreignBodies=%u. Actual early Birth witness; ten named bodies only, no complete scene/lifetime proof."),
                Run,Row.Actor,*Row.Entry,*Row.Subject,*Row.Parent,int(Row.PositiveBefore),int(Row.Attempted),int(Row.Refused),int(Row.PositiveAfter),
                int(Row.OwnOriginal),int(Row.OwnReference),int(Row.ForeignOriginal),int(Row.ForeignReference),Row.OwnBodies,Row.ForeignBodies));
            TestTrue(TEXT("typed real refusal preserves named original/reference bodies with positive validator controls"),Row.PositiveBefore&&Row.Attempted&&Row.Refused&&Row.PositiveAfter
                &&Row.OwnOriginal&&Row.OwnReference&&Row.ForeignOriginal&&Row.ForeignReference&&Row.OwnBodies==5&&Row.ForeignBodies==5&&Row.Actor!=0);
        }
        AddInfo(FString::Printf(TEXT("[PrivateRoadTypedReferenceInitializers] run=%s rows=%u typedInitializers=%u validatorReturns=%u positiveChecks=%u. Step/Retirement, isolated mixed backend and private-final inverse invalidation NOT_PROVED; no witness retention or reconstruction."),
            Run,uint32(Probe.ReferenceInitializerCases.size()),Probe.EarlyReferenceInitializers,Probe.ReferenceInitializerSubjectChecks,Probe.ReferenceInitializerPositiveChecks));
        TestEqual(TEXT("all actual typed initializer attempts exported including early abort"),Probe.ReferenceInitializerCases.size(),Expected);
        TestTrue(TEXT("actual successful subject validator returns are populated"),Probe.ReferenceInitializerSubjectChecks>0);
        TestEqual(TEXT("two positive subjects before and after each refusal"),Probe.ReferenceInitializerPositiveChecks,uint32(Expected*4));
    };
    FString Why;auto Foreign=P::FPrivateRoadPhysicsWorld::Create(Seeds,Why);
    if(!TestTrue(*FString::Printf(TEXT("actual independent foreign graph: %s"),*Why),bool(Foreign)))return false;
    P::FPrivateRoadBirthInitializationProbe EarlyAbort;
    auto EarlyAborted=P::FPrivateRoadPhysicsWorld::CreateWithBirthInitializationProbeForDiagnostics(
        Seeds,*Foreign,P::EPrivateRoadBirthProbeMode::AbortAfterEarlyReferenceOwner,EarlyAbort,Why);
    CheckTypedInitializers(TEXT("early-abort"),EarlyAbort,size_t(6));
    TestFalse(TEXT("real early reference diagnostic returns no owner"),bool(EarlyAborted));
    TestTrue(TEXT("actual early owner returned before explicit partial-birth abort"),EarlyAbort.CreateReturnedNull&&EarlyAbort.EarlyReferenceAbortReached);
    TestEqual(TEXT("one early owner actually returned"),EarlyAbort.EarlyReferenceOwnerReturns,uint32(1));
    TestEqual(TEXT("four two-null leaves returned; not non-null transition counts"),EarlyAbort.EarlyReferenceTwoNullReturns,uint32(4));
    TestEqual(TEXT("five actual facade initializers returned"),EarlyAbort.EarlyReferenceInitializers,uint32(5));
    TestEqual(TEXT("three invalid actor/component early entries refused"),EarlyAbort.EarlyReferenceGuardRefusals,uint32(3));
    TestTrue(TEXT("early refusals keep named own/reference/original and foreign owner targets"),EarlyAbort.EarlyReferenceRefusalsBeforeWrite);
    TestTrue(TEXT("real callback-boundary checks returned"),EarlyAbort.EarlyReferenceBoundaryChecks>0);
    if(!TestEqual(TEXT("four actual configuration observers retained after pre-vehicle abort"),EarlyAbort.LifetimeObservers.size(),size_t(4)))
    {
        AddInfo(TEXT("[PrivateRoadEarlyReferenceOwnerAbortUnknown] reason=missing-actual-lifetime-observers nativeConstructors=UNKNOWN"));
        return false;
    }
    uint64 EarlyNativeConstructors=0,EarlyConfigurationDestructions=0;
    for(const auto& Trace:EarlyAbort.LifetimeObservers)
    {
        if(!TestTrue(TEXT("actual early configuration observer is populated"),bool(Trace)))return false;
        const auto V=Trace->Read();EarlyNativeConstructors+=V.NativeConstructionReturns;
        EarlyConfigurationDestructions+=V.ConfigurationDestructionEntries;
        TestEqual(TEXT("pre-vehicle abort has no native destructor invented"),V.NativeBaseDestructionReturns,V.NativeConstructionReturns);
        TestEqual(TEXT("actual preexisting configuration destructor returned"),V.ConfigurationDestructionEntries,uint64(1));
        TestFalse(TEXT("configuration did not precede actual native destructor return"),V.ConfigurationDestructionBeforeNativeReturn);
    }
    TestEqual(TEXT("neither reference nor original vehicle was constructed"),EarlyNativeConstructors,uint64(0));
    AddInfo(FString::Printf(TEXT("[PrivateRoadEarlyReferenceOwnerAbort] ownerReturns=%u twoNullReturns=%u initializers=%u boundaryChecks=%u refusals=%u beforeWrite=%d nativeConstructors=%llu lifetimeObservers=%llu configurationDestructionEntries=%llu. Actual nonempty independent observers read only after real partial-owner unwind. Two-null returns include null-to-null; no transition or lifetime completeness claim."),
        EarlyAbort.EarlyReferenceOwnerReturns,EarlyAbort.EarlyReferenceTwoNullReturns,EarlyAbort.EarlyReferenceInitializers,
        EarlyAbort.EarlyReferenceBoundaryChecks,EarlyAbort.EarlyReferenceGuardRefusals,int(EarlyAbort.EarlyReferenceRefusalsBeforeWrite),EarlyNativeConstructors,
        static_cast<uint64>(EarlyAbort.LifetimeObservers.size()),EarlyConfigurationDestructions));
    P::FPrivateRoadBirthInitializationProbe Abort;
    auto Aborted=P::FPrivateRoadPhysicsWorld::CreateWithBirthInitializationProbeForDiagnostics(
        Seeds,*Foreign,P::EPrivateRoadBirthProbeMode::ProbeFirstOwnerAndAbort,Abort,Why);
    CheckTypedInitializers(TEXT("original-abort"),Abort,size_t(6));
    AddInfo(FString::Printf(TEXT("[PrivateRoadFirstOwnerBirthAbortAttempt] null=%d stage=%s probeEntered=%d probeReturned=%d probeSucceeded=%d locationApi=%d rotationApi=%d location=%s rotation=%s why=%s"),
        int(Abort.CreateReturnedNull),*Abort.FirstOwnerLastStage,int(Abort.FirstOwnerProbeCallEntered),int(Abort.FirstOwnerProbeCallReturned),
        int(Abort.FirstOwnerProbeSucceeded),int(Abort.FirstOwnerActorLocationSet),int(Abort.FirstOwnerActorRotationSet),
        *Abort.FirstOwnerActorLocationReadback.ToString(),*Abort.FirstOwnerActorRotationReadback.ToString(),*Abort.FirstOwnerCreateWhy));
    TestFalse(TEXT("nondefault first-owner fixture actually aborts before original vehicle construction"),bool(Aborted));
    TestTrue(*FString::Printf(TEXT("actual first-owner forced abort returned no owner after complete probe (stage=%s why=%s)"),
        *Abort.FirstOwnerLastStage,*Abort.FirstOwnerCreateWhy),Abort.FirstOwnerProbeReturned&&Abort.FirstOwnerAbortReached&&Abort.CreateReturnedNull);
    TestEqual(TEXT("first five actual initializers returned"),Abort.FirstOwnerInitializersReturned,uint32(5));
    TestEqual(TEXT("second original initializer loop never entered in this early fixture"),Abort.InitializersReturned,uint32(0));
    TestEqual(TEXT("all five original parents really began null"),Abort.FirstOwnerInitialNullParents,uint32(5));
    TestEqual(TEXT("all nine real pre-owner/COM/pose guards refused"),Abort.FirstOwnerGuardRefusals,uint32(9));
    TestTrue(TEXT("all named target values unchanged at every refusal"),Abort.FirstOwnerAllRefusalsBeforeWrite);
    TestTrue(TEXT("genuine nonzero captured actor pose and COM precedence"),Abort.FirstOwnerPoseAndCOMMatch);
    TestTrue(TEXT("empty live configuration roster still retains original born Chaos configs"),Abort.FirstOwnerClearKeepsConfigurations);
    TestTrue(TEXT("chassis uses rotated actual stored tensor and distinguishes recomputation"),Abort.FirstOwnerStoredChassisTensorMatch&&Abort.FirstOwnerStoredTensorDistinguished);
    TestTrue(TEXT("all five children receive the original origin-derived kinematics"),Abort.FirstOwnerChildrenMatch);
    TestTrue(TEXT("actual wheel velocity matches original wheel FP order and distinguishes base-only shortcut"),Abort.FirstOwnerWheelVelocityOrderDistinguished);
    AddInfo(FString::Printf(TEXT("[PrivateRoadFirstOwnerChildArithmetic] actualWheelOrderDistinguished=%d"),int(Abort.FirstOwnerWheelVelocityOrderDistinguished)));
    TestEqual(TEXT("only genuine preexisting counters retained"),Abort.LifetimeObservers.size(),size_t(4));
    uint64 Constructors=0;
    for(const auto& Trace:Abort.LifetimeObservers)
    {
        if(!Trace)return false;const auto V=Trace->Read();Constructors+=V.NativeConstructionReturns;
        TestEqual(TEXT("every actually constructed simulation returned its native base destructor"),V.NativeBaseDestructionReturns,V.NativeConstructionReturns);
        TestEqual(TEXT("every actual configuration destructor entered"),V.ConfigurationDestructionEntries,uint64(1));
        TestFalse(TEXT("configuration outlived native destructor"),V.ConfigurationDestructionBeforeNativeReturn);
    }
    TestEqual(TEXT("only the first reference simulation was actually constructed before this abort"),Constructors,uint64(1));
    P::FPrivateRoadBirthInitializationProbe Normal;
    auto Own=P::FPrivateRoadPhysicsWorld::CreateWithBirthInitializationProbeForDiagnostics(
        Seeds,*Foreign,P::EPrivateRoadBirthProbeMode::Observe,Normal,Why);
    CheckTypedInitializers(TEXT("normal"),Normal,size_t(12));
    if(!TestTrue(*FString::Printf(TEXT("normal actual original construction: %s"),*Why),bool(Own)))return false;
    TestEqual(TEXT("normal two reference owner entries returned"),Normal.EarlyReferenceOwnerReturns,uint32(2));
    TestEqual(TEXT("normal eight exact two-null entries returned"),Normal.EarlyReferenceTwoNullReturns,uint32(8));
    TestEqual(TEXT("normal ten reference first initializers returned"),Normal.EarlyReferenceInitializers,uint32(10));
    TestEqual(TEXT("normal six invalid early requests refused"),Normal.EarlyReferenceGuardRefusals,uint32(6));
    TestTrue(TEXT("normal invalid early requests unchanged before effects"),Normal.EarlyReferenceRefusalsBeforeWrite);
    TestEqual(TEXT("normal first five per actor actually returned"),Normal.FirstOwnerInitializersReturned,uint32(10));
    TestEqual(TEXT("normal SECOND five per actor still execute after vehicle binding/authored offsets"),Normal.InitializersReturned,uint32(10));
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
    if(!TestTrue(*FString::Printf(TEXT("actual facade-only poison before real Step: %s"),*Why),
        Own->PoisonFacadeMassForValueCommandDiagnostics(Why)))return false;
    P::FPrivateRoadHitAcceptanceProbe Step;
    if(!Own->StepWithHitAcceptanceProbeForDiagnostics(Requests,false,*Foreign,Step,Why)||!Foreign->Step(Requests,false,Why))return false;
    TestTrue(TEXT("real Step refuses the Birth-only WHOLE first owner traversal before writes"),Step.BirthOwnerTraversalRefusedDuringStep);
    const auto X=Own->ReadLatest(),Y=Foreign->ReadLatest();if(!X||!Y||X->Frame!=0||Y->Frame!=0||X->Actors.size()!=2||Y->Actors.size()!=2)return false;
    for(size_t Index=0;Index<2;++Index)
    {
        const auto* A=X->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==X->Actors[Index].NativeStableId;});
        const auto* B=Y->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==Y->Actors[Index].NativeStableId;});
        if(!A||!B)return false;
        const auto& M=X->Actors[Index];const auto& Control=Y->Actors[Index];
        TestTrue(TEXT("actual native Birth value-store observations available"),M.BirthMassValueStoresAvailable&&Control.BirthMassValueStoresAvailable);
        TestEqual(TEXT("five real original body parts observed per actor"),M.BirthMassBodyCount,uint32(5));
        TestEqual(TEXT("five independent control body parts observed"),Control.BirthMassBodyCount,uint32(5));
        TestTrue(TEXT("actual writes returned with exact requested destination bits"),M.BirthMassValueStoresMatch&&Control.BirthMassValueStoresMatch);
        TestTrue(TEXT("each real body stored at least once and at most the two real initializer traversals"),
            M.BirthMassValueReturns>=5&&M.BirthMassValueReturns<=10&&Control.BirthMassValueReturns>=5&&Control.BirthMassValueReturns<=10);
        TestTrue(TEXT("actual facade mass differs while original backing is retained"),M.BirthMassFacadeDistinct);
        AddInfo(FString::Printf(TEXT("[PrivateRoadBirthMassValueStore] actor=%llu frame=%llu bodies=%u writes=%llu available=%d stored=%d facadeDistinct=%d controlBodies=%u controlWrites=%llu controlAvailable=%d controlStored=%d. Actual typed native Birth value stores and one subsequent real independent Step; diagnostic counters and five retained original bodies per actor only. Four Mass overrides and other reference/lifetime paths remain open."),
            M.Actor,X->Frame,M.BirthMassBodyCount,M.BirthMassValueReturns,int(M.BirthMassValueStoresAvailable),int(M.BirthMassValueStoresMatch),
            int(M.BirthMassFacadeDistinct),Control.BirthMassBodyCount,Control.BirthMassValueReturns,int(Control.BirthMassValueStoresAvailable),int(Control.BirthMassValueStoresMatch)));
        const auto& K=A->COMState;const auto& L=B->COMState;
        TestTrue(TEXT("independent canonical six-field COM parity after normal complete constructor"),K.Location==L.Location&&K.Velocity==L.Velocity
            &&K.Acceleration==L.Acceleration&&K.Rotation==L.Rotation&&K.AngularVelocity==L.AngularVelocity&&K.AngularAcceleration==L.AngularAcceleration);
    }
    AddInfo(FString::Printf(TEXT("[PrivateRoadEarlyReferenceOwnerNormal] ownerReturns=%u twoNullReturns=%u initializers=%u boundaryChecks=%u refusals=%u beforeWrite=%d. Existing default/custom owner arithmetic and callback order shared; ordinary binder family, parent SetOwner, initializer/fourMass/full state/K/playable remain OPEN."),
        Normal.EarlyReferenceOwnerReturns,Normal.EarlyReferenceTwoNullReturns,Normal.EarlyReferenceInitializers,
        Normal.EarlyReferenceBoundaryChecks,Normal.EarlyReferenceGuardRefusals,int(Normal.EarlyReferenceRefusalsBeforeWrite)));
    TestTrue(TEXT("both real owners close"),Own->Close()&&Foreign->Close());
    AddInfo(FString::Printf(TEXT("[PrivateRoadFirstOwnerBirthTraversal] firstNormal=%u secondNormal=%u firstAbort=%u secondAbort=%u nullParents=%u birthRefusals=%u beforeWrite=%d nonzeroPoseCOM=%d clearKeepsConfig=%d storedChassisTensor=%d distinguishesRecompute=%d children=%d earlyAbortCtorReturns=%llu actualStepRefusesBirth=%d. Actual allocated first-owner fixtures and normal/aborting lifetime returns only; no full causal state/source/lifetime lease or connected restoration qualification."),
        Normal.FirstOwnerInitializersReturned,Normal.InitializersReturned,Abort.FirstOwnerInitializersReturned,Abort.InitializersReturned,Abort.FirstOwnerInitialNullParents,
        Abort.FirstOwnerGuardRefusals,int(Abort.FirstOwnerAllRefusalsBeforeWrite),int(Abort.FirstOwnerPoseAndCOMMatch),int(Abort.FirstOwnerClearKeepsConfigurations),
        int(Abort.FirstOwnerStoredChassisTensorMatch),int(Abort.FirstOwnerStoredTensorDistinguished),int(Abort.FirstOwnerChildrenMatch),Constructors,int(Step.BirthOwnerTraversalRefusedDuringStep)));
    return !HasAnyErrors();
}
#endif
