#if WITH_DEV_AUTOMATION_TESTS && !UE_BUILD_SHIPPING
#include "IAmSpeed/Netcode/client-prediction/PrivateRoadPhysicsWorld.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "Misc/AutomationTest.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIASPrivateRoadWheelSimulationBindings,
    "IAmSpeed.Netcode.Prediction.PrivateRoadWheelSimulationBindings",
    EAutomationTestFlags_ApplicationContextMask|EAutomationTestFlags::EngineFilter)
bool FIASPrivateRoadWheelSimulationBindings::RunTest(const FString&)
{
    namespace P=Speed::Netcode::Prediction;using namespace Speed::Netcode;
    if(!TestTrue(TEXT("real Birth binding controls run on GT"),IsInGameThread()))return false;
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
    P::FPrivateRoadBirthInitializationProbe Birth;
    auto Own=P::FPrivateRoadPhysicsWorld::CreateWithBirthInitializationProbeForDiagnostics(
        Seeds,*Foreign,P::EPrivateRoadBirthProbeMode::Observe,Birth,Why);
    if(!TestTrue(*FString::Printf(TEXT("actual native factory reaches binding route: %s"),*Why),bool(Own)))return false;
    TestEqual(TEXT("both actual factory binding entries returned"),Birth.WheelBindingFactoryReturns,uint32(2));
    TestEqual(TEXT("one positive nondefault original binding entry returned"),Birth.WheelBindingPositiveEntryReturns,uint32(1));
    TestEqual(TEXT("all eight actual Birth foreign/late invalid guards refused"),Birth.WheelBindingGuardRefusals,uint32(8));
    TestTrue(TEXT("real native Binding probe returned"),Birth.WheelBindingProbeReturned);
    TestTrue(TEXT("all named actual target/config/simulator values unchanged by refusals"),Birth.WheelBindingAllRefusalsBeforeWrite);
    TestTrue(TEXT("positive real mass/index/pointer values match"),Birth.WheelBindingPositiveValuesMatch);
    TestTrue(TEXT("both Raise/Drop override predicates retained"),Birth.WheelBindingOverridePredicatesMatch);
    TestEqual(TEXT("two actual reference factory entries returned"),Birth.ReferenceBindingFactoryReturns,uint32(2));
    TestEqual(TEXT("sixteen actual reference foreign/mixed/late guards refused"),Birth.ReferenceBindingGuardRefusals,uint32(16));
    TestEqual(TEXT("one actual nondefault reference entry returned"),Birth.ReferenceBindingPositiveReturns,uint32(1));
    TestTrue(TEXT("actual reference binding probe returned"),Birth.ReferenceBindingProbeReturned);
    TestTrue(TEXT("reference guards refuse before index/pointer/config/target mass writes"),Birth.ReferenceBindingBeforeWrite);
    TestTrue(TEXT("reference values match and actual original parts/simulator remain unchanged"),Birth.ReferenceBindingValuesMatch);
    TestTrue(TEXT("reference both Raise/Drop override predicates retained"),Birth.ReferenceBindingOverridePredicatesMatch);
    TestEqual(TEXT("two actual original bindings compared around their real calls"),Birth.OriginalBindingReferenceComparisons,uint32(2));
    TestTrue(TEXT("both actual original bindings keep reference values unchanged"),Birth.OriginalBindingKeepsReference);
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
    P::FPrivateRoadHitAcceptanceProbe Step;
    if(!TestTrue(*FString::Printf(TEXT("real Step phase entry: %s"),*Why),Own->StepWithHitAcceptanceProbeForDiagnostics(Requests,false,*Foreign,Step,Why)))return false;
    TestTrue(TEXT("actual Step refuses Birth-only binding before target writes"),Step.BirthWheelBindingRefusedDuringStep);
    TestTrue(TEXT("actual Step refuses reference body Birth preflight before writes"),Step.ReferenceBindingBodyRefusedDuringStep);
    TestTrue(TEXT("actual Step refuses Retirement clear before named original/reference effects"),Step.RetirementClearRefusedInStep&&Step.RetirementStepFootprintUnchanged);
    AddInfo(FString::Printf(TEXT("[PrivateRoadRetirementStepBoundary] refused=%d beforeWrite=%d"),int(Step.RetirementClearRefusedInStep),int(Step.RetirementStepFootprintUnchanged)));
    if(!TestTrue(TEXT("independent reference still advances"),Foreign->Step(Requests,false,Why)))return false;
    const auto A=Own->ReadLatest(),B=Foreign->ReadLatest();
    if(!TestTrue(TEXT("real frame-zero publications"),A&&B&&A->Frame==0&&B->Frame==0&&A->Actors.size()==2&&B->Actors.size()==2))return false;
    for(size_t Index=0;Index<2;++Index)
    {
        const auto* X=A->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==A->Actors[Index].NativeStableId;});
        const auto* Y=B->Physics.PresentationBodies.FindByPredicate([&](const auto& V){return V.StableId==B->Actors[Index].NativeStableId;});
        if(!TestTrue(TEXT("actual native presentation binding present"),X&&Y))return false;
        const auto& K=X->COMState;const auto& L=Y->COMState;
        TestTrue(TEXT("fixture cleanup preserves independent same-frame COM parity"),K.Location==L.Location&&K.Velocity==L.Velocity&&K.Acceleration==L.Acceleration
            &&K.Rotation==L.Rotation&&K.AngularVelocity==L.AngularVelocity&&K.AngularAcceleration==L.AngularAcceleration);
    }
    TestTrue(TEXT("both real original owners close"),Own->Close()&&Foreign->Close());
    AddInfo(FString::Printf(TEXT("[PrivateRoadWheelSimulationBindings] factoryReturns=%u positiveReturns=%u birthRefusals=%u beforeWrite=%d positiveValues=%d overridePredicates=%d actualStepRefusesBirth=%d. Real allocated original binding fixtures and named cleanup only; no full-state/source/lifetime restoration or connected client/server qualification."),
        Birth.WheelBindingFactoryReturns,Birth.WheelBindingPositiveEntryReturns,Birth.WheelBindingGuardRefusals,
        int(Birth.WheelBindingAllRefusalsBeforeWrite),int(Birth.WheelBindingPositiveValuesMatch),int(Birth.WheelBindingOverridePredicatesMatch),int(Step.BirthWheelBindingRefusedDuringStep)));
    AddInfo(FString::Printf(TEXT("[PrivateRoadReferenceWheelBirth] factoryReturns=%u positiveReturns=%u birthRefusals=%u beforeWrite=%d referenceValuesOriginalUnchanged=%d overridePredicates=%d originalCompared=%u originalKeepsReference=%d actualStepRefusesReferenceBody=%d. Only actual reference Birth and named fixture effects; ordinary ambient binders, arbitrary public targets, retirement population closure, fourMass, source/lifetime/state/K remain OPEN."),
        Birth.ReferenceBindingFactoryReturns,Birth.ReferenceBindingPositiveReturns,Birth.ReferenceBindingGuardRefusals,
        int(Birth.ReferenceBindingBeforeWrite),int(Birth.ReferenceBindingValuesMatch),int(Birth.ReferenceBindingOverridePredicatesMatch),Birth.OriginalBindingReferenceComparisons,int(Birth.OriginalBindingKeepsReference),int(Step.ReferenceBindingBodyRefusedDuringStep)));
    return !HasAnyErrors();
}
#endif
