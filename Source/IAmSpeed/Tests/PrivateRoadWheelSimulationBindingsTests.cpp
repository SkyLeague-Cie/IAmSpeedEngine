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
    TestTrue(TEXT("direct Retirement Kind rejects genuine Birth with other root/gate predicates positive"),Birth.RetirementBirthOtherPredicatesPositive&&Birth.RetirementKindRefusedInBirth);
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
    P::FPrivateRoadOrdinaryBindingNoOpProbe NoOp;
    const bool NoOpReturned=Own->ProbeOrdinaryBindingsWithoutOperationForDiagnostics(*Foreign,NoOp,Why);
    // This caller is outside all actual retained roots/gates/GC guards. Native
    // diagnostics never invoke test logging/assertion callbacks under their locks.
    for(const auto& Row:NoOp.Cases)
    {
        AddInfo(FString::Printf(TEXT("[PrivateRoadOrdinaryBindingNoOpCase] entry=%s argument=%s attempted=%d refused=%d noActualOperation=%d liveStorageRoutes=%d ownOriginal=%d ownReference=%d ownFacade=%d foreignOriginal=%d foreignReference=%d foreignFacade=%d ownCars=%u foreignCars=%u originalVehicles=%u referenceVehicles=%u originalWheelSims=%u referenceWheelSims=%u configurations=%u referenceConfigurations=%u holderObserved=%d holderWasNonNull=%d holderUnchanged=%d. Actual retained owners and per-entry declared binding/setup footprints only; no snapshot restoration, operation permission or complete lease."),
            *Row.Entry,*Row.Argument,int(Row.Attempted),int(Row.Refused),int(Row.NoActualOperation),int(Row.LiveStorageRoutes),
            int(Row.OwnOriginalUnchanged),int(Row.OwnReferenceUnchanged),int(Row.OwnFacadeUnchanged),int(Row.ForeignOriginalUnchanged),int(Row.ForeignReferenceUnchanged),int(Row.ForeignFacadeUnchanged),
            Row.OwnCars,Row.ForeignCars,Row.OriginalVehicles,Row.ReferenceVehicles,Row.OriginalWheelSims,Row.ReferenceWheelSims,Row.Configurations,Row.ReferenceConfigurations,
            int(Row.ArgumentHolderObserved),int(Row.ArgumentHolderWasNonNull),int(Row.ArgumentHolderUnchanged)));
        TestTrue(TEXT("actual ordinary refusal precedes declared own/reference/facade/foreign effects"),Row.Attempted&&Row.Refused&&Row.NoActualOperation&&Row.LiveStorageRoutes
            &&Row.OwnOriginalUnchanged&&Row.OwnReferenceUnchanged&&Row.OwnFacadeUnchanged&&Row.ForeignOriginalUnchanged&&Row.ForeignReferenceUnchanged&&Row.ForeignFacadeUnchanged);
        TestEqual(TEXT("actual own car population"),Row.OwnCars,uint32(2));TestEqual(TEXT("actual foreign car population"),Row.ForeignCars,uint32(2));
        TestEqual(TEXT("actual original vehicle population"),Row.OriginalVehicles,uint32(4));TestEqual(TEXT("actual reference vehicle population"),Row.ReferenceVehicles,uint32(4));
        TestEqual(TEXT("actual original wheel population"),Row.OriginalWheelSims,uint32(16));TestEqual(TEXT("actual reference wheel population"),Row.ReferenceWheelSims,uint32(16));
        TestEqual(TEXT("actual original configurations"),Row.Configurations,uint32(16));TestEqual(TEXT("actual reference configurations"),Row.ReferenceConfigurations,uint32(16));
        if(Row.Entry==TEXT("IAS-setup"))TestTrue(TEXT("actual Setup unique holder identity/Get remains unchanged"),Row.ArgumentHolderObserved&&Row.ArgumentHolderUnchanged
            &&Row.ArgumentHolderWasNonNull==(Row.Argument!=TEXT("null")));
        else TestFalse(TEXT("no invented Setup holder observation for other entries"),Row.ArgumentHolderObserved);
    }
    TestEqual(TEXT("all seventeen real ordinary entry attempts observed"),NoOp.Cases.size(),size_t(17));
    if(!TestTrue(*FString::Printf(TEXT("actual NoOp entry diagnostic returned: %s"),*Why),NoOpReturned&&NoOp.Returned))return false;
    P::FPrivateRoadOrdinaryBindingNoOpProbe InitializerNoOp;
    const bool InitializerNoOpReturned=Own->ProbeOrdinaryInitializersWithoutOperationForDiagnostics(*Foreign,InitializerNoOp,Why);
    // This caller is outside all actual retained roots/gates/GC guards. Native
    // diagnostics never invoke test logging/assertion callbacks under their locks.
    for(const auto& Row:InitializerNoOp.Cases)
    {
        AddInfo(FString::Printf(TEXT("[PrivateRoadOrdinaryInitializerNoOpCase] entry=%s argument=%s attempted=%d refused=%d noActualOperation=%d liveStorageRoutes=%d ownOriginal=%d ownReference=%d ownFacade=%d foreignOriginal=%d foreignReference=%d foreignFacade=%d ownCars=%u foreignCars=%u originalVehicles=%u referenceVehicles=%u originalWheelSims=%u referenceWheelSims=%u configurations=%u referenceConfigurations=%u holderObserved=%d holderWasNonNull=%d holderUnchanged=%d originalBodies=%u referenceBodies=%u hitboxes=%u wheelBodies=%u. Actual retained owners and per-entry declared initializer/binding footprints only; no snapshot restoration, operation permission or complete lease."),
            *Row.Entry,*Row.Argument,int(Row.Attempted),int(Row.Refused),int(Row.NoActualOperation),int(Row.LiveStorageRoutes),
            int(Row.OwnOriginalUnchanged),int(Row.OwnReferenceUnchanged),int(Row.OwnFacadeUnchanged),int(Row.ForeignOriginalUnchanged),int(Row.ForeignReferenceUnchanged),int(Row.ForeignFacadeUnchanged),
            Row.OwnCars,Row.ForeignCars,Row.OriginalVehicles,Row.ReferenceVehicles,Row.OriginalWheelSims,Row.ReferenceWheelSims,Row.Configurations,Row.ReferenceConfigurations,
            int(Row.ArgumentHolderObserved),int(Row.ArgumentHolderWasNonNull),int(Row.ArgumentHolderUnchanged),Row.OriginalBodies,Row.ReferenceBodies,Row.Hitboxes,Row.WheelBodies));
        TestTrue(TEXT("actual ordinary refusal precedes declared own/reference/facade/foreign effects"),Row.Attempted&&Row.Refused&&Row.NoActualOperation&&Row.LiveStorageRoutes
            &&Row.OwnOriginalUnchanged&&Row.OwnReferenceUnchanged&&Row.OwnFacadeUnchanged&&Row.ForeignOriginalUnchanged&&Row.ForeignReferenceUnchanged&&Row.ForeignFacadeUnchanged);
        TestEqual(TEXT("actual original bodies captured"),Row.OriginalBodies,uint32(20));TestEqual(TEXT("actual reference bodies captured"),Row.ReferenceBodies,uint32(20));
        TestEqual(TEXT("actual hitboxes captured"),Row.Hitboxes,uint32(4));TestEqual(TEXT("actual wheel bodies captured"),Row.WheelBodies,uint32(16));
        TestEqual(TEXT("actual own car population"),Row.OwnCars,uint32(2));TestEqual(TEXT("actual foreign car population"),Row.ForeignCars,uint32(2));
        TestEqual(TEXT("actual original vehicle population"),Row.OriginalVehicles,uint32(4));TestEqual(TEXT("actual reference vehicle population"),Row.ReferenceVehicles,uint32(4));
        TestEqual(TEXT("actual original wheel population"),Row.OriginalWheelSims,uint32(16));TestEqual(TEXT("actual reference wheel population"),Row.ReferenceWheelSims,uint32(16));
        TestEqual(TEXT("actual original configurations"),Row.Configurations,uint32(16));TestEqual(TEXT("actual reference configurations"),Row.ReferenceConfigurations,uint32(16));
        if(Row.Entry==TEXT("IAS-setup"))TestTrue(TEXT("actual Setup unique holder identity/Get remains unchanged"),Row.ArgumentHolderObserved&&Row.ArgumentHolderUnchanged
            &&Row.ArgumentHolderWasNonNull==(Row.Argument!=TEXT("null")));
        else TestFalse(TEXT("no invented Setup holder observation for other entries"),Row.ArgumentHolderObserved);
    }
    TestEqual(TEXT("all twenty-seven real ordinary initializer attempts observed"),InitializerNoOp.Cases.size(),size_t(27));
    if(!TestTrue(*FString::Printf(TEXT("actual InitializerNoOp entry diagnostic returned: %s"),*Why),InitializerNoOpReturned&&InitializerNoOp.Returned))return false;
    P::FPrivateRoadOrdinaryBindingNoOpProbe ParentInitializerNoOp;
    const bool ParentInitializerNoOpReturned=Own->ProbeOrdinaryParentInitializationWithoutOperationForDiagnostics(*Foreign,ParentInitializerNoOp,Why);
    // This caller is outside all actual retained roots/gates/GC guards. Native
    // diagnostics never invoke test logging/assertion callbacks under their locks.
    for(const auto& Row:ParentInitializerNoOp.Cases)
    {
        AddInfo(FString::Printf(TEXT("[PrivateRoadOrdinaryParentInitializerNoOpCase] entry=%s argument=%s attempted=%d refused=%d noActualOperation=%d liveStorageRoutes=%d ownOriginal=%d ownReference=%d ownFacade=%d foreignOriginal=%d foreignReference=%d foreignFacade=%d ownCars=%u foreignCars=%u originalVehicles=%u referenceVehicles=%u originalWheelSims=%u referenceWheelSims=%u configurations=%u referenceConfigurations=%u holderObserved=%d holderWasNonNull=%d holderUnchanged=%d originalBodies=%u referenceBodies=%u hitboxes=%u wheelBodies=%u. Actual retained owners and per-entry declared parent/component/pending-list/membership and initializer/binding footprints only; UE-private async scheduling UNCAPTURED, registry PARTIAL; no snapshot restoration, operation permission or complete lease."),
            *Row.Entry,*Row.Argument,int(Row.Attempted),int(Row.Refused),int(Row.NoActualOperation),int(Row.LiveStorageRoutes),
            int(Row.OwnOriginalUnchanged),int(Row.OwnReferenceUnchanged),int(Row.OwnFacadeUnchanged),int(Row.ForeignOriginalUnchanged),int(Row.ForeignReferenceUnchanged),int(Row.ForeignFacadeUnchanged),
            Row.OwnCars,Row.ForeignCars,Row.OriginalVehicles,Row.ReferenceVehicles,Row.OriginalWheelSims,Row.ReferenceWheelSims,Row.Configurations,Row.ReferenceConfigurations,
            int(Row.ArgumentHolderObserved),int(Row.ArgumentHolderWasNonNull),int(Row.ArgumentHolderUnchanged),Row.OriginalBodies,Row.ReferenceBodies,Row.Hitboxes,Row.WheelBodies));
        TestTrue(TEXT("actual ordinary refusal precedes declared own/reference/facade/foreign effects"),Row.Attempted&&Row.Refused&&Row.NoActualOperation&&Row.LiveStorageRoutes
            &&Row.OwnOriginalUnchanged&&Row.OwnReferenceUnchanged&&Row.OwnFacadeUnchanged&&Row.ForeignOriginalUnchanged&&Row.ForeignReferenceUnchanged&&Row.ForeignFacadeUnchanged);
        TestEqual(TEXT("actual original bodies captured"),Row.OriginalBodies,uint32(20));TestEqual(TEXT("actual reference bodies captured"),Row.ReferenceBodies,uint32(20));
        TestEqual(TEXT("actual hitboxes captured"),Row.Hitboxes,uint32(4));TestEqual(TEXT("actual wheel bodies captured"),Row.WheelBodies,uint32(16));
        TestEqual(TEXT("actual own car population"),Row.OwnCars,uint32(2));TestEqual(TEXT("actual foreign car population"),Row.ForeignCars,uint32(2));
        TestEqual(TEXT("actual original vehicle population"),Row.OriginalVehicles,uint32(4));TestEqual(TEXT("actual reference vehicle population"),Row.ReferenceVehicles,uint32(4));
        TestEqual(TEXT("actual original wheel population"),Row.OriginalWheelSims,uint32(16));TestEqual(TEXT("actual reference wheel population"),Row.ReferenceWheelSims,uint32(16));
        TestEqual(TEXT("actual original configurations"),Row.Configurations,uint32(16));TestEqual(TEXT("actual reference configurations"),Row.ReferenceConfigurations,uint32(16));
        if(Row.Entry==TEXT("IAS-setup"))TestTrue(TEXT("actual Setup unique holder identity/Get remains unchanged"),Row.ArgumentHolderObserved&&Row.ArgumentHolderUnchanged
            &&Row.ArgumentHolderWasNonNull==(Row.Argument!=TEXT("null")));
        else TestFalse(TEXT("no invented Setup holder observation for other entries"),Row.ArgumentHolderObserved);
    }
    TestEqual(TEXT("all seven real ordinary initializer attempts observed"),ParentInitializerNoOp.Cases.size(),size_t(7));
    if(!TestTrue(*FString::Printf(TEXT("actual ParentInitializerNoOp entry diagnostic returned: %s"),*Why),ParentInitializerNoOpReturned&&ParentInitializerNoOp.Returned))return false;
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
    TestTrue(TEXT("direct Retirement Kind rejects genuine Step with other root/gate predicates positive"),Step.RetirementStepOtherPredicatesPositive&&Step.RetirementKindRefusedInStep);
    AddInfo(FString::Printf(TEXT("[PrivateRoadRetirementKindIsolation] birthOtherPredicates=%d birthRefused=%d stepOtherPredicates=%d stepRefused=%d. Direct validator; no StorageAccess reset or leaf confound."),
        int(Birth.RetirementBirthOtherPredicatesPositive),int(Birth.RetirementKindRefusedInBirth),int(Step.RetirementStepOtherPredicatesPositive),int(Step.RetirementKindRefusedInStep)));
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
    AddInfo(FString::Printf(TEXT("[PrivateRoadReferenceWheelBirth] factoryReturns=%u positiveReturns=%u birthRefusals=%u beforeWrite=%d referenceValuesOriginalUnchanged=%d overridePredicates=%d originalCompared=%u originalKeepsReference=%d actualStepRefusesReferenceBody=%d. Only actual reference Birth and named fixture effects; ordinary family controls are scoped separately; arbitrary public targets, parent SetOwner, independent initializer, retirement population closure, fourMass, source/lifetime/state/K remain OPEN."),
        Birth.ReferenceBindingFactoryReturns,Birth.ReferenceBindingPositiveReturns,Birth.ReferenceBindingGuardRefusals,
        int(Birth.ReferenceBindingBeforeWrite),int(Birth.ReferenceBindingValuesMatch),int(Birth.ReferenceBindingOverridePredicatesMatch),Birth.OriginalBindingReferenceComparisons,int(Birth.OriginalBindingKeepsReference),int(Step.ReferenceBindingBodyRefusedDuringStep)));
    return !HasAnyErrors();
}
#endif
