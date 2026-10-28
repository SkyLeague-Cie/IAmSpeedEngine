#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Driving/DrivingInput.h"
#include "IAmSpeed/Driving/StockRoadCar.h"
#include "IAmSpeed/Input/Testing/TestInputProducerV2.h"
#include "IAmSpeed/World/Simulation/CanonicalFrameContext.h"
#include "Misc/AutomationTest.h"
#include "UObject/StrongObjectPtr.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedStockDrivingComponentTargets,
    "IAmSpeed.Netcode.DrivingActions.NativeComponentTargets",
    EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedStockDrivingComponentTargets::RunTest(const FString&)
{
    namespace V2 = Speed::Input::V2;
    using Speed::Input::FActionId;
    // Exact canonical frames, after mapping. Portable DrivingActionsTests covers
    // the stock raw bindings. This fixture uses the actual UObject component's
    // public input preparation/validation/commit path, without friends or setters
    // for private state. It does NOT step forces, contacts or a network world.
    if (!TestTrue(TEXT("UObject input fixture runs on game thread"),IsInGameThread())) return false;
    const auto Contract=Speed::Driving::MakeContract(Speed::Driving::FBindings{});
    if (!TestTrue(TEXT("actual stock seven-action contract"),Contract && Contract->GetDescription().Actions.size()==7)) return false;
    struct FCase { FActionId Action; int16 Value; };
    const FCase Cases[]={{0,255},{1,255},{2,-127},{2,127},{3,1},{4,1},{5,-127},{5,127},{6,-127},{6,127}};
    for (const FCase& Case:Cases)
    {
        std::vector<V2::FInputFrame> Frames;
        for (uint64 N=0;N<5;++N)
        {
            V2::FInputFrameData D;
            D.ConsumptionFrame=N;D.SourceSequence=N+1;
            D.StreamEpoch={7001};D.DeviceGeneration={1};
            D.Producer={Speed::Input::EProducerKind::Device,7002};D.Reset=N==0 || N==4;
            const bool Held=N==1 || N==2 || N==4;
            if (Held) { D.Values[Case.Action]=Case.Value;D.ActiveMask=uint32(1)<<Case.Action; }
            if (N==1 || N==3) D.Transitions.push_back({Case.Action,N==1?V2::ETransition::Started:V2::ETransition::Completed,
                int16(N==1?Case.Value:0),{N+1,0}});
            Frames.emplace_back(Contract,std::move(D));
        }
        auto Producer=V2::FTestInputProducer::Create(Contract,{7001},{Speed::Input::EProducerKind::Device,7002},0,Frames);
        if (!TestTrue(TEXT("sealed actual V2 producer"),bool(Producer))) return false;
        auto Stream=std::make_shared<V2::FInputStream>(std::move(Producer),Contract,V2::FStreamEpoch{7001},
            Speed::Input::FProducerIdentity{Speed::Input::EProducerKind::Device,7002});
        TStrongObjectPtr<USpeedWheeledComponent> Component(NewObject<USpeedWheeledComponent>());
        if (!TestTrue(TEXT("same accepted stock road profile as ASpeedCar construction"),Speed::Driving::ApplyStockRoadCarToGenericMovement(*Component))) return false;
        if (!TestTrue(TEXT("native component attaches V2 authority"),Component->SetFrameInputStreamV2(Stream))
            || !TestTrue(TEXT("actual stream activates"),Stream->Activate())) return false;
        for (uint64 N=0;N<5;++N)
        {
            const FString Label=FString::Printf(TEXT("action=%u value=%d frame=%llu"),unsigned(Case.Action),int(Case.Value),N);
            if (!TestTrue(Label+TEXT(" actual component prepares inputs"),Component->PrepareCanonicalInputs(FCanonicalFrameContext(N)))) return false;
            const bool Held=N==1 || N==2 || N==4;
            TestEqual(Label+TEXT(" throttle"),Component->GetPhysThrottleInput(),Held && Case.Action==0?1.f:0.f);
            TestEqual(Label+TEXT(" brake"),Component->GetPhysBrakeInput(),Held && Case.Action==1?1.f:0.f);
            TestEqual(Label+TEXT(" steering"),Component->GetPhysSteeringInput(),Held && Case.Action==2?(Case.Value<0?-1.f:1.f):0.f);
            const auto Before=Stream->ReadLatest();
            TestTrue(Label+TEXT(" preparation does not publish"),N==0?!Before:Before && Before->Frame.GetData().ConsumptionFrame==N-1);
            if (!TestTrue(Label+TEXT(" actual component validates commit"),Component->ValidateCanonicalFrameCommit(N))
                || !TestTrue(Label+TEXT(" actual component commits input"),Component->CommitCanonicalFrame(N))) return false;
            const auto Completed=Stream->ReadLatest();
            TestTrue(Label+TEXT(" same frame published after component commit"),Completed && Completed->Frame.GetData().ConsumptionFrame==N);
        }
        // Missing input is a real source exhaustion, not an invented neutral
        // packet. The component must reject the frame and remove held axes.
        TestFalse(TEXT("exhausted source rejects preparation"),Component->PrepareCanonicalInputs(FCanonicalFrameContext(5)));
        TestEqual(TEXT("exhaustion neutralizes throttle"),Component->GetPhysThrottleInput(),0.f);
        TestEqual(TEXT("exhaustion neutralizes brake"),Component->GetPhysBrakeInput(),0.f);
        TestEqual(TEXT("exhaustion neutralizes steering"),Component->GetPhysSteeringInput(),0.f);
        TestFalse(TEXT("rejected frame cannot commit"),Component->ValidateCanonicalFrameCommit(5));
        Component->AbortCanonicalFrame(5,ECanonicalFrameAbortReason::PreparationFailed);
        TestTrue(TEXT("explicit component detach after abort"),Component->SetFrameInputStreamV2(nullptr));
    }
    AddInfo(TEXT("Native component INPUT targets only; no world step, force, surface, controller pause/reset, socket, prediction, restore or match qualification."));
    return !HasAnyErrors();
}
#endif
