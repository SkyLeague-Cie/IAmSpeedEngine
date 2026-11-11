#include "CoreMinimal.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/IConsoleManager.h"
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "IAmSpeed/World/Simulation/SimulationWorker.h"
#include <thread>
#include <atomic>

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFreshScenarioSeedInspectionTest,
    "IAmSpeed.Input.NativePreparation.FreshSeedIsReadOnly",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FFreshScenarioSeedInspectionTest::RunTest(const FString&)
{
    using namespace Speed::Input::V2;
    constexpr auto throttle=Speed::Input::Throttle;
    FInputActionContractDescription description;
    description.Revision={1};description.Actions=FInputActionContract::BaseActions();
    description.Physical={{throttle,EPhysicalDestination::Throttle},
        {Speed::Input::Brake,EPhysicalDestination::Brake},{Speed::Input::Steering,EPhysicalDestination::Steering}};
    const auto contract=FInputActionContract::Create(description);
    if(!TestTrue(TEXT("portable action contract"),bool(contract)))return false;
    FInputFrameData seed;
    seed.SourceSequence=1;seed.ConsumptionFrame=0;seed.Producer={Speed::Input::EProducerKind::Device,1};
    seed.StreamEpoch={1};seed.DeviceGeneration={1};seed.Reset=true;
    seed.Values[throttle]=37;seed.ActiveMask=std::uint32_t{1}<<throttle;
    FSessionDescriptor binding;
    binding.Id=binding.Epoch=binding.Controller=binding.Producer=1;
    binding.Contract=contract;binding.Kind=EProducerContract::ExactScenario;
    binding.AllowScenarioAppend=true;binding.Actors={{11,1}};
    binding.Scenario.emplace_back(contract,seed);
    auto commands=std::make_shared<FInputSessionCommands>();
    FInputSessionRegistry registry(commands,1);
    FBoundaryCommandDescriptor command;command.Id=command.WorkerGeneration=1;command.Binding=binding;
    if(!TestTrue(TEXT("real exact producer bound"),commands->Submit(command)==ECommandAdmission::Enqueued&&registry.ServiceBoundary()))return false;
    const auto expected=registry.ReadRegistry(),copied=registry.ReadRegistry();
    if(!TestTrue(TEXT("independent published copies"),expected&&copied&&expected!=copied))return false;
    const auto first=registry.InspectFreshExactScenarioSeed(0,*expected);
    const auto again=registry.InspectFreshExactScenarioSeed(0,*copied);
    if(!TestTrue(TEXT("repeat inspection accepts complete copied values"),first&&again))return false;
    const auto& observed=first->Seed.GetData();
    TestTrue(TEXT("actual complete seed retained"),observed.Version==seed.Version&&
        observed.SourceSequence==seed.SourceSequence&&observed.ConsumptionFrame==seed.ConsumptionFrame&&
        observed.Producer.Kind==seed.Producer.Kind&&observed.Producer.Id==seed.Producer.Id&&
        observed.StreamEpoch.Value==seed.StreamEpoch.Value&&observed.DeviceGeneration.Value==seed.DeviceGeneration.Value&&
        observed.Values==seed.Values&&observed.ActiveMask==seed.ActiveMask&&observed.Reset&&observed.Transitions.empty());
    TestTrue(TEXT("binding fingerprint stable"),first->BindingFingerprint==again->BindingFingerprint);
    TestTrue(TEXT("inspection does not poll or install"),registry.PollCount(1)==0&&
        !registry.ReadInstalled()&&!registry.ReadLatest()&&!registry.ReadInitial());
    auto bad=*expected;auto changed=seed;changed.Reset=false;
    bad.Bindings[0].Scenario[0]=FInputFrame(contract,changed);
    TestFalse(TEXT("changed reset refused"),bool(registry.InspectFreshExactScenarioSeed(0,bad)));
    bad=*expected;changed=seed;++changed.Values[throttle];
    bad.Bindings[0].Scenario[0]=FInputFrame(contract,changed);
    TestFalse(TEXT("changed value refused"),bool(registry.InspectFreshExactScenarioSeed(0,bad)));
    bad=*expected;bad.Phases[0].Phase=ESessionPhase::Paused;
    TestFalse(TEXT("changed phase refused"),bool(registry.InspectFreshExactScenarioSeed(0,bad)));
    bad=*expected;++bad.Bindings[0].Actors[0].Generation;
    TestFalse(TEXT("changed actor generation refused"),bool(registry.InspectFreshExactScenarioSeed(0,bad)));
    bool foreignAccepted=true;
    std::thread foreign([&]{foreignAccepted=bool(registry.InspectFreshExactScenarioSeed(0,*expected));});foreign.join();
    TestFalse(TEXT("foreign thread refused"),foreignAccepted);
    TestTrue(TEXT("refusals preserve the unconsumed seed"),bool(registry.InspectFreshExactScenarioSeed(0,*expected)));
    if(!TestTrue(TEXT("real portable poll still succeeds"),registry.PrepareFrame(0)))return false;
    TestFalse(TEXT("inspection refuses after poll"),bool(registry.InspectFreshExactScenarioSeed(0,*expected)));
    if(!TestTrue(TEXT("real portable install still succeeds"),registry.InstallAll()))return false;
    const auto installed=registry.ReadInstalled();
    if(!TestTrue(TEXT("actual installed snapshot"),installed&&installed->Inputs.size()==1&&installed->Inputs[0].Snapshot))return false;
    const auto snapshot=installed->Inputs[0].Snapshot;
    TestTrue(TEXT("inspection left the seed and first poll intact"),registry.PollCount(1)==1&&
        snapshot->Input.GetData().Reset&&snapshot->Input.GetData().Transitions.empty()&&snapshot->Requested==seed.Values&&
        snapshot->Applied==seed.Values&&snapshot->CutoffSequence==seed.SourceSequence&&
        snapshot->BindingFingerprint==first->BindingFingerprint);
    // This completes ONLY the portable input token protocol. No World, actor,
    // preparation callback or physical simulation is present in this fixture.
    TestTrue(TEXT("portable input protocol completes"),registry.BeginAll()&&registry.ValidateComplete()&&registry.CompleteAll());
    TestFalse(TEXT("completed frame is no longer fresh"),bool(registry.InspectFreshExactScenarioSeed(0,*expected)));
    TestTrue(TEXT("owner lane ledger closes"),registry.CloseWorkerLedger());
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FUnboundPreparationResultTest,
    "IAmSpeed.Simulation.NativePreparation.UnboundPrefixResultIsRefused",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FUnboundPreparationResultTest::RunTest(const FString&)
{
    std::atomic<int32> calls{0},closes{0};
    FSimulationWorker worker([&]{++calls;return ESimulationWorkerResult::PreparationPrefixStopped;},
        [](FSimulationWorkerWaitContext&){},FSimulationWorker::FServiceBoundary{},[&]{++closes;});
    if(!TestTrue(TEXT("real isolated worker starts paused"),worker.Start(true)))return false;
    if(!TestTrue(TEXT("real service suspension acknowledged"),worker.TrySuspendBoundaryService(1000)))return false;
    const auto result=worker.RunOnePausedWorkForTesting(1000);
    TestTrue(TEXT("the real work callback returned the prefix enum"),result.WorkCalled&&result.WorkReturned&&
        result.WorkResult==uint8(ESimulationWorkerResult::PreparationPrefixStopped));
    TestFalse(TEXT("an enum without native Host/window/source authority is refused"),result.GuardsAccepted);
    TestTrue(TEXT("refused result closes and joins instead of creating a park ACK"),
        result.Quiescence==Speed::OnePausedWork::EQuiescence::ClosedAndJoined&&!worker.IsRunning());
    TestEqual(TEXT("exactly one work call"),calls.load(),1);
    TestEqual(TEXT("real worker cleanup ran once"),closes.load(),1);
    return !HasAnyErrors();
}

namespace
{
FAutoConsoleCommand PreparationPrefixUnits(TEXT("IAmSpeed.Analytic.RunPreparationPrefixUnits"),
    TEXT("Two input/scheduler boundary units; no positive preparation or physics authority."),
    FConsoleCommandDelegate::CreateLambda([]
{
    static bool used=false;auto& f=FAutomationTestFramework::Get();
    if(!IsRunningCommandlet()||!FParse::Param(FCommandLine::Get(),TEXT("SpeedPreparationPrefixUnits"))||used||
        GIsAutomationTesting||GIsSlowTask||GIsPlayInEditorWorld||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty())
    {UE_LOG(LogTemp,Error,TEXT("PreparationPrefix units ownership refused"));return;}used=true;
    const TCHAR* types[]={TEXT("FFreshScenarioSeedInspectionTest"),TEXT("FUnboundPreparationResultTest")};
    const TCHAR* names[]={TEXT("IAmSpeed.Input.NativePreparation.FreshSeedIsReadOnly"),
        TEXT("IAmSpeed.Simulation.NativePreparation.UnboundPrefixResultIsRefused")};
    for(int32 i=0;i<2;++i)
    {
        if(!f.ContainsTest(types[i])){UE_LOG(LogTemp,Error,TEXT("PreparationPrefix test absent"));return;}
        f.SetRequestedTestFilter(EAutomationTestFlags::EngineFilter);f.StartTestByName(types[i],0,names[i]);
        if(!GIsAutomationTesting||!f.GetCurrentTest()||f.GetCurrentTestFullPath()!=names[i]||!f.IsLatentCommandQueueEmpty())
        {UE_LOG(LogTemp,Error,TEXT("PreparationPrefix test ownership changed"));return;}
        FAutomationTestExecutionInfo info;const bool stopped=f.StopTest(info);
        UE_LOG(LogTemp,Display,TEXT("NATIVE_PREPARATION_PREFIX_UNIT name=%s passed=%d errors=%d warnings=%d"),names[i],
            stopped&&info.bSuccessful&&info.GetErrorTotal()==0&&info.GetWarningTotal()==0,info.GetErrorTotal(),info.GetWarningTotal());
        if(GIsAutomationTesting||f.GetCurrentTest()||!f.IsLatentCommandQueueEmpty())
        {UE_LOG(LogTemp,Error,TEXT("PreparationPrefix final ownership changed"));return;}
    }
}));
}
#endif
