#include "NativeFiniteProcessRetention.h"
#include "Misc/CoreMisc.h"
#include <filesystem>
#include <fstream>
#include <utility>

namespace Speed::Netcode
{
FNativeFiniteProcessRetention::FNativeFiniteProcessRetention(std::string Path,
    std::shared_ptr<const Prediction::FNativeProcessCodePins> Pins):ReportBase(std::move(Path)),Code(std::move(Pins)){}
bool FNativeFiniteProcessRetention::Write(const char* Phase,const FFrameExecutorServiceObservation& Actual) const
{
    // Immutable actual OS/service observations, never lifetime or restore permission.
    const auto Path=std::filesystem::path(ReportBase+".process-retention-"+Phase+".json");
    std::ofstream Out(Path,std::ios::out|std::ios::trunc);if(!Out)return false;
    const auto State=[&Out](const FFrameExecutorServiceObservation& V)
    {Out<<"{\"configured\":"<<int(V.Configured)<<",\"stopping\":"<<int(V.Stopping)
        <<",\"retentions\":"<<V.ActualRetentions<<",\"dispatches\":"<<V.ActiveDispatches<<",\"workers\":"<<V.Workers<<'}';};
    Out<<"{\"scope\":\"actual_finite_process_code_and_executor_retention_only\",\"phase\":\""<<Phase
        <<"\",\"original_pid\":"<<Code->OriginalProcess()<<",\"pin_status\":"<<int(Code->Status())
        <<",\"self_module\":"<<Code->ActualSelfModule()<<",\"pin_error\":"<<Code->OriginalError()
        <<",\"before\":";State(Before);Out<<",\"held\":";State(Held);Out<<",\"actual\":";State(Actual);
    Out<<",\"modules\":[";
    bool First=true;for(const auto& R:Code->Records())
    {
        if(!First)Out<<',';First=false;
        Out<<"{\"enumerated_base\":"<<R.EnumeratedBase<<",\"returned_base\":"<<R.ReturnedBase
            <<",\"attempted\":"<<int(R.Attempted)<<",\"pin_returned\":"<<int(R.PinReturned)<<",\"error\":"<<R.Error<<'}';
    }
    Out<<"],\"initial_roster_incarnations\":\"UNKNOWN\",\"preexisting_callback_provenance\":\"UNKNOWN\",\"future_modules\":\"NOT_COVERED\",\"full_world_lease\":false,\"restoreK\":false,\"localized\":false}\n";
    Out.close();return bool(Out);
}
std::unique_ptr<FNativeFiniteProcessRetention> FNativeFiniteProcessRetention::Create(
    const std::string& Path,std::uint32_t OriginalPid,std::string& Why)
{
    Why.clear();
    if(!IsInGameThread()||(!IsRunningCommandlet()&&(GIsEditor||!IsRunningGame())))
    {Why="finite code/service profile requires original GT and private commandlet or non-Editor Game";return {};}
    const std::filesystem::path Report(Path);
    if(Path.empty()||!Report.is_absolute()||(Path.rfind("E:/",0)!=0&&Path.rfind("E:\\",0)!=0))
    {Why="finite retention requires its own absolute E report path";return {};}
    auto Code=Prediction::FNativeProcessCodePins::PinObservedProcessModules();
    if(Code->Status()!=Prediction::EProcessCodePinStatus::PinnedBaseSetAtPinTimes||Code->OriginalProcess()!=OriginalPid)
    {Why="actual process PIN-time code retention failed or original PID differs";return {};}
    auto Root=std::unique_ptr<FNativeFiniteProcessRetention>(new FNativeFiniteProcessRetention(Path,std::move(Code)));
    Root->Before=ObserveFrameExecutorServiceForDiagnostics();
    if(Root->Before.Stopping||(!Root->Before.Configured&&EnsurePrivateFrameWorkers(1)<=0))
    {Why="actual frame service unavailable before private coroutine birth";return {};}
    Root->Service=RetainConfiguredFrameExecutorService();
    Root->Held=ObserveFrameExecutorServiceForDiagnostics();
    if(!Root->Service){Why="actual configured frame service retention refused";return {};}
    if(!Root->Write("acquired",Root->Held)){Why="actual process retention sidecar write failed";return {};}
    return Root;
}
FNativeFiniteProcessRetention::~FNativeFiniteProcessRetention()
{
    // The enclosing FImpl declares us FIRST: native coroutine, immutable pump
    // values and their real C++ finalizers have returned before this release.
    Service.reset();
    try
    {
        if(!Write("released",ObserveFrameExecutorServiceForDiagnostics()))
            UE_LOG(LogTemp,Error,TEXT("[IASFiniteProcessRetention] actual release sidecar write failed"));
    }
    catch(...){UE_LOG(LogTemp,Error,TEXT("[IASFiniteProcessRetention] actual release observation failed"));}
    // Windows retains every actual FLAG_PIN module until the process exits,
    // including through this object's report/destructor return.
}
}
