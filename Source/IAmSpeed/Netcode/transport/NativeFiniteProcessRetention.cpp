#include "NativeFiniteProcessRetention.h"
#include "IAmSpeed/Netcode/client-prediction/ProcessCodePinJson.h"
#include "Misc/CoreMisc.h"
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>

namespace Speed::Netcode
{
namespace
{
bool WriteCodeRefusal(const std::string& Base,const Prediction::FNativeProcessCodePins& Code) noexcept try
{
    std::ofstream Out(std::filesystem::path(Base+".process-code-refusal.json"),std::ios::out|std::ios::trunc);
    if(!Out)return false;
    Out<<R"({"scope":"actual_refused_process_pin_observations_only","admitted":false,"original_pid":)"<<Code.OriginalProcess()
        <<R"(,"pin_status":)"<<int(Code.Status())<<R"(,"pin_error":)"<<Code.OriginalError()<<R"(,"self_module":)"<<Code.ActualSelfModule();
    Prediction::AppendProcessCodePinRecords(Out,Code);
    Prediction::AppendProcessCodePinPasses(Out,Code);
    Out<<R"(,"initial_roster_incarnations":"UNKNOWN","preexisting_callback_provenance":"UNKNOWN","future_modules":"NOT_COVERED","full_world_lease":false,"restoreK":false,"localized":false})"<<'\n';
    Out.close();return bool(Out);
}
catch(...){return false;}
}

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
    Prediction::AppendProcessCodePinRecords(Out,*Code);
    Prediction::AppendProcessCodePinPasses(Out,*Code);
    Out<<",\"initial_roster_incarnations\":\"UNKNOWN\",\"preexisting_callback_provenance\":\"UNKNOWN\",\"future_modules\":\"NOT_COVERED\",\"full_world_lease\":false,\"restoreK\":false,\"localized\":false}\n";
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
    {
        // Preserve the actual refusing report before it retires. This is only
        // diagnostic data: the same condition still refuses before service
        // acquisition or private coroutine birth, with no retry or relaxation.
        Why="actual process PIN-time code retention refused: pin_status="+std::to_string(unsigned(Code->Status()))
            +" pin_error="+std::to_string(Code->OriginalError())
            +" actual_pid="+std::to_string(Code->OriginalProcess())
            +" expected_pid="+std::to_string(OriginalPid)
            +" observed_modules="+std::to_string(Code->Records().size());
        if(!WriteCodeRefusal(Path,*Code))Why+=" refusal_report_write_failed=1";
        return {};
    }
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
