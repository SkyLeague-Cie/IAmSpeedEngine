#include "NativeEpochDiagnostics.h"
namespace Speed::NativeEpoch
{
namespace {thread_local FDiagnosticOutbox* Current=nullptr;}
FDiagnosticOutbox::FDiagnosticOutbox() {Records.reserve(4096);}
FDiagnosticOutbox::~FDiagnosticOutbox() {check(!Active&&Current!=this);}
bool FDiagnosticOutbox::Begin() noexcept
{
    if(!IsInGameThread()||Active||Current)return false;
    Records.clear();Characters=0;Overflow=false;Active=true;Current=this;return true;
}
void FDiagnosticOutbox::End() noexcept
{
    check(IsInGameThread()&&Active&&Current==this);
    Current=nullptr;Active=false;
}
bool HasOriginalDiagnosticOutbox() noexcept {return Current!=nullptr;}
void QueueOriginalDiagnostic(const ANSICHAR* File,int32 Line,const FLogCategoryName& Category,
    ELogVerbosity::Type Verbosity,FString Message)
{
    check(Current&&Current->Active&&IsInGameThread());
    auto& Q=*Current;
    if(Q.Records.size()>=4096||uint64(Message.Len())>1048576-Q.Characters)
    {
        // Overflow is an explicit whole-epoch failure/diagnostic coverage
        // UNKNOWN. It never becomes a successful filtered log or publication.
        Q.Overflow=true;return;
    }
    Q.Characters+=uint64(Message.Len());
    Q.Records.push_back({File,Line,Category,Verbosity,MoveTemp(Message)});
}
bool FDiagnosticOutbox::Flush()
{
    check(IsInGameThread()&&!Active&&!Current);
    for(const auto& R:Records)
        FMsg::Logf_Internal(R.File,R.Line,R.Category,R.Verbosity,TEXT("%s"),*R.Message);
    if(Overflow)UE_LOG(LogTemp,Error,TEXT("[NativeEpochDiagnosticOverflow] Original stage refused; diagnostic coverage UNKNOWN; no successful publication."));
    Records.clear();return !Overflow;
}
}
