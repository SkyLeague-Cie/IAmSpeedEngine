#include "PresentationClientWorld.h"
#include "Engine/World.h"
#include "Engine/NetDriver.h"
#include "Engine/NetConnection.h"
#include "Components/PrimitiveComponent.h"
#include "Physics/NetworkPhysicsComponent.h"
#include "UObject/UObjectIterator.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/PlatformTime.h"
#include "IAmSpeed/World/ReplayInertWorld.h"
#include "IAmSpeed/World/Simulation/SpeedPhysicsSimulation.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
namespace Speed::Netcode::PresentationClient
{
namespace
{
struct FRecord
{
    uint64 Generation;FGate Gate;FString Why;
    TWeakObjectPtr<UNetDriver> OriginalDriver;TWeakObjectPtr<UNetConnection> OriginalConnection;
    bool bSawDriver=false,bSawConnection=false;
    explicit FRecord(uint64 G):Generation(G),Gate(true,G,FPlatformTime::Seconds()+15.0){}
};
TMap<TWeakObjectPtr<UWorld>,TUniquePtr<FRecord>> Records;
uint64 NextGeneration=1;
FRecord& Record(UWorld& World)
{
    check(IsInGameThread());
    for(auto It=Records.CreateIterator();It;++It)if(!It.Key().IsValid())It.RemoveCurrent();
    const TWeakObjectPtr<UWorld> Key(&World);
    if(auto* Found=Records.Find(Key))return **Found;
    const uint64 G=NextGeneration==MAX_uint64?0:NextGeneration++;
    return *Records.Add(Key,MakeUnique<FRecord>(G));
}
}
bool IsSelected()
{
    // Immutable process intent; never enabled by a replicated packet or a pawn.
    static const bool Selected=FParse::Param(FCommandLine::Get(),TEXT("SpeedExactRemotePresentationClient"));
    return Selected;
}
void Fail(UWorld& World,const FString& Why)
{
    if(!IsSelected())return;
    auto& R=Record(World);const bool First=R.Gate.GetState()!=EState::Failed;
    R.Gate.Fail();if(R.Why.IsEmpty())R.Why=Why;
    if(First)UE_LOG(LogTemp,Error,TEXT("[PresentationClientFailed] %s"),*R.Why);
}
EState Query(UWorld& World,uint64& Generation,FString& Why)
{
    check(IsInGameThread());Generation=0;Why.Reset();if(!IsSelected())return EState::Unselected;
    auto& R=Record(World);Generation=R.Generation;
    auto* Driver=World.GetNetDriver();auto* Connection=Driver?Driver->ServerConnection.Get():nullptr;
    const bool ChangedOriginal=(R.bSawDriver&&(!R.OriginalDriver.IsValid()||R.OriginalDriver.Get()!=Driver))
        ||(R.bSawConnection&&(!R.OriginalConnection.IsValid()||R.OriginalConnection.Get()!=Connection));
    if(!R.bSawDriver&&Driver){R.OriginalDriver=Driver;R.bSawDriver=true;}
    if(!R.bSawConnection&&Connection){R.OriginalConnection=Connection;R.bSawConnection=true;}
    const bool Conflict=ChangedOriginal||FParse::Param(FCommandLine::Get(),TEXT("SpeedExactRemoteDedicatedHost"))
        ||Speed::ReplayIsolation::IsInertReplayWorld(&World)||!World.IsGameWorld();
    const auto Mode=World.GetNetMode();
    R.Gate.Observe(FPlatformTime::Seconds(),Mode==NM_Client,Mode==NM_ListenServer||Mode==NM_DedicatedServer,
        reinterpret_cast<std::uintptr_t>(Driver),reinterpret_cast<std::uintptr_t>(Connection),Conflict);
    if(R.Gate.GetState()==EState::Failed&&R.Why.IsEmpty())
    {R.Why=TEXT("Presentation intent expired, conflicted, changed original connection, or attempted a non-client fallback");UE_LOG(LogTemp,Error,TEXT("[PresentationClientFailed] %s"),*R.Why);}
    Why=R.Why;return R.Gate.GetState();
}
bool SuppressesLocalPhysics(const UWorld* World)
{
    if(!IsSelected())return false;
    if(IsInGameThread()&&World){uint64 Generation;FString Why;Query(*const_cast<UWorld*>(World),Generation,Why);}
    return true; // Includes null world, Pending, Failed and off-thread attempts.
}
bool Audit(UWorld& World,FString& Why)
{
    check(IsInGameThread());uint64 Generation=0;const auto State=Query(World,Generation,Why);
    if(State==EState::Unselected){Why=TEXT("Presentation intent absent");return false;}
    if(State==EState::Failed)return false;
    for(TObjectIterator<ASpeedPhysicsSimulation> It;It;++It)
        if(!It->IsTemplate()&&It->GetWorld()==&World&&It->HasOwnedWorkerForPresentationAdmission())
        {Why=TEXT("Existing IAS physical worker; no retroactive admission");Fail(World,Why);return false;}
    for(TObjectIterator<UNetworkPhysicsComponent> It;It;++It)
        if(!It->IsTemplate()&&It->GetWorld()==&World&&(It->IsRegistered()
            ||It->GetStateHistory_External()||It->GetInputHistory_External()))
        {Why=TEXT("Existing network physics registration/history; retroactive removal forbidden");Fail(World,Why);return false;}
    for(TObjectIterator<UPrimitiveComponent> It;It;++It)
        if(!It->IsTemplate()&&It->GetWorld()==&World&&It->IsRegistered()&&It->IsSimulatingPhysics())
        {Why=TEXT("Uninventoried simulating Chaos body; host must reject its Blueprint/spawn path");Fail(World,Why);return false;}
    auto* Bridge=World.GetSubsystem<USpeedWorldSubsystem>();
    if(Bridge&&Bridge->HasPhysicalRegistrationsForPresentationAdmission())
    {Why=TEXT("Existing IAS physical registration or pending operation; no retroactive flush");Fail(World,Why);return false;}
    Why.Reset();return true;
}
bool Matches(UWorld& World,uint64 Generation,const FTuple& Tuple,FString& Why)
{
    uint64 Actual=0;if(Query(World,Actual,Why)!=EState::Admitted)return false;
    if(Generation==Actual&&Record(World).Gate.Matches(Tuple))return true;
    Why=TEXT("Stale or wrong presentation tuple/generation");Fail(World,Why);return false;
}
bool Admit(UWorld& World,uint64 Generation,UNetDriver& Driver,UNetConnection& Connection,
    const FTuple& Trusted,const FTuple& Received,FString& Why)
{
    uint64 Actual=0;const auto State=Query(World,Actual,Why);
    if(State!=EState::Pending)
    {if(State==EState::Admitted){Why=TEXT("Re-admission of an immutable client generation forbidden");Fail(World,Why);}return false;}
    if(Generation!=Actual||World.GetNetMode()!=NM_Client||World.GetNetDriver()!=&Driver
        ||Driver.ServerConnection.Get()!=&Connection||Connection.GetDriver()!=&Driver||!Audit(World,Why))
    {if(Why.IsEmpty())Why=TEXT("Original client world/driver/connection/generation mismatch");Fail(World,Why);return false;}
    auto& R=Record(World);
    if(!R.Gate.Admit(FPlatformTime::Seconds(),Generation,reinterpret_cast<std::uintptr_t>(&Driver),
        reinterpret_cast<std::uintptr_t>(&Connection),Trusted,Received,true,true))
    {Why=TEXT("Complete authenticated tuple/contract-policy-mode mismatch");Fail(World,Why);return false;}
    return true;
}
}
