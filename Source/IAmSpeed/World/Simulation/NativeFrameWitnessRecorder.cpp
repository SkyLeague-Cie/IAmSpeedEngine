#include "NativeFrameWitnessRecorder.h"
#include "FrameWitnessBlob.h"
#if !UE_BUILD_SHIPPING
#include "FrameInputWitness.h"
#include "FrameSolveWitness.h"
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "HAL/IConsoleManager.h"
#include "HAL/FileManager.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
namespace Speed
{
namespace
{
TAutoConsoleVariable<int32> CVarNativeFrameWitness(TEXT("p.IAmSpeed.FrameWitness.Enabled"),0,
 TEXT("Correctness-only native frame witness; requires SLFrameWitnessOutput and joined native case identity. Keep 0 for timing."));
TAutoConsoleVariable<int32> CVarNativeFrameWitnessCompression(TEXT("p.IAmSpeed.FrameWitness.Compression"),0,
 TEXT("Lossless IASZLIB1 diagnostic blobs; keep witness disabled in performance runs."));
thread_local FNativeFrameWitnessRecorder* ActiveRecorder=nullptr;
thread_local FString ReceiptEpoch;
thread_local TMap<uint64,uint64> ReceiptCursors;
FString JBool(bool B){return B?TEXT("true"):TEXT("false");}
bool SafeIdentity(const FString& S)
{if(S.IsEmpty()||S.Len()>64)return false;for(TCHAR C:S)if(!FChar::IsAlnum(C)&&C!='-'&&C!='_')return false;return true;}
}
struct FNativeFrameWitnessRecorder::FImpl
{
 uint64 Frame=0;float Delta=0;FString Directory,Run,Epoch;
 FNativeFrameWitnessCaseIdentity Identity{};
 FFramePointerRegistry Registry;
 TArray<uint8> Installed,Completed,ContactsRaw,ContactsCanonical;
 TUniquePtr<FFrameSolveWitnessScope> Solve;
 TSet<uint64> Actors;TArray<FString> ActorRows;TArray<FString> AdapterCensus;
 int32 ExpectedActors=0;
 bool bInstalled=false,bCompleted=false,bInstalledAbsent=false,bFinished=false,bWriteOK=true,bCompressed=false;
 bool Write(const FString& Name,const TArray<uint8>& Bytes)
 {
  const FString PlainPath=FPaths::Combine(Directory,Name);
  const FString Path=bCompressed?PlainPath+TEXT(".zlib"):PlainPath;
  if(IFileManager::Get().FileExists(*PlainPath)||IFileManager::Get().FileExists(*(PlainPath+TEXT(".zlib")))){bWriteOK=false;return false;}
  TArray<uint8> Packed;
  if(bCompressed && !FrameWitnessBlob::Encode(Bytes,Packed)){bWriteOK=false;return false;}
  const bool OK=FFileHelper::SaveArrayToFile(bCompressed?Packed:Bytes,*Path);bWriteOK&=OK;return OK;
 }
};
FNativeFrameWitnessRecorder::FNativeFrameWitnessRecorder(TUniquePtr<FImpl>&& In):Impl(MoveTemp(In))
{Impl->Solve=MakeUnique<FFrameSolveWitnessScope>(Impl->Frame,Impl->ContactsRaw,Impl->Registry);ActiveRecorder=this;}
TUniquePtr<FNativeFrameWitnessRecorder> FNativeFrameWitnessRecorder::Begin(uint64 Frame,float Delta,
 const Input::V2::FRegistryFrame* Installed,USpeedWorldSubsystem& World,const FNativeFrameWitnessCaseIdentity& Case,bool bNoSessionsAdmitted)
{
 if(!CVarNativeFrameWitness.GetValueOnAnyThread())return {};
 if(ActiveRecorder){UE_LOG(LogTemp,Error,TEXT("[FrameWitnessRejected] nested recorder"));return {};}
 FString Root;FNativeFrameWitnessCaseIdentity ID=Case;ID.CanonicalFrame=Frame;
 if(!FParse::Value(FCommandLine::Get(),TEXT("SLFrameWitnessOutput="),Root)||FPaths::IsRelative(Root)||
   !ID.bConfigured||!ID.DriverKey||!ID.Epoch||ID.NativeCaseId<0)
 {UE_LOG(LogTemp,Error,TEXT("[FrameWitnessRejected] missing absolute output or owned identity Frame=%llu"),Frame);return {};}
 const FString Run(ID.RunUUID);if(!SafeIdentity(Run))return {};
 auto Data=MakeUnique<FImpl>();Data->bCompressed=CVarNativeFrameWitnessCompression.GetValueOnAnyThread()!=0;Data->Frame=Frame;Data->Delta=Delta;Data->Identity=ID;Data->Run=Run;
 Data->Epoch=FString::Printf(TEXT("%s_%llu_%llu_%d"),*Run,ID.DriverKey,ID.Epoch,ID.NativeCaseId);
 if(ReceiptEpoch!=Data->Epoch){ReceiptCursors.Reset();ReceiptEpoch=Data->Epoch;}
 Data->Directory=FPaths::Combine(Root,Data->Epoch,FString::Printf(TEXT("frame_%020llu"),Frame));
 if(IFileManager::Get().DirectoryExists(*Data->Directory)||!IFileManager::Get().MakeDirectory(*Data->Directory,true))
 {UE_LOG(LogTemp,Error,TEXT("[FrameWitnessRejected] existing/unwritable frame directory %s"),*Data->Directory);return {};}
 World.RegisterFramePointerWitnessForTesting(Data->Registry,Data->ExpectedActors);
 World.FrameAdapterCensusForTesting(Data->AdapterCensus);
 Data->Registry.Seal();
 Data->bInstalledAbsent=!Installed && bNoSessionsAdmitted;
 Data->bInstalled=AppendFrameInputEndpointWitness(Frame,EFrameInputWitnessStage::Installed,Installed,bNoSessionsAdmitted,Data->Installed);
 return TUniquePtr<FNativeFrameWitnessRecorder>(new FNativeFrameWitnessRecorder(MoveTemp(Data)));
}
FNativeFrameWitnessRecorder* FNativeFrameWitnessRecorder::Current(){return ActiveRecorder;}
const FFramePointerRegistry& FNativeFrameWitnessRecorder::PointerRegistry()const{return Impl->Registry;}
bool FNativeFrameWitnessRecorder::ExtendPointerRegistry(TFunctionRef<bool(FFramePointerRegistry&)> Declare)
{
 auto Extended=Impl->Registry.CloneForAdditionalDeclarations();const bool OK=Declare(Extended);
 const bool Sealed=Extended.Seal();Impl->Registry=MoveTemp(Extended);return OK&&Sealed;
}
uint64 FNativeFrameWitnessRecorder::CanonicalFrame()const{return Impl->Frame;}
const FString& FNativeFrameWitnessRecorder::OutputDirectory()const{return Impl->Directory;}
uint64 FNativeFrameWitnessRecorder::ReceiptCursor(uint64 Actor)const
{const uint64* V=ReceiptCursors.Find(Actor);return V?*V:0;}
void FNativeFrameWitnessRecorder::RecordActor(uint64 Actor,uint64 LocalFrame,const TArray<uint8>& Raw,
 const TArray<uint8>& Canonical,const TArray<uint8>& Ledger,const TArray<uint8>& Effects,
 const TArray<FFramePointerObservation>& Pointers,uint64 Next,bool Boundary,bool Fields,bool Known,bool LedgerOK,bool EffectsOK)
{
 if(Impl->bFinished||!Actor||Impl->Actors.Contains(Actor)){Impl->bWriteOK=false;return;}
 Impl->Actors.Add(Actor);const FString Prefix=FString::Printf(TEXT("actor_%020llu"),Actor);
 Impl->Write(Prefix+TEXT(".mechanics.raw"),Raw);Impl->Write(Prefix+TEXT(".mechanics.canonical"),Canonical);
 Impl->Write(Prefix+TEXT(".ledger.raw"),Ledger);Impl->Write(Prefix+TEXT(".effects.raw"),Effects);
 TArray<uint8> Sidecar;using W=FEpisodeStateBytesForTesting;W::Add(Sidecar,uint32(4));W::Add(Sidecar,Pointers.Num());
 for(const auto& P:Pointers){W::Add(Sidecar,P.Offset);W::Add(Sidecar,P.RawAddress);W::Add(Sidecar,P.bKnown);
  W::Add(Sidecar,P.Key.Domain);W::Add(Sidecar,P.Key.Owner);W::Add(Sidecar,P.Key.Generation);W::Add(Sidecar,P.Key.Role);W::Add(Sidecar,P.Key.Element);W::Add(Sidecar,uint32(P.Key.Membership.Num()));for(uint64 Id:P.Key.Membership)W::Add(Sidecar,Id);
  W::Add(Sidecar,P.bObjectId);W::Add(Sidecar,P.bPairToken);W::Add(Sidecar,P.SecondKey.Domain);W::Add(Sidecar,P.SecondKey.Owner);
  W::Add(Sidecar,P.SecondKey.Generation);W::Add(Sidecar,P.SecondKey.Role);W::Add(Sidecar,P.SecondKey.Element);W::Add(Sidecar,uint32(P.SecondKey.Membership.Num()));for(uint64 Id:P.SecondKey.Membership)W::Add(Sidecar,Id);}
 Impl->Write(Prefix+TEXT(".pointers.raw"),Sidecar);
 Impl->ActorRows.Add(FString::Printf(TEXT("{\"actor\":%llu,\"local_frame\":%llu,\"prefix\":\"%s\",\"boundary\":%s,\"fields\":%s,\"pointers\":%s,\"ledger\":%s,\"effects\":%s}"),
  Actor,LocalFrame,*Prefix,*JBool(Boundary),*JBool(Fields),*JBool(Known),*JBool(LedgerOK),*JBool(EffectsOK)));
 if(EffectsOK)ReceiptCursors.Add(Actor,Next);
}
void FNativeFrameWitnessRecorder::Finish(const Input::V2::FRegistryFrame* Completed,
 const TArray<uint8>& Snapshot,int32 Resolved,const TArray<uint8>& Camera,bool CameraOK,bool bNoSessionsAdmitted)
{
 if(Impl->bFinished)return;
 Impl->bCompleted=Impl->bInstalled && bNoSessionsAdmitted==Impl->bInstalledAbsent &&
  AppendFrameInputEndpointWitness(Impl->Frame,EFrameInputWitnessStage::RegistryCompleted,Completed,bNoSessionsAdmitted,Impl->Completed);
 const int32 Observed=Impl->Solve->NumRows();
 const int32 MainObserved=Impl->Solve->NumMainRows(),DynamicObserved=Impl->Solve->NumDynamicRows();
 const bool ContactsOK=Impl->Solve->MakeCanonical(Impl->ContactsCanonical);
 Impl->Solve.Reset();
 Impl->Write(TEXT("installed_inputs.raw"),Impl->Installed);Impl->Write(TEXT("completed_inputs.raw"),Impl->Completed);
 Impl->Write(TEXT("snapshot.raw"),Snapshot);Impl->Write(TEXT("ordered_contacts.raw"),Impl->ContactsRaw);
 Impl->Write(TEXT("ordered_contacts.canonical"),Impl->ContactsCanonical);Impl->Write(TEXT("camera.raw"),Camera);
 uint32 DeltaBits=0;FMemory::Memcpy(&DeltaBits,&Impl->Delta,sizeof(DeltaBits));
 const FString Meta=FString::Printf(TEXT("{\"schema\":1,\"state\":\"COMMITTED_FRAME_PROCESS_UNCLOSED\",\"run_uuid\":\"%s\",\"driver\":%llu,\"epoch\":%llu,\"world_generation\":%llu,\"case_id\":%d,\"batch_index\":%d,\"mode\":%d,\"frame\":%llu,\"delta_bits_u32\":%u,\"installed\":%s,\"completed\":%s,\"solve_rows\":%d,\"main_solve_rows\":%d,\"dynamic_solve_rows\":%d,\"resolved_events\":%d,\"solve_complete\":%s,\"camera_complete\":%s,\"expected_actors\":%d,\"observed_actors\":%d,\"actors\":[%s],\"full_physics_qualified\":false,\"write_ok\":%s}"),
  *Impl->Run,Impl->Identity.DriverKey,Impl->Identity.Epoch,Impl->Identity.WorldGeneration,Impl->Identity.NativeCaseId,
  Impl->Identity.BatchIndex,Impl->Identity.Mode,Impl->Frame,DeltaBits,*JBool(Impl->bInstalled),*JBool(Impl->bCompleted),Observed,MainObserved,DynamicObserved,Resolved,
  *JBool(ContactsOK&&MainObserved==Resolved),*JBool(CameraOK),Impl->ExpectedActors,Impl->Actors.Num(),*FString::Join(Impl->ActorRows,TEXT(",")),*JBool(Impl->bWriteOK));
 // Census is diagnostic identity, not a substitute for an actor state capture.
 const FString Census=FString::Printf(TEXT("{\"schema\":1,\"frame\":%llu,\"expected_actors\":%d,\"adapters\":[%s]}"),
  Impl->Frame,Impl->ExpectedActors,*FString::Join(Impl->AdapterCensus,TEXT(",")));
 const FString CensusPath=FPaths::Combine(Impl->Directory,TEXT("adapter_census.json"));
 if(IFileManager::Get().FileExists(*CensusPath)||!FFileHelper::SaveStringToFile(Census,*CensusPath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))Impl->bWriteOK=false;
 const FString MetaPath=FPaths::Combine(Impl->Directory,TEXT("frame.json"));
 if(IFileManager::Get().FileExists(*MetaPath)||!FFileHelper::SaveStringToFile(Meta,*MetaPath,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))Impl->bWriteOK=false;
 Impl->bFinished=true;
 UE_LOG(LogTemp,Display,TEXT("[FrameWitnessCommitted] Case=%d Frame=%llu Actors=%d/%d SolveMain=%d/%d SolveDynamic=%d SolveTotal=%d WriteOK=%d FullPhysicsQualified=0"),
  Impl->Identity.NativeCaseId,Impl->Frame,Impl->Actors.Num(),Impl->ExpectedActors,MainObserved,Resolved,DynamicObserved,Observed,int32(Impl->bWriteOK));
}
FNativeFrameWitnessRecorder::~FNativeFrameWitnessRecorder()
{
 Impl->Solve.Reset();if(ActiveRecorder==this)ActiveRecorder=nullptr;
 if(!Impl->bFinished)UE_LOG(LogTemp,Error,TEXT("[FrameWitnessUnclosed] Case=%d Frame=%llu"),Impl->Identity.NativeCaseId,Impl->Frame);
}
}
#endif
