#include "FrameAppliedCommandWitness.h"
#include "NativeFrameWitnessRecorder.h"
#if !UE_BUILD_SHIPPING
#include "IAmSpeed/Base/EpisodeStateBytesForTesting.h"
#include "IAmSpeed/World/Analytic/AnalyticCaseSatDiagnostics.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
namespace Speed
{
namespace {thread_local FFrameAppliedCommandWitnessScope* ActiveCommands=nullptr;}
FFrameAppliedCommandWitnessScope::FFrameAppliedCommandWitnessScope(uint64 F,int32 Expected):Frame(F),ExpectedJournal(Expected)
{Previous=ActiveCommands;ActiveCommands=this;if(Previous||Expected<0)bUnknown=true;}
FFrameAppliedCommandWitnessScope::~FFrameAppliedCommandWitnessScope(){check(ActiveCommands==this);ActiveCommands=Previous;}
bool FFrameAppliedCommandWitnessScope::IsActive(){return ActiveCommands!=nullptr;}
void FFrameAppliedCommandWitnessScope::ExpectAnalogActor(uint64 Actor)noexcept
{auto*S=ActiveCommands;if(!S)return;try{if(S->bExpectedAnalogSealed||!Actor||S->ExpectedAnalog.Contains(Actor)){S->bUnknown=true;return;}S->ExpectedAnalog.Add(Actor);}catch(...){S->bUnknown=true;}}
bool FFrameAppliedCommandWitnessScope::SealExpectedAnalogActors() noexcept
{auto*S=ActiveCommands;if(!S)return false;if(S->bExpectedAnalogSealed){S->bUnknown=true;return false;}S->bExpectedAnalogSealed=true;return !S->bUnknown;}
void FFrameAppliedCommandWitnessScope::MarkCoverageUnknown() noexcept
{if(auto*S=ActiveCommands)S->bUnknown=true;}
void FFrameAppliedCommandWitnessScope::RecordJournal(uint64 F,uint64 Actor,const TArray<uint8>& Payload,bool Accepted)noexcept
{
 auto*S=ActiveCommands;if(!S)return;try{
 if(F!=S->Frame||!Actor||Payload.Num()>1024*1024||S->ObservedJournal>=65536){S->bUnknown=true;return;}
 using W=FEpisodeStateBytesForTesting;W::Add(S->Raw,uint8(1));W::Add(S->Raw,F);W::Add(S->Raw,Actor);W::Add(S->Raw,Accepted);
 W::Add(S->Raw,Payload.Num());S->Raw.Append(Payload);++S->ObservedJournal;if(!Accepted)S->bUnknown=true;
 }catch(...){S->bUnknown=true;}
}
void FFrameAppliedCommandWitnessScope::RecordAnalog(uint64 Actor,uint64 LocalFrame,const TArray<uint8>& Payload)noexcept
{
 auto*S=ActiveCommands;if(!S)return;try{
 if(!S->bExpectedAnalogSealed||!S->ExpectedAnalog.Contains(Actor))S->bUnknown=true;
 if(!Actor||Payload.Num()>65536||S->ObservedAnalog.Contains(Actor)){S->bUnknown=true;return;}
 using W=FEpisodeStateBytesForTesting;W::Add(S->Raw,uint8(2));W::Add(S->Raw,S->Frame);W::Add(S->Raw,Actor);W::Add(S->Raw,LocalFrame);
 W::Add(S->Raw,Payload.Num());S->Raw.Append(Payload);S->ObservedAnalog.Add(Actor);
 }catch(...){S->bUnknown=true;}
}
bool FFrameAppliedCommandWitnessScope::Complete()const
{
 if(!bExpectedAnalogSealed||bUnknown||ExpectedJournal!=ObservedJournal||ExpectedAnalog.Num()!=ObservedAnalog.Num())return false;
 for(uint64 Actor:ExpectedAnalog)if(!ObservedAnalog.Contains(Actor))return false;return true;
}
bool FFrameAppliedCommandWitnessScope::SaveCommittedFrame()const
{
 const auto* Recorder=FNativeFrameWitnessRecorder::Current();
 if(!Recorder || Recorder->CanonicalFrame()!=Frame)return false;
 const FString Folder=Recorder->OutputDirectory();
 const FString Data=FPaths::Combine(Folder,TEXT("actual_commands.raw"));const FString Meta=FPaths::Combine(Folder,TEXT("commands.json"));
 if(IFileManager::Get().FileExists(*Data)||IFileManager::Get().FileExists(*Meta))return false;
 if(!FFileHelper::SaveArrayToFile(Raw,*Data))return false;
 const FString Text=FString::Printf(TEXT("{\"schema\":1,\"frame\":%llu,\"complete\":%s,\"expected_journal\":%d,\"observed_journal\":%d,\"expected_analog_actors\":%d,\"observed_analog_actors\":%d}"),
  Frame,Complete()?TEXT("true"):TEXT("false"),ExpectedJournal,ObservedJournal,ExpectedAnalog.Num(),ObservedAnalog.Num());
 return FFileHelper::SaveStringToFile(Text,*Meta,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
}
}
#endif
