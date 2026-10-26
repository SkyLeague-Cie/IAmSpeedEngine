#pragma once
#include "CoreMinimal.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"
namespace Speed::Replay
{
inline bool PlaybackDiagnostics32Enabled()
{return FParse::Param(FCommandLine::Get(),TEXT("ReplayPlaybackDiagnostics32"));}
// Boundary-only private diagnostic IO. A fresh explicit -UserDir is mandatory;
// never fall back to project/official Saved. File growth is capped64KiB.
inline bool AppendPlaybackDiagnostics32(const FString& Line)
{
    FString Root;
    if(!FParse::Value(FCommandLine::Get(),TEXT("UserDir="),Root)||Root.IsEmpty()||FPaths::IsRelative(Root))return false;
    Root=FPaths::ConvertRelativePathToFull(Root);FPaths::NormalizeDirectoryName(Root);
    const FString Saved=Root/TEXT("Saved");const FString Path=Saved/TEXT("ReplayPlaybackDiagnostics32.ndjson");
    if(!FPaths::IsUnderDirectory(Path,Root))return false;
    const FString Output=Line+TEXT("\n");const FTCHARToUTF8 UTF8(*Output);
    const int64 Existing=IFileManager::Get().FileSize(*Path);
    if(UTF8.Length()>8192||Existing>65536-UTF8.Length())return false;
    if(!IFileManager::Get().MakeDirectory(*Saved,true))return false;
    return FFileHelper::SaveStringToFile(Output,*Path,FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
        &IFileManager::Get(),FILEWRITE_Append);
}
class FPlaybackDiagnosticScope32
{
    double* Sum=nullptr;double Start=0;
public:
    FPlaybackDiagnosticScope32(bool Enabled,double& Value)
    {if(Enabled){Sum=&Value;Start=FPlatformTime::Seconds();}}
    ~FPlaybackDiagnosticScope32(){if(Sum)*Sum+=FPlatformTime::Seconds()-Start;}
};
}
