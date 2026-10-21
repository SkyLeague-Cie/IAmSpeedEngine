#include "PrivateInputFailureFile.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/CommandLine.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Containers/StringConv.h"

namespace
{
FString PrivateFailureJsonString(const FString& Value)
{
    FString Out(TEXT("\""));
    for (TCHAR C : Value)
    {
        if (C == TCHAR('"')) Out += TEXT("\\\"");
        else if (C == TCHAR('\\')) Out += TEXT("\\\\");
        else if (C < 0x20) Out += FString::Printf(TEXT("\\u%04x"), uint32(C));
        else Out.AppendChar(C);
    }
    Out += TEXT("\""); return Out;
}
}

void RecordPrivateInputFailure(const TCHAR* Site, const FString& Evidence)
{
    static const bool Enabled = FParse::Param(FCommandLine::Get(), TEXT("SLPrivateInputFailureFile"));
    static uint32 ProcessRecordAttempts = 0;
    if (!Enabled || !IsInGameThread() || ProcessRecordAttempts >= 8) return;
    ++ProcessRecordAttempts; // Includes failed writes; no retry loop on disk failure.
    const FString Line = FString::Printf(
        TEXT("{\"schema\":\"sl.private-input-first-fault/v1\",\"utc\":%s,\"pid\":%u,\"record\":%u,\"site\":%s,\"evidence\":%s}\n"),
        *PrivateFailureJsonString(FDateTime::UtcNow().ToIso8601()),
        FPlatformProcess::GetCurrentProcessId(), ProcessRecordAttempts,
        *PrivateFailureJsonString(FString(Site ? Site : TEXT("")).Left(160)),
        *PrivateFailureJsonString(Evidence.Left(2048)));
    const FTCHARToUTF8 Encoded(*Line);
    if (Encoded.Length() > 4096) return; // Entire UTF8 JSONL record including newline.
    const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Diagnostics"));
    if (!IFileManager::Get().MakeDirectory(*Directory, true)) return;
    const FString Filename = FPaths::Combine(Directory, TEXT("InputFirstFault.jsonl"));
    FFileHelper::SaveStringToFile(Line, *Filename, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
        &IFileManager::Get(), FILEWRITE_Append);
}
