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
using namespace Speed::Input::Diagnostics;
FPrivateInputRecordBudgets PrivateInputBudgets;
bool PrivateInputFilesEnabled()
{
    if (!IsInGameThread()) return false;
    static const bool Enabled = FParse::Param(FCommandLine::Get(), TEXT("SLPrivateInputFailureFile"));
    return Enabled;
}
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
void WritePrivateInputRecord(EPrivateInputChannel Channel, const TCHAR* Site, const FString& Evidence)
{
    if (!PrivateInputFilesEnabled()) return;
    const uint32 Record = PrivateInputBudgets.Take(Channel);
    if (!Record) return;
    const TCHAR* Schema = Channel == EPrivateInputChannel::Retirement ? TEXT("sl.private-input-retirement/v1")
        : Channel == EPrivateInputChannel::Startup ? TEXT("sl.private-input-startup/v1") : TEXT("sl.private-input-first-fault/v1");
    const TCHAR* Basename = Channel == EPrivateInputChannel::Retirement ? TEXT("InputRetirement.jsonl")
        : Channel == EPrivateInputChannel::Startup ? TEXT("InputStartup.jsonl") : TEXT("InputFirstFault.jsonl");
    const FString Line = FString::Printf(
        TEXT("{\"schema\":%s,\"utc\":%s,\"pid\":%u,\"record\":%u,\"site\":%s,\"evidence\":%s}\n"),
        *PrivateFailureJsonString(Schema), *PrivateFailureJsonString(FDateTime::UtcNow().ToIso8601()),
        FPlatformProcess::GetCurrentProcessId(), Record,
        *PrivateFailureJsonString(FString(Site ? Site : TEXT("")).Left(160)),
        *PrivateFailureJsonString(Evidence.Left(2048)));
    const FTCHARToUTF8 Encoded(*Line);
    if (Encoded.Length() > 4096) return; // Entire UTF8 JSONL record including newline.
    const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Diagnostics"));
    if (!IFileManager::Get().MakeDirectory(*Directory, true)) return;
    const FString Filename = FPaths::Combine(Directory, Basename);
    FFileHelper::SaveStringToFile(Line, *Filename, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM,
        &IFileManager::Get(), FILEWRITE_Append);
}
}

void RecordPrivateInputFailure(const TCHAR* Site, const FString& Evidence)
{
    using Speed::Input::Diagnostics::EPrivateInputChannel;
    if (!PrivateInputFilesEnabled()) return;
    constexpr TCHAR RetirementPrefix[] = TEXT("PrivateRetirement.");
    const bool Retirement = Site && FCString::Strncmp(Site, RetirementPrefix, UE_ARRAY_COUNT(RetirementPrefix) - 1) == 0;
    WritePrivateInputRecord(Retirement ? EPrivateInputChannel::Retirement : EPrivateInputChannel::FirstFault, Site, Evidence);
}

void RecordPrivateInputStartup(const TCHAR* Site, const FString& Evidence)
{
    WritePrivateInputRecord(Speed::Input::Diagnostics::EPrivateInputChannel::Startup, Site, Evidence);
}

bool CanRecordPrivateInputStartup()
{
    return PrivateInputFilesEnabled() && PrivateInputBudgets.HasCapacity(Speed::Input::Diagnostics::EPrivateInputChannel::Startup);
}
