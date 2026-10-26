#pragma once
#include "CoreMinimal.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "Engine/GameInstance.h"
#include "UObject/Package.h"
namespace Speed::ReplayIsolation
{
// Canonical name is only a candidate-match key. It never authorizes a world.
// Exact context, instance and later component/bootstrap checks remain mandatory.
inline FName CanonicalReplayWorldPackage(const UWorld& World)
{
    const FString Name=World.GetOutermost()->GetName();
    return FName(*(World.WorldType==EWorldType::PIE?UWorld::RemovePIEPrefix(Name):Name));
}
struct ReplayNativeWorldIdentity
{
    EWorldType::Type Type=EWorldType::None;
    int32 PIEInstance=INDEX_NONE;
    FName Package;
};
inline bool ReadReplayNativeWorldIdentity(const UWorld& World,ReplayNativeWorldIdentity& Out)
{
    check(IsInGameThread());Out={};
    if(!GEngine||!World.GetGameInstance()||World.GetNetMode()!=NM_Standalone)return false;
    const FWorldContext* Context=GEngine->GetWorldContextFromWorld(&World);
    if(!Context||Context->World()!=&World||Context->WorldType!=World.WorldType||
       Context->OwningGameInstance!=World.GetGameInstance())return false;
    const UPackage* Package=World.GetOutermost();
    if(World.WorldType==EWorldType::Game)
    {
        if(Package->HasAnyPackageFlags(PKG_PlayInEditor))return false;
        Out.Type=EWorldType::Game;Out.Package=Package->GetFName();return true;
    }
#if WITH_EDITOR
    if(World.WorldType==EWorldType::PIE)
    {
        int32 PrefixInstance=INDEX_NONE;
        const FString Canonical=UWorld::RemovePIEPrefix(Package->GetName(),&PrefixInstance);
        const int32 Instance=Package->GetPIEInstanceID();
        if(!Package->HasAnyPackageFlags(PKG_PlayInEditor)||Instance<0||Context->PIEInstance!=Instance||
           PrefixInstance!=Instance||UWorld::ConvertToPIEPackageName(Canonical,Instance)!=Package->GetName())return false;
        Out.Type=EWorldType::PIE;Out.PIEInstance=Instance;Out.Package=FName(*Canonical);return true;
    }
#endif
    return false;
}
}
