#include "IAmSpeed/Components/SpeedWheeledComponent.h"
#include "IAmSpeed/Base/CarBumpCooldown.h"
#include "IAmSpeed/Base/CarBumpSnapshot.h"
#include "IAmSpeed/World/Subsystem/SpeedWorldSubsystem.h"
#include "Engine/World.h"

namespace
{
bool ReadBumpPayload(TConstArrayView<uint8> Bytes,uint16& Frames,uint64& VictimId)
{
    return Speed::Collision::DecodeCarBumpSnapshot(Bytes.GetData(),Bytes.Num(),Frames,VictimId);
}
USpeedWheeledComponent* ResolveBumpVictim(const UWorld* World,uint64 Id)
{
    if(!World || Id==0) return nullptr;
    const auto* SpeedWorld=World->GetSubsystem<USpeedWorldSubsystem>();
    ISpeedComponent* Adapter=SpeedWorld?SpeedWorld->FindSimulationAdapter(Id):nullptr;
    return Adapter?Adapter->AsWheeledPhysicsComponent():nullptr;
}
}
bool USpeedWheeledComponent::IsCarBumpInCooldown(const USpeedWheeledComponent& Victim) const
{
    return Speed::Collision::IsCarBumpInCooldown(
        WheeledPhysicsState.LastCarBumpVictim.Get(),&Victim,WheeledPhysicsState.CarBumpCooldownFrames);
}
bool USpeedWheeledComponent::TryBeginCarBump(USpeedWheeledComponent& Victim)
{
    if(&Victim==this || IsCarBumpInCooldown(Victim)) return false;
    WheeledPhysicsState.LastCarBumpVictim=&Victim;
    WheeledPhysicsState.CarBumpCooldownFrames=static_cast<uint16>(FMath::Clamp(
        FMath::CeilToInt(GetEngineFPS()*0.25f),1,65535));
    return true;
}
void USpeedWheeledComponent::ResetCarBumpCooldown()
{
    WheeledPhysicsState.LastCarBumpVictim.Reset();
    WheeledPhysicsState.CarBumpCooldownFrames=0;
}
void USpeedWheeledComponent::AppendSimulationSnapshot(TArray<uint8>& Out) const
{
    USpeedWheeledComponent* Victim=WheeledPhysicsState.LastCarBumpVictim.Get();
    const uint16 Frames=Victim?WheeledPhysicsState.CarBumpCooldownFrames:0;
    const uint64 Id=Frames?Victim->GetPublishedSimulationStableId():0;
    checkf(!Frames || Id!=0,TEXT("Bump victim must belong to the canonical registry before capture"));
    const auto Bytes=Speed::Collision::EncodeCarBumpSnapshot(Frames,Id);
    Out.Append(Bytes.data(),static_cast<int32>(Bytes.size()));
}
bool USpeedWheeledComponent::CanRestoreSimulationSnapshot(TConstArrayView<uint8> Payload) const
{
    uint16 Frames=0;uint64 Id=0;
    if(!ReadBumpPayload(Payload,Frames,Id)) return false;
    if(Id==0) return true;
    const auto* Victim=ResolveBumpVictim(GetWorld(),Id);
    return Victim && Victim!=this;
}
void USpeedWheeledComponent::RestoreSimulationSnapshotPayload(TConstArrayView<uint8> Payload)
{
    uint16 Frames=0;uint64 Id=0;
    const bool bRead=ReadBumpPayload(Payload,Frames,Id);
    check(bRead);
    if(!bRead) return;
    WheeledPhysicsState.LastCarBumpVictim=ResolveBumpVictim(GetWorld(),Id);
    WheeledPhysicsState.CarBumpCooldownFrames=Frames;
}
