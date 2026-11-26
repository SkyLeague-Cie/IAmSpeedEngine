#pragma once
#include "CoreMinimal.h"
#include <type_traits>
#include "IAmSpeed/Input/InputSessionRegistry.h"
#if !UE_BUILD_SHIPPING
namespace Speed
{
enum class EFrameInputWitnessStage:uint8 { Installed=1,RegistryCompleted=2 };
inline void AppendAbsentFrameInputWitness(uint64 Frame,EFrameInputWitnessStage Stage,TArray<uint8>& Out)
{
 Out.Reset();const uint32 Version=2;const uint8 S=uint8(Stage),Present=0;
 Out.Append(reinterpret_cast<const uint8*>(&Version),sizeof(Version));Out.Add(S);
 Out.Append(reinterpret_cast<const uint8*>(&Frame),sizeof(Frame));Out.Add(Present);
}
// Pointer-free capture of the real registry endpoint values. Installed and
// RegistryCompleted are distinct facts; neither proves actor-effect execution.
// The owner must additionally observe UpdateInputs/actual discrete dispatch.
inline bool AppendFrameInputWitness(const Input::V2::FRegistryFrame& Frame,
    EFrameInputWitnessStage Stage,TArray<uint8>& Bytes)
{
    using namespace Input::V2;
    if (!Frame.Registry || Frame.Inputs.size()>16 ||
        (Stage!=EFrameInputWitnessStage::Installed && Stage!=EFrameInputWitnessStage::RegistryCompleted)) return false;
    TArray<uint8> Out;
    const auto Add=[&Out](const auto& V)
    {
        using T=std::decay_t<decltype(V)>;
        if constexpr(std::is_enum_v<T>) { const auto E=static_cast<std::underlying_type_t<T>>(V);Out.Append(reinterpret_cast<const uint8*>(&E),sizeof(E)); }
        else if constexpr(std::is_same_v<T,bool>) { const uint8 B=V?1:0;Out.Add(B); }
        else { static_assert(std::is_integral_v<T>);Out.Append(reinterpret_cast<const uint8*>(&V),sizeof(V)); }
    };
    const auto Range=[&Add](const auto& R)
    { Add(uint64(R.size()));for(const auto& V:R)Add(V); };
    const bool Tagged=std::any_of(Frame.Inputs.begin(),Frame.Inputs.end(),[](const auto& E){return bool(E.Requested);});
    Add(uint32(Tagged?3:1));Add(Stage);Add(Frame.Frame);Add(Frame.Registry->Version);
    Add(Frame.Registry->WorkerGeneration);Add(Frame.Registry->Terminal);Add(uint64(Frame.Inputs.size()));
    for (const FInstalledSessionInput& Entry:Frame.Inputs)
    {
        if(Entry.Actors.size()>64||bool(Entry.Snapshot)==bool(Entry.Requested))return false;
        Add(Entry.Session);Add(Entry.Epoch);Add(Entry.AlreadyAppliedAtInitialBoundary);
        Add(uint64(Entry.Actors.size()));for(const auto& Actor:Entry.Actors){Add(Actor.Id);Add(Actor.Generation);}
        Add(Entry.FilterBefore.has_value());if(Entry.FilterBefore)Range(*Entry.FilterBefore);
        if(Tagged)Add(uint8(Entry.Requested?1:0));
        if(Entry.Requested)
        {
            const auto& R=*Entry.Requested;
            if(!Entry.RequestedMapping||!R.Request||!R.Request->GetContract()||R.Request->GetState().Frame!=Frame.Frame)return false;
            const auto& Q=*R.Request;const auto& S=Q.GetState();
            Add(FRequestedInputFrame::Version);Add(S.Frame);Range(S.Values);Add(S.ActiveMask);Add(S.Reset);
            Add(Q.GetOrigin());Add(Q.GetAuthorityCause().has_value());if(Q.GetAuthorityCause())Add(*Q.GetAuthorityCause());
            Add(Q.GetOriginal().has_value());
            if(Q.GetOriginal())
            {
                const auto& F=*Q.GetOriginal();if(!F.IsValidFor(*Q.GetContract()))return false;const auto& D=F.GetData();
                Add(D.Version);Add(D.SourceSequence);Add(D.ConsumptionFrame);Add(D.Producer.Kind);Add(D.Producer.Id);
                Add(D.StreamEpoch.Value);Add(D.DeviceGeneration.Value);Range(D.Values);Add(D.ActiveMask);Add(D.Reset);
                Add(uint64(D.Transitions.size()));for(const auto& E:D.Transitions)
                {Add(E.Action);Add(E.State);Add(E.ValueAtTransition);Add(E.Order.Sequence);Add(E.Order.WithinSequence);}
            }
            Range(Q.GetContract()->GetFingerprint());Range(R.BindingFingerprint);Range(R.BeforeProcessing);Range(R.Applied);
            Add(R.Cutoff.Ordinal.Value);Add(R.Cutoff.SourceGeneration);Add(R.CancelDigitalMask);
            Entry.RequestedMapping->AppendCanonical([&](std::uint64_t V){Add(V);});continue;
        }
        if(Entry.RequestedMapping||!Entry.Snapshot->Input.GetContract()
            ||Entry.Snapshot->Input.GetData().ConsumptionFrame!=Frame.Frame
            ||!Entry.Snapshot->Input.IsValidFor(*Entry.Snapshot->Input.GetContract()))return false;
        const FOwnerInputSnapshot& Snapshot=*Entry.Snapshot;
        const auto& Data=Snapshot.Input.GetData();
        Add(Data.Version);Add(Data.SourceSequence);Add(Data.ConsumptionFrame);
        Add(Data.Producer.Kind);Add(Data.Producer.Id);Add(Data.StreamEpoch.Value);Add(Data.DeviceGeneration.Value);
        Range(Data.Values);Add(Data.ActiveMask);Add(Data.Reset);
        Add(uint64(Data.Transitions.size()));for(const auto& Transition:Data.Transitions)
        { Add(Transition.Action);Add(Transition.State);Add(Transition.ValueAtTransition);Add(Transition.Order.Sequence);Add(Transition.Order.WithinSequence); }
        Range(Snapshot.Input.GetContract()->GetFingerprint());Range(Snapshot.BindingFingerprint);
        Range(Snapshot.Requested);Range(Snapshot.Applied);Add(Snapshot.CutoffSequence);
    }
    Bytes=MoveTemp(Out);return true;
}
// The driver certifies absence only from its admitted immutable registry view.
// A null pointer alone and a stale non-null endpoint never prove coverage.
inline bool AppendFrameInputEndpointWitness(uint64 ExpectedFrame,EFrameInputWitnessStage Stage,
 const Input::V2::FRegistryFrame* Endpoint,bool bNoSessionsAdmitted,TArray<uint8>& Bytes)
{
 if(Stage!=EFrameInputWitnessStage::Installed && Stage!=EFrameInputWitnessStage::RegistryCompleted)return false;
 if(Endpoint)return !bNoSessionsAdmitted && Endpoint->Frame==ExpectedFrame && AppendFrameInputWitness(*Endpoint,Stage,Bytes);
 if(!bNoSessionsAdmitted)return false;
 AppendAbsentFrameInputWitness(ExpectedFrame,Stage,Bytes);return true;
}
}
#endif
