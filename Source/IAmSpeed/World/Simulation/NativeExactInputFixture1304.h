#pragma once
#include "IAmSpeed/Input/InputSessionRegistry.h"
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
namespace Speed::Testing1304
{
inline bool SameExactInput(const Input::V2::FInputFrame& A,const Input::V2::FInputFrame& B)
{
 const auto& X=A.GetData();const auto& Y=B.GetData();
 if(!A.GetContract()||!B.GetContract()||A.GetContract()->GetFingerprint()!=B.GetContract()->GetFingerprint())return false;
 if(X.Version!=Y.Version||X.SourceSequence!=Y.SourceSequence||X.ConsumptionFrame!=Y.ConsumptionFrame
  ||X.Producer.Id!=Y.Producer.Id||X.Producer.Kind!=Y.Producer.Kind||X.StreamEpoch.Value!=Y.StreamEpoch.Value
  ||X.DeviceGeneration.Value!=Y.DeviceGeneration.Value||X.Values!=Y.Values||X.ActiveMask!=Y.ActiveMask
  ||X.Reset!=Y.Reset||X.Transitions.size()!=Y.Transitions.size())return false;
 for(std::size_t I=0;I<X.Transitions.size();++I)
 {
  const auto& L=X.Transitions[I];const auto& R=Y.Transitions[I];
  if(L.Action!=R.Action||L.State!=R.State||L.ValueAtTransition!=R.ValueAtTransition
   ||L.Order.Sequence!=R.Order.Sequence||L.Order.WithinSequence!=R.Order.WithinSequence)return false;
 }
 return true;
}
inline void AddExactInput(TArray<uint8>& O,const Input::V2::FInputFrame& Input,const Input::FActionValues& Applied)
{
 using W=FEpisodeStateBytesForTesting;const auto& X=Input.GetData();
 const auto& Fingerprint=Input.GetContract()->GetFingerprint();W::Add(O,uint32(Fingerprint.size()));for(const auto B:Fingerprint)W::Add(O,B);
 W::Add(O,X.Version);W::Add(O,X.SourceSequence);W::Add(O,X.ConsumptionFrame);
 W::Add(O,X.Producer.Id);W::Add(O,X.Producer.Kind);W::Add(O,X.StreamEpoch.Value);W::Add(O,X.DeviceGeneration.Value);
 W::Add(O,X.ActiveMask);W::Add(O,X.Reset);for(const auto V:X.Values)W::Add(O,V);
 W::Add(O,uint32(X.Transitions.size()));
 for(const auto& E:X.Transitions){W::Add(O,E.Action);W::Add(O,E.State);W::Add(O,E.ValueAtTransition);W::Add(O,E.Order.Sequence);W::Add(O,E.Order.WithinSequence);}
 for(const auto V:Applied)W::Add(O,V);
}
}
