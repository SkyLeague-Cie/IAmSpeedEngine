#include "IAmSpeed/Netcode/presentation-client/PresentationClientGate.h"
#include <iostream>
#include <stdexcept>
#include <limits>
namespace P=Speed::Netcode::PresentationClient;
namespace C=Speed::Netcode::ContractOffer;
namespace V=Speed::Input::V2;
static unsigned Checks=0;
#define CHECK(X) do{++Checks;if(!(X))throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #X);}while(false)
static P::FTuple Tuple(V::EExactRemoteAdmission Mode=V::EExactRemoteAdmission::OriginalDeviceExact)
{
 V::FInputActionContractDescription D;D.Revision.Value=1;D.Actions=V::FInputActionContract::BaseActions();
 D.Physical={{0,V::EPhysicalDestination::Throttle},{1,V::EPhysicalDestination::Brake},{2,V::EPhysicalDestination::Steering}};
 auto Offer=C::Encode(D,{},Mode);CHECK(Offer);return {11,12,13,14,15,16,*Offer};
}
static P::FGate Ready(){P::FGate G(true,7,20);CHECK(G.Observe(1,false,false,0,0,false)==P::EState::Pending);CHECK(G.SuppressesPhysicalBootstrap());return G;}
static void ValidAndSticky()
{
 auto T=Tuple();auto G=Ready();CHECK(G.Admit(2,7,101,102,T,T,true,true));CHECK(G.GetState()==P::EState::Admitted&&G.Matches(T)&&G.SuppressesPhysicalBootstrap());
 CHECK(G.Observe(100,true,false,101,102,false)==P::EState::Admitted); // Pending timeout is not session expiry.
 CHECK(G.Observe(101,false,false,0,0,false)==P::EState::Failed);CHECK(!G.Matches(T));
 CHECK(!G.Admit(102,7,101,102,T,T,true,true));CHECK(G.SuppressesPhysicalBootstrap());
 P::FGate Legacy(false,0,0);CHECK(Legacy.Observe(999,false,true,0,0,true)==P::EState::Unselected);CHECK(!Legacy.SuppressesPhysicalBootstrap());
}
static void WholeTupleAndHostApproval()
{
 const auto T=Tuple();
 for(unsigned Field=0;Field<6;++Field){auto Bad=T;switch(Field){case 0:++Bad.Session;break;case 1:++Bad.Epoch;break;case 2:++Bad.Controller;break;case 3:++Bad.Producer;break;case 4:++Bad.Actor;break;default:++Bad.Incarnation;break;}auto G=Ready();CHECK(!G.Admit(2,7,101,102,T,Bad,true,true));CHECK(G.GetState()==P::EState::Failed);}
 const auto Processed=Tuple(V::EExactRemoteAdmission::OriginalDeviceAuthoritativeProcessing);
 CHECK(C::Decode(T.CanonicalOffer.data(),T.CanonicalOffer.size())->Contract->GetFingerprint()==C::Decode(Processed.CanonicalOffer.data(),Processed.CanonicalOffer.size())->Contract->GetFingerprint());
 {auto G=Ready();CHECK(!G.Admit(2,7,101,102,T,Processed,true,true));CHECK(G.GetState()==P::EState::Failed);}
 for(unsigned Field=0;Field<6;++Field){auto Bad=T;switch(Field){case 0:Bad.Session=0;break;case 1:Bad.Epoch=0;break;case 2:Bad.Controller=0;break;case 3:Bad.Producer=0;break;case 4:Bad.Actor=0;break;default:Bad.Incarnation=0;break;}CHECK(!Bad.Valid());auto G=Ready();CHECK(!G.Admit(2,7,101,102,Bad,Bad,true,true));}
 for(std::size_t N=0;N<T.CanonicalOffer.size();++N){auto Bad=T;Bad.CanonicalOffer.resize(N);CHECK(!Bad.Valid());}
 auto Bad=T;Bad.CanonicalOffer.push_back(0);CHECK(!Bad.Valid());auto G=Ready();CHECK(!G.Admit(2,7,101,102,T,Bad,true,true));
}
static void RoleGenerationPristineAndOriginalConnection()
{
 const auto T=Tuple();
 for(unsigned Failure=0;Failure<6;++Failure){auto G=Ready();CHECK(!G.Admit(2,Failure==0?8:7,Failure==1?0:101,Failure==2?0:102,T,T,Failure!=3,Failure!=4&&Failure!=5));CHECK(G.GetState()==P::EState::Failed);}
 for(unsigned Failure=0;Failure<5;++Failure){auto G=Ready();CHECK(G.Observe(2,false,false,101,102,false)==P::EState::Pending);const auto State=G.Observe(Failure==4?20:3,false,Failure==2,Failure==0?103:101,Failure==1?104:102,Failure==3);CHECK(State==P::EState::Failed);CHECK(!G.Admit(4,7,101,102,T,T,true,true));}
 P::FGate BadGeneration(true,0,20);CHECK(BadGeneration.GetState()==P::EState::Failed);
 P::FGate BadDeadline(true,7,std::numeric_limits<double>::infinity());CHECK(BadDeadline.GetState()==P::EState::Failed);
 auto G=Ready();CHECK(G.Observe(std::numeric_limits<double>::quiet_NaN(),false,false,0,0,false)==P::EState::Failed);
}
int main(){try{ValidAndSticky();WholeTupleAndHostApproval();RoleGenerationPristineAndOriginalConnection();std::cout<<"PASS checks="<<Checks<<" scope=pure_presentation_gate_only_no_UE_map_or_socket_qualification\n";return 0;}catch(const std::exception& E){std::cerr<<"FAIL "<<E.what()<<'\n';return 1;}}
