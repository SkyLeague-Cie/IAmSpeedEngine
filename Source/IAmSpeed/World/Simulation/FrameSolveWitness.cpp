#include "FrameSolveWitness.h"
#if !UE_BUILD_SHIPPING
#include "IAmSpeed/Base/EpisodeShapeFieldsForTesting.h"
#include "SimulationWorld.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "IAmSpeed/Components/ISpeedWheeledComponent.h"
#include "FrameFloatingPointContext.h"
#endif
#include "IAmSpeed/Components/ISpeedComponent.h"
namespace Speed
{
namespace { thread_local FFrameSolveWitnessScope* ActiveSolveWitness=nullptr; }
FFrameSolveWitnessScope::FFrameSolveWitnessScope(uint64 InFrame,TArray<uint8>& Raw,
    const FFramePointerRegistry& Registry,int32 MaximumRows)
    : Frame(InFrame),Bytes(Raw),PointerCapture(Raw,Registry),WheelRegistry1368(Registry),Limit(FMath::Clamp(MaximumRows,1,65536))
{ Previous=ActiveSolveWitness;ActiveSolveWitness=this; }
FFrameSolveWitnessScope::~FFrameSolveWitnessScope()
{ check(ActiveSolveWitness==this);ActiveSolveWitness=Previous; }
bool FFrameSolveWitnessScope::IsActive() { return ActiveSolveWitness!=nullptr; }
void FFrameSolveWitnessScope::Record(uint64 InFrame,int32 Iteration,int32 Order,
    uint64 ActorStableId,int32 ResolverIndex,uint64 PairKey,const SHitResult& Hit,
    bool bIgnored,int32 GenerationBefore,int32 GenerationAfter,const USSubBody* Resolver,bool bStaticRestingWorld) try
{
    auto* Scope=ActiveSolveWitness;if(!Scope)return;
    if(InFrame!=Scope->Frame || Scope->Rows>=Scope->Limit)
    { Scope->bOverflow=true;return; }
    auto& B=Scope->Bytes;
    using W=FEpisodeStateBytesForTesting;
 #if WITH_DEV_AUTOMATION_TESTS
    if(Scope->bRowBoundaries1534)Scope->RawRowBoundaries1534.Add(B.Num());
 #endif
    W::Add(B,uint32(1));W::Add(B,InFrame);W::Add(B,Scope->Rows++);++Scope->MainRows;
    W::Add(B,Iteration);W::Add(B,Order);W::Add(B,ActorStableId);W::Add(B,ResolverIndex);
    FFramePointerCaptureScope::ObserveSolverPairToken(B,PairKey,Resolver,Hit,bStaticRestingWorld);
    W::Add(B,PairKey);W::Add(B,Hit);W::Add(B,bIgnored);W::Add(B,GenerationBefore);W::Add(B,GenerationAfter);
}
catch(...)
{
    // Diagnostics must never turn an already-selected physical contact into
    // an aborted frame. Retain the partial raw stream and mark UNKNOWN.
    if(ActiveSolveWitness)ActiveSolveWitness->bOverflow=true;
}
namespace
{
void AddDynamicPair(TArray<uint8>& B,const ::FDynamicContactPair& P)
{
 using W=FEpisodeStateBytesForTesting;
 const USSubBody* A=P.BodyA.Get();const USSubBody* Other=P.BodyB.Get();
 W::Add(B,A);W::Add(B,P.BodyA.IsExplicitlyNull());W::Add(B,P.BodyA.IsStale(true,true));
 W::Add(B,Other);W::Add(B,P.BodyB.IsExplicitlyNull());W::Add(B,P.BodyB.IsStale(true,true));
 FFramePointerCaptureScope::ObserveDynamicPairToken(B,P.PairKey,A,Other);W::Add(B,P.PairKey);
 W::Add(B,P.LocalAnchorA);W::Add(B,P.LocalAnchorB);W::Add(B,P.LocalNormalB);
 W::Add(B,P.LastCOMA);W::Add(B,P.LastCOMB);
 W::Add(B,P.FirstSeenFrame);W::Add(B,P.LastSeenFrame);W::Add(B,P.LastSolvedFrame);
 W::Add(B,P.SupportedFrameCount);W::Add(B,P.ActiveContactFrameCount);
 W::Add(B,P.ReactionFrameCount);W::Add(B,P.FeatureTransitionCount);
 W::Add(B,P.AcquisitionNormalSpeed);W::Add(B,P.AccumulatedNormalImpulse);
 W::Add(B,P.AccumulatedNormalActivity);W::Add(B,P.AccumulatedNormalSupportActivity);
 W::Add(B,P.PeakNormalActivity);W::Add(B,P.PeakNormalSupportActivity);
 W::Add(B,P.MinimumSeparation);W::Add(B,P.MaximumSeparation);
 W::Add(B,P.bRollingManifoldReady);W::Add(B,P.bActiveContactLastSolve);
}
void AddDynamicBody(TArray<uint8>& B,const USSubBody* Body)
{
 using W=FEpisodeStateBytesForTesting;
 W::Add(B,Body);W::Add(B,Body!=nullptr);if(!Body)return;
 W::Add(B,Body->GetKinematicState());
 const ISpeedComponent* Parent=Body->GetParentComponent();
 W::Add(B,Parent!=nullptr);if(!Parent)return;
 W::Add(B,Parent->GetPublishedSimulationStableId());W::Add(B,Parent->GetKinematicState());
}
void AddDynamicContext(TArray<uint8>& B,const ::FFakePhysicsImpactContext& C)
{
 using W=FEpisodeStateBytesForTesting;
 W::Add(B,C.SelfParentKinematics);W::Add(B,C.OtherParentKinematics);
 W::Add(B,C.FakeImpulseScale);W::Add(B,C.bProjectFakeImpulseOntoContactPlane);
 W::Add(B,C.FakeRelativeVelocityBias);W::Add(B,C.FakeTangentialVelocityScale);
 W::Add(B,C.bUseContactPointRelativeVelocity);
}
}
uint64 FFrameSolveWitnessScope::BeginDynamicPass(uint64 InFrame,EFrameDynamicContactPass Kind,
 float Dt,uint64 FilterKey,bool bFiltered,bool bFirstSeen,int32 PairCount,
 const ::FDynamicContactPair* FilterPair) noexcept try
{
 auto* S=ActiveSolveWitness;if(!S)return 0;
 if(InFrame!=S->Frame || S->Rows>=S->Limit || S->NextDynamicPass==MAX_uint64)
 {S->bOverflow=true;return 0;}
 FDynamicPass Pass;Pass.Id=++S->NextDynamicPass;Pass.Kind=Kind;Pass.Dt=Dt;
 Pass.bFiltered=bFiltered;Pass.bFirstSeen=bFirstSeen;S->OpenDynamicPasses.Add(Pass);
 RecordDynamicEvent(Pass.Id,EFrameDynamicContactEvent::PassBegin,PairCount,FilterKey,FilterPair,
  FilterPair?FilterPair->BodyA.Get():nullptr,FilterPair?FilterPair->BodyB.Get():nullptr);
 return Pass.Id;
}
catch(...){if(ActiveSolveWitness)ActiveSolveWitness->bOverflow=true;return 0;}
uint64 FFrameSolveWitnessScope::CurrentDynamicPass() noexcept
{
 auto* S=ActiveSolveWitness;
 return S && !S->OpenDynamicPasses.IsEmpty()?S->OpenDynamicPasses.Last().Id:0;
}
void FFrameSolveWitnessScope::EndDynamicPass(uint64 Pass,int32 PairCount) noexcept try
{
 auto* S=ActiveSolveWitness;if(!S)return;
 if(!Pass || S->OpenDynamicPasses.IsEmpty() || S->OpenDynamicPasses.Last().Id!=Pass)
 {S->bOverflow=true;return;}
 RecordDynamicEvent(Pass,EFrameDynamicContactEvent::PassEnd,PairCount,0,nullptr,nullptr,nullptr,
  EFrameDynamicContactOutcome::Completed);
 S->OpenDynamicPasses.Pop(EAllowShrinking::No);
}
catch(...){if(ActiveSolveWitness)ActiveSolveWitness->bOverflow=true;}
void FFrameSolveWitnessScope::RecordDynamicEvent(uint64 Pass,EFrameDynamicContactEvent Event,
 int32 PairIndex,uint64 PairKey,const ::FDynamicContactPair* Pair,const USSubBody* BodyA,
 const USSubBody* BodyB,EFrameDynamicContactOutcome Outcome,bool bRolling,const FVector& Vector,
 const FVector& Point,float Scalar1,float Scalar2,const SHitResult* Hit,
 const ::FFakePhysicsImpactContext* Context) noexcept try
{
 auto* S=ActiveSolveWitness;if(!S)return;
 if(!Pass || S->OpenDynamicPasses.IsEmpty() || S->OpenDynamicPasses.Last().Id!=Pass || S->Rows>=S->Limit)
 {S->bOverflow=true;return;}
 const FDynamicPass& P=S->OpenDynamicPasses.Last();auto& B=S->Bytes;
 using W=FEpisodeStateBytesForTesting;
 #if WITH_DEV_AUTOMATION_TESTS
 if(S->bRowBoundaries1534)S->RawRowBoundaries1534.Add(B.Num());
 #endif
 W::Add(B,uint32(2));W::Add(B,S->Frame);W::Add(B,S->Rows++);
 W::Add(B,Pass);W::Add(B,uint8(P.Kind));W::Add(B,uint8(Event));W::Add(B,P.Dt);
 W::Add(B,P.bFiltered);W::Add(B,P.bFirstSeen);W::Add(B,PairIndex);
 FFramePointerCaptureScope::ObserveDynamicPairToken(B,PairKey,BodyA,BodyB);W::Add(B,PairKey);
 W::Add(B,uint8(Outcome));W::Add(B,bRolling);W::Add(B,Pair!=nullptr);
 if(Pair)
 {
  if(Pair->BodyA.IsStale(true,true) || Pair->BodyB.IsStale(true,true))S->bOverflow=true;
  AddDynamicPair(B,*Pair);
 }
 AddDynamicBody(B,BodyA);AddDynamicBody(B,BodyB);
 W::Add(B,Vector);W::Add(B,Point);W::Add(B,Scalar1);W::Add(B,Scalar2);
 W::Add(B,Hit!=nullptr);if(Hit)W::Add(B,*Hit);
 W::Add(B,Context!=nullptr);if(Context)AddDynamicContext(B,*Context);
}
catch(...){if(ActiveSolveWitness)ActiveSolveWitness->bOverflow=true;}

#if WITH_DEV_AUTOMATION_TESTS
bool FFrameSolveWitnessScope::IsWheelManifoldActive1368() noexcept
{return ActiveSolveWitness&&ActiveSolveWitness->bCaptureWheelManifold1368;}
void FFrameSolveWitnessScope::RecordWheelManifold1368(uint8 Event,const ISpeedComponent* Owner,
 int32 DetailA,int32 DetailB,TConstArrayView<SWheelGroundContact> Contacts,
 TConstArrayView<int32> Order,TConstArrayView<float> Scalars,
 TConstArrayView<double> DoubleScalars,TConstArrayView<FVector> Vectors) noexcept try
{
 auto* Scope=ActiveSolveWitness;if(!Scope||!Scope->bCaptureWheelManifold1368)return;
 const auto Numeric=CaptureFrameFloatingPointContext();FScopedFrameFloatingPointContext Guard(Numeric);
 if(!Guard.IsApplied()){Scope->bOverflow=true;return;}
 const bool KnownEvent=Event<=6||Event==8||Event==9||Event==10||Event==11||Event==12||Event==13||Event==254||Event==255;
 if(!KnownEvent||!Owner||Scope->WheelRows1368>=Scope->Limit||Contacts.Num()>32||Order.Num()>32
  ||Scalars.Num()>32||DoubleScalars.Num()>32||Vectors.Num()>8)
 {Scope->bOverflow=true;return;}
 if(Event==0)
 {
  if(Scope->OpenWheelOwners1368.Num()>=32){Scope->bOverflow=true;return;}
  Scope->OpenWheelOwners1368.Add(Owner);
 }
 else if(Scope->OpenWheelOwners1368.IsEmpty()||Scope->OpenWheelOwners1368.Last()!=Owner)
 {Scope->bOverflow=true;return;}
 // A separate pointer capture owns this row. Main/dynamic bytes and sequence
 // are not touched. No additional component/vehicle kinematic getter is called.
 TArray<uint8> Raw,Canonical;FFramePointerCaptureScope Pointers(Raw,Scope->WheelRegistry1368);
 using W=FEpisodeStateBytesForTesting;
 W::Add(Raw,uint32(3));W::Add(Raw,Scope->Frame);W::Add(Raw,Scope->WheelRows1368++);
 W::Add(Raw,Scope->Rows);W::Add(Raw,Event);W::Add(Raw,Owner);
 W::Add(Raw,DetailA);W::Add(Raw,DetailB);W::Add(Raw,Contacts.Num());
 for(const auto& C:Contacts)
 {
  if(C.SurfaceComponent.IsStale(true,true)){Scope->bOverflow=true;break;}
  W::Add(Raw,C.Wheel);W::Add(Raw,C.SurfaceComponent.GetEvenIfUnreachable());W::Add(Raw,C.SurfaceComponent.IsExplicitlyNull());
  W::Add(Raw,C.SurfacePoint);W::Add(Raw,C.SurfaceFaceIndex);W::Add(Raw,C.SurfaceSourceId);W::Add(Raw,C.SurfaceId);W::Add(Raw,C.SurfaceFeatureId);
  W::Add(Raw,C.WorldPos);W::Add(Raw,C.Normal);W::Add(Raw,C.r);W::Add(Raw,C.vN);W::Add(Raw,C.InvMassEff);
  W::Add(Raw,C.SpringDisplacement);W::Add(Raw,C.bVelocityLocked);W::Add(Raw,C.bNewContact);W::Add(Raw,C.bAtBumpStop);
 }
 W::Add(Raw,Order.Num());for(const auto V:Order)W::Add(Raw,V);
 W::Add(Raw,Scalars.Num());for(const auto V:Scalars)W::Add(Raw,V);
 W::Add(Raw,DoubleScalars.Num());for(const auto V:DoubleScalars)W::Add(Raw,V);
 W::Add(Raw,Vectors.Num());for(const auto& V:Vectors)W::Add(Raw,V);
 W::Add(Scope->WheelRaw1368,uint32(0x334d5749));W::Add(Scope->WheelRaw1368,uint32(Raw.Num()));Scope->WheelRaw1368.Append(Raw);
 if(!Scope->bOverflow&&Pointers.MakeCanonical(Canonical))
 {
  W::Add(Scope->WheelCanonical1368,uint32(0x334d5749));W::Add(Scope->WheelCanonical1368,uint32(Canonical.Num()));Scope->WheelCanonical1368.Append(Canonical);
  ++Scope->WheelCanonicalRows1368;
 }
 else Scope->bOverflow=true;
 if(Event==255)Scope->OpenWheelOwners1368.Pop(EAllowShrinking::No);
 if(!Guard.Restore())Scope->bOverflow=true;
}
catch(...){if(ActiveSolveWitness)ActiveSolveWitness->bOverflow=true;}
#endif

}
#endif
