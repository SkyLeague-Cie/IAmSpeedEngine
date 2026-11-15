// Included after actual frame/owner-probe/integration values and geometry helpers.
namespace UncoveredGeneralFloorPoseDetail
{
void Log(const FKinematicState& before,const FKinematicState& candidate,const char* stage)
{
    const auto values=[&](const char* label,const FKinematicState& v)
    {
        UE_LOG(LogTemp,Display,TEXT("[NativeGeneralFloorUncoveredPoseCandidate] Stage=%hs Values=%hs P=(%.17g,%.17g,%.17g) Q=(%.17g,%.17g,%.17g,%.17g) V=(%.17g,%.17g,%.17g) A=(%.17g,%.17g,%.17g) W=(%.17g,%.17g,%.17g) Alpha=(%.17g,%.17g,%.17g)"),
            stage,label,v.Location.X,v.Location.Y,v.Location.Z,v.Rotation.X,v.Rotation.Y,v.Rotation.Z,v.Rotation.W,
            v.Velocity.X,v.Velocity.Y,v.Velocity.Z,v.Acceleration.X,v.Acceleration.Y,v.Acceleration.Z,
            v.AngularVelocity.X,v.AngularVelocity.Y,v.AngularVelocity.Z,v.AngularAcceleration.X,v.AngularAcceleration.Y,v.AngularAcceleration.Z);
    };
    values("ActualBefore",before);values("RealSetterRHS",candidate);
}
}
thread_local FNativeOwnerPoseMutationScope* NativeOwnerPoseMutation=nullptr;
#include "PreparedNativeFinalWheelSupport.inl"
struct FNativeOwnerPoseMutationScope::FState
{
    FStepScope* Scope=nullptr;FOrdinaryIntegrationState* Transaction=nullptr;
    FPreparedWorldSnapshotPtr Snapshot;const ISpeedComponent* Owner=nullptr;
    const FOrdinaryOwnerProbePhaseScope* Phase=nullptr;uint64 PhaseOrdinal=0;
    FStepIdentity Step;int32 OwnerIndex=INDEX_NONE;uint64 Ordinal=0,PoseVersion=0;
    bool Primitive=false,Completed=false,Planar=false,FinalWheelSupport=false;FKinematicState Before,Candidate;
    TArray<FGeneralBoxTransportBinding> Bindings;
    TArray<FNativeBoxPrimitiveGeometryValuesPtr> PrimitiveProofs;
    TArray<FNativeBoxPoseCorrectionGeometryValuesPtr> CorrectionProofs;
};

FNativeOwnerPoseMutationScope::FNativeOwnerPoseMutationScope(const ISpeedComponent& owner,
    const FKinematicState& actualBefore,const FKinematicState& candidate)
{
    if(!Requested()||!StepScope||!StepScope->Active||!StepScope->Window||
        StepScope->Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame||
        !StepScope->GeneralBoxOwners.Contains(&owner))return;
    if(FNativePlanarResponseCommitScope::BeforeOwnerSetter(owner,actualBefore,candidate))
    {
        State=MakeUnique<FState>();State->Planar=true;State->Owner=&owner;State->Candidate=candidate;return;
    }
    // These arguments are the real setter's own stored field and RHS. This
    // early values-only test adds no owner/UObject lookup for force changes.
    if(GeneralBoxTransportDetail::Pose(actualBefore,candidate))return;
    if(FNativeFinalWheelSupportScope::BeginSetter(owner,actualBefore,candidate))
    {
        State=MakeUnique<FState>();State->FinalWheelSupport=true;State->Owner=&owner;return;
    }
    auto* scope=StepScope;
    if(!scope->OrdinaryIntegration||scope->OrdinaryIntegration->ActiveIndex==INDEX_NONE)
    {
        UncoveredGeneralFloorPoseDetail::Log(actualBefore,candidate,"outside_covered_integration");
        Refuse("General Box pose assignment has no active covered integration");
    }
    scope->RequireOrdinaryIntegrationContext(true);auto& tx=*scope->OrdinaryIntegration;
    if(!tx.Owners.IsValidIndex(tx.ActiveIndex)||tx.Owners[tx.ActiveIndex].Owner!=&owner)
        Refuse("General Box actual pose setter owner changed");
    auto& row=tx.Owners[tx.ActiveIndex];
    if(row.GeneralBindings.IsEmpty())return; // Other closed native shape routes unchanged.
    if(NativeOwnerPoseMutation||row.OpenPoseSetter||row.PoseAssignmentsBegun!=row.PoseAssignmentsCompleted||
        row.PoseAssignmentsBegun==MAX_uint64||!FiniteState(actualBefore)||!FiniteState(candidate)||
        !SameKinematicBits(actualBefore,owner.GetKinematicState())||
        row.GeneralProofs.Num()!=row.GeneralBindings.Num())Refuse("General Box pose setter entry lacks exact current coverage");
    auto state=MakeUnique<FState>();state->Scope=scope;state->Transaction=&tx;state->Snapshot=scope->Snapshot;
    state->Owner=&owner;state->OwnerIndex=tx.ActiveIndex;state->Before=actualBefore;state->Candidate=candidate;
    state->Bindings=row.GeneralBindings;state->PrimitiveProofs=row.GeneralProofs;state->PoseVersion=row.PoseVersion;
    state->Ordinal=row.PoseAssignmentsBegun+1;state->Phase=OwnerProbePhase;
    if(!ReadCurrentStep(state->Step))Refuse("General Box pose setter lost actual Step");
    const uint64 priorCompleted=row.PoseAssignmentsCompleted;
    for(const auto& binding:row.GeneralBindings)
        if(!binding.Request||binding.Request->GetSnapshot()!=scope->Snapshot||!binding.HasExactlyOneKind()||!GeneralBoxTransportDetail::Configuration(binding.Request->GetValues()))
            Refuse("General Box pose setter configuration changed");
    state->Primitive=row.GeneralOpen&&!row.GeneralReturned;
    if(state->Primitive)
    {
        if(OwnerProbePhase||row.Stages.IsEmpty()||row.Stages.Last().Stage!=ENativeIntegrationStage::PrimitiveEnter||
            !SameKinematicBits(row.Stages.Last().OwnerState,actualBefore)||!SameKinematicBits(row.GeneralExpected,candidate))
            Refuse("General Box actual primitive setter differs from its pure expected return");
        // Keep GeneralExpected and the original primitive proofs unchanged.
    }
    else
    {
        if(!OwnerProbePhase||!OwnerProbePhase->Check()||!OwnerProbePhase->State||
            OwnerProbePhase->State->Owner!=&owner||OwnerProbePhase->State->Snapshot!=scope->Snapshot||
            (OwnerProbePhase->State->Phase!=ENativeOwnerProbePhase::PostIntegrate&&
             OwnerProbePhase->State->Phase!=ENativeOwnerProbePhase::IntegrationProjection)||
            !GeneralBoxTransportDetail::Pose(actualBefore,row.GeneralExpected)||row.PoseVersion==MAX_uint64)
            Refuse("General Box pose correction lacks its real Post/Projection baseline");
        for(const auto& binding:row.GeneralBindings)
            if(binding.FloorMotion||(binding.Released&&(!binding.Released->IsEstablishedBoxFloorClearValues()||
                OwnerProbePhase->State->Phase!=ENativeOwnerProbePhase::IntegrationProjection)))
            {
                UncoveredGeneralFloorPoseDetail::Log(actualBefore,candidate,"post_or_projection_without_floor_corridor");
                Refuse("FLOOR actual pose correction requires its separate finite corridor proof");
            }
        state->PhaseOrdinal=OwnerProbePhase->State->Ordinal;
        const auto service=scope->Identity.Provider->GetPreparedQueryService();
        if(!service||service->GetPreparedSnapshot()!=scope->Snapshot)Refuse("General Box correction service differs");
        for(const auto& binding:row.GeneralBindings)
        {
            FNativeBoxPoseCorrectionGeometryValuesPtr proof;
            const auto established=binding.Released?binding.Released->GetReleasedFloorDescriptorForInspection():FPreparedNativeSolidGeometryPtr{};
            const auto curved=binding.Prediction&&binding.Prediction->IsCurvedArrivalPrefix()?
                binding.Prediction:FNativeCanonicalBoxSweepClearanceValuesPtr{};
            const auto curvedActual=binding.Prediction&&binding.Prediction->IsCurvedActualMotionClear()?
                binding.Prediction:FNativeCanonicalBoxSweepClearanceValuesPtr{};
            const bool completeClear=!established&&!curved&&binding.Prediction&&binding.Prediction->RequiresGeneralBoxTransport()&&
                (curvedActual||binding.Request->GetValues().Trace.IgnoredComponents.Num()==1);
            const bool proved=completeClear?
                service->DescribeNativeBoxPoseCorrectionWithCompleteClear(binding.Request,actualBefore,candidate,proof,binding.Prediction):
                service->DescribeNativeBoxPoseCorrectionGeometryValues(binding.Request->GetValues(),actualBefore,candidate,proof,established,curved);
            if(!proved||!proof||!proof->Matches(scope->Snapshot,binding.Request->GetValues(),actualBefore,candidate)||
                !proof->MatchesEstablishedFloorBinding(established)||!proof->MatchesCurvedArrivalBinding(curved)||
                !proof->MatchesCurvedActualBinding(curvedActual))
            {
                if(binding.Prediction&&binding.Prediction->IsCurvedArrivalPrefix())
                {
                    UE_LOG(LogTemp,Display,TEXT("[CurvedBoxCorrectionRefused] Frame=%llu Body=%d Phase=%u SameRotation=%d"),
                        state->Step.Frame,binding.Request->GetValues().BodyIndex,
                        uint32(OwnerProbePhase->State->Phase),int32(SameQuat(actualBefore.Rotation,candidate.Rotation)));
                    UncoveredGeneralFloorPoseDetail::Log(actualBefore,candidate,"curved_box_post_corridor_refused");
                }
                static const auto* trace=IConsoleManager::Get().FindTConsoleVariableDataInt(TEXT("p.IAmSpeed.AnalyticWorld.OrdinaryCurvedBoxTrace"));
                if(trace&&trace->GetValueOnAnyThread()==1)
                {
                    const auto& request=binding.Request->GetValues();
                    UE_LOG(LogTemp,Display,TEXT("[GeneralBoxCorridorRefused] Frame=%llu Serial=%llu Body=%d Phase=%u Proved=%d HasProof=%d FloorMotion=%d Released=%d Established=%d Prediction=%d CurvedArrival=%d CurvedWhole=%d CurvedActual=%d CompleteClearAdmission=%d IgnoredSources=%d IgnoredComponents=%d FirstIgnoredComponent=%u SameRotation=%d"),
                        state->Step.Frame,state->Step.QuerySerial,request.BodyIndex,uint32(OwnerProbePhase->State->Phase),
                        int32(proved),int32(bool(proof)),int32(bool(binding.FloorMotion)),int32(bool(binding.Released)),int32(bool(established)),
                        int32(bool(binding.Prediction)),int32(bool(curved)),int32(binding.Prediction&&binding.Prediction->IsCurvedWholeClear()),
                        int32(binding.Prediction&&binding.Prediction->IsCurvedActualMotionClear()),int32(completeClear),
                        request.Trace.IgnoredSourceObjects.Num(),request.Trace.IgnoredComponents.Num(),
                        request.Trace.IgnoredComponents.IsEmpty()?0u:request.Trace.IgnoredComponents[0],
                        int32(SameQuat(actualBefore.Rotation,candidate.Rotation)));
                    if(!curved)UncoveredGeneralFloorPoseDetail::Log(actualBefore,candidate,"general_box_corridor_refused");
                }
                Refuse("General Box actual pose correction corridor is not strictly clear of the complete static world");
            }
            state->CorrectionProofs.Add(MoveTemp(proof));
        }
    }
    // Geometry is pure, but still rejoin actual values/configuration and Step
    // before publishing an open setter. No incomplete constructor can leave
    // a dangling open ticket or manufacture a native completion.
    scope->RequireOrdinaryIntegrationContext(true);FStepIdentity now;
    if(StepScope!=scope||scope->OrdinaryIntegration.Get()!=&tx||tx.ActiveIndex!=state->OwnerIndex||
        scope->Snapshot!=state->Snapshot||OwnerProbePhase!=state->Phase||!ReadCurrentStep(now)||
        !OrdinaryWorldDetail::Epoch(state->Step,now)||state->Step.QuerySerial!=now.QuerySerial||
        row.OpenPoseSetter||NativeOwnerPoseMutation||row.PoseAssignmentsBegun+1!=state->Ordinal||
        row.PoseAssignmentsCompleted!=priorCompleted||row.PoseVersion!=state->PoseVersion||
        !SameKinematicBits(actualBefore,owner.GetKinematicState())||
        row.GeneralBindings.Num()!=state->Bindings.Num()||row.GeneralProofs.Num()!=state->PrimitiveProofs.Num())
        Refuse("General Box pose setter changed during geometric qualification");
    for(int32 i=0;i<state->Bindings.Num();++i)
        if(row.GeneralBindings[i].Request!=state->Bindings[i].Request||row.GeneralBindings[i].Prediction!=state->Bindings[i].Prediction||row.GeneralBindings[i].Released!=state->Bindings[i].Released||row.GeneralBindings[i].FloorMotion!=state->Bindings[i].FloorMotion||
            !SameFloat(row.GeneralBindings[i].FloorMotionWorldSeconds,state->Bindings[i].FloorMotionWorldSeconds)||
            row.GeneralProofs[i]!=state->PrimitiveProofs[i]||!GeneralBoxTransportDetail::Configuration(state->Bindings[i].Request->GetValues()))
            Refuse("General Box pose setter retained binding changed");
    State=MoveTemp(state);row.PoseAssignmentsBegun=State->Ordinal;row.OpenPoseSetter=this;NativeOwnerPoseMutation=this;
}

FOrdinaryIntegrationOwnerValues& FNativeOwnerPoseMutationScope::Require() const
{
    if(!State||State->Completed||StepScope!=State->Scope||NativeOwnerPoseMutation!=this)
        Refuse("General Box pose setter completion identity changed");
    auto* scope=State->Scope;scope->RequireOrdinaryIntegrationContext(true);
    if(scope->OrdinaryIntegration.Get()!=State->Transaction||scope->Snapshot!=State->Snapshot||
        !State->Transaction->Owners.IsValidIndex(State->OwnerIndex)||State->Transaction->ActiveIndex!=State->OwnerIndex||
        OwnerProbePhase!=State->Phase)Refuse("General Box pose setter transaction changed");
    auto& row=State->Transaction->Owners[State->OwnerIndex];FStepIdentity now;
    if(row.Owner!=State->Owner||row.OpenPoseSetter!=this||row.PoseAssignmentsBegun!=State->Ordinal||
        row.PoseAssignmentsCompleted+1!=State->Ordinal||row.PoseVersion!=State->PoseVersion||!ReadCurrentStep(now)||
        !OrdinaryWorldDetail::Epoch(State->Step,now)||State->Step.QuerySerial!=now.QuerySerial||
        row.GeneralBindings.Num()!=State->Bindings.Num()||row.GeneralProofs.Num()!=State->PrimitiveProofs.Num()||
        (State->Phase&&(!State->Phase->State||State->Phase->State->Ordinal!=State->PhaseOrdinal)))
        Refuse("General Box pose setter ordinal/Step/phase changed");
    for(int32 i=0;i<State->Bindings.Num();++i)
        if(row.GeneralBindings[i].Request!=State->Bindings[i].Request||row.GeneralBindings[i].Prediction!=State->Bindings[i].Prediction||row.GeneralBindings[i].Released!=State->Bindings[i].Released||row.GeneralBindings[i].FloorMotion!=State->Bindings[i].FloorMotion||
            !SameFloat(row.GeneralBindings[i].FloorMotionWorldSeconds,State->Bindings[i].FloorMotionWorldSeconds)||
            row.GeneralProofs[i]!=State->PrimitiveProofs[i]||!GeneralBoxTransportDetail::Configuration(State->Bindings[i].Request->GetValues()))
            Refuse("General Box actual pose setter changed intrinsic configuration");
    return row;
}
void FNativeOwnerPoseMutationScope::Returned()
{
    if(!State)return;
    if(State->Planar)
    {
        if(!FNativePlanarResponseCommitScope::AfterOwnerSetter(*State->Owner,State->Owner->GetKinematicState()))
            Refuse("Planar actual setter return lost its lexical transaction");
        State->Completed=true;return;
    }
    if(State->FinalWheelSupport)
    {
        FNativeFinalWheelSupportScope::EndSetter(*State->Owner,State->Owner->GetKinematicState());
        State->Completed=true;return;
    }
    auto& row=Require();
    if(!SameKinematicBits(State->Candidate,State->Owner->GetKinematicState()))
        Refuse("General Box pose setter did not return its exact copied candidate");
    if(!State->Primitive)
    {
        if(State->CorrectionProofs.Num()!=State->Bindings.Num()||!GeneralBoxTransportDetail::Pose(row.GeneralExpected,State->Before))
            Refuse("General Box pose correction completion lost original coverage");
        for(int32 i=0;i<State->Bindings.Num();++i)
        {
            const auto& binding=State->Bindings[i];
            const auto established=binding.Released?binding.Released->GetReleasedFloorDescriptorForInspection():FPreparedNativeSolidGeometryPtr{};
            const auto curved=binding.Prediction&&binding.Prediction->IsCurvedArrivalPrefix()?
                binding.Prediction:FNativeCanonicalBoxSweepClearanceValuesPtr{};
            if(!State->CorrectionProofs[i]||!State->CorrectionProofs[i]->Matches(State->Snapshot,
                binding.Request->GetValues(),State->Before,State->Candidate)||
                !State->CorrectionProofs[i]->MatchesEstablishedFloorBinding(established)||
                !State->CorrectionProofs[i]->MatchesCurvedArrivalBinding(curved))
                Refuse("General Box pose correction proof differs at actual setter return");
        }
        row.LatestPoseCorrections=State->CorrectionProofs;
        row.GeneralExpected.Location=State->Candidate.Location;row.GeneralExpected.Rotation=State->Candidate.Rotation;
        ++row.PoseVersion; // Real sync must observe this new version later.
    }
    ++row.PoseAssignmentsCompleted;row.OpenPoseSetter=nullptr;State->Completed=true;
}
FNativeOwnerPoseMutationScope::~FNativeOwnerPoseMutationScope() noexcept
{
    if(!State)return;
    if(State->Planar)
    {if(!State->Completed)RecordPreparedPoison("planar_native_setter_not_completed");return;}
    if(State->FinalWheelSupport)
    {if(!State->Completed)RecordPreparedPoison("case585_final_wheel_support_setter_not_completed");return;}
    if(!State->Completed)RecordPreparedPoison("native_owner_pose_setter_not_completed");
    if(NativeOwnerPoseMutation==this)NativeOwnerPoseMutation=nullptr;
    else RecordPreparedPoison("native_owner_pose_setter_scope_identity");
    if(!State->Completed&&State->Transaction&&State->Transaction->Owners.IsValidIndex(State->OwnerIndex))
    {
        auto& row=State->Transaction->Owners[State->OwnerIndex];
        if(row.OpenPoseSetter==this)row.OpenPoseSetter=nullptr;
        // Preserve the unbalanced begun/completed counters and sticky poison.
    }
}
