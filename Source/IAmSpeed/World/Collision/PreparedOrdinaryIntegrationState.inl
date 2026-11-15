struct FGeneralBoxTransportBinding
{
    FNativeSolidStaticRequestPtr Request;FNativeCanonicalBoxSweepClearanceValuesPtr Prediction;
    TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> Released,FloorMotion;
    float FloorMotionWorldSeconds=0;
    bool HasExactlyOneKind()const
    {
        if(!Request)return false;
        if(FloorMotion)return !Prediction&&!Released&&
            (FloorMotion->IsFirstGeneralBoxFloorContactValues()||FloorMotion->IsWholeGeneralBoxFloorClearValues())&&
            FloorMotion->GetRequestForInspection()==Request&&FMath::IsFinite(FloorMotionWorldSeconds)&&FloorMotionWorldSeconds>0&&
            FloorMotionWorldSeconds<=FloorMotion->GetGeneralFloorMotionMaximumAdvanceSeconds();
        return C::KinematicCenterDetail::Bits(FloorMotionWorldSeconds)==0&&
            ((Prediction&&!Released&&Prediction->RequiresGeneralBoxTransport())||
             (!Prediction&&Released&&Released->IsReleasedBoxFloorMotionValues()&&Released->GetRequestForInspection()==Request));
    }
};
struct FOrdinaryIntegrationStageValues
{
    ENativeIntegrationStage Stage=ENativeIntegrationStage::PrimitiveEnter;float Delta=0;int32 Index=INDEX_NONE,Count=0;
    bool Projected=false,Captured=false;FKinematicState OwnerState;FStepIdentity Step;
};
struct FOrdinaryIntegrationOriginValues
{
    FKinematicState ActualOrigin,OwnerState;TArray<FKinematicState> Children;FStepIdentity Step;bool InExactPostProbe=false,InExactIntegrationProjection=false,Captured=false;
};
enum class EOrdinaryPrimitiveValuesStatus:uint8 {Unknown,OutsideTransport,ValuesMatch,ValuesDiffer};
struct FOrdinaryPrimitiveComparisonValues
{
    int32 EnterStage=INDEX_NONE,ReturnStage=INDEX_NONE;FKinematicState ExpectedAfter;bool Equal=false;
};
struct FOrdinaryIntegrationOwnerValues
{
    const ISpeedComponent* Owner=nullptr;int32 Index=INDEX_NONE;bool Opened=false,Called=false,Returned=false,Closed=false;
    bool CompactStateCapture=false,RecycledStateBuffers=false;
    FOrdinaryConsumedStateValues Before,After;
    TArray<FNativeIntegrationPathValues> Paths;TArray<FOrdinaryIntegrationStageValues> Stages;
    TArray<FOrdinaryIntegrationOriginValues> ChildSync;
    int32 RestorationAttempts=0,RestorationReturns=0;
    EOrdinaryPrimitiveValuesStatus PrimitiveStatus=EOrdinaryPrimitiveValuesStatus::Unknown;
    TArray<FGeneralBoxTransportBinding> GeneralBindings;TArray<FNativeBoxPrimitiveGeometryValuesPtr> GeneralProofs;
    FKinematicState GeneralExpected;int32 GeneralNext=0,GeneralCount=0,GeneralSyncs=0;bool GeneralOpen=false,GeneralReturned=false;
    TArray<FNativeBoxPoseCorrectionGeometryValuesPtr> LatestPoseCorrections;
    const FNativeOwnerPoseMutationScope* OpenPoseSetter=nullptr;
    uint64 PoseAssignmentsBegun=0,PoseAssignmentsCompleted=0,PoseVersion=0,SyncVersion=0;
    TArray<FOrdinaryPrimitiveComparisonValues> PrimitiveComparisons; // values only, no advance/effect grant
    // Absence of an inherited Path is explicit, not a guessed derived route.
};
struct FOrdinaryIntegrationState
{
    FStepIdentity EntryStep;FPreparedWorldSnapshotPtr Snapshot;
    bool HasActualSelection=false,SelectorJoined=false,Closed=false;
    TArray<SComponentTOI> Selected;float Seconds=0,Horizon=0,ActualBefore=0,ActualAfter=0;
    FOrdinaryConsumedStateValues Before,After;TArray<FOrdinaryIntegrationOwnerValues> Owners;
    int32 NextIndex=0,ActiveIndex=INDEX_NONE;
    bool Restoring=false,RecycleOwnerStateBuffers=false;
    // Raw observations only, no expected free-motion/constraints/effect grant.
};
