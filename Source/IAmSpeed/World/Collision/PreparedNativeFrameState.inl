struct FOrdinaryNativeAdvanceTicket
{
    TArray<FGeneralBoxTransportBinding> GeneralBindings;
    FStepIdentity Queried;
    TArray<SComponentTOI> Selected;
    float Seconds=0,Horizon=0,Before=0;
    bool HasSelection=false,Consumed=false;
};
// Owned values for ONE final-operation boundary. No query, result or authority
// cache. Integration/response observations retain their complete old types.
struct FFinalGeneralBoxBodyValues
{
    const USSubBody* Body=nullptr;const void* Class=nullptr;int32 Index=INDEX_NONE;uint32 GTObjectId=0;
    FKinematicState State;
    FVector LocalOffset=FVector::ZeroVector,BoxExtent=FVector::ZeroVector;
    FQuat LocalRotation=FQuat::Identity;
    FNativeSolidResponseConfigValues Config;
    bool SolidConfigCaptured=false,BoxShapeCaptured=false;
};
struct FFinalGeneralBoxOwnerValues
{
    const ISpeedComponent* Owner=nullptr;uint64 StableId=0;
    FKinematicState State;bool Frozen=false,Prescribed=false;
    FVector LocalCOM=FVector::ZeroVector;FQuat PhysRotation=FQuat::Identity;
    TArray<FFinalGeneralBoxBodyValues> Bodies;
};
struct FFinalGeneralBoxStateValues
{
    FStepIdentity Step;
    TArray<FFinalGeneralBoxOwnerValues,TInlineAllocator<1>> Owners;
    bool Captured=false;
};
struct FFinalGeneralBoxObservation
{FFinalGeneralBoxStateValues Before,After;};

struct FOrdinaryFinalOperation
{
    ENativeFrameFinalOperation Kind=ENativeFrameFinalOperation::StaticProjectionOwner;
    const ISpeedComponent* Owner=nullptr;
    int32 Index=INDEX_NONE;
    bool Called=true,Returned=false;
    FOrdinaryConsumedStateValues Before,After;
    // Format is selected once on entry and retained through the actual callback.
    TUniquePtr<FFinalGeneralBoxObservation> GeneralBoxOnly;
    TUniquePtr<FFinalGeneralBoxStateValues> WheelSupportExpected;
};
struct FOrdinaryFinalState
{
    FStepIdentity Entry;
    int32 Stage=0,NextIndex=0,WheelSupportOperation=INDEX_NONE;
    bool CallOpen=false,Closed=false;
    float ActualEndTime=0;
    TArray<FOrdinaryFinalOperation> Operations;
};
