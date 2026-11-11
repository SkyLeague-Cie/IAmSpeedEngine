// Included before FStepScope ctor/destructor. VALUES ONLY, not capabilities.
struct FOrdinaryConsumedBodyValues
{
    const USSubBody* Body=nullptr;const void* Class=nullptr;int32 Index=INDEX_NONE;uint32 GTObjectId=0;
    FKinematicState State;SHitResult Current,Future;
    // Owned value storage only. The common small identity list stays with
    // this observation; larger lists keep the standard heap fallback.
    TArray<const UPrimitiveComponent*,TInlineAllocator<8>> Ignored;
    FVector LocalOffset=FVector::ZeroVector,BoxExtent=FVector::ZeroVector;FQuat LocalRotation=FQuat::Identity;
    FNativeSolidResponseConfigValues Config;bool SolidConfigCaptured=false,SphereShapeCaptured=false,BoxShapeCaptured=false;
    float PhysicalRadius=0;double SupportRadius=0,ContactOffset=0;
    // Derived caches/counters and nested Accept invocations remain unqualified.
};
struct FOrdinaryConsumedOwnerValues
{
    const ISpeedComponent* Owner=nullptr;uint64 StableId=0;FCanonicalPreparedFrameClock Clock;
    FKinematicState State;bool Frozen=false,Prescribed=false,SleepingEligibility=false; // reread eligibility, NOT a native branch decision
    FVector LocalCOM=FVector::ZeroVector;FQuat PhysRotation=FQuat::Identity;
    TArray<FPhysicalContactConstraint> Constraints;TArray<FOrdinaryConsumedBodyValues> Bodies;
};
#if !UE_BUILD_SHIPPING
// Closed actual response row retained only for its immediate next sweep.
// The lexical consumer still proves and verifies all real child refresh writes.
struct FFixedNoPoseSweepBaseline
{
    FPreparedWorldSnapshotPtr Snapshot;
    FStepIdentity ResponseStep,SweepStep;
    const UBoxSubBody* Resolver=nullptr;
    FOrdinaryConsumedOwnerValues After;
    TSharedPtr<const FOrdinaryGroundSweepEvidence,ESPMode::ThreadSafe> Ground;
    uint8 SweepRefreshCount=0;
    uint64 SweepRefreshQuerySerial=0;
};
#endif
struct FOrdinaryConsumedStateValues
{FStepIdentity Step;TArray<FOrdinaryConsumedOwnerValues> Owners;bool Captured=false;};
struct FOrdinaryBodyInertiaValue
{
    const USolidSubBody* Body=nullptr;const ISpeedComponent* Owner=nullptr;int32 BodyIndex=INDEX_NONE;
    EOrdinaryBodyInertiaOrigin Origin=EOrdinaryBodyInertiaOrigin::SphereCachedOperator;
    FMatrix Value=FMatrix::Identity;FQuat Basis=FQuat::Identity;FStepIdentity Step;
    FKinematicState OwnerState,BodyState;SHitResult Current;FCanonicalPreparedFrameClock Clock;
    // Box Basis is the actual RotateLocalInverseInertia argument. It may be
    // World, solver or chassis-local; no World interpretation is fabricated.
};
// Private expectation/closure of RECORDED values for the nonvirtual plain
// SetFutureHit assignment only. It grants no call, class/phase qualification,
// hidden-field/lifetime theorem, Accept/Resolve, advance or physical readiness.
struct FOrdinarySetFutureHitValueClosure
{
    const USSubBody* Body=nullptr;int32 OwnerIndex=INDEX_NONE,BodyIndex=INDEX_NONE;
    SHitResult ExpectedFuture;bool BoundBeforeCall=false,MatchedAfterReturn=false;
};
class FClosedPlanarResponseEffect;
struct FOrdinaryResponseCallValues
{
    int32 Call=INDEX_NONE;bool Opened=false,Returned=false,Closed=false;
    FOrdinaryConsumedStateValues Before,After;TArray<FOrdinaryBodyInertiaValue> Inertia;
    FOrdinarySetFutureHitValueClosure SetFutureValues;
    TSharedPtr<const FClosedPlanarResponseEffect,ESPMode::ThreadSafe> PlanarEffect;
};
struct FOrdinaryResponseEntryValues
{
    SComponentTOI Selected;const USSubBody* RawResolver=nullptr;bool Eligible=false,Allowed=false,Closed=false;
    bool PairBefore=false,PairAfter=false,Ignored=false,ExactRecordDecision=false;
    uint8 GenerationBefore=0,GenerationAfter=0;TArray<FOrdinaryResponseCallValues> Calls;
    FOrdinaryConsumedStateValues Entry,After;
};
struct FOrdinaryResponseState
{
    FStepIdentity EntryStep,SelectorStep;FPreparedWorldSnapshotPtr Snapshot;
    const Collision::FResolvedPairSet* PairsIdentity=nullptr; // borrowed while WorldStep lives; never read in destructor/reset
    TArray<SComponentTOI> Selected;TArray<FOrdinaryResponseEntryValues> Entries;
    FOrdinaryConsumedStateValues EntryValues;
    float Seconds=0,Before=0,ActualAfter=0,SimTime=0;
    int32 NextIndex=0,ActiveIndex=INDEX_NONE,ActiveCall=INDEX_NONE;bool Closed=false;
    // No geometric, integration, nested-call/effect or response qualification.
};
