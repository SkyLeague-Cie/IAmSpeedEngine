#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/SHitResult.h"
#include "FramePointerWitness.h"
#if !UE_BUILD_SHIPPING
struct FDynamicContactPair;
struct SWheelGroundContact;
class ISpeedComponent;
struct FFakePhysicsImpactContext;
namespace Speed
{
// Serial observer of ACTUAL selected contacts, persistent/rolling solves and
// projections. Does not choose contacts, sort, sample clocks or mutate physics.
enum class EFrameDynamicContactPass : uint8 { Persistent=1, Projection=2 };
enum class EFrameDynamicContactEvent : uint8
{
 PassBegin=1,PassEnd,PairBegin,PairEnd,NormalCandidate,
 NormalBeforeA,NormalAfterA,NormalBeforeB,NormalAfterB,
 TangentialCandidate,TangentialBeforeA,TangentialAfterA,TangentialBeforeB,TangentialAfterB,
 FakePhysicsBefore,FakePhysicsAfter,
 ProjectActiveBegin,ProjectActiveEnd,ProjectPendingBegin,ProjectPendingEnd,
 ProjectFreshBegin,ProjectFreshEnd,ProjectBefore,ProjectAfter,PassDisabled
};
enum class EFrameDynamicContactOutcome : uint8
{
 Unspecified=0,Completed,Filtered,AlreadySolved,InvalidBody,Unseen,FrameRewind,
 FirstSeenDeferred,DiscontinuousMotion,RejectedGeometry,ZeroNormal,SeparatingVelocity,
 IdentityOnly,ZeroDenominator,ProjectionDisabled
};
class IAMSPEED_API FFrameSolveWitnessScope
{
public:
    FFrameSolveWitnessScope(uint64 InFrame,TArray<uint8>& Raw,
        const FFramePointerRegistry& Registry,int32 MaximumRows=65536);
    ~FFrameSolveWitnessScope();
    static bool IsActive();
    FFrameSolveWitnessScope(const FFrameSolveWitnessScope&)=delete;
    FFrameSolveWitnessScope& operator=(const FFrameSolveWitnessScope&)=delete;
    static void Record(uint64 Frame,int32 Iteration,int32 Order,uint64 ActorStableId,
        int32 ResolverIndex,uint64 PairKey,const SHitResult& Hit,bool bIgnored,
        int32 GenerationBefore,int32 GenerationAfter,const USSubBody* Resolver=nullptr,
        bool bStaticRestingWorld=false);
    // The schema2 rows share the same global sequence as actual main resolver rows.
    // They observe real persistent/rolling/projection invocations, never choose them.
    static uint64 BeginDynamicPass(uint64 InFrame,EFrameDynamicContactPass Kind,float Dt,
        uint64 FilterKey,bool bFiltered,bool bFirstSeen,int32 PairCount,
        const ::FDynamicContactPair* FilterPair=nullptr) noexcept;
    static void EndDynamicPass(uint64 Pass,int32 PairCount) noexcept;
    static uint64 CurrentDynamicPass() noexcept;
    static void RecordDynamicEvent(uint64 Pass,EFrameDynamicContactEvent Event,int32 PairIndex,
        uint64 PairKey,const ::FDynamicContactPair* Pair,const USSubBody* BodyA,const USSubBody* BodyB,
        EFrameDynamicContactOutcome Outcome=EFrameDynamicContactOutcome::Unspecified,bool bRolling=false,
        const FVector& Vector=FVector::ZeroVector,const FVector& Point=FVector::ZeroVector,
        float Scalar1=0.0f,float Scalar2=0.0f,const SHitResult* Hit=nullptr,const ::FFakePhysicsImpactContext* Context=nullptr) noexcept;
#if WITH_DEV_AUTOMATION_TESTS
    // Explicit diagnostic opt-in. Bounds identify the actual global solve row,
    // including schema2 events, without changing or framing its existing bytes.
    bool EnableRowBoundaryCapture1534()
    {if(Rows||bRowBoundaries1534)return false;bRowBoundaries1534=true;RawRowBoundaries1534.Reserve(FMath::Min(Limit,256));return true;}
    bool MakeCanonicalWithRowBoundaries1534(TArray<uint8>& Out,TArray<int32>& Boundaries)const
    {
        if(!bRowBoundaries1534||bOverflow||!OpenDynamicPasses.IsEmpty()||!OpenWheelOwners1368.IsEmpty()||RawRowBoundaries1534.Num()!=Rows)return false;
        TArray<int32> Raw=RawRowBoundaries1534;Raw.Add(Bytes.Num());
        return PointerCapture.MakeCanonicalWithBoundaries1534(Out,Raw,Boundaries);
    }
    bool CaptureCanonicalSnapshotWithRowBoundaries1534(FFramePointerCanonicalSnapshot1456& Out,TArray<int32>& RawBoundaries)const
    {
        if(!bRowBoundaries1534||bOverflow||!OpenDynamicPasses.IsEmpty()||!OpenWheelOwners1368.IsEmpty()||RawRowBoundaries1534.Num()!=Rows)return false;
        if(!PointerCapture.CaptureCanonicalSnapshot1456(Out))return false;
        RawBoundaries=RawRowBoundaries1534;RawBoundaries.Add(Bytes.Num());return true;
    }
    // Capture remains on the canonical owner. Only pure byte projection moves.
    // Open passes/unknown pointers/overflow never become a valid certificate.
    bool CaptureCanonicalSnapshot1456(FFramePointerCanonicalSnapshot1456& Out) const
    {
        return !bOverflow&&OpenDynamicPasses.IsEmpty()&&OpenWheelOwners1368.IsEmpty()
            &&PointerCapture.CaptureCanonicalSnapshot1456(Out);
    }
    // Explicit opt-in only. Legacy main/dynamic streams remain schemas1/2.
    bool EnableWheelManifoldCapture1368() noexcept
    {
        if(Rows||WheelRows1368||bCaptureWheelManifold1368)return false;
        bCaptureWheelManifold1368=true;return true;
    }
    static bool IsWheelManifoldActive1368() noexcept;
    const TArray<uint8>& RawWheelManifold1368()const noexcept{return WheelRaw1368;}
    bool MakeWheelManifoldCanonical1368(TArray<uint8>& Out)const
    {
        if(!bCaptureWheelManifold1368||bOverflow||!OpenWheelOwners1368.IsEmpty()||WheelCanonicalRows1368!=WheelRows1368)return false;
        Out=WheelCanonical1368;return true;
    }
    // Framed schema3 sidecar: actual suspension inputs/order/impulses.
    static void RecordWheelManifold1368(uint8 Event,const ISpeedComponent* Owner,
        int32 DetailA,int32 DetailB,TConstArrayView<SWheelGroundContact> Contacts,
        TConstArrayView<int32> Order,TConstArrayView<float> Scalars={},
        TConstArrayView<double> DoubleScalars={},TConstArrayView<FVector> Vectors={}) noexcept;
    int32 NumWheelManifoldRows1368() const {return WheelRows1368;}
#endif
    bool MakeCanonical(TArray<uint8>& Out) const
    { return !bOverflow && OpenDynamicPasses.IsEmpty() && OpenWheelOwners1368.IsEmpty() && PointerCapture.MakeCanonical(Out); }
    int32 NumRows() const { return Rows; }
    int32 NumMainRows() const { return MainRows; }
    int32 NumDynamicRows() const { return Rows-MainRows; }
    bool HasOverflow() const { return bOverflow || PointerCapture.HasOverflow(); }
    bool HasUnknownPointer() const { return PointerCapture.HasUnknown(); }
    int32 OpenDynamicPassCount() const { return OpenDynamicPasses.Num(); }
    const TArray<FFramePointerObservation>& PointerObservations() const
    { return PointerCapture.Observations(); }
private:
    struct FDynamicPass { uint64 Id=0;EFrameDynamicContactPass Kind=EFrameDynamicContactPass::Persistent;
        float Dt=0;bool bFiltered=false,bFirstSeen=false; };
    TArray<FDynamicPass> OpenDynamicPasses;
    uint64 NextDynamicPass=0;
    uint64 Frame;
    TArray<uint8>& Bytes;
    FFramePointerCaptureScope PointerCapture;
    const FFramePointerRegistry& WheelRegistry1368;
    FFrameSolveWitnessScope* Previous=nullptr;
    int32 Limit=0,Rows=0,MainRows=0,WheelRows1368=0;
    TArray<const ISpeedComponent*,TInlineAllocator<4>> OpenWheelOwners1368;
    bool bCaptureWheelManifold1368=false;int32 WheelCanonicalRows1368=0;
    TArray<uint8> WheelRaw1368,WheelCanonical1368;
    bool bOverflow=false;
#if WITH_DEV_AUTOMATION_TESTS
    bool bRowBoundaries1534=false;TArray<int32> RawRowBoundaries1534;
#endif
};
}
#endif
