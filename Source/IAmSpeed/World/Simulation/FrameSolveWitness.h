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
#if WITH_DEV_AUTOMATION_TESTS
enum class EWheelWitnessRelease1368 : uint8
{ NotCaptured, WriterOpen, DestinationNotEmpty, AliasedDestinations, AlreadyReleased, TransferredValid, TransferredInvalid };
// Final owner capture; object must stay immobile and alive until solve scope
// closure. Evidence is released after the physical owner has joined.
class IAMSPEED_API FWheelWitnessSnapshot1368
{
    friend class FFrameSolveWitnessScope;
    TArray<uint8> Raw,Canonical,RawDebris,CanonicalDebris;
    bool bCaptured=false,bWriterOpen=false,bValid=false,bReleased=false;
public:
    EWheelWitnessRelease1368 ReleaseAfterClosure(TArray<uint8>& OutRaw,TArray<uint8>& OutCanonical,
        TArray<uint8>& OutRawDebris,TArray<uint8>& OutCanonicalDebris);
};
#endif
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
    // Final one-shot handoff; caller has returned from the joined physical body
    // and performs no further solve/contact records. Destination outlives scope.
    bool SealAndConsumeCanonicalSnapshot1456(FFramePointerCanonicalSnapshot1456& Out)
    {
        return !bOverflow&&!bRowBoundaries1534&&OpenDynamicPasses.IsEmpty()&&OpenWheelOwners1368.IsEmpty()
            &&PointerCapture.ConsumeCanonicalSnapshot1456(Out);
    }
    // The row-framed path retains its exact raw starts plus original stream
    // end before ownership moves. The same deferred token/boundary projector
    // still validates all canonical bytes; no row capture is disabled.
    bool SealAndConsumeCanonicalSnapshotWithRowBoundaries1534(FFramePointerCanonicalSnapshot1456& Out,TArray<int32>& RawBoundaries)
    {
        if(!bRowBoundaries1534||bOverflow||!OpenDynamicPasses.IsEmpty()||!OpenWheelOwners1368.IsEmpty()||RawRowBoundaries1534.Num()!=Rows)return false;
        const int32 RawEnd=Bytes.Num();int32 PreviousBoundary=-1;
        for(const int32 B:RawRowBoundaries1534){if(B<PreviousBoundary||B<0||B>RawEnd)return false;PreviousBoundary=B;}
        TArray<int32> CapturedBounds=RawRowBoundaries1534;CapturedBounds.Add(RawEnd);
        if(!PointerCapture.ConsumeCanonicalSnapshot1456(Out))return false;
        RawBoundaries=MoveTemp(CapturedBounds);return true;
    }
    // Explicit opt-in only. Legacy main/dynamic streams remain schemas1/2.
    bool EnableWheelManifoldCapture1368() noexcept
    {
        if(Rows||WheelRows1368||bCaptureWheelManifold1368)return false;
        bCaptureWheelManifold1368=true;return true;
    }
#if WITH_DEV_AUTOMATION_TESTS
    bool EnableWheelManifoldCapture1368(bool ReusePointerEvents1368) noexcept
    {
        if(!EnableWheelManifoldCapture1368())return false;
        bReuseWheelPointerEvents1368=ReusePointerEvents1368;return true;
    }
#endif
    static bool IsWheelManifoldActive1368() noexcept;
    const TArray<uint8>& RawWheelManifold1368()const noexcept{return WheelRaw1368;}
    bool MakeWheelManifoldCanonical1368(TArray<uint8>& Out)const
    {
        if(ConsumedWheel1368||!bCaptureWheelManifold1368||bOverflow||!OpenWheelOwners1368.IsEmpty()||WheelCanonicalRows1368!=WheelRows1368)return false;
        Out=WheelCanonical1368;return true;
    }
    // One-shot final transfer after the last joined physical/wheel callback.
    // Nonempty main bytes must first be sealed/transferred by their own API.
    // Return success is provisional until scope closure; it is never a verdict.
    bool ConsumeWheelWitness1368(FWheelWitnessSnapshot1368& Out);
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
    // Per-scope row storage; the explicit transactional scratch API swaps the
    // two canonical buffers only after complete validation and encoding.
    TArray<uint8> WheelRowRawScratch1368,WheelRowCanonicalScratch1368,WheelRowCanonicalBuildScratch1368;
#if WITH_DEV_AUTOMATION_TESTS
    bool bReuseWheelPointerEvents1368=false;
    TArray<FFramePointerObservation> WheelRowPointerEventsScratch1368;
    FWheelWitnessSnapshot1368* ConsumedWheel1368=nullptr;
#endif
    bool bOverflow=false;
#if WITH_DEV_AUTOMATION_TESTS
    bool bRowBoundaries1534=false;TArray<int32> RawRowBoundaries1534;
#endif
};
}
#endif
