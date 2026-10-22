#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/SHitResult.h"
#if !UE_BUILD_SHIPPING
class USSubBody;
class UPrimitiveComponent;

namespace Speed
{
// The owner declares semantic identities; addresses never generate identities.
// Different pointers with one key, or one pointer with two keys, are conflicts.
// A shared pointer is registered with the same key at every expected alias.
struct FFramePointerKey
{
    uint32 Domain = 0;
    uint64 Owner = 0, Generation = 0, Role = 0, Element = 0;
    TArray<uint64> Membership; // exact semantic source aliases, sorted by declarer
    bool operator==(const FFramePointerKey& R) const
    { return Domain==R.Domain && Owner==R.Owner && Generation==R.Generation && Role==R.Role && Element==R.Element && Membership==R.Membership; }
    friend uint32 GetTypeHash(const FFramePointerKey& K)
    { uint32 H=HashCombine(::GetTypeHash(K.Domain), HashCombine(::GetTypeHash(K.Owner), HashCombine(::GetTypeHash(K.Generation), HashCombine(::GetTypeHash(K.Role), ::GetTypeHash(K.Element))))); for(uint64 Id:K.Membership)H=HashCombine(H,::GetTypeHash(Id));return H; }
};

class FFramePointerRegistry
{
public:
    explicit FFramePointerRegistry(int32 InLimit=16384) : Limit(FMath::Clamp(InLimit,1,16384)) {}
    bool Register(UPTRINT Address, const FFramePointerKey& Key)
    {
        if (bSealed || !Address || !Key.Domain) { bValid=false; return false; }
        if (const FFramePointerKey* Existing = ByAddress.Find(Address))
        { if (*Existing==Key) return true; bValid=false; return false; }
        if (ByKey.Contains(Key) || ByAddress.Num()>=Limit) { bValid=false; return false; }
        ByAddress.Add(Address,Key); ByKey.Add(Key,Address); return true;
    }
    template<class T> bool Register(const T* Address,const FFramePointerKey& Key)
    { return Register(reinterpret_cast<UPTRINT>(Address),Key); }
    bool Seal() { bSealed=true; return bValid; }
    bool IsValidSealed() const { return bSealed && bValid; }
    const FFramePointerKey* Find(UPTRINT Address) const { return ByAddress.Find(Address); }
    FFramePointerRegistry CloneForAdditionalDeclarations() const
    {
        FFramePointerRegistry Out=*this;
        Out.bValid=IsValidSealed();Out.bSealed=false;return Out;
    }
private:
    TMap<UPTRINT,FFramePointerKey> ByAddress;
    TMap<FFramePointerKey,UPTRINT> ByKey;
    int32 Limit;
    bool bSealed=false,bValid=true;
};

struct FFramePointerObservation
{
    int32 Offset=0;
    UPTRINT RawAddress=0;
    FFramePointerKey Key;
    bool bKnown=false;
    bool bPairToken=false;
    bool bSolverPairToken=false;
    uint64 PairProviderId=0,PairFeatureSalt=0;
    bool bObjectId=false;
    FFramePointerKey SecondKey;
};

#if WITH_DEV_AUTOMATION_TESTS
class FFramePointerCanonicalSnapshot1456;
#endif

// Sidecar only: the existing byte witness still receives its original raw
// pointer bytes. No simulation field or hash is replaced by canonical bytes.
class IAMSPEED_API FFramePointerCaptureScope
{
public:
    FFramePointerCaptureScope(TArray<uint8>& Bytes,const FFramePointerRegistry& Registry,int32 InLimit=16384);
    ~FFramePointerCaptureScope();
    FFramePointerCaptureScope(const FFramePointerCaptureScope&)=delete;
    FFramePointerCaptureScope& operator=(const FFramePointerCaptureScope&)=delete;
    static void Observe(TArray<uint8>& Bytes,UPTRINT Address);
    static void ObserveObjectId(TArray<uint8>& Bytes,uint32 Id,const UObject* Object);
    static void ObservePairToken(TArray<uint8>& Bytes,uint64 Token,const USSubBody* Source,const UPrimitiveComponent* Other);
    // Validate the ACTUAL main-solver key: UObject pair, provider annotation,
    // and the optional resting-plane vertex annotation, in that exact order.
    // Metadata is copied only after that formula and both declared identities
    // match. An arbitrary integer is never accepted as a semantic pair key.
    static void ObserveSolverPairToken(TArray<uint8>& Bytes,uint64 Token,
        const USSubBody* Source,const SHitResult& Hit,bool bStaticRestingWorld);
    // Persistent world keys use Lo in the high word, unlike MakePairKey.
    // Validate that exact native packing before using the same semantic pair domain.
    static void ObserveDynamicPairToken(TArray<uint8>& Bytes,uint64 Token,const USSubBody* A,const USSubBody* B);
    // False means UNKNOWN, including unknown identity, overflow or byte drift.
    // In that case Canonical is left untouched and Raw remains available.
    bool MakeCanonical(TArray<uint8>& Canonical) const;
#if WITH_DEV_AUTOMATION_TESTS
    // Explicit test witness path only. Canonical must be empty; all three
    // arrays must be distinct. Failure preserves Canonical and raw Target,
    // while BuildScratch may retain uncertified partial bytes. Only success
    // swaps a complete build into Canonical, retaining both buffer capacities.
    bool MakeCanonicalWithScratch1368(TArray<uint8>& Canonical,TArray<uint8>& BuildScratch) const;
    // Copies immutable values only. Success means captured, not yet canonical:
    // the same byte/token validation still runs in the deferred MakeCanonical.
    bool CaptureCanonicalSnapshot1456(FFramePointerCanonicalSnapshot1456& Out) const;
    // Final transfer only after every physical writer has joined. Out must
    // outlive this scope and stay unmoved/unassigned until it closes. Raw/events
    // retain their exact bytes/keys by array swap;
    // projection is blocked until scope destruction validates no later write.
    bool ConsumeCanonicalSnapshot1456(FFramePointerCanonicalSnapshot1456& Out);
    bool RejectRecordAfterConsume1456() noexcept;
    // Transactional mapping of exact raw field/row boundaries through the SAME
    // validated projector. A boundary inside a pointer token is unavailable.
    bool MakeCanonicalWithBoundaries1534(TArray<uint8>& Out,TConstArrayView<int32> RawBoundaries,TArray<int32>& CanonicalBoundaries) const;
#endif
    const TArray<FFramePointerObservation>& Observations() const { return Events; }
    bool HasUnknown() const { return bUnknown; }
    bool HasOverflow() const { return bOverflow; }
private:
    TArray<uint8>& Target;
    const FFramePointerRegistry& Identities;
    TArray<FFramePointerObservation> Events;
    FFramePointerCaptureScope* Previous=nullptr;
    int32 Limit=0;
    bool bUnknown=false,bOverflow=false;
#if WITH_DEV_AUTOMATION_TESTS
    bool bConsumed1456=false;
    FFramePointerCanonicalSnapshot1456* ConsumedSnapshot1456=nullptr;
#endif
};

#if WITH_DEV_AUTOMATION_TESTS
// Evidence copying has a separate result from canonical validity: UNKNOWN
// snapshots still retain their exact captured raw stream after scope closure.
enum class EFramePointerRawEvidenceCopy1456 : uint8
{ NotCaptured, WriterScopeOpen, DestinationNotEmpty, Copied };
// Owns raw bytes and complete semantic observations. No UObject, registry or
// live scope access is needed when projecting after the physical owner joins.
class IAMSPEED_API FFramePointerCanonicalSnapshot1456
{
public:
    void Reserve1456(int32 RawBytes,int32 Observations);
    // Explicit fixture preparation only, before capture. Touch raw storage on
    // the caller, retain Num=0, and keep normal growth/failure semantics.
    // Refuses without mutation if any existing capture/data would be erased.
    bool ReserveAndPreTouch1456(int32 RawBytes,int32 Observations);
    bool MakeCanonical(TArray<uint8>& Out) const;
    bool MakeCanonicalWithBoundaries1534(TArray<uint8>& Out,TConstArrayView<int32> RawBoundaries,TArray<int32>& CanonicalBoundaries) const;
    // Copies only after writer-scope closure. Nonempty destinations are never
    // overwritten: callers can preserve illegal late-write debris separately.
    // A captured UNKNOWN is still evidence and is deliberately copyable.
    EFramePointerRawEvidenceCopy1456 CopyRawForEvidenceAfterClosure1456(TArray<uint8>& Out) const;
    const TArray<uint8>& RawForEvidence1456() const { return Raw1456; }
    int32 NumObservations1456() const { return Events1456.Num(); }
private:
    friend class FFramePointerCaptureScope;
    TArray<uint8> Raw1456;
    TArray<FFramePointerObservation> Events1456;
    bool bCaptured1456=false,bValidSealed1456=false,bUnknown1456=false,bOverflow1456=false;
    bool bWriterScopeOpen1456=false;
};
#endif
}
#endif
