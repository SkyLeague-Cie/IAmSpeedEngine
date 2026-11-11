#pragma once
#include "UObject/WeakObjectPtr.h"
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "IAmSpeed/World/Analytic/TensorDistanceBounds.h"
#include "UObject/WeakObjectPtrTemplates.h"
class UWorld;
class UPrimitiveComponent;
class ALandscapeProxy;
namespace Speed
{
struct FPreparedHeightfieldValues
{
    TArray<uint16> Heights;
    TArray<uint8> Materials;
    FVector3d Scale=FVector3d::ZeroVector;
    double Min=0,Max=0,HeightPerUnit=0;
    int32 Rows=0,Cols=0;
};
// A separately captured physical provider. It never changes the authored DA,
// canonical packet, reference aliases, or C2/symmetry claims of the stadium.
struct FPreparedLandscapeMember
{
    TWeakObjectPtr<UPrimitiveComponent> Component;
    const void* Identity=nullptr;
    const void* Heightfield=nullptr;
    const void* Complex=nullptr;
    const void* Simple=nullptr;
    FGuid Guid;
    FTransform Transform;
    FTransform PhysicsTransform;
    FPreparedHeightfieldValues ComplexValues,SimpleValues;
    FBox3d ComplexBounds=FBox3d(EForceInit::ForceInit);
    FBox3d SimpleBounds=FBox3d(EForceInit::ForceInit);
    FWeakObjectPtr MaterialToken;
    uint32 ObjectId=0,OwnerObjectId=0;
};
struct FPreparedLandscapePlane
{
    TWeakObjectPtr<ALandscapeProxy> Owner;
    FWeakObjectPtr OwnerToken;
    const void* OwnerIdentity=nullptr;
    Analytic::FBoundedPlane Plane;
    TArray<FPreparedLandscapeMember> Members;
    uint32 OwnerObjectId=0;
};
class IAMSPEED_API FPreparedExternalLandscape final
{
    TArray<FPreparedLandscapePlane> Planes;
public:
    static bool CaptureGT(UWorld&,TSharedPtr<const FPreparedExternalLandscape,ESPMode::ThreadSafe>&,FString&);
    bool MatchesGT(UWorld&,FString&) const;
    bool ContainsMember(const UPrimitiveComponent*) const;
    TConstArrayView<FPreparedLandscapePlane> GetPlanes() const {return Planes;}
    static bool Policy(const Analytic::FWorldQuery&,const Analytic::FBoundedPlane&);
    // Outward full-volume exclusion, not a fabricated native collision miss.
    // A positive native Landscape CCD is deliberately not inferred from the
    // canonical stadium solver: an unseparated external remains Unknown.
    bool StrictlySeparated(const Analytic::FWorldQuery&,const Analytic::Certified::FIntervalPoint&) const;
    bool EvaluateFixed(const Analytic::FWorldQuery&,bool overlap,Analytic::FWorldHit&) const;
};
using FPreparedExternalLandscapePtr=TSharedPtr<const FPreparedExternalLandscape,ESPMode::ThreadSafe>;
}
