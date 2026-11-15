#include "PreparedExternalLandscape.h"
#include "IAmSpeed/World/Analytic/AnalyticLandscapeAdapter.h"
#include "IAmSpeed/World/Analytic/FixedShapeQueryBounds.h"
#include "LandscapeProxy.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Chaos/HeightField.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "PhysicsEngine/BodyInstance.h"
namespace Speed
{
namespace
{
using CPoint=Analytic::Certified::FIntervalPoint;
void CaptureField(const Chaos::FHeightField& field,FPreparedHeightfieldValues& out)
{
    const auto& d=field.GeomData;out.Heights=d.Heights;out.Materials=d.MaterialIndices;out.Scale=FVector3d(d.Scale);
    out.Min=d.MinValue;out.Max=d.MaxValue;out.HeightPerUnit=d.HeightPerUnit;out.Rows=d.NumRows;out.Cols=d.NumCols;
}
bool SameField(const Chaos::FHeightField& field,const FPreparedHeightfieldValues& values)
{
    const auto& d=field.GeomData;
    return d.Heights==values.Heights&&d.MaterialIndices==values.Materials&&FVector3d(d.Scale)==values.Scale&&
        d.MinValue==values.Min&&d.MaxValue==values.Max&&d.HeightPerUnit==values.HeightPerUnit&&d.NumRows==values.Rows&&d.NumCols==values.Cols;
}
bool Bounds(const Chaos::FHeightField* field,FBox3d& out)
{
    if(!field||field->GetNumRows()<2||field->GetNumCols()<2)return false;
    const auto box=field->BoundingBox();out=FBox3d(FVector3d(box.Min()),FVector3d(box.Max()));
    return out.IsValid&&!out.Min.ContainsNaN()&&!out.Max.ContainsNaN();
}
bool Field(const Chaos::FHeightField* field,const FTransform& transform,FBox3d& local,FBox3d& world,
    uint8& material,bool scan)
{
    if(!Bounds(field,local))return false;
    const int32 nx=field->GetNumCols(),ny=field->GetNumRows();
    const auto& indices=field->GeomData.MaterialIndices;
    if(field->GeomData.Heights.Num()!=nx*ny||(indices.Num()!=1&&indices.Num()!=(nx-1)*(ny-1)))return false;
    // Chaos has already applied CollisionScale and LANDSCAPE_ZSCALE to the
    // heightfield. Match CreatePhysicsState: apply rotation/translation ONLY.
    world=FBox3d(EForceInit::ForceInit);
    for(int32 y:{0,ny-1})for(int32 x:{0,nx-1})
        world+=FVector3d(transform.TransformPositionNoScale(FVector(field->GetPointScaled(y*nx+x))));
    if(!world.IsValid||world.Min.ContainsNaN()||world.Max.ContainsNaN()||world.Min.Z!=world.Max.Z||
        !(world.Min.X<world.Max.X&&world.Min.Y<world.Max.Y))return false;
    material=indices[0];
    if(material==MAX_uint8)return false;
    if(scan)
    {
        for(int32 y=0;y<ny;++y)for(int32 x=0;x<nx;++x)
        {
            const auto p=FVector3d(transform.TransformPositionNoScale(FVector(field->GetPointScaled(y*nx+x))));
            if(p.ContainsNaN()||p.Z!=world.Min.Z||p.X<world.Min.X||p.X>world.Max.X||p.Y<world.Min.Y||p.Y>world.Max.Y)return false;
            if(x<nx-1&&y<ny-1&&indices[indices.Num()==1?0:y*(nx-1)+x]!=material)return false;
        }
    }
    return true;
}
uint64 Blocking(const UPrimitiveComponent& c)
{
    uint64 result=0;
    for(int32 i=0;i<UE_ARRAY_COUNT(FCollisionResponseContainer::EnumArray);++i)
        if(c.GetCollisionResponseToChannel(ECollisionChannel(i))==ECR_Block)result|=uint64(1)<<i;
    return result;
}
bool RectangularUnion(const TArray<FBox3d>& tiles,const FBox3d& whole)
{
    TArray<double> xs;for(const auto& b:tiles){xs.AddUnique(b.Min.X);xs.AddUnique(b.Max.X);}xs.Sort();
    for(int32 i=1;i<xs.Num();++i)
    {
        TArray<FVector2d> ys;
        for(const auto& b:tiles)if(b.Min.X<=xs[i-1]&&b.Max.X>=xs[i])ys.Emplace(b.Min.Y,b.Max.Y);
        ys.Sort([](const auto& a,const auto& b){return a.X<b.X;});
        double end=whole.Min.Y;
        for(const auto& y:ys){if(y.X!=end||!(y.Y>y.X))return false;end=y.Y;}
        if(end!=whole.Max.Y)return false;
    }
    return xs.Num()>1&&xs[0]==whole.Min.X&&xs.Last()==whole.Max.X;
}
bool CapturePlane(ALandscapeProxy& owner,FPreparedLandscapePlane& out,FString& reason)
{
    auto fail=[&](const TCHAR* why){reason=FString::Printf(TEXT("Landscape %s: %s"),*owner.GetPathName(),why);return false;};
    if(owner.CollisionComponents.IsEmpty()||owner.GetActorQuat()!=FQuat::Identity)return fail(TEXT("missing collision or unsupported rotation"));
    out.Owner=&owner;out.OwnerToken=FWeakObjectPtr(&owner);out.OwnerIdentity=&owner;out.OwnerObjectId=owner.GetUniqueID();
    Analytic::FFlatLandscapeSource source;
    source.SourceId=Analytic::StableStringId(UWorld::RemovePIEPrefix(owner.GetPathName()));
    source.bHoleCoverageValidated=true;source.bMaterialCoverageValidated=true;source.bCollisionPolicyValidated=true;
    TArray<FBox3d> tiles;TSet<const void*> seen;UPhysicalMaterial* material=nullptr;
    for(ULandscapeHeightfieldCollisionComponent* component:owner.CollisionComponents)
    {
        if(!IsValid(component)||component->GetOwner()!=&owner||!component->IsRegistered()||!component->IsQueryCollisionEnabled()||
            component->GetComponentQuat()!=FQuat::Identity||component->GetComponentScale().GetMin()<=0||seen.Contains(component)||
            !component->HeightfieldRef||!component->HeightfieldRef->HeightfieldGeometry)return fail(TEXT("incomplete live collision member"));
        seen.Add(component);FPreparedLandscapeMember member;member.Component=component;member.Identity=component;
        member.ObjectId=component->GetUniqueID();member.OwnerObjectId=owner.GetUniqueID();member.Transform=component->GetComponentTransform();
        auto* body=component->GetBodyInstance();if(!body||!body->IsValidBodyInstance())return fail(TEXT("live physics body unavailable"));
        member.PhysicsTransform=body->GetUnrealWorldTransform(false,true);
        if(member.PhysicsTransform.GetRotation()!=FQuat::Identity)return fail(TEXT("unsupported physical rotation"));
        member.Guid=component->HeightfieldGuid;member.Heightfield=component->HeightfieldRef.GetReference();
        member.Complex=component->HeightfieldRef->HeightfieldGeometry.GetReference();
        member.Simple=component->HeightfieldRef->HeightfieldSimpleGeometry.GetReference();
        FBox3d world;uint8 index=0;
        if(!Field(component->HeightfieldRef->HeightfieldGeometry.GetReference(),member.PhysicsTransform,member.ComplexBounds,world,index,true))
            return fail(TEXT("actual complex heightfield is not exactly flat, hole-free and uniform"));
        CaptureField(*component->HeightfieldRef->HeightfieldGeometry,member.ComplexValues);
        auto* actualMaterial=component->GetPhysicalMaterial(0,0,EHeightfieldSource::Complex);
        if(!actualMaterial)return fail(TEXT("actual physical material unavailable"));
        member.MaterialToken=FWeakObjectPtr(actualMaterial);
        if(member.Simple)
        {
            FBox3d simpleWorld;uint8 simpleIndex=0;
            if(!Field(component->HeightfieldRef->HeightfieldSimpleGeometry.GetReference(),member.PhysicsTransform,member.SimpleBounds,simpleWorld,simpleIndex,true)||
                simpleWorld!=world||component->GetPhysicalMaterial(0,0,EHeightfieldSource::Simple)!=actualMaterial)
                return fail(TEXT("simple and complex collision differ"));
            CaptureField(*component->HeightfieldRef->HeightfieldSimpleGeometry,member.SimpleValues);
        }
        const uint64 mask=Blocking(*component);const uint32 object=uint32(component->GetCollisionObjectType());
        if(out.Members.IsEmpty())
        {source.ObjectType=object;source.BlockingChannels=mask;source.bQueryCollisionEnabled=true;material=actualMaterial;}
        else if(source.ObjectType!=object||source.BlockingChannels!=mask||material!=actualMaterial||source.WorldHeights[0]!=world.Min.Z)
            return fail(TEXT("mixed member policy, material or height"));
        source.WorldBounds+=world;source.WorldHeights.Add(world.Min.Z);tiles.Add(world);out.Members.Add(MoveTemp(member));
    }
    if(!RectangularUnion(tiles,source.WorldBounds))return fail(TEXT("collision tiles do not close one exact rectangle"));
    source.MaterialId=uint32(Analytic::StableStringId(material->GetPathName()));
    const auto adapted=Analytic::BuildFlatLandscapePlane(source,0.);
    if(adapted.Result!=Analytic::EFlatLandscapeAdapterResult::SuccessAuthorityEligible)return fail(*adapted.Diagnostic);
    out.Plane=adapted.Plane;
    if(out.Plane.Origin.X-out.Plane.HalfExtents.X!=source.WorldBounds.Min.X||
        out.Plane.Origin.X+out.Plane.HalfExtents.X!=source.WorldBounds.Max.X||
        out.Plane.Origin.Y-out.Plane.HalfExtents.Y!=source.WorldBounds.Min.Y||
        out.Plane.Origin.Y+out.Plane.HalfExtents.Y!=source.WorldBounds.Max.Y)
        return fail(TEXT("finite plane endpoints do not exactly reproduce the physical rectangle"));
    UE_LOG(LogTemp,Display,TEXT("[PreparedLandscapePlane] Actor=%s Members=%d Height=%.17g Bounds=(%.17g,%.17g)-(%.17g,%.17g) CollisionRetained=1"),
        *owner.GetPathName(),out.Members.Num(),out.Plane.Origin.Z,source.WorldBounds.Min.X,source.WorldBounds.Min.Y,source.WorldBounds.Max.X,source.WorldBounds.Max.Y);
    return true;
}
}
bool FPreparedExternalLandscape::CaptureGT(UWorld& world,FPreparedExternalLandscapePtr& out,FString& reason)
{
    out.Reset();if(!IsInGameThread()){reason=TEXT("External Landscape capture requires GT lease");return false;}
    TSharedPtr<FPreparedExternalLandscape,ESPMode::ThreadSafe> result=MakeShareable(new FPreparedExternalLandscape);
    for(TActorIterator<ALandscapeProxy> it(&world);it;++it)
    {FPreparedLandscapePlane plane;if(!CapturePlane(**it,plane,reason))return false;result->Planes.Add(MoveTemp(plane));}
    out=MoveTemp(result);return true;
}
bool FPreparedExternalLandscape::ContainsMember(const UPrimitiveComponent* component) const
{for(const auto& p:Planes)for(const auto& member:p.Members)if(member.Identity==component)return true;return false;}
bool FPreparedExternalLandscape::MatchesGT(UWorld& world,FString& reason) const
{
    if(!IsInGameThread())return false;int32 count=0;
    for(TActorIterator<ALandscapeProxy> it(&world);it;++it)
    {
        ++count;const FPreparedLandscapePlane* p=nullptr;
        for(const auto& row:Planes)if(row.Owner.Get()==*it){p=&row;break;}
        if(!p||p->Members.Num()!=it->CollisionComponents.Num())return false;
        for(const auto& member:p->Members)
        {
            auto* c=Cast<ULandscapeHeightfieldCollisionComponent>(member.Component.Get());
            if(!c||c!=member.Identity||c->GetOwner()!=*it||!it->CollisionComponents.Contains(c)||!c->IsRegistered()||
                c->GetWorld()!=&world||!c->IsQueryCollisionEnabled()||!c->GetComponentTransform().Equals(member.Transform,0.)||
                c->GetUniqueID()!=member.ObjectId||it->GetUniqueID()!=member.OwnerObjectId||c->HeightfieldGuid!=member.Guid||
                c->HeightfieldRef.GetReference()!=member.Heightfield||!c->HeightfieldRef||
                c->HeightfieldRef->HeightfieldGeometry.GetReference()!=member.Complex||
                c->HeightfieldRef->HeightfieldSimpleGeometry.GetReference()!=member.Simple||
                uint32(c->GetCollisionObjectType())!=p->Plane.ObjectType||Blocking(*c)!=p->Plane.BlockingChannels)return false;
            // Compare every raw height/material value and scale, including an
            // in-place edit on a retained object. No bounds-only flatness seal.
            auto* body=c->GetBodyInstance();if(!body||!body->IsValidBodyInstance()||
                !body->GetUnrealWorldTransform(false,true).Equals(member.PhysicsTransform,0.)||
                !SameField(*c->HeightfieldRef->HeightfieldGeometry,member.ComplexValues)||
                (member.Simple&&!SameField(*c->HeightfieldRef->HeightfieldSimpleGeometry,member.SimpleValues)))return false;
            FBox3d b;
            if(!Bounds(c->HeightfieldRef->HeightfieldGeometry.GetReference(),b)||b!=member.ComplexBounds)return false;
            if(member.Simple&&(!Bounds(c->HeightfieldRef->HeightfieldSimpleGeometry.GetReference(),b)||b!=member.SimpleBounds))return false;
            auto* m=c->GetPhysicalMaterial(0,0,EHeightfieldSource::Complex);
            if(!m||m!=member.MaterialToken.Get()||uint32(Analytic::StableStringId(m->GetPathName()))!=p->Plane.MaterialId||
                (member.Simple&&c->GetPhysicalMaterial(0,0,EHeightfieldSource::Simple)!=m))return false;
        }
    }
    if(count!=Planes.Num()){reason=TEXT("External Landscape census changed");return false;}return true;
}
bool FPreparedExternalLandscape::Policy(const Analytic::FWorldQuery& q,const Analytic::FBoundedPlane& p)
{
    if((q.RequiredSourceId&&q.RequiredSourceId!=p.SourceId)||(q.RequiredSurfaceId&&q.RequiredSurfaceId!=p.SurfaceId)||
        q.RequiredCanonicalGroupId||!p.bQueryCollisionEnabled||(q.bExcludeAuthorityEligible&&p.bAuthorityEligible))return false;
    if(!q.bApplyCollisionFilter)return true;
    if(p.ObjectType>=64)return false;
    return q.bObjectQuery?(q.ObjectTypes&(uint64(1)<<p.ObjectType))!=0:
        q.TraceChannel<64&&(q.BlockingObjectTypes&(uint64(1)<<p.ObjectType))&&(p.BlockingChannels&(uint64(1)<<q.TraceChannel));
}
bool FPreparedExternalLandscape::StrictlySeparated(const Analytic::FWorldQuery& q,const CPoint& hull) const
{
    if(!Analytic::Certified::Detail::ValidPoint(hull))return false;
    for(const auto& row:Planes)if(Policy(q,row.Plane))
    {
        const auto& p=row.Plane;
        if(hull.Axis[2].Lower>p.Origin.Z||hull.Axis[2].Upper<p.Origin.Z)continue;
        if(hull.Axis[0].Upper<p.Origin.X-p.HalfExtents.X||hull.Axis[0].Lower>p.Origin.X+p.HalfExtents.X||
            hull.Axis[1].Upper<p.Origin.Y-p.HalfExtents.Y||hull.Axis[1].Lower>p.Origin.Y+p.HalfExtents.Y)continue;
        return false;
    }
    return true;
}
bool FPreparedExternalLandscape::EvaluateFixed(const Analytic::FWorldQuery& q,bool overlap,Analytic::FWorldHit& out) const
{
    out={};CPoint hull;
    if(!Analytic::Certified::BuildFixedWorldQueryVolumeBounds(q,hull))return false;
    if(!FMath::IsFinite(q.DomainTolerance)||q.DomainTolerance<0)return false;
    for(auto& axis:hull.Axis)axis=AnalyticCertified::Add(axis,AnalyticCertified::Interval{-q.DomainTolerance,q.DomainTolerance,true});
    for(const auto& row:Planes)if(Policy(q,row.Plane))
    {
        const auto& p=row.Plane;
        // Face kernel misses cannot exclude a finite plane's edge vicinity.
        // A complete query-volume interior (or strict whole-plane exclusion)
        // closes that gap; unsupported exterior edge contact remains Unknown.
        const bool inside=hull.Axis[0].Lower>p.Origin.X-p.HalfExtents.X&&hull.Axis[0].Upper<p.Origin.X+p.HalfExtents.X&&
            hull.Axis[1].Lower>p.Origin.Y-p.HalfExtents.Y&&hull.Axis[1].Upper<p.Origin.Y+p.HalfExtents.Y;
        if(!inside)
        {
            if(hull.Axis[2].Lower>p.Origin.Z||hull.Axis[2].Upper<p.Origin.Z||
                hull.Axis[0].Upper<p.Origin.X-p.HalfExtents.X||hull.Axis[0].Lower>p.Origin.X+p.HalfExtents.X||
                hull.Axis[1].Upper<p.Origin.Y-p.HalfExtents.Y||hull.Axis[1].Lower>p.Origin.Y+p.HalfExtents.Y)continue;
            return false;
        }
        Analytic::FWorldHit hit;if(!Analytic::FWorldQueryService::EvaluateFinitePlaneGeometry(q,p,overlap,hit))return false;
        if(hit.bHit&&(!out.bHit||(overlap?hit.PenetrationDepth>out.PenetrationDepth:hit.Time<out.Time)))out=MoveTemp(hit);
    }
    return true;
}
}
