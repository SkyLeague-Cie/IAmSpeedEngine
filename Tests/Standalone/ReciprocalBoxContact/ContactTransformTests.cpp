#include "IAmSpeed/World/Collision/ReciprocalBoxContact.h"
#include <cstdio>
using Speed::Collision::MakeReciprocalBoxContact;
static int failures=0;
static void Require(bool ok,const char* property){if(!ok){if(failures<4)std::printf("FAIL %s\n",property);++failures;}}
static bool SameEvent(const SHitResult& a,const SHitResult& b){
    return a.bHit==b.bHit&&a.bBlockingHit==b.bBlockingHit&&a.Location==b.Location&&a.ImpactPoint==b.ImpactPoint&&
    a.TOI==b.TOI&&a.FaceIndex==b.FaceIndex&&a.bStartPenetrating==b.bStartPenetrating&&a.PenetrationDepth==b.PenetrationDepth&&
    a.GeometricErrorBoundCm==b.GeometricErrorBoundCm&&a.bSurfaceNormalMayVary==b.bSurfaceNormalMayVary&&
    a.SourceId==b.SourceId&&a.SurfaceId==b.SurfaceId&&a.FeatureId==b.FeatureId&&a.PrimitiveId==b.PrimitiveId&&
    a.CanonicalGroupId==b.CanonicalGroupId&&a.MaterialId==b.MaterialId&&a.FrameTag==b.FrameTag;
}
static bool SameContact(const SHitResult& a,const SHitResult& b){
    return SameEvent(a,b)&&a.ImpactNormal==b.ImpactNormal&&a.Component.Get()==b.Component.Get()&&a.SubBody.Get()==b.SubBody.Get()&&
    a.ContactPointThis==b.ContactPointThis&&a.ContactPointOther==b.ContactPointOther&&a.ContactFeatureThis==b.ContactFeatureThis&&
    a.ContactFeatureOther==b.ContactFeatureOther&&a.ContactFeatureIndexThis==b.ContactFeatureIndexThis&&a.ContactFeatureIndexOther==b.ContactFeatureIndexOther;
}
int main(){
    USSubBody a,b;
    const FVector normals[]={{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{1,2,-3}};
    const int indices[]={-1,0,7};
    unsigned cases=0;
    for(int overlap=0;overlap<2;++overlap)for(const auto& normal:normals)
    for(int fa=0;fa<4;++fa)for(int fb=0;fb<4;++fb)for(int index:indices){
        SHitResult original;
        original.bHit=true;original.bBlockingHit=!overlap;original.bStartPenetrating=overlap!=0;
        original.ImpactNormal=normal;original.Component=&b;original.SubBody=&b;
        original.Location={2,5,11};original.ImpactPoint={13,-17,19};original.TOI=0.003f;
        original.ContactPointThis={31,37,41};original.ContactPointOther={-43,47,-53};
        original.ContactFeatureThis=static_cast<Speed::EContactFeatureKind>(fa);
        original.ContactFeatureOther=static_cast<Speed::EContactFeatureKind>(fb);
        original.ContactFeatureIndexThis=static_cast<int8>(index);original.ContactFeatureIndexOther=static_cast<int8>(6-index);
        original.PenetrationDepth=overlap?0.2f:0;original.FaceIndex=23;
        original.GeometricErrorBoundCm=0.125f;original.bSurfaceNormalMayVary=true;
        original.SourceId=101;original.SurfaceId=103;original.FeatureId=107;original.PrimitiveId=109;
        original.CanonicalGroupId=113;original.MaterialId=127;original.FrameTag=131;
        const SHitResult saved=original;
        const SHitResult reciprocal=MakeReciprocalBoxContact(original,&a,&a);
        Require(SameContact(original,saved),"input contact immutable");
        Require(SameEvent(original,reciprocal),"one contact retains event identity and witness time");
        Require(reciprocal.ImpactNormal==-original.ImpactNormal,"reciprocal normal has opposite orientation without normalization");
        Require(reciprocal.Component.Get()==&a&&reciprocal.SubBody.Get()==&a,"reciprocal target is originating body");
        Require(reciprocal.ContactPointThis==original.ContactPointOther&&reciprocal.ContactPointOther==original.ContactPointThis,
            "world witnesses follow their owning participant");
        Require(reciprocal.ContactFeatureThis==original.ContactFeatureOther&&reciprocal.ContactFeatureOther==original.ContactFeatureThis&&
            reciprocal.ContactFeatureIndexThis==original.ContactFeatureIndexOther&&reciprocal.ContactFeatureIndexOther==original.ContactFeatureIndexThis,
            "feature kind and index follow the same witness owner");
        Require(SameContact(original,MakeReciprocalBoxContact(reciprocal,&b,&b)),"exchanging participants twice is an involution");
        ++cases;
    }
    std::printf("CONTACT_TRANSFORM cases=%u failures=%d scope=production_header_with_storage_shims\n",cases,failures);
    return failures?1:0;
}
