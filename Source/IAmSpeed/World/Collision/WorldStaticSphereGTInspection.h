#pragma once
#include "CoreMinimal.h"
#include "IAmSpeed/Base/Kinematic.h"
namespace Speed
{
// Values read by the actual sphere member on GT. NOT a native request,
// lifetime lease, initial-contact witness, support or permission to mutate.
// The closed caller separately owns pins + parked-worker publication lease.
struct FWorldStaticSphereGTInspection
{
    float PhysicalRadius=0,StaticContactOffset=0,VirtualMargin=0,ActualQueryRadius=0;
    double SupportRadius=0;
    FKinematicState BodyState,OwnerState;
    FVector LocalOffset=FVector::ZeroVector;FQuat LocalRotation=FQuat::Identity;
    uint64 OwnerFrame=0;bool OwnerFrozen=false;
    // Additional production initial-operation observations. Existing 52 shape/
    // state words stay unchanged; the complete operation encoding is separate.
    bool NoInitialContactCaches=false,IgnoreBlocks=false;
    uint8 TraceChannel=0;uint64 BlockingObjectTypes=0;
    TArray<uint32> IgnoredSourceObjectIds,IgnoredComponentIds;
    TArray<uint64> ExactInitialOperationBits() const
    {
        auto out=ExactValueBits();out.Add(NoInitialContactCaches?1:0);out.Add(IgnoreBlocks?1:0);
        out.Add(TraceChannel);out.Add(BlockingObjectTypes);
        out.Add(IgnoredSourceObjectIds.Num());for(uint32 id:IgnoredSourceObjectIds)out.Add(id);
        out.Add(IgnoredComponentIds.Num());for(uint32 id:IgnoredComponentIds)out.Add(id);return out;
    }
    TArray<uint64> ExactValueBits() const
    {
        TArray<uint64> out;out.Reserve(53);
        auto f=[&](float v){uint32 b=0;FMemory::Memcpy(&b,&v,4);out.Add(b);};
        auto d=[&](double v){uint64 b=0;FMemory::Memcpy(&b,&v,8);out.Add(b);};
        auto vec=[&](const FVector& v){d(v.X);d(v.Y);d(v.Z);};
        auto quat=[&](const FQuat& q){d(q.X);d(q.Y);d(q.Z);d(q.W);};
        auto state=[&](const FKinematicState& s){vec(s.Location);vec(s.Velocity);vec(s.Acceleration);quat(s.Rotation);vec(s.AngularVelocity);vec(s.AngularAcceleration);};
        f(PhysicalRadius);f(StaticContactOffset);f(VirtualMargin);f(ActualQueryRadius);d(SupportRadius);
        state(BodyState);state(OwnerState);vec(LocalOffset);quat(LocalRotation);out.Add(OwnerFrame);out.Add(OwnerFrozen?1:0);return out;
    }
};
}
