// Consumes existing before/after snapshots; no observer or virtual getter is
// added. All pointer/weak values are compared without resolving/dereferencing.
namespace OrdinarySetFutureDetail
{
bool Bind(const FOrdinaryConsumedStateValues& before,const USSubBody* target,const SHitResult& hit,
    FOrdinarySetFutureHitValueClosure& out)
{
    const SHitResult expected=hit;out={}; // permit an argument borrowed from prior output
    if(!before.Captured||!target||expected.FirstRsResponseData||before.Owners.IsEmpty())return false;
    TSet<const ISpeedComponent*> owners;TSet<uint64> stable;TSet<const USSubBody*> bodies;TSet<uint32> objectIds;
    int32 oi=INDEX_NONE,bi=INDEX_NONE;
    for(int32 i=0;i<before.Owners.Num();++i)
    {
        const auto& o=before.Owners[i];
        if(!o.Owner||!o.StableId||owners.Contains(o.Owner)||stable.Contains(o.StableId))return false;
        owners.Add(o.Owner);stable.Add(o.StableId);
        for(int32 j=0;j<o.Bodies.Num();++j)
        {
            const auto& b=o.Bodies[j];
            // Captured=true came from the reader that checked ObjectIdCaptured.
            // Numeric ID zero is valid; only duplicate IDs are ambiguous.
            if(!b.Body||!b.Class||b.Index!=j||bodies.Contains(b.Body)||objectIds.Contains(b.GTObjectId))return false;
            bodies.Add(b.Body);objectIds.Add(b.GTObjectId);
            if(b.Body==target)
            {
                if(oi!=INDEX_NONE||b.Current.FirstRsResponseData||b.Future.FirstRsResponseData)return false;
                oi=i;bi=j;
            }
        }
    }
    if(oi==INDEX_NONE)return false;
    out.Body=target;out.OwnerIndex=oi;out.BodyIndex=bi;out.ExpectedFuture=expected;out.BoundBeforeCall=true;return true;
}
bool Constraint(const FPhysicalContactConstraint& a,const FPhysicalContactConstraint& b)
{
    return SameVector(a.Normal,b.Normal)&&SameVector(a.ContactPoint,b.ContactPoint)&&
        a.OtherComponent.HasSameIndexAndSerialNumber(b.OtherComponent)&&a.SourceSubBody.HasSameIndexAndSerialNumber(b.SourceSubBody)&&
        SameFloat(a.PenetrationDepth,b.PenetrationDepth)&&SameFloat(a.TOI,b.TOI)&&a.PairKey==b.PairKey&&
        a.FramesSinceSeen==b.FramesSinceSeen&&a.bPersistent==b.bPersistent;
}
bool Body(const FOrdinaryConsumedBodyValues& a,const FOrdinaryConsumedBodyValues& b,const SHitResult& future)
{
    return a.Body==b.Body&&a.Class==b.Class&&a.Index==b.Index&&a.GTObjectId==b.GTObjectId&&
        SameKinematicBits(a.State,b.State)&&SameNativeAnalyticContactValues(a.Current,b.Current)&&
        SameNativeAnalyticContactValues(future,b.Future)&&a.Ignored==b.Ignored&&
        SameVector(a.LocalOffset,b.LocalOffset)&&SameQuat(a.LocalRotation,b.LocalRotation)&&SameVector(a.BoxExtent,b.BoxExtent)&&
        a.SolidConfigCaptured==b.SolidConfigCaptured&&a.SphereShapeCaptured==b.SphereShapeCaptured&&a.BoxShapeCaptured==b.BoxShapeCaptured&&
        SameNativeSolidResponseConfigValues(a.Config,b.Config)&&SameFloat(a.PhysicalRadius,b.PhysicalRadius)&&
        AnalyticCertified::Bits(a.SupportRadius)==AnalyticCertified::Bits(b.SupportRadius)&&
        AnalyticCertified::Bits(a.ContactOffset)==AnalyticCertified::Bits(b.ContactOffset);
}
// Exact native zero-duration return: no transport, Post, projection or callback.
// This comparator grants no motion or contact; every captured value must stay equal.
bool ExactZeroDelta(float value) noexcept
{
    uint32 bits=0;FMemory::Memcpy(&bits,&value,sizeof(bits));return (bits&0x7fffffffu)==0;
}
bool Unchanged(const FOrdinaryConsumedStateValues& before,const FOrdinaryConsumedStateValues& after)
{
    if(!before.Captured||!after.Captured||before.Owners.IsEmpty()||before.Owners.Num()!=after.Owners.Num()||
        !OrdinaryWorldDetail::Epoch(before.Step,after.Step)||before.Step.QuerySerial!=after.Step.QuerySerial)return false;
    for(int32 i=0;i<before.Owners.Num();++i)
    {
        const auto& a=before.Owners[i];const auto& b=after.Owners[i];
        if(a.Owner!=b.Owner||a.StableId!=b.StableId||!(a.Clock==b.Clock)||!SameKinematicBits(a.State,b.State)||
            a.Frozen!=b.Frozen||a.Prescribed!=b.Prescribed||a.SleepingEligibility!=b.SleepingEligibility||
            !SameVector(a.LocalCOM,b.LocalCOM)||!SameQuat(a.PhysRotation,b.PhysRotation)||
            a.Constraints.Num()!=b.Constraints.Num()||a.Bodies.Num()!=b.Bodies.Num())return false;
        for(int32 k=0;k<a.Constraints.Num();++k)if(!Constraint(a.Constraints[k],b.Constraints[k]))return false;
        for(int32 j=0;j<a.Bodies.Num();++j)
            if(!Body(a.Bodies[j],b.Bodies[j],a.Bodies[j].Future))return false;
    }
    return true;
}
bool Match(const FOrdinaryConsumedStateValues& before,const FOrdinaryConsumedStateValues& after,
    const FOrdinarySetFutureHitValueClosure& expected)
{
    if(!expected.BoundBeforeCall||!before.Captured||!after.Captured||before.Owners.Num()!=after.Owners.Num()||
        !OrdinaryWorldDetail::Epoch(before.Step,after.Step)||before.Step.QuerySerial!=after.Step.QuerySerial||
        !before.Owners.IsValidIndex(expected.OwnerIndex)||!before.Owners[expected.OwnerIndex].Bodies.IsValidIndex(expected.BodyIndex)||
        before.Owners[expected.OwnerIndex].Bodies[expected.BodyIndex].Body!=expected.Body)return false;
    // Recheck the exact precondition/unique target from the immutable Before;
    // no later annotation may silently change which native branch was assumed.
    FOrdinarySetFutureHitValueClosure rebound;
    if(!Bind(before,expected.Body,expected.ExpectedFuture,rebound)||rebound.OwnerIndex!=expected.OwnerIndex||
        rebound.BodyIndex!=expected.BodyIndex)return false;
    for(int32 i=0;i<before.Owners.Num();++i)
    {
        const auto& a=before.Owners[i];const auto& b=after.Owners[i];
        if(a.Owner!=b.Owner||a.StableId!=b.StableId||!(a.Clock==b.Clock)||!SameKinematicBits(a.State,b.State)||
            a.Frozen!=b.Frozen||a.Prescribed!=b.Prescribed||a.SleepingEligibility!=b.SleepingEligibility||
            !SameVector(a.LocalCOM,b.LocalCOM)||!SameQuat(a.PhysRotation,b.PhysRotation)||
            a.Constraints.Num()!=b.Constraints.Num()||a.Bodies.Num()!=b.Bodies.Num())return false;
        for(int32 k=0;k<a.Constraints.Num();++k)if(!Constraint(a.Constraints[k],b.Constraints[k]))return false;
        for(int32 j=0;j<a.Bodies.Num();++j)
            if(!Body(a.Bodies[j],b.Bodies[j],i==expected.OwnerIndex&&j==expected.BodyIndex?expected.ExpectedFuture:a.Bodies[j].Future))return false;
    }
    return true;
}
}
#if WITH_DEV_AUTOMATION_TESTS
bool TestOrdinarySetFutureHitValueClosure(FString& reason)
{
    reason.Reset();const auto fail=[&](const TCHAR* why){reason=why;return false;};
    // Identity-only mathematical tokens, never UObjects, gameplay profiles,
    // poses, compiled class coverage, a real call or execution permissions.
    struct alignas(64) FIdentityToken {uint8 Value=0;};FIdentityToken tokens[8];
    FOrdinaryConsumedStateValues before;before.Captured=true;
    for(int32 i=0;i<2;++i)
    {
        auto& o=before.Owners.AddDefaulted_GetRef();o.Owner=reinterpret_cast<const ISpeedComponent*>(&tokens[i]);o.StableId=uint64(i+1);
        o.Constraints.AddDefaulted();
        for(int32 j=0;j<2;++j)
        {
            auto& b=o.Bodies.AddDefaulted_GetRef();b.Body=reinterpret_cast<const USSubBody*>(&tokens[2+2*i+j]);
            b.Class=&tokens[6];b.Index=j;b.GTObjectId=uint32(2*i+j+1);
            b.SolidConfigCaptured=true;b.Config.BodyMass=2;b.Config.OwnerPhysMass=5;
        }
    }
    if(!OrdinarySetFutureDetail::Unchanged(before,before))return fail(TEXT("Exact no-motion values rejected"));
    float subnormal=0;const uint32 tinyBits=1;FMemory::Memcpy(&subnormal,&tinyBits,sizeof(subnormal));
    if(!OrdinarySetFutureDetail::ExactZeroDelta(0.f)||!OrdinarySetFutureDetail::ExactZeroDelta(-0.f)||
        OrdinarySetFutureDetail::ExactZeroDelta(subnormal)||OrdinarySetFutureDetail::ExactZeroDelta(1.f))
        return fail(TEXT("No-motion delta accepted a nonzero bit pattern"));
    for(int32 mutation=0;mutation<8;++mutation)
    {
        auto changed=before;auto& o=changed.Owners[1];auto& b=o.Bodies[0];
        switch(mutation)
        {
        case 0:o.State.Velocity.X=1;break;
        case 1:o.LocalCOM.X=-0.;break;
        case 2:o.Constraints[0].PairKey=1;break;
        case 3:b.Future.FrameTag=1;break;
        case 4:b.Current.bHit=true;break;
        case 5:b.Ignored.Add(nullptr);break;
        case 6:b.BoxExtent.X=1;break;
        case 7:b.GTObjectId+=10;break;
        }
        if(OrdinarySetFutureDetail::Unchanged(before,changed))return fail(TEXT("No-motion closure hid an actual value change"));
    }
    const auto* target=before.Owners[0].Bodies[1].Body;SHitResult selected;selected.bHit=selected.bBlockingHit=true;selected.TOI=.25f;selected.SourceId=7;
    FOrdinarySetFutureHitValueClosure expected;
    if(!OrdinarySetFutureDetail::Bind(before,target,selected,expected))return fail(TEXT("unannotated complete value fixture did not bind"));
    auto after=before;after.Owners[0].Bodies[1].Future=selected;
    if(!OrdinarySetFutureDetail::Match(before,after,expected))return fail(TEXT("unique Future assignment rejected"));
    if(!OrdinarySetFutureDetail::Bind(before,target,expected.ExpectedFuture,expected)||
        !OrdinarySetFutureDetail::Match(before,after,expected))return fail(TEXT("expected hit alias lifetime lost"));
    for(int32 mutation=0;mutation<19;++mutation)
    {
        auto changed=after;auto& o=changed.Owners[1];auto& b=o.Bodies[0];
        switch(mutation)
        {
        case 0:o.State.AngularVelocity.X=1;break;
        case 1:o.LocalCOM.X=-0.;break;
        case 2:o.PhysRotation.X=-0.;break;
        case 3:o.SleepingEligibility=true;break;
        case 4:o.Constraints[0].PairKey=1;break;
        case 5:o.Constraints[0].FramesSinceSeen=1;break;
        case 6:b.Ignored.Add(nullptr);break;
        case 7:b.Config.CachedBodyLocalInvInertia.M[0][1]=-0.;break;
        case 8:b.BoxExtent.X=1;break;
        case 9:b.SupportRadius=-0.;break;
        case 10:b.Future.FrameTag=1;break;
        case 11:b.Current.bHit=true;break;
        case 12:changed.Step.QuerySerial=1;break;
        case 13:changed.Captured=false;break;
        case 14:changed.Owners[0].Bodies[1].Current.bHit=true;break;
        case 15:changed.Owners[0].Bodies[1].Future=before.Owners[0].Bodies[1].Future;break;
        case 16:o.Bodies.Swap(0,1);break;
        case 17:o.Clock.OwnerLocalFrame=1;break;
        case 18:b.SphereShapeCaptured=true;break;
        }
        if(OrdinarySetFutureDetail::Match(before,changed,expected))return fail(TEXT("extra owner/body/clock/config/cache mutation escaped exact footprint"));
    }
    auto zeroId=before;zeroId.Owners[0].Bodies[1].GTObjectId=0;
    auto zeroAfter=zeroId;zeroAfter.Owners[0].Bodies[1].Future=selected;
    if(!OrdinarySetFutureDetail::Bind(zeroId,target,selected,expected)||
        !OrdinarySetFutureDetail::Match(zeroId,zeroAfter,expected))return fail(TEXT("captured numeric ID zero was rejected"));
    auto duplicateZero=zeroId;duplicateZero.Owners[1].Bodies[0].GTObjectId=0;
    if(OrdinarySetFutureDetail::Bind(duplicateZero,target,selected,expected)||expected.BoundBeforeCall)
        return fail(TEXT("duplicate numeric zero IDs retained an expectation"));
    auto incomplete=before;incomplete.Captured=false;
    if(OrdinarySetFutureDetail::Bind(incomplete,target,selected,expected)||expected.BoundBeforeCall)return fail(TEXT("incomplete input retained an expectation"));
    auto duplicate=before;duplicate.Owners[1].Bodies[0].Body=target;
    if(OrdinarySetFutureDetail::Bind(duplicate,target,selected,expected)||expected.BoundBeforeCall)return fail(TEXT("duplicate resolver retained an expectation"));
    return true;
}
#endif
