// Included after actual observation types and SameKinematicBits are defined.
// Compare the real primitive's returned values only. A match is not a model
// of the virtual setter/callback effects, velocity clamps, projection, Post,
// child sync, constraints, trajectory or an authorization to advance.
namespace OrdinaryPrimitiveDetail
{
void CloseValues(FOrdinaryIntegrationOwnerValues& row,float seconds,const FStepIdentity& current)
{
    ::PreparedFrameworkProfile1::FScope observedFramework(::PreparedFrameworkProfile1::PrimitiveClose);
    row.PrimitiveComparisons.Reset();row.PrimitiveStatus=EOrdinaryPrimitiveValuesStatus::Unknown;
    const auto fp=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(!row.Owner||!row.Opened||!row.Called||!row.Returned||row.Closed||row.Paths.Num()!=1||
        fp.Status!=Analytic::EAnalyticNumericStatus::Complete||fp.ControlFingerprint()!=current.NumericControl)return;
    const auto& path=row.Paths[0];
    if(!SameFloat(path.Delta,seconds))return;
    if(path.Path!=ENativeIntegrationPath::Transport)
    {
        for(const auto& stage:row.Stages)
            if(stage.Stage==ENativeIntegrationStage::PrimitiveEnter||stage.Stage==ENativeIntegrationStage::PrimitiveReturn)return;
        // Outside this checker, NOT a claim that no effects/Post occurred.
        row.PrimitiveStatus=EOrdinaryPrimitiveValuesStatus::OutsideTransport;return;
    }
    // The inherited implementation has at most four transport stages. This
    // bounds technical comparison work; no physical quality margin is added.
    const int32 count=path.TransportSteps,restore=row.RestorationAttempts;
    if(!path.ReactionsComputed||!path.SleepingComputed||path.SleepingResult||!path.TransportComputed||
        !FMath::IsFinite(path.Delta)||path.Delta<=0||count<1||count>4||
        (!path.RequiresTransport&&count!=1)||!FMath::IsFinite(path.TransportDelta)||path.TransportDelta<=0||
        !SameFloat(path.TransportDelta,path.Delta/static_cast<float>(count))||
        restore<0||restore>1||row.RestorationReturns!=restore||row.Stages.Num()!=3*count+1+restore)return;
    const auto stage=[&](int32 at,ENativeIntegrationStage kind,float delta,int32 index,int32 total)
    {
        if(!row.Stages.IsValidIndex(at))return false;const auto& value=row.Stages[at];
        return value.Captured&&value.Stage==kind&&SameFloat(value.Delta,delta)&&value.Index==index&&value.Count==total&&
            (kind==ENativeIntegrationStage::ProjectionReturn||!value.Projected)&&FiniteState(value.OwnerState)&&
            OrdinaryWorldDetail::Epoch(value.Step,current)&&value.Step.QuerySerial<=current.QuerySerial;
    };
    TArray<FOrdinaryPrimitiveComparisonValues> compared;compared.Reserve(count);bool equal=true;
    for(int32 i=0;i<count;++i)
    {
        const int32 at=3*i;
        if(!stage(at,ENativeIntegrationStage::PrimitiveEnter,path.TransportDelta,i,count)||
            !stage(at+1,ENativeIntegrationStage::PrimitiveReturn,path.TransportDelta,i,count)||
            !stage(at+2,ENativeIntegrationStage::ProjectionReturn,path.TransportDelta,i,count))return;
        const auto& entered=row.Stages[at];const auto& returned=row.Stages[at+1];
        if(entered.Step.QuerySerial!=returned.Step.QuerySerial)return;
        FOrdinaryPrimitiveComparisonValues value;value.EnterStage=at;value.ReturnStage=at+1;
        // EACH actual Enter is independent: previous projection/clamps may
        // have changed it. Never integrate row.Before or the previous Return.
        value.ExpectedAfter=entered.OwnerState.Integrate(entered.Delta);
        if(!FiniteState(value.ExpectedAfter))return;
        value.Equal=SameKinematicBits(value.ExpectedAfter,returned.OwnerState);equal=equal&&value.Equal;
        compared.Add(MoveTemp(value));
    }
    if(!stage(3*count,ENativeIntegrationStage::PostReturn,seconds,INDEX_NONE,0)||
        (restore&&!stage(3*count+1,ENativeIntegrationStage::ReactionRestored,seconds,INDEX_NONE,0)))return;
    const auto after=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
    if(after.Status!=Analytic::EAnalyticNumericStatus::Complete||after.ControlFingerprint()!=current.NumericControl)return;
    row.PrimitiveComparisons=MoveTemp(compared);
    row.PrimitiveStatus=equal?EOrdinaryPrimitiveValuesStatus::ValuesMatch:EOrdinaryPrimitiveValuesStatus::ValuesDiffer;
}
}

#if WITH_DEV_AUTOMATION_TESTS
bool TestOrdinaryPrimitiveValueClosure(FString& reason)
{
    reason.Reset();const uint32 controls=VectorGetControlRegister();ON_SCOPE_EXIT{VectorSetControlRegister(controls);};
    const auto fail=[&](const TCHAR* text){reason=text;return false;};
    for(uint32 flush:{0u,0x40u,0x8000u,0x8040u})
    {
        VectorSetControlRegister((controls&~uint32(0x8040))|flush);
        const auto fp=Analytic::InspectCurrentAnalyticNumericContext(Analytic::EAnalyticNumericContract::Binary64NearestAllowFlush);
        if(fp.Status!=Analytic::EAnalyticNumericStatus::Complete)return fail(TEXT("Primitive value fixture needs actual RNE controls"));
        FStepIdentity step;step.QuerySerial=7;step.NumericControl=fp.ControlFingerprint();
        FOrdinaryIntegrationOwnerValues good;uint8 opaqueOwner=0;
        good.Owner=reinterpret_cast<const ISpeedComponent*>(&opaqueOwner); // identity only, never dereferenced
        good.Opened=good.Called=good.Returned=true;good.RestorationAttempts=good.RestorationReturns=1;
        FNativeIntegrationPathValues path;path.Path=ENativeIntegrationPath::Transport;path.Delta=.25f;
        path.TransportComputed=path.ReactionsComputed=path.SleepingComputed=true;path.RequiresTransport=true;
        path.TransportSteps=2;path.TransportDelta=.125f;good.Paths.Add(path);
        const auto append=[&](ENativeIntegrationStage kind,float delta,int32 index,int32 count,const FKinematicState& state)
        {
            auto& stage=good.Stages.AddDefaulted_GetRef();stage.Stage=kind;stage.Delta=delta;stage.Index=index;stage.Count=count;
            stage.OwnerState=state;stage.Step=step;stage.Captured=true;
        };
        FKinematicState first;first.Location=FVector(1,2,3);first.Velocity=FVector(4,-2,1);first.Acceleration=FVector(2,1,-4);
        first.AngularVelocity=FVector(.25,-.125,.5);first.AngularAcceleration=FVector(.125,.25,-.25);
        const auto returned0=first.Integrate(.125f);
        append(ENativeIntegrationStage::PrimitiveEnter,.125f,0,2,first);
        append(ENativeIntegrationStage::PrimitiveReturn,.125f,0,2,returned0);
        append(ENativeIntegrationStage::ProjectionReturn,.125f,0,2,returned0);
        // A distinct Enter represents intervening clamp/projection. Requiring
        // equality with the previous Return would incorrectly reject it.
        auto second=returned0;second.Velocity=FVector(-3,2,1);
        append(ENativeIntegrationStage::PrimitiveEnter,.125f,1,2,second);
        const auto returned1=second.Integrate(.125f);
        append(ENativeIntegrationStage::PrimitiveReturn,.125f,1,2,returned1);
        append(ENativeIntegrationStage::ProjectionReturn,.125f,1,2,returned1);
        append(ENativeIntegrationStage::PostReturn,.25f,INDEX_NONE,0,returned1);
        append(ENativeIntegrationStage::ReactionRestored,.25f,INDEX_NONE,0,returned1);
        OrdinaryPrimitiveDetail::CloseValues(good,.25f,step);
        if(good.PrimitiveStatus!=EOrdinaryPrimitiveValuesStatus::ValuesMatch||good.PrimitiveComparisons.Num()!=2)
            return fail(TEXT("Independent actual Enter pairs did not close"));
        auto wrong=good;wrong.Stages[4].OwnerState.AngularVelocity.X+=1;
        OrdinaryPrimitiveDetail::CloseValues(wrong,.25f,step);
        if(wrong.PrimitiveStatus!=EOrdinaryPrimitiveValuesStatus::ValuesDiffer||wrong.PrimitiveComparisons.Num()!=2||wrong.PrimitiveComparisons[1].Equal)
            return fail(TEXT("Changed returned angular value was hidden"));
        for(int32 mutation=0;mutation<5;++mutation)
        {
            wrong=good;
            if(mutation==0)wrong.Stages[1].Delta=.25f;
            if(mutation==1)wrong.Stages[1].Step.NumericControl^=uint64(0x40);
            if(mutation==2)wrong.Stages[1].Step.QuerySerial=6;
            if(mutation==3)wrong.Stages.RemoveAt(1);
            if(mutation==4)wrong.RestorationReturns=0;
            OrdinaryPrimitiveDetail::CloseValues(wrong,.25f,step);
            if(wrong.PrimitiveStatus!=EOrdinaryPrimitiveValuesStatus::Unknown||!wrong.PrimitiveComparisons.IsEmpty())
                return fail(TEXT("Partial or stale primitive sequence retained a comparison closure"));
        }
        wrong=good;wrong.Paths.Reset();OrdinaryPrimitiveDetail::CloseValues(wrong,.25f,step);
        if(wrong.PrimitiveStatus!=EOrdinaryPrimitiveValuesStatus::Unknown||!wrong.PrimitiveComparisons.IsEmpty())
            return fail(TEXT("Missing actual path produced a vacuous match"));
        wrong=good;wrong.Paths[0].Path=ENativeIntegrationPath::Sleeping;wrong.Stages.RemoveAt(0,6);
        OrdinaryPrimitiveDetail::CloseValues(wrong,.25f,step);
        if(wrong.PrimitiveStatus!=EOrdinaryPrimitiveValuesStatus::OutsideTransport||!wrong.PrimitiveComparisons.IsEmpty())
            return fail(TEXT("Nontransport Post effects were mislabeled as matching a primitive"));
    }
    return true;
}
#endif
