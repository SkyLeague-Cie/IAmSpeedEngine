// Included once after owner-return definitions in the existing IAS TU.
// Closed VALUES of the executed World selector, never earliest physical
// geometry, dynamic/cache clearance, integration or response permission.
struct FOrdinaryWorldCandidate
{
    SComponentTOI Raw;FOrdinaryWorldFilterValues Filter;bool HasClamped=false;
    float Clamped=0;uint8 Generation=0;bool SortIdRead=false;uint32 ActualSortId=0;
};
struct FOrdinaryWorldReplayEntry {float TOI=0;int32 Row=INDEX_NONE;};
struct FOrdinaryWorldSelectionState
{
    FStepIdentity Entry,Completion;
    // Identity borrowing ONLY while World Step is alive. Destructors/reset
    // never read it: ResolvedPairs dies before FStepScope on stack unwinding.
    const Collision::FResolvedPairSet* PairsIdentity=nullptr;
    float Horizon=0,Before=0,Delta=0,Tolerance=0,Earliest=0,Seconds=0;
    TArray<FOrdinaryWorldCandidate> Rows;TArray<FOrdinaryWorldReplayEntry> Expected;
    TArray<SComponentTOI> Sorted;bool Closed=false;
};
namespace OrdinaryWorldDetail
{
bool Epoch(const FStepIdentity& a,const FStepIdentity& b)
{
    return a.Host==b.Host&&a.Bridge==b.Bridge&&a.World==b.World&&a.Worker==b.Worker&&a.Provider==b.Provider&&
        a.Lease==b.Lease&&a.InputGeneration==b.InputGeneration&&a.Frame==b.Frame&&a.StepSerial==b.StepSerial&&
        a.WindowSerial==b.WindowSerial&&a.WindowNonce==b.WindowNonce&&a.QueryEpoch==b.QueryEpoch&&a.Iteration==b.Iteration&&
        a.NumericControl==b.NumericControl&&a.ThreadId==b.ThreadId&&SameFloat(a.Remaining,b.Remaining)&&
        SameFloat(a.TimePassed,b.TimePassed)&&SameFloat(a.Delta,b.Delta)&&SameFloat(a.SimTime,b.SimTime);
}
bool Entry(const SComponentTOI& actual,const SComponentTOI& raw,float clamped)
{
    return actual.bHit==raw.bHit&&SameFloat(actual.TOI,clamped)&&actual.PairKey==raw.PairKey&&
        actual.PreparedEvidence==raw.PreparedEvidence&&actual.Resolver.HasSameIndexAndSerialNumber(raw.Resolver)&&
        SameNativeAnalyticContactValues(actual.Hit,raw.Hit);
}
}
bool FStepScope::HasOpenOrdinaryWorldSelection() const noexcept
{return OrdinaryReturns&&OrdinaryReturns->World&&!OrdinaryReturns->World->Closed;}
void FStepScope::BeginOrdinaryWorldSelection(const Collision::FResolvedPairSet& pairs,float horizon,float before,float delta,float tolerance)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(StepScope!=this||!SweepPhase||ComponentSweepScope||MemberSweepScope||!ReadCurrentStep(now)||
        !SameFloat(horizon,now.Remaining)||!SameFloat(before,now.TimePassed)||!SameFloat(delta,now.Delta)||
        !FMath::IsFinite(tolerance)||tolerance<0||!Registry)
        Refuse("Ordinary World selector entry differs from actual Step clock");
    if(!OrdinaryReturns)OrdinaryReturns=MakeUnique<FOrdinarySweepReturnState>();
    if(OrdinaryReturns->World||OrdinaryReturns->Pending||!OrdinaryReturns->Rows.IsEmpty())
        Refuse("Ordinary World selector entry duplicated or late");
    auto tx=MakeShared<FOrdinaryWorldSelectionState,ESPMode::ThreadSafe>();
    tx->Entry=now;tx->PairsIdentity=&pairs;tx->Horizon=horizon;tx->Before=before;tx->Delta=delta;
    tx->Tolerance=tolerance;tx->Earliest=horizon;OrdinaryReturns->World=MoveTemp(tx);
}
void FStepScope::ObserveOrdinaryWorldCandidate(const ISpeedComponent* owner,const SComponentTOI& actual,
    const Collision::FResolvedPairSet& pairs,const FOrdinaryWorldFilterValues& filter,bool clamped,float time,
    TConstArrayView<SComponentTOI> after,float earliest)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(StepScope!=this||!SweepPhase||ComponentSweepScope||MemberSweepScope||!ReadCurrentStep(now)||
        !OrdinaryReturns||!OrdinaryReturns->World||OrdinaryReturns->Pending)
        Refuse("Ordinary World candidate has no actual returned owner");
    auto& tx=*OrdinaryReturns->World;const int32 index=tx.Rows.Num();
    if(tx.Closed||tx.PairsIdentity!=&pairs||!OrdinaryWorldDetail::Epoch(tx.Entry,now)||
        OrdinaryReturns->Rows.Num()!=index+1||!Registry->GetOrderedAdapters().IsValidIndex(index)||
        Registry->GetOrderedAdapters()[index]!=owner)
        Refuse("Ordinary World candidate order/epoch differs");
    const auto& returned=OrdinaryReturns->Rows[index];const auto& bundle=returned->Component;
    if(!returned->Returned||returned->Owner!=owner||!bundle||!bundle->Sealed||actual.PreparedEvidence!=bundle||
        actual.bHit!=bundle->SelectedHit||!SameFloat(actual.TOI,bundle->SelectedSeconds)||
        (actual.bHit&&(actual.PairKey!=bundle->SelectedPairKey||!actual.Resolver.HasSameIndexAndSerialNumber(bundle->SelectedResolver)||
            !SameNativeAnalyticContactValues(actual.Hit,bundle->SelectedOrientedHit))))
        Refuse("Ordinary World raw Ctoi differs from actual component return");
    bool allowed=false;uint8 generation=0;
    if(!actual.bHit)
    {
        if(filter.ResolverValid||filter.EligibleRepeated||filter.Applied||filter.PairWasMember||filter.ProgressRead||
            filter.OrdinaryRead||filter.Allowed||clamped)Refuse("Ordinary World miss acquired an unexecuted filter/clamp");
    }
    else
    {
        if(filter.EligibleRepeated&&!filter.ResolverValid)Refuse("Ordinary repeated-contact virtual had no resolver");
        if(filter.Applied!=(actual.Hit.SurfaceId!=0))Refuse("Ordinary SurfaceId0 filter bypass changed");
        allowed=true;
        if(filter.Applied)
        {
            const bool member=pairs.Contains(actual.PairKey);generation=pairs.GetGeneration(actual.PairKey);
            const bool progress=member&&filter.EligibleRepeated;
            if(filter.PairWasMember!=member||filter.ProgressRead!=progress||filter.OrdinaryRead!=(!member))
                Refuse("Ordinary pair filter branch changed");
            // Keep the hit's ORIGINAL time separate from Ctoi's clamp time.
            allowed=member?filter.EligibleRepeated&&pairs.CanSelectExactProgress(actual.PairKey,actual.Hit.TOI,tx.Before,tx.Delta):
                pairs.CanResolve(actual.PairKey,filter.EligibleRepeated);
        }
        else if(filter.PairWasMember||filter.ProgressRead||filter.OrdinaryRead)
            Refuse("Ordinary dynamic bypass invented a pair-filter call");
        if(filter.Allowed!=allowed||clamped!=allowed)Refuse("Ordinary filter result or actual clamp presence changed");
        if(clamped&&(!FMath::IsFinite(time)||!SameFloat(time,FMath::Clamp(actual.TOI,0.f,tx.Horizon))))
            Refuse("Ordinary actual World clamp bits changed");
    }
    FOrdinaryWorldCandidate row;row.Raw=actual;row.Filter=filter;row.HasClamped=clamped;row.Clamped=time;row.Generation=generation;
    tx.Rows.Add(MoveTemp(row));
    if(allowed)Collision::AppendEarliestCollision(FOrdinaryWorldReplayEntry{time,index},time,tx.Tolerance,tx.Expected,tx.Earliest);
    if(!SameFloat(earliest,tx.Earliest)||after.Num()!=tx.Expected.Num())Refuse("Ordinary executed batch minimum/cardinality changed");
    for(int32 i=0;i<after.Num();++i)
    {
        const auto& expected=tx.Expected[i];
        if(!OrdinaryWorldDetail::Entry(after[i],tx.Rows[expected.Row].Raw,expected.TOI))
            Refuse("Ordinary executed batch payload/provenance order changed");
    }
}
void FStepScope::ObserveOrdinaryWorldSortKey(const SComponentTOI& actual,const USSubBody* raw,uint32 objectId)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(StepScope!=this||!ReadCurrentStep(now)||!SweepPhase||!OrdinaryReturns||!OrdinaryReturns->World)
        Refuse("Ordinary sort key is outside World selector");
    auto& tx=*OrdinaryReturns->World;
    if(tx.Closed||!OrdinaryWorldDetail::Epoch(tx.Entry,now)||tx.Rows.Num()!=Registry->GetOrderedAdapters().Num())
        Refuse("Ordinary sort key epoch or full owner inventory changed");
    FOrdinaryWorldCandidate* found=nullptr;
    // Read immutable candidate rows, never the array currently permuted by Sort.
    for(const auto& entry:tx.Expected)
    {
        auto& row=tx.Rows[entry.Row];
        if(actual.PreparedEvidence==row.Raw.PreparedEvidence)
        {
            if(found||!OrdinaryWorldDetail::Entry(actual,row.Raw,entry.TOI))Refuse("Ordinary sort candidate is ambiguous or changed");
            found=&row;
        }
    }
    const auto bundle=found?found->Raw.PreparedEvidence:FComponentSweepEvidencePtr{};
    if(!found||!bundle||raw!=bundle->SelectedRawResolver||!raw||objectId!=bundle->SelectedResolverGTObjectId||
        (found->SortIdRead&&found->ActualSortId!=objectId))Refuse("Ordinary actual sort ID/raw resolver differs from GT provenance");
    found->SortIdRead=true;found->ActualSortId=objectId;
}
void FStepScope::FinishOrdinaryWorldSelection(TConstArrayView<SComponentTOI> actual,float earliest,float seconds,
    float horizon,float before,float delta)
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(StepScope!=this||!ReadCurrentStep(now)||!SweepPhase||ComponentSweepScope||MemberSweepScope||!OrdinaryReturns||
        !OrdinaryReturns->World||OrdinaryReturns->Pending)Refuse("Ordinary selector cannot close inside another callback");
    auto& tx=*OrdinaryReturns->World;
    if(tx.Closed||!OrdinaryWorldDetail::Epoch(tx.Entry,now)||tx.Rows.Num()!=Registry->GetOrderedAdapters().Num()||
        OrdinaryReturns->Rows.Num()!=tx.Rows.Num()||actual.Num()!=tx.Expected.Num()||
        !SameFloat(horizon,tx.Horizon)||!SameFloat(before,tx.Before)||!SameFloat(delta,tx.Delta)||
        !SameFloat(earliest,tx.Earliest)||!SameFloat(seconds,FMath::Clamp(actual.IsEmpty()?horizon:earliest,0.f,horizon)))
        Refuse("Ordinary sorted selector minimum/clock/cardinality changed");
    TSet<int32> matched;const FOrdinaryWorldCandidate* previous=nullptr;
    for(const auto& value:actual)
    {
        int32 found=INDEX_NONE;
        for(const auto& entry:tx.Expected)if(value.PreparedEvidence==tx.Rows[entry.Row].Raw.PreparedEvidence)
        {
            if(found!=INDEX_NONE||!OrdinaryWorldDetail::Entry(value,tx.Rows[entry.Row].Raw,entry.TOI))
                Refuse("Ordinary sorted selector entry is ambiguous or changed");
            found=entry.Row;
        }
        if(found==INDEX_NONE||matched.Contains(found))Refuse("Ordinary sorted selector lost multiplicity/provenance");
        matched.Add(found);const auto& row=tx.Rows[found];
        if(previous&&(previous->Raw.PairKey>row.Raw.PairKey||
            (previous->Raw.PairKey==row.Raw.PairKey&&(!previous->SortIdRead||!row.SortIdRead||previous->ActualSortId>row.ActualSortId))))
            Refuse("Ordinary actual PairKey/ResolverId sorted order changed");
        previous=&row;tx.Sorted.Add(value);
    }
    tx.Seconds=seconds;tx.Completion=now;tx.Closed=true;
}
void FStepScope::CheckOrdinaryWorldSelection(TConstArrayView<FComponentSweepEvidencePtr> bundles,float seconds,
    bool hasSelection,TConstArrayView<SComponentTOI> actual,float horizon) const
{
    if(!Active||!Window||Window->GetPurpose()!=OnePausedWork::EPurpose::FullFrame)return;
    FStepIdentity now;
    if(StepScope!=this||!ReadCurrentStep(now)||!OrdinaryReturns||!OrdinaryReturns->World)Refuse("Ordinary advance has no executed selector values");
    const auto& tx=*OrdinaryReturns->World;
    if(!tx.Closed||!hasSelection||!OrdinaryWorldDetail::Epoch(tx.Completion,now)||now.QuerySerial!=tx.Completion.QuerySerial||
        !SameFloat(seconds,tx.Seconds)||!SameFloat(horizon,tx.Horizon)||bundles.Num()!=tx.Rows.Num()||actual.Num()!=tx.Sorted.Num())
        Refuse("Ordinary closed selector output is stale or different");
    for(int32 i=0;i<bundles.Num();++i)if(bundles[i]!=tx.Rows[i].Raw.PreparedEvidence)
        Refuse("Ordinary selector owner/bundle provenance changed");
    for(int32 i=0;i<actual.Num();++i)if(!OrdinaryWorldDetail::Entry(actual[i],tx.Sorted[i],tx.Sorted[i].TOI))
        Refuse("Ordinary advance selected output differs from actually sorted batch");
    // This only closes selector VALUE transport. RequireAdvanceEvidence keeps
    // every existing geometry/dynamic/cache/trajectory/effect refusal below it.
}
