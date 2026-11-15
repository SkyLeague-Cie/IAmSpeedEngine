#pragma once
#include "IAmSpeed/World/Analytic/AnalyticWorldQuery.h"
#include "Engine/EngineTypes.h"
namespace Speed
{
struct FPreparedPhysicalFilterRecord
{
    const void* Owner=nullptr;const void* Body=nullptr;FString BodyPath;uint8 TraceChannel=0;uint64 BlockingObjectTypes=0;
    bool operator==(const FPreparedPhysicalFilterRecord& other) const
    {return Owner==other.Owner&&Body==other.Body&&BodyPath==other.BodyPath&&TraceChannel==other.TraceChannel&&BlockingObjectTypes==other.BlockingObjectTypes;}
};
// Filter-domain closure ONLY. No geometry, TOI, response, camera or physical
// authority follows from this packet. Owner/body pointers bind a single owned
// input generation; they never select contact ordering or authored source IDs.
class FPreparedPhysicalQueryDomain final
{
    uint64 Generation=0;TArray<FPreparedPhysicalFilterRecord> Records;TSet<const void*> Adapters;int32 EmptyAdapterCount=0;
    static bool Normalize(TArray<FPreparedPhysicalFilterRecord>& records,const TArray<const void*>& adapters)
    {
        if(adapters.IsEmpty())return false;TSet<const void*> members,bodies;TSet<FString> paths;
        for(const auto* adapter:adapters){if(!adapter||members.Contains(adapter))return false;members.Add(adapter);}
        for(const auto& row:records)
        {
            if(!members.Contains(row.Owner)||!row.Body||row.BodyPath.IsEmpty()||row.TraceChannel>=64||bodies.Contains(row.Body)||paths.Contains(row.BodyPath))return false;
            bodies.Add(row.Body);paths.Add(row.BodyPath);
        }
        records.Sort([](const auto& a,const auto& b){return a.BodyPath<b.BodyPath;});return true;
    }
    FPreparedPhysicalQueryDomain(uint64 generation,TArray<FPreparedPhysicalFilterRecord> records,const TArray<const void*>& adapters):Generation(generation),Records(MoveTemp(records))
    {TSet<const void*> owners;for(const auto* adapter:adapters)Adapters.Add(adapter);for(const auto& row:Records)owners.Add(row.Owner);EmptyAdapterCount=Adapters.Num()-owners.Num();}
public:
    static constexpr uint64 PhysicalObjectQueries=uint64(1)<<ECC_WorldStatic;
    static bool Build(uint64 generation,TArray<FPreparedPhysicalFilterRecord> records,const TArray<const void*>& adapters,TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe>& output)
    {
        output.Reset();if(!generation||!Normalize(records,adapters))return false;
        output=MakeShareable(new FPreparedPhysicalQueryDomain(generation,MoveTemp(records),adapters));return true;
    }
    uint64 GetGeneration() const {return Generation;}
    TConstArrayView<FPreparedPhysicalFilterRecord> GetRecords() const {return Records;}
    int32 GetAdapterCount() const {return Adapters.Num();}
    int32 GetEmptyAdapterCount() const {return EmptyAdapterCount;}
    // Membership is an unordered identity set; zero-body adapters are retained
    // without creating a fake query body or granting query coverage.
    bool Matches(TArray<FPreparedPhysicalFilterRecord> actual,const TArray<const void*>& adapters) const
    {
        if(!Normalize(actual,adapters)||actual!=Records||adapters.Num()!=Adapters.Num())return false;
        for(const auto* adapter:adapters)if(!Adapters.Contains(adapter))return false;
        return true;
    }
    // Object queries ignore trace responses. Trace queries need BOTH sides to
    // block. All64channels are retained; no Camera/Visibility/name exemption.
    static bool PrimitiveCanParticipate(uint32 objectType,uint64 blockingChannels,TConstArrayView<FPreparedPhysicalFilterRecord> records)
    {
        if(objectType>=64)return true;const uint64 object=uint64(1)<<objectType;
        if(PhysicalObjectQueries&object)return true;
        for(const auto& row:records)if(row.TraceChannel>=64||((blockingChannels&(uint64(1)<<row.TraceChannel))&&(row.BlockingObjectTypes&object)))return true;
        return false;
    }
    // Future consumers must bind the REAL purpose, generation and current body
    // and revalidate this filter before consuming any geometric ticket. A new,
    // unfiltered, wider, camera or unknown query is explicitly outside coverage.
    bool CoversPhysicalFilter(bool physicsPurpose,uint64 generation,const void* owner,const void* body,const Analytic::FWorldQuery& query) const
    {
        if(!physicsPurpose||generation!=Generation||!query.bApplyCollisionFilter)return false;
        const auto* row=Records.FindByPredicate([&](const auto& r){return r.Owner==owner&&r.Body==body;});if(!row)return false;
        if(query.bObjectQuery)return (query.ObjectTypes&~PhysicalObjectQueries)==0;
        return query.TraceChannel==row->TraceChannel&&(query.BlockingObjectTypes&~row->BlockingObjectTypes)==0;
    }
};
using FPreparedPhysicalQueryDomainPtr=TSharedPtr<const FPreparedPhysicalQueryDomain,ESPMode::ThreadSafe>;
}
