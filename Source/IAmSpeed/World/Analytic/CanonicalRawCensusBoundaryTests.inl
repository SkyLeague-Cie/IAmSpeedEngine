#include "IAmSpeed/World/Collision/PreparedPhysicalExecution.h"
#include "IAmSpeed/World/Collision/PreparedNativeOwnerBindings.h"
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCanonicalRawCensusBoundaryTest,
    "IAmSpeed.AnalyticWorld.NativeSolid.CanonicalRawCensusStorageBoundary",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FCanonicalRawCensusBoundaryTest::RunTest(const FString&)
{
    using namespace Speed;using namespace Speed::PreparedExecution;
    // Pure raw-metadata tokens, never constructed/dereferenced as actors,
    // components or bodies. This inspector intentionally reads pointers only.
    alignas(16) uint64 tokens[200]={};
    for(int32 count:{64,65})
    {
        TArray<ISpeedComponent*> adapters,ordered;TArray<FPreparedNativeOwnerBinding> rows;
        for(int32 i=0;i<count;++i)
        {
            auto* owner=reinterpret_cast<ISpeedComponent*>(&tokens[i]);
            adapters.Add(owner);ordered.Add(owner);
            auto& row=rows.AddDefaulted_GetRef();row.Adapter=owner;
            row.NativeObject=reinterpret_cast<const UActorComponent*>(&tokens[i]);
            row.Class=&tokens[199];row.BodyCensusCaptured=true;
            auto& body=row.Bodies.AddDefaulted_GetRef();body.Index=0;
            body.Body=reinterpret_cast<const USSubBody*>(&tokens[100+i]);
            body.Class=&tokens[199];body.ObjectId=uint32(i+1);body.ObjectIdCaptured=true;
        }
        const auto inspect=[&](bool pending=false,bool dirty=false)
        {return InspectCanonicalCensusForRefusal(rows,adapters,ordered,pending,dirty);};
        TestTrue(TEXT("64/65 valid inventory remains a structural refusal, never physical coverage"),
            inspect()==ECanonicalCensusRefusal::NoCompiledPhaseCoverage);
        TestTrue(TEXT("pending mutation retains priority"),inspect(true,true)==ECanonicalCensusRefusal::PendingRegistry);
        TestTrue(TEXT("dirty order remains refused"),inspect(false,true)==ECanonicalCensusRefusal::DirtyOrder);
        rows.Last().Bodies[0].ObjectId=rows[0].Bodies[0].ObjectId;
        TestTrue(TEXT("duplicate child id after storage boundary refused"),inspect()==ECanonicalCensusRefusal::InvalidChildCensus);
        rows.Last().Bodies[0].ObjectId=uint32(count);
        rows.Last().Bodies[0].Body=rows[0].Bodies[0].Body;
        TestTrue(TEXT("duplicate child pointer after storage boundary refused"),inspect()==ECanonicalCensusRefusal::InvalidChildCensus);
        rows.Last().Bodies[0].Body=reinterpret_cast<const USSubBody*>(&tokens[100+count-1]);
        ordered.Last()=ordered[0];
        TestTrue(TEXT("duplicate final ordered owner refused"),inspect()==ECanonicalCensusRefusal::IncompleteInventory);
        ordered.Last()=adapters.Last();adapters.Last()=adapters[0];
        TestTrue(TEXT("duplicate final adapter refused"),inspect()==ECanonicalCensusRefusal::DuplicateOwner);
    }
    return !HasAnyErrors();
}
