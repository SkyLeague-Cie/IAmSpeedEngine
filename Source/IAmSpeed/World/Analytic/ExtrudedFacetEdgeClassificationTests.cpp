#if WITH_DEV_AUTOMATION_TESTS
#include "AnalyticWorldQuery.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FIAmSpeedExtrudedFacetEdgeClassificationTest,
    "IAmSpeed.AnalyticWorld.ExtrudedFacetEdgeClassification",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FIAmSpeedExtrudedFacetEdgeClassificationTest::RunTest(const FString& Parameters)
{
    using namespace Speed::Analytic;
    const auto Expect = [this](const TCHAR* Name, const int32 Count, const int32 Segment,
        const int8 A, const int8 B, const EExtrudedFacetEdgeRole Expected)
    {
        TestEqual(Name, FWorldQueryService::ClassifyExtrudedFacetEdgeForTest(
            Count, Segment, A, B), Expected);
        TestEqual(FString::Printf(TEXT("%s winding invariant"), Name),
            FWorldQueryService::ClassifyExtrudedFacetEdgeForTest(Count, Segment, B, A), Expected);
    };
    Expect(TEXT("first start is real"),3,0,0,3,EExtrudedFacetEdgeRole::SectionStartBoundary);
    Expect(TEXT("first end is internal"),3,0,1,2,EExtrudedFacetEdgeRole::InternalSectionEnd);
    Expect(TEXT("middle start is internal"),3,1,0,3,EExtrudedFacetEdgeRole::InternalSectionStart);
    Expect(TEXT("middle end is internal"),3,1,1,2,EExtrudedFacetEdgeRole::InternalSectionEnd);
    Expect(TEXT("last start is internal"),3,2,0,3,EExtrudedFacetEdgeRole::InternalSectionStart);
    Expect(TEXT("last end is real"),3,2,1,2,EExtrudedFacetEdgeRole::SectionEndBoundary);
    Expect(TEXT("single start is real"),1,0,0,3,EExtrudedFacetEdgeRole::SectionStartBoundary);
    Expect(TEXT("single end is real"),1,0,1,2,EExtrudedFacetEdgeRole::SectionEndBoundary);
    Expect(TEXT("extrusion min remains real"),3,1,0,1,EExtrudedFacetEdgeRole::ExtrusionMinBoundary);
    Expect(TEXT("extrusion max remains real"),3,1,2,3,EExtrudedFacetEdgeRole::ExtrusionMaxBoundary);
    Expect(TEXT("first quad diagonal"),3,0,1,3,EExtrudedFacetEdgeRole::InternalTriangulationEdge);
    Expect(TEXT("opposite diagonal"),3,1,0,2,EExtrudedFacetEdgeRole::InternalTriangulationEdge);
    Expect(TEXT("two segment first end is internal"),2,0,1,2,EExtrudedFacetEdgeRole::InternalSectionEnd);
    Expect(TEXT("two segment last start is internal"),2,1,0,3,EExtrudedFacetEdgeRole::InternalSectionStart);
    for (const int32 Bad : {-1, 4})
        TestEqual(TEXT("invalid tag is unknown"), FWorldQueryService::ClassifyExtrudedFacetEdgeForTest(3,1,Bad,0),
            EExtrudedFacetEdgeRole::Unknown);
    TestEqual(TEXT("repeated tag is unknown"), FWorldQueryService::ClassifyExtrudedFacetEdgeForTest(3,1,0,0),
        EExtrudedFacetEdgeRole::Unknown);
    TestEqual(TEXT("invalid segment is unknown"), FWorldQueryService::ClassifyExtrudedFacetEdgeForTest(3,3,0,3),
        EExtrudedFacetEdgeRole::Unknown);
    return true;
}
#endif
