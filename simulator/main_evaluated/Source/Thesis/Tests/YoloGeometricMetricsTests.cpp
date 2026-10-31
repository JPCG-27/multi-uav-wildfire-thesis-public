#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../Vision/YoloGeometricMetrics.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FYoloOneToOneMetricsTest, "Thesis.Vision.OneToOneGeometricMetrics",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FYoloOneToOneMetricsTest::RunTest(const FString& Parameters)
{
    auto Duplicate = YoloGeometricMetrics::Match({55,55,INDEX_NONE}, {.4f,.9f,1.0f}, {55}, 10);
    auto DuplicatePerson = YoloGeometricMetrics::MatchPersonIndices({55,55,INDEX_NONE}, {.4f,.9f,1.0f}, {55}, 10);
    TestFalse(TEXT("Duplicate low-confidence box is FP"), Duplicate[0]);
    TestTrue(TEXT("Highest confidence matches"), Duplicate[1]);
    TestFalse(TEXT("Invalid projection never matches"), Duplicate[2]);
    TestEqual(TEXT("Attribution preserves duplicate-box assignment"), DuplicatePerson[0], INDEX_NONE);
    TestEqual(TEXT("Attribution identifies the legacy duplicate-box winner"), DuplicatePerson[1], 0);
    TestEqual(TEXT("Attribution preserves invalid-projection rejection"), DuplicatePerson[2], INDEX_NONE);
    auto Collocated = YoloGeometricMetrics::Match({55,55,55}, {.9f,.8f,.7f}, {55,55}, 10);
    auto CollocatedPerson = YoloGeometricMetrics::MatchPersonIndices({55,55,55}, {.9f,.8f,.7f}, {55,55}, 10);
    TestTrue(TEXT("First collocated person matches"), Collocated[0]);
    TestTrue(TEXT("Second collocated person matches"), Collocated[1]);
    TestFalse(TEXT("Third box cannot invent another person"), Collocated[2]);
    TestEqual(TEXT("Collocated attribution keeps stable person order"), CollocatedPerson[0], 0);
    TestEqual(TEXT("Second collocated attribution uses next person"), CollocatedPerson[1], 1);
    TestEqual(TEXT("Third collocated box remains unmatched"), CollocatedPerson[2], INDEX_NONE);
    auto Tie = YoloGeometricMetrics::Match({55,54}, {.9f,.9f}, {54,56}, 10);
    auto TiePerson = YoloGeometricMetrics::MatchPersonIndices({55,54}, {.9f,.9f}, {54,56}, 10);
    TestTrue(TEXT("Stable first box wins tied nearest-person claim"), Tie[0]);
    TestFalse(TEXT("Person-ID tie break leaves distant candidate unavailable"), Tie[1]);
    TestEqual(TEXT("Tie attribution records same winning person as legacy metric"), TiePerson[0], 0);
    TestEqual(TEXT("Tie attribution leaves the second box unmatched"), TiePerson[1], INDEX_NONE);
    auto Edge = YoloGeometricMetrics::Match({9}, {.9f}, {10}, 10);
    auto EdgePerson = YoloGeometricMetrics::MatchPersonIndices({9}, {.9f}, {10}, 10);
    TestFalse(TEXT("Linear adjacent indices cannot wrap across grid edges"), Edge[0]);
    TestEqual(TEXT("Edge attribution does not wrap across grid edges"), EdgePerson[0], INDEX_NONE);
    auto Empty = YoloGeometricMetrics::Match({55}, {.9f}, {}, 10);
    TestFalse(TEXT("No in-frustum person means FP"), Empty[0]);
    return true;
}
#endif
