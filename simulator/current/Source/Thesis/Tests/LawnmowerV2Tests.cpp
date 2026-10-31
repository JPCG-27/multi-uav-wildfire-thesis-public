#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../Drone/LawnmowerV2Planner.h"
#include "../Drone/DronePlannerFactory.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLawnmowerV2LaneGeometryTest,
    "Thesis.Planner.LawnmowerV2.FovLaneGeometry",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLawnmowerV2LaneGeometryTest::RunTest(const FString& Parameters)
{
    TestEqual(TEXT("Four cell range at 90 degree FOV gives four-cell lane spacing"),
        ULawnmowerV2Planner::ComputeLaneStride(4.0f, 90.0f, 30, 20), 4);
    TestEqual(TEXT("Narrow range clamps spacing to one cell"),
        ULawnmowerV2Planner::ComputeLaneStride(0.5f, 60.0f, 30, 20), 1);

    // Rows 0, 4, 8 and the far edge row 9 are always considered, including
    // sparse ownership and both initial corner directions.
    TArray<int32> Owned = { 0, 3, 4 * 5 + 1, 4 * 5 + 4, 8 * 5 + 2, 9 * 5 + 0, 9 * 5 + 4,
                            1, 2 * 5 + 3 };
    const TArray<int32> FromBottomLeft = ULawnmowerV2Planner::BuildSweepOrder(Owned, 5, 10, 4, false, false);
    const TArray<int32> Expected = { 0, 1, 3, 24, 21, 42, 49, 45 };
    TestTrue(TEXT("Only owned cells on the globally anchored FOV lanes are visited"), FromBottomLeft == Expected);

    const TArray<int32> FromTopRight = ULawnmowerV2Planner::BuildSweepOrder(Owned, 5, 10, 4, true, true);
    const TArray<int32> ExpectedReverse = { 49, 45, 42, 24, 21, 0, 1, 3 };
    TestTrue(TEXT("Top-right start reverses global lane traversal with alternating directions"),
        FromTopRight == ExpectedReverse);
    TestEqual(TEXT("Invalid dimensions return no candidates"),
        ULawnmowerV2Planner::BuildSweepOrder(Owned, 0, 10, 4, false, false).Num(), 0);

    TestTrue(TEXT("V1 remains registered and available"),
        FDronePlannerFactory::CreatePlannerClass(FName(TEXT("LawnmowerV1"))) != nullptr);
    TestTrue(TEXT("V2 is independently registered"),
        FDronePlannerFactory::CreatePlannerClass(FName(TEXT("LawnmowerV2"))) == ULawnmowerV2Planner::StaticClass());

    TArray<uint8> EpochOne = { 0, 1, 0 };
    TArray<uint8> LocalCompleted = { 0, 0, 0 };
    TestFalse(TEXT("A completed waypoint remains skipped after an ownership transfer"),
        ULawnmowerV2Planner::IsWaypointAvailable(1, &EpochOne, LocalCompleted));
    TestTrue(TEXT("An unfinished waypoint remains eligible after an ownership transfer"),
        ULawnmowerV2Planner::IsWaypointAvailable(2, &EpochOne, LocalCompleted));
    EpochOne.Init(0, 3); // shared epoch reset permits revisiting prior waypoints
    TestTrue(TEXT("Clearing the shared epoch makes prior waypoints eligible again"),
        ULawnmowerV2Planner::IsWaypointAvailable(1, &EpochOne, LocalCompleted));
    return true;
}

#endif
