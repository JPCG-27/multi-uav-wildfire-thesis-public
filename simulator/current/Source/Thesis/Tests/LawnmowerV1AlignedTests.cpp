#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../Drone/LawnmowerV1AlignedPlanner.h"
#include "../Drone/DronePlannerFactory.h"
#include "../Benchmark/MethodSpec.h"
#include "../Partition/SpawnAwareUniformPartition.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLawnmowerV1AlignedSweepOrderTest,
    "Thesis.Planner.LawnmowerV1Aligned.SpawnOrientedCoverage",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLawnmowerV1AlignedSweepOrderTest::RunTest(const FString& Parameters)
{
    TArray<int32> Owned;
    for (int32 Cell = 0; Cell < 12; ++Cell) Owned.Add(Cell);

    const TArray<int32> BottomLeft = ULawnmowerV1AlignedPlanner::BuildSweepOrder(Owned, 4, 3, false, false);
    const TArray<int32> ExpectedBottomLeft = { 0, 1, 2, 3, 7, 6, 5, 4, 8, 9, 10, 11 };
    TestTrue(TEXT("Bottom-left starts bottom-up and eastward, then alternates X direction"),
        BottomLeft == ExpectedBottomLeft);

    const TArray<int32> BottomRight = ULawnmowerV1AlignedPlanner::BuildSweepOrder(Owned, 4, 3, false, true);
    const TArray<int32> ExpectedBottomRight = { 3, 2, 1, 0, 4, 5, 6, 7, 11, 10, 9, 8 };
    TestTrue(TEXT("Bottom-right starts bottom-up and westward"), BottomRight == ExpectedBottomRight);

    const TArray<int32> TopRight = ULawnmowerV1AlignedPlanner::BuildSweepOrder(Owned, 4, 3, true, true);
    const TArray<int32> ExpectedTopRight = { 11, 10, 9, 8, 4, 5, 6, 7, 3, 2, 1, 0 };
    TestTrue(TEXT("Top-right starts top-down and westward"), TopRight == ExpectedTopRight);

    const TArray<int32> TopLeft = ULawnmowerV1AlignedPlanner::BuildSweepOrder(Owned, 4, 3, true, false);
    const TArray<int32> ExpectedTopLeft = { 8, 9, 10, 11, 7, 6, 5, 4, 0, 1, 2, 3 };
    TestTrue(TEXT("Top-left starts top-down and eastward"), TopLeft == ExpectedTopLeft);

    const TArray<int32> Sparse = ULawnmowerV1AlignedPlanner::BuildSweepOrder(
        TArray<int32>{ 11, 0, 3, 4, 8, 3, -1, 12 }, 4, 3, true, true);
    const TArray<int32> ExpectedSparse = { 11, 8, 4, 3, 0 };
    TestTrue(TEXT("Sparse ownership is deduplicated and invalid cells are ignored"), Sparse == ExpectedSparse);
    TestEqual(TEXT("Top-right ownership changes resume at the same oriented rank"),
        ULawnmowerV1AlignedPlanner::FindIndexAtOrAfterRank(TopRight, 4, 3, 6, true, true), 6);
    TestEqual(TEXT("Invalid grid dimensions yield no route"),
        ULawnmowerV1AlignedPlanner::BuildSweepOrder(Owned, 0, 3, false, false).Num(), 0);

    // The corner cell is the first route point for each spawn orientation.
    TestEqual(TEXT("Bottom-left initial target is its own corner cell"), BottomLeft[0], 0);
    TestEqual(TEXT("Bottom-right initial target is its own corner cell"), BottomRight[0], 3);
    TestEqual(TEXT("Top-right initial target is its own corner cell"), TopRight[0], 11);
    TestEqual(TEXT("Top-left initial target is its own corner cell"), TopLeft[0], 8);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLawnmowerV1AlignedSpawnPartitionTest,
    "Thesis.Planner.LawnmowerV1Aligned.SpawnAwareStaticPartition",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLawnmowerV1AlignedSpawnPartitionTest::RunTest(const FString& Parameters)
{
    const TArray<FVector2f> Spawns = {
        FVector2f(0.5f, 0.5f), FVector2f(99.5f, 0.5f),
        FVector2f(99.5f, 99.5f), FVector2f(0.5f, 99.5f)
    };
    const TArray<uint16> Owners = FSpawnAwareUniformPartition::BuildPartition(100, 100, Spawns);
    TestEqual(TEXT("Partition covers the complete grid"), Owners.Num(), 10000);
    const int32 CornerCells[] = { 0, 99, 9999, 9900 };
    int32 Counts[4] = { 0, 0, 0, 0 };
    for (int32 Cell = 0; Cell < Owners.Num(); ++Cell)
    {
        TestTrue(TEXT("Owner is a valid drone index"), Owners[Cell] < 4);
        if (Owners[Cell] < 4) ++Counts[Owners[Cell]];
    }
    for (int32 Drone = 0; Drone < 4; ++Drone)
    {
        TestEqual(FString::Printf(TEXT("Drone %d owns its spawn corner"), Drone),
            static_cast<int32>(Owners[CornerCells[Drone]]), Drone);
        TestEqual(FString::Printf(TEXT("Drone %d has one quarter of the grid"), Drone), Counts[Drone], 2500);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLawnmowerV1AlignedMethodSpecTest,
    "Thesis.Planner.LawnmowerV1Aligned.MethodSpecsAndFactory",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLawnmowerV1AlignedMethodSpecTest::RunTest(const FString& Parameters)
{
    FMethodSpec Spec;
    TestTrue(TEXT("Static aligned method is recognized"),
        FMethodSpec::FromMethodId(TEXT("lawnmower_v1_aligned_static"), Spec));
    TestTrue(TEXT("Static aligned method selects spawn-aware uniform partition"),
        Spec.Regime == ERegionRegime::StaticUniform && Spec.PartitionStrategy == FName(TEXT("SpawnAwareUniform")));
    TestEqual(TEXT("Static aligned method selects only the new planner"),
        Spec.PlannerStrategy, FName(TEXT("LawnmowerV1Aligned")));

    TestTrue(TEXT("Dynamic aligned method is recognized"),
        FMethodSpec::FromMethodId(TEXT("lawnmower_v1_aligned_dynamic"), Spec));
    TestTrue(TEXT("Dynamic aligned method keeps V1 zone-growing partition"),
        Spec.Regime == ERegionRegime::Dynamic && Spec.PartitionStrategy == FName(TEXT("ZoneRegionGrowing")));
    TestEqual(TEXT("Dynamic aligned method selects only the new planner"),
        Spec.PlannerStrategy, FName(TEXT("LawnmowerV1Aligned")));

    TestTrue(TEXT("Existing V1 static configuration remains unchanged"),
        FMethodSpec::FromMethodId(TEXT("lawnmower_v1_static"), Spec) &&
        Spec.PartitionStrategy == FName(TEXT("Uniform")) && Spec.PlannerStrategy == FName(TEXT("LawnmowerV1")));
    TestTrue(TEXT("Existing V2 dynamic configuration remains unchanged"),
        FMethodSpec::FromMethodId(TEXT("lawnmower_v2_dynamic"), Spec) &&
        Spec.PartitionStrategy == FName(TEXT("ZoneRegionGrowing")) && Spec.PlannerStrategy == FName(TEXT("LawnmowerV2")));
    TestTrue(TEXT("Factory exposes an independent aligned planner class"),
        FDronePlannerFactory::CreatePlannerClass(FName(TEXT("LawnmowerV1Aligned"))) == ULawnmowerV1AlignedPlanner::StaticClass());
    return true;
}

#endif
