#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../Drone/LawnmowerV1Planner.h"
#include "../Benchmark/MethodSpec.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLawnmowerV1SweepOrderTest,
    "Thesis.Planner.LawnmowerV1.SerpentineCoverage",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLawnmowerV1SweepOrderTest::RunTest(const FString& Parameters)
{
    // Three sparse lanes on a four-column grid verify global row parity,
    // one-cell lane spacing, de-duplication, and complete owned-cell coverage.
    const TArray<int32> Owned = { 10, 0, 6, 4, 9, 1, 8, 2, 5, 0 };
    const TArray<int32> Expected = { 0, 1, 2, 6, 5, 4, 8, 9, 10 };
    const TArray<int32> First = ULawnmowerV1Planner::BuildSweepOrder(Owned, 4);
    const TArray<int32> Replay = ULawnmowerV1Planner::BuildSweepOrder(Owned, 4);

    TestTrue(TEXT("Order follows ascending rows and alternates X direction"), First == Expected);
    TestTrue(TEXT("Repeated construction is byte-for-byte stable"), First == Replay);
    TestEqual(TEXT("Every distinct owned cell appears once"), First.Num(), 9);
    TSet<int32> Covered;
    for (const int32 Cell : First)
    {
        Covered.Add(Cell);
    }
    for (const int32 Cell : Owned)
    {
        TestTrue(TEXT("The route covers every owned cell"), Covered.Contains(Cell));
    }
    TestTrue(TEXT("Invalid grid width yields no route"),
        ULawnmowerV1Planner::BuildSweepOrder(Owned, 0).IsEmpty());

    const TArray<int32> Rectangular = ULawnmowerV1Planner::BuildSweepOrder(
        TArray<int32>{ 0, 1, 2, 3, 4, 5, 6, 7, 8 }, 3);
    const TArray<int32> ExpectedRectangular = { 0, 1, 2, 5, 4, 3, 6, 7, 8 };
    TestTrue(TEXT("Three full lanes stay one cell apart and reverse direction"),
        Rectangular == ExpectedRectangular);

    const TArray<int32> UpdatedOwnership = ULawnmowerV1Planner::BuildSweepOrder(
        TArray<int32>{ 0, 1, 2, 3, 5, 6, 7, 8 }, 3);
    TestEqual(TEXT("Ownership refresh resumes at the next surviving sweep rank"),
        UpdatedOwnership[ULawnmowerV1Planner::FindIndexAtOrAfterRank(UpdatedOwnership, 3, 4)], 3);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FLawnmowerV1MethodSpecTest,
    "Thesis.Planner.LawnmowerV1.MethodSpecs",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLawnmowerV1MethodSpecTest::RunTest(const FString& Parameters)
{
    FMethodSpec Spec;
    TestTrue(TEXT("Static method id is recognized"),
        FMethodSpec::FromMethodId(TEXT("lawnmower_v1_static"), Spec));
    TestTrue(TEXT("Static arm uses static uniform ownership"),
        Spec.Regime == ERegionRegime::StaticUniform &&
        Spec.PartitionStrategy == FName(TEXT("Uniform")));
    TestEqual(TEXT("Static arm selects the lawnmower strategy"),
        Spec.PlannerStrategy, FName(TEXT("LawnmowerV1")));

    TestTrue(TEXT("Dynamic method id is recognized"),
        FMethodSpec::FromMethodId(TEXT("lawnmower_v1_dynamic"), Spec));
    TestTrue(TEXT("Dynamic arm uses dynamic region growing"),
        Spec.Regime == ERegionRegime::Dynamic &&
        Spec.PartitionStrategy == FName(TEXT("ZoneRegionGrowing")));
    TestEqual(TEXT("Dynamic arm selects the lawnmower strategy"),
        Spec.PlannerStrategy, FName(TEXT("LawnmowerV1")));
    TestTrue(TEXT("Both arms use sensor belief, not the oracle information source"),
        Spec.Info == EInformationSource::BeliefFromSensors);
    return true;
}

#endif
