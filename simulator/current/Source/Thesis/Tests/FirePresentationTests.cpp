#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/FirePresentationMath.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFireConnectedCoverageTest, "Thesis.Fire.ConnectedCoverageBudget",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFireConnectedCoverageTest::RunTest(const FString& Parameters)
{
    TArray<int32> Cells, IgnitionTicks;
    IgnitionTicks.Init(1, 480);
    for (int32 Cell = 0; Cell < 400; ++Cell) Cells.Add(Cell);
    Cells.Add(478); // A separate small ignition must not be starved.
    IgnitionTicks[210] = 0;
    IgnitionTicks[478] = 0;
    const TArray<int32> Order = FirePresentation::ConnectedCellOrder(Cells, 20, 24, IgnitionTicks);
    TestEqual(TEXT("Every active cell appears once"), Order.Num(), 401);
    TestEqual(TEXT("Main region starts at its earliest ignition"), Order[0], 210);
    TestEqual(TEXT("Separate region receives a slot immediately"), Order[1], 478);
    TSet<int32> Selected;
    bool bConnectedPrefix = true;
    for (int32 I = 0; I < 128; ++I)
    {
        const int32 Cell = Order[I];
        if (I > 1)
            bConnectedPrefix &= (Cell % 20 > 0 && Selected.Contains(Cell - 1))
                || (Cell % 20 < 19 && Selected.Contains(Cell + 1))
                || Selected.Contains(Cell - 20) || Selected.Contains(Cell + 20);
        Selected.Add(Cell);
    }
    TestTrue(TEXT("Budget keeps connected patches rather than scattered cells"), bConnectedPrefix);
    TestEqual(TEXT("Budget has no duplicate slots"), Selected.Num(), 128);
    Cells.Sort([](int32 A, int32 B) { return A > B; });
    TestTrue(TEXT("Input order cannot change the selected front"),
        Order == FirePresentation::ConnectedCellOrder(Cells, 20, 24, IgnitionTicks));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFireNiagaraStepPlanTest,
    "Thesis.Fire.NiagaraPreviewStepPlan",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFireNiagaraStepPlanTest::RunTest(const FString& Parameters)
{
    using FirePresentation::PlanNiagaraSteps;
    constexpr int32 Period = 240;
    constexpr int32 Warmup = 30;

    const auto FirstUse = PlanNiagaraSteps(true, false, false, 10, -1, 30, -1, -1, Period, Warmup);
    TestTrue(TEXT("First use reconstructs"), FirstUse.bReconstruct);
    TestEqual(TEXT("First use warms up to tick age"), FirstUse.StepsToAdvance, 60);

    const auto SameTick = PlanNiagaraSteps(true, true, false, 10, 10, 30, 30, 60, Period, Warmup);
    TestFalse(TEXT("Same simulation age continues"), SameTick.bReconstruct);
    TestEqual(TEXT("Same age advances zero steps"), SameTick.StepsToAdvance, 0);

    const auto Forward = PlanNiagaraSteps(true, true, false, 11, 10, 33, 30, 60, Period, Warmup);
    TestFalse(TEXT("Forward tick continues"), Forward.bReconstruct);
    TestEqual(TEXT("Forward tick advances only elapsed steps"), Forward.StepsToAdvance, 3);

    const auto SkippedCaptureTicks = PlanNiagaraSteps(true, true, false, 14, 11, 39, 33, 63, Period, Warmup);
    TestFalse(TEXT("Skipped capture ticks continue within the same loop"), SkippedCaptureTicks.bReconstruct);
    TestEqual(TEXT("Skipped captures advance exactly the elapsed fixed steps"),
        SkippedCaptureTicks.StepsToAdvance, 6);

    const auto CanonicalAba = PlanNiagaraSteps(false, true, false, 14, 11, 39, 33, 63, Period, Warmup);
    TestTrue(TEXT("Canonical diagnostic mode reconstructs every capture"), CanonicalAba.bReconstruct);
    TestEqual(TEXT("Canonical diagnostic uses the bounded absolute target"), CanonicalAba.StepsToAdvance, 69);

    // A full-loop jump has the same modulo target as before, so comparing only
    // target steps would miss the required reset.
    const auto FullPeriodJump = PlanNiagaraSteps(true, true, false, 90, 10, 270, 30, 60, Period, Warmup);
    TestTrue(TEXT("Crossing a full Niagara loop reconstructs"), FullPeriodJump.bReconstruct);
    TestEqual(TEXT("Loop reconstruction uses canonical bounded target"), FullPeriodJump.StepsToAdvance, 60);

    const auto Wrap = PlanNiagaraSteps(true, true, false, 81, 80, 243, 240, 270, Period, Warmup);
    TestTrue(TEXT("Modulo age wrap reconstructs"), Wrap.bReconstruct);

    const auto CaptureLoopBoundary = PlanNiagaraSteps(true, true, false,
        81, 80, 241, 239, 269, Period, Warmup);
    TestTrue(TEXT("Capture crossing a loop boundary reconstructs"), CaptureLoopBoundary.bReconstruct);
    TestEqual(TEXT("Loop boundary capture returns to the canonical bounded target"),
        CaptureLoopBoundary.StepsToAdvance, 31);

    const auto Rewind = PlanNiagaraSteps(true, true, false, 3, 10, 9, 30, 60, Period, Warmup);
    TestTrue(TEXT("Tick rewind reconstructs"), Rewind.bReconstruct);

    const auto AssignmentChanged = PlanNiagaraSteps(true, true, true, 11, 10, 33, 30, 60, Period, Warmup);
    TestTrue(TEXT("Assignment or phase invalidation reconstructs"), AssignmentChanged.bReconstruct);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFirePlumePresentationTest, "Thesis.Fire.PlumePresentation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFirePlumePresentationTest::RunTest(const FString& Parameters)
{
    using FirePresentation::SmokeWindOffset;
    TestTrue(TEXT("Source stays grounded"), SmokeWindOffset(0, FVector2D(1, 0), 100, 50, 2500).IsZero());
    TestTrue(TEXT("No drift in calm air"), SmokeWindOffset(200, FVector2D(1, 0), 0, 50, 2500).IsZero());
    TestTrue(TEXT("Zero updraft remains finite"), SmokeWindOffset(200, FVector2D(1, 0), 100, 0, 2500).IsZero());
    const FVector Drift = SmokeWindOffset(200, FVector2D(0, -4), 100, 50, 2500);
    TestTrue(TEXT("Wind is normalized; upper layer moves downwind"), Drift.Equals(FVector(0, -400, 0), 0.0));
    TestTrue(TEXT("Weak updraft is bounded"), SmokeWindOffset(200, FVector2D(1, 0), 100, .01f, 2500).Equals(FVector(2500, 0, 0), 0.0));
    TestTrue(TEXT("Repeated evaluation exact"), Drift.Equals(SmokeWindOffset(200, FVector2D(0, -4), 100, 50, 2500), 0.0));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFireClusterPresentationTest, "Thesis.Fire.ClusterPresentation",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFireClusterPresentationTest::RunTest(const FString& Parameters)
{
    using FirePresentation::ClusterScaleVariation;
    const float First = ClusterScaleVariation(1234, 0);
    TestTrue(TEXT("Cluster scale variation is bounded"), First >= 0.86f && First <= 1.14f);
    TestEqual(TEXT("Cluster variation is stable for the same identity"),
        ClusterScaleVariation(1234, 0), First);
    TestTrue(TEXT("Different clusters receive visual variation"),
        !FMath::IsNearlyEqual(First, ClusterScaleVariation(1234, 1), 1.e-6f));
    const float Unrelated = ClusterScaleVariation(87, 5);
    TestTrue(TEXT("Unrelated cluster has its own variation"), Unrelated >= 0.86f && Unrelated <= 1.14f);
    TestEqual(TEXT("Prior evaluations do not change a cluster result"),
        ClusterScaleVariation(1234, 0), First);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFireSmokeLayerProfileTest, "Thesis.Fire.SmokeLayerProfile",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFireSmokeLayerProfileTest::RunTest(const FString& Parameters)
{
    using FirePresentation::SmokeLayerProfile;
    const auto Bottom = SmokeLayerProfile(0, 3);
    const auto Middle = SmokeLayerProfile(1, 3);
    const auto Top = SmokeLayerProfile(2, 3);
    TestEqual(TEXT("Bottom layer keeps base size"), Bottom.Scale, 1.0f);
    TestTrue(TEXT("Plume broadens with height"), Bottom.Scale < Middle.Scale && Middle.Scale < Top.Scale);
    TestTrue(TEXT("Smoke thins with height"), Bottom.Intensity > Middle.Intensity && Middle.Intensity > Top.Intensity);
    TestTrue(TEXT("Upper smoke breaks up more"), Bottom.NoiseStrength < Middle.NoiseStrength && Middle.NoiseStrength < Top.NoiseStrength);
    TestTrue(TEXT("Single layer is the compact dense source"),
        SmokeLayerProfile(0, 1).Scale == 1.0f && SmokeLayerProfile(0, 1).Intensity == 1.0f);
    TestEqual(TEXT("Layer profile is stable"), SmokeLayerProfile(2, 3).Intensity, Top.Intensity);
    return true;
}
#endif
