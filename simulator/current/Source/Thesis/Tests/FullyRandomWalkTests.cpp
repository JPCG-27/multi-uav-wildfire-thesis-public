#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../Drone/FullyRandomWalk.h"
#include "../Benchmark/MethodSpec.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFullyRandomWalkTest,
    "Thesis.Planner.FullyRandomV1.WalkAndContracts",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFullyRandomWalkTest::RunTest(const FString& Parameters)
{
    auto AllOwned = [](int32) { return true; };
    TSet<int32> Choices;
    int32 First = 12, Replay = 12;
    for (int32 Tick = 0; Tick < 256; ++Tick)
    {
        const int32 Next = FullyRandomWalk::ChooseStep(5,5,12,27,0,Tick,AllOwned);
        Choices.Add(Next);
        TestTrue(TEXT("Every sample is a neighbouring cell"), Next != 12 &&
            FMath::Abs(Next % 5 - 2) <= 1 && FMath::Abs(Next / 5 - 2) <= 1);
        First = FullyRandomWalk::ChooseStep(5,5,First,27,0,Tick,AllOwned);
        Replay = FullyRandomWalk::ChooseStep(5,5,Replay,27,0,Tick,AllOwned);
        TestEqual(TEXT("Entire walk replays exactly"), First, Replay);
    }
    TestEqual(TEXT("All eight directions are sampled"), Choices.Num(), 8);
    auto OnlyRightOwned = [](int32 Cell) { return Cell == 12 || Cell == 13; };
    TestEqual(TEXT("Inside region only legal owned neighbours are selected"),
        FullyRandomWalk::ChooseStep(5,5,12,27,0,1,OnlyRightOwned),13);
    auto Isolated = [](int32 Cell) { return Cell == 12; };
    TestEqual(TEXT("Isolated region holds safely"),
        FullyRandomWalk::ChooseStep(5,5,12,27,0,1,Isolated),12);
    auto NoneOwned = [](int32) { return false; };
    TestTrue(TEXT("Outside territory walks without nearest-region guidance"),
        FullyRandomWalk::ChooseStep(5,5,12,27,0,1,NoneOwned) != 12);
    FMethodSpec Spec;
    TestTrue(TEXT("Static method is recognized"),FMethodSpec::FromMethodId(TEXT("fully_random_v1_static"),Spec));
    TestTrue(TEXT("Static random does not mean uniform"), Spec.Regime == ERegionRegime::StaticRandom);
    TestEqual(TEXT("Random territory strategy"),Spec.PartitionStrategy,FName(TEXT("FullyRandomV1")));
    TestTrue(TEXT("Dynamic method is recognized"),FMethodSpec::FromMethodId(TEXT("fully_random_v1_dynamic"),Spec));
    TestTrue(TEXT("Dynamic regenerates"),Spec.Regime == ERegionRegime::Dynamic);
    return true;
}
#endif
