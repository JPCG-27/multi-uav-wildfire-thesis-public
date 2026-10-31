#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../Partition/SpawnAwareUniformPartition.h"
#include "../Benchmark/MethodSpec.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FSpawnAwareUniformPartitionTest,
    "Thesis.Partition.SpawnAwareUniform.Geometry",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FSpawnAwareUniformPartitionTest::RunTest(const FString& Parameters)
{
    const TArray<FVector2f> FourSpawns = {
        FVector2f(2.0f, 2.0f), FVector2f(8.0f, 2.0f),
        FVector2f(8.0f, 8.0f), FVector2f(2.0f, 8.0f)
    };
    const TArray<uint16> Owners = FSpawnAwareUniformPartition::BuildPartition(10, 10, FourSpawns);
    TestEqual(TEXT("Returns one owner per grid cell"), Owners.Num(), 100);
    for (int32 Drone = 0; Drone < FourSpawns.Num(); ++Drone)
    {
        const int32 X = FMath::FloorToInt(FourSpawns[Drone].X);
        const int32 Y = FMath::FloorToInt(FourSpawns[Drone].Y);
        TestEqual(TEXT("Each drone owns the block containing its spawn"), Owners[Y * 10 + X], static_cast<uint16>(Drone));
    }

    const TArray<FVector2f> RectangularSpawns = {
        FVector2f(1.0f, 1.0f), FVector2f(10.0f, 1.0f),
        FVector2f(10.0f, 4.0f), FVector2f(1.0f, 4.0f)
    };
    const TArray<uint16> Rectangular = FSpawnAwareUniformPartition::BuildPartition(12, 6, RectangularSpawns);
    TestEqual(TEXT("Non-square grids retain complete coverage"), Rectangular.Num(), 72);
    for (int32 Drone = 0; Drone < RectangularSpawns.Num(); ++Drone)
    {
        const int32 X = FMath::FloorToInt(RectangularSpawns[Drone].X);
        const int32 Y = FMath::FloorToInt(RectangularSpawns[Drone].Y);
        TestEqual(TEXT("Non-square grid assigns each home block by spawn"), Rectangular[Y * 12 + X], static_cast<uint16>(Drone));
    }

    const TArray<FVector2f> CollidingSpawns = {
        FVector2f(2.0f, 2.0f), FVector2f(2.0f, 2.0f),
        FVector2f(8.0f, 8.0f), FVector2f(8.0f, 8.0f)
    };
    const TArray<uint16> CollisionResult = FSpawnAwareUniformPartition::BuildPartition(10, 10, CollidingSpawns);
    TestTrue(TEXT("Collision fallback is repeatable"),
        CollisionResult == FSpawnAwareUniformPartition::BuildPartition(10, 10, CollidingSpawns));
    TSet<uint16> SeenOwners;
    for (const uint16 Owner : CollisionResult) SeenOwners.Add(Owner);
    TestEqual(TEXT("Collision fallback gives each drone one nonempty block"), SeenOwners.Num(), 4);

    TestTrue(TEXT("Invalid dimensions return an empty result"),
        FSpawnAwareUniformPartition::BuildPartition(0, 10, FourSpawns).IsEmpty());
    TestEqual(TEXT("No drones leave cells unowned"),
        FSpawnAwareUniformPartition::BuildPartition(10, 10, TArray<FVector2f>{})[0], static_cast<uint16>(0xFFFFu));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FSpawnAwareUniformMethodSpecTest,
    "Thesis.Partition.SpawnAwareUniform.MethodSpecs",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FSpawnAwareUniformMethodSpecTest::RunTest(const FString& Parameters)
{
    FMethodSpec Spec;
    TestTrue(TEXT("V2 static method is recognized"), FMethodSpec::FromMethodId(TEXT("lawnmower_v2_static"), Spec));
    TestTrue(TEXT("V2 static selects spawn-aware uniform ownership"),
        Spec.Regime == ERegionRegime::StaticUniform && Spec.PartitionStrategy == FName(TEXT("SpawnAwareUniform")));
    TestEqual(TEXT("V2 static selects LawnmowerV2"), Spec.PlannerStrategy, FName(TEXT("LawnmowerV2")));

    TestTrue(TEXT("V2 dynamic method is recognized"), FMethodSpec::FromMethodId(TEXT("lawnmower_v2_dynamic"), Spec));
    TestTrue(TEXT("V2 dynamic retains zone region growing"),
        Spec.Regime == ERegionRegime::Dynamic && Spec.PartitionStrategy == FName(TEXT("ZoneRegionGrowing")));
    TestEqual(TEXT("V2 dynamic selects LawnmowerV2"), Spec.PlannerStrategy, FName(TEXT("LawnmowerV2")));
    return true;
}

#endif
