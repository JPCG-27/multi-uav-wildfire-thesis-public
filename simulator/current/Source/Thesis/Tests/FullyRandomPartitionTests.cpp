#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../Partition/FullyRandomPartition.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FFullyRandomPartitionDeterminismTest,
    "Thesis.Partition.FullyRandomV1.Deterministic",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFullyRandomPartitionDeterminismTest::RunTest(const FString& Parameters)
{
    const TArray<uint16> First = FFullyRandomPartition::ComputeOwners(31, 19, 6, 12345, 50);
    const TArray<uint16> Second = FFullyRandomPartition::ComputeOwners(31, 19, 6, 12345, 50);
    TestTrue(TEXT("Same seed and tick produce the exact same owner array"), First == Second);

    FPartitionParams Params;
    Params.GridWidth = 31;
    Params.GridHeight = 19;
    Params.RunSeed = 12345;
    TArray<TObjectPtr<ADrone>> Drones;
    Drones.Add(nullptr);
    Drones.Add(nullptr);
    Drones.Add(nullptr);
    TArray<FVector2f> PositionsA = { FVector2f(0, 0), FVector2f(1, 1), FVector2f(2, 2) };
    TArray<FVector2f> PositionsB = { FVector2f(900, -20), FVector2f(-50, 400), FVector2f(5, 700) };
    FFullyRandomPartition Strategy;
    TestTrue(TEXT("Position changes do not affect the partition"),
        Strategy.ComputePartition(Drones, PositionsA, 50, Params) == Strategy.ComputePartition(Drones, PositionsB, 50, Params));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FFullyRandomPartitionCoverageAndContiguityTest,
    "Thesis.Partition.FullyRandomV1.CoverageAndContiguity",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFullyRandomPartitionCoverageAndContiguityTest::RunTest(const FString& Parameters)
{
    constexpr int32 Width = 29;
    constexpr int32 Height = 17;
    constexpr int32 DroneCount = 7;
    const TArray<uint16> Owners = FFullyRandomPartition::ComputeOwners(Width, Height, DroneCount, -77, 12);
    TestEqual(TEXT("Owner array covers every cell"), Owners.Num(), Width * Height);

    TArray<int32> CellCounts;
    CellCounts.Init(0, DroneCount);
    for (const uint16 Owner : Owners)
    {
        TestTrue(TEXT("Every cell has a valid owner"), Owner < DroneCount);
        if (Owner < DroneCount) ++CellCounts[Owner];
    }
    for (int32 Owner = 0; Owner < DroneCount; ++Owner)
    {
        TestTrue(TEXT("Every drone receives a non-empty territory"), CellCounts[Owner] > 0);

        int32 Start = INDEX_NONE;
        for (int32 Cell = 0; Cell < Owners.Num(); ++Cell)
        {
            if (Owners[Cell] == Owner) { Start = Cell; break; }
        }
        TArray<uint8> Visited;
        Visited.Init(0, Owners.Num());
        TArray<int32> Queue;
        Queue.Add(Start);
        Visited[Start] = 1;
        int32 Reached = 0;
        for (int32 Head = 0; Head < Queue.Num(); ++Head)
        {
            const int32 Cell = Queue[Head];
            ++Reached;
            const int32 X = Cell % Width;
            const int32 Y = Cell / Width;
            const int32 Neighbors[4] = { X > 0 ? Cell - 1 : INDEX_NONE,
                X + 1 < Width ? Cell + 1 : INDEX_NONE,
                Y > 0 ? Cell - Width : INDEX_NONE,
                Y + 1 < Height ? Cell + Width : INDEX_NONE };
            for (const int32 Neighbor : Neighbors)
            {
                if (Neighbor != INDEX_NONE && !Visited[Neighbor] && Owners[Neighbor] == Owner)
                {
                    Visited[Neighbor] = 1;
                    Queue.Add(Neighbor);
                }
            }
        }
        TestEqual(TEXT("Each owner's territory is 4-connected"), Reached, CellCounts[Owner]);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FFullyRandomPartitionVariationAndBoundsTest,
    "Thesis.Partition.FullyRandomV1.SeedVariationAndBounds",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFullyRandomPartitionVariationAndBoundsTest::RunTest(const FString& Parameters)
{
    const TArray<uint16> A = FFullyRandomPartition::ComputeOwners(20, 15, 5, 1, 8);
    const TArray<uint16> B = FFullyRandomPartition::ComputeOwners(20, 15, 5, 2, 8);
    TestTrue(TEXT("Changing the seed changes the sampled partition"), A != B);

    const TArray<uint16> MoreDronesThanCells = FFullyRandomPartition::ComputeOwners(2, 2, 9, 5, 0);
    TestEqual(TEXT("Owner count is capped by cell count"), MoreDronesThanCells.Num(), 4);
    TSet<uint16> UniqueOwners;
    for (const uint16 Owner : MoreDronesThanCells) UniqueOwners.Add(Owner);
    TestEqual(TEXT("Each cell gets its own owner when drones exceed cells"), UniqueOwners.Num(), 4);
    TestTrue(TEXT("Invalid dimensions return an empty result"), FFullyRandomPartition::ComputeOwners(0, 2, 1, 0, 0).IsEmpty());
    return true;
}

#endif
