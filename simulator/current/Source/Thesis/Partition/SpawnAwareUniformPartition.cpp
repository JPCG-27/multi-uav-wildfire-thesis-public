#include "SpawnAwareUniformPartition.h"

namespace
{
    void ChooseBlockLayout(int32 W, int32 H, int32 N, int32& OutRows, int32& OutCols)
    {
        OutRows = 1;
        OutCols = N;
        float BestScore = FLT_MAX;
        const float GridAspect = static_cast<float>(W) / static_cast<float>(FMath::Max(1, H));
        for (int32 Rows = 1; Rows <= N; ++Rows)
        {
            if (N % Rows != 0) continue;
            const int32 Cols = N / Rows;
            const float BlockAspect = (static_cast<float>(W) / Cols) / (static_cast<float>(H) / Rows);
            const float Score = FMath::Abs(FMath::Loge(FMath::Max(0.0001f, BlockAspect / GridAspect)));
            if (Score < BestScore ||
                (FMath::IsNearlyEqual(Score, BestScore) && FMath::Abs(Rows - Cols) < FMath::Abs(OutRows - OutCols)) ||
                (FMath::IsNearlyEqual(Score, BestScore) && FMath::Abs(Rows - Cols) == FMath::Abs(OutRows - OutCols) && Rows < OutRows))
            {
                BestScore = Score;
                OutRows = Rows;
                OutCols = Cols;
            }
        }
    }
}

TArray<uint16> FSpawnAwareUniformPartition::BuildPartition(
    int32 GridWidth, int32 GridHeight, const TArray<FVector2f>& DronePositionsCellSpace)
{
    TArray<uint16> Owners;
    if (GridWidth <= 0 || GridHeight <= 0 || GridWidth > MAX_int32 / GridHeight)
    {
        return Owners;
    }
    Owners.Init(0xFFFFu, GridWidth * GridHeight);
    const int32 N = DronePositionsCellSpace.Num();
    if (N <= 0 || N > 0xFFFF) return Owners;

    int32 Rows = 1;
    int32 Cols = N;
    ChooseBlockLayout(GridWidth, GridHeight, N, Rows, Cols);

    // Claim each drone's home block in index order. If several drones start in
    // one block, assign the displaced drone to the nearest remaining block.
    TArray<int32> BlockOwner;
    BlockOwner.Init(INDEX_NONE, N);
    TArray<int32> DroneBlock;
    DroneBlock.Init(INDEX_NONE, N);
    for (int32 Drone = 0; Drone < N; ++Drone)
    {
        const FVector2f P = DronePositionsCellSpace[Drone];
        const int32 X = FMath::Clamp(FMath::FloorToInt(P.X), 0, GridWidth - 1);
        const int32 Y = FMath::Clamp(FMath::FloorToInt(P.Y), 0, GridHeight - 1);
        const int32 Home = (FMath::Min(Rows - 1, (Y * Rows) / GridHeight) * Cols)
                         + FMath::Min(Cols - 1, (X * Cols) / GridWidth);
        if (BlockOwner[Home] == INDEX_NONE)
        {
            BlockOwner[Home] = Drone;
            DroneBlock[Drone] = Home;
        }
    }
    for (int32 Drone = 0; Drone < N; ++Drone)
    {
        if (DroneBlock[Drone] != INDEX_NONE) continue;
        const FVector2f P = DronePositionsCellSpace[Drone];
        int32 BestBlock = INDEX_NONE;
        float BestDistSq = FLT_MAX;
        for (int32 Block = 0; Block < N; ++Block)
        {
            if (BlockOwner[Block] != INDEX_NONE) continue;
            const int32 Row = Block / Cols;
            const int32 Col = Block % Cols;
            const FVector2f Center((static_cast<float>(Col) + 0.5f) * GridWidth / Cols,
                                   (static_cast<float>(Row) + 0.5f) * GridHeight / Rows);
            const float DistSq = FVector2f::DistSquared(P, Center);
            if (DistSq < BestDistSq || (DistSq == BestDistSq && Block < BestBlock))
            {
                BestDistSq = DistSq;
                BestBlock = Block;
            }
        }
        BlockOwner[BestBlock] = Drone;
        DroneBlock[Drone] = BestBlock;
    }

    for (int32 Y = 0; Y < GridHeight; ++Y)
    {
        const int32 Row = FMath::Min(Rows - 1, (Y * Rows) / GridHeight);
        for (int32 X = 0; X < GridWidth; ++X)
        {
            const int32 Col = FMath::Min(Cols - 1, (X * Cols) / GridWidth);
            const int32 GeometricBlock = Row * Cols + Col;
            Owners[Y * GridWidth + X] = static_cast<uint16>(BlockOwner[GeometricBlock]);
        }
    }
    return Owners;
}

TArray<uint16> FSpawnAwareUniformPartition::ComputePartition(
    const TArray<TObjectPtr<ADrone>>& ActiveDrones,
    const TArray<FVector2f>& DronePositionsCellSpace,
    int32 Tick,
    const FPartitionParams& Params)
{
    const int32 N = FMath::Min(ActiveDrones.Num(), DronePositionsCellSpace.Num());
    TArray<FVector2f> Positions;
    Positions.Append(DronePositionsCellSpace.GetData(), N);
    TArray<uint16> Owners = BuildPartition(Params.GridWidth, Params.GridHeight, Positions);
    UE_LOG(LogTemp, Log, TEXT("Thesis Partition tick %d: spawn-aware uniform blocks for %d drones."), Tick, N);
    return Owners;
}
