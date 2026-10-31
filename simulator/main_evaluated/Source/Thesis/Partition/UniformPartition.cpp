#include "UniformPartition.h"

TArray<uint16> FUniformPartition::ComputePartition(
    const TArray<TObjectPtr<ADrone>>& ActiveDrones,
    const TArray<FVector2f>& DronePositionsCellSpace,
    int32 Tick,
    const FPartitionParams& Params)
{
    const int32 W = Params.GridWidth;
    const int32 H = Params.GridHeight;
    const int32 N = FMath::Min(ActiveDrones.Num(), DronePositionsCellSpace.Num());

    TArray<uint16> Owners;
    Owners.Init(0xFFFFu, W * H);
    if (W <= 0 || H <= 0 || N <= 0)
    {
        return Owners;
    }

    int32 BestRows = 1;
    int32 BestCols = N;
    float BestScore = FLT_MAX;
    const float GridAspect = static_cast<float>(W) / static_cast<float>(FMath::Max(1, H));

    for (int32 Rows = 1; Rows <= N; ++Rows)
    {
        if (N % Rows != 0)
        {
            continue;
        }
        const int32 Cols = N / Rows;
        const float BlockAspect = (static_cast<float>(W) / Cols) / (static_cast<float>(H) / Rows);
        const float Score = FMath::Abs(FMath::Loge(FMath::Max(0.0001f, BlockAspect / GridAspect)));
        if (Score < BestScore ||
            (FMath::IsNearlyEqual(Score, BestScore) && FMath::Abs(Rows - Cols) < FMath::Abs(BestRows - BestCols)) ||
            (FMath::IsNearlyEqual(Score, BestScore) && FMath::Abs(Rows - Cols) == FMath::Abs(BestRows - BestCols) && Rows < BestRows))
        {
            BestScore = Score;
            BestRows = Rows;
            BestCols = Cols;
        }
    }

    for (int32 Y = 0; Y < H; ++Y)
    {
        const int32 Row = FMath::Min(BestRows - 1, (Y * BestRows) / H);
        for (int32 X = 0; X < W; ++X)
        {
            const int32 Col = FMath::Min(BestCols - 1, (X * BestCols) / W);
            const int32 DroneIdx = Row * BestCols + Col;
            Owners[Y * W + X] = static_cast<uint16>(DroneIdx);
        }
    }

    UE_LOG(LogTemp, Log,
           TEXT("Thesis Partition tick %d: uniform static %dx%d blocks for %d drones."),
           Tick, BestRows, BestCols, N);
    return Owners;
}
