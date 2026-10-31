#include "LawnmowerV2Planner.h"
#include "Drone.h"
#include "Algo/Reverse.h"

int32 ULawnmowerV2Planner::ComputeLaneStride(float ViewRadiusCells, float HorizontalFOVDegrees,
                                              int32 GridWidth, int32 GridHeight)
{
    if (GridWidth <= 0 || GridHeight <= 0) return 1;
    const float HalfFov = FMath::Min(90.0f, FMath::Max(0.0f, HorizontalFOVDegrees * 0.5f));
    const float CoveredRadius = FMath::Max(0.0f, ViewRadiusCells) *
        FMath::Sin(FMath::DegreesToRadians(HalfFov));
    // A spacing of twice the cross-track footprint radius avoids blind strips.
    return FMath::Clamp(2 * FMath::FloorToInt(CoveredRadius), 1, FMath::Max(GridWidth, GridHeight));
}

TArray<int32> ULawnmowerV2Planner::BuildSweepOrder(const TArray<int32>& OwnedCells, int32 GridWidth,
                                                   int32 GridHeight, int32 LaneStride,
                                                   bool bStartFromTop, bool bStartFromRight)
{
    TArray<int32> Result;
    if (GridWidth <= 0 || GridHeight <= 0) return Result;
    TArray<uint8> Mask;
    Mask.Init(0, GridWidth * GridHeight);
    for (int32 Cell : OwnedCells) if (Mask.IsValidIndex(Cell)) Mask[Cell] = 1;

    LaneStride = FMath::Max(1, LaneStride);
    TArray<int32> Rows;
    for (int32 Y = 0; Y < GridHeight; Y += LaneStride) Rows.Add(Y);
    if (Rows.IsEmpty() || Rows.Last() != GridHeight - 1) Rows.Add(GridHeight - 1);
    if (bStartFromTop) Algo::Reverse(Rows);

    for (int32 Rank = 0; Rank < Rows.Num(); ++Rank)
    {
        const int32 Y = Rows[Rank];
        const bool bRightToLeft = bStartFromRight ^ ((Rank & 1) != 0);
        if (!bRightToLeft)
        {
            for (int32 X = 0; X < GridWidth; ++X) { const int32 C = Y * GridWidth + X; if (Mask[C]) Result.Add(C); }
        }
        else
        {
            for (int32 X = GridWidth - 1; X >= 0; --X) { const int32 C = Y * GridWidth + X; if (Mask[C]) Result.Add(C); }
        }
    }
    return Result;
}

bool ULawnmowerV2Planner::IsWaypointAvailable(int32 Cell, const TArray<uint8>* GlobalCompleted,
                                               const TArray<uint8>& LocalCompleted)
{
    if (!LocalCompleted.IsValidIndex(Cell) || LocalCompleted[Cell]) return false;
    return !GlobalCompleted || !GlobalCompleted->IsValidIndex(Cell) || !(*GlobalCompleted)[Cell];
}

void ULawnmowerV2Planner::Replan(ADrone& Drone, int32 CurrentTick)
{
    Drone.RunLawnmowerV2Replan(CurrentTick);
}
