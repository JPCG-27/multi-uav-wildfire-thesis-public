#include "LawnmowerV1Planner.h"

#include "Drone.h"

TArray<int32> ULawnmowerV1Planner::BuildSweepOrder(const TArray<int32>& OwnedCells, int32 GridWidth)
{
    TArray<int32> Order;
    if (GridWidth <= 0)
    {
        return Order;
    }

    Order = OwnedCells;
    Order.Sort();
    for (int32 Index = Order.Num() - 1; Index > 0; --Index)
    {
        if (Order[Index] == Order[Index - 1])
        {
            Order.RemoveAt(Index, 1, EAllowShrinking::No);
        }
    }
    Order.Sort([GridWidth](const int32 A, const int32 B)
    {
        const int32 AY = A / GridWidth;
        const int32 BY = B / GridWidth;
        if (AY != BY)
        {
            return AY < BY;
        }

        const int32 AX = A % GridWidth;
        const int32 BX = B % GridWidth;
        // Alternate direction each grid row to produce a serpentine scan.
        return (AY & 1) == 0 ? AX < BX : AX > BX;
    });
    return Order;
}

int32 ULawnmowerV1Planner::FindIndexAtOrAfterRank(const TArray<int32>& SweepOrder, int32 GridWidth,
                                                   int32 PreviousNextCell)
{
    if (GridWidth <= 0 || PreviousNextCell == INDEX_NONE)
    {
        return 0;
    }

    const int32 PreviousY = PreviousNextCell / GridWidth;
    const int32 PreviousX = PreviousNextCell % GridWidth;
    for (int32 Index = 0; Index < SweepOrder.Num(); ++Index)
    {
        const int32 Cell = SweepOrder[Index];
        const int32 Y = Cell / GridWidth;
        const int32 X = Cell % GridWidth;
        const bool bBeforePrevious = Y < PreviousY ||
            (Y == PreviousY && ((Y & 1) == 0 ? X < PreviousX : X > PreviousX));
        if (!bBeforePrevious)
        {
            return Index;
        }
    }
    return 0;
}

void ULawnmowerV1Planner::Replan(ADrone& Drone, int32 CurrentTick)
{
    Drone.RunLawnmowerV1Replan(CurrentTick);
}
