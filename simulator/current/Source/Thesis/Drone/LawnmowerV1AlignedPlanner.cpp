#include "LawnmowerV1AlignedPlanner.h"
#include "Drone.h"

TArray<int32> ULawnmowerV1AlignedPlanner::BuildSweepOrder(const TArray<int32>& OwnedCells,
                                                           int32 GridWidth, int32 GridHeight,
                                                           bool bStartFromTop, bool bStartFromRight)
{
    TArray<int32> Order;
    if (GridWidth <= 0 || GridHeight <= 0 || GridWidth > MAX_int32 / GridHeight) return Order;

    const int32 Total = GridWidth * GridHeight;
    Order.Reserve(OwnedCells.Num());
    for (const int32 Cell : OwnedCells)
    {
        if (Cell >= 0 && Cell < Total) Order.Add(Cell);
    }
    Order.Sort();
    for (int32 Index = Order.Num() - 1; Index > 0; --Index)
    {
        if (Order[Index] == Order[Index - 1]) Order.RemoveAt(Index, 1, EAllowShrinking::No);
    }
    Order.Sort([GridWidth, GridHeight, bStartFromTop, bStartFromRight](const int32 A, const int32 B)
    {
        const int32 AY = A / GridWidth;
        const int32 BY = B / GridWidth;
        const int32 ARankY = bStartFromTop ? GridHeight - 1 - AY : AY;
        const int32 BRankY = bStartFromTop ? GridHeight - 1 - BY : BY;
        if (ARankY != BRankY) return ARankY < BRankY;

        const int32 RowRank = bStartFromTop ? GridHeight - 1 - AY : AY;
        const bool bRightToLeft = bStartFromRight ^ ((RowRank & 1) != 0);
        const int32 AX = A % GridWidth;
        const int32 BX = B % GridWidth;
        return bRightToLeft ? AX > BX : AX < BX;
    });
    return Order;
}

int32 ULawnmowerV1AlignedPlanner::FindIndexAtOrAfterRank(const TArray<int32>& SweepOrder,
                                                          int32 GridWidth, int32 GridHeight,
                                                          int32 PreviousNextCell,
                                                          bool bStartFromTop, bool bStartFromRight)
{
    if (GridWidth <= 0 || GridHeight <= 0 || PreviousNextCell == INDEX_NONE) return 0;
    const int32 PreviousY = PreviousNextCell / GridWidth;
    const int32 PreviousX = PreviousNextCell % GridWidth;
    const int32 PreviousRowRank = bStartFromTop ? GridHeight - 1 - PreviousY : PreviousY;
    const bool bPreviousRightToLeft = bStartFromRight ^ ((PreviousRowRank & 1) != 0);
    const int32 PreviousXRank = bPreviousRightToLeft ? GridWidth - 1 - PreviousX : PreviousX;

    for (int32 Index = 0; Index < SweepOrder.Num(); ++Index)
    {
        const int32 Cell = SweepOrder[Index];
        const int32 Y = Cell / GridWidth;
        const int32 X = Cell % GridWidth;
        const int32 RowRank = bStartFromTop ? GridHeight - 1 - Y : Y;
        if (RowRank != PreviousRowRank) { if (RowRank > PreviousRowRank) return Index; continue; }
        const bool bRightToLeft = bStartFromRight ^ ((RowRank & 1) != 0);
        const int32 XRank = bRightToLeft ? GridWidth - 1 - X : X;
        if (XRank >= PreviousXRank) return Index;
    }
    return 0;
}

void ULawnmowerV1AlignedPlanner::Replan(ADrone& Drone, int32 CurrentTick)
{
    Drone.RunLawnmowerV1AlignedReplan(CurrentTick);
}
