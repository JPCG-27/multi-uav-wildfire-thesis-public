#pragma once
#include "CoreMinimal.h"

namespace YoloGeometricMetrics
{
    // Person cells must be ordered by stable person ID. Negative detection cells
    // represent rejected classes, below-threshold boxes, or invalid projections.
    /** Returns the matched index in PersonCells for each detection, or INDEX_NONE.
     *  This is the exact legacy greedy assignment, with the winning person index
     *  exposed so replay diagnostics can attribute matches without changing them. */
    inline TArray<int32> MatchPersonIndices(const TArray<int32>& DetectionCells, const TArray<float>& Confidence,
                                            const TArray<int32>& PersonCells, int32 GridWidth)
    {
        check(GridWidth > 0 && DetectionCells.Num() == Confidence.Num());
        TArray<int32> Order;
        for (int32 K = 0; K < DetectionCells.Num(); ++K) Order.Add(K);
        Order.Sort([&](int32 A, int32 B) { return Confidence[A] == Confidence[B] ? A < B : Confidence[A] > Confidence[B]; });
        TArray<bool> Used;
        Used.Init(false, PersonCells.Num());
        TArray<int32> MatchedPerson;
        MatchedPerson.Init(INDEX_NONE, DetectionCells.Num());
        for (int32 K : Order)
        {
            const int32 Cell = DetectionCells[K];
            if (Cell < 0) continue;
            int32 Best = INDEX_NONE, BestDistance = MAX_int32;
            for (int32 P = 0; P < PersonCells.Num(); ++P)
            {
                if (Used[P]) continue;
                const int32 DX = FMath::Abs(Cell % GridWidth - PersonCells[P] % GridWidth);
                const int32 DY = FMath::Abs(Cell / GridWidth - PersonCells[P] / GridWidth);
                const int32 Distance = DX * DX + DY * DY;
                if (DX <= 1 && DY <= 1 && Distance < BestDistance) { Best = P; BestDistance = Distance; }
            }
            if (Best != INDEX_NONE) { Used[Best] = true; MatchedPerson[K] = Best; }
        }
        return MatchedPerson;
    }

    /** Legacy bool interface; retained as a wrapper over the same assignment. */
    inline TArray<bool> Match(const TArray<int32>& DetectionCells, const TArray<float>& Confidence,
                             const TArray<int32>& PersonCells, int32 GridWidth)
    {
        const TArray<int32> MatchedPerson = MatchPersonIndices(DetectionCells, Confidence, PersonCells, GridWidth);
        TArray<bool> Matched;
        Matched.SetNum(MatchedPerson.Num());
        for (int32 K = 0; K < MatchedPerson.Num(); ++K)
        {
            Matched[K] = MatchedPerson[K] != INDEX_NONE;
        }
        return Matched;
    }
}
