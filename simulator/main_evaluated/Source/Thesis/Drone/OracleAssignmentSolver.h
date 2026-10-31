#pragma once

#include "CoreMinimal.h"

/**
 * Deterministic minimum-cost bipartite assignment used by the benchmark oracle.
 *
 * Tasks are columns and drones are rows.  Each task can be assigned at most
 * once.  The solver first maximises the number of assigned drones, then
 * minimises total cost, then uses the lexicographically-smallest task vector
 * (drone-index order) as the explicit deterministic tie-break.
 */
class THESIS_API FOracleAssignmentSolver
{
public:
    static constexpr int64 InfeasibleCost = TNumericLimits<int64>::Max() / 4;

    struct FResult
    {
        TArray<int32> TaskByDrone;
        int32 AssignedCount = 0;
        int64 TotalCost = 0;
    };

    static FResult Solve(const TArray<TArray<int64>>& CostByDroneTask);
};
