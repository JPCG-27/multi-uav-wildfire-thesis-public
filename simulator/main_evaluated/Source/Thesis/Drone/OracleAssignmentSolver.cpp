#include "OracleAssignmentSolver.h"

namespace
{
    struct FOracleAssignmentState
    {
        bool bValid = false;
        int64 Cost = 0;
        TArray<int32> TaskByDrone;
    };

    bool IsLexicographicallyBetter(const TArray<int32>& Candidate, const TArray<int32>& Existing)
    {
        for (int32 Drone = 0; Drone < Candidate.Num(); ++Drone)
        {
            const int32 CandidateTask = Candidate[Drone] == INDEX_NONE ? MAX_int32 : Candidate[Drone];
            const int32 ExistingTask = Existing[Drone] == INDEX_NONE ? MAX_int32 : Existing[Drone];
            if (CandidateTask != ExistingTask)
            {
                return CandidateTask < ExistingTask;
            }
        }
        return false;
    }

    bool ShouldReplace(const FOracleAssignmentState& Candidate, const FOracleAssignmentState& Existing)
    {
        return !Existing.bValid ||
               Candidate.Cost < Existing.Cost ||
               (Candidate.Cost == Existing.Cost &&
                IsLexicographicallyBetter(Candidate.TaskByDrone, Existing.TaskByDrone));
    }
}

FOracleAssignmentSolver::FResult FOracleAssignmentSolver::Solve(
    const TArray<TArray<int64>>& CostByDroneTask)
{
    FResult Result;
    const int32 NumDrones = CostByDroneTask.Num();
    Result.TaskByDrone.Init(INDEX_NONE, NumDrones);
    if (NumDrones == 0)
    {
        return Result;
    }

    checkf(NumDrones <= 20, TEXT("Oracle assignment bitmask supports at most 20 drones."));

    int32 NumTasks = 0;
    for (const TArray<int64>& Row : CostByDroneTask)
    {
        NumTasks = FMath::Max(NumTasks, Row.Num());
    }

    const int32 NumMasks = 1 << NumDrones;
    TArray<FOracleAssignmentState> States;
    States.SetNum(NumMasks);
    States[0].bValid = true;
    States[0].TaskByDrone.Init(INDEX_NONE, NumDrones);

    for (int32 Task = 0; Task < NumTasks; ++Task)
    {
        TArray<FOracleAssignmentState> Next = States; // This task may be skipped.
        for (int32 Mask = 0; Mask < NumMasks; ++Mask)
        {
            if (!States[Mask].bValid)
            {
                continue;
            }

            for (int32 Drone = 0; Drone < NumDrones; ++Drone)
            {
                if ((Mask & (1 << Drone)) != 0 || !CostByDroneTask[Drone].IsValidIndex(Task))
                {
                    continue;
                }

                const int64 EdgeCost = CostByDroneTask[Drone][Task];
                if (EdgeCost >= InfeasibleCost)
                {
                    continue;
                }

                const int32 NewMask = Mask | (1 << Drone);
                FOracleAssignmentState Candidate = States[Mask];
                Candidate.bValid = true;
                Candidate.Cost += EdgeCost;
                Candidate.TaskByDrone[Drone] = Task;
                if (ShouldReplace(Candidate, Next[NewMask]))
                {
                    Next[NewMask] = MoveTemp(Candidate);
                }
            }
        }
        States = MoveTemp(Next);
    }

    int32 BestMask = 0;
    for (int32 Mask = 1; Mask < NumMasks; ++Mask)
    {
        if (!States[Mask].bValid)
        {
            continue;
        }
        const int32 Count = FPlatformMath::CountBits(static_cast<uint32>(Mask));
        const int32 BestCount = FPlatformMath::CountBits(static_cast<uint32>(BestMask));
        if (Count > BestCount ||
            (Count == BestCount && ShouldReplace(States[Mask], States[BestMask])))
        {
            BestMask = Mask;
        }
    }

    Result.TaskByDrone = States[BestMask].TaskByDrone;
    Result.AssignedCount = FPlatformMath::CountBits(static_cast<uint32>(BestMask));
    Result.TotalCost = States[BestMask].Cost;
    return Result;
}
