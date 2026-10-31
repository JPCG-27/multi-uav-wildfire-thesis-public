#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "OracleAssignmentSolver.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FOracleAssignmentGlobalOptimumTest,
    "Thesis.Oracle.Assignment.GlobalOptimum",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FOracleAssignmentGlobalOptimumTest::RunTest(const FString& Parameters)
{
    const TArray<TArray<int64>> Costs = {{1, 2}, {1, 100}};
    const FOracleAssignmentSolver::FResult Result = FOracleAssignmentSolver::Solve(Costs);
    TestEqual(TEXT("Both drones assigned"), Result.AssignedCount, 2);
    TestEqual(TEXT("Global optimum cost"), Result.TotalCost, int64(3));
    TestEqual(TEXT("Drone 0 takes task 1"), Result.TaskByDrone[0], 1);
    TestEqual(TEXT("Drone 1 takes task 0"), Result.TaskByDrone[1], 0);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FOracleAssignmentTieBreakTest,
    "Thesis.Oracle.Assignment.DeterministicTieBreak",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FOracleAssignmentTieBreakTest::RunTest(const FString& Parameters)
{
    const TArray<TArray<int64>> Costs = {{5, 5}, {5, 5}};
    const FOracleAssignmentSolver::FResult Result = FOracleAssignmentSolver::Solve(Costs);
    TestEqual(TEXT("Drone 0 gets lowest task"), Result.TaskByDrone[0], 0);
    TestEqual(TEXT("Drone 1 gets next task"), Result.TaskByDrone[1], 1);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FOracleAssignmentInfeasibleTest,
    "Thesis.Oracle.Assignment.InfeasibleEdges",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FOracleAssignmentInfeasibleTest::RunTest(const FString& Parameters)
{
    const int64 X = FOracleAssignmentSolver::InfeasibleCost;
    const TArray<TArray<int64>> Costs = {{X, 4}, {X, X}, {3, X}};
    const FOracleAssignmentSolver::FResult Result = FOracleAssignmentSolver::Solve(Costs);
    TestEqual(TEXT("Only feasible drones assigned"), Result.AssignedCount, 2);
    TestEqual(TEXT("Drone 0 assignment"), Result.TaskByDrone[0], 1);
    TestEqual(TEXT("Drone 1 remains unassigned"), Result.TaskByDrone[1], INDEX_NONE);
    TestEqual(TEXT("Drone 2 assignment"), Result.TaskByDrone[2], 0);
    return true;
}

#endif
