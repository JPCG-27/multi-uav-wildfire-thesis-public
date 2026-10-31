#include "OracleAllocationPartition.h"
#include "GroundTruthOracle.h"

TArray<uint16> FOracleAllocationPartition::ComputePartition(
    const TArray<TObjectPtr<ADrone>>& ActiveDrones,
    const TArray<FVector2f>& DronePositionsCellSpace,
    int32 Tick,
    const FPartitionParams& Params)
{
    const int32 W = Params.GridWidth;
    const int32 H = Params.GridHeight;
    const int32 N = FMath::Min(ActiveDrones.Num(), DronePositionsCellSpace.Num());
    const int32 NumCells = W * H;

    TArray<uint16> Owners;
    Owners.Init(0xFFFFu, NumCells);
    if (W <= 0 || H <= 0 || N <= 0)
    {
        return Owners;
    }

    if (!Params.Oracle || !Params.Oracle->IsOracleInformationEnabled())
    {
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis Partition tick %d: OracleAllocation requested without oracle info; leaving cells unowned."),
               Tick);
        return Owners;
    }

    TArray<float> CellLoad;
    CellLoad.SetNumUninitialized(NumCells);
    float MaxLoad = 0.0f;
    for (int32 Cell = 0; Cell < NumCells; ++Cell)
    {
        const float Load = Params.Oracle->ComputeOracleCellLoad(Cell, Tick, Params.OracleLookaheadTicks);
        CellLoad[Cell] = Load;
        MaxLoad = FMath::Max(MaxLoad, Load);
    }

    TArray<double> DroneLoad;
    DroneLoad.Init(0.0, N);
    for (int32 Cell = 0; Cell < NumCells; ++Cell)
    {
        const int32 X = Cell % W;
        const int32 Y = Cell / W;
        const FVector2f CellPos(X + 0.5f, Y + 0.5f);

        int32 BestDrone = 0;
        double BestScore = TNumericLimits<double>::Max();
        for (int32 D = 0; D < N; ++D)
        {
            const double DistSq = static_cast<double>((CellPos - DronePositionsCellSpace[D]).SizeSquared());
            const double LoadAttraction = static_cast<double>(CellLoad[Cell] / FMath::Max(1.0f, MaxLoad));
            const double BalancePenalty = DroneLoad[D] * 0.015;
            const double Score = DistSq / (1.0 + 5.0 * LoadAttraction) + BalancePenalty;
            if (Score < BestScore || (Score == BestScore && D < BestDrone))
            {
                BestScore = Score;
                BestDrone = D;
            }
        }

        Owners[Cell] = static_cast<uint16>(BestDrone);
        DroneLoad[BestDrone] += FMath::Max(1.0f, CellLoad[Cell]);
    }

    FString Loads;
    for (int32 D = 0; D < N; ++D)
    {
        Loads += FString::Printf(TEXT("%.1f "), DroneLoad[D]);
    }
    UE_LOG(LogTemp, Log,
           TEXT("Thesis Partition tick %d: oracle allocation loads=[%s] lookahead=%d."),
           Tick, *Loads, Params.OracleLookaheadTicks);
    return Owners;
}
