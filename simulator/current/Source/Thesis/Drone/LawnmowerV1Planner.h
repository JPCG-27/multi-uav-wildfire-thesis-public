#pragma once

#include "CoreMinimal.h"
#include "DronePlannerBase.h"
#include "LawnmowerV1Planner.generated.h"

/**
 * Deterministic coverage baseline. Visits owned cell centres in a row-wise
 * serpentine order, with adjacent rows one cell apart. Initial coverage starts
 * at the first owned cell in that order. Cadence-triggered replans preserve the
 * current waypoint; progression occurs only on arrival.
 */
UCLASS(Blueprintable)
class THESIS_API ULawnmowerV1Planner : public UDronePlannerBase
{
    GENERATED_BODY()

public:
    virtual void Replan(ADrone& Drone, int32 CurrentTick) override;

    /** Shared with automation tests so coverage order is checked through the
     *  exact deterministic helper used by the runtime planner. */
    static TArray<int32> BuildSweepOrder(const TArray<int32>& OwnedCells, int32 GridWidth);
    static int32 FindIndexAtOrAfterRank(const TArray<int32>& SweepOrder, int32 GridWidth,
                                        int32 PreviousNextCell);
};
