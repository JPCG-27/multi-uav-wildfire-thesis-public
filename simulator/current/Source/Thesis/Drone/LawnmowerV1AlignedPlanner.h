#pragma once

#include "CoreMinimal.h"
#include "DronePlannerBase.h"
#include "LawnmowerV1AlignedPlanner.generated.h"

/** V1's one-cell sweep oriented from each drone's spawn corner. */
UCLASS(Blueprintable)
class THESIS_API ULawnmowerV1AlignedPlanner : public UDronePlannerBase
{
    GENERATED_BODY()
public:
    virtual void Replan(ADrone& Drone, int32 CurrentTick) override;

    static TArray<int32> BuildSweepOrder(const TArray<int32>& OwnedCells, int32 GridWidth,
                                         int32 GridHeight, bool bStartFromTop,
                                         bool bStartFromRight);
    static int32 FindIndexAtOrAfterRank(const TArray<int32>& SweepOrder, int32 GridWidth,
                                        int32 GridHeight, int32 PreviousNextCell,
                                        bool bStartFromTop, bool bStartFromRight);
};
