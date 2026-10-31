#pragma once

#include "CoreMinimal.h"
#include "PartitionStrategy.h"

/**
 * Fully random, reproducible partitioning. Each owner starts at a distinct
 * uniformly selected cell and receives cells by random frontier growth, so
 * every territory remains 4-connected while all cells are assigned.
 */
class THESIS_API FFullyRandomPartition final : public IPartitionStrategy
{
public:
    virtual FName GetStrategyName() const override { return FName(TEXT("FullyRandomV1")); }

    virtual TArray<uint16> ComputePartition(
        const TArray<TObjectPtr<ADrone>>& ActiveDrones,
        const TArray<FVector2f>& DronePositionsCellSpace,
        int32 Tick,
        const FPartitionParams& Params) override;

    /** Pure deterministic helper; positions, beliefs, priorities, and world state are not inputs. */
    static TArray<uint16> ComputeOwners(int32 Width, int32 Height, int32 DroneCount, int32 RunSeed, int32 Tick);
};
