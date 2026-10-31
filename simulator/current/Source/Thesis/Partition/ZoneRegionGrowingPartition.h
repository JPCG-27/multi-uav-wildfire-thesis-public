#pragma once

#include "CoreMinimal.h"
#include "PartitionStrategy.h"

/**
 * Default partition strategy: adaptive zoning + home-zone seeding + workload-
 * balanced region growing on the zone-adjacency graph.
 *
 * Moved verbatim from ASimulationManager::ComputeBalancedPartition — algorithm
 * unchanged. The allocation rationale includes
 * determinism rules, and the list of rejected alternatives.
 */
class THESIS_API FZoneRegionGrowingPartition final : public IPartitionStrategy
{
public:
    virtual FName GetStrategyName() const override { return FName(TEXT("ZoneRegionGrowing")); }

    virtual TArray<uint16> ComputePartition(
        const TArray<TObjectPtr<ADrone>>& ActiveDrones,
        const TArray<FVector2f>& DronePositionsCellSpace,
        int32 Tick,
        const FPartitionParams& Params) override;
};
