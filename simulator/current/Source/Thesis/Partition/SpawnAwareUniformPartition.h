#pragma once

#include "CoreMinimal.h"
#include "PartitionStrategy.h"

/** Static rectangular blocks seeded from each drone's current spawn block. */
class THESIS_API FSpawnAwareUniformPartition final : public IPartitionStrategy
{
public:
    virtual FName GetStrategyName() const override { return FName(TEXT("SpawnAwareUniform")); }

    virtual TArray<uint16> ComputePartition(
        const TArray<TObjectPtr<ADrone>>& ActiveDrones,
        const TArray<FVector2f>& DronePositionsCellSpace,
        int32 Tick,
        const FPartitionParams& Params) override;

    /** Pure helper so the assignment geometry is testable without actors. */
    static TArray<uint16> BuildPartition(int32 GridWidth, int32 GridHeight,
                                         const TArray<FVector2f>& DronePositionsCellSpace);
};
