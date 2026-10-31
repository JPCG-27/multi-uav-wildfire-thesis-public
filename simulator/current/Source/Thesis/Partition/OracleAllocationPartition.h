#pragma once

#include "CoreMinimal.h"
#include "PartitionStrategy.h"

class THESIS_API FOracleAllocationPartition final : public IPartitionStrategy
{
public:
    virtual FName GetStrategyName() const override { return FName(TEXT("OracleAllocation")); }

    virtual TArray<uint16> ComputePartition(
        const TArray<TObjectPtr<ADrone>>& ActiveDrones,
        const TArray<FVector2f>& DronePositionsCellSpace,
        int32 Tick,
        const FPartitionParams& Params) override;
};
