#include "PartitionStrategy.h"
#include "ZoneRegionGrowingPartition.h"
#include "UniformPartition.h"
#include "RandomPartition.h"
#include "OracleAllocationPartition.h"

// Central strategy registry. To add a strategy:
//   1. Implement IPartitionStrategy in Partition/.
//   2. Add a row to GetRegistry() below.
//   3. Select it via ASimulationManager::PartitionStrategyName (or run config).

namespace
{
    using FStrategyCreator = TFunction<TUniquePtr<IPartitionStrategy>()>;

    struct FRegistryRow
    {
        FName Name;
        FStrategyCreator Create;
    };

    static const TArray<FRegistryRow>& GetRegistry()
    {
        static const TArray<FRegistryRow> Registry = {
            { FName(TEXT("ZoneRegionGrowing")), [] { return TUniquePtr<IPartitionStrategy>(MakeUnique<FZoneRegionGrowingPartition>()); } },
            { FName(TEXT("Uniform")), [] { return TUniquePtr<IPartitionStrategy>(MakeUnique<FUniformPartition>()); } },
            { FName(TEXT("Random")), [] { return TUniquePtr<IPartitionStrategy>(MakeUnique<FRandomPartition>()); } },
            { FName(TEXT("OracleAllocation")), [] { return TUniquePtr<IPartitionStrategy>(MakeUnique<FOracleAllocationPartition>()); } },
        };
        return Registry;
    }
}

TUniquePtr<IPartitionStrategy> FPartitionStrategyFactory::Create(FName StrategyName)
{
    for (const FRegistryRow& Row : GetRegistry())
    {
        if (Row.Name == StrategyName)
        {
            return Row.Create();
        }
    }
    return nullptr;
}

FName FPartitionStrategyFactory::DefaultStrategyName()
{
    return FName(TEXT("ZoneRegionGrowing"));
}

TArray<FName> FPartitionStrategyFactory::RegisteredNames()
{
    TArray<FName> Names;
    for (const FRegistryRow& Row : GetRegistry())
    {
        Names.Add(Row.Name);
    }
    return Names;
}
