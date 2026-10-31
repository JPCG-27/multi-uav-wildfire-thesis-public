#pragma once

#include "CoreMinimal.h"

class ADrone;
class IGroundTruthOracle;

/** Tunables + grid dimensions handed to a partition strategy on every pass.
 *  Copied from ASimulationManager's UPROPERTYs so strategies never read the
 *  manager directly (keeps them headless-testable and swappable). */
struct THESIS_API FPartitionParams
{
    int32 GridWidth = 0;
    int32 GridHeight = 0;

    // Three-tier cell weight .
    float ConfirmedDetectionThreshold = 1750.0f;
    int32 WeightConfirmed = 5;
    int32 WeightExpected = 3;
    int32 WeightNormal = 1;

    // Adaptive zoning fractions.
    float BackgroundTileFraction = 1.0f / 15.0f;
    float MaxFireZoneFraction = 1.0f / 5.0f;

    int32 RunSeed = 0;
    int32 OracleLookaheadTicks = 500;
    const IGroundTruthOracle* Oracle = nullptr;
};

/**
 * Abstract server-side partition strategy.
 *
 * Contract (INVARIANT — every implementation must hold these):
 *  - Deterministic: same drones/positions/tick/params + same per-drone belief
 *    state must produce an identical owner array. All tie-breaks explicit.
 *  - Returns one owner index per cell, row-major (y * GridWidth + x),
 *    0xFFFF = unowned. Size must be GridWidth * GridHeight.
 *  - Called only from the single-threaded server slot. May read drone belief
 *    via ADrone::ComputeCellLoadPriority / ComputeCellDetectionMass; must not
 *    mutate any drone or grid state.
 *
 * Strategies are registered in FPartitionStrategyFactory (PartitionStrategyFactory.cpp)
 * and selected by name via ASimulationManager::PartitionStrategyName.
 */
class THESIS_API IPartitionStrategy
{
public:
    virtual ~IPartitionStrategy() = default;

    virtual FName GetStrategyName() const = 0;

    virtual TArray<uint16> ComputePartition(
        const TArray<TObjectPtr<ADrone>>& ActiveDrones,
        const TArray<FVector2f>& DronePositionsCellSpace,
        int32 Tick,
        const FPartitionParams& Params) = 0;
};

/** Name → strategy factory. Add new strategies in PartitionStrategyFactory.cpp. */
class THESIS_API FPartitionStrategyFactory
{
public:
    /** Returns nullptr for unknown names (caller logs and falls back to default). */
    static TUniquePtr<IPartitionStrategy> Create(FName StrategyName);

    static FName DefaultStrategyName();

    /** All registered names, for logging/config validation. */
    static TArray<FName> RegisteredNames();
};
