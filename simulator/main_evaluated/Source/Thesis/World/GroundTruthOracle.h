#pragma once

#include "CoreMinimal.h"

class AHumanManager;

class THESIS_API IGroundTruthOracle
{
public:
    virtual ~IGroundTruthOracle() = default;

    virtual bool IsOracleInformationEnabled() const = 0;
    virtual int32 GetOracleGridWidth() const = 0;
    virtual int32 GetOracleGridHeight() const = 0;
    virtual int32 GetFirstFireTickByCellIndex(int32 CellIndex) const = 0;
    virtual int32 GetRealPersonCountByCellIndex(int32 CellIndex) const = 0;
    virtual float ComputeOracleCellLoad(int32 CellIndex, int32 Tick, int32 LookaheadTicks) const = 0;
    virtual const AHumanManager* GetOracleHumanManager() const = 0;
};
