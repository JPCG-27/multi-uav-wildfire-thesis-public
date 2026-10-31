#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "FireDataImporter.generated.h"

/**
 * Cell2Fire CSV → firstFireTick array. Stateless utility; pure C++ caller-friendly,
 * also exposed to BP for tooling.
 *
 * Output indexing: y * GridWidth + x .
 * Sentinel for "never burns": -1.
 */
UCLASS()
class THESIS_API UFireDataImporter : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

public:
    /**
     * Parses arrival-minutes from CsvPath into OutFirstFireTicks.
     *
     * @param CsvPath                  Absolute or project-relative file path.
     * @param GridWidth                Cells along X.
     * @param GridHeight               Cells along Y.
     * @param ArrivalMinutesPerTick    Conversion factor: minutes-per-sim-tick.
     * @param BurnDurationTicks        PLACEHOLDER: not consumed yet; reserved for a
     *                                 future burn-end array (Phase 2).
     * @param OutFirstFireTicks        Resized to GridWidth*GridHeight; pre-filled
     *                                 with -1; populated row-major from the CSV.
     * @return true on successful load + parse, false on file-load failure.
     */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Fire")
    static bool ImportFirstFireTicksFromCSV(
        const FString& CsvPath,
        int32 GridWidth,
        int32 GridHeight,
        float ArrivalMinutesPerTick,
        int32 BurnDurationTicks,
        TArray<int32>& OutFirstFireTicks);

    /**
     * Fallback ignition pattern for testing without a Cell2Fire CSV. Ignites
     * a disc at the grid centre with radial-distance arrival times (cells
     * further from the centre catch fire later).
     *
     * @param IgnitionRadiusCells   Radius of the initial fire disc, in cells.
     *                              Cells inside burn at tick 0.
     * @param SpreadTicksPerCell    Per-cell ignition delay outside the disc.
     *                              0 means only the disc burns (rest -1).
     * @param MaxSpreadCells        Outer cutoff radius. Cells beyond → -1.
     */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Fire")
    static void GenerateFallbackCentreFire(
        int32 GridWidth,
        int32 GridHeight,
        int32 IgnitionRadiusCells,
        int32 SpreadTicksPerCell,
        int32 MaxSpreadCells,
        TArray<int32>& OutFirstFireTicks);
};
