#include "FireDataImporter.h"

#include "Misc/FileHelper.h"

bool UFireDataImporter::ImportFirstFireTicksFromCSV(
    const FString& CsvPath,
    int32 GridWidth,
    int32 GridHeight,
    float ArrivalMinutesPerTick,
    int32 BurnDurationTicks,
    TArray<int32>& OutFirstFireTicks)
{
    (void)BurnDurationTicks; // PLACEHOLDER: reserved for Phase 2 burn-end array.

    if (GridWidth <= 0 || GridHeight <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("FireDataImporter: invalid grid dims (%d x %d)"),
               GridWidth, GridHeight);
        return false;
    }

    if (ArrivalMinutesPerTick <= 0.0f)
    {
        UE_LOG(LogTemp, Error, TEXT("FireDataImporter: ArrivalMinutesPerTick must be > 0 (got %f)"),
               ArrivalMinutesPerTick);
        return false;
    }

    OutFirstFireTicks.Init(-1, GridWidth * GridHeight);

    FString FileContents;
    if (!FFileHelper::LoadFileToString(FileContents, *CsvPath))
    {
        UE_LOG(LogTemp, Error, TEXT("FireDataImporter: failed to load CSV: %s"), *CsvPath);
        return false;
    }

    TArray<FString> Lines;
    FileContents.ParseIntoArrayLines(Lines);

    for (int32 y = 0; y < Lines.Num() && y < GridHeight; ++y)
    {
        TArray<FString> Cells;
        Lines[y].ParseIntoArray(Cells, TEXT(","), false);

        for (int32 x = 0; x < Cells.Num() && x < GridWidth; ++x)
        {
            const FString Token = Cells[x].TrimStartAndEnd();
            if (Token.IsEmpty())
            {
                continue; // leave default -1
            }

            const float ArrivalMinutes = FCString::Atof(*Token);
            if (ArrivalMinutes <= 0.0f)
            {
                continue; // -1 / 0 / negative → never burns
            }

            const int32 Tick = FMath::RoundToInt(ArrivalMinutes / ArrivalMinutesPerTick);
            OutFirstFireTicks[y * GridWidth + x] = Tick;
        }
    }

    return true;
}

void UFireDataImporter::GenerateFallbackCentreFire(
    int32 GridWidth,
    int32 GridHeight,
    int32 IgnitionRadiusCells,
    int32 SpreadTicksPerCell,
    int32 MaxSpreadCells,
    TArray<int32>& OutFirstFireTicks)
{
    OutFirstFireTicks.Init(-1, FMath::Max(GridWidth * GridHeight, 0));
    if (GridWidth <= 0 || GridHeight <= 0)
    {
        return;
    }

    const float Cx = (GridWidth  - 1) * 0.5f;
    const float Cy = (GridHeight - 1) * 0.5f;

    const int32 RadIgnite = FMath::Max(IgnitionRadiusCells, 0);
    const int32 RadOuter  = FMath::Max(MaxSpreadCells, RadIgnite);

    for (int32 y = 0; y < GridHeight; ++y)
    {
        for (int32 x = 0; x < GridWidth; ++x)
        {
            const float Dx = static_cast<float>(x) - Cx;
            const float Dy = static_cast<float>(y) - Cy;
            const float Dist = FMath::Sqrt(Dx * Dx + Dy * Dy);

            if (Dist <= RadIgnite)
            {
                OutFirstFireTicks[y * GridWidth + x] = 0;
            }
            else if (SpreadTicksPerCell > 0 && Dist <= RadOuter)
            {
                const int32 Beyond = FMath::CeilToInt(Dist - RadIgnite);
                OutFirstFireTicks[y * GridWidth + x] = Beyond * SpreadTicksPerCell;
            }
            // else stays -1 (never burns).
        }
    }
}
