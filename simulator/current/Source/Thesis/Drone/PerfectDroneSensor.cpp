#include "PerfectDroneSensor.h"
#include "Grid.h"
#include "DroneTypes.h"
#include "HumanTypes.h"

void UPerfectDroneSensor::Sense(
    int32 CurrentTick,
    const Grid* GridData,
    const FVector& DroneWorldPos,
    float CellSizeUU,
    const FVector& GridOriginWorld,
    int32 FacingDir,
    float HorizontalFOVDegrees,
    float MaxViewDistanceUU,
    TArray<FObservation>& OutObservations)
{
    OutObservations.Reset();

    if (!GridData)
        return;

    // ------------------------------------------------------------
    // Direction convention:
    //
    // 0 = E
    // 1 = NE
    // 2 = N
    // 3 = NW
    // 4 = W
    // 5 = SW
    // 6 = S
    // 7 = SE
    // ------------------------------------------------------------

    static constexpr int32 DirX[8] =
    {
         1,  1,  0, -1,
        -1, -1,  0,  1
    };

    static constexpr int32 DirY[8] =
    {
         0, -1, -1, -1,
         0,  1,  1,  1
    };

    FacingDir = ((FacingDir % 8) + 8) % 8;

    const FVector2D Forward(
        static_cast<float>(DirX[FacingDir]),
        static_cast<float>(DirY[FacingDir]));

    const FVector2D Fwd =
        Forward.GetSafeNormal();

    // ------------------------------------------------------------
    // Drone position in grid coordinates
    // ------------------------------------------------------------

    const float DroneGX =
        (DroneWorldPos.X - GridOriginWorld.X) / CellSizeUU;

    const float DroneGY =
        (DroneWorldPos.Y - GridOriginWorld.Y) / CellSizeUU;

    // ------------------------------------------------------------
    // Camera frustum parameters
    // ------------------------------------------------------------

    const float MaxDistCells =
        MaxViewDistanceUU / CellSizeUU;

    const float CosHalfFOV =
        FMath::Cos(
            FMath::DegreesToRadians(
                HorizontalFOVDegrees * 0.5f));

    // Conservative bounds for search
    const int32 SearchRadius =
        FMath::CeilToInt(MaxDistCells) + 2;

    const int32 BaseX =
        FMath::FloorToInt(DroneGX);

    const int32 BaseY =
        FMath::FloorToInt(DroneGY);

    const int32 MinX =
        FMath::Clamp(BaseX - SearchRadius,
                     0,
                     GridData->Width - 1);

    const int32 MaxX =
        FMath::Clamp(BaseX + SearchRadius,
                     0,
                     GridData->Width - 1);

    const int32 MinY =
        FMath::Clamp(BaseY - SearchRadius,
                     0,
                     GridData->Height - 1);

    const int32 MaxY =
        FMath::Clamp(BaseY + SearchRadius,
                     0,
                     GridData->Height - 1);

    // ------------------------------------------------------------
    // Camera-style frustum test
    // ------------------------------------------------------------

    for (int32 y = MinY; y <= MaxY; ++y)
    {
        for (int32 x = MinX; x <= MaxX; ++x)
        {
            const FVector2D CellCenter(
                static_cast<float>(x) + 0.5f,
                static_cast<float>(y) + 0.5f);

            const FVector2D Delta =
                CellCenter - FVector2D(DroneGX, DroneGY);

            const float Dist =
                Delta.Length();

            if (Dist <= KINDA_SMALL_NUMBER)
                continue;

            if (Dist > MaxDistCells)
                continue;

            const FVector2D DirToCell =
                Delta / Dist;

            const float CosAngle =
                FVector2D::DotProduct(Fwd, DirToCell);

            // Outside camera FOV
            if (CosAngle < CosHalfFOV)
                continue;

            // ----------------------------------------------------
            // Observation generation
            // ----------------------------------------------------

            const int32 FirstFire =
                GridData->getFirstFireTick(x, y);

            const bool bFireHere =
                (FirstFire >= 0) &&
                (CurrentTick >= FirstFire);

            const int32 NumHumans =
                (GridData->getRealPriorityAndNumHumans(x, y) >> 16) & 0xFFFF;

            bool bPersonHere =
                NumHumans > 0;

            // INVARIANT (sensor realism): a POSITIVE person observation requires
            // a person physically inside the view cone, not merely inside an
            // observed cell. Without this gate, a cell whose centre is in view
            // "detects" people up to half a cell diagonal outside the camera
            // footprint (+/-25 m at 36 m cells, +/-102 m at 144 m cells), which
            // inflates detection at coarse cell sizes. Belief, staleness, and
            // negative observations remain cell-based on purpose: coarse
            // bookkeeping is a property of the belief grid under study, whereas
            // physical detection capability must not vary with cell size.
            // Deterministic: fixed snapshot iteration order, pure math; the
            // snapshot array is immutable during the drone parallel phase.
            if (bPersonHere && bWorldSpacePersonGating && PersonGroundTruth)
            {
                bPersonHere = false;
                for (const FHumanSnapshot& S : *PersonGroundTruth)
                {
                    if (S.PersonId == INDEX_NONE ||
                        S.State == EHumanState::Dead)
                        continue;   // matches the per-cell count semantics

                    // Same clamped-cell assignment HumanManager uses to write
                    // the per-cell counts, so count>0 <=> some snapshot maps here.
                    const int32 PX = FMath::Clamp(
                        static_cast<int32>(S.PosCell.X), 0, GridData->Width - 1);
                    const int32 PY = FMath::Clamp(
                        static_cast<int32>(S.PosCell.Y), 0, GridData->Height - 1);
                    if (PX != x || PY != y)
                        continue;

                    // Same cone test applied to cell centres, in grid units.
                    const FVector2D PDelta(
                        static_cast<float>(S.PosCell.X) - DroneGX,
                        static_cast<float>(S.PosCell.Y) - DroneGY);
                    const float PDist = PDelta.Length();
                    if (PDist <= KINDA_SMALL_NUMBER || PDist > MaxDistCells)
                        continue;
                    if (FVector2D::DotProduct(Fwd, PDelta / PDist) < CosHalfFOV)
                        continue;

                    bPersonHere = true;
                    break;
                }
            }

            FObservation Obs;
            Obs.bHasPersonEvidence = true;

            Obs.CellIndex =
                y * GridData->Width + x;

            Obs.Tick =
                CurrentTick;

            Obs.FireLogOddsUpdate =
                bFireHere ? FirePositiveLLR
                          : FireNegativeLLR;

            Obs.PersonLogOddsUpdate =
                bPersonHere ? PersonPositiveLLR
                            : PersonNegativeLLR;

            OutObservations.Add(Obs);
        }
    }
}