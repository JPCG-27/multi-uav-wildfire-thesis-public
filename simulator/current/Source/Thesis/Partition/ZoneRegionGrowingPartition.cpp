#include "ZoneRegionGrowingPartition.h"
#include "Drone.h"

// Algorithm moved verbatim from ASimulationManager::ComputeBalancedPartition.
// Behaviour must be bit-identical to the pre-refactor implementation; the only
// changes are mechanical (member variables -> Params fields / locals).

TArray<uint16> FZoneRegionGrowingPartition::ComputePartition(
    const TArray<TObjectPtr<ADrone>>& ActiveDrones,
    const TArray<FVector2f>& DronePositionsCellSpace,
    int32 Tick,
    const FPartitionParams& Params)
{
    const int32 GridWidth = Params.GridWidth;
    const int32 GridHeight = Params.GridHeight;

    const int32 N = FMath::Min(ActiveDrones.Num(), DronePositionsCellSpace.Num());
    const int32 NumCells = GridWidth * GridHeight;
    TArray<uint16> Owners;
    Owners.Init(0xFFFFu, NumCells);
    if (N <= 0 || NumCells <= 0) return Owners;

    // ---------- A. Cell-level fields ----------
    // BeliefMass: max-over-drones detection mass (drives detection-CC formation and tiered weight).
    // (GlobalLoad/ZonePriority removed: dead since home-zone seeding replaced the
    //  priority-ordered Round 1 seeding; they were computed but never read.)
    TArray<float> BeliefMass;  BeliefMass.SetNumZeroed(NumCells);
    for (int32 d = 0; d < N; ++d)
    {
        ADrone* DronePtr = ActiveDrones[d].Get();
        if (!DronePtr) continue;
        for (int32 c = 0; c < NumCells; ++c)
        {
            const float M = DronePtr->ComputeCellDetectionMass(c);
            if (M > BeliefMass[c]) BeliefMass[c] = M;
        }
    }

    // Per-cell monitoring weight (5 confirmed / 3 expected / 1 normal).
    // ZoneWeight (Σ over its cells) is the workload measure Round 2 uses for "lightest drone".
    TArray<int32> CellWeight;
    CellWeight.SetNumUninitialized(NumCells);
    for (int32 c = 0; c < NumCells; ++c)
    {
        if (BeliefMass[c] > Params.ConfirmedDetectionThreshold)
            CellWeight[c] = Params.WeightConfirmed;
        else if (BeliefMass[c] > 0.0f)
            CellWeight[c] = Params.WeightExpected;
        else
            CellWeight[c] = Params.WeightNormal;
    }

    // ---------- B. Adaptive zoning ----------
    const int32 BgTileSide = FMath::Max(2,
        FMath::FloorToInt(FMath::Min(GridWidth, GridHeight) * Params.BackgroundTileFraction));
    const int32 MaxFireZoneSide = FMath::Max(BgTileSide,
        FMath::FloorToInt(FMath::Min(GridWidth, GridHeight) * Params.MaxFireZoneFraction));

    // B.1 Detection CC labelling (8-conn, raster sweep, union-find, min-label root).
    TArray<int32> Parent;
    Parent.Reserve(64);
    auto FindRoot = [&Parent](int32 X) -> int32
    {
        while (Parent[X] != X) { Parent[X] = Parent[Parent[X]]; X = Parent[X]; }
        return X;
    };
    auto UnionLabels = [&Parent, &FindRoot](int32 A, int32 B)
    {
        const int32 Ra = FindRoot(A);
        const int32 Rb = FindRoot(B);
        if (Ra == Rb) return;
        if (Ra < Rb) Parent[Rb] = Ra; else Parent[Ra] = Rb;
    };

    TArray<int32> CcLabel;
    CcLabel.Init(-1, NumCells);
    int32 NextLabel = 0;
    const int32 NbrDx[4] = { -1, -1,  0,  1 };
    const int32 NbrDy[4] = {  0, -1, -1, -1 };
    for (int32 cy = 0; cy < GridHeight; ++cy)
    {
        for (int32 cx = 0; cx < GridWidth; ++cx)
        {
            const int32 c = cy * GridWidth + cx;
            if (BeliefMass[c] <= 0.0f) continue;
            int32 Best = -1;
            for (int32 k = 0; k < 4; ++k)
            {
                const int32 nx = cx + NbrDx[k];
                const int32 ny = cy + NbrDy[k];
                if (nx < 0 || nx >= GridWidth || ny < 0 || ny >= GridHeight) continue;
                const int32 nl = CcLabel[ny * GridWidth + nx];
                if (nl >= 0 && (Best < 0 || nl < Best)) Best = nl;
            }
            if (Best < 0)
            {
                CcLabel[c] = NextLabel;
                Parent.Add(NextLabel);
                ++NextLabel;
            }
            else
            {
                CcLabel[c] = Best;
                for (int32 k = 0; k < 4; ++k)
                {
                    const int32 nx = cx + NbrDx[k];
                    const int32 ny = cy + NbrDy[k];
                    if (nx < 0 || nx >= GridWidth || ny < 0 || ny >= GridHeight) continue;
                    const int32 nl = CcLabel[ny * GridWidth + nx];
                    if (nl >= 0 && nl != Best) UnionLabels(Best, nl);
                }
            }
        }
    }
    TArray<int32> RootToCC;
    RootToCC.Init(-1, NextLabel);
    int32 NumCCs = 0;
    for (int32 c = 0; c < NumCells; ++c)
    {
        if (CcLabel[c] < 0) continue;
        const int32 Root = FindRoot(CcLabel[c]);
        if (RootToCC[Root] < 0) RootToCC[Root] = NumCCs++;
        CcLabel[c] = RootToCC[Root];
    }

    // B.2 CC bounding boxes — used to drive sub-tiling for large CCs.
    struct FCCBBox { int32 MinX, MinY, MaxX, MaxY; };
    TArray<FCCBBox> CCBBox;
    CCBBox.SetNum(NumCCs);
    for (int32 cc = 0; cc < NumCCs; ++cc)
        CCBBox[cc] = { GridWidth, GridHeight, -1, -1 };
    for (int32 cy = 0; cy < GridHeight; ++cy)
    {
        for (int32 cx = 0; cx < GridWidth; ++cx)
        {
            const int32 cc = CcLabel[cy * GridWidth + cx];
            if (cc < 0) continue;
            FCCBBox& B = CCBBox[cc];
            if (cx < B.MinX) B.MinX = cx;
            if (cy < B.MinY) B.MinY = cy;
            if (cx > B.MaxX) B.MaxX = cx;
            if (cy > B.MaxY) B.MaxY = cy;
        }
    }

    // B.3 Assign zone IDs.
    // Detection cells: zone = (CC id, sub-tile coord inside CC bbox). A CC that fits in
    //                  MaxFireZoneSide × MaxFireZoneSide ends up as one zone; bigger CCs
    //                  get sub-tiled (one zone per occupied sub-tile).
    // Background cells: zone = uniform tile in absolute grid coords (cx/BgTileSide, cy/BgTileSide).
    TArray<int32> ZoneId;
    ZoneId.Init(-1, NumCells);
    int32 NextZoneId = 0;

    TArray<TMap<int64, int32>> CCSubZoneMap;
    CCSubZoneMap.SetNum(NumCCs);
    for (int32 cy = 0; cy < GridHeight; ++cy)
    {
        for (int32 cx = 0; cx < GridWidth; ++cx)
        {
            const int32 c  = cy * GridWidth + cx;
            const int32 cc = CcLabel[c];
            if (cc < 0) continue;
            const FCCBBox& B = CCBBox[cc];
            const int32 sx = (cx - B.MinX) / MaxFireZoneSide;
            const int32 sy = (cy - B.MinY) / MaxFireZoneSide;
            const int64 Key = (static_cast<int64>(sy) << 32) | static_cast<uint32>(sx);
            const int32* Existing = CCSubZoneMap[cc].Find(Key);
            int32 zid;
            if (Existing) zid = *Existing;
            else { zid = NextZoneId++; CCSubZoneMap[cc].Add(Key, zid); }
            ZoneId[c] = zid;
        }
    }

    TMap<int64, int32> BgZoneMap;
    for (int32 cy = 0; cy < GridHeight; ++cy)
    {
        for (int32 cx = 0; cx < GridWidth; ++cx)
        {
            const int32 c = cy * GridWidth + cx;
            if (ZoneId[c] >= 0) continue;
            const int32 tx = cx / BgTileSide;
            const int32 ty = cy / BgTileSide;
            const int64 Key = (static_cast<int64>(ty) << 32) | static_cast<uint32>(tx);
            const int32* Existing = BgZoneMap.Find(Key);
            int32 zid;
            if (Existing) zid = *Existing;
            else { zid = NextZoneId++; BgZoneMap.Add(Key, zid); }
            ZoneId[c] = zid;
        }
    }

    const int32 NumZones = NextZoneId;
    if (NumZones <= 0) return Owners;

    // B.4 Per-zone stats.
    // ZoneWeight[z] = Σ CellWeight[c] over its cells — pre-computed once, reused throughout
    // Round 2 via incremental DroneWorkload updates. Never recomputed from cells.
    TArray<int32>     ZoneWeight;       ZoneWeight.Init(0, NumZones);
    TArray<FVector2f> ZoneCentroidSum;  ZoneCentroidSum.Init(FVector2f::ZeroVector, NumZones);
    TArray<int32>     ZoneCellCount;    ZoneCellCount.Init(0, NumZones);
    for (int32 cy = 0; cy < GridHeight; ++cy)
    {
        for (int32 cx = 0; cx < GridWidth; ++cx)
        {
            const int32 c = cy * GridWidth + cx;
            const int32 z = ZoneId[c];
            if (z < 0) continue;
            ZoneWeight[z]      += CellWeight[c];
            ZoneCentroidSum[z] += FVector2f(cx + 0.5f, cy + 0.5f);
            ZoneCellCount[z]   += 1;
        }
    }
    TArray<FVector2f> ZoneCentroid;  ZoneCentroid.SetNumUninitialized(NumZones);
    for (int32 z = 0; z < NumZones; ++z)
    {
        ZoneCentroid[z] = (ZoneCellCount[z] > 0)
            ? (ZoneCentroidSum[z] / static_cast<float>(ZoneCellCount[z]))
            : FVector2f::ZeroVector;
    }

    // B.5 Zone adjacency (4-conn over cells → zones).
    TArray<TSet<int32>> ZoneAdj;
    ZoneAdj.SetNum(NumZones);
    for (int32 cy = 0; cy < GridHeight; ++cy)
    {
        for (int32 cx = 0; cx < GridWidth; ++cx)
        {
            const int32 c = cy * GridWidth + cx;
            const int32 z = ZoneId[c];
            if (cx + 1 < GridWidth)
            {
                const int32 z2 = ZoneId[c + 1];
                if (z2 != z) { ZoneAdj[z].Add(z2); ZoneAdj[z2].Add(z); }
            }
            if (cy + 1 < GridHeight)
            {
                const int32 z2 = ZoneId[c + GridWidth];
                if (z2 != z) { ZoneAdj[z].Add(z2); ZoneAdj[z2].Add(z); }
            }
        }
    }

    // ---------- C. Round 1: home-zone seeding ----------
    // Each drone is seeded with the zone containing its current position — so it never has
    // to traverse the whole grid to reach its territory. If two drones share a zone, the
    // lowest drone index wins (deterministic); the displaced drone falls back to the closest
    // un-owned zone by Euclidean distance.
    TArray<int32> ZoneOwner;     ZoneOwner.Init(-1, NumZones);
    TArray<bool>  DroneAssigned; DroneAssigned.Init(false, N);

    // Pass 1: claim home zone (drone-index order — lowest index wins shared zones).
    for (int32 d = 0; d < N; ++d)
    {
        const int32 cx = FMath::Clamp(FMath::FloorToInt(DronePositionsCellSpace[d].X), 0, GridWidth  - 1);
        const int32 cy = FMath::Clamp(FMath::FloorToInt(DronePositionsCellSpace[d].Y), 0, GridHeight - 1);
        const int32 HomeZone = ZoneId[cy * GridWidth + cx];
        if (HomeZone >= 0 && ZoneOwner[HomeZone] < 0)
        {
            ZoneOwner[HomeZone] = d;
            DroneAssigned[d]    = true;
        }
    }

    // Pass 2: any drone displaced by a same-zone collision takes the closest un-owned zone.
    for (int32 d = 0; d < N; ++d)
    {
        if (DroneAssigned[d]) continue;
        int32 BestZ  = -1;
        float BestD2 = FLT_MAX;
        for (int32 z = 0; z < NumZones; ++z)
        {
            if (ZoneOwner[z] >= 0) continue;
            const float D2 = (DronePositionsCellSpace[d] - ZoneCentroid[z]).SizeSquared();
            if (D2 < BestD2 || (D2 == BestD2 && (BestZ < 0 || z < BestZ)))
            {
                BestD2 = D2;
                BestZ  = z;
            }
        }
        if (BestZ < 0) break;  // no un-owned zones left (more drones than zones — pathological)
        ZoneOwner[BestZ]    = d;
        DroneAssigned[d]    = true;
    }

    // ---------- D. Round 2: region growing by lightest drone ----------
    // DroneWorkload[d] = Σ ZoneWeight[z] over zones owned by d (incrementally maintained).
    // Frontier[d]      = set of unclaimed zones adjacent to d's current territory.
    TArray<int32>      DroneWorkload; DroneWorkload.Init(0, N);
    TArray<TSet<int32>> Frontier;     Frontier.SetNum(N);

    for (int32 z = 0; z < NumZones; ++z)
    {
        const int32 d = ZoneOwner[z];
        if (d < 0) continue;
        DroneWorkload[d] += ZoneWeight[z];
        for (int32 zn : ZoneAdj[z])
            if (ZoneOwner[zn] < 0) Frontier[d].Add(zn);
    }

    while (true)
    {
        // Lightest drone with a non-empty frontier; tie-break by drone index ascending.
        int32 BestDrone = -1;
        int32 BestLoad  = INT32_MAX;
        for (int32 d = 0; d < N; ++d)
        {
            if (Frontier[d].Num() == 0) continue;
            if (DroneWorkload[d] < BestLoad ||
                (DroneWorkload[d] == BestLoad && (BestDrone < 0 || d < BestDrone)))
            {
                BestLoad  = DroneWorkload[d];
                BestDrone = d;
            }
        }
        if (BestDrone < 0) break;

        // Closest frontier zone to the drone's seed position (= its current cell-space pos).
        // Picking the nearest zone makes the territory grow as a disc around the drone, instead
        // of snaking toward whatever direction has the highest-priority zones. Compact shapes
        // let the drone patrol without backtracking through cells it just visited.
        // Tie-break by zone id ascending.
        const FVector2f SeedPos = DronePositionsCellSpace[BestDrone];
        int32 BestZone  = -1;
        float BestDist2 = FLT_MAX;
        for (int32 z : Frontier[BestDrone])
        {
            const float D2 = (ZoneCentroid[z] - SeedPos).SizeSquared();
            if (D2 < BestDist2 || (D2 == BestDist2 && (BestZone < 0 || z < BestZone)))
            {
                BestDist2 = D2;
                BestZone  = z;
            }
        }
        if (BestZone < 0) break;

        ZoneOwner[BestZone] = BestDrone;
        DroneWorkload[BestDrone] += ZoneWeight[BestZone];
        for (int32 d = 0; d < N; ++d) Frontier[d].Remove(BestZone);
        for (int32 zn : ZoneAdj[BestZone])
            if (ZoneOwner[zn] < 0) Frontier[BestDrone].Add(zn);
    }

    // ---------- E. Compose cell-level ownership ----------
    for (int32 c = 0; c < NumCells; ++c)
    {
        const int32 z = ZoneId[c];
        if (z < 0) continue;
        const int32 d = ZoneOwner[z];
        if (d >= 0) Owners[c] = static_cast<uint16>(d);
    }

    if (N > 0)
    {
        int32 NumDetectionZones = 0;
        for (int32 cc = 0; cc < NumCCs; ++cc) NumDetectionZones += CCSubZoneMap[cc].Num();
        const int32 NumBackgroundZones = BgZoneMap.Num();

        TArray<int32> ZonesPerDrone; ZonesPerDrone.Init(0, N);
        for (int32 z = 0; z < NumZones; ++z)
        {
            const int32 d = ZoneOwner[z];
            if (d >= 0) ++ZonesPerDrone[d];
        }

        FString WorkloadStr, ZonesPerDroneStr;
        for (int32 d = 0; d < N; ++d)
        {
            WorkloadStr      += FString::Printf(TEXT("%d "), DroneWorkload[d]);
            ZonesPerDroneStr += FString::Printf(TEXT("%d "), ZonesPerDrone[d]);
        }
        UE_LOG(LogTemp, Log,
               TEXT("Thesis Partition tick %d: nZones=%d (detection=%d background=%d), workloads=[%s], zonesPerDrone=[%s]"),
               Tick, NumZones, NumDetectionZones, NumBackgroundZones, *WorkloadStr, *ZonesPerDroneStr);
    }

    return Owners;
}
