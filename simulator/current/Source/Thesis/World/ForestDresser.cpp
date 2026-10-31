#include "ForestDresser.h"
#include "Grid.h"
#include "ProceduralTerrain.h"
#include "Components/HierarchicalInstancedStaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Math/RandomStream.h"
#include "Misc/PackageName.h"
#include "HAL/IConsoleManager.h"
#include "StaticMeshResources.h"
#include "ForestDresserBatching.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
    struct FForestLayoutItem
    {
        int32 MeshIndex = INDEX_NONE;
        FTransform Transform = FTransform::Identity;
        int32 CellIdx = INDEX_NONE;
        int32 CellX = 0;
        int32 CellY = 0;
        uint8 ComponentRole = 0;
        bool bVegetation = false;
        bool bCastShadow = false;
    };
    struct FForestLayoutCacheEntry
    {
        FString Key;
        TArray<FString> MeshPaths;
        TArray<FForestLayoutItem> Items;
        uint64 Bytes() const
        {
            uint64 Total = static_cast<uint64>(Items.GetAllocatedSize());
            for (const FString& MeshPath : MeshPaths) Total += MeshPath.GetAllocatedSize();
            return Total;
        }
    };
    FForestLayoutCacheEntry GForestLayoutCache;

    uint64 ForestDoubleBits(double Value)
    {
        uint64 Bits = 0; FMemory::Memcpy(&Bits, &Value, sizeof(Bits)); return Bits;
    }

    struct FForestDensityCacheEntry
    {
        FString Key;
        float Coverage = 0.0f;
        float NoiseScale = 0.0f;
        TArray<float> Values;
    };
    TArray<FForestDensityCacheEntry> GForestDensityCache;
}

AForestDresser::AForestDresser()
{
    PrimaryActorTick.bCanEverTick = false;

    USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);
}

UHierarchicalInstancedStaticMeshComponent* AForestDresser::GetOrCreateComponentForMesh(
    UStaticMesh* Mesh, TArray<int32>& ExistingComponents,
    int32 ComponentOrdinal, bool bCastShadow)
{
    if (!Mesh)
    {
        return nullptr;
    }
    if (ExistingComponents.IsValidIndex(ComponentOrdinal))
    {
        const int32 Existing = ExistingComponents[ComponentOrdinal];
        return FloraComponents.IsValidIndex(Existing) ? FloraComponents[Existing] : nullptr;
    }
    check(ComponentOrdinal == ExistingComponents.Num());

    UHierarchicalInstancedStaticMeshComponent* C =
        NewObject<UHierarchicalInstancedStaticMeshComponent>(this);
    // BuildForest publishes the completed hierarchy synchronously after the
    // batch upload; avoid scheduling a redundant asynchronous instance build.
    C->bAutoRebuildTreeOnInstanceChanges = false;
    C->RegisterComponent();
    C->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
    C->SetMobility(EComponentMobility::Movable);
    C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    C->SetCanEverAffectNavigation(false);
    C->SetStaticMesh(Mesh);
    C->SetCastShadow(bCastShadow);
    if (bWriteScorchToPerInstanceCustomData)
    {
        C->NumCustomDataFloats = FMath::Max(C->NumCustomDataFloats, ScorchCustomDataIndex + 1);
    }
    C->SetVisibleInSceneCaptureOnly(bVisibleOnlyToDroneCaptures);
    if (bDisableFoliageWind)
    {
        C->SetEvaluateWorldPositionOffset(false);
        C->SetEvaluateWorldPositionOffsetInRayTracing(false);
        C->SetWorldPositionOffsetDisableDistance(1);
        C->UpdateInitialEvaluateWorldPositionOffset();
    }

    ExistingComponents.Add(FloraComponents.Num());
    FloraComponents.Add(C);
    return C;
}

void AForestDresser::SetVisualsCaptureOnly(bool bCaptureOnly)
{
    bVisibleOnlyToDroneCaptures = bCaptureOnly;
    for (const TObjectPtr<UHierarchicalInstancedStaticMeshComponent>& C : FloraComponents)
    {
        if (C)
        {
            C->SetVisibleInSceneCaptureOnly(bCaptureOnly);
        }
    }
}

int32 AForestDresser::GetVegetationFireAnchors(int32 CellX, int32 CellY, TArray<FVector>& OutAnchors, int32 MaxAnchors) const
{
    if (BuiltGridW <= 0 || MaxAnchors <= 0)
    {
        return 0;
    }

    const int32 CellIdx = CellY * BuiltGridW + CellX;
    const TArray<FCellInstanceRef>* Refs = CellToVegetationInstances.Find(CellIdx);
    if (!Refs)
    {
        return 0;
    }

    const int32 StartNum = OutAnchors.Num();
    for (const FCellInstanceRef& Ref : *Refs)
    {
        if (OutAnchors.Num() - StartNum >= MaxAnchors)
        {
            break;
        }
        if (!FloraComponents.IsValidIndex(Ref.ComponentIndex) || !FloraComponents[Ref.ComponentIndex])
        {
            continue;
        }

        FTransform InstanceTransform;
        if (FloraComponents[Ref.ComponentIndex]->GetInstanceTransform(
                Ref.InstanceIndex, InstanceTransform, /*bWorldSpace=*/true))
        {
            OutAnchors.Add(InstanceTransform.GetLocation());
        }
    }

    return OutAnchors.Num() - StartNum;
}

void AForestDresser::BuildForest(Grid* WorldGrid, const FVector& GridOriginWorld, float CellSizeUU, int32 SimSeed,
                                 const AProceduralTerrain* Terrain)
{
    if (!WorldGrid)
    {
        return;
    }

    const int32 W = WorldGrid->Width;
    const int32 H = WorldGrid->Height;
    const int32 Seed = (SeedOverride != 0) ? SeedOverride : SimSeed;

    // Tear down previous build (rebuilds happen on reset / grid-size change).
    for (const TObjectPtr<UHierarchicalInstancedStaticMeshComponent>& C : FloraComponents)
    {
        if (C)
        {
            C->DestroyComponent();
        }
    }
    FloraComponents.Reset();
    NumInstancesSpawned = 0;
    CellToVegetationInstances.Reset();
    ScorchedCells.Reset();
    ScorchedInstances.Reset();
    BuiltGridW = W;

    // Effective mesh lists (legacy single-tree slot folded in).
    TArray<UStaticMesh*> Trees;
    for (const TObjectPtr<UStaticMesh>& M : TreeMeshes) if (M) Trees.Add(M);
    if (TreeMesh && !Trees.Contains(TreeMesh.Get())) Trees.Add(TreeMesh);
    TArray<UStaticMesh*> Bushes;
    for (const TObjectPtr<UStaticMesh>& M : BushMeshes) if (M) Bushes.Add(M);
    TArray<UStaticMesh*> Cover;
    for (const TObjectPtr<UStaticMesh>& M : GroundCoverMeshes) if (M) Cover.Add(M);
    TArray<UStaticMesh*> SurfaceDebris;
    for (const TObjectPtr<UStaticMesh>& M : SurfaceDebrisMeshes) if (M) SurfaceDebris.Add(M);
    TArray<UStaticMesh*> SparseRockCover;

    // Zero-config flora: when nothing is assigned, pick up the standard
    // Poly Haven set imported by Tools/import_polyhaven.py (if it exists).
    if (bAutoPopulateFromThesisFlora && Trees.Num() == 0 && Bushes.Num() == 0 && Cover.Num() == 0
        && SurfaceDebris.Num() == 0)
    {
        auto TryLoad = [](const TCHAR* AssetName) -> UStaticMesh*
        {
            const FString Path = FString::Printf(
                TEXT("/Game/ThesisFlora/%s/SM_%s.SM_%s"), AssetName, AssetName, AssetName);
            return LoadObject<UStaticMesh>(nullptr, *Path);
        };
        // Portuguese forest canopy: Maritime pine (pine_sapling_small) and
        // Fir sapling (stands in for cork-oak / Stone-pine saplings).
        // quiver_tree_02 (African quiver tree) and searsia_lucida (S. African
        // karee) have been removed — they look like savanna, not maquis.
        // Canopy set updated to the European Beech (Megascans) pack: a green
        // Atlantic mountain canopy that reads as a Portuguese (Geres-like)
        // forest from drone altitude. Full mature trees as canopy, saplings as
        // understory, seedlings as ground cover. Do not use the boulder as
        // regular ground cover: one rock draw per cell turns the map into a
        // boulder field instead of a forest floor.
        auto LoadByPath = [](const TCHAR* ObjectPath) -> UStaticMesh*
        {
            return LoadObject<UStaticMesh>(nullptr, ObjectPath);
        };
        const TCHAR* BeechCanopy[] = {
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Forest_02.SM_EuropeanBeech_Forest_02"),
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Forest_04.SM_EuropeanBeech_Forest_04"),
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Forest_06.SM_EuropeanBeech_Forest_06"),
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Forest_08.SM_EuropeanBeech_Forest_08"),
        };
        for (const TCHAR* P : BeechCanopy)
            if (UStaticMesh* M = LoadByPath(P)) Trees.Add(M);
        const TCHAR* BeechUnderstory[] = {
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Sapling_01.SM_EuropeanBeech_Sapling_01"),
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Sapling_03.SM_EuropeanBeech_Sapling_03"),
        };
        for (const TCHAR* P : BeechUnderstory)
            if (UStaticMesh* M = LoadByPath(P)) Bushes.Add(M);
        const TCHAR* BeechCover[] = {
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Seedling_01.SM_EuropeanBeech_Seedling_01"),
            TEXT("/Game/EuropeanBeech/Geometry/SimpleWind/SM_EuropeanBeech_Seedling_03.SM_EuropeanBeech_Seedling_03"),
        };
        for (const TCHAR* P : BeechCover)
            if (UStaticMesh* M = LoadByPath(P)) Cover.Add(M);

        // Optional CC0 Poly Haven Geres replacement set from
        // Tools/download_geres_polyhaven.py + Tools/import_polyhaven.py.
        // Keep rock faces / mountainsides manual: they are too large for
        // per-cell procedural scatter.
        const TCHAR* PolyHavenUnderstory[] = {
            TEXT("shrub_02"),
            TEXT("shrub_03"),
            TEXT("shrub_04"),
        };
        for (const TCHAR* Name : PolyHavenUnderstory)
            if (UStaticMesh* M = TryLoad(Name)) Bushes.Add(M);

        const TCHAR* PolyHavenGroundCover[] = {
            TEXT("fern_02"),
            TEXT("moss_01"),
            TEXT("grass_medium_01"),
            TEXT("grass_medium_02"),
        };
        for (const TCHAR* Name : PolyHavenGroundCover)
            if (UStaticMesh* M = TryLoad(Name)) Cover.Add(M);

        const TCHAR* PolyHavenOrganicCover[] = {
            TEXT("dead_tree_trunk"),
            TEXT("dead_tree_trunk_02"),
            TEXT("dry_branches_medium_01"),
            TEXT("pine_roots"),
            TEXT("bark_debris_01"),
        };
        for (const TCHAR* Name : PolyHavenOrganicCover)
            if (UStaticMesh* M = TryLoad(Name)) SurfaceDebris.Add(M);

        if (UStaticMesh* M = TryLoad(TEXT("tree_stump_02")))
        {
            Cover.Add(M);
        }

        const TCHAR* PolyHavenSparseRocks[] = {
            TEXT("rock_moss_set_01"),
            TEXT("rock_moss_set_02"),
            TEXT("rock_07"),
            TEXT("rock_09"),
        };
        for (const TCHAR* Name : PolyHavenSparseRocks)
            if (UStaticMesh* M = TryLoad(Name)) SparseRockCover.Add(M);

        // Fallback: legacy Poly Haven set if the beech pack is absent.
        if (Trees.Num() == 0)
        {
            for (const TCHAR* Name : { TEXT("fir_sapling"), TEXT("pine_sapling_small") })
                if (UStaticMesh* M = TryLoad(Name)) Trees.Add(M);
            if (UStaticMesh* M = TryLoad(TEXT("tree_stump_01"))) Cover.Add(M);
            if (UStaticMesh* M = TryLoad(TEXT("boulder_01"))) SparseRockCover.Add(M);
        }

        if (Cover.Num() == 0)
        {
            for (const TCHAR* Name : { TEXT("tree_stump_01"), TEXT("tree_stump_02") })
                if (UStaticMesh* M = TryLoad(Name)) Cover.Add(M);
        }

        UE_LOG(LogTemp, Log,
               TEXT("Thesis forest: auto-populated flora from /Game/EuropeanBeech + /Game/ThesisFlora (%d trees, %d bushes, %d upright cover, %d debris, %d sparse rocks)."),
               Trees.Num(), Bushes.Num(), Cover.Num(), SurfaceDebris.Num(), SparseRockCover.Num());
        if (Trees.Num() == 0)
        {
            UE_LOG(LogTemp, Warning,
                   TEXT("Thesis forest: /Game/ThesisFlora is empty — run Tools/download_flora.py then Tools/import_polyhaven.py, or assign meshes on the ForestDresser."));
        }
    }

    // Residency changes storage, not geometry, placement, pool ordering or RNG.
    // Never rebuild imported meshes during a simulation tick.
    if (bPreferFullyResidentMeshes)
    {
        int32 ResidentMeshCount = 0;
        auto PreferResident = [&ResidentMeshCount](TArray<UStaticMesh*>& Meshes)
        {
            for (UStaticMesh*& Mesh : Meshes)
            {
                const FString SourcePath = Mesh->GetPathName();
                if (!SourcePath.StartsWith(TEXT("/Game/")) || SourcePath.StartsWith(TEXT("/Game/ThesisSensorMeshes/"))) continue;
                const FString ResidentPath = TEXT("/Game/ThesisSensorMeshes/") + SourcePath.Mid(6);
                if (!FPackageName::DoesPackageExist(FPackageName::ObjectPathToPackageName(ResidentPath))) continue;
                UStaticMesh* Resident = LoadObject<UStaticMesh>(nullptr, *ResidentPath);
                if (Resident && Resident->GetNaniteSettings().bEnabled &&
                    Resident->GetNaniteSettings().TargetMinimumResidencyInKB == MAX_uint32)
                {
                    Mesh = Resident;
                    ++ResidentMeshCount;
                }
            }
        };
        PreferResident(Trees);
        PreferResident(Bushes);
        PreferResident(Cover);
        PreferResident(SurfaceDebris);
        PreferResident(SparseRockCover);
        UE_LOG(LogTemp, Display, TEXT("Thesis forest: selected %d fully resident Nanite meshes"), ResidentMeshCount);
    }

    // Noise offset derived from the seed so different seeds give different woods.
    FRandomStream SeedStream(Seed);
    const FVector2D NoiseOffset(
        SeedStream.FRandRange(-1000.0f, 1000.0f),
        SeedStream.FRandRange(-1000.0f, 1000.0f));

    const float Threshold = 1.0f - FMath::Clamp(ForestCoverage, 0.0f, 1.0f);
    const float InvScale = 1.0f / FMath::Max(1.0f, NoiseScaleCells);

    const FString DensityKey = FString::Printf(TEXT("%d:%d:%d"), W, H, Seed);
    const bool bReuseDisabled = FParse::Param(FCommandLine::Get(), TEXT("ThesisDisableSceneReuse"));
    const int32 DensityCount = W * H;
    TArray<float> DensityValues;
    bool bDensityCacheHit = false;
    if (!bReuseDisabled)
    {
        for (const FForestDensityCacheEntry& Entry : GForestDensityCache)
        {
            if (Entry.Key == DensityKey && Entry.Coverage == ForestCoverage
                && Entry.NoiseScale == NoiseScaleCells && Entry.Values.Num() == DensityCount)
            {
                DensityValues = Entry.Values;
                bDensityCacheHit = true;
                break;
            }
        }
    }
    if (!bDensityCacheHit)
    {
        DensityValues.SetNumUninitialized(DensityCount);
        for (int32 Y = 0; Y < H; ++Y)
        {
            for (int32 X = 0; X < W; ++X)
            {
                const FVector2D P = (FVector2D(X, Y) + NoiseOffset) * InvScale;
                const float Noise01 = 0.5f * (FMath::PerlinNoise2D(P) + 1.0f);
                DensityValues[Y * W + X] = (Noise01 <= Threshold || Threshold >= 1.0f)
                    ? 0.0f
                    : FMath::Clamp((Noise01 - Threshold) / FMath::Max(0.0001f, 1.0f - Threshold), 0.0f, 1.0f);
            }
        }
        if (!bReuseDisabled)
        {
            if (GForestDensityCache.Num() >= 4) GForestDensityCache.RemoveAt(0);
            FForestDensityCacheEntry& Entry = GForestDensityCache.AddDefaulted_GetRef();
            Entry.Key = DensityKey;
            Entry.Coverage = ForestCoverage;
            Entry.NoiseScale = NoiseScaleCells;
            Entry.Values = DensityValues;
        }
    }

    // Batched transforms (one AddInstances call per bounded mesh component).
    // Components are split by mesh, visual role, and spatial tile. Grouping
    // raster-order instances alone lets a mesh's bounds span most of the map,
    // reducing useful frustum and shadow culling.
    TMap<UStaticMesh*, TMap<uint8, TMap<int64, TArray<int32>>>> MeshRoleTileComponents;
    TMap<UStaticMesh*, int32> MeshChunkLimits;
    TMap<UStaticMesh*, TMap<uint8, TMap<int64, int32>>> MeshRoleTileInstanceCounts;
    TArray<TArray<FTransform>> Batches;
    TArray<FForestLayoutItem> GeneratedLayout;
    TArray<UStaticMesh*> LayoutMeshAssets;
    TMap<UStaticMesh*, int32> LayoutMeshIndices;
    auto AddLayoutMeshes = [&](const TArray<UStaticMesh*>& Meshes)
    {
        for (UStaticMesh* M : Meshes)
        {
            if (M && !LayoutMeshIndices.Contains(M))
            {
                LayoutMeshIndices.Add(M, LayoutMeshAssets.Num());
                LayoutMeshAssets.Add(M);
            }
        }
    };
    AddLayoutMeshes(Trees); AddLayoutMeshes(Bushes); AddLayoutMeshes(Cover);
    AddLayoutMeshes(SurfaceDebris); AddLayoutMeshes(SparseRockCover);
    bool bCaptureGeneratedLayout = false;
    const IConsoleVariable* TriangleBudgetCVar =
        IConsoleManager::Get().FindConsoleVariable(TEXT("foliage.MaxTrianglesToRender"));
    const int64 TriangleBudget = TriangleBudgetCVar
        ? FMath::Max<int64>(0, TriangleBudgetCVar->GetInt())
        : 100000000;

    auto AddInstance = [&](UStaticMesh* Mesh, const FTransform& T, int32 CellIdx,
                           bool bVegetation, uint8 ComponentRole, bool bCastShadow,
                           int32 CellX, int32 CellY)
    {
        if (!Mesh)
        {
            return;
        }
        // Visual-only: in headless/no-capture runs we still write floraDensity
        // (the fuel map) but skip all HISM instancing, which nothing renders.
        if (!bSpawnVisualInstances)
        {
            return;
        }
        if (bCaptureGeneratedLayout)
        {
            FForestLayoutItem& Item = GeneratedLayout.AddDefaulted_GetRef();
            Item.MeshIndex = LayoutMeshIndices.FindChecked(Mesh);
            Item.Transform = T;
            Item.CellIdx = CellIdx;
            Item.CellX = CellX;
            Item.CellY = CellY;
            Item.bVegetation = bVegetation;
            Item.ComponentRole = ComponentRole;
            Item.bCastShadow = bCastShadow;
        }
        int32* ChunkLimit = MeshChunkLimits.Find(Mesh);
        if (!ChunkLimit)
        {
            int64 MaxTrianglesPerInstance = 0;
            if (const FStaticMeshRenderData* RenderData = Mesh->GetRenderData())
            {
                for (const FStaticMeshLODResources& LOD : RenderData->LODResources)
                {
                    MaxTrianglesPerInstance = FMath::Max<int64>(MaxTrianglesPerInstance, LOD.GetNumTriangles());
                }
            }
            ChunkLimit = &MeshChunkLimits.Add(
                Mesh, ForestDresserBatching::ComputeInstancesPerComponent(MaxTrianglesPerInstance, TriangleBudget));
            if (MaxTrianglesPerInstance > 0 && TriangleBudget > 0
                && MaxTrianglesPerInstance > (TriangleBudget - 1) / 2)
            {
                UE_LOG(LogTemp, Warning,
                    TEXT("Forest mesh %s has %lld triangles in one instance, above the safe per-instance foliage budget %lld."),
                    *Mesh->GetPathName(), MaxTrianglesPerInstance, (TriangleBudget - 1) / 2);
            }
        }

        const int32 TileSide = FMath::Max(0, HISMSpatialTileSideCells);
        const int64 TileKey = TileSide == 0 ? 0 : ForestDresserBatching::MakeSpatialTileKey(
            ForestDresserBatching::GetSpatialTileCoordinate(CellX, TileSide),
            ForestDresserBatching::GetSpatialTileCoordinate(CellY, TileSide));
        TMap<uint8, TMap<int64, TArray<int32>>>& RoleMaps = MeshRoleTileComponents.FindOrAdd(Mesh);
        TArray<int32>& ComponentIndices = RoleMaps.FindOrAdd(ComponentRole).FindOrAdd(TileKey);
        int32& MeshInstanceCount = MeshRoleTileInstanceCounts.FindOrAdd(Mesh)
            .FindOrAdd(ComponentRole).FindOrAdd(TileKey, 0);
        const int32 MeshInstanceOrdinal = MeshInstanceCount++;
        const ForestDresserBatching::FChunkAddress ChunkAddress =
            ForestDresserBatching::GetChunkAddress(MeshInstanceOrdinal, *ChunkLimit);
        if (ComponentIndices.Num() <= ChunkAddress.ComponentOrdinal)
        {
            if (!GetOrCreateComponentForMesh(Mesh, ComponentIndices,
                                             ChunkAddress.ComponentOrdinal, bCastShadow))
            {
                return;
            }
        }
        const int32 Idx = ComponentIndices[ChunkAddress.ComponentOrdinal];
        if (Batches.Num() <= Idx)
        {
            Batches.SetNum(Idx + 1);
        }
        const int32 InstIdx = Batches[Idx].Num();
        check(InstIdx == ChunkAddress.InstanceOrdinal);
        Batches[Idx].Add(T);
        if (bVegetation && CellIdx != INDEX_NONE)
        {
            CellToVegetationInstances.FindOrAdd(CellIdx).Add({ Idx, InstIdx });
        }
    };

    auto GroundZ = [&](float CX, float CY) -> float
    {
        return Terrain ? Terrain->GetHeightAtCell(CX, CY) : GridOriginWorld.Z;
    };

    auto GroundNormal = [&](float CX, float CY) -> FVector
    {
        return Terrain ? Terrain->GetNormalAtCell(CX, CY) : FVector::UpVector;
    };

    enum class EPlacementFit : uint8
    {
        Upright,
        SurfaceAlignedBounds
    };

    auto MakeTransform = [&](UStaticMesh* Mesh, float CellX, float CellY, FRandomStream& Stream,
                             float ScaleMin, float ScaleMax, EPlacementFit Fit,
                             float EmbedUU) -> FTransform
    {
        const float OffX = Stream.FRandRange(0.05f, 0.95f);
        const float OffY = Stream.FRandRange(0.05f, 0.95f);
        const float Yaw = Stream.FRandRange(0.0f, 360.0f);
        const float Scale = Stream.FRandRange(ScaleMin, ScaleMax);
        const float CX = CellX + OffX;
        const float CY = CellY + OffY;

        const FVector SurfacePoint(
            GridOriginWorld.X + CX * CellSizeUU,
            GridOriginWorld.Y + CY * CellSizeUU,
            GroundZ(CX, CY));

        FVector Location = SurfacePoint;
        FQuat Rotation = FQuat(FRotator(0.0f, Yaw, 0.0f));

        if (Fit == EPlacementFit::SurfaceAlignedBounds)
        {
            const FVector Normal = GroundNormal(CX, CY);
            const FQuat AlignToNormal = FQuat::FindBetweenNormals(FVector::UpVector, Normal);
            const FQuat YawAroundNormal(Normal, FMath::DegreesToRadians(Yaw));
            Rotation = YawAroundNormal * AlignToNormal;

            if (Mesh)
            {
                const FBoxSphereBounds Bounds = Mesh->GetBounds();
                const float LocalBottomZ = Bounds.Origin.Z - Bounds.BoxExtent.Z;
                const FVector BottomOffset = Rotation.RotateVector(FVector(0.0f, 0.0f, LocalBottomZ * Scale));
                Location = SurfacePoint - BottomOffset - Normal * FMath::Max(0.0f, EmbedUU);
            }
            else
            {
                Location = SurfacePoint - Normal * FMath::Max(0.0f, EmbedUU);
            }
        }
        else
        {
            Location.Z += TreeZOffsetUU;
        }

        FTransform T;
        T.SetLocation(Location);
        T.SetRotation(Rotation);
        T.SetScale3D(FVector(Scale));
        return T;
    };

    FString LayoutKey;
    auto KeyInt = [&LayoutKey](int32 V) { LayoutKey.Appendf(TEXT("i%d;"), V); };
    auto KeyInt64 = [&LayoutKey](int64 V) { LayoutKey.Appendf(TEXT("l%lld;"), V); };
    auto KeyFloat = [&LayoutKey](float V) { uint32 B = 0; FMemory::Memcpy(&B, &V, sizeof(B)); LayoutKey.Appendf(TEXT("f%08x;"), B); };
    auto KeyDouble = [&LayoutKey](double V) { LayoutKey.Appendf(TEXT("d%016llx;"), ForestDoubleBits(V)); };
    auto KeyMeshes = [&LayoutKey](TCHAR Category, const TArray<UStaticMesh*>& Meshes)
    {
        LayoutKey.AppendChar(Category);
        for (const UStaticMesh* M : Meshes)
        {
            LayoutKey.Appendf(TEXT("%s;"), *M->GetPathName());
            const FBoxSphereBounds Bounds = M->GetBounds();
            auto AppendBits = [&LayoutKey](double V)
            {
                LayoutKey.Appendf(TEXT("%016llx,"), ForestDoubleBits(V));
            };
            AppendBits(Bounds.Origin.X); AppendBits(Bounds.Origin.Y); AppendBits(Bounds.Origin.Z);
            AppendBits(Bounds.BoxExtent.X); AppendBits(Bounds.BoxExtent.Y); AppendBits(Bounds.BoxExtent.Z);
            if (const FStaticMeshRenderData* RenderData = M->GetRenderData())
            {
                for (const FStaticMeshLODResources& LOD : RenderData->LODResources)
                    LayoutKey.Appendf(TEXT("t%d,"), LOD.GetNumTriangles());
            }
        }
        LayoutKey.AppendChar(TEXT('|'));
    };
    KeyInt(W); KeyInt(H); KeyInt(Seed); KeyInt(SimSeed);
    KeyDouble(GridOriginWorld.X); KeyDouble(GridOriginWorld.Y); KeyDouble(GridOriginWorld.Z); KeyFloat(CellSizeUU);
    KeyFloat(ForestCoverage); KeyFloat(NoiseScaleCells); KeyInt(PresentationMarginCells);
    KeyInt(PresentationApronMaxPatches); KeyInt(MaxTreesPerCell); KeyInt(MaxBushesPerCell);
    KeyInt(GroundCoverPerCell); KeyInt(SurfaceDebrisPerCell); KeyFloat(SurfaceDebrisEmbedUU);
    KeyFloat(SparseRockEmbedUU); KeyFloat(TreeScaleMin); KeyFloat(TreeScaleMax);
    KeyFloat(TreeZOffsetUU);
    KeyInt(HISMSpatialTileSideCells); KeyInt(bGroundCoverCastsShadow ? 1 : 0);
    KeyInt(bSurfaceDebrisCastsShadow ? 1 : 0); KeyInt(bSpawnVisualInstances ? 1 : 0);
    KeyMeshes('T', Trees); KeyMeshes('B', Bushes); KeyMeshes('C', Cover);
    KeyMeshes('D', SurfaceDebris); KeyMeshes('R', SparseRockCover);
    KeyInt(Terrain ? 1 : 0);
    if (Terrain)
    {
        KeyInt(static_cast<int32>(Terrain->Profile)); KeyFloat(Terrain->SlopeIntensity);
        KeyFloat(Terrain->DetailDensity); KeyInt(Terrain->SeedOverride);
        KeyInt(Terrain->PresentationMarginCells); KeyFloat(Terrain->PresentationMarginMeters);
    }
    KeyInt64(TriangleBudget);
    if (Terrain) LayoutKey.Append(Terrain->GetHeightFieldSignature());
    const bool bLayoutCacheHit = bSpawnVisualInstances && !bReuseDisabled
        && GForestLayoutCache.Key == LayoutKey && GForestLayoutCache.Items.Num() > 0;
    TArray<UStaticMesh*> ResolvedLayoutMeshes;
    if (bLayoutCacheHit)
    {
        TMap<FString, UStaticMesh*> MeshByPath;
        for (UStaticMesh* M : LayoutMeshAssets) if (M) MeshByPath.Add(M->GetPathName(), M);
        ResolvedLayoutMeshes.Reserve(GForestLayoutCache.MeshPaths.Num());
        for (const FString& Path : GForestLayoutCache.MeshPaths)
        {
            UStaticMesh* const* Found = MeshByPath.Find(Path);
            ResolvedLayoutMeshes.Add(Found ? *Found : LoadObject<UStaticMesh>(nullptr, *Path));
        }
    }
    bCaptureGeneratedLayout = bSpawnVisualInstances && !bReuseDisabled && !bLayoutCacheHit;

    int32 ApronPatchesVisited = 0;
    int32 ApronCanopyCount = 0;
    int32 ApronBushCount = 0;
    int32 ApronCoverCount = 0;
    int32 ApronDebrisCount = 0;
    int32 ApronRockCount = 0;
    if (bLayoutCacheHit)
    {
        // The detailed raster loop also publishes the immutable fuel field.
        // Reapply it on cache hits before replaying visual layout descriptors.
        for (int32 Y = 0; Y < H; ++Y)
            for (int32 X = 0; X < W; ++X)
                WorldGrid->setFloraDensity(X, Y, DensityValues[Y * W + X]);
        for (const FForestLayoutItem& Item : GForestLayoutCache.Items)
        {
            UStaticMesh* M = ResolvedLayoutMeshes.IsValidIndex(Item.MeshIndex)
                ? ResolvedLayoutMeshes[Item.MeshIndex] : nullptr;
            AddInstance(M, Item.Transform, Item.CellIdx, Item.bVegetation,
                Item.ComponentRole, Item.bCastShadow, Item.CellX, Item.CellY);
        }
    }
    else
    {
    const int32 TerrainMarginCells = Terrain
        ? FMath::Clamp(Terrain->PresentationMarginCells, 0, 64)
        : 64;
    const int32 MarginCells = FMath::Clamp(PresentationMarginCells, 0, TerrainMarginCells);
    for (int32 y = -MarginCells; y < H + MarginCells; ++y)  // fixed raster order — deterministic
    {
        for (int32 x = -MarginCells; x < W + MarginCells; ++x)
        {
            // Perlin in [-1,1] -> [0,1]
            // Interior fuel density is a cached exact result; margin-only
            // placement remains evaluated with the original seeded noise.
            float Density;
            if (x >= 0 && x < W && y >= 0 && y < H)
            {
                Density = DensityValues[y * W + x];
            }
            else
            {
                const FVector2D P = (FVector2D(x, y) + NoiseOffset) * InvScale;
                const float Noise01 = 0.5f * (FMath::PerlinNoise2D(P) + 1.0f);
                Density = (Noise01 <= Threshold || Threshold >= 1.0f)
                    ? 0.0f
                    : FMath::Clamp((Noise01 - Threshold) / FMath::Max(0.0001f, 1.0f - Threshold), 0.0f, 1.0f);
            }

            // INVARIANT (ground-truth/belief wall): floraDensity is ground truth /
            // fuel model input — never belief. The manager exports it as the fire
            // generator's fuel map, so the fire burns where these trees stand.
            const bool bInsideGrid = x >= 0 && x < W && y >= 0 && y < H;
            if (bInsideGrid)
            {
                WorldGrid->setFloraDensity(x, y, Density);
            }

            // Exterior decoration shares the interior's coordinate-derived
            // placement stream without creating grid-index aliases.
            const int32 GridCellIdx = bInsideGrid ? y * W + x : INDEX_NONE;

            // Per-cell stream: placement independent of neighbouring cells.
            const uint32 CellSeed = bInsideGrid
                ? HashCombine(GetTypeHash(Seed), GetTypeHash(y * W + x))
                : HashCombine(GetTypeHash(Seed),
                              HashCombine(GetTypeHash(x), GetTypeHash(y)));
            FRandomStream CellStream(CellSeed);

            // ---- Canopy ----
            if (Trees.Num() > 0 && Density > 0.0f && MaxTreesPerCell > 0)
            {
                const int32 NumTrees = FMath::RoundToInt(Density * MaxTreesPerCell);
                for (int32 t = 0; t < NumTrees; ++t)
                {
                    UStaticMesh* Mesh = Trees[CellStream.RandRange(0, Trees.Num() - 1)];
                    AddInstance(Mesh, MakeTransform(Mesh, x, y, CellStream, TreeScaleMin, TreeScaleMax,
                                                    EPlacementFit::Upright, 0.0f),
                                GridCellIdx, /*bVegetation=*/true, /*role=*/0, true, x, y);
                }
            }

            // ---- Understory: thrives at forest edges and under sparse canopy ----
            if (Bushes.Num() > 0 && Density > 0.02f && MaxBushesPerCell > 0)
            {
                const float EdgeFactor = 1.0f - FMath::Abs(Density - 0.4f);   // peaks at mid density
                const int32 NumBushes = FMath::RoundToInt(
                    FMath::Clamp(EdgeFactor, 0.0f, 1.0f) * MaxBushesPerCell);
                for (int32 b = 0; b < NumBushes; ++b)
                {
                    UStaticMesh* Mesh = Bushes[CellStream.RandRange(0, Bushes.Num() - 1)];
                    AddInstance(Mesh, MakeTransform(Mesh, x, y, CellStream, 0.5f, 1.0f,
                                                    EPlacementFit::Upright, 0.0f),
                                GridCellIdx, /*bVegetation=*/true, /*role=*/0, true, x, y);
                }
            }

            // ---- Ground cover: everywhere, clearings included ----
            if (Cover.Num() > 0 && GroundCoverPerCell > 0)
            {
                for (int32 g = 0; g < GroundCoverPerCell; ++g)
                {
                    UStaticMesh* Mesh = Cover[CellStream.RandRange(0, Cover.Num() - 1)];
                    AddInstance(Mesh, MakeTransform(Mesh, x, y, CellStream, 0.6f, 1.2f,
                                                    EPlacementFit::Upright, 0.0f),
                                GridCellIdx, /*bVegetation=*/false, /*role=*/1,
                                bGroundCoverCastsShadow, x, y);
                }
            }

            // Dead trunks, branches and bark lie on the terrain rather than
            // standing upright.  Fit them to the local normal and sink by a
            // small deterministic amount so slopes do not leave them hovering.
            if (SurfaceDebris.Num() > 0 && SurfaceDebrisPerCell > 0)
            {
                const float DebrisChance = FMath::Lerp(0.28f, 0.12f, Density);
                for (int32 d = 0; d < SurfaceDebrisPerCell; ++d)
                {
                    if (CellStream.FRand() < DebrisChance)
                    {
                        UStaticMesh* Mesh = SurfaceDebris[CellStream.RandRange(0, SurfaceDebris.Num() - 1)];
                        AddInstance(Mesh, MakeTransform(Mesh, x, y, CellStream, 0.50f, 0.95f,
                                                        EPlacementFit::SurfaceAlignedBounds, SurfaceDebrisEmbedUU),
                                    GridCellIdx, /*bVegetation=*/false, /*role=*/2,
                                    bSurfaceDebrisCastsShadow, x, y);
                    }
                }
            }

            // Rocks are important for Geres, but large rocks per cell quickly
            // read as a quarry. Keep them as sparse accents, more common in
            // clearings where the forest floor is exposed.
            if (SparseRockCover.Num() > 0)
            {
                const float RockChance = FMath::Lerp(0.12f, 0.04f, Density);
                if (CellStream.FRand() < RockChance)
                {
                    UStaticMesh* Mesh = SparseRockCover[CellStream.RandRange(0, SparseRockCover.Num() - 1)];
                    AddInstance(Mesh, MakeTransform(Mesh, x, y, CellStream, 0.45f, 0.85f,
                                                    EPlacementFit::SurfaceAlignedBounds, SparseRockEmbedUU),
                                GridCellIdx, /*bVegetation=*/false, /*role=*/3, true, x, y);
                }
            }
        }
    }

    // Fill the distant terrain mesh with a bounded coarse flora continuation.
    // The detailed edge band above remains unchanged; this separate scatter
    // uses a private coordinate-seeded stream and never receives a grid cell
    // index, so it cannot alter fuel, fire anchors, or scorch bookkeeping.
    TSet<UStaticMesh*> ApronMeshesUsed;
    const float OuterApronMarginCells = Terrain && CellSizeUU > 0.0f
        ? FMath::Max(static_cast<float>(TerrainMarginCells),
            FMath::Clamp(Terrain->PresentationMarginMeters, 0.0f, 10000.0f) * 100.0f / CellSizeUU)
        : static_cast<float>(MarginCells);
    if (bSpawnVisualInstances && Terrain && PresentationApronMaxPatches > 0
        && OuterApronMarginCells > MarginCells + 1.0f
        && CellSizeUU > 0.0f && (Trees.Num() > 0 || Bushes.Num() > 0 || Cover.Num() > 0
                                || SurfaceDebris.Num() > 0 || SparseRockCover.Num() > 0))
    {
        // A sub-cell fringe cannot fit a coarse patch while keeping the
        // existing detailed band and full transforms inside terrain bounds.
        const float OuterMarginCells = OuterApronMarginCells;
        const int32 OuterCells = FMath::CeilToInt(OuterMarginCells);
        const int32 MinX = -OuterCells;
        const int32 MinY = -OuterCells;
        const int32 MaxX = W + OuterCells;
        const int32 MaxY = H + OuterCells;
        const int64 ExtendedArea = static_cast<int64>(MaxX - MinX) * (MaxY - MinY);
        const int64 DetailedArea = static_cast<int64>(W + MarginCells * 2) * (H + MarginCells * 2);
        const int64 ApronArea = FMath::Max<int64>(0, ExtendedArea - DetailedArea);
        const int32 DetailMinX = -MarginCells;
        const int32 DetailMinY = -MarginCells;
        const int32 DetailMaxX = W + MarginCells;
        const int32 DetailMaxY = H + MarginCells;
        // Reserve part of the patch budget for a fine, dense annulus just
        // beyond the existing detail band. Those patches carry mature canopy
        // and ground layers across the grid edge before the distant scatter
        // becomes coarser.
        const float NearBandCells = FMath::Min(24.0f,
            FMath::Max(0.0f, OuterMarginCells - MarginCells - 1.0f));
        const int32 NearMinX = FMath::Max(MinX, FMath::FloorToInt(DetailMinX - NearBandCells));
        const int32 NearMinY = FMath::Max(MinY, FMath::FloorToInt(DetailMinY - NearBandCells));
        const int32 NearMaxX = FMath::Min(MaxX, FMath::CeilToInt(DetailMaxX + NearBandCells));
        const int32 NearMaxY = FMath::Min(MaxY, FMath::CeilToInt(DetailMaxY + NearBandCells));
        const int64 NearArea = FMath::Max<int64>(0,
            static_cast<int64>(NearMaxX - NearMinX) * (NearMaxY - NearMinY) - DetailedArea);
        const int32 NearPatchBudget = NearBandCells > 0.0f && PresentationApronMaxPatches > 1
            ? FMath::Clamp(PresentationApronMaxPatches / 5, 1, PresentationApronMaxPatches - 1) : 0;
        int32 NearPatchSide = NearPatchBudget > 0
            ? FMath::Max(1, FMath::CeilToInt(FMath::Sqrt(
                static_cast<double>(NearArea) / NearPatchBudget))) : 1;
        const int32 CoarsePatchBudget = FMath::Max(1,
            PresentationApronMaxPatches - NearPatchBudget);
        int32 PatchSide = FMath::Max(1, FMath::CeilToInt(FMath::Sqrt(
            static_cast<double>(ApronArea) / CoarsePatchBudget)));
        auto CountIntervals = [](int32 Min, int32 Max, int32 Side)
        {
            return (Max - Min + Side - 1) / Side;
        };
        auto CountInteriorIntervals = [](int32 Min, int32 DetailMin, int32 DetailMax, int32 Side)
        {
            const int32 First = FMath::DivideAndRoundUp(DetailMin - Min, Side);
            const int32 End = FMath::Max(First, (DetailMax - Min) / Side);
            return End - First;
        };
        if (NearPatchBudget > 0)
        {
            const int32 MaxNearPatchSide = FMath::Max(NearMaxX - NearMinX, NearMaxY - NearMinY);
            while (NearPatchSide < MaxNearPatchSide)
            {
                const int64 NearTotalPatches = static_cast<int64>(CountIntervals(
                    NearMinX, NearMaxX, NearPatchSide))
                    * CountIntervals(NearMinY, NearMaxY, NearPatchSide);
                const int64 NearInteriorPatches = static_cast<int64>(CountInteriorIntervals(
                    NearMinX, DetailMinX, DetailMaxX, NearPatchSide))
                    * CountInteriorIntervals(NearMinY, DetailMinY, DetailMaxY, NearPatchSide);
                if (NearTotalPatches - NearInteriorPatches <= NearPatchBudget) break;
                ++NearPatchSide;
            }
        }
        const int64 PatchBudget = CoarsePatchBudget;
        const int32 MaxPatchSide = FMath::Max(MaxX - MinX, MaxY - MinY);
        while (PatchSide < MaxPatchSide)
        {
            const int64 TotalPatchCount = static_cast<int64>(CountIntervals(MinX, MaxX, PatchSide))
                * CountIntervals(MinY, MaxY, PatchSide);
            const int64 InteriorPatchCount = static_cast<int64>(CountInteriorIntervals(
                    MinX, DetailMinX, DetailMaxX, PatchSide))
                * CountInteriorIntervals(MinY, DetailMinY, DetailMaxY, PatchSide);
            if (TotalPatchCount - InteriorPatchCount <= PatchBudget) break;
            ++PatchSide;
        }
        const float BlendDistanceCells = FMath::Max(48.0f, PatchSide * 2.0f);
        const uint32 ApronSeed = HashCombine(GetTypeHash(Seed), 0xA93F5C17u);
        auto KeepOutsideDetail = [&](float& PositionX, float& PositionY)
        {
            PositionX = FMath::Clamp(PositionX, -OuterMarginCells, W + OuterMarginCells - 0.95f);
            PositionY = FMath::Clamp(PositionY, -OuterMarginCells, H + OuterMarginCells - 0.95f);
            if (PositionX >= DetailMinX && PositionX < DetailMaxX
                && PositionY >= DetailMinY && PositionY < DetailMaxY)
            {
                const float ToWest = PositionX - DetailMinX;
                const float ToEast = DetailMaxX - PositionX;
                const float ToSouth = PositionY - DetailMinY;
                const float ToNorth = DetailMaxY - PositionY;
                const float Nearest = FMath::Min(FMath::Min(ToWest, ToEast), FMath::Min(ToSouth, ToNorth));
                if (Nearest == ToWest) PositionX = DetailMinX - 1.0f;
                else if (Nearest == ToEast) PositionX = DetailMaxX;
                else if (Nearest == ToSouth) PositionY = DetailMinY - 1.0f;
                else PositionY = DetailMaxY;
            }
            if (PositionX < DetailMinX && PositionX > DetailMinX - 1.0f) PositionX = DetailMinX - 1.0f;
            if (PositionX >= DetailMaxX && PositionX < DetailMaxX + 1.0f) PositionX = DetailMaxX;
            if (PositionY < DetailMinY && PositionY > DetailMinY - 1.0f) PositionY = DetailMinY - 1.0f;
            if (PositionY >= DetailMaxY && PositionY < DetailMaxY + 1.0f) PositionY = DetailMaxY;
        };

        // The first apron band mirrors the mature forest's areal density. A
        // handful of independent placements inside each small patch forms
        // irregular groves instead of isolated saplings on exposed terrain.
        if (NearPatchBudget > 0)
        {
            for (int32 PatchY = NearMinY; PatchY < NearMaxY; PatchY += NearPatchSide)
            {
                const int32 PatchEndY = FMath::Min(NearMaxY, PatchY + NearPatchSide);
                for (int32 PatchX = NearMinX; PatchX < NearMaxX; PatchX += NearPatchSide)
                {
                    const int32 PatchEndX = FMath::Min(NearMaxX, PatchX + NearPatchSide);
                    const int32 InteriorWidth = FMath::Max(0,
                        FMath::Min(PatchEndX, DetailMaxX) - FMath::Max(PatchX, DetailMinX));
                    const int32 InteriorHeight = FMath::Max(0,
                        FMath::Min(PatchEndY, DetailMaxY) - FMath::Max(PatchY, DetailMinY));
                    const int32 PatchCells = (PatchEndX - PatchX) * (PatchEndY - PatchY)
                        - InteriorWidth * InteriorHeight;
                    if (PatchCells <= 0) continue;

                    ++ApronPatchesVisited;
                    const uint32 PatchSeed = HashCombine(HashCombine(ApronSeed, 0xB12E3A57u),
                        HashCombine(GetTypeHash(PatchX), GetTypeHash(PatchY)));
                    FRandomStream PatchFloraStream(static_cast<int32>(HashCombine(PatchSeed, 0x46D12983u)));
                    const float RawX = PatchFloraStream.FRandRange(
                        static_cast<float>(PatchX), static_cast<float>(PatchEndX - 1));
                    const float RawY = PatchFloraStream.FRandRange(
                        static_cast<float>(PatchY), static_cast<float>(PatchEndY - 1));
                    float SampleX = FMath::Clamp(RawX, -OuterMarginCells,
                        W + OuterMarginCells - 0.95f);
                    float SampleY = FMath::Clamp(RawY, -OuterMarginCells,
                        H + OuterMarginCells - 0.95f);
                    if (SampleX >= DetailMinX && SampleX < DetailMaxX
                        && SampleY >= DetailMinY && SampleY < DetailMaxY)
                    {
                        const float ToWest = SampleX - DetailMinX;
                        const float ToEast = DetailMaxX - SampleX;
                        const float ToSouth = SampleY - DetailMinY;
                        const float ToNorth = DetailMaxY - SampleY;
                        const float Nearest = FMath::Min(FMath::Min(ToWest, ToEast), FMath::Min(ToSouth, ToNorth));
                        if (Nearest == ToWest) SampleX = DetailMinX - 1.0f;
                        else if (Nearest == ToEast) SampleX = DetailMaxX;
                        else if (Nearest == ToSouth) SampleY = DetailMinY - 1.0f;
                        else SampleY = DetailMaxY;
                    }
                    if (SampleX < DetailMinX && SampleX > DetailMinX - 1.0f) SampleX = DetailMinX - 1.0f;
                    if (SampleX >= DetailMaxX && SampleX < DetailMaxX + 1.0f) SampleX = DetailMaxX;
                    if (SampleY < DetailMinY && SampleY > DetailMinY - 1.0f) SampleY = DetailMinY - 1.0f;
                    if (SampleY >= DetailMaxY && SampleY < DetailMaxY + 1.0f) SampleY = DetailMaxY;

                    const FVector2D NoisePoint = (FVector2D(SampleX, SampleY) + NoiseOffset) * InvScale;
                    const float Noise01 = 0.5f * (FMath::PerlinNoise2D(NoisePoint) + 1.0f);
                    const float NoiseDensity = (Noise01 <= Threshold || Threshold >= 1.0f)
                        ? 0.0f : FMath::Clamp((Noise01 - Threshold)
                            / FMath::Max(0.0001f, 1.0f - Threshold), 0.0f, 1.0f);
                    const float EdgeX = FMath::Clamp(SampleX, static_cast<float>(DetailMinX),
                                                      static_cast<float>(DetailMaxX - 1));
                    const float EdgeY = FMath::Clamp(SampleY, static_cast<float>(DetailMinY),
                                                      static_cast<float>(DetailMaxY - 1));
                    const FVector2D EdgeNoisePoint = (FVector2D(EdgeX, EdgeY) + NoiseOffset) * InvScale;
                    const float EdgeNoise01 = 0.5f * (FMath::PerlinNoise2D(EdgeNoisePoint) + 1.0f);
                    const float EdgeDensity = (EdgeNoise01 <= Threshold || Threshold >= 1.0f)
                        ? 0.0f : FMath::Clamp((EdgeNoise01 - Threshold)
                            / FMath::Max(0.0001f, 1.0f - Threshold), 0.0f, 1.0f);
                    const float OutsideX = SampleX < DetailMinX ? DetailMinX - SampleX
                        : SampleX >= DetailMaxX ? SampleX - DetailMaxX : 0.0f;
                    const float OutsideY = SampleY < DetailMinY ? DetailMinY - SampleY
                        : SampleY >= DetailMaxY ? SampleY - DetailMaxY : 0.0f;
                    const float Distance = FMath::Max(OutsideX, OutsideY);
                    const float BlendT = FMath::Clamp(Distance / BlendDistanceCells, 0.0f, 1.0f);
                    const float SmoothBlend = BlendT * BlendT * (3.0f - 2.0f * BlendT);
                    const float Density = FMath::Lerp(EdgeDensity, NoiseDensity, SmoothBlend);
                    const float CellArea = static_cast<float>(PatchCells);
                    const float CanopyFade = FMath::Lerp(1.0f, 0.72f, SmoothBlend);
                    const float GroundFade = FMath::Lerp(1.0f, 0.82f, SmoothBlend);

                    auto RoundedCount = [&PatchFloraStream, PatchCells](float Expected, int32 PerPatchMax)
                    {
                        const int32 Whole = FMath::FloorToInt(Expected);
                        const int32 Rounded = Whole + (PatchFloraStream.FRand() < Expected - Whole ? 1 : 0);
                        return FMath::Clamp(Rounded, 0, FMath::Min(PatchCells, PerPatchMax));
                    };
                    auto SpawnNear = [&](TArray<UStaticMesh*>& Meshes, int32 Count,
                                         float ScaleMin, float ScaleMax, EPlacementFit Fit,
                                         float EmbedUU, uint8 ComponentRole, bool bCastShadow,
                                         int32& OutCount)
                    {
                        for (int32 I = 0; I < Count; ++I)
                        {
                            UStaticMesh* Mesh = Meshes[PatchFloraStream.RandRange(0, Meshes.Num() - 1)];
                            const float Radius = NearPatchSide * 0.45f;
                            const float CenterX = FMath::Clamp(SampleX,
                                static_cast<float>(PatchX), static_cast<float>(PatchEndX - 1));
                            const float CenterY = FMath::Clamp(SampleY,
                                static_cast<float>(PatchY), static_cast<float>(PatchEndY - 1));
                            float PositionX = PatchFloraStream.FRandRange(
                                FMath::Max(static_cast<float>(PatchX), CenterX - Radius),
                                FMath::Min(static_cast<float>(PatchEndX - 1), CenterX + Radius));
                            float PositionY = PatchFloraStream.FRandRange(
                                FMath::Max(static_cast<float>(PatchY), CenterY - Radius),
                                FMath::Min(static_cast<float>(PatchEndY - 1), CenterY + Radius));
                            KeepOutsideDetail(PositionX, PositionY);
                            AddInstance(Mesh, MakeTransform(Mesh, PositionX, PositionY, PatchFloraStream,
                                                            ScaleMin, ScaleMax, Fit, EmbedUU),
                                        INDEX_NONE, false, ComponentRole, bCastShadow,
                                        FMath::FloorToInt(PositionX), FMath::FloorToInt(PositionY));
                            ++OutCount;
                            ApronMeshesUsed.Add(Mesh);
                        }
                    };

                    const float CanopyExpected = Density * FMath::Max(0, MaxTreesPerCell)
                        * CellArea * CanopyFade;
                    SpawnNear(Trees, Trees.Num() > 0
                        ? RoundedCount(CanopyExpected, 8) : 0,
                        TreeScaleMin, TreeScaleMax, EPlacementFit::Upright, 0.0f, 0, true, ApronCanopyCount);
                    const float EdgeFactor = FMath::Clamp(1.0f - FMath::Abs(Density - 0.4f), 0.0f, 1.0f);
                    SpawnNear(Bushes, Bushes.Num() > 0
                        ? RoundedCount(EdgeFactor * FMath::Max(0, MaxBushesPerCell) * CellArea * CanopyFade, 8) : 0,
                        0.5f, 1.0f, EPlacementFit::Upright, 0.0f, 0, true, ApronBushCount);
                    SpawnNear(Cover, Cover.Num() > 0
                        ? RoundedCount(FMath::Max(0, GroundCoverPerCell) * CellArea * GroundFade, 8) : 0,
                        0.6f, 1.2f, EPlacementFit::Upright, 0.0f, 1, bGroundCoverCastsShadow, ApronCoverCount);
                    const float DebrisExpected = FMath::Lerp(0.28f, 0.12f, Density)
                        * FMath::Max(0, SurfaceDebrisPerCell) * CellArea * GroundFade;
                    SpawnNear(SurfaceDebris, SurfaceDebris.Num() > 0
                        ? RoundedCount(DebrisExpected * GroundFade, 4) : 0,
                        0.50f, 0.95f, EPlacementFit::SurfaceAlignedBounds,
                        SurfaceDebrisEmbedUU, 2, bSurfaceDebrisCastsShadow, ApronDebrisCount);
                    const float RockExpected = FMath::Lerp(0.12f, 0.04f, Density) * CellArea * GroundFade;
                    SpawnNear(SparseRockCover, SparseRockCover.Num() > 0
                        ? RoundedCount(RockExpected * GroundFade, 4) : 0,
                        0.45f, 0.85f, EPlacementFit::SurfaceAlignedBounds,
                        SparseRockEmbedUU, 3, true, ApronRockCount);
                }
            }
        }

        for (int32 PatchY = MinY; PatchY < MaxY; PatchY += PatchSide)
        {
            const int32 PatchEndY = FMath::Min(MaxY, PatchY + PatchSide);
            for (int32 PatchX = MinX; PatchX < MaxX; PatchX += PatchSide)
            {
                const int32 PatchEndX = FMath::Min(MaxX, PatchX + PatchSide);
                const bool bEntirelyDetailed = PatchX >= DetailMinX && PatchEndX <= DetailMaxX
                    && PatchY >= DetailMinY && PatchEndY <= DetailMaxY;
                if (bEntirelyDetailed)
                {
                    continue;
                }

                const uint32 PatchSeed = HashCombine(ApronSeed,
                    HashCombine(GetTypeHash(PatchX), GetTypeHash(PatchY)));
                FRandomStream PatchStream(static_cast<int32>(PatchSeed));
                float SampleX = PatchStream.FRandRange(static_cast<float>(PatchX), static_cast<float>(PatchEndX - 1));
                float SampleY = PatchStream.FRandRange(static_cast<float>(PatchY), static_cast<float>(PatchEndY - 1));
                // MakeTransform adds an intra-cell offset in [0.05, 0.95].
                // Bound the base coordinates so the resulting world locations
                // stay inside the terrain's exact fractional meter margin.
                SampleX = FMath::Clamp(SampleX, -OuterMarginCells,
                    W + OuterMarginCells - 0.95f);
                SampleY = FMath::Clamp(SampleY, -OuterMarginCells,
                    H + OuterMarginCells - 0.95f);
                // A boundary patch may overlap the detailed band. Project its
                // jitter to the nearest exterior edge rather than leaving a gap.
                if (SampleX >= DetailMinX && SampleX < DetailMaxX
                    && SampleY >= DetailMinY && SampleY < DetailMaxY)
                {
                    const float ToWest = SampleX - DetailMinX;
                    const float ToEast = DetailMaxX - SampleX;
                    const float ToSouth = SampleY - DetailMinY;
                    const float ToNorth = DetailMaxY - SampleY;
                    const float Nearest = FMath::Min(FMath::Min(ToWest, ToEast), FMath::Min(ToSouth, ToNorth));
                    if (Nearest == ToWest) SampleX = DetailMinX - 1.0f;
                    else if (Nearest == ToEast) SampleX = DetailMaxX;
                    else if (Nearest == ToSouth) SampleY = DetailMinY - 1.0f;
                    else SampleY = DetailMaxY;
                }
                // MakeTransform adds an intra-cell offset. Keep every final
                // apron instance outside the detailed band (and always outside
                // the authoritative grid even when margin cells are disabled).
                if (SampleX < DetailMinX && SampleX > DetailMinX - 1.0f)
                    SampleX = DetailMinX - 1.0f;
                if (SampleX >= DetailMaxX && SampleX < DetailMaxX + 1.0f)
                    SampleX = DetailMaxX;
                if (SampleY < DetailMinY && SampleY > DetailMinY - 1.0f)
                    SampleY = DetailMinY - 1.0f;
                if (SampleY >= DetailMaxY && SampleY < DetailMaxY + 1.0f)
                    SampleY = DetailMaxY;

                const FVector2D NoisePoint = (FVector2D(SampleX, SampleY) + NoiseOffset) * InvScale;
                const float Noise01 = 0.5f * (FMath::PerlinNoise2D(NoisePoint) + 1.0f);
                const float NoiseDensity = (Noise01 <= Threshold || Threshold >= 1.0f)
                    ? 0.0f
                    : FMath::Clamp((Noise01 - Threshold) / FMath::Max(0.0001f, 1.0f - Threshold), 0.0f, 1.0f);

                // Continue the nearest detailed boundary cell's local flora
                // style for a short distance, then blend into the same seeded
                // terrain-noise field used inside the simulation footprint.
                const float EdgeX = FMath::Clamp(SampleX, static_cast<float>(DetailMinX),
                                                  static_cast<float>(DetailMaxX - 1));
                const float EdgeY = FMath::Clamp(SampleY, static_cast<float>(DetailMinY),
                                                  static_cast<float>(DetailMaxY - 1));
                const FVector2D EdgeNoisePoint = (FVector2D(EdgeX, EdgeY) + NoiseOffset) * InvScale;
                const float EdgeNoise01 = 0.5f * (FMath::PerlinNoise2D(EdgeNoisePoint) + 1.0f);
                const float EdgeDensity = (EdgeNoise01 <= Threshold || Threshold >= 1.0f)
                    ? 0.0f
                    : FMath::Clamp((EdgeNoise01 - Threshold) / FMath::Max(0.0001f, 1.0f - Threshold), 0.0f, 1.0f);
                const float OutsideX = SampleX < DetailMinX ? DetailMinX - SampleX
                    : SampleX >= DetailMaxX ? SampleX - DetailMaxX : 0.0f;
                const float OutsideY = SampleY < DetailMinY ? DetailMinY - SampleY
                    : SampleY >= DetailMaxY ? SampleY - DetailMaxY : 0.0f;
                const float Distance = FMath::Max(OutsideX, OutsideY);
                if (Distance < NearBandCells)
                {
                    continue; // this area is covered by the fine, dense annulus above
                }
                ++ApronPatchesVisited;
                const float BlendT = FMath::Clamp(Distance / BlendDistanceCells, 0.0f, 1.0f);
                const float SmoothBlend = BlendT * BlendT * (3.0f - 2.0f * BlendT);
                const float Density = FMath::Lerp(EdgeDensity, NoiseDensity, SmoothBlend);
                const float ApronT = FMath::Clamp((Distance - NearBandCells)
                    / FMath::Max(1.0f, OuterMarginCells - NearBandCells), 0.0f, 1.0f);
                const float ApronBlend = ApronT * ApronT * (3.0f - 2.0f * ApronT);
                const float CanopyFalloff = FMath::Lerp(0.72f, 0.04f, ApronBlend);
                const float UnderstoryFalloff = FMath::Lerp(0.72f, 0.02f, ApronBlend);
                const float GroundFalloff = FMath::Lerp(0.82f, 0.015f, ApronBlend);
                const float DebrisFalloff = FMath::Lerp(0.82f, 0.03f, ApronBlend);
                const float RockFalloff = FMath::Lerp(0.82f, 0.03f, ApronBlend);
                const float PatchAreaCells = static_cast<float>((PatchEndX - PatchX) * (PatchEndY - PatchY));
                FRandomStream PatchFloraStream(static_cast<int32>(HashCombine(PatchSeed, 0x46D12983u)));

                auto CountCoarse = [&PatchFloraStream](float Expected, int32 MaxCount)
                {
                    const int32 Whole = FMath::FloorToInt(Expected);
                    return FMath::Clamp(Whole + (PatchFloraStream.FRand() < Expected - Whole ? 1 : 0),
                                        0, MaxCount);
                };
                auto SpawnCoarse = [&](TArray<UStaticMesh*>& Meshes, int32 Count,
                                       float ScaleMin, float ScaleMax, EPlacementFit Fit,
                                       float EmbedUU, uint8 ComponentRole, bool bCastShadow,
                                       int32& OutCount)
                {
                    for (int32 I = 0; I < Count; ++I)
                    {
                        UStaticMesh* Mesh = Meshes[PatchFloraStream.RandRange(0, Meshes.Num() - 1)];
                        const float Radius = PatchSide * 0.35f;
                        const float CenterX = FMath::Clamp(SampleX,
                            static_cast<float>(PatchX), static_cast<float>(PatchEndX - 1));
                        const float CenterY = FMath::Clamp(SampleY,
                            static_cast<float>(PatchY), static_cast<float>(PatchEndY - 1));
                        float PositionX = PatchFloraStream.FRandRange(
                            FMath::Max(static_cast<float>(PatchX), CenterX - Radius),
                            FMath::Min(static_cast<float>(PatchEndX - 1), CenterX + Radius));
                        float PositionY = PatchFloraStream.FRandRange(
                            FMath::Max(static_cast<float>(PatchY), CenterY - Radius),
                            FMath::Min(static_cast<float>(PatchEndY - 1), CenterY + Radius));
                        KeepOutsideDetail(PositionX, PositionY);
                        AddInstance(Mesh, MakeTransform(Mesh, PositionX, PositionY, PatchFloraStream,
                                                        ScaleMin, ScaleMax, Fit, EmbedUU),
                                    INDEX_NONE, false, ComponentRole, bCastShadow,
                                    FMath::FloorToInt(PositionX), FMath::FloorToInt(PositionY));
                        ++OutCount;
                        ApronMeshesUsed.Add(Mesh);
                    }
                };
                const float EdgeFactor = FMath::Clamp(1.0f - FMath::Abs(Density - 0.4f), 0.0f, 1.0f);
                const float DebrisChance = FMath::Lerp(0.28f, 0.12f, Density);
                const float RockChance = FMath::Lerp(0.12f, 0.04f, Density);
                SpawnCoarse(Trees, Trees.Num() > 0 ? CountCoarse(Density
                    * FMath::Max(0, MaxTreesPerCell) * PatchAreaCells * CanopyFalloff, 12) : 0,
                    TreeScaleMin, TreeScaleMax, EPlacementFit::Upright, 0.0f, 0, true, ApronCanopyCount);
                SpawnCoarse(Bushes, Bushes.Num() > 0 ? CountCoarse(EdgeFactor
                    * FMath::Max(0, MaxBushesPerCell) * PatchAreaCells * UnderstoryFalloff, 6) : 0,
                    0.5f, 1.0f, EPlacementFit::Upright, 0.0f, 0, true, ApronBushCount);
                SpawnCoarse(Cover, Cover.Num() > 0 ? CountCoarse(FMath::Max(0, GroundCoverPerCell)
                    * PatchAreaCells * GroundFalloff, 6) : 0,
                    0.6f, 1.2f, EPlacementFit::Upright, 0.0f, 1, bGroundCoverCastsShadow, ApronCoverCount);
                SpawnCoarse(SurfaceDebris, SurfaceDebris.Num() > 0 ? CountCoarse(DebrisChance
                    * FMath::Max(0, SurfaceDebrisPerCell) * PatchAreaCells * DebrisFalloff, 3) : 0,
                    0.50f, 0.95f, EPlacementFit::SurfaceAlignedBounds,
                    SurfaceDebrisEmbedUU, 2, bSurfaceDebrisCastsShadow, ApronDebrisCount);
                SpawnCoarse(SparseRockCover, SparseRockCover.Num() > 0 ? CountCoarse(RockChance
                    * PatchAreaCells * RockFalloff, 3) : 0,
                    0.45f, 0.85f, EPlacementFit::SurfaceAlignedBounds,
                    SparseRockEmbedUU, 3, true, ApronRockCount);
            }
        }
    }

    if (ApronPatchesVisited > 0)
    {
        UE_LOG(LogTemp, Log,
            TEXT("Thesis forest apron: %d/%d coarse patches; %d canopy, %d understory, %d cover, %d debris, %d rocks; %d distinct meshes."),
            ApronPatchesVisited, FMath::Max(0, PresentationApronMaxPatches), ApronCanopyCount,
            ApronBushCount, ApronCoverCount, ApronDebrisCount, ApronRockCount, ApronMeshesUsed.Num());
    }
    }

    bCaptureGeneratedLayout = false;
    const int32 GeneratedLayoutCount = GeneratedLayout.Num();
    if (!bLayoutCacheHit && bSpawnVisualInstances && !bReuseDisabled)
    {
        FForestLayoutCacheEntry NewLayout;
        NewLayout.Key = LayoutKey;
        NewLayout.MeshPaths.Reserve(LayoutMeshAssets.Num());
        for (UStaticMesh* M : LayoutMeshAssets) NewLayout.MeshPaths.Add(M->GetPathName());
        NewLayout.Items = MoveTemp(GeneratedLayout);
        if (NewLayout.Bytes() <= 96ull * 1024ull * 1024ull)
            GForestLayoutCache = MoveTemp(NewLayout);
    }

    for (int32 i = 0; i < Batches.Num(); ++i)
    {
        if (FloraComponents.IsValidIndex(i) && FloraComponents[i] && Batches[i].Num() > 0)
        {
            FloraComponents[i]->AddInstances(Batches[i], /*bShouldReturnIndices=*/false, /*bWorldSpace=*/true);
            // AddInstances queues an async cluster-tree build by default. Finish a
            // synchronous build now so live views never see a partial HISM tree.
            FloraComponents[i]->BuildTreeIfOutdated(/*Async=*/false, /*ForceUpdate=*/true);
            NumInstancesSpawned += Batches[i].Num();
        }
    }

    UE_LOG(LogTemp, Log,
           TEXT("Thesis forest: %d flora instances across %d HISM components over %dx%d cells (seed %d, coverage %.2f, captures-only=%d, layout-cache=%s, cached-items=%d)."),
           NumInstancesSpawned, FloraComponents.Num(), W, H, Seed, ForestCoverage,
           bVisibleOnlyToDroneCaptures ? 1 : 0,
           bLayoutCacheHit ? TEXT("hit") : TEXT("miss"),
           bLayoutCacheHit ? GForestLayoutCache.Items.Num() : GeneratedLayoutCount);
}

void AForestDresser::ScorchCell(int32 CellX, int32 CellY)
{
    if (BuiltGridW <= 0)
    {
        return;
    }
    const int32 CellIdx = CellY * BuiltGridW + CellX;
    if (ScorchedCells.Contains(CellIdx))
    {
        return;
    }
    ScorchedCells.Add(CellIdx);

    if (!bWriteScorchToPerInstanceCustomData)
    {
        return;
    }

    const TArray<FCellInstanceRef>* Refs = CellToVegetationInstances.Find(CellIdx);
    if (!Refs)
    {
        return;
    }

    TSet<int32> DirtyComponents;
    for (const FCellInstanceRef& Ref : *Refs)
    {
        if (!FloraComponents.IsValidIndex(Ref.ComponentIndex) || !FloraComponents[Ref.ComponentIndex])
        {
            continue;
        }
        FloraComponents[Ref.ComponentIndex]->SetCustomDataValue(
            Ref.InstanceIndex, ScorchCustomDataIndex, 1.0f, /*bMarkRenderStateDirty=*/false);
        ScorchedInstances.Add(Ref);
        DirtyComponents.Add(Ref.ComponentIndex);
    }
    for (const int32 ComponentIndex : DirtyComponents)
    {
        FloraComponents[ComponentIndex]->MarkRenderStateDirty();
    }
}

void AForestDresser::ResetScorch()
{
    TSet<int32> DirtyComponents;
    for (const FCellInstanceRef& R : ScorchedInstances)
    {
        if (!FloraComponents.IsValidIndex(R.ComponentIndex) || !FloraComponents[R.ComponentIndex])
        {
            continue;
        }
        FloraComponents[R.ComponentIndex]->SetCustomDataValue(
            R.InstanceIndex, ScorchCustomDataIndex, 0.0f, /*bMarkRenderStateDirty=*/false);
        DirtyComponents.Add(R.ComponentIndex);
    }
    for (const int32 ComponentIndex : DirtyComponents)
    {
        FloraComponents[ComponentIndex]->MarkRenderStateDirty();
    }
    ScorchedInstances.Reset();
    ScorchedCells.Reset();
}
