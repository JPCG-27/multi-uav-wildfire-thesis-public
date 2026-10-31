#include "ProceduralTerrain.h"
#include "ProceduralMeshComponent.h"
#include "Materials/MaterialInterface.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/Texture2D.h"
#include "Math/RandomStream.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "UObject/ConstructorHelpers.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

namespace
{
    struct FTerrainGeometryCacheEntry
    {
        FString Key;
        TArray<FVector> Vertices;
        TArray<FVector> Normals;
        TArray<FVector2D> UVs;
        TArray<FLinearColor> Colors;
        uint64 Bytes() const
        {
            return static_cast<uint64>(Vertices.Num()) * sizeof(FVector)
                + static_cast<uint64>(Normals.Num()) * sizeof(FVector)
                + static_cast<uint64>(UVs.Num()) * sizeof(FVector2D)
                + static_cast<uint64>(Colors.Num()) * sizeof(FLinearColor);
        }
    };
    FTerrainGeometryCacheEntry GTerrainGeometryCache;

    uint32 TerrainFloatBits(float Value)
    {
        uint32 Bits = 0;
        FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
        return Bits;
    }
    uint64 TerrainDoubleBits(double Value)
    {
        uint64 Bits = 0;
        FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
        return Bits;
    }

    bool IsSceneReuseDisabled()
    {
        return FParse::Param(FCommandLine::Get(), TEXT("ThesisDisableSceneReuse"));
    }

    struct FProfileParams
    {
        float AmplitudeFractionOfExtent;   // relief amplitude as fraction of min(W,H)*CellSize
        float BaseFreqPerCell;             // base noise frequency in 1/cells
        bool bRidged;
        float WarpStrengthCells;
        float RidgeStrength;
    };

    FProfileParams GetProfileParams(ETerrainProfile Profile)
    {
        switch (Profile)
        {
            case ETerrainProfile::Flat:         return { 0.00f,  0.020f, false, 0.0f, 0.00f };
            case ETerrainProfile::RollingHills: return { 0.045f, 0.022f, false, 2.5f, 0.10f };
            case ETerrainProfile::Foothills:    return { 0.120f, 0.026f, false, 5.0f, 0.24f };
            case ETerrainProfile::Mountains:    return { 0.220f, 0.032f, true,  8.0f, 0.48f };
            default:                            return { 0.120f, 0.026f, false, 5.0f, 0.24f };
        }
    }
}

AProceduralTerrain::AProceduralTerrain()
{
    PrimaryActorTick.bCanEverTick = false;

    Mesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("TerrainMesh"));
    SetRootComponent(Mesh);
    Mesh->SetMobility(EComponentMobility::Movable);
    Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Mesh->SetCanEverAffectNavigation(false);
    Mesh->bUseAsyncCooking = false;   // determinism: no async physics cook

    // Prefer the project's textured terrain material (M_ThesisTerrain, built by
    // Tools/create_terrain_material.py). Fall back to the engine vertex-color
    // debug material if it has not been created yet, so a fresh checkout still
    // shows plausible shading with no assets.
    static ConstructorHelpers::FObjectFinder<UMaterialInterface> ThesisTerrainMat(
        TEXT("/Game/Thesis/Materials/M_ThesisTerrain.M_ThesisTerrain"));
    if (ThesisTerrainMat.Succeeded())
    {
        TerrainMaterial = ThesisTerrainMat.Object;
    }
    else
    {
        static ConstructorHelpers::FObjectFinder<UMaterialInterface> VertexColorMat(
            TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial"));
        if (VertexColorMat.Succeeded())
        {
            TerrainMaterial = VertexColorMat.Object;
        }
    }
}

void AProceduralTerrain::SetVisualsCaptureOnly(bool bCaptureOnly)
{
    bVisibleOnlyToDroneCaptures = bCaptureOnly;
    SetVisualsEnabled(bRenderVisuals);
}

void AProceduralTerrain::SetVisualsEnabled(bool bEnabled)
{
    bRenderVisuals = bEnabled;
    if (Mesh)
    {
        Mesh->SetVisibility(bEnabled, true);
        Mesh->SetHiddenInGame(!bEnabled);
        Mesh->SetVisibleInSceneCaptureOnly(bEnabled && bVisibleOnlyToDroneCaptures);
    }
    for (UProceduralMeshComponent* Tile : AdditionalRenderTiles)
    {
        Tile->SetVisibility(bEnabled);
        Tile->SetHiddenInGame(!bEnabled);
        Tile->SetVisibleInSceneCaptureOnly(bEnabled && bVisibleOnlyToDroneCaptures);
    }
}

void AProceduralTerrain::ClearRenderTiles()
{
    for (UProceduralMeshComponent* Tile : AdditionalRenderTiles)
        if (Tile) Tile->DestroyComponent();
    AdditionalRenderTiles.Reset();
    Mesh->ClearAllMeshSections();
}

float AProceduralTerrain::SampleNoise(float CellX, float CellY) const
{
    const FVector2D P(CellX, CellY);
    FVector2D WarpedP = P;
    if (CachedWarpStrengthCells > 0.0f && CachedBaseFreq > 0.0f)
    {
        const float WarpFreq = CachedBaseFreq * 0.45f;
        const float Wx = FMath::PerlinNoise2D((P + CachedNoiseOffset) * WarpFreq);
        const float Wy = FMath::PerlinNoise2D((P + CachedWarpOffset) * WarpFreq);
        WarpedP += FVector2D(Wx, Wy) * CachedWarpStrengthCells;
    }

    float Amp = 1.0f;
    float Freq = CachedBaseFreq;
    float Sum = 0.0f;
    float Norm = 0.0f;

    for (int32 Octave = 0; Octave < 6; ++Octave)
    {
        float N = FMath::PerlinNoise2D(
            (WarpedP + CachedNoiseOffset * (Octave + 1)) * Freq);
        Sum += N * Amp;
        Norm += Amp;
        Amp *= 0.52f;
        Freq *= 1.95f;
    }

    float FBm = (Norm > 0.0f) ? (Sum / Norm) : 0.0f;

    const float RidgeRaw = FMath::PerlinNoise2D((WarpedP + CachedRidgeOffset) * CachedBaseFreq * 1.65f);
    const float RidgeSigned = (1.0f - FMath::Abs(RidgeRaw)) * 2.0f - 1.0f;
    const float RidgeBlend = bCachedRidged
        ? FMath::Max(CachedRidgeStrength, 0.42f)
        : CachedRidgeStrength;
    FBm = FMath::Lerp(FBm, RidgeSigned, FMath::Clamp(RidgeBlend, 0.0f, 0.85f));

    // A very broad low-frequency component gives the mesh recognisable valleys
    // and shoulders instead of evenly noisy waves.
    const float Landform = FMath::PerlinNoise2D((P + CachedLandformOffset) * CachedBaseFreq * 0.32f);
    FBm = FMath::Lerp(FBm, Landform, 0.28f);

    return FMath::Clamp(FBm, -1.0f, 1.0f);
}

FString AProceduralTerrain::GetHeightFieldSignature() const
{
    FString S;
    auto AddFloat = [&S](float V) { S.Appendf(TEXT("f%08x;"), TerrainFloatBits(V)); };
    auto AddDouble = [&S](double V) { S.Appendf(TEXT("d%016llx;"), TerrainDoubleBits(V)); };
    S.Appendf(TEXT("%d:%d;"), CachedGridW, CachedGridH);
    AddDouble(CachedGridOrigin.X); AddDouble(CachedGridOrigin.Y); AddDouble(CachedGridOrigin.Z);
    AddFloat(CachedCellSizeUU); AddFloat(BuiltAmplitudeUU); AddFloat(CachedBaseFreq);
    AddDouble(CachedNoiseOffset.X); AddDouble(CachedNoiseOffset.Y);
    AddDouble(CachedWarpOffset.X); AddDouble(CachedWarpOffset.Y);
    AddDouble(CachedRidgeOffset.X); AddDouble(CachedRidgeOffset.Y);
    AddDouble(CachedLandformOffset.X); AddDouble(CachedLandformOffset.Y);
    AddFloat(CachedWarpStrengthCells); AddFloat(CachedRidgeStrength);
    S.Appendf(TEXT("b%d:r%d:m%d"), bBuilt ? 1 : 0, bCachedRidged ? 1 : 0,
        CachedPresentationMarginCells);
    return S;
}

float AProceduralTerrain::GetHeightAtCell(float CellX, float CellY) const
{
    if (!bBuilt || BuiltAmplitudeUU <= 0.0f)
    {
        return CachedGridOrigin.Z;
    }
    return CachedGridOrigin.Z + SampleNoise(CellX, CellY) * BuiltAmplitudeUU;
}

float AProceduralTerrain::PresentationVertexCellCoordinate(int32 VertexIndex,
                                                            int32 VerticesPerCell,
                                                            int32 MarginCells)
{
    const int32 Res = FMath::Max(1, VerticesPerCell);
    const int32 MarginVertices = FMath::Max(0, MarginCells) * Res;
    // Subtract in integer vertex space before converting. This keeps the
    // in-grid samples bit-identical to the original IX / Res coordinates.
    const float StepCells = 1.0f / static_cast<float>(Res);
    return static_cast<float>(VertexIndex - MarginVertices) * StepCells;
}

FVector AProceduralTerrain::GetNormalAtCell(float CellX, float CellY) const
{
    if (!bBuilt || BuiltAmplitudeUU <= 0.0f || CachedCellSizeUU <= 0.0f)
    {
        return FVector::UpVector;
    }

    const float HxP = GetHeightAtCell(CellX + 0.5f, CellY);
    const float HxM = GetHeightAtCell(CellX - 0.5f, CellY);
    const float HyP = GetHeightAtCell(CellX, CellY + 0.5f);
    const float HyM = GetHeightAtCell(CellX, CellY - 0.5f);
    const float DhDx = (HxP - HxM) / CachedCellSizeUU;
    const float DhDy = (HyP - HyM) / CachedCellSizeUU;
    return FVector(-DhDx, -DhDy, 1.0f).GetSafeNormal(UE_SMALL_NUMBER, FVector::UpVector);
}

float AProceduralTerrain::SlopeAtCell(int32 CellX, int32 CellY) const
{
    if (!bBuilt || BuiltAmplitudeUU <= 0.0f || CachedCellSizeUU <= 0.0f)
    {
        return 0.0f;
    }
    const float HxP = GetHeightAtCell(CellX + 1.0f, CellY);
    const float HxM = GetHeightAtCell(CellX - 1.0f, CellY);
    const float HyP = GetHeightAtCell(CellX, CellY + 1.0f);
    const float HyM = GetHeightAtCell(CellX, CellY - 1.0f);
    const float DhDx = (HxP - HxM) / (2.0f * CachedCellSizeUU);
    const float DhDy = (HyP - HyM) / (2.0f * CachedCellSizeUU);
    return FMath::Sqrt(DhDx * DhDx + DhDy * DhDy);
}

void AProceduralTerrain::BuildTerrain(int32 GridW, int32 GridH, const FVector& GridOriginWorld,
                                      float CellSizeUU, int32 SimSeed)
{
    if (GridW <= 0 || GridH <= 0 || CellSizeUU <= 0.0f || !Mesh)
    {
        return;
    }

    const int32 Seed = (SeedOverride != 0) ? SeedOverride : SimSeed;
    const FProfileParams P = GetProfileParams(Profile);

    CachedGridW = GridW;
    CachedGridH = GridH;
    CachedGridOrigin = GridOriginWorld;
    CachedCellSizeUU = CellSizeUU;
    CachedBaseFreq = P.BaseFreqPerCell * FMath::Clamp(DetailDensity, 0.25f, 4.0f);
    bCachedRidged = P.bRidged;
    CachedWarpStrengthCells = P.WarpStrengthCells;
    CachedRidgeStrength = P.RidgeStrength;

    FRandomStream SeedStream(Seed * 7919 + 17);
    CachedNoiseOffset = FVector2D(
        SeedStream.FRandRange(-512.0f, 512.0f),
        SeedStream.FRandRange(-512.0f, 512.0f));
    CachedWarpOffset = FVector2D(
        SeedStream.FRandRange(-512.0f, 512.0f),
        SeedStream.FRandRange(-512.0f, 512.0f));
    CachedRidgeOffset = FVector2D(
        SeedStream.FRandRange(-512.0f, 512.0f),
        SeedStream.FRandRange(-512.0f, 512.0f));
    CachedLandformOffset = FVector2D(
        SeedStream.FRandRange(-512.0f, 512.0f),
        SeedStream.FRandRange(-512.0f, 512.0f));

    const float ExtentUU = FMath::Min(GridW, GridH) * CellSizeUU;
    BuiltAmplitudeUU = ExtentUU * P.AmplitudeFractionOfExtent * FMath::Max(0.0f, SlopeIntensity);
    bBuilt = true;

    SetActorLocation(GridOriginWorld);

    // --- Headless fast path -------------------------------------------------
    // Heights are analytic: GetHeightAtCell/GetNormalAtCell/SlopeAtCell and the
    // movement-cost CSV all read SampleNoise() via the cached params set above,
    // never the render mesh. The vertex build, CreateMeshSection, dynamic
    // material and burn-mask texture below are PURELY visual. In a headless /
    // no-capture run (paper-mode) nothing renders the terrain, so skip the
    // O(grid) vertex build entirely. Determinism is unaffected -- the simulation
    // never samples the mesh. (YOLO/visible runs set bRenderVisuals or
    // bVisibleOnlyToDroneCaptures and fall through to the full build below.)
    const bool bNeedsVisualMesh = bRenderVisuals || bVisibleOnlyToDroneCaptures;
    if (!bNeedsVisualMesh)
    {
        ClearRenderTiles();
        SetVisualsEnabled(false);
        InitBurnMask();
        UE_LOG(LogTemp, Log,
               TEXT("Thesis terrain: built %dx%d analytic-only (visual mesh skipped, headless), profile %d, amplitude %.0f UU, seed %d."),
               GridW, GridH, static_cast<int32>(Profile), BuiltAmplitudeUU, Seed);
        return;
    }

    // ---------------- Mesh ----------------
    const int32 Res = FMath::Clamp(VerticesPerCell, 1, 4);
    // Extend the rendered domain only. Cell-space zero remains the grid origin,
    // and every original in-grid vertex samples the unchanged pure height field.
    // Keep at least one unburned mask texel outside the grid when distant ground
    // is enabled, so clamped sampling cannot spread edge-cell scorch outward.
    const int32 MarginCells = FMath::Clamp(PresentationMarginCells,
        PresentationMarginMeters > 0.0f ? 1 : 0, 64);
    CachedPresentationMarginCells = MarginCells;
    const float OuterMarginCells = FMath::Max(static_cast<float>(MarginCells),
        FMath::Clamp(PresentationMarginMeters, 0.0f, 10000.0f) * 100.0f / CellSizeUU);
    const int32 OuterBands = OuterMarginCells > MarginCells ? 32 : 0;
    const int32 BaseNX = (GridW + MarginCells * 2) * Res + 1;
    const int32 BaseNY = (GridH + MarginCells * 2) * Res + 1;
    const int32 NX = BaseNX + OuterBands * 2;
    const int32 NY = BaseNY + OuterBands * 2;
    const float StepUU = CellSizeUU / Res;
    auto AxisCoordinate = [&](int32 Index, int32 BaseCount, int32 GridCells)
    {
        const int32 InnerIndex = Index - OuterBands;
        if (InnerIndex < 0)
            return FMath::Lerp(-OuterMarginCells, -static_cast<float>(MarginCells),
                static_cast<float>(Index) / OuterBands);
        if (InnerIndex >= BaseCount)
            return FMath::Lerp(static_cast<float>(GridCells + MarginCells), GridCells + OuterMarginCells,
                static_cast<float>(InnerIndex - BaseCount + 1) / OuterBands);
        return PresentationVertexCellCoordinate(InnerIndex, Res, MarginCells);
    };
    auto AxisPosition = [&](int32 Index, int32 BaseCount, float CellCoordinate)
    {
        const int32 InnerIndex = Index - OuterBands;
        // Preserve the original arithmetic for bit-identical interior vertices.
        return InnerIndex >= 0 && InnerIndex < BaseCount
            ? (InnerIndex - MarginCells * Res) * StepUU : CellCoordinate * CellSizeUU;
    };

    TArray<FVector> Vertices;
    TArray<FVector> Normals;
    TArray<FVector2D> UVs;
    TArray<FLinearColor> Colors;
    const FString GeometryKey = FString::Printf(
        TEXT("%d:%d:%d:%d:%d:%d:%08x:%08x:%08x:%08x:%08x"),
        GridW, GridH, static_cast<int32>(Profile), Seed, Res, MarginCells,
        TerrainFloatBits(CellSizeUU), TerrainFloatBits(SlopeIntensity), TerrainFloatBits(DetailDensity),
        TerrainFloatBits(PresentationMarginMeters), TerrainFloatBits(StepUU));
    FString CompleteGeometryKey = GeometryKey;
    CompleteGeometryKey.Appendf(TEXT(":%016llx:%016llx:%016llx"),
        TerrainDoubleBits(GridOriginWorld.X), TerrainDoubleBits(GridOriginWorld.Y), TerrainDoubleBits(GridOriginWorld.Z));
    const bool bGeometryCacheHit = !IsSceneReuseDisabled()
        && GTerrainGeometryCache.Key == CompleteGeometryKey
        && GTerrainGeometryCache.Vertices.Num() == NX * NY;
    if (bGeometryCacheHit)
    {
        Vertices = GTerrainGeometryCache.Vertices;
        Normals = GTerrainGeometryCache.Normals;
        UVs = GTerrainGeometryCache.UVs;
        Colors = GTerrainGeometryCache.Colors;
    }
    else
    {
        Vertices.Reserve(NX * NY);
        Normals.Reserve(NX * NY);
        UVs.Reserve(NX * NY);
        Colors.Reserve(NX * NY);


    // GerÃªs / Peneda-GerÃªs palette (no snow â€” wildfires don't happen on
    // snowy peaks, and white tops confuse the vision model).
    //   DenseGreen : heather + fern valley floors â€” very dark, saturated Atlantic green
    //   ForestGreen: mixed pine / oak / scrub mid-slopes â€” still green but lighter
    //   Granite    : exposed GerÃªs granite ridges â€” pale pinkish-grey
    const FLinearColor Humus(0.035f, 0.026f, 0.018f);
    const FLinearColor LeafLitter(0.115f, 0.075f, 0.034f);
    const FLinearColor Moss(0.020f, 0.050f, 0.014f);
    const FLinearColor Granite(0.190f, 0.185f, 0.165f);

    for (int32 IY = 0; IY < NY; ++IY)        // fixed raster order â€” deterministic
    {
        for (int32 IX = 0; IX < NX; ++IX)
        {
            const float Cx = AxisCoordinate(IX, BaseNX, GridW);
            const float Cy = AxisCoordinate(IY, BaseNY, GridH);
            const float HWorld = GetHeightAtCell(Cx, Cy);
            const float HLocal = HWorld - GridOriginWorld.Z;

            Vertices.Add(FVector(AxisPosition(IX, BaseNX, Cx),
                                 AxisPosition(IY, BaseNY, Cy), HLocal));
            UVs.Add(FVector2D(Cx * 0.25f, Cy * 0.25f));

            // Analytic-ish normal via central differences (in cells).
            const float DhDx = (GetHeightAtCell(Cx + 0.5f, Cy) - GetHeightAtCell(Cx - 0.5f, Cy)) / CellSizeUU;
            const float DhDy = (GetHeightAtCell(Cx, Cy + 0.5f) - GetHeightAtCell(Cx, Cy - 0.5f)) / CellSizeUU;
            Normals.Add(FVector(-DhDx, -DhDy, 1.0f).GetSafeNormal());

            // Procedural shading: lush green valleys fade to granite on steep slopes.
            // GerÃªs: height drives DenseGreenâ†’ForestGreen (wetter low ground vs
            // more open mid-slope), slope drives everythingâ†’Granite (the park's
            // defining visual â€” pale rock ridges above the treeline).
            // With M_ThesisTerrain assigned these vertex colours are overridden;
            // retained so the fallback VertexColorMaterial still looks plausible.
            const float Slope = FMath::Sqrt(DhDx * DhDx + DhDy * DhDy);
            const float Patch = 0.5f + 0.5f * FMath::PerlinNoise2D(
                (FVector2D(Cx, Cy) + CachedLandformOffset) * CachedBaseFreq * 2.4f);
            // Low ground: dense heather/fern green; higher: lighter pine/scrub green.
            FLinearColor C = FMath::Lerp(Humus, LeafLitter, FMath::SmoothStep(0.18f, 0.78f, Patch));
            // Steep slopes: expose pale GerÃªs granite.
            C = FMath::Lerp(
                C, Moss,
                FMath::SmoothStep(0.82f, 0.97f, Patch) *
                (1.0f - FMath::SmoothStep(0.12f, 0.36f, Slope)));
            C = FMath::Lerp(C, Granite, FMath::SmoothStep(0.18f, 0.50f, Slope));
            Colors.Add(C);
        }
    }

        // Hold at most one modest plain-data geometry entry. Large presentation
        // terrains are deliberately not retained to keep persistent benchmark
        // reloads from accumulating hundreds of MB.
        const uint64 GeometryBytes = static_cast<uint64>(Vertices.Num()) * sizeof(FVector) * 2
            + static_cast<uint64>(UVs.Num()) * sizeof(FVector2D)
            + static_cast<uint64>(Colors.Num()) * sizeof(FLinearColor);
        if (!IsSceneReuseDisabled() && GeometryBytes <= 96ull * 1024ull * 1024ull)
        {
            FTerrainGeometryCacheEntry NewEntry;
            NewEntry.Key = CompleteGeometryKey;
            NewEntry.Vertices = Vertices;
            NewEntry.Normals = Normals;
            NewEntry.UVs = UVs;
            NewEntry.Colors = Colors;
            GTerrainGeometryCache = MoveTemp(NewEntry);
        }
    }

    ClearRenderTiles();
    const int32 QuadLimit = RenderTileQuads > 0 ? FMath::Clamp(RenderTileQuads, 1, 128) : FMath::Max(NX, NY);
    // The distant apron has coarse quads: cap physical span as well as count,
    // otherwise its outer tiles would still have kilometre-wide bounds.
    const float SpanLimitUU = 12800.0f;
    auto TileEnd = [&](int32 Start, int32 Count, bool bXAxis)
    {
        int32 End = FMath::Min(Start + QuadLimit, Count - 1);
        if (RenderTileQuads > 0)
        {
            auto Position = [&](int32 I) { return bXAxis ? Vertices[I].X : Vertices[I * NX].Y; };
            while (End > Start + 1 && Position(End) - Position(Start) > SpanLimitUU) --End;
        }
        return End;
    };
    int32 TileOrdinal = 0;
    for (int32 StartY = 0; StartY < NY - 1;)
    {
        const int32 EndY = TileEnd(StartY, NY, false);
        for (int32 StartX = 0; StartX < NX - 1;)
        {
            const int32 EndX = TileEnd(StartX, NX, true);
            UProceduralMeshComponent* Tile = Mesh;
            if (TileOrdinal++ > 0)
            {
                Tile = NewObject<UProceduralMeshComponent>(this);
                Tile->SetupAttachment(Mesh);
                Tile->SetMobility(EComponentMobility::Movable);
                Tile->SetCollisionEnabled(ECollisionEnabled::NoCollision);
                Tile->SetCanEverAffectNavigation(false);
                Tile->bUseAsyncCooking = false;
                Tile->RegisterComponent();
                AdditionalRenderTiles.Add(Tile);
            }
            const int32 TileNX = EndX - StartX + 1;
            TArray<FVector> TileVertices, TileNormals;
            TArray<FVector2D> TileUVs;
            TArray<FLinearColor> TileColors;
            TArray<int32> TileTriangles;
            for (int32 Y = StartY; Y <= EndY; ++Y)
                for (int32 X = StartX; X <= EndX; ++X)
                {
                    const int32 I = Y * NX + X;
                    // Copy the global samples exactly: shared edges have the same
                    // positions, normals, material UVs and colors on both tiles.
                    TileVertices.Add(Vertices[I]);
                    TileNormals.Add(Normals[I]);
                    TileUVs.Add(UVs[I]);
                    TileColors.Add(Colors[I]);
                }
            for (int32 Y = 0; Y < EndY - StartY; ++Y)
                for (int32 X = 0; X < EndX - StartX; ++X)
                {
                    const int32 I00 = Y * TileNX + X;
                    const int32 I10 = I00 + 1, I01 = I00 + TileNX, I11 = I01 + 1;
                    TileTriangles.Append({ I00, I01, I10, I10, I01, I11 });
                }
            Tile->CreateMeshSection_LinearColor(0, TileVertices, TileTriangles,
                TileNormals, TileUVs, TileColors, TArray<FProcMeshTangent>(), false);
            StartX = EndX;
        }
        StartY = EndY;
    }

    // ---------- Material / MID ----------
    if (!TerrainMaterial)
    {
        if (UMaterialInterface* ProjectTerrainMat = LoadObject<UMaterialInterface>(
            nullptr, TEXT("/Game/Thesis/Materials/M_ThesisTerrain.M_ThesisTerrain")))
        {
            TerrainMaterial = ProjectTerrainMat;
        }
        else if (UMaterialInterface* VertexColorMat = LoadObject<UMaterialInterface>(
            nullptr, TEXT("/Engine/EngineDebugMaterials/VertexColorMaterial.VertexColorMaterial")))
        {
            TerrainMaterial = VertexColorMat;
        }
    }

    if (TerrainMaterial)
    {
        // CreateDynamicMaterialInstance sets the material on slot 0 and returns the MID.
        TerrainMID = Mesh->CreateDynamicMaterialInstance(0, TerrainMaterial);
        if (TerrainMID)
        {
            // Per-build scalars/vectors consumed by M_ThesisTerrain's graph
            // (slope mask, height mask, burn UV).  Must match the param names
            // declared in the material.
            // GetHeightAtCell spans origin +/- amplitude. Feed the material
            // the true visual range so height bands do not clamp half the hills.
            TerrainMID->SetScalarParameterValue(TEXT("P_TerrainMinZ"),
                CachedGridOrigin.Z - BuiltAmplitudeUU);
            TerrainMID->SetScalarParameterValue(TEXT("P_TerrainAmplitude"),
                FMath::Max(1.0f, BuiltAmplitudeUU * 2.0f));
            TerrainMID->SetVectorParameterValue(TEXT("P_GridOriginXY"),
                FLinearColor(CachedGridOrigin.X - MarginCells * CellSizeUU,
                             CachedGridOrigin.Y - MarginCells * CellSizeUU, 0.f, 0.f));
            TerrainMID->SetVectorParameterValue(TEXT("P_GridWorldSizeXY"),
                FLinearColor((GridW + MarginCells * 2) * CellSizeUU,
                             (GridH + MarginCells * 2) * CellSizeUU, 0.f, 0.f));
            TerrainMID->SetScalarParameterValue(TEXT("P_BurnMaskEnabled"), 0.0f);
            // T_BurnMask is wired in InitBurnMask() below, after the texture exists.
        }
        else
        {
            Mesh->SetMaterial(0, TerrainMaterial);
            UE_LOG(LogTemp, Warning,
                   TEXT("Thesis terrain: could not create MID for %s; using the base material without runtime parameters."),
                   *TerrainMaterial->GetPathName());
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis terrain: no TerrainMaterial available; procedural mesh will use Unreal's default material."));
        TerrainMID = nullptr;
    }

    for (UProceduralMeshComponent* Tile : AdditionalRenderTiles)
        Tile->SetMaterial(0, TerrainMID ? static_cast<UMaterialInterface*>(TerrainMID) : TerrainMaterial.Get());
    SetVisualsEnabled(bRenderVisuals);

    // ---------- Burn mask ----------
    InitBurnMask();

    UE_LOG(LogTemp, Log,
           TEXT("Thesis terrain: built %dx%d (res %d/cell, %d verts, %d render tiles), profile %d, amplitude %.0f UU, seed %d, geometry-cache=%s."),
           GridW, GridH, Res, Vertices.Num(), TileOrdinal, static_cast<int32>(Profile), BuiltAmplitudeUU, Seed,
           bGeometryCacheHit ? TEXT("hit") : TEXT("miss"));
}

void AProceduralTerrain::InitBurnMask()
{
    const int32 Margin = FMath::Max(0, CachedPresentationMarginCells);
    const int32 MaskW = CachedGridW + Margin * 2;
    const int32 MaskH = CachedGridH + Margin * 2;
    const int32 Total = MaskW * MaskH;
    if (Total <= 0)
    {
        return;
    }

    // Zero-initialise the CPU mirror.
    BurnMaskCPU.Init(0, Total);
    bBurnDirty = false;

    // Create a transient PF_G8 texture: one byte per cell, no sRGB gamma,
    // nearest filtering so cell boundaries stay crisp.
    BurnMaskTexture = UTexture2D::CreateTransient(MaskW, MaskH, PF_G8, TEXT("ThesisBurnMask"));
    if (!BurnMaskTexture)
    {
        UE_LOG(LogTemp, Warning, TEXT("Thesis terrain: failed to create BurnMaskTexture."));
        return;
    }
    // This texture is presentation-only. Bilinear sampling softens cell-square
    // scorch borders without altering the discrete authoritative fire grid.
    BurnMaskTexture->Filter = TF_Bilinear;
    BurnMaskTexture->SRGB   = 0;
    BurnMaskTexture->AddressX = TA_Clamp;
    BurnMaskTexture->AddressY = TA_Clamp;
    BurnMaskTexture->UpdateResource();

    // Upload zeroed data immediately so the texture is valid before any tick.
    uint8* DataCopy = new uint8[Total]();   // zero-initialised
    FUpdateTextureRegion2D* Region = new FUpdateTextureRegion2D(0, 0, 0, 0, MaskW, MaskH);
    BurnMaskTexture->UpdateTextureRegions(
        /*MipIndex=*/0, /*NumRegions=*/1, Region,
        /*SrcPitch=*/MaskW, /*SrcBpp=*/1, DataCopy,
        /*DataCleanupFunc=*/[](uint8* InData, const FUpdateTextureRegion2D* InRegion)
        {
            delete[] InData;
            delete InRegion;
        });

    // Bind to the MID so the material can sample it.
    if (TerrainMID)
    {
        TerrainMID->SetTextureParameterValue(TEXT("T_BurnMask"), BurnMaskTexture);
        TerrainMID->SetScalarParameterValue(TEXT("P_BurnMaskEnabled"), 1.0f);
    }

    UE_LOG(LogTemp, Log,
           TEXT("Thesis terrain: BurnMaskTexture %dx%d created including a zeroed %d-cell presentation apron (%.1f KB)."),
           MaskW, MaskH, Margin, Total / 1024.0f);
}

void AProceduralTerrain::ScorchCell(int32 CellX, int32 CellY)
{
    if (!bBuilt || BurnMaskCPU.Num() == 0 ||
        CellX < 0 || CellX >= CachedGridW || CellY < 0 || CellY >= CachedGridH)
    {
        return;
    }
    const int32 MaskW = CachedGridW + CachedPresentationMarginCells * 2;
    const int32 I = (CellY + CachedPresentationMarginCells) * MaskW +
                    (CellX + CachedPresentationMarginCells);
    if (!BurnMaskCPU.IsValidIndex(I) || BurnMaskCPU[I] == 255)
    {
        return;   // idempotent
    }
    BurnMaskCPU[I] = 255;
    bBurnDirty = true;
}

void AProceduralTerrain::FlushScorch()
{
    if (!bBurnDirty || !BurnMaskTexture || BurnMaskCPU.Num() == 0)
    {
        return;
    }

    // Heap-allocate both the data copy and the region struct: UpdateTextureRegions
    // enqueues a render-thread command, so the pointers must outlive this frame.
    // The DataCleanupFunc deletes them once the render thread is done.
    const int32 Total = BurnMaskCPU.Num();
    uint8* DataCopy = new uint8[Total];
    FMemory::Memcpy(DataCopy, BurnMaskCPU.GetData(), Total);

    const int32 MaskW = CachedGridW + CachedPresentationMarginCells * 2;
    const int32 MaskH = CachedGridH + CachedPresentationMarginCells * 2;
    FUpdateTextureRegion2D* Region = new FUpdateTextureRegion2D(
        0, 0, 0, 0, MaskW, MaskH);

    BurnMaskTexture->UpdateTextureRegions(
        /*MipIndex=*/0, /*NumRegions=*/1, Region,
        /*SrcPitch=*/MaskW, /*SrcBpp=*/1, DataCopy,
        /*DataCleanupFunc=*/[](uint8* InData, const FUpdateTextureRegion2D* InRegion)
        {
            delete[] InData;
            delete InRegion;
        });

    bBurnDirty = false;
}

void AProceduralTerrain::ResetScorch()
{
    if (BurnMaskCPU.Num() == 0)
    {
        return;
    }
    FMemory::Memzero(BurnMaskCPU.GetData(), BurnMaskCPU.Num());
    bBurnDirty = true;
    FlushScorch();
}

bool AProceduralTerrain::ExportMovementCostCsv(const FString& AbsolutePath) const
{
    if (!bBuilt || CachedGridW <= 0 || CachedGridH <= 0)
    {
        return false;
    }

    IFileManager::Get().MakeDirectory(*FPaths::GetPath(AbsolutePath), true);

    FString Csv;
    Csv.Reserve(CachedGridW * CachedGridH * 14);
    for (int32 Y = 0; Y < CachedGridH; ++Y)
    {
        for (int32 X = 0; X < CachedGridW; ++X)
        {
            const float Cost = FMath::Clamp(
                1.0f + SlopeMovementCostFactor * SlopeAtCell(X, Y), 1.0f, 10.0f);
            // "height:cost" per cell. The fire generator (generate_fire_csv.py)
            // reads the height to compute real per-edge grade for slope barriers
            // (uphill faster, downhill slower, true cliffs block); the human
            // generator reads only the cost after the colon. Plain "cost" cells
            // stay valid for both parsers.
            const float HeightUU = GetHeightAtCell(static_cast<float>(X), static_cast<float>(Y));
            Csv += FString::Printf(TEXT("%.2f:%.3f"), HeightUU, Cost);
            Csv += (X + 1 < CachedGridW) ? TEXT(",") : TEXT("\n");
        }
    }

    const bool bOk = FFileHelper::SaveStringToFile(Csv, *AbsolutePath);
    UE_LOG(LogTemp, Log, TEXT("Thesis terrain: height:cost CSV %s -> %s"),
           bOk ? TEXT("written") : TEXT("FAILED"), *AbsolutePath);
    return bOk;
}
