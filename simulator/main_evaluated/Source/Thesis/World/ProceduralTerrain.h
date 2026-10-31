#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralTerrain.generated.h"

class UProceduralMeshComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class UTexture2D;

/** Terrain realism presets. Each sets base relief amplitude and noise character. */
UENUM(BlueprintType)
enum class ETerrainProfile : uint8
{
    /** Perfectly flat ground plane (relief = 0). */
    Flat,
    /** Gentle rolling hills — mostly walkable everywhere. */
    RollingHills,
    /** Pronounced hills and valleys; some slopes slow humans noticeably. */
    Foothills,
    /** Ridged mountainous relief; steep slopes become near-barriers. */
    Mountains
};

/**
 * Procedural, deterministic terrain (replaces PLAN.md B.1's manual-level
 * export with automatic generation).
 *
 * Auto-built by ASimulationManager to match the current grid size: a
 * ProceduralMeshComponent heightfield from seeded multi-octave Perlin (fBm,
 * ridged for Mountains), colored procedurally (grass -> rock by slope, snow by
 * altitude) so it looks plausible in drone captures with NO downloaded assets.
 * Assign a Megascans landscape material to TerrainMaterial for full realism.
 *
 * The SAME height function drives everything that must agree with the visuals:
 *   - movement-cost CSV (slope-based) consumed by Tools/generate_humans.py,
 *     so humans flee slower uphill exactly where the mesh is steep;
 *   - tree placement Z (AForestDresser);
 *   - human actor placement Z (AHumanManager).
 *
 * Toggles: Profile picks the base character; SlopeIntensity scales relief
 * amplitude (more/steeper slopes); DetailDensity scales noise frequency
 * (more, smaller features vs fewer, larger ones).
 *
 * Determinism: heights are a pure function of (seed, profile, toggles, grid
 * size). The exported CSV is byte-stable for identical inputs.
 */
UCLASS(Blueprintable)
class THESIS_API AProceduralTerrain : public AActor
{
    GENERATED_BODY()

public:
    AProceduralTerrain();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    ETerrainProfile Profile = ETerrainProfile::Foothills;

    /** Scales relief amplitude (and therefore slopes). 1 = profile default,
     *  2 = twice as tall/steep. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "0.0", ClampMax = "5.0"))
    float SlopeIntensity = 1.0f;

    /** Scales noise frequency: higher = more, smaller hills packed together;
     *  lower = fewer, broader landforms. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "0.25", ClampMax = "4.0"))
    float DetailDensity = 1.0f;

    /** 0 = reuse the simulation seed passed by the manager. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    int32 SeedOverride = 0;

    /** Mesh vertices per grid cell side (2 = smooth enough for 10 m cells). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "1", ClampMax = "4"))
    int32 VerticesPerCell = 2;

    /** Presentation-only terrain apron beyond all four simulation-grid edges.
     *  Height/noise continues the same analytic landscape; it adds no cells to
     *  the simulation grid or movement-cost export. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain|Presentation",
              meta = (ClampMin = "0", ClampMax = "64"))
    int32 PresentationMarginCells = 12;

    /** Minimum visual ground distance beyond each grid edge, in metres.
     *  The distant apron uses coarse bands; simulation cells are unchanged. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain|Presentation",
              meta = (ClampMin = "0", ClampMax = "10000", Units = "m"))
    float PresentationMarginMeters = 1000.0f;

    /** Movement-cost penalty per unit slope for the human generator's CSV.
     *  cost = clamp(1 + factor * slope, 1, 10); slope = rise/run. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "0.0"))
    float SlopeMovementCostFactor = 6.0f;

    /** Terrain material (e.g. M_ThesisTerrain — triplanar + burn-mask).
     *  BuildTerrain creates a MID from this and pushes per-build scalars.
     *  Empty = built-in vertex-color shading (grass/rock). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    TObjectPtr<UMaterialInterface> TerrainMaterial;

    /** Render only in drone captures (like the forest). Default off — one
     *  static mesh is cheap and seeing the terrain in the viewport is useful. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    bool bVisibleOnlyToDroneCaptures = false;

    /** Render the terrain mesh at all. Height queries, normals, burn/scorch
     *  state, and movement-cost CSV export still work when this is false. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    bool bRenderVisuals = true;

    /** Runtime view-mode helper used by ASimulationManager. */
    void SetVisualsCaptureOnly(bool bCaptureOnly);
    void SetVisualsEnabled(bool bEnabled);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Terrain")
    TObjectPtr<UProceduralMeshComponent> Mesh;

    /** Independent primitives give each camera and shadow view tight culling
     *  bounds. Zero retains a single mesh for reference comparisons. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain|Presentation",
              meta = (ClampMin = "0", ClampMax = "128"))
    int32 RenderTileQuads = 64;

    /** Relief amplitude actually used on the last build (UU). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Terrain")
    float BuiltAmplitudeUU = 0.0f;

    /** (Re)builds the heightfield for the given grid. Called by the manager. */
    void BuildTerrain(int32 GridW, int32 GridH, const FVector& GridOriginWorld,
                      float CellSizeUU, int32 SimSeed);

    /** Continuous world-space ground height at fractional cell coords. */
    float GetHeightAtCell(float CellX, float CellY) const;

    /** Converts a rendered heightfield vertex index to the unchanged grid-
     *  relative cell coordinate, including the presentation-only apron. */
    static float PresentationVertexCellCoordinate(int32 VertexIndex,
                                                  int32 VerticesPerCell,
                                                  int32 MarginCells);

    /** Continuous world-space terrain normal at fractional cell coords. */
    FVector GetNormalAtCell(float CellX, float CellY) const;

    /** Writes the per-cell movement-cost CSV for Tools/generate_humans.py. */
    bool ExportMovementCostCsv(const FString& AbsolutePath) const;

    // ---------- Scorched-ground feedback (visual only; reads ground truth) ----------

    /** Marks a burned-out cell in the burn mask (one byte write, O(1)).
     *  Nothing uploads until FlushScorch(). Called by AFireVisualsManager. */
    void ScorchCell(int32 CellX, int32 CellY);

    /** Uploads the pending burn mask to the GPU texture.
     *  Call on a modest cadence — full PF_G8 texture upload (~90 KB). */
    void FlushScorch();

    /** Clears the burn mask and uploads immediately. Used on simulation reset. */
    void ResetScorch();

    bool IsBuilt() const { return bBuilt; }
    /** Exact immutable signature of the currently built analytic height field. */
    FString GetHeightFieldSignature() const;

private:
    friend class FTerrainSceneCacheTest;

    /** Mesh is the first tile/root; these are the remaining render primitives. */
    UPROPERTY(Transient)
    TArray<TObjectPtr<UProceduralMeshComponent>> AdditionalRenderTiles;

    void ClearRenderTiles();

    // Cached build parameters (the pure height function's inputs).
    bool bBuilt = false;
    int32 CachedGridW = 0;
    int32 CachedGridH = 0;
    FVector CachedGridOrigin = FVector::ZeroVector;
    float CachedCellSizeUU = 1000.0f;
    int32 CachedPresentationMarginCells = 0;
    float CachedBaseFreq = 0.05f;
    FVector2D CachedNoiseOffset = FVector2D::ZeroVector;
    FVector2D CachedWarpOffset = FVector2D::ZeroVector;
    FVector2D CachedRidgeOffset = FVector2D::ZeroVector;
    FVector2D CachedLandformOffset = FVector2D::ZeroVector;
    float CachedWarpStrengthCells = 0.0f;
    float CachedRidgeStrength = 0.0f;
    bool bCachedRidged = false;

    /** fBm noise in roughly [-1, 1] at fractional cell coords. */
    float SampleNoise(float CellX, float CellY) const;

    /** Slope (rise/run, unitless) at integer cell coords via central differences. */
    float SlopeAtCell(int32 CellX, int32 CellY) const;

    // ---------- Material Instance Dynamic (Part 1 — triplanar material wiring) ----------

    /** MID created at BuildTerrain from TerrainMaterial; receives per-build
     *  scalars (P_TerrainMinZ, P_TerrainAmplitude, P_GridOriginXY,
     *  P_GridWorldSizeXY, P_BurnMaskEnabled, T_BurnMask). Null if
     *  TerrainMaterial is unset. */
    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> TerrainMID;

    // ---------- Burn-mask scorch (Part 2 — replaces vertex-color rewrite) ----------

    /** GridW x GridH, PF_G8, sRGB=off, TF_Bilinear. Sampled by T_BurnMask in
     *  M_ThesisTerrain. 0 = unburned, 255 = fully charred. */
    UPROPERTY(Transient)
    TObjectPtr<UTexture2D> BurnMaskTexture;

    /** CPU-side mirror of BurnMaskTexture. Source of truth; uploaded on flush. */
    TArray<uint8> BurnMaskCPU;

    /** True when BurnMaskCPU has pending writes not yet on the GPU. */
    bool bBurnDirty = false;

    /** Allocates BurnMaskCPU (zeroed) and creates/uploads BurnMaskTexture.
     *  Called at the end of BuildTerrain. */
    void InitBurnMask();
};
