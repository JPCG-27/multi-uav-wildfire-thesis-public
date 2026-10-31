#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ForestDresser.generated.h"

class UHierarchicalInstancedStaticMeshComponent;
class AProceduralTerrain;
class UStaticMesh;
class Grid;

/**
 * Procedural, deterministic flora (PLAN.md C.1).
 *
 * Generates a flora-density field from seeded Perlin noise, writes it into the
 * Grid's floraDensity (the FUEL map: it is exported and fed to the fire
 * generator, so the fire burns exactly where the visual forest stands), and
 * spawns instanced flora in three layers for realism:
 *
 *   - TreeMeshes:        canopy, only where density is high; count scales with density.
 *   - BushMeshes:        understory, anywhere with some vegetation; smaller scale.
 *   - GroundCoverMeshes: upright small plants, scattered in clearings too.
 *   - SurfaceDebrisMeshes: slope-fitted logs / branches / bark.
 *
 * Assign multiple meshes per layer (e.g. 2-3 Megascans trees + bushes) â€” each
 * instance picks one deterministically, which breaks up the repetitive look.
 * Bounded HISM chunks per unique mesh keep instancing efficient without
 * exceeding Unreal's per-traversal foliage triangle budget.
 *
 * Determinism: placement is a pure function of (Seed, grid size); per-cell
 * RandomStream, fixed iteration order, no wall clock.
 */
UCLASS(Blueprintable)
class THESIS_API AForestDresser : public AActor
{
    GENERATED_BODY()

public:
    AForestDresser();

    /** Prefer full-quality, fully resident Nanite duplicates prepared by
     *  Tools/prepare_sensor_meshes.py. Original geometry is the fallback. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Flora")
    bool bPreferFullyResidentMeshes = true;

    /** Canopy trees (assign 1-3 Fab/Quixel trees). Legacy TreeMesh is appended. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Flora")
    TArray<TObjectPtr<UStaticMesh>> TreeMeshes;

    /** Understory bushes/saplings; spawned wherever vegetation exists. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Flora")
    TArray<TObjectPtr<UStaticMesh>> BushMeshes;

    /** Upright ground cover, e.g. seedlings or small plant clumps. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Flora")
    TArray<TObjectPtr<UStaticMesh>> GroundCoverMeshes;

    /** Lying debris, e.g. dead branches, logs, bark and exposed roots. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Flora")
    TArray<TObjectPtr<UStaticMesh>> SurfaceDebrisMeshes;

    /** Legacy single-tree slot (kept for older levels; appended to TreeMeshes). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Flora")
    TObjectPtr<UStaticMesh> TreeMesh;

    /** When every mesh list above is empty, auto-load the standard set imported
     *  by Tools/import_polyhaven.py from /Game/ThesisFlora (if present). Zero
     *  manual assignment needed after running the two asset scripts. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Flora")
    bool bAutoPopulateFromThesisFlora = true;

    /** Skip spawning HISM mesh instances (visual only) while still writing the
     *  floraDensity fuel map. The manager sets this false for headless / no-capture
     *  (paper-mode) runs, where nothing ever renders the instances. */
    bool bSpawnVisualInstances = true;

    /** 0 = reuse the simulation seed passed by the manager. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest")
    int32 SeedOverride = 0;

    /** Approximate fraction of the grid covered by forest.
     *  0.85 gives a closed-canopy Portuguese woodland look. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float ForestCoverage = 0.92f;

    /** Noise feature size in cells (bigger = larger continuous woods/clearings). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "1.0"))
    float NoiseScaleCells = 12.0f;

    /** Extra deterministic flora outside the simulation grid for edge views.
     *  Exterior dressing never writes fuel, fire, or any other grid state. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Presentation",
              meta = (ClampMin = "0", ClampMax = "64"))
    int32 PresentationMarginCells = 12;

    /** Maximum deterministic scatter patches in the distant terrain apron.
     *  The detailed simulation-edge band remains unchanged; this budget keeps
     *  kilometre-scale presentation terrain from multiplying per-cell flora. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Presentation",
              meta = (ClampMin = "0", ClampMax = "16384"))
    int32 PresentationApronMaxPatches = 4096;

    /** Spatial extent of each foliage HISM group. Smaller groups get tighter
     *  camera and shadow bounds at the cost of more components. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Performance",
              meta = (ClampMin = "0", ClampMax = "128"))
    int32 HISMSpatialTileSideCells = 0;

    /** Small ground clutter contributes little to the image but can dominate
     *  virtual-shadow-map marking work across a large forest. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Performance")
    bool bGroundCoverCastsShadow = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Performance")
    bool bSurfaceDebrisCastsShadow = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 MaxTreesPerCell = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 MaxBushesPerCell = 4;

    /** Upright ground-cover instances per cell (uniform everywhere). 0 disables. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 GroundCoverPerCell = 4;

    /** Surface-fitted debris attempts per cell. 0 disables. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 SurfaceDebrisPerCell = 2;

    /** Positive values sink lying debris slightly into the sampled terrain. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.0", ClampMax = "200.0"))
    float SurfaceDebrisEmbedUU = 14.0f;

    /** Positive values sink rocks slightly into the sampled terrain. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.0", ClampMax = "250.0"))
    float SparseRockEmbedUU = 28.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.01"))
    float TreeScaleMin = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.01"))
    float TreeScaleMax = 0.9f;

    /** Render flora only inside drone scene captures. Off by default â€” Nanite
     *  keeps the viewport cheap; turn on if the editor ever gets too heavy. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest")
    bool bVisibleOnlyToDroneCaptures = false;

    /** Runtime view-mode helper used by ASimulationManager. */
    void SetVisualsCaptureOnly(bool bCaptureOnly);

    /** Returns deterministic vegetation base locations inside a cell, used by
     *  fire visuals to anchor small flame clusters around living trees/bushes. */
    int32 GetVegetationFireAnchors(int32 CellX, int32 CellY, TArray<FVector>& OutAnchors, int32 MaxAnchors) const;

    /** Disable material world-position-offset wind on generated foliage.
     *  Megascans SimpleWind materials can make HISM trees appear to breathe
     *  (grow/shrink). Static foliage is better for deterministic captures. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest")
    bool bDisableFoliageWind = true;

    /** Z offset (UU) added to every tree/bush instance after the terrain height
     *  lookup.  Use a small negative value (e.g. -20) if meshes appear to hover
     *  above the ground due to a non-zero pivot in their local space. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest",
              meta = (ClampMin = "-500.0", ClampMax = "500.0"))
    float TreeZOffsetUU = 0.0f;

    /** Called by ASimulationManager after the grid exists (and before drones
     *  copy floraDensity). Regenerates everything from scratch. Terrain may be
     *  null (flat ground at the grid origin's Z). */
    void BuildForest(Grid* WorldGrid, const FVector& GridOriginWorld, float CellSizeUU, int32 SimSeed,
                     const AProceduralTerrain* Terrain);

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Forest")
    int32 NumInstancesSpawned = 0;

    // ---------- Scorched-ground feedback (visual only; reads ground truth) ----------

    /** Write 0 (live) / 1 (burned) to HISM per-instance custom data. Foliage
     *  materials can consume this value to desaturate and char without replacing
     *  meshes or changing the simulation's flora/fuel field. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Scorch")
    bool bWriteScorchToPerInstanceCustomData = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest|Scorch", meta = (ClampMin = "0", ClampMax = "7"))
    int32 ScorchCustomDataIndex = 0;

    /** Marks the cell as burned for future vegetation feedback. Trees and
     *  bushes stay standing when the cell burns out; burned forests should
     *  char/thin later, not vanish. */
    void ScorchCell(int32 CellX, int32 CellY);

    /** Clears runtime scorch bookkeeping on simulation reset. */
    void ResetScorch();

private:
    /** Bounded HISM chunks per mesh, rebuilt on every BuildForest. */
    UPROPERTY()
    TArray<TObjectPtr<UHierarchicalInstancedStaticMeshComponent>> FloraComponents;

    UHierarchicalInstancedStaticMeshComponent* GetOrCreateComponentForMesh(
        UStaticMesh* Mesh, TArray<int32>& ExistingComponents,
        int32 ComponentOrdinal, bool bCastShadow);

    // Scorch bookkeeping: which HISM instances live in which cell (vegetation
    // only), and which cells/instances have received visual burn state.
    struct FCellInstanceRef
    {
        int32 ComponentIndex = INDEX_NONE;
        int32 InstanceIndex = INDEX_NONE;
    };
    TMap<int32, TArray<FCellInstanceRef>> CellToVegetationInstances;
    TSet<int32> ScorchedCells;
    TArray<FCellInstanceRef> ScorchedInstances;
    int32 BuiltGridW = 0;
};
