#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/EngineTypes.h"
#include "Grid.h"
#include "Drone.h"
#include "DroneSensorBase.h"
#include "PartitionStrategy.h"
#include "SimMetricsRecorder.h"
#include "VisionTypes.h"
#include "VisionBridge.h"          // needed: TUniquePtr<FVisionBridge> member requires complete type in gen.cpp
#include "ProceduralTerrain.h"
#include "MethodSpec.h"
#include "GroundTruthOracle.h"
#include "HumanTypes.h"
#include "YoloDatasetExporter.h"
#include "ChanceNullScorer.h"
#include "SimulationManager.generated.h"

class ADrone;
class AFireVisualsManager;
class UFireVisualProfile;
class AForestDresser;
class AHumanManager;
class UGpuBeliefManager;
class ASimulationHUD;
class UDroneSensorBase;
class UDronePlannerBase;

UENUM(BlueprintType)
enum class ESimState : uint8
{
    Stopped,
    Running,
    Paused
};

UENUM(BlueprintType)
enum class EDebugPrioritySource : uint8
{
    None,
    ServerPriority,
    DroneLocalPriority,
    GroundTruthPriority,
    FireGroundTruth
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnTickAdvanced, int32, NewTick);

struct FPendingWallClockPhaseTiming
{
    int32 Tick = -1;
    FString PhaseName;
    float ElapsedMs = 0.0f;
};

UCLASS(Blueprintable)
class THESIS_API ASimulationManager : public AActor, public IGroundTruthOracle
{
    GENERATED_BODY()

public:
    ASimulationManager();
    /** Destructor defined in .cpp so FVisionBridge (TUniquePtr member) is complete
     *  at the deletion point — forward declaration in the header is not enough. */
    ~ASimulationManager();

    // ---------- static instance (used by HUD) ----------
    static ASimulationManager* StaticInstance;

    // ---------- public accessors for HUD and debug ----------
    FVector GetGridOriginWorld() const { return GridOriginWorld; }
    float ComputeGroundTruthPriority(int32 X, int32 Y, int32 Tick) const;
    bool IsFireActive(int32 X, int32 Y, int32 Tick) const;
    const TArray<float>& GetCachedServerPriority() const { return CachedServerPriority; }
    int32 GetBenchmarkRunSeed() const { return FireGeneratorSeed; }

    void ApplyMethodId(const FString& InMethodId);
    void RefreshMethodSpecFromNames();
    bool IsStaticUniformRegime() const { return ActiveMethodSpec.Regime == ERegionRegime::StaticUniform; }
    bool IsStaticPartitionRegime() const { return IsStaticUniformRegime() || ActiveMethodSpec.Regime == ERegionRegime::StaticRandom; }
    virtual bool IsOracleInformationEnabled() const override { return ActiveMethodSpec.Info == EInformationSource::GroundTruthOracle; }
    virtual int32 GetOracleGridWidth() const override { return GridWidth; }
    virtual int32 GetOracleGridHeight() const override { return GridHeight; }
    virtual int32 GetFirstFireTickByCellIndex(int32 CellIndex) const override;
    virtual int32 GetRealPersonCountByCellIndex(int32 CellIndex) const override;
    virtual float ComputeOracleCellLoad(int32 CellIndex, int32 Tick, int32 LookaheadTicks) const override;
    virtual const AHumanManager* GetOracleHumanManager() const override { return HumanManager.Get(); }
    bool FindOracleTargetForDrone(const ADrone& Drone, int32 AtTick, int32& OutCell, float& OutScore) const;

    UPROPERTY(BlueprintAssignable, Category = "Thesis|Simulation")
    FOnTickAdvanced OnTickAdvanced;

    /** Switch priority overlay to a specific source. Call from BP or key bind. */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Debug")
    void SetDebugPrioritySource(EDebugPrioritySource NewSource);

    /** Cycle through: None → GroundTruth → DroneLocal → ServerPriority → None */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Debug")
    void CycleDebugPrioritySource();

    /** Set which drone's local priority to display. */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Debug")
    void SetDebugPriorityDroneIndex(int32 NewIndex);

    // ---------- Drone view camera (spectating) ----------

    /** Blend the player view to drone I's onboard camera. Out-of-range = ignored. */
    UFUNCTION(BlueprintCallable, Category = "Thesis|View")
    void ViewDrone(int32 DroneIdx);

    /** Return the player view to the free overview pawn. */
    UFUNCTION(BlueprintCallable, Category = "Thesis|View")
    void ViewOverview();

    /** V key: toggle between overview and the last-viewed drone. */
    UFUNCTION(BlueprintCallable, Category = "Thesis|View")
    void ToggleDroneView();

    /** ] key: next drone (wraps; from overview goes to drone 0). */
    UFUNCTION(BlueprintCallable, Category = "Thesis|View")
    void ViewNextDrone();

    /** [ key: previous drone (wraps; from overview goes to the last drone). */
    UFUNCTION(BlueprintCallable, Category = "Thesis|View")
    void ViewPreviousDrone();

    /** INDEX_NONE = overview. Read by the HUD view buttons. */
    UFUNCTION(BlueprintPure, Category = "Thesis|View")
    int32 GetViewedDroneIndex() const { return ViewedDroneIndex; }

    /** Camera blend seconds when switching view targets. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|View", meta = (ClampMin = "0.0"))
    float DroneViewBlendTime = 0.4f;

    /** Overview mode hides photoreal terrain/forest/human meshes from the main
     *  viewport while keeping them visible to drone scene captures. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|View")
    bool bOverviewShowsGridOnly = true;

    UFUNCTION(BlueprintCallable, Category = "Thesis|Simulation")
    void StartSimulation();
    UFUNCTION(BlueprintCallable, Category = "Thesis|Simulation")
    void PauseSimulation();
    UFUNCTION(BlueprintCallable, Category = "Thesis|Simulation")
    void ResumeSimulation();
    UFUNCTION(BlueprintCallable, Category = "Thesis|Simulation")
    void ToggleSimulationPause();
    UFUNCTION(BlueprintCallable, Category = "Thesis|Simulation")
    void ResetSimulation();

    /** In-process scenario batching (opt-in via -SimConfigManifest=<file>): runs
     *  many SAME-GRID scenarios in one process to amortise editor startup. The
     *  default per-process path (no manifest) is completely unchanged. */
    bool SwapToNextScenario();
    void CaptureInitialDroneSpawnLocations();
    void RecordInitialDroneIdentityMap();
    TArray<FString> ScenarioManifestPaths;
    int32 CurrentScenarioIndex = 0;
    TArray<FVector> InitialDroneSpawnLocations;

    /** Runs Tools/generate_fire_csv.py using the manager's fire-generator settings.
     *  If bLoadGeneratedFireCsvAfterRun is true, the output path becomes FirstFireTickCSVPath
     *  and the simulation is reset so the new fire map is loaded immediately.
     */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Fire Generator")
    bool GenerateFireCsvFromManagerSettings();

    UFUNCTION(BlueprintPure, Category = "Thesis|Simulation")
    int32 GetCurrentTick() const { return CurrentTick; }
    UFUNCTION(BlueprintPure, Category = "Thesis|Simulation")
    float GetSimTime() const { return CurrentTick * SecondsPerTick; }
    UFUNCTION(BlueprintPure, Category = "Thesis|Simulation")
    ESimState GetSimState() const { return State; }

    Grid* GetGrid() const { return WorldGrid.Get(); }

    UFUNCTION(BlueprintPure, Category = "Thesis|Debug")
    int32 DebugGetFirstFireTick(int32 X, int32 Y) const;

    // ---------- Grid & simulation parameters ----------

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Grid", meta = (ClampMin = "1"))
    int32 GridWidth = 30;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Grid", meta = (ClampMin = "1"))
    int32 GridHeight = 30;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Simulation")
    float SecondsPerTick = 1.0f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Simulation")
    bool bAutoStart = true;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Simulation", meta = (ClampMin = "0"))
    float MaxTicksPerSecond = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Fire", meta = (FilePathFilter = "csv"))
    FFilePath FirstFireTickCSVPath;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Fire", meta = (ClampMin = "0.001"))
    float ArrivalMinutesPerTick = 1.0f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Fire", meta = (ClampMin = "1"))
    int32 BurnDurationTicks = 60;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Fire")
    bool bUseFallbackFireIfNoCSV = true;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Fire", meta = (ClampMin = "0"))
    int32 FallbackIgnitionRadiusCells = 2;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Fire", meta = (ClampMin = "0"))
    int32 FallbackSpreadTicksPerCell = 5;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Fire", meta = (ClampMin = "0"))
    int32 FallbackMaxSpreadCells = 10;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Grid", meta = (ClampMin = "1.0"))
    float CellSizeUU = 500.0f;

    // ---------- Terrain (procedural, auto-sized to the grid) ----------

    /** Auto-build an AProceduralTerrain matching the grid at BeginPlay (finds a
     *  level-placed one first; spawns one otherwise) and export its slope-based
     *  movement-cost CSV for the human generator. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    bool bAutoGenerateTerrain = true;

    /** Pushed to the terrain actor before every BuildTerrain — applies to both
     *  level-placed and auto-spawned terrain, so you never need to touch the
     *  AProceduralTerrain Details panel. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    ETerrainProfile TerrainProfile = ETerrainProfile::Foothills;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "0.0", ClampMax = "5.0"))
    float TerrainSlopeIntensity = 1.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "0.25", ClampMax = "4.0"))
    float TerrainDetailDensity = 0.85f;

    /** 0 = reuse FireGeneratorSeed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    int32 TerrainSeedOverride = 0;

    /** Mesh vertices per grid-cell side (2 is smooth enough for 10 m cells). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "1", ClampMax = "4"))
    int32 TerrainVerticesPerCell = 3;

    /** Minimum rendered terrain beyond every grid edge; distant geometry is coarser. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "0", ClampMax = "10000", Units = "m"))
    float TerrainPresentationMarginMeters = 1000.0f;

    /** cost = clamp(1 + factor * slope, 1, 10); slope = rise/run. Controls the
     *  terrain CSV consumed by Tools/generate_humans.py. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain", meta = (ClampMin = "0.0"))
    float TerrainSlopeMovementCostFactor = 6.0f;

    /** Render terrain only inside drone scene captures. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    bool bTerrainVisibleOnlyToDroneCaptures = false;

    /** Render the terrain mesh. Perfect-sensor benchmark configs disable this
     *  while keeping terrain data alive for height/cost/fire/human logic. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    bool bRenderTerrainVisuals = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Terrain")
    FFilePath GeneratedTerrainCsvOutputPath;

    /** floraDensity export consumed by the fire generator as its fuel map. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator")
    FFilePath GeneratedFuelCsvOutputPath;

    // ---------- Forest ----------

    /** Pushed to the AForestDresser actor before every BuildForest — applies to
     *  both level-placed and auto-spawned forest.  Mesh arrays (TreeMeshes,
     *  BushMeshes, GroundCoverMeshes) must still be assigned on the actor since
     *  they are asset references the manager cannot hold for you. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float ForestCoverage = 0.92f;

    /** Noise feature size in cells (larger = bigger continuous forest patches). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "1.0"))
    float ForestNoiseScaleCells = 12.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 ForestMaxTreesPerCell = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 ForestMaxBushesPerCell = 4;

    /** Upright ground-cover instances per cell (uniform everywhere, clearings included). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 ForestGroundCoverPerCell = 4;

    /** Surface-fitted debris attempts per cell. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0"))
    int32 ForestSurfaceDebrisPerCell = 2;

    /** Positive values sink lying debris slightly into the sampled terrain. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.0", ClampMax = "200.0"))
    float ForestSurfaceDebrisEmbedUU = 14.0f;

    /** Positive values sink rocks slightly into the sampled terrain. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.0", ClampMax = "250.0"))
    float ForestSparseRockEmbedUU = 28.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.01"))
    float ForestTreeScaleMin = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "0.01"))
    float ForestTreeScaleMax = 0.9f;

    /** Z offset (UU) per tree/bush instance after the terrain height lookup.
     *  Use a small negative value if meshes hover above ground (pivot offset). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest", meta = (ClampMin = "-500.0", ClampMax = "500.0"))
    float ForestTreeZOffsetUU = 0.0f;

    /** 0 = reuse FireGeneratorSeed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest")
    int32 ForestSeedOverride = 0;

    /** When the forest dresser has no mesh arrays assigned, auto-load the
     *  standard set from /Game/ThesisFlora (requires import_polyhaven.py). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest")
    bool bForestAutoPopulate = true;

    /** Render flora only inside drone scene captures. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest")
    bool bForestVisibleOnlyToDroneCaptures = false;

    /** Disable Megascans foliage wind/world-position-offset on generated HISM
     *  trees so they do not grow/shrink during the simulation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Forest")
    bool bForestDisableFoliageWind = true;

    // ---------- Fire Visuals ----------

    /** Spawn an AFireVisualsManager (flames/smoke VFX slots + flickering fire
     *  lights) and drive it from the tick. Lights need no assets.
     *  All settings below are pushed to the actor before Init — no need to
     *  set them on the AFireVisualsManager Details panel. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bEnableFireVisuals = true;

    /** Optional presentation-only data asset. When assigned it controls the
     *  VFX assets, visual lifecycle, deterministic Niagara age, and wind. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    TObjectPtr<UFireVisualProfile> FireVisualProfile;

    /** If the FireVisualsManager has no assigned systems, try bundled defaults
     *  from Niagara Fluids or Starter Content. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bFireVisualsAutoLoadDefaultVfx = true;

    /** Flickering orange point lights at burning cells (no VFX asset required). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bFireLightsEnabled = true;

    /** Legacy compatibility flag. Hidden showcase overrides are no longer
     *  applied; the visible fields/profile are now always authoritative. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals",
              meta = (DeprecatedProperty, DeprecationMessage = "Showcase overrides are no longer applied; use FireVisualProfile or the explicit visual fields."))
    bool bFireVisualsShowcaseDensity = false;

    /** Performance mode: only create/activate expensive fire VFX while a
     *  YOLO-connected drone scene capture is rendering. Fire state and scorch
     *  still advance every tick. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bFireVisualsOnlyForYoloCaptures = true;

    /** Show tick-driven fire/smoke during Play/Simulate in Editor even when
     *  capture-only performance mode is enabled. Does not affect standalone runs. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bPreviewFireInEditor = true;

    /** World-space radius around the YOLO drone capture used to activate fire
     *  VFX clusters. Smaller = faster; larger = more smoke/fire in captures. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance", meta = (ClampMin = "500.0"))
    float FireVisualsYoloCaptureRadiusUU = 4500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0", ClampMax = "256"))
    int32 FireVisualsMaxVfx = 128;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Cluster", meta = (ClampMin = "1", ClampMax = "8"))
    int32 FireVisualsClustersPerCell = 4;

    /** Join adjacent burning cells into an overlapping flame footprint. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bFireVisualsConnectedFireFront = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Cluster", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float FireVisualsClusterRadiusUU = 190.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0", ClampMax = "64"))
    int32 FireVisualsMaxLights = 12;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "100.0"))
    float FireVisualsLightRadiusUU = 3000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0"))
    float FireVisualsLightIntensity = 1200000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0", ClampMax = "16"))
    int32 FireVisualsMaxShadowCastingLights = 0;

    /** Reassign pooled VFX to the moving fire front every N ticks. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "1"))
    int32 FireVisualsReassignEveryNTicks = 1;

    /** Scale flame VFX with the cell's fuel density (bigger fire in dense forest). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bFireVisualsScaleWithFuel = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "20.0"))
    float FireVisualsVfxScaleMultiplier = 0.62f;

    /** Broaden ground coverage without adding particle systems. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.1", ClampMax = "10.0"))
    float FireVisualsFlameWidthScaleMultiplier = 1.2f;

    /** Extend flame tongues up vegetation independently of ground width. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.1", ClampMax = "10.0"))
    float FireVisualsFlameHeightScaleMultiplier = 4.0f;

    /** Enlarge bundled flame sprites as well as their particle positions. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.1", ClampMax = "10.0"))
    float FireVisualsBundledFlameSpriteSizeMultiplier = 2.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bFireVisualsEnableBundledHeatDistortion = false;


    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "20.0"))
    float FireVisualsOuterFlameScaleMultiplier = 0.90f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "20.0"))
    float FireVisualsEmberScaleMultiplier = 0.75f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "30.0"))
    float FireVisualsSmokeVfxScaleMultiplier = 1.20f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "1", ClampMax = "8"))
    int32 FireVisualsSmokeLayerCount = 2;

    /** Share one plume stack across a cell's flame clusters to avoid overlapping smoke work. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bFireVisualsShareSmokeAcrossCellClusters = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float FireVisualsSmokeLayerRadiusUU = 260.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float FireVisualsSmokeLayerZStepUU = 240.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "-1000.0", ClampMax = "5000.0"))
    float FireVisualsFlameZOffsetUU = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "-1000.0", ClampMax = "8000.0"))
    float FireVisualsSmokeZOffsetUU = 260.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "-1000.0", ClampMax = "5000.0"))
    float FireVisualsLightZOffsetUU = 120.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float FireVisualsLightSourceRadiusUU = 650.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "20.0"))
    float FireVisualsLightVolumetricScatteringIntensity = 8.0f;

    /** Render fire VFX only inside drone captures; lights always show in viewport. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bFireVisualsVfxOnlyInDroneCaptures = false;

    /** Darken terrain and collapse vegetation when cells burn out (visual only). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bFireVisualsScorchedGround = true;

    /** Terrain scorch texture upload cadence in ticks (full PF_G8 upload per call). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "1"))
    int32 FireVisualsScorchFlushEveryNTicks = 15;

    // ---------- Humans (precomputed trajectory replay) ----------

    /** humans.csv from Tools/generate_humans.py. Empty = no humans. At BeginPlay
     *  the manager finds (or spawns) an AHumanManager and loads this file. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Humans", meta = (FilePathFilter = "csv"))
    FFilePath HumansCSVPath;

    /** Auto-generate the whole scenario at BeginPlay, sized to the current grid:
     *  fire CSV (Tools/generate_fire_csv.py) and human trajectories
     *  (Tools/generate_humans.py, fled from that fire). Both deterministic from
     *  the seeds below. Disabled automatically when a run config supplies
     *  explicit CSV paths (see auto_generate_scenario in SimRunConfig). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Human Generator")
    bool bAutoGenerateScenarioOnBeginPlay = true;

    /** Persons to generate. 0 = auto-scale with grid area (~1 per 750 cells, min 4). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Human Generator", meta = (ClampMin = "0"))
    int32 HumanGeneratorCount = 0;

    /** Trajectory length in ticks. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Human Generator", meta = (ClampMin = "1"))
    int32 HumanGeneratorMaxTicks = 2000;

    /** Seed for human placement/behaviour. 0 = reuse FireGeneratorSeed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Human Generator")
    int32 HumanGeneratorSeed = 0;

    /** Visual-only scale for the human mesh actors used by drone captures. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals", meta = (ClampMin = "0.1"))
    float HumanMeshScale = 1.0f;

    /** mixed (Mixamo + Manny/Quinn), mixamo, or mannequin. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    FString HumanVisualMeshSet = TEXT("mixed");

    /** automatic, varied, standing, walking, running, waving, crouching, seated, lying. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    FString HumanPoseMode = TEXT("automatic");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Human Generator")
    FFilePath GeneratedHumansCsvOutputPath;

    /** Runs Tools/generate_humans.py against the current fire CSV and grid size.
     *  On success, HumansCSVPath is pointed at the output. */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Human Generator")
    bool GenerateHumansCsvFromManagerSettings();

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Partition", meta = (ClampMin = "1"))
    int32 PartitionRecomputeEveryNTicks = 50;

    /** Which IPartitionStrategy to use. Registered names live in
     *  PartitionStrategyFactory.cpp ("ZoneRegionGrowing" = current algorithm).
     *  Unknown names log a warning and fall back to the default. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Partition")
    FString PartitionStrategyName = TEXT("ZoneRegionGrowing");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    FString MethodId = TEXT("mine_dynamic");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    FString RegionRegimeName = TEXT("dynamic");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    FString InformationSourceName = TEXT("belief");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (ClampMin = "1"))
    int32 OracleLookaheadTicks = 500;

    /** Provenance guard for the physical clairvoyant oracle. Every objective
     *  result still requires a drone flight and sensor observation. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Benchmark")
    int32 OraclePolicyVersion = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (ClampMin = "1"))
    int32 RecallDeadlineTicks = 500;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float DangerExposureThreshold = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (ClampMin = "1"))
    int32 TrackingWindowTicks = 50;

    /** Wall-clock timing is useful for profiling, but volatile. Keep off for
     *  byte-identical determinism gates. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    bool bRecordWallClockTiming = false;

    /** Live viewport only; sensor captures retain their calibrated exposure. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Live View")
    bool bCorrectLiveViewLighting = true;

    /** Use prepared Nanite geometry for presentation-only play. Any enabled
     *  sensor/dataset capture keeps the deterministic fallback renderer. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Live View")
    bool bUseNaniteForLiveView = true;

    /** Stops relative to the level/camera exposure. -1 halves exposure. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Live View", meta = (ClampMin = "-5.0", ClampMax = "5.0"))
    float LiveViewExposureCompensation = -1.0f;

    // ---------- Benchmark / metrics ----------

    /** Record per-tick metrics CSVs to Saved/Benchmarks/<RunId>/. Recording never
     *  influences the simulation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    bool bRecordMetrics = true;

    /** Score deterministic circular shifts of the cached detection stream without
     *  feeding shifted detections into sensing, belief, or planning. Requires Replay. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    bool bEnableCachedDetectionChanceNull = false;

    /** Use the four-offset instrument pilot set (0, 10%, 50%, 90%); full analysis
     *  uses the frozen 17 nonzero fraction ranks plus offset zero. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark",
              meta = (EditCondition = "bEnableCachedDetectionChanceNull"))
    bool bCachedDetectionChanceNullPilot = true;

    /** Explicit output cohort namespace, e.g. mixed or all-yolo. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark",
              meta = (EditCondition = "bEnableCachedDetectionChanceNull"))
    FString CachedDetectionChanceNullCohort = TEXT("mixed1");

    /** Skip the unused overview in offscreen benchmarks; RHI and explicit drone
     *  captures remain active. Restored on EndPlay, including batch reloads. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    bool bSuppressBenchmarkViewport = false;

    /** "full" writes tick/drone trace CSVs; "summary" keeps paper summary/person
     *  metrics and sparse events but skips the large per-tick/per-drone files. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    FString MetricsDetailMode = TEXT("full");

    /** Folder name for this run's metrics. Empty = auto timestamp. Set explicitly
     *  for reproducible output paths (e.g. golden-run comparisons). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    FString MetricsRunId;

    /** Optional subdirectory under Saved/Benchmarks for this run's CSVs.
     *  Empty keeps the legacy flat Saved/Benchmarks/<MetricsRunId>/ layout. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    FString MetricsRunSubdir;

    /** Stop the simulation when CurrentTick reaches this value. 0 = run forever. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (ClampMin = "0"))
    int32 MaxSimTicks = 0;

    /** Quit the process after MaxSimTicks (for headless batch runs). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    bool bExitOnMaxTicks = false;

    /** Cadence of optional GPU grid-stats reduction (mean/max staleness + coverage)
     *  recorded into events.csv. 0 = off. Paper/person/fire benchmark metrics do
     *  not need this readback, so it defaults off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (ClampMin = "0"))
    int32 GridStatsEveryNTicks = 0;

    /** Ticks between dispatching the stats reduction and (blocking) consumption.
     *  Gives the GPU time so the fence wait is usually free. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (ClampMin = "1"))
    int32 GridStatsReadbackLagTicks = 5;

    /** Optional JSON run config applied at BeginPlay. Overridden by the
     *  -SimConfig=<path> command line argument. Only keys present in the file
     *  override editor values; see SimRunConfig.h. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark", meta = (FilePathFilter = "json"))
    FFilePath SimConfigPath;

    // ---------- Vision / drone captures (Phase C) ----------

    /** Give every drone a downward scene-capture camera and render it on the
     *  cadence below. Off by default — enable once the level has visual assets
     *  (ForestDresser trees, human meshes) worth photographing. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    bool bEnableDroneCaptures = false;

    /** Writes read-only camera pose, cache association and live pixel hash audit rows. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    bool bEnableCameraStateAudit = false;

    /** Offline annotations only; never supplies observations to online belief. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Dataset")
    bool bEnableYoloDatasetExport = false;

    /** Explicit clear-air capture subset; ordinary cameras keep fog/clouds by default. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    bool bDroneCaptureClearAir = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Dataset")
    FString YoloDatasetId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Dataset")
    FString ScenarioFamilyId;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Dataset")
    FString YoloDatasetOutputDir;

    /** planner_trajectory (default) or scripted_coverage (offline export only). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Dataset")
    FString YoloViewSource = TEXT("planner_trajectory");

    /** Only the first N registered drones get real SceneCapture/YOLO frames.
     *  Other drones keep using the configured deterministic sensor directly. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "0"))
    int32 NumRealCameraDrones = 1;

    /** Each drone captures when (Tick + DroneIndex) % this == 0 — staggered so
     *  at most ~one capture renders per sim tick. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "1"))
    int32 DroneCaptureEveryNTicks = 10;

    /** Square capture resolution (sized for the vision model, e.g. 640 for YOLO). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "16"))
    int32 DroneCaptureResolution = 640;

    /** Versioned sensor appearance. Fast profiles require separate experiment/cache identities. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    FString CameraRenderProfile = TEXT("legacy");

    /** Camera pitch relative to the drone (-90 = straight down). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "-90.0", ClampMax = "0.0"))
    float DroneCapturePitchDegrees = -60.0f;

    /** Optional adaptive exposure for look development. Leave disabled for
     *  repeatable sensor captures, which use fixed exposure compensation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    bool bDroneCaptureAutoExposure = false;

    /** Exposure compensation in EV (lower = darker). Used in auto and manual. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    float DroneCaptureExposureBias = 0.0f;

    /** Fixed physical exposure reference. Compensation is applied on top. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "-4.0", ClampMax = "20.0"))
    float DroneCaptureManualEV100 = 12.0f;

    /** Bloom intensity for captures; the old default (1.65) washed images out. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "0.0"))
    float DroneCaptureBloomIntensity = 0.15f;

    /** Dump every capture as PNG to Saved/Captures/ (slow — inspection only). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    bool bSaveDroneCapturePngs = false;

    /** Send captures to Tools/vision_server.py and convert its detections to
     *  person observations (server + capturing drone's local belief). Requires
     *  bEnableDroneCaptures. In Replay cache mode no server is needed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    bool bEnableVisionBridge = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    FString VisionServerHost = TEXT("127.0.0.1");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "1", ClampMax = "65535"))
    int32 VisionServerPort = 8765;

    /** Record = live server + write cache; Replay = cache only (deterministic —
     *  use for benchmark runs); Off = live server, no cache. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    EVisionCacheMode VisionCacheMode = EVisionCacheMode::Record;

    /** Cache folder under Saved/VisionCache/. Empty = auto "seed<S>_<W>x<H>". */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    FString VisionCacheDirName;

    /** Save raw + YOLO-box + expected-ground-truth PNGs for YOLO presence errors:
     *  total misses, partial misses, and false-positive frames. Needs live
     *  pixels (Record/Off cache mode); Replay keeps counts. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision")
    bool bSaveYoloDebugImages = true;

    /** Cap on saved presence-error debug frame triplets per run (0 = unlimited). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "0"))
    int32 YoloMaxErrorImages = 200;

    /** Cap on saved frames with at least one true-positive YOLO person box.
     *  0 = disabled; intended for smoke inspection, not benchmark sweeps. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "0"))
    int32 YoloMaxCorrectImages = 0;

    /** Minimum YOLO confidence for a box to count as a person claim. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float YoloPersonConfidenceThreshold = 0.25f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Drones")
    TArray<TObjectPtr<ADrone>> RegisteredDrones;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Drones",
              meta = (ClampMin = "1", DisplayName = "Number of Drones",
                      ToolTip = "Target total drone count for the simulation. On BeginPlay, if fewer drones than this are present in the level, the remaining ones are auto-spawned in a ring around the grid centre (requires Auto Spawn Drones). Drones placed manually count toward this total."))
    int32 AutoDroneCount = 4;

    /** Optional benchmark identity mapped by dense registered slot. Empty preserves the historical slot IDs. */
    TArray<int32> BenchmarkDroneLogicalIds;

    /** Optional initial cell-coordinate positions mapped by dense registered slot. Empty uses the historical ring. */
    TArray<FVector2D> BenchmarkDroneSpawnCellPositions;

    /** Optional direct ideal-witness metric for sensor-only control runs. */
    bool bBenchmarkRecordIdealSensorWitnesses = false;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Drones",
              meta = (DisplayName = "Auto Spawn Drones",
                      ToolTip = "If true, BeginPlay auto-spawns extra drones so total reaches Number of Drones. Disable to use only manually-placed drones."))
    bool bAutoSpawnDronesIfEmpty = true;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Drones", meta = (ClampMin = "0.0"))
    float AutoDroneSpawnRadiusCells = 8.0f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Drones")
    float AutoDroneSpawnZ = 5200.0f;

    /** Hard safety floor above generated terrain. Drones are clamped to at
     *  least this AGL before sensing/capture to avoid terrain/canopy clipping. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Movement", meta = (ClampMin = "0.0"))
    float DroneMinimumAltitudeAboveGroundUU = 4200.0f;

    /** Nominal flight altitude above generated terrain. Movement follows a
     *  terrain-relative height high enough for mature European canopy. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Movement", meta = (ClampMin = "0.0"))
    float DroneCruiseAltitudeAboveGroundUU = 5200.0f;

    /** Camera offset in front of the drone root. Keeps the spectator/capture
     *  camera out of placeholder meshes on Blueprint drone actors. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "0.0"))
    float DroneCameraForwardOffsetUU = 140.0f;

    /** Camera offset below the drone root. The capture hides the owner actor,
     *  but this also keeps the user-facing view out of the placeholder sphere. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Vision", meta = (ClampMin = "0.0"))
    float DroneCameraDownOffsetUU = 80.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Drones")
    TSubclassOf<ADrone> AutoDroneClass;
    // ---------- Centralized per-drone settings ----------
    // These override the matching settings on every registered/spawned drone.
    // Configure drone behavior here on the SimulationManager so experiments do not
    // depend on per-actor defaults accidentally left in the level.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Movement", meta = (ClampMin = "0.0"))
    float DroneSpeedUUPerTick = 200.0f;

    /** Optional CPU-only parallel drone phase for benchmark A/B timing. The
     *  original sequential path remains the baseline and is used automatically
     *  when the active planner/sensor cannot safely run in the worker path.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Performance")
    bool bParallelDroneProcessing = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor", meta = (ClampMin = "1.0", ClampMax = "360.0"))
    float DroneHorizontalFOVDegrees = 90.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor", meta = (ClampMin = "1.0"))
    float DroneMaxViewDistanceUU = 2000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor")
    TSubclassOf<UDroneSensorBase> DroneSensorClass;

    /** Planner strategy pushed to every drone. Empty = UGreedyFovPlanner
     *  (built-in FOV-aware greedy planner — current behaviour). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    TSubclassOf<UDronePlannerBase> DronePlannerClass;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    FString PlannerStrategyName = TEXT("GreedyFov");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor|Perfect", meta = (ClampMin = "0.0"))
    float SensorFirePositiveLLR = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor|Perfect", meta = (ClampMax = "0.0"))
    float SensorFireNegativeLLR = -0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor|Perfect", meta = (ClampMin = "0.0"))
    float SensorPersonPositiveLLR = 2.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor|Perfect", meta = (ClampMax = "0.0"))
    float SensorPersonNegativeLLR = -0.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "1"))
    int32 DroneReplanCadenceTicks = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "0.0"))
    float DroneCommitmentMargin = 0.20f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "0.0"))
    float DroneDistancePenaltyPerCell = 0.02f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "1"))
    int32 DroneTopKPathBonusCandidates = 32;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "1.0"))
    float DroneArrivalRadiusUU = 150.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "1"))
    int32 DroneFallbackReplanTicks = 25;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "0.0"))
    float DroneDetourBudgetFraction = 0.20f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    bool bDroneUseFastRoutePlanner = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "0.0"))
    float DroneFastRouteFOVGainWeight = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    FString DronePlannerScoringMode = TEXT("full_fov");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "0.0"))
    float DroneNeighborPriorityWeight = 0.10f;

    // Drone-local GPU priority is an async, fence-timed cache and is NOT
    // bit-deterministic (the consumed result depends on GPU/fence timing, i.e. the
    // wall clock). For deterministic execution it must be OFF for any benchmark /
    // determinism run, so it defaults to false. It is also force-disabled whenever
    // parallel drone processing is active (see ApplyManagerConfigurationToDrone),
    // because worker threads cannot consume the readback and the sequential-vs-
    // parallel results must match. Enable only for interactive visualisation.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|GPU Planning")
    bool bDroneUseLocalGpuPriority = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|GPU Planning", meta = (ClampMin = "1"))
    int32 DroneLocalPriorityRequestIntervalTicks = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|GPU Planning", meta = (ClampMin = "0"))
    int32 DroneMaxLocalPriorityCacheAgeTicks = 8;

    // ---------- Centralized priority/partition model settings ----------
    // These replace the old compile-time ThesisPriority constants for drone planning
    // and CPU partitioning where practical. Server debug shaders still use their own
    // shader-side constants unless you update those passes separately.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Drone", meta = (ClampMin = "0.0"))
    float PriorityWeightStaleness = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Drone", meta = (ClampMin = "0.0"))
    float PriorityWeightDanger = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Drone", meta = (ClampMin = "0.0"))
    float PriorityWeightPerson = 0.30f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Drone", meta = (ClampMin = "1.0"))
    float PriorityStalenessSaturationTicks = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Drone", meta = (ClampMin = "0.0"))
    float PriorityProbabilityScale = 10.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Drone", meta = (ClampMin = "1.0"))
    float PriorityMonitoringUrgencyTau = 50.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Drone", meta = (ClampMin = "0.0"))
    float PriorityUnseenExplorationBonus = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Fire", meta = (ClampMin = "0.0"))
    float PriorityFireRampMultiplier = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Fire", meta = (ClampMin = "1.0"))
    float PriorityFireMonitoringUrgencyTau = 12.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Fire", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float PriorityFireMinimumUrgency = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Fire", meta = (ClampMin = "0"))
    int32 PriorityFireDiscoveryDelayTicks = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Fire", meta = (ClampMin = "1"))
    int32 PriorityFireDiscoveryRampTicks = 12;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Person", meta = (ClampMin = "0"))
    int32 PriorityPersonDiscoveryDelayTicks = 30;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Person", meta = (ClampMin = "1"))
    int32 PriorityPersonDiscoveryRampTicks = 50;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "0.0"))
    float LoadWeightStaleness = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "0.0"))
    float LoadWeightDanger = 350.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "0.0"))
    float LoadWeightPerson = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "0.0"))
    float ConfirmedDetectionThreshold = 1750.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "1"))
    int32 PartitionWeightConfirmed = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "1"))
    int32 PartitionWeightExpected = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "1"))
    int32 PartitionWeightNormal = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "0.001", ClampMax = "1.0"))
    float BackgroundTileFraction = 0.0666667f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Priority Model|Partition", meta = (ClampMin = "0.001", ClampMax = "1.0"))
    float MaxFireZoneFraction = 0.2f;

    // ---------- Fire CSV generator ----------

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator")
    bool bGenerateFireCsvOnBeginPlay = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator")
    bool bLoadGeneratedFireCsvAfterRun = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Benchmark")
    bool bReuseGeneratedScenarioCsvs = false;

    /** Leave empty/Auto to use Unreal's bundled Python first, then py/python from PATH.
     *  On Windows this avoids the Microsoft Store python alias when no real Python is installed.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (DisplayName = "Python Executable (empty = auto)"))
    FString PythonExecutablePath = TEXT("");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator")
    FFilePath FireGeneratorScriptPath;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator")
    FFilePath GeneratedFireCsvOutputPath;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator")
    int32 FireGeneratorSeed = 27;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "1"))
    int32 FireGeneratorIgnitionCount = 1;

    /** Benchmark alias for ignition count. 0 = use FireGeneratorIgnitionCount. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0"))
    int32 FireGeneratorNumFocusPoints = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0", ClampMax = "2"))
    int32 FireGeneratorSpreadVariant = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator",
              meta = (ClampMin = "0.0", ToolTip = "Minimum distance in grid cells between initial ignition points. 0 disables forced separation. If too strict, the Python generator gradually relaxes it."))
    float FireGeneratorMinIgnitionDistance = 0.0f;

    /** Percentage of VEGETATED cells (fuel above the barrier) that should burn,
     *  not of the whole grid. The generator logs the achieved fraction. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0", ClampMax = "100.0"))
    float FireGeneratorBurnPercentage = 14.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.001"))
    float FireGeneratorStartMin = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0"))
    float FireGeneratorStartMax = 8.0f;

    /** First ignition starts on this simulation tick. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "1"))
    int32 FireGeneratorFirstIgnitionTick = 1;

    /** Earliest start tick for ignition points after the first. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "1"))
    int32 FireGeneratorAdditionalIgnitionStartMinTick = 250;

    /** Latest start tick for ignition points after the first. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "1"))
    int32 FireGeneratorAdditionalIgnitionStartMaxTick = 1500;

    /** Fire time horizon in minutes. 0 = auto-scale to grid size (recommended):
     *  the script sizes it so the fire can cross the longer grid axis. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0"))
    float FireGeneratorMaxMinutes = 0.0f;

    /** If MaxSimTicks is 0, generated fire auto-scales to expected burn size and
     *  is capped by this value. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "1"))
    int32 FireGeneratorDefaultDurationTicks = 50000;

    /** No-MaxSimTicks auto-duration target: approximate ticks per generated
     *  burning cell, capped by FireGeneratorDefaultDurationTicks. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "1.0"))
    float FireGeneratorAutoDurationTicksPerBurningCell = 12.0f;

    /** Latest generated ignition occurs near this fraction of the chosen duration. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    float FireGeneratorDurationFillFraction = 0.95f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.001"))
    float FireGeneratorSpreadMinutesPerCell = 40.0f;

    /** Preferred spread-rate knob: nominal simulation ticks to traverse one cell. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.001"))
    float FireGeneratorSpreadTicksPerCell = 15.0f;

    /** Front acceleration as the burning region grows. 0 = constant spread rate. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0"))
    float FireGeneratorGrowthAcceleration = 2.5f;

    /** How strongly terrain movement cost slows the generated fire front. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0"))
    float FireGeneratorTerrainCostInfluence = 0.30f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator")
    bool bFireGeneratorUseRandomWindDirection = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0", ClampMax = "360.0", EditCondition = "!bFireGeneratorUseRandomWindDirection"))
    float FireGeneratorWindAngleDegrees = 35.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float FireGeneratorWindStrength = 0.65f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float FireGeneratorSpottingChance = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float FireGeneratorMoisture = 0.22f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Generator", meta = (ClampMin = "0"))
    int32 FireGeneratorPrecision = 2;

    // ---------- Debug visualization ----------

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    bool bAutoPositionCamera = true;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    float CameraOverviewHeightUU = 2500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    bool bDrawGridBounds = true;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    bool bDrawDrones = true;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    float GridDebugHeightUU = 2000.0f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    float DroneDebugRadiusUU = 150.0f;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    bool bDrawCellOwnership = true;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug", meta = (ClampMin = "1"))
    int32 CellOwnershipDisplayStep = 1;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    EDebugPrioritySource DebugPrioritySource = EDebugPrioritySource::None;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    int32 DebugPriorityDroneIndex = 0;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug", meta = (ClampMin = "1"))
    int32 DebugPriorityDisplayStep = 2;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    float DebugPriorityZOffset = 200.0f;
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug", meta = (ClampMin = "1"))
    int32 DebugPriorityReadbackInterval = 10;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Thesis|Debug")
    bool bShowPriorityNumbers = true;

    UFUNCTION(BlueprintCallable, Category = "Thesis|Drones")
    void RegisterDrone(ADrone* Drone);

    /** Pushes the manager-owned drone/sensor/planner settings to every registered drone.
     *  Useful after editing manager values during PIE. ResetSimulation() also calls this.
     */
    UFUNCTION(BlueprintCallable, Category = "Thesis|Drones")
    void ApplyManagerSettingsToAllDrones();
    UFUNCTION(BlueprintPure, Category = "Thesis|Debug")
    int32 DebugGetOwner(int32 X, int32 Y) const;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaSeconds) override;

private:
    struct FCameraAuditCaptureState
    {
        int32 CaptureTick = INDEX_NONE;
        uint64 EngineFrame = 0;
        bool bRendered = false;
        FTransform CameraTransform = FTransform::Identity;
        float FOVDegrees = 0.0f;
        int32 Width = 0;
        int32 Height = 0;
        FString ProjectionStateJson;
    };

    bool IsCameraStateAuditEnabled() const;
    void BeginCameraStateAudit();
    void RecordCameraCaptureAudit(const ADrone* Drone, const UDroneCaptureComponent* Capture,
                                 int32 CaptureTick, bool bRendered);
    void RecordCameraConsumeAudit(const ADrone* Drone, const UDroneCaptureComponent* Capture,
                                  int32 CaptureTick, int32 ConsumeTick, bool bReplay,
                                  const TArray<FColor>* Pixels, int32 Width, int32 Height);
    bool WriteCameraStateAuditCsv(const FString& Text);
    void FailCameraStateAudit(const FString& Reason);
    void RecordGpuStateAudit(UGpuBeliefManager* BeliefManager, int32 SourceTick);
    FString CameraStateAuditDirectory;
    FString CameraStateAuditCsvPath;
    TUniquePtr<FArchive> CameraStateAuditCsvWriter;
    bool bCameraStateAuditFailed = false;
    FString CameraStateAuditFailure;
    TMap<int32, FCameraAuditCaptureState> CameraAuditCaptureStates;

    TUniquePtr<FYoloDatasetExporter> DatasetExporter;
    bool InitializeDatasetExport();
    bool ExportDatasetCapture(UDroneCaptureComponent* Capture);
    bool SetupScriptedDatasetView(UDroneCaptureComponent* Capture, int32 Tick, int32 DroneIndex,
                                  FTransform& OutOriginalTransform, float& OutOriginalFov);
    bool CompleteDatasetExport();
    bool FailDatasetExport(const FString& Error);
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Simulation", meta = (AllowPrivateAccess = "true"))
    int32 CurrentTick = 0;
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Simulation", meta = (AllowPrivateAccess = "true"))
    ESimState State = ESimState::Stopped;

    TUniquePtr<Grid> WorldGrid;

    // V2 coverage ledger is written only in the server slot and immutable during
    // all worker tasks. Completed waypoints survive ownership handovers.
    TArray<uint8> LawnmowerV2CompletedCells;
    int32 LawnmowerV2CompletedLaneCells = 0;
    int32 LawnmowerV2CoverageEpoch = 0;
    void UpdateLawnmowerV2Progress();

    void ApplyPartitionOwnersToGrid(const TArray<uint16>& Owners);
    void ApplyManagerConfigurationToDrone(ADrone* Drone) const;
    bool IsDroneVisionCameraEnabled(int32 DroneIdx) const;
    void ApplyViewVisualMode();
    bool ProjectRayToTerrain(const FVector& Origin, const FVector& Dir, FVector& OutHit) const;
    FString ResolveProjectRelativePath(const FFilePath& Path, const FString& DefaultRelativePath) const;

    // Delegates to the active IPartitionStrategy (selected by PartitionStrategyName).
    // The default FZoneRegionGrowingPartition is the adaptive-zone + home-zone-seeded
    // region-growing algorithm previously implemented inline here — moved verbatim to
    // Partition/ZoneRegionGrowingPartition.cpp. Returns a flat owner array ready for
    // ApplyPartitionOwnersToGrid.
    TArray<uint16> ComputeBalancedPartition(const TArray<TObjectPtr<ADrone>>& ActiveDrones,
                                            const TArray<FVector2f>& DronePositionsCellSpace,
                                            int32 Tick);

    /** Instantiates/replaces the partition strategy to match PartitionStrategyName. */
    void EnsurePartitionStrategy();

    /** Resolves PlannerStrategyName to DronePlannerClass. */
    void EnsurePlannerStrategy();

    /** Builds FPartitionParams from the manager's current tunables. */
    FPartitionParams MakePartitionParams() const;

    /** Records one tick's metrics (no-op when the recorder is inactive). */
    void RecordTickMetrics(float TickComputeMs);
    void RecordWallClockPhaseTiming(int32 Tick, const TCHAR* PhaseName, double StartSeconds);
    void FlushPendingWallClockPhaseTimings();
    void RecordDroneTickProfileTimings(int32 Tick, const TCHAR* Prefix, const TArray<FDroneTickProfile>& Profiles);
    float ComputeMeanBeliefError() const;
    bool CanUseParallelDroneProcessing() const;
    void RecordDroneProcessingModeEvent();
    void TickDronesSequential(int32 CurrentTick);
    void TickDronesParallelCpu(int32 CurrentTick);

    TUniquePtr<IPartitionStrategy> PartitionStrategy;
    FSimMetricsRecorder MetricsRecorder;
    FMethodSpec ActiveMethodSpec;

    /** Finds or spawns the AHumanManager and loads HumansCSVPath (no-op if empty). */
    void EnsureHumanManager();

    /** Finds the level's AForestDresser (if any) and (re)builds the forest +
     *  floraDensity. Must run before drones snapshot floraDensity. */
    void BuildForestIfPresent();

    /** Writes the grid's floraDensity as CSV — the fire generator's fuel map. */
    void ExportFuelCsvFromGrid();

    /** Finds or spawns the fire-visuals actor and (re)initialises it from the
     *  current fire plan. Call after the fire CSV is imported. */
    void SetupFireVisuals();
    bool IsFireEditorPreviewEnabled() const;

    UPROPERTY()
    TObjectPtr<AFireVisualsManager> FireVisualsActor;

    /** Separate preview history, excluded from sensor captures. */
    UPROPERTY(Transient)
    TObjectPtr<AFireVisualsManager> EditorFirePreviewActor;

    /** Server-slot vision step: for every unprocessed capture, get detections
     *  (live or cache), project them to grid cells, and feed both the server
     *  observation queue and the capturing drone's local belief. */
    bool ProcessVisionCaptures();
    bool WriteCachedDetectionChanceNull();

    /** Finds or spawns the AProceduralTerrain, builds it for the current grid,
     *  and exports the movement-cost CSV. No-op when bAutoGenerateTerrain is
     *  off and no terrain actor exists in the level. */
    void BuildTerrainIfEnabled();

    UPROPERTY()
    TObjectPtr<AProceduralTerrain> TerrainActor;

    /** Projects a normalized image point (U right, V down) from a drone capture
     *  camera onto the ground plane. Returns cell index or INDEX_NONE. */
    int32 ProjectImagePointToCell(const class UDroneCaptureComponent* Capture, float U, float V) const;
    int32 ProjectImagePointToCell(const FTransform& CameraTransform, float CameraFOV,
                                 float U, float V, bool& bOutOfGrid) const;
    /** Projects a world-space point into a drone capture. Returns false if it is outside the image. */
    bool ProjectWorldPointToImage(const class UDroneCaptureComponent* Capture, const FVector& WorldPoint,
                                  float& OutU, float& OutV) const;

    /** True if CellIndex or any 8-neighbour holds a ground-truth person. */
    bool CellOrNeighborsHaveRealPerson(int32 CellIndex) const;
    /** Cell centres geometrically inside a capture frustum; returns projected alive GT anchors.
     *  This does not account for terrain/building occlusion. */
    int32 CollectCaptureFootprint(const class UDroneCaptureComponent* Capture, TArray<int32>& OutCells) const;
    /** Writes <RunDir>/vision/<Tag>_droneN_tT_{raw,annotated,expected}.png. */
    void SaveYoloImagePair(const TCHAR* Tag, int32 DroneIndex, int32 Tick, int32 W, int32 H,
                           const TArray<FColor>& Pixels, const TArray<FVisionDetection>& PersonDets,
                           const TArray<bool>& IsTruePositive,
                           const class UDroneCaptureComponent* Capture) const;

    TUniquePtr<class FVisionBridge> VisionBridge;
    TArray<int32> VisionProcessedTick;   // per drone: last capture tick fed to vision
    TArray<TArray<ChanceNullScorer::FFrame>> CachedDetectionNullFrames;
    bool bCachedDetectionNullProjectionMismatch = false;
    bool bSensorSkyPrepared = false;
    bool bSensorTexturesPrepared = false;
    TMap<TWeakObjectPtr<class UTexture2D>, bool> SensorTextureResidency;
    TMap<TWeakObjectPtr<class USceneComponent>, bool> SensorOriginalVisibility;
    TMap<TWeakObjectPtr<class UDirectionalLightComponent>, bool> SensorOriginalAtmosphereSun;
    TMap<TWeakObjectPtr<class USkyLightComponent>, bool> SensorOriginalRealtimeSky;
    TMap<TWeakObjectPtr<class USkyLightComponent>, int32> SensorOriginalSkySource;
    UPROPERTY(Transient)
    TMap<TObjectPtr<class USkyLightComponent>, TObjectPtr<class UTextureCube>> SensorOriginalSkyCubemap;
    void RestoreSensorResources();
    void NormalizeEnvironmentLighting();
    friend class FEnvironmentLightingLifecycleTest;
    TSharedPtr<class FLiveViewPresentationExtension, ESPMode::ThreadSafe> LiveViewPresentation;
    int32 YoloErrorImagePairsSaved = 0;      // debug frames counted toward YoloMaxErrorImages
    int32 YoloCorrectImagePairsSaved = 0;    // debug frames counted toward YoloMaxCorrectImages

    UPROPERTY()
    TObjectPtr<AForestDresser> ForestDresserActor;

    /** Per-tick person-detection check against server CPU belief (primary metric).
     *  O(persons) — no grid scan. */
    void UpdatePersonDetectionMetrics();

    /** -1 = not yet detected this run. Reset when a metrics run starts. */
    TArray<int32> PersonFirstDetectedTick;

    /** Recomputed once per simulation tick before any oracle drone moves. */
    void UpdateOracleAssignments(int32 AtTick);
    void RebuildOracleTrajectoryCache();

    TArray<TArray<FHumanKeyframe>> OracleTrajectoryByPerson;
    TArray<int32> OracleAssignedCellByDrone;
    TArray<float> OracleAssignedScoreByDrone;
    int32 OracleCachedKeyframeCount = -1;
    int32 OracleAssignmentTick = INDEX_NONE;
    int32 OracleFirstFireCell = INDEX_NONE;
    int32 OracleFirstFireTick = INDEX_NONE;

    UPROPERTY()
    TObjectPtr<AHumanManager> HumanManager;

    void DrawDebugVisuals() const;
    void DrawPriorityHeatmap() const;

    void SetupDebugKeyBindings();
    void OnKey_One();
    void OnKey_Two();
    void OnKey_Three();
    void OnKey_Zero();
    void OnKey_Tab();
    void OnKey_Four();
    void OnKey_Space();
    void OnKey_V();
    void OnKey_LeftBracket();
    void OnKey_RightBracket();

    // INDEX_NONE = overview; otherwise index into RegisteredDrones.
    int32 ViewedDroneIndex = INDEX_NONE;
    // Drone V returns to after toggling back from overview.
    int32 LastViewedDroneIndex = 0;

    FVector GridOriginWorld = FVector::ZeroVector;

    float TickAccumulator = 0.0f;

    // Measured sim rate (ticks advanced per wall second) for the debug HUD.
    int32 TicksThisSecond = 0;
    float TickRateWindowSeconds = 0.0f;
    float MeasuredTicksPerSecond = 0.0f;

    // Server priority readback cache
    TArray<float> CachedServerPriority;
    int32 CachedServerPriorityTick = -1;
    int32 TicksSinceLastPriorityReadback = 9999;
    TArray<FObservation> PendingObservations;
    TArray<ADrone*> ParallelDroneScratchDrones;
    TArray<FVector> ParallelDroneScratchLocations;
    TArray<FDroneParallelTickResult> ParallelDroneScratchResults;

    TArray<FPendingWallClockPhaseTiming> PendingWallClockPhaseTimings;
    bool bViewportSuppressionApplied = false;
    bool bPreviousDisableWorldRendering = false;
};
