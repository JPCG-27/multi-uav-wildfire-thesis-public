#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FireVisualsManager.generated.h"

class Grid;
class AProceduralTerrain;
class AForestDresser;
class UNiagaraSystem;
class UNiagaraComponent;
class UParticleSystem;
class UParticleSystemComponent;
class UPointLightComponent;
class UFireVisualProfile;
class UDeterministicCaptureEffectsComponent;

/**
 * Visual representation of the burning fire front: flame + smoke VFX and
 * flickering orange point lights ("changes of light caused by the flames").
 *
 * How it stays cheap and deterministic:
 *  - At Init, the firstFireTick array is converted ONCE into sorted visual
 *    ignition/scorch events. Flames persist while simulation fire persists;
 *    authoritative simulation fire is never written by this actor.
 *  - A fixed-size pool of VFX/light components is assigned to a deterministic
 *    subset of burning cells (every Nth burning cell, ascending cell index),
 *    so hundreds of burning cells render with a bounded budget.
 *  - All updates run in the server slot via ApplyTick; flicker is a pure
 *    function of (tick, cell) — no wall clock.
 *
 * VFX slots accept either Niagara or legacy Cascade systems, so any free pack
 * works (e.g. "FX Variety Pack" on Fab, or Starter Content P_Fire / P_Smoke).
 * With no system assigned you still get procedural flame/smoke markers plus
 * the dynamic fire lights.
 */
UCLASS(Blueprintable)
class THESIS_API AFireVisualsManager : public AActor
{
    GENERATED_BODY()

public:
    AFireVisualsManager();

    /** Optional data-driven presentation profile. No simulation state lives here. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    TObjectPtr<UFireVisualProfile> VisualProfile;

    // ---------- VFX assets (assign any free pack; all optional) ----------

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    TObjectPtr<UNiagaraSystem> FlameSystem;

    /** Optional second flame layer. If empty, FlameSystem is reused at a
     *  different scale/offset so the fire still has a core + outer structure. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    TObjectPtr<UNiagaraSystem> OuterFlameSystem;

    /** Optional ember/spark layer. NiagaraExamples' continuous spark system is
     *  auto-loaded when available; otherwise this layer is simply skipped. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    TObjectPtr<UNiagaraSystem> EmberSystem;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    TObjectPtr<UNiagaraSystem> SmokeSystem;

    /** Legacy Cascade alternatives (e.g. Starter Content P_Fire / P_Smoke). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    TObjectPtr<UParticleSystem> FlameSystemCascade;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    TObjectPtr<UParticleSystem> SmokeSystemCascade;

    /** If no VFX assets are assigned, try to load bundled Niagara Fluids /
     *  Starter Content fire and smoke systems. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    bool bAutoLoadDefaultVfxAssets = true;

    /** If no Niagara/Cascade assets are available, create cheap deterministic
     *  procedural mesh flame/smoke markers so the fire is still visible in PIE. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Assets")
    bool bUseProceduralFallbackVfx = true;

    /** Performance mode: defer VFX asset/pool creation until the YOLO-connected
     *  drone is about to render a capture, and deactivate everything right after
     *  that capture. Fire truth/scorch still advances every tick. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bLoadVfxOnlyForYoloCapture = true;

    /** In live PIE preview, advance tick-driven Niagara particles by elapsed
     *  deterministic steps instead of rebuilding their full history every
     *  simulation tick. Benchmark capture reuse is controlled separately. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bIncrementalNiagaraPreviewStepping = false;

    /** Opt-in deterministic sensor-capture mode with fewer canonical Niagara
     *  simulation steps. This changes particle appearance, but keeps each
     *  capture reproducible from its simulation tick. Disabled by default. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bFastDeterministicCapture = false;

    /** Use the deterministic lightweight shader fire effects pool (fast_shader_v3). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bUseFastShaderV3 = false;

    /** Advance a paused deterministic Niagara state by fixed steps when the
     *  assignment, transform and inputs match; reconstruct on loop wraps.
     *  Intended for isolated benchmark profiles only. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bReuseDeterministicCaptureState = false;

    /** Use ordinary wall-clock Niagara ticking for the isolated live editor
     *  preview actor. The canonical manager used by drone captures must leave
     *  this false and retain tick-derived age/reconstruction. */
    bool bUseRealtimeNiagaraForPreview = false;

    /** Keep this actor's primitive VFX out of every scene capture. Intended for
     *  editor preview actors that must not perturb sensor captures. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bHideFromSceneCaptures = false;

    // ---------- Budget / behaviour ----------

    /** Flickering orange point lights at burning cells (works with no assets). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bFireLights = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0", ClampMax = "256"))
    int32 MaxFireVfx = 128;

    /** Each selected burning cell is rendered as several small, ground-anchored
     *  fire clusters instead of one oversized VFX blob. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Cluster", meta = (ClampMin = "1", ClampMax = "8"))
    int32 FireClustersPerCell = 4;

    /** Radius used for deterministic fallback cluster placement inside a cell,
     *  and for small offsets around tree-base anchors. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Cluster", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float FireClusterRadiusUU = 190.0f;

    /** Prefer one overlapping flame support per active cell before allocating
     *  secondary campfire detail clusters. The sprite footprint scales with
     *  cell size but remains bounded by the shared Niagara pool. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Cluster")
    bool bConnectedFireFront = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0", ClampMax = "64"))
    int32 MaxFireLights = 12;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "100.0"))
    float LightRadiusUU = 3000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0"))
    float LightIntensity = 1200000.0f;

    /** Fire-light shadows are opt-in: large non-Nanite forests make them costly. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0", ClampMax = "16"))
    int32 MaxShadowCastingFireLights = 0;

    /** Reassign pooled VFX to the moving fire front every N sim ticks. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "1"))
    int32 ReassignEveryNTicks = 1;

    /** Scale VFX with the cell's fuel (bigger flames in dense forest). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bScaleWithFuel = true;

    /** Flame scale multiplier. Keep this modest: large flame systems look like
     *  they float because their particle origin stays above uneven terrain. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "20.0"))
    float VfxScaleMultiplier = 0.62f;

    /** Non-uniform component scale for flame tongues. A wider footprint fills
     *  gaps between deterministic ground clusters; the vertical multiplier
     *  makes the same pooled particles climb higher without adding systems. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.1", ClampMax = "5.0"))
    float FlameWidthScaleMultiplier = 1.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.1", ClampMax = "5.0"))
    float FlameHeightScaleMultiplier = 4.0f;

    /** Resize authored flame quads; component transforms only scale their centres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.1", ClampMax = "10.0"))
    float BundledFlameSpriteSizeMultiplier = 2.4f;

    /** Refraction adds a scene-color distortion pass; opt in for close-up shots. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bEnableBundledHeatDistortion = false;

    /** Calibrates the bundled Vefects flame materials for daylight exposure.
     *  Applied to private runtime copies only; smoke materials are excluded. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "1.0", ClampMax = "65536.0"))
    float BundledFlameEmissiveMultiplier = 4096.0f;

    /** Secondary orange/red flame layer scale relative to the core. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "20.0"))
    float OuterFlameScaleMultiplier = 0.90f;

    /** Ember/spark layer scale. Kept modest so sparks read as detail, not fog. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "20.0"))
    float EmberScaleMultiplier = 0.75f;

    /** Smoke scale multiplier. Wildfire smoke should be much larger than the
     *  visible flame tongues. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.01", ClampMax = "30.0"))
    float SmokeVfxScaleMultiplier = 1.20f;

    /** A small number of overlapping smoke systems can thicken the source, but
     *  high values make the smoke look like a flat beige blanket. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "1", ClampMax = "8"))
    int32 SmokeLayerCount = 2;

    /** Share the configured smoke layers across clusters in the same burning
     *  cell. This bounds smoke simulation and overdraw while keeping the flame
     *  clusters independently distributed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Performance")
    bool bShareSmokeAcrossCellClusters = true;

    /** Deterministic horizontal spread for extra smoke layers. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float SmokeLayerRadiusUU = 260.0f;

    /** Vertical staggering between smoke layers so the plume starts dense near
     *  the source but also climbs like a wildfire column. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float SmokeLayerZStepUU = 240.0f;

    /** Upper smoke layers drift downwind even for VFX assets without a wind
     *  parameter. Zero restores vertically stacked sources. Presentation only. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals|Cluster", meta = (ClampMin = "0.0", ClampMax = "10000.0"))
    float SmokeMaxWindDriftUU = 2500.0f;

    /** Lift flame systems slightly above the terrain so floor-oriented effects
     *  do not clip into uneven ground or low shrubs. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "-1000.0", ClampMax = "5000.0"))
    float FlameZOffsetUU = 0.0f;

    /** Smoke starts close to the burning vegetation. Extra layers climb upward. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "-1000.0", ClampMax = "8000.0"))
    float SmokeZOffsetUU = 260.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "-1000.0", ClampMax = "5000.0"))
    float LightZOffsetUU = 120.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "5000.0"))
    float LightSourceRadiusUU = 650.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "0.0", ClampMax = "20.0"))
    float LightVolumetricScatteringIntensity = 8.0f;

    /** Render fire VFX only in drone captures (lights always render). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bVfxOnlyInDroneCaptures = false;

    /** Scorched-ground feedback: at the configured scorch deadline, darken the
     *  terrain and notify the forest presentation layer. Visual only — reads ground truth (allowed; the
     *  belief wall applies to sensors, not visuals). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals")
    bool bScorchedGroundFeedback = true;

    /** Terrain scorch colors upload every N ticks (full vertex-buffer rewrite —
     *  keep the cadence modest). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Fire Visuals", meta = (ClampMin = "1"))
    int32 ScorchFlushEveryNTicks = 15;

    // ---------- Lifecycle (called by ASimulationManager) ----------

    /** Precomputes ignite events from the grid's firstFireTick. */
    void Init(const Grid* WorldGrid, const FVector& GridOriginWorld, float CellSizeUU,
              int32 BurnDurationTicks, AProceduralTerrain* Terrain, AForestDresser* Forest);

    /** Advances the burning set to Tick and updates the VFX/light pool. */
    void ApplyTick(int32 Tick);

    /** Advances only the cheap fire state/scorch bookkeeping. */
    void AdvanceFireState(int32 Tick);

    /** Activates nearby pooled VFX/lights for one explicit YOLO drone capture. */
    void PrepareForYoloCapture(int32 Tick, const FVector& CaptureWorldLocation, float CaptureRadiusUU);

    /** Deactivates all pooled VFX/lights after the YOLO capture has rendered. */
    void EndYoloCapture();

    int32 GetNumBurningCells() const { return BurningCells.Num() + SmoulderingCells.Num(); }

    /** Runtime view-mode helper used by ASimulationManager. */
    void SetVfxCaptureOnly(bool bCaptureOnly);

private:
    struct FFireEvent
    {
        int32 Tick = 0;
        int32 Cell = INDEX_NONE;
    };

    TArray<FFireEvent> IgniteEvents;       // sorted by tick, then cell
    TArray<FFireEvent> ScorchEvents;     // delayed scorch; never ends active flames
    TArray<FFireEvent> SmoulderEndEvents;  // visual-only end of residual smoke
    int32 IgniteCursor = 0;
    int32 ScorchCursor = 0;
    int32 SmoulderEndCursor = 0;
    int32 LastAppliedTick = -1;

    TSet<int32> BurningCells;
    TSet<int32> SmoulderingCells;
    TArray<float> CellFuel;                // copied at Init (ground truth, static)
    TArray<int32> CellIgnitionTick;         // visual age origin; copied from fire plan

    bool bUseFiniteVisualLifetime = true;
    bool bEnableOuterFlameLayer = false;
    bool bEnableEmberLayer = true;
    bool bDriveNiagaraAgeFromSimulationTick = false;
    int32 VisualFlameDurationTicks = 1;
    int32 VisualSmoulderDurationTicks = 0;
    float VisualSecondsPerTick = 0.10f;
    float SmoulderSmokeScale = 0.70f;
    float SmoulderEmberScale = 0.35f;
    FVector2D VisualWindDirection = FVector2D(1.0f, 0.0f);
    float VisualWindHorizontalSpeed = 100.0f;
    float VisualSmokeRiseSpeed = 45.0f;
    FName NiagaraSpawnRateParameter = TEXT("User.SpawnRate");
    FName NiagaraSpawnRateScaleParameter = TEXT("User.SpawnRateScale");
    FName NiagaraIntensityParameter = TEXT("User.Intensity");
    FName NiagaraNoiseStrengthParameter = TEXT("User.NoiseStrength");
    FName NiagaraWindParameter = TEXT("User.Wind");

    int32 GridW = 0;
    int32 GridH = 0;
    FVector GridOrigin = FVector::ZeroVector;
    float CellSize = 1000.0f;
    TWeakObjectPtr<AProceduralTerrain> Terrain;
    TWeakObjectPtr<AForestDresser> Forest;

    // Pools (index-aligned: each slot shares one deterministic visual cluster).
    UPROPERTY()
    TArray<TObjectPtr<USceneComponent>> FlamePool;
    UPROPERTY()
    TArray<TObjectPtr<USceneComponent>> OuterFlamePool;
    UPROPERTY()
    TArray<TObjectPtr<USceneComponent>> EmberPool;
    UPROPERTY()
    TArray<TObjectPtr<USceneComponent>> SmokePool;
    UPROPERTY()
    TArray<TObjectPtr<UPointLightComponent>> LightPool;
    /** High-water pooled shader effects; only currently assigned slots are updated. */
    UPROPERTY()
    TArray<TObjectPtr<UDeterministicCaptureEffectsComponent>> FastShaderPool;
    TArray<int32> PoolCells;               // cell currently shown by pool slot i
    TArray<int32> PoolClusterIndices;      // sub-cluster currently shown by pool slot i
    TArray<uint8> PoolSmouldering;          // 1 when the slot is residual smoke/embers

    struct FPreviewNiagaraState
    {
        uint32 Seed = 0;
        int32 LastTick = INDEX_NONE;
        int64 LastAgeSteps = -1;
        int32 LastTargetSteps = INDEX_NONE;
        int32 Cell = INDEX_NONE;
        int32 ClusterIndex = INDEX_NONE;
        int32 LayerIndex = INDEX_NONE;
        int32 IgnitionTick = INDEX_NONE;
        bool bValid = false;
    };
    struct FCaptureNiagaraState
    {
        TWeakObjectPtr<UNiagaraSystem> Asset;
        FTransform ComponentTransform = FTransform::Identity;
        FVector Wind = FVector::ZeroVector;
        float Emission = 0.0f;
        float Intensity = 0.0f;
        float NoiseStrength = 0.0f;
        uint32 Seed = 0;
        int32 LastTick = INDEX_NONE;
        int64 LastAgeSteps = -1;
        int32 TargetSteps = INDEX_NONE;
        int32 Cell = INDEX_NONE;
        int32 ClusterIndex = INDEX_NONE;
        int32 LayerIndex = INDEX_NONE;
        int32 IgnitionTick = INDEX_NONE;
        bool bValid = false;
    };
    TMap<TWeakObjectPtr<UNiagaraComponent>, FPreviewNiagaraState> PreviewNiagaraStates;
    TMap<TWeakObjectPtr<UNiagaraComponent>, FCaptureNiagaraState> CaptureNiagaraStates;
    int32 CaptureReuseAdvances = 0;
    int32 CaptureReuseResets = 0;
    int32 CaptureReuseNoOps = 0;
    bool bCanonicalVfxReconstruction = false;

    /** Private runtime copies: source Niagara assets are never edited or saved. */
    UPROPERTY(Transient)
    TMap<TObjectPtr<UNiagaraSystem>, TObjectPtr<UNiagaraSystem>> DeterministicSystems;

    UNiagaraSystem* GetDeterministicSystem(UNiagaraSystem* Source);
    UNiagaraSystem* GetRealtimePreviewSystem(UNiagaraSystem* Source);
    void AdvanceNiagaraVisual(UNiagaraComponent* Component, int32 Tick, int32 IgnitionTick,
                             int32 Cell, int32 ClusterIndex, int32 LayerIndex,
                             bool bForceReconstruction, float EffectiveEmission,
                             float EffectIntensity, float EffectiveNoiseStrength,
                             const FVector& EffectiveWind);
    void HideVfxComponent(USceneComponent* Component);

    void EnsurePools();
    void EnsureFastShaderPool(int32 ActiveSlotCount);
    void ResetPools();
    void ReassignPool(int32 Tick, const FVector* FocusWorld = nullptr, float FocusRadiusUU = 0.0f);
    void UpdateFlicker(int32 Tick);
    void EnsureVfxReady();
    void DeactivateAllVisuals(bool bResetAssignments);
    FVector CellCenterWorld(int32 Cell) const;
    FVector GroundWorldAtCellFraction(float CellX, float CellY) const;
    FVector ClusterWorldLocation(int32 Cell, int32 ClusterIdx) const;
    USceneComponent* MakeVfxComponent(UNiagaraSystem* Niagara, UParticleSystem* Cascade, bool bFlameFallback);
    USceneComponent* MakeProceduralFallbackComponent(bool bFlame);
    void EnsureDefaultVfxAssets();
    void ApplyVisualProfile(int32 BurnDurationTicks);
    void ScorchCellVisuals(int32 Cell);
};
