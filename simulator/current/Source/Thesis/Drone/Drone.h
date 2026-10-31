#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DroneTypes.h"
#include "RHIGPUReadback.h"
#include "RenderCommandFence.h"
#include "Grid.h"
#include "../World/PriorityModel.h"
#include "Drone.generated.h"

class UDroneSensorBase;
class UDronePlannerBase;
class UDroneCaptureComponent;
class UCameraComponent;
class AProceduralTerrain;
struct FObservation;
struct FHumanSnapshot;

struct FDroneTickProfile
{
    bool bValid = false;
    bool bReplanned = false;
    int32 ObservationCount = 0;
    float ValidateMs = 0.0f;
    float SenseMs = 0.0f;
    float LocalBeliefMs = 0.0f;
    float ReplanDecisionMs = 0.0f;
    float ReplanMs = 0.0f;
    float MoveMs = 0.0f;
    float TotalMs = 0.0f;
};

struct FIdealSensorWitnessEvent
{
    int32 SensingTick = -1;
    int32 SnapshotTick = -1;
    int32 DroneIndex = INDEX_NONE;
    int32 PersonId = INDEX_NONE;
    int32 CellIndex = INDEX_NONE;
};

struct FDroneParallelTickResult
{
    bool bDidRun = false;
    bool bHasNewLocation = false;
    FVector NewLocation = FVector::ZeroVector;
    FDroneTickProfile Profile;
    TArray<FIdealSensorWitnessEvent> IdealSensorWitnessEvents;
};

UCLASS(Blueprintable)
class THESIS_API ADrone : public AActor
{
    GENERATED_BODY()

public:
    ADrone();

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Drone")
    int32 DroneIndex = INDEX_NONE;

    UFUNCTION(BlueprintPure, Category = "Thesis|Drone")
    int32 GetDroneIndex() const { return DroneIndex; }

    /** Stable experiment identity; dense DroneIndex remains the array slot. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Thesis|Drone")
    int32 LogicalDroneId = INDEX_NONE;

    UFUNCTION(BlueprintPure, Category = "Thesis|Drone")
    int32 GetLogicalDroneId() const { return LogicalDroneId == INDEX_NONE ? DroneIndex : LogicalDroneId; }

    void SetLogicalDroneId(int32 InLogicalDroneId) { LogicalDroneId = InLogicalDroneId; }

    /** Called by SimulationManager once after registration. */
    void InitFromManager(Grid* SharedGrid, int32 InDroneIndex,
                         const FVector& GridOriginWorld, float CellSizeUU);

    /** Main update – called single‑threaded by SimulationManager. */
    void TickDrone(int32 CurrentTick, bool bProfileTiming = false);

    /** CPU-only worker update used by the optional parallel drone phase. Actor
     *  transforms are snapshotted/applied by the manager on the game thread.
     */
    void TickDroneParallelCpu(int32 CurrentTick,
                              const FVector& SnapshotLocation,
                              bool bUseRandomPlanner,
                              bool bUseFullyRandomPlanner,
                              bool bUseLawnmowerPlanner,
                              int32 RunSeed,
                              bool bProfileTiming,
                              bool bRecordIdealSensorWitnesses,
                              FDroneParallelTickResult& OutResult);

    bool SupportsParallelCpuTick() const;
    void ApplyParallelCpuTickResult(const FDroneParallelTickResult& Result);
    const FDroneTickProfile& GetLastTickProfile() const { return LastTickProfile; }

    /** Inject ground-truth person snapshots for the sensor's world-space gating
     *  of positive person observations. Stored on the drone and re-applied
     *  whenever the sensor instance is (re)created, so call order relative to
     *  ApplyManagerConfiguration does not matter. Sensor-only ground truth:
     *  the planner never reads it (ground-truth/belief wall). */
    void SetPersonGroundTruth(const TArray<FHumanSnapshot>* InSnapshots);

    /** Apply all editor-facing drone/sensor/planner settings owned by ASimulationManager.
     *  This keeps runtime tuning centralized on the manager instead of per-drone actors.
     */
    void ApplyManagerConfiguration(
        float InSpeedUUPerTick,
        float InHorizontalFOVDegrees,
        float InMaxViewDistanceUU,
        int32 InReplanCadenceTicks,
        float InCommitmentMargin,
        float InDistancePenaltyPerCell,
        int32 InTopKPathBonusCandidates,
        float InArrivalRadiusUU,
        int32 InFallbackReplanTicks,
        float InDetourBudgetFraction,
        bool bInUseFastRoutePlanner,
        float InFastRouteFOVGainWeight,
        bool bInUseTargetCellScoring,
        bool bInUseDroneLocalGpuPriority,
        int32 InDroneLocalPriorityRequestIntervalTicks,
        int32 InMaxDroneLocalPriorityCacheAgeTicks,
        float InNeighborPriorityWeight,
        TSubclassOf<UDroneSensorBase> InSensorClass,
        TSubclassOf<UDronePlannerBase> InPlannerClass,
        float InFirePositiveLLR,
        float InFireNegativeLLR,
        float InPersonPositiveLLR,
        float InPersonNegativeLLR,
        float InPriorityWeightStaleness,
        float InPriorityWeightDanger,
        float InPriorityWeightPerson,
        float InPriorityStalenessSaturationTicks,
        float InPriorityProbabilityScale,
        float InPriorityMonitoringUrgencyTau,
        float InPriorityUnseenExplorationBonus,
        float InPriorityFireRampMultiplier,
        float InPriorityFireMonitoringUrgencyTau,
        float InPriorityFireMinimumUrgency,
        int32 InPriorityFireDiscoveryDelayTicks,
        int32 InPriorityFireDiscoveryRampTicks,
        int32 InPriorityPersonDiscoveryDelayTicks,
        int32 InPriorityPersonDiscoveryRampTicks,
        float InLoadWeightStaleness,
        float InLoadWeightDanger,
        float InLoadWeightPerson);

    /** Visual/vision flight frame supplied by the manager after terrain build. */
    void ConfigureTerrainFlight(AProceduralTerrain* InTerrain,
                                float InMinimumAltitudeAboveGroundUU,
                                float InCruiseAltitudeAboveGroundUU,
                                float InCameraForwardOffsetUU,
                                float InCameraDownOffsetUU);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Movement")
    float SpeedUUPerTick = 200.0f;

    // Angular sensor model. Direction convention throughout the drone code is:
    // 0=E, 1=NE, 2=N, 3=NW, 4=W, 5=SW, 6=S, 7=SE.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Thesis|Drone Config|Sensor")
    float HorizontalFOVDegrees = 90.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Thesis|Drone Config|Sensor")
    float MaxViewDistanceUU = 2000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    int32 ReplanCadenceTicks = 5;
    // INVARIANT: additive score delta (not multiplicative ratio) since switching to subtractive cost.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    float CommitmentMargin = 0.20f;

    // Target scoring uses local priority, deterministic tie-breaks, and distance penalty.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    float DistancePenaltyPerCell = 0.02f;

    // Limit the number of candidate targets that go through path evaluation.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    int32 TopKPathBonusCandidates = 32;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    float ArrivalRadiusUU = 150.0f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    int32 FallbackReplanTicks = 25;

    /** Maximum extra path length allowed relative to the shortest territory‑confined path (e.g. 0.20 = 20%). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    float DetourBudgetFraction = 0.20f;

    // Fast route planner used by default. The old layered DP can find better
    // high-gain detours, but it is much more expensive. The fast planner follows
    // the distance field to the target while allowing priority/FOV gain to choose
    // among equally valid next steps.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    bool bUseFastRoutePlanner = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner", meta = (ClampMin = "0.0"))
    float FastRouteFOVGainWeight = 0.05f;

    /** When enabled, greedy target and route reward use only the destination cell priority. */
    bool bUseTargetCellScoring = false;

    // Drone-local GPU priority cache. This does NOT read from the server/global
    // priority texture. Each drone uploads its own local belief arrays from OwnGrid,
    // computes the priority field with DroneLocalPriorityCompute.usf, and consumes the
    // result asynchronously on later replans. CPU local priority remains the fallback.
    // Defaults to false: the async cache is not bit-deterministic, so benchmark /
    // determinism runs must use the CPU priority fallback. The manager force-disables
    // it under parallel drone processing so sequential and parallel results match.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|GPU Planning")
    bool bUseDroneLocalGpuPriority = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|GPU Planning", meta = (ClampMin = "1"))
    int32 DroneLocalPriorityRequestIntervalTicks = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|GPU Planning", meta = (ClampMin = "0"))
    int32 MaxDroneLocalPriorityCacheAgeTicks = 8;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Sensor")
    TSubclassOf<UDroneSensorBase> SensorClass;

    /** Planner strategy. Empty = UGreedyFovPlanner (the built-in FOV-aware greedy
     *  planner — identical to pre-refactor behaviour). Swap to change target
     *  selection without touching ADrone. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Drone Config|Planner")
    TSubclassOf<UDronePlannerBase> PlannerClass;

    UFUNCTION(BlueprintPure, Category = "Thesis|Debug")
    int32 GetTargetCellIndex() const { return TargetCellIndex; }
    UFUNCTION(BlueprintPure, Category = "Thesis|Debug")
    float GetCurrentTargetScore() const { return CurrentTargetScore; }
    /** Number of cells this drone currently owns (metrics/debug only). */
    UFUNCTION(BlueprintPure, Category = "Thesis|Debug")
    int32 GetOwnedCellCount() const { return OwnedCells.Num(); }

    /** Built-in FOV-aware greedy replan (select target + path). Public so
     *  UDronePlannerBase implementations can delegate to it. */
    void RunGreedyFovReplan(int32 CurrentTick);

    /** Planner-facing helper: plan to a specific owned target using the same
     *  territory-confined pathing and speed limits as the greedy planner. */
    bool PlanDirectTargetCell(int32 TargetCell, int32 CurrentTick, float Score);

    /** Deterministic coverage replan shared by sequential and worker tick paths. */
    void RunLawnmowerV1Replan(int32 CurrentTick);
    /** V1 one-cell sweep with spawn-corner orientation and guarded initial leg. */
    void RunLawnmowerV1AlignedReplan(int32 CurrentTick);
    void ResetLawnmowerV1AlignedState();
    /** Persistent lane coverage planner. The completion snapshot is immutable during a drone phase. */
    void ConfigureLawnmowerV2Progress(const TArray<uint8>* CompletedCells) { LawnmowerV2CompletedSnapshot = CompletedCells; }
    void DrainLawnmowerV2CompletedCells(TArray<int32>& OutCells);
    void ResetLawnmowerV2State();
    void ResetLawnmowerV2ProgressEpoch();
    void RunLawnmowerV2Replan(int32 CurrentTick);

    /** Oracle-only direct flight. It deliberately ignores benchmark partition
     *  ownership: the upper-bound policy is global, while motion and sensing
     *  still use the same physical limits as every other planner. */
    bool PlanOracleTargetCell(int32 TargetCell, int32 CurrentTick, float Score);

    /** Seeded lower-bound policy: choose a random owned reachable target. */
    void RunRandomReplan(int32 CurrentTick, int32 RunSeed);
    void RunFullyRandomReplan(int32 CurrentTick, int32 RunSeed);

    const TArray<int32>& GetOwnedCellsForPlanning() const { return OwnedCells; }
    int32 GetCurrentCellIndex() const;
    int32 GetGridWidthForPlanning() const { return OwnGrid ? OwnGrid->Width : 0; }
    int32 GetGridHeightForPlanning() const { return OwnGrid ? OwnGrid->Height : 0; }

    /** Creates (or reconfigures) the downward scene-capture camera. Called by
     *  the manager when drone captures are enabled. */
    void ConfigureCapture(bool bEnable, int32 ResX, int32 ResY,
                          float CaptureFOVDegrees, float PitchDegrees, bool bSavePng);

    /** Applies observations produced OUTSIDE the drone's own sensor (e.g. the
     *  vision bridge) to the drone's local belief. Called from the server slot
     *  only, never during the parallel phase. Does NOT append to the private
     *  observation buffer — the caller is responsible for server-side ingestion. */
    void ApplyExternalObservations(const TArray<FObservation>& Observations, int32 CurrentTick);

    /** When enabled, only external observations supply person evidence.
     *  Configure between drone phases; the internal sensor still supplies fire. */
    void SetExternalPersonSensing(bool bEnable) { bExternalPersonSensing = bEnable; }
    // Shared across every drone in a mixed-sensor run, including ownership transfers.
    void SetSeparatePersonFreshness(bool bEnable) { bSeparatePersonFreshness = bEnable; }

    UDroneCaptureComponent* GetCaptureComponent() const { return CaptureComponent; }
    UDroneSensorBase* GetSensorInstance() const { return Sensor; }
    UDronePlannerBase* GetPlannerInstance() const { return Planner; }

    /** True only for drones that currently own a real render target capture. */
    bool HasActiveCapture() const;

    /** Creates (or reconfigures) the spectator view camera matching the capture
     *  transform. Pure visualization — never read by sensors or the planner, so
     *  it cannot affect determinism. */
    void ConfigureViewCamera(float FOVDegrees, float PitchDegrees);

    /** Public so SimulationManager can read it for debug visualization. */
    float ComputeCellPriority(int32 CellIndex, int32 CurrentTick) const;

    /** Intrinsic per-cell load (no monitoring urgency, no discovery gate). Used by
     *  SimulationManager's adaptive zone partition so a freshly-observed fire cell still counts
     *  as heavy — the urgency/gate mechanisms are planning-only. */
    float ComputeCellLoadPriority(int32 CellIndex, int32 CurrentTick) const;

    /** Detection-only load (fire + person evidence, no staleness term). Used by the
     *  adaptive partition to keep fire/person evidence heavy for workload balancing. */
    float ComputeCellDetectionMass(int32 CellIndex) const;

    /** Replace the drone's owned-cell list. NewlyOwnedCells are belief-synced from the shared grid. */
    void SetOwnedCells(TArray<int32>&& InOwnedCells, const TArray<int32>& NewlyOwnedCells);

    TArray<FObservation> ConsumePrivateObservationBuffer();
    void AppendPrivateObservationsTo(TArray<FObservation>& OutObservations);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Thesis|Drone Config|Planner")
    float NeighborPriorityWeight = 0.10f;


protected:
    virtual void BeginPlay() override;

private:
    TUniquePtr<Grid> OwnGrid;          // local belief copy
    Grid* SharedGrid = nullptr;        // reference to the authoritative grid (for ownership)
    FDroneTickProfile LastTickProfile;
    int32 CurrentFacingDir = 0;
    UPROPERTY()
    TObjectPtr<UDroneSensorBase> Sensor;

    /** Sensor-only ground truth for world-space person gating (see
     *  SetPersonGroundTruth). Never read by planning code. */
    const TArray<FHumanSnapshot>* PersonGroundTruth = nullptr;

    /** Mirror of UPerfectDroneSensor::bWorldSpacePersonGating for the
     *  parallel-CPU sensing path (SensePerfectFromLocation), which does not go
     *  through the sensor object. Refreshed in ApplyManagerConfiguration. */
    bool bWorldSpacePersonGatingCached = true;

    bool bExternalPersonSensing = false;
    bool bSeparatePersonFreshness = false;
    void FilterInternalPersonEvidence(TArray<FObservation>& Observations) const;

    UPROPERTY()
    TObjectPtr<UDronePlannerBase> Planner;

    UPROPERTY()
    TObjectPtr<UDroneCaptureComponent> CaptureComponent;

    bool bCaptureEnabled = false;

    /** Spectator camera (SetViewTargetWithBlend finds it automatically). */
    UPROPERTY()
    TObjectPtr<UCameraComponent> ViewCamera;

    TArray<FObservation> PrivateObservationBuffer;

    // Cells this drone owns under the current partition. Updated by SimulationManager.
    TArray<int32> OwnedCells;
    TArray<uint8> OwnedCellMask;

    bool IsOwnedCell(int32 Cell) const
    {
        return Cell >= 0 && Cell < OwnedCellMask.Num() && OwnedCellMask[Cell] != 0;
    }

    int32 TargetCellIndex = INDEX_NONE;
    int32 TicksSinceReplan = 0;
    int32 LastReplanTick = -1;
    float CurrentTargetScore = 0.0f;
    TArray<int32> LawnmowerOwnedCells;
    TArray<int32> LawnmowerSweepOrder;
    int32 LawnmowerNextSweepIndex = 0;
    bool bLawnmowerSweepDirty = true;
    TArray<int32> LawnmowerV1AlignedOwnedCells;
    TArray<int32> LawnmowerV1AlignedSweepOrder;
    int32 LawnmowerV1AlignedNextSweepIndex = 0;
    bool bLawnmowerV1AlignedSweepDirty = true;
    bool bLawnmowerV1AlignedInitialised = false;
    bool bLawnmowerV1AlignedHasStarted = false;
    bool bLawnmowerV1AlignedStartFromTop = false;
    bool bLawnmowerV1AlignedStartFromRight = false;
    const TArray<uint8>* LawnmowerV2CompletedSnapshot = nullptr;
    TArray<int32> LawnmowerV2CompletedOutbox;
    TArray<uint8> LawnmowerV2CompletedThisDrone;
    TArray<int32> LawnmowerV2SweepOrder;
    bool bLawnmowerV2SweepDirty = true;
    bool bLawnmowerV2Initialised = false;
    bool bLawnmowerV2HasStarted = false;
    bool bLawnmowerV2StartFromTop = false;
    bool bLawnmowerV2StartFromRight = false;

    FVector GridOriginCached = FVector::ZeroVector;
    float CellSizeCached = 0.0f;
    TWeakObjectPtr<AProceduralTerrain> TerrainProvider;
    float MinimumAltitudeAboveGroundUU = 4200.0f;
    float CruiseAltitudeAboveGroundUU = 5200.0f;
    float CameraForwardOffsetUU = 140.0f;
    float CameraDownOffsetUU = 80.0f;

    // Runtime priority model copied from ASimulationManager. Kept here so each
    // drone can plan from its own local belief without reading server priority.
    float PriorityWeightStaleness = ThesisPriority::WeightStaleness;
    float PriorityWeightDanger = ThesisPriority::WeightDanger;
    float PriorityWeightPerson = ThesisPriority::WeightPerson;
    float PriorityStalenessSaturationTicks = ThesisPriority::StalenessSaturationTicks;
    float PriorityProbabilityScale = ThesisPriority::ProbabilityScale;
    float PriorityMonitoringUrgencyTau = ThesisPriority::MonitoringUrgencyTau;
    float PriorityUnseenExplorationBonus = ThesisPriority::UnseenExplorationBonus;
    float PriorityFireRampMultiplier = ThesisPriority::FirePriorityRampMultiplier;
    float PriorityFireMonitoringUrgencyTau = ThesisPriority::FireMonitoringUrgencyTau;
    float PriorityFireMinimumUrgency = ThesisPriority::FireMinimumUrgency;
    int32 PriorityFireDiscoveryDelayTicks = ThesisPriority::FireDangerDiscoveryDelayTicks;
    int32 PriorityFireDiscoveryRampTicks = ThesisPriority::FireDangerDiscoveryRampTicks;
    int32 PriorityPersonDiscoveryDelayTicks = ThesisPriority::DroneDangerDiscoveryDelayTicks;
    int32 PriorityPersonDiscoveryRampTicks = ThesisPriority::DroneDangerDiscoveryRampTicks;
    float LoadWeightStaleness = ThesisPriority::LoadWeightStaleness;
    float LoadWeightDanger = ThesisPriority::LoadWeightDanger;
    float LoadWeightPerson = ThesisPriority::LoadWeightPerson;

    float CachedFirePositiveLLR = 3.0f;
    float CachedFireNegativeLLR = -0.3f;
    float CachedPersonPositiveLLR = 2.5f;
    float CachedPersonNegativeLLR = -0.2f;

    // ----- New pathfinding data -----
    /** Offsets (dx, dy) of all cell centres inside the FOV rectangle for each of the 8 compass directions.
     *  Index = direction ordinal: 0=E, 1=NE, 2=N, 3=NW, 4=W, 5=SW, 6=S, 7=SE. */
    TArray<FIntPoint> FOVOffsets[8];

    /** Pre‑computed once at init. */
    void PrecomputeFOVFootprint();

    /** The planned sequence of cell indices (starting from the cell after the current location). */
    TArray<int32> CurrentPath;
    int32 CurrentPathIndex = 0;
    bool bForceReplan = false;   // Set to true when territory changes

    // Per-drone planner scratch buffers. Hoisted out of RunGreedyFovReplanFromLocation
    // and ComputePathToTarget so a replan reuses these buffers instead of heap-allocating
    // several full-grid arrays on every call. In the parallel drone phase there is exactly
    // one task per drone, so a drone only ever touches its own scratch and this stays
    // lock-free.
    TArray<float> ScratchReplanPriority;
    TArray<float> ScratchCpuPlanningPriority;
    TArray<int32> ScratchCpuPlanningPriorityTick;
    TArray<float> ScratchReplanNeighborhood;
    TArray<int32> ScratchPathDist;
    TArray<int32> ScratchPathTouchedDistCells;
    TArray<int32> ScratchPathBFSQueue;
    TArray<bool>  ScratchPathVisited;
    TArray<int32> ScratchPathVisitedCells;
    TArray<float> ScratchPathCellDirGain;
    TArray<float> ScratchPathPrevGain;
    TArray<float> ScratchPathCurrGain;
    TArray<int32> ScratchPathActivePrev;
    TArray<int32> ScratchPathActiveCurr;
    TArray<TArray<int32>> ScratchPathParentLayers;
    TArray<int32> ScratchPathFallbackParent;
    TArray<int32> ScratchPathFallbackQueue;
    TArray<int32> ScratchPathRevPath;
    TArray<int32> ScratchRandomVisitMarker;
    TArray<int32> ScratchRandomVisitedCells;
    TArray<int32> ScratchRandomCandidates;
    TArray<FObservation> ScratchObservations;

    TArray<float> CachedLocalGpuPriority;
    TArray<float> CachedLocalGpuNeighborhoodScore;
    int32 CachedLocalGpuPriorityTick = -1;
    int32 LastLocalGpuPriorityRequestTick = -999999;
    int32 PendingLocalGpuPriorityTick = -1;
    int32 PendingLocalGpuPriorityCopyTick = -1;
    bool bLocalGpuPriorityRequestInFlight = false;
    bool bLocalGpuPriorityCopyQueued = false;
    TUniquePtr<FRHIGPUBufferReadback> LocalPriorityReadback;
    FRenderCommandFence LocalPriorityReadbackFence;
    FRenderCommandFence LocalPriorityCpuCopyFence;
    struct FLocalGpuPlanningReadback
    {
        TArray<float> Priority;
        TArray<float> NeighborhoodScore;
    };
    TSharedPtr<FLocalGpuPlanningReadback, ESPMode::ThreadSafe> PendingLocalPriorityCpuCopy;

    /** Compute a path from StartCell to TargetCell that stays inside OwnedCells,
     *  maximises observed priority, and respects DetourBudgetFraction.
     *  Returns false if no path exists within budget; then CurrentPath is set to the shortest path.
    */
    bool ComputePathToTarget(int32 StartCell, int32 TargetCell,
                             const TArray<float>* PriorityCache,
                             int32 CurrentTick,
                             bool bUseCpuPriorityFallback,
                             bool bTargetCellScoring = false);
    float GetCachedCpuPlanningPriority(int32 CellIndex, int32 CurrentTick);

    bool TryConsumeLocalGpuPriority(TArray<float>& OutPriorityCache,
                                    TArray<float>& OutNeighborhoodScoreCache,
                                    int32 CurrentTick);
    void RequestLocalGpuPriority(int32 CurrentTick);
    void ResetLocalGpuPriorityCache();

    /** Move one step along the planned path. */
    void MoveAlongPath();

    void SensePerfectFromLocation(int32 CurrentTick,
                                  const FVector& DroneWorldPos,
                                  TArray<FObservation>& OutObservations,
                                  TArray<FIdealSensorWitnessEvent>* OutWitnessEvents = nullptr) const;
    void RunGreedyFovReplanFromLocation(int32 CurrentTick,
                                        const FVector& DroneWorldPos,
                                        bool bAllowLocalGpuPriority,
                                        bool bUseTerrainAdjustedCenters);
    void RunRandomReplanFromLocation(int32 CurrentTick,
                                     const FVector& DroneWorldPos,
                                     int32 RunSeed);
    void RunFullyRandomReplanFromLocation(int32 CurrentTick,
                                         const FVector& DroneWorldPos,
                                         int32 RunSeed);
    FVector MoveAlongPathFromLocation(const FVector& CurrentLocation,
                                      bool& bOutMoved,
                                      bool bUseTerrainAdjustedTarget);
    int32 GetCurrentCellIndexFromLocation(const FVector& WorldLocation) const;
    FVector GetCellWorldCenterForPlanning(int32 CellIndex,
                                          float ReferenceZ,
                                          bool bUseTerrainAdjustedCenter) const;

    // (Replan moved to public RunGreedyFovReplan — see Planner API above.)

    // Helpers
    void CellIndexToXY(int32 Idx, int32& OutX, int32& OutY) const;
    FVector GetCellWorldCenter(int32 CellIndex) const;
    float GetTerrainHeightAtWorldXY(float WorldX, float WorldY) const;
    float GetTerrainFlightReferenceZAtWorldXY(float WorldX, float WorldY) const;
    FVector ApplyTerrainFlightAltitude(const FVector& InLocation, bool bUseCruiseAltitude) const;

    // Compass direction mapping
    static constexpr int32 DirOffsetsX[8] = { 1, 1, 0,-1,-1,-1, 0, 1 };
    static constexpr int32 DirOffsetsY[8] = { 0,-1,-1,-1, 0, 1, 1, 1 };
    int32 DirectionIndex(int32 FromCell, int32 ToCell) const;
    int32 DirectionIndexFromDelta(int32 dx, int32 dy) const;
};
