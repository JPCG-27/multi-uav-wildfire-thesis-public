#pragma once

#include "CoreMinimal.h"

/**
 * Per-run metrics recorder, fed exclusively from the single-threaded server
 * slot (deterministic row order). Writes CSVs to Saved/Benchmarks/<RunId>/ by
 * default, or to a configured subdirectory under Saved/Benchmarks.
 *
 * Files:
 *   ticks.csv   — one row per tick: global counters (observation/detection counts).
 *   drones.csv  — long format, one row per (tick, drone): position, target, score,
 *                 owned-cell count. Long format plots directly in pandas/matplotlib.
 *   events.csv  — sparse events: partition passes (workloads), person appear/detect
 *                 (wired in when humans land).
 *   summary.csv — single row written at EndRun.
 *   config.json — provenance snapshot of the resolved run configuration.
 *
 * Performance: rows are buffered in memory and flushed every FlushEveryNTicks
 * (and at EndRun). No grid-wide scans happen here — grid-level statistics come
 * from GPU reductions and are pushed in via RecordGridStats when available.
 *
 * Determinism: recording must never influence the simulation. This class only
 * consumes values; it has no callbacks back into sim state.
 */
struct THESIS_API FDroneTickSample
{
    int32 DroneIndex = INDEX_NONE;
    float PosCellX = 0.0f;
    float PosCellY = 0.0f;
    int32 TargetCellIndex = INDEX_NONE;
    float TargetScore = 0.0f;
    int32 OwnedCellCount = 0;
};

struct THESIS_API FPersonRecord
{
    int32 PersonId = INDEX_NONE;
    int32 AppearTick = -1;
    int32 FirstDetectTick = -1;
    int32 LastObservedTick = -1;
    int32 NumReacquisitions = 0;
    int32 NumTrackingGaps = 0;
    int32 TotalTrackingGapTicks = 0;
    int32 CurrentTrackingGapTicks = 0;
    int32 MaxTrackingGapTicks = 0;
    int32 UntrackedDangerTicks = 0;
    float WeightedUntrackedExposure = 0.0f;
    bool bWasTracked = false;
    int32 FirstGeometricProcessingTick = -1;
    int32 FirstGeometricCaptureTick = -1;
    int32 GeometricMatchCount = 0;
    int32 IdealWitnessEventCount = 0;
    int32 FirstIdealSensingTick = -1;
    int32 FirstIdealSnapshotTick = -1;
};

struct THESIS_API FSimPhaseTimingAccumulator
{
    double TotalMs = 0.0;
    float MaxMs = 0.0f;
    int32 Samples = 0;
};

enum class ESimMetricsDetailMode : uint8
{
    Full,
    Summary
};

class THESIS_API FSimMetricsRecorder
{
public:
    /** Starts a run. RunId must be filesystem-safe; ConfigSnapshotJson is written
     *  verbatim as config.json. Returns false if the output directory could not
     *  be created (recording stays disabled — sim proceeds unaffected). */
    bool BeginRun(const FString& RunId, const FString& ConfigSnapshotJson, const FString& OutputSubdir = FString());

    /** One call per tick from the server slot, after observation collection. */
    void RecordTick(int32 Tick,
                    const TArray<FDroneTickSample>& DroneSamples,
                    int32 NumObservations,
                    int32 NumPositiveFireObs,
                    int32 NumPositivePersonObs,
                    float TickComputeMs);

    /** Called on partition recompute ticks. */
    void RecordPartitionPass(int32 Tick, const TArray<int32>& DroneWorkloads);

    /** Writes the opt-in dense-slot to stable-identity and pre-movement spawn map. */
    void RecordDroneIdentity(int32 DenseSlot, int32 LogicalDroneId, float SpawnCellX,
                             float SpawnCellY, const FVector& SpawnWorld, const FRotator& SpawnRotation);

    /** Grid-wide stats produced by GPU reduction (pushed when a readback lands).
     *  SourceTick identifies the tick the stats describe (readback may lag). */
    void RecordGridStats(int32 SourceTick,
                         float MeanStaleness,
                         float MaxStaleness,
                         float CoverageFraction);

    /** Sparse named event (person_appear, person_first_detect, ...). */
    void RecordEvent(int32 Tick, const FString& EventType, const FString& Detail);

    /** Total persons in the scenario (for the detection-fraction summary). */
    void SetPersonCount(int32 InPersonCount);
    void SetRecallDeadlineTicks(int32 InDeadlineTicks) { RecallDeadlineTicks = FMath::Max(1, InDeadlineTicks); }

    /** First detection of a person by server belief. Latency = Tick - AppearTick.
     *  Call at most once per person per run. */
    void RecordPersonDetection(int32 Tick, int32 PersonId, int32 AppearTick);

    /** Optional, read-only replay diagnostic. Does not feed simulation state. */
    void SetPersonGeometricRecoveryEnabled(bool bEnabled);
    void SetIdealSensorWitnessRecordingEnabled(bool bEnabled);
    bool IsIdealSensorWitnessRecordingEnabled() const { return bIdealSensorWitnessRecordingEnabled; }
    void SetMixedObservationRecoveryEnabled(bool bEnabled);
    bool IsMixedObservationRecoveryEnabled() const { return bMixedObservationRecoveryEnabled; }
    void RecordIdealSensorWitness(int32 SensingTick, int32 SnapshotTick, int32 DroneIndex,
                                  int32 PersonId, int32 CellIndex);
    void RecordPersonGeometricMatch(int32 ProcessingTick, int32 CaptureTick, int32 DroneIndex,
                                    int32 PersonId, int32 DetectionIndex, float Confidence,
                                    int32 DetectionCell, int32 PersonCell);

    void RecordPersonState(int32 Tick,
                           int32 PersonId,
                           int32 AppearTick,
                           bool bTracked,
                           float Danger,
                           float DangerThreshold);

    void RecordBeliefError(float MeanBeliefError);

    /** One-to-one geometric alive-person tally; opaque/smoke visibility is not inferred. */
    void RecordYoloFrame(int32 TruePositives, int32 FalsePositives, int32 Missed,
                         int32 PersonsInView, int32 NonPersonDetections, int32 Tick = -1, int32 DroneIndex = -1);

    /** Optional wall-clock timing sample for one named setup/tick phase. */
    void RecordPhaseTiming(int32 Tick, const FString& PhaseName, float ElapsedMs);

    /** Writes summary.csv and closes all files. Safe to call when inactive. */
    void EndRun(int32 FinalTick);

    bool IsActive() const { return bActive; }
    const FString& GetRunDirectory() const { return RunDir; }
    void SetDetailMode(ESimMetricsDetailMode InDetailMode) { DetailMode = InDetailMode; }
    ESimMetricsDetailMode GetDetailMode() const { return DetailMode; }

    int32 FlushEveryNTicks = 256;

private:
    void Flush();
    void AppendToFile(const FString& FilePath, const FString& Content) const;

    bool bActive = false;
    ESimMetricsDetailMode DetailMode = ESimMetricsDetailMode::Full;
    FString RunDir;
    FString TicksPath, DronesPath, EventsPath, PersonsPath, PhaseTimingsPath, PhaseSummaryPath;

    FString TicksBuffer, DronesBuffer, EventsBuffer, PhaseTimingsBuffer;
    FString YoloFramesPath, YoloFramesBuffer;
    FString PersonGeometricEventsPath, PersonGeometricEventsBuffer;
    FString PersonGeometricSummaryPath;
    FString DroneIdentityPath;
    FString IdealWitnessEventsPath, IdealWitnessPeoplePath, IdealWitnessSummaryPath;
    FString IdealWitnessEventsBuffer;
    bool bDroneIdentityFileInitialized = false;
    bool bIdealSensorWitnessRecordingEnabled = false;
    bool bPersonGeometricRecoveryEnabled = false;
    bool bMixedObservationRecoveryEnabled = false;
    FString MixedObservationEventsPath, MixedObservationPeoplePath, MixedObservationSummaryPath;
    FString MixedObservationEventsBuffer;
    int32 TicksSinceFlush = 0;
    bool bPhaseTimingsFileInitialized = false;
    TMap<FString, FSimPhaseTimingAccumulator> PhaseTimingByName;

    // Summary accumulators.
    int64 TotalObservations = 0;
    int64 TotalPositiveFireObs = 0;
    int64 TotalPositivePersonObs = 0;
    int32 NumPartitionPasses = 0;
    int32 FirstRecordedTick = -1;
    int32 LastRecordedTick = -1;
    double SumMeanStaleness = 0.0;
    int32 NumGridStatSamples = 0;
    float LastCoverageFraction = 0.0f;
    float LastMaxStaleness = 0.0f;
    double TotalDistanceCells = 0.0;
    double SumTickMs = 0.0;
    int32 NumTickMsSamples = 0;
    int32 FireDiscoveryTick = -1;
    float LastWorkloadImbalance = 0.0f;
    float MeanBeliefError = 0.0f;

    // Person-detection accumulators (primary dissertation metric).
    int32 PersonCount = 0;
    int32 PersonsDetected = 0;
    int64 SumDetectionLatency = 0;
    int32 RecallDeadlineTicks = 500;
    TArray<FPersonRecord> PersonRecords;
    TArray<float> LastDroneX;
    TArray<float> LastDroneY;
    TArray<bool> HasLastDronePos;

    // YOLO detection-accuracy accumulators (vision-sensor runs; zero otherwise).
    int64 YoloTruePositives = 0;
    int64 YoloFalsePositives = 0;
    int64 YoloMissed = 0;
    int64 YoloPersonsInView = 0;
    int64 YoloNonPersonDetections = 0;
    int32 YoloFrames = 0;
};
