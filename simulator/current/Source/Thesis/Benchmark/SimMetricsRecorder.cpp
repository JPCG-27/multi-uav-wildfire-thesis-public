#include "SimMetricsRecorder.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
    static FString SanitizePhaseCsvToken(FString Value)
    {
        Value.ReplaceInline(TEXT(","), TEXT("_"));
        Value.ReplaceInline(TEXT("\r"), TEXT("_"));
        Value.ReplaceInline(TEXT("\n"), TEXT("_"));
        return Value;
    }

    static FString MakePhaseSummarySuffix(const FString& PhaseName)
    {
        FString Out = PhaseName.ToLower();
        for (int32 I = 0; I < Out.Len(); ++I)
        {
            const TCHAR C = Out[I];
            const bool bAllowed =
                (C >= TEXT('a') && C <= TEXT('z')) ||
                (C >= TEXT('0') && C <= TEXT('9'));
            if (!bAllowed)
            {
                Out[I] = TEXT('_');
            }
        }
        return Out;
    }
}

bool FSimMetricsRecorder::BeginRun(const FString& RunId, const FString& ConfigSnapshotJson, const FString& OutputSubdir)
{
    if (bMetricsWriteFailed && HasPendingWrites())
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis metrics: refusing to start another run while failed-run buffers remain pending."));
        return false;
    }
    if (bActive)
    {
        EndRun(LastRecordedTick);
    }

    FString SafeSubdir = OutputSubdir;
    SafeSubdir.ReplaceInline(TEXT("\\"), TEXT("/"));
    if (SafeSubdir.IsEmpty())
    {
        SafeSubdir = RunId;
    }
    if (SafeSubdir.StartsWith(TEXT("/")) || SafeSubdir.Contains(TEXT("..")))
    {
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis metrics: invalid benchmark output subdir '%s'; falling back to run id '%s'."),
               *SafeSubdir, *RunId);
        SafeSubdir = RunId;
    }

    RunDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"), SafeSubdir);
    if (IFileManager::Get().DirectoryExists(*RunDir))
    {
        TArray<FString> ExistingEntries;
        IFileManager::Get().FindFiles(ExistingEntries, *(RunDir / TEXT("*")), true, true);
        if (!ExistingEntries.IsEmpty())
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis metrics: refusing to reuse non-empty run directory %s; existing evidence is preserved."), *RunDir);
            bActive = false;
            return false;
        }
    }
    if (!IFileManager::Get().MakeDirectory(*RunDir, /*Tree=*/true))
    {
        UE_LOG(LogTemp, Warning, TEXT("Thesis metrics: failed to create %s — recording disabled."), *RunDir);
        bActive = false;
        return false;
    }

    bMetricsWriteFailed = false;
    MetricsWriteFailurePath = FPaths::Combine(RunDir, TEXT("metrics_write_failure.txt"));

    TicksPath  = FPaths::Combine(RunDir, TEXT("ticks.csv"));
    DronesPath = FPaths::Combine(RunDir, TEXT("drones.csv"));
    EventsPath = FPaths::Combine(RunDir, TEXT("events.csv"));
    PersonsPath = FPaths::Combine(RunDir, TEXT("persons.csv"));
    PhaseTimingsPath = FPaths::Combine(RunDir, TEXT("phase_timings.csv"));
    PhaseSummaryPath = FPaths::Combine(RunDir, TEXT("phase_summary.csv"));
    YoloFramesPath = FPaths::Combine(RunDir, TEXT("yolo_frames.csv"));
    WriteToFile(YoloFramesPath, TEXT("tick,drone,true_positives,false_positives,false_negatives,persons_in_view,nonperson_detections\n"));
    YoloFramesBuffer.Reset();
    PersonGeometricEventsPath = FPaths::Combine(RunDir, TEXT("person_geometric_events.csv"));
    PersonGeometricSummaryPath = FPaths::Combine(RunDir, TEXT("person_geometric_matches.csv"));
    DroneIdentityPath = FPaths::Combine(RunDir, TEXT("drone_identity.csv"));
    bDroneIdentityFileInitialized = false;
    IdealWitnessEventsPath = FPaths::Combine(RunDir, TEXT("ideal_sensor_witness_events.csv"));
    IdealWitnessPeoplePath = FPaths::Combine(RunDir, TEXT("ideal_sensor_witness_people.csv"));
    IdealWitnessSummaryPath = FPaths::Combine(RunDir, TEXT("ideal_sensor_witness_summary.csv"));
    IdealWitnessEventsBuffer.Reset();
    bIdealSensorWitnessRecordingEnabled = false;
    PersonGeometricEventsBuffer.Reset();
    bPersonGeometricRecoveryEnabled = false;
    bMixedObservationRecoveryEnabled = false;
    MixedObservationEventsPath = FPaths::Combine(RunDir, TEXT("mixed_observation_events.csv"));
    MixedObservationPeoplePath = FPaths::Combine(RunDir, TEXT("mixed_observation_people.csv"));
    MixedObservationSummaryPath = FPaths::Combine(RunDir, TEXT("mixed_observation_summary.csv"));
    MixedObservationEventsBuffer.Reset();
    IFileManager::Get().Delete(*MixedObservationEventsPath, false, true, true);
    IFileManager::Get().Delete(*MixedObservationPeoplePath, false, true, true);
    IFileManager::Get().Delete(*MixedObservationSummaryPath, false, true, true);

    // Initialize fresh output files. Non-empty run directories were rejected above.
    if (DetailMode == ESimMetricsDetailMode::Full)
    {
        WriteToFile(TicksPath, TEXT("tick,num_observations,num_positive_fire_obs,num_positive_person_obs\n"));
        WriteToFile(DronesPath, TEXT("tick,drone,pos_cell_x,pos_cell_y,target_cell,target_score,owned_cells\n"));
    }
    else
    {
        IFileManager::Get().Delete(*TicksPath, false, true, true);
        IFileManager::Get().Delete(*DronesPath, false, true, true);
    }
    WriteToFile(EventsPath, TEXT("tick,event,detail\n"));
    WriteToFile(PersonsPath, TEXT("person_id,appear_tick,first_detect_tick,latency_ticks,num_reacquisitions,num_tracking_gaps,total_tracking_gap_ticks,max_tracking_gap_ticks,untracked_danger_ticks,weighted_untracked_exposure\n"));
    IFileManager::Get().Delete(*PhaseTimingsPath, false, true, true);
    IFileManager::Get().Delete(*PhaseSummaryPath, false, true, true);
    WriteToFile(FPaths::Combine(RunDir, TEXT("config.json")), ConfigSnapshotJson);

    TicksBuffer.Reset();
    DronesBuffer.Reset();
    EventsBuffer.Reset();
    PhaseTimingsBuffer.Reset();
    TicksSinceFlush = 0;
    bPhaseTimingsFileInitialized = false;

    TotalObservations = 0;
    TotalPositiveFireObs = 0;
    TotalPositivePersonObs = 0;
    NumPartitionPasses = 0;
    FirstRecordedTick = -1;
    LastRecordedTick = -1;
    SumMeanStaleness = 0.0;
    NumGridStatSamples = 0;
    LastCoverageFraction = 0.0f;
    LastMaxStaleness = 0.0f;
    TotalDistanceCells = 0.0;
    SumTickMs = 0.0;
    NumTickMsSamples = 0;
    FireDiscoveryTick = -1;
    LastWorkloadImbalance = 0.0f;
    MeanBeliefError = 0.0f;
    PersonCount = 0;
    PersonsDetected = 0;
    SumDetectionLatency = 0;
    PersonRecords.Reset();
    LastDroneX.Reset();
    LastDroneY.Reset();
    HasLastDronePos.Reset();
    YoloTruePositives = 0;
    YoloFalsePositives = 0;
    YoloMissed = 0;
    YoloPersonsInView = 0;
    YoloNonPersonDetections = 0;
    YoloFrames = 0;
    PhaseTimingByName.Reset();

    bActive = !bMetricsWriteFailed;
    UE_LOG(LogTemp, Log, TEXT("Thesis metrics: recording run '%s' -> %s"), *RunId, *RunDir);
    return bActive;
}

void FSimMetricsRecorder::RecordTick(int32 Tick,
                                     const TArray<FDroneTickSample>& DroneSamples,
                                     int32 NumObservations,
                                     int32 NumPositiveFireObs,
                                     int32 NumPositivePersonObs,
                                     float TickComputeMs)
{
    if (!bActive) return;

    if (FirstRecordedTick < 0) FirstRecordedTick = Tick;
    LastRecordedTick = Tick;

    if (DetailMode == ESimMetricsDetailMode::Full)
    {
        TicksBuffer += FString::Printf(TEXT("%d,%d,%d,%d\n"),
            Tick, NumObservations, NumPositiveFireObs, NumPositivePersonObs);
    }

    for (const FDroneTickSample& S : DroneSamples)
    {
        if (DetailMode == ESimMetricsDetailMode::Full)
        {
            DronesBuffer += FString::Printf(TEXT("%d,%d,%.4f,%.4f,%d,%.6f,%d\n"),
                Tick, S.DroneIndex, S.PosCellX, S.PosCellY,
                S.TargetCellIndex, S.TargetScore, S.OwnedCellCount);
        }

        if (S.DroneIndex >= 0)
        {
            if (LastDroneX.Num() <= S.DroneIndex)
            {
                const int32 NewSize = S.DroneIndex + 1;
                LastDroneX.SetNumZeroed(NewSize);
                LastDroneY.SetNumZeroed(NewSize);
                HasLastDronePos.SetNumZeroed(NewSize);
            }
            if (HasLastDronePos[S.DroneIndex])
            {
                const float Dx = S.PosCellX - LastDroneX[S.DroneIndex];
                const float Dy = S.PosCellY - LastDroneY[S.DroneIndex];
                TotalDistanceCells += FMath::Sqrt(Dx * Dx + Dy * Dy);
            }
            LastDroneX[S.DroneIndex] = S.PosCellX;
            LastDroneY[S.DroneIndex] = S.PosCellY;
            HasLastDronePos[S.DroneIndex] = true;
        }
    }

    TotalObservations += NumObservations;
    TotalPositiveFireObs += NumPositiveFireObs;
    TotalPositivePersonObs += NumPositivePersonObs;
    if (FireDiscoveryTick < 0 && NumPositiveFireObs > 0)
    {
        FireDiscoveryTick = Tick;
    }
    SumTickMs += FMath::Max(0.0f, TickComputeMs);
    ++NumTickMsSamples;

    if (++TicksSinceFlush >= FMath::Max(1, FlushEveryNTicks))
    {
        Flush();
    }
}

void FSimMetricsRecorder::RecordDroneIdentity(int32 DenseSlot, int32 LogicalDroneId, float SpawnCellX,
                                              float SpawnCellY, const FVector& SpawnWorld,
                                              const FRotator& SpawnRotation)
{
    if (!bActive || DenseSlot < 0 || LogicalDroneId < 0) return;
    if (!bDroneIdentityFileInitialized)
    {
        WriteToFile(DroneIdentityPath, TEXT("dense_slot,logical_drone_id,spawn_cell_x,spawn_cell_y,spawn_world_x,spawn_world_y,spawn_world_z,spawn_pitch,spawn_yaw,spawn_roll\n"));
        bDroneIdentityFileInitialized = true;
    }
    const FString Row = FString::Printf(TEXT("%d,%d,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f\n"),
        DenseSlot, LogicalDroneId, SpawnCellX, SpawnCellY, SpawnWorld.X, SpawnWorld.Y, SpawnWorld.Z,
        SpawnRotation.Pitch, SpawnRotation.Yaw, SpawnRotation.Roll);
    AppendToFile(DroneIdentityPath, Row);
}

void FSimMetricsRecorder::RecordPartitionPass(int32 Tick, const TArray<int32>& DroneWorkloads)
{
    if (!bActive) return;
    ++NumPartitionPasses;
    if (DroneWorkloads.Num() > 0)
    {
        int32 MinW = DroneWorkloads[0];
        int32 MaxW = DroneWorkloads[0];
        for (const int32 W : DroneWorkloads)
        {
            MinW = FMath::Min(MinW, W);
            MaxW = FMath::Max(MaxW, W);
        }
        LastWorkloadImbalance = static_cast<float>(MaxW - MinW);
    }

    FString Workloads;
    for (int32 i = 0; i < DroneWorkloads.Num(); ++i)
    {
        Workloads += (i > 0 ? TEXT(";") : TEXT(""));
        Workloads += FString::FromInt(DroneWorkloads[i]);
    }
    EventsBuffer += FString::Printf(TEXT("%d,partition_pass,%s\n"), Tick, *Workloads);
}

void FSimMetricsRecorder::RecordGridStats(int32 SourceTick,
                                          float MeanStaleness,
                                          float MaxStaleness,
                                          float CoverageFraction)
{
    if (!bActive) return;
    EventsBuffer += FString::Printf(TEXT("%d,grid_stats,mean_staleness=%.4f;max_staleness=%.4f;coverage=%.6f\n"),
        SourceTick, MeanStaleness, MaxStaleness, CoverageFraction);
    SumMeanStaleness += MeanStaleness;
    ++NumGridStatSamples;
    LastCoverageFraction = CoverageFraction;
    LastMaxStaleness = MaxStaleness;
}

void FSimMetricsRecorder::SetPersonCount(int32 InPersonCount)
{
    PersonCount = FMath::Max(0, InPersonCount);
    PersonRecords.SetNum(PersonCount);
    for (int32 I = 0; I < PersonRecords.Num(); ++I)
    {
        PersonRecords[I].PersonId = I;
    }
}

void FSimMetricsRecorder::RecordPersonDetection(int32 Tick, int32 PersonId, int32 AppearTick)
{
    if (!bActive) return;
    if (PersonId < 0)
    {
        return;
    }
    if (PersonRecords.Num() <= PersonId)
    {
        SetPersonCount(PersonId + 1);
    }
    FPersonRecord& R = PersonRecords[PersonId];
    R.PersonId = PersonId;
    if (R.AppearTick < 0)
    {
        R.AppearTick = AppearTick;
    }
    if (R.FirstDetectTick >= 0)
    {
        return;
    }
    const int32 Latency = FMath::Max(0, Tick - AppearTick);
    R.FirstDetectTick = Tick;
    R.LastObservedTick = Tick;
    ++PersonsDetected;
    SumDetectionLatency += Latency;
    EventsBuffer += FString::Printf(TEXT("%d,person_first_detect,person=%d;latency=%d\n"),
        Tick, PersonId, Latency);
}

void FSimMetricsRecorder::SetPersonGeometricRecoveryEnabled(bool bEnabled)
{
    bPersonGeometricRecoveryEnabled = bActive && bEnabled;
    if (!bPersonGeometricRecoveryEnabled)
    {
        IFileManager::Get().Delete(*PersonGeometricEventsPath, false, true, true);
        IFileManager::Get().Delete(*PersonGeometricSummaryPath, false, true, true);
        PersonGeometricEventsBuffer.Reset();
        return;
    }

    WriteToFile(PersonGeometricEventsPath, TEXT("capture_tick,processing_tick,drone_index,person_id,detection_index,confidence,detection_cell,person_cell,metric_version\n"));
    WriteToFile(PersonGeometricSummaryPath, TEXT("person_id,appear_tick,proxy_first_detect_tick,proxy_latency_ticks,first_geometric_processing_tick,first_geometric_capture_tick,first_geometric_processing_latency_ticks,first_geometric_capture_latency_ticks,geometric_match_count,proxy_first_without_geometric_match,proxy_first_before_geometric_match\n"));
}

void FSimMetricsRecorder::SetIdealSensorWitnessRecordingEnabled(bool bEnabled)
{
    bIdealSensorWitnessRecordingEnabled = bActive && bEnabled;
    IdealWitnessEventsBuffer.Reset();
    if (bIdealSensorWitnessRecordingEnabled)
    {
        IFileManager::Get().Delete(*IdealWitnessEventsPath, false, true, true);
        IFileManager::Get().Delete(*IdealWitnessPeoplePath, false, true, true);
        IFileManager::Get().Delete(*IdealWitnessSummaryPath, false, true, true);
        WriteToFile(IdealWitnessEventsPath, TEXT("sensing_tick,snapshot_tick,logical_drone_id,person_id,cell_index,metric_version\n"));
    }
}

void FSimMetricsRecorder::RecordPersonGeometricMatch(int32 ProcessingTick, int32 CaptureTick, int32 DroneIndex,
                                                      int32 PersonId, int32 DetectionIndex, float Confidence,
                                                      int32 DetectionCell, int32 PersonCell)
{
    if (!bActive || !bPersonGeometricRecoveryEnabled || PersonId < 0)
    {
        return;
    }
    if (PersonRecords.Num() <= PersonId)
    {
        SetPersonCount(PersonId + 1);
    }

    FPersonRecord& R = PersonRecords[PersonId];
    R.PersonId = PersonId;
    if (R.FirstGeometricProcessingTick < 0)
    {
        R.FirstGeometricProcessingTick = ProcessingTick;
        R.FirstGeometricCaptureTick = CaptureTick;
    }
    ++R.GeometricMatchCount;
    PersonGeometricEventsBuffer += FString::Printf(
        TEXT("%d,%d,%d,%d,%d,%.8f,%d,%d,geometric_alive_one_to_one_cell_neighborhood_v2\n"),
        CaptureTick, ProcessingTick, DroneIndex, PersonId, DetectionIndex, Confidence, DetectionCell, PersonCell);
    if (bMixedObservationRecoveryEnabled)
    {
        MixedObservationEventsBuffer += FString::Printf(
            TEXT("camera_geometric,-1,%d,%d,%d,%d,%d,%d,%d,%d,%.8f,legacy_greedy_alive_person_cell_neighbourhood,heterogeneous_sensor_witness_coverage_v1\n"),
            ProcessingTick, ProcessingTick - 1, CaptureTick, ProcessingTick, DroneIndex,
            PersonId, PersonCell, DetectionIndex, Confidence);
    }
}

void FSimMetricsRecorder::SetMixedObservationRecoveryEnabled(bool bEnabled)
{
    bMixedObservationRecoveryEnabled = bActive && bEnabled && bPersonGeometricRecoveryEnabled;
    if (!bMixedObservationRecoveryEnabled)
    {
        IFileManager::Get().Delete(*MixedObservationEventsPath, false, true, true);
        IFileManager::Get().Delete(*MixedObservationPeoplePath, false, true, true);
        IFileManager::Get().Delete(*MixedObservationSummaryPath, false, true, true);
        MixedObservationEventsBuffer.Reset();
        return;
    }
    WriteToFile(MixedObservationEventsPath, TEXT("source,sensing_tick,availability_tick,truth_snapshot_tick,capture_tick,processing_tick,drone_index,person_id,cell_index,detection_index,confidence,attribution,metric_version\n"));
}

void FSimMetricsRecorder::RecordIdealSensorWitness(int32 SensingTick, int32 SnapshotTick, int32 DroneIndex,
                                                    int32 PersonId, int32 CellIndex)
{
    if (!bActive || (!bMixedObservationRecoveryEnabled && !bIdealSensorWitnessRecordingEnabled) || PersonId < 0) return;
    if (PersonRecords.Num() <= PersonId) SetPersonCount(PersonId + 1);
    FPersonRecord& R = PersonRecords[PersonId];
    R.PersonId = PersonId;
    if (R.FirstIdealSensingTick < 0)
    {
        R.FirstIdealSensingTick = SensingTick;
        R.FirstIdealSnapshotTick = SnapshotTick;
    }
    ++R.IdealWitnessEventCount;
    if (bMixedObservationRecoveryEnabled)
    {
        MixedObservationEventsBuffer += FString::Printf(
            TEXT("ideal_gate_witness,%d,%d,%d,-1,-1,%d,%d,%d,-1,-1,first_qualifying_binary_gate_witness,heterogeneous_sensor_witness_coverage_v1\n"),
            SensingTick, SensingTick, SnapshotTick, DroneIndex, PersonId, CellIndex);
    }
    if (bIdealSensorWitnessRecordingEnabled)
    {
        IdealWitnessEventsBuffer += FString::Printf(
            TEXT("%d,%d,%d,%d,%d,ideal_gate_witness_v1\n"),
            SensingTick, SnapshotTick, DroneIndex, PersonId, CellIndex);
    }
}

void FSimMetricsRecorder::RecordPersonState(int32 Tick,
                                            int32 PersonId,
                                            int32 AppearTick,
                                            bool bTracked,
                                            float Danger,
                                            float DangerThreshold)
{
    if (!bActive || PersonId < 0)
    {
        return;
    }
    if (PersonRecords.Num() <= PersonId)
    {
        SetPersonCount(PersonId + 1);
    }

    FPersonRecord& R = PersonRecords[PersonId];
    R.PersonId = PersonId;
    if (R.AppearTick < 0)
    {
        R.AppearTick = AppearTick;
        EventsBuffer += FString::Printf(TEXT("%d,person_appear,person=%d\n"), AppearTick, PersonId);
    }

    if (bTracked)
    {
        if (!R.bWasTracked && R.LastObservedTick >= 0)
        {
            ++R.NumReacquisitions;
            R.MaxTrackingGapTicks = FMath::Max(R.MaxTrackingGapTicks, R.CurrentTrackingGapTicks);
            R.CurrentTrackingGapTicks = 0;
        }
        R.LastObservedTick = Tick;
        R.bWasTracked = true;
        if (R.FirstDetectTick < 0)
        {
            RecordPersonDetection(Tick, PersonId, R.AppearTick);
        }
    }
    else
    {
        if (R.FirstDetectTick >= 0)
        {
            if (R.bWasTracked && R.CurrentTrackingGapTicks == 0)
            {
                ++R.NumTrackingGaps;
            }
            ++R.TotalTrackingGapTicks;
            ++R.CurrentTrackingGapTicks;
            R.MaxTrackingGapTicks = FMath::Max(R.MaxTrackingGapTicks, R.CurrentTrackingGapTicks);
        }
        R.bWasTracked = false;
    }

    if (Danger > DangerThreshold && !bTracked)
    {
        ++R.UntrackedDangerTicks;
        R.WeightedUntrackedExposure += Danger;
    }
}

void FSimMetricsRecorder::RecordBeliefError(float InMeanBeliefError)
{
    if (!bActive)
    {
        return;
    }
    MeanBeliefError = FMath::Max(0.0f, InMeanBeliefError);
}

void FSimMetricsRecorder::RecordYoloFrame(int32 TruePositives, int32 FalsePositives, int32 Missed,
                                          int32 PersonsInView, int32 NonPersonDetections, int32 Tick, int32 DroneIndex)
{
    if (!bActive)
    {
        return;
    }
    YoloTruePositives += FMath::Max(0, TruePositives);
    YoloFalsePositives += FMath::Max(0, FalsePositives);
    YoloMissed += FMath::Max(0, Missed);
    YoloPersonsInView += FMath::Max(0, PersonsInView);
    YoloNonPersonDetections += FMath::Max(0, NonPersonDetections);
    ++YoloFrames;
    YoloFramesBuffer += FString::Printf(TEXT("%d,%d,%d,%d,%d,%d,%d\n"), Tick, DroneIndex,
        TruePositives, FalsePositives, Missed, PersonsInView, NonPersonDetections);
}

void FSimMetricsRecorder::RecordPhaseTiming(int32 Tick, const FString& PhaseName, float ElapsedMs)
{
    if (!bActive || PhaseName.IsEmpty())
    {
        return;
    }

    if (!bPhaseTimingsFileInitialized)
    {
        WriteToFile(PhaseTimingsPath, TEXT("tick,phase,elapsed_ms\n"));
        bPhaseTimingsFileInitialized = true;
    }

    const FString SafePhase = SanitizePhaseCsvToken(PhaseName);
    const float SafeElapsedMs = FMath::Max(0.0f, ElapsedMs);
    PhaseTimingsBuffer += FString::Printf(TEXT("%d,%s,%.6f\n"),
        Tick, *SafePhase, SafeElapsedMs);

    FSimPhaseTimingAccumulator& Acc = PhaseTimingByName.FindOrAdd(SafePhase);
    Acc.TotalMs += SafeElapsedMs;
    Acc.MaxMs = FMath::Max(Acc.MaxMs, SafeElapsedMs);
    ++Acc.Samples;
}

void FSimMetricsRecorder::RecordEvent(int32 Tick, const FString& EventType, const FString& Detail)
{
    if (!bActive) return;
    // Commas in Detail would break CSV parsing — replace defensively.
    FString SafeDetail = Detail.Replace(TEXT(","), TEXT(";"));
    EventsBuffer += FString::Printf(TEXT("%d,%s,%s\n"), Tick, *EventType, *SafeDetail);
}

void FSimMetricsRecorder::EndRun(int32 FinalTick)
{
    if (!bActive) return;

    Flush();

    const FString SummaryPath = FPaths::Combine(RunDir, TEXT("summary.csv"));
    TArray<int32> Latencies;
    int32 RecallHits = 0;
    float TotalWeightedExposure = 0.0f;
    int32 TotalUntrackedDangerTicks = 0;
    int32 TotalReacquisitions = 0;
    int32 TotalTrackingGaps = 0;
    int32 TotalTrackingGapTicks = 0;
    int32 MaxTrackingGapTicks = 0;
    FString PersonsCsv;
    FString PersonGeometricSummaryCsv;
    for (const FPersonRecord& R : PersonRecords)
    {
        const int32 Latency = (R.FirstDetectTick >= 0 && R.AppearTick >= 0)
            ? FMath::Max(0, R.FirstDetectTick - R.AppearTick)
            : -1;
        if (Latency >= 0)
        {
            Latencies.Add(Latency);
            if (Latency <= RecallDeadlineTicks)
            {
                ++RecallHits;
            }
        }
        TotalWeightedExposure += R.WeightedUntrackedExposure;
        TotalUntrackedDangerTicks += R.UntrackedDangerTicks;
        TotalReacquisitions += R.NumReacquisitions;
        TotalTrackingGaps += R.NumTrackingGaps;
        TotalTrackingGapTicks += R.TotalTrackingGapTicks;
        MaxTrackingGapTicks = FMath::Max(MaxTrackingGapTicks, R.MaxTrackingGapTicks);
        PersonsCsv += FString::Printf(TEXT("%d,%d,%d,%d,%d,%d,%d,%d,%d,%.6f\n"),
            R.PersonId,
            R.AppearTick,
            R.FirstDetectTick,
            Latency,
            R.NumReacquisitions,
            R.NumTrackingGaps,
            R.TotalTrackingGapTicks,
            R.MaxTrackingGapTicks,
            R.UntrackedDangerTicks,
            R.WeightedUntrackedExposure);
        if (bPersonGeometricRecoveryEnabled)
        {
            const int32 ProxyLatency = (R.FirstDetectTick >= 0 && R.AppearTick >= 0)
                ? FMath::Max(0, R.FirstDetectTick - R.AppearTick) : -1;
            const int32 GeometricProcessingLatency = (R.FirstGeometricProcessingTick >= 0 && R.AppearTick >= 0)
                ? FMath::Max(0, R.FirstGeometricProcessingTick - R.AppearTick) : -1;
            const int32 GeometricCaptureLatency = (R.FirstGeometricCaptureTick >= 0 && R.AppearTick >= 0)
                ? FMath::Max(0, R.FirstGeometricCaptureTick - R.AppearTick) : -1;
            const bool bProxyWithoutGeometric = R.FirstDetectTick >= 0 && R.FirstGeometricProcessingTick < 0;
            const bool bProxyBeforeGeometric = R.FirstDetectTick >= 0 && R.FirstGeometricProcessingTick >= 0 &&
                R.FirstDetectTick < R.FirstGeometricProcessingTick;
            PersonGeometricSummaryCsv += FString::Printf(
                TEXT("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"),
                R.PersonId, R.AppearTick, R.FirstDetectTick, ProxyLatency,
                R.FirstGeometricProcessingTick, R.FirstGeometricCaptureTick,
                GeometricProcessingLatency, GeometricCaptureLatency, R.GeometricMatchCount,
                bProxyWithoutGeometric ? 1 : 0, bProxyBeforeGeometric ? 1 : 0);
        }
    }
    if (!PersonsCsv.IsEmpty())
    {
        AppendToFile(PersonsPath, PersonsCsv);
    }
    if (bPersonGeometricRecoveryEnabled && !PersonGeometricSummaryCsv.IsEmpty())
    {
        AppendToFile(PersonGeometricSummaryPath, PersonGeometricSummaryCsv);
    }
    if (bMixedObservationRecoveryEnabled)
    {
        int32 Ideal = 0, Camera = 0, Both = 0, IdealOnly = 0, CameraOnly = 0, Neither = 0;
        FString MixedPeople = TEXT("person_id,appear_tick,ideal_witness_event_count,first_ideal_sensing_tick,first_ideal_snapshot_tick,camera_geometric_match_count,first_camera_processing_tick,first_camera_capture_tick,first_camera_snapshot_tick,first_team_availability_tick,first_team_source,availability_latency_ticks,overlap,ideal_only,camera_only,neither,metric_version\n");
        for (const FPersonRecord& R : PersonRecords)
        {
            const bool bIdeal = R.FirstIdealSensingTick >= 0;
            const bool bCamera = R.FirstGeometricProcessingTick >= 0;
            const int32 FirstAvailability = bIdeal && bCamera
                ? FMath::Min(R.FirstIdealSensingTick, R.FirstGeometricProcessingTick)
                : (bIdeal ? R.FirstIdealSensingTick : (bCamera ? R.FirstGeometricProcessingTick : -1));
            const TCHAR* Source = !bIdeal && !bCamera ? TEXT("none") :
                (bIdeal && bCamera && R.FirstIdealSensingTick == R.FirstGeometricProcessingTick ? TEXT("tie") :
                (bIdeal && (!bCamera || R.FirstIdealSensingTick < R.FirstGeometricProcessingTick) ? TEXT("ideal") : TEXT("camera")));
            const int32 Latency = FirstAvailability >= 0 && R.AppearTick >= 0 ? FMath::Max(0, FirstAvailability - R.AppearTick) : -1;
            Ideal += bIdeal ? 1 : 0; Camera += bCamera ? 1 : 0; Both += bIdeal && bCamera ? 1 : 0;
            IdealOnly += bIdeal && !bCamera ? 1 : 0; CameraOnly += bCamera && !bIdeal ? 1 : 0; Neither += !bIdeal && !bCamera ? 1 : 0;
            MixedPeople += FString::Printf(TEXT("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%d,%d,%d,%d,%d,heterogeneous_sensor_witness_coverage_v1\n"),
                R.PersonId, R.AppearTick, R.IdealWitnessEventCount, R.FirstIdealSensingTick, R.FirstIdealSnapshotTick,
                R.GeometricMatchCount, R.FirstGeometricProcessingTick, R.FirstGeometricCaptureTick,
                R.FirstGeometricProcessingTick >= 0 ? R.FirstGeometricProcessingTick - 1 : -1,
                FirstAvailability, Source, Latency, bIdeal && bCamera ? 1 : 0, bIdeal && !bCamera ? 1 : 0,
                bCamera && !bIdeal ? 1 : 0, !bIdeal && !bCamera ? 1 : 0);
        }
        WriteToFile(MixedObservationPeoplePath, MixedPeople);
        const FString MixedSummary = FString::Printf(
            TEXT("total_people,ideal_witnessed_persons,camera_geometric_persons,overlap_persons,ideal_only,camera_only,neither,union_persons,availability_latency_basis,metric_version\n%d,%d,%d,%d,%d,%d,%d,%d,first source availability minus appear tick (ideal sensing tick; camera processing tick),heterogeneous_sensor_witness_coverage_v1\n"),
            PersonCount, Ideal, Camera, Both, IdealOnly, CameraOnly, Neither, Ideal + Camera - Both);
        WriteToFile(MixedObservationSummaryPath, MixedSummary);
    }
    if (bIdealSensorWitnessRecordingEnabled)
    {
        int32 Witnessed = 0;
        FString People = TEXT("person_id,appear_tick,ideal_witness_event_count,first_ideal_sensing_tick,first_ideal_snapshot_tick,metric_version\n");
        for (const FPersonRecord& R : PersonRecords)
        {
            const bool bWitnessed = R.FirstIdealSensingTick >= 0;
            Witnessed += bWitnessed ? 1 : 0;
            People += FString::Printf(TEXT("%d,%d,%d,%d,%d,ideal_gate_witness_v1\n"),
                R.PersonId, R.AppearTick, R.IdealWitnessEventCount,
                R.FirstIdealSensingTick, R.FirstIdealSnapshotTick);
        }
        WriteToFile(IdealWitnessPeoplePath, People);
        const FString Summary = FString::Printf(
            TEXT("total_people,ideal_witnessed_persons,coverage,metric_version\n%d,%d,%.8f,ideal_gate_witness_v1\n"),
            PersonCount, Witnessed, PersonCount > 0 ? static_cast<double>(Witnessed) / PersonCount : 0.0);
        WriteToFile(IdealWitnessSummaryPath, Summary);
    }
    Latencies.Sort();
    const float MedianLatency = Latencies.Num() > 0
        ? static_cast<float>(Latencies[Latencies.Num() / 2])
        : 0.0f;

    FString TimingHeader;
    FString TimingValues;
    FString PhaseSummaryCsv = TEXT("phase,samples,total_ms,avg_ms,max_ms\n");
    bool bHasPhaseSummary = false;
    TArray<FString> PhaseNames;
    PhaseTimingByName.GenerateKeyArray(PhaseNames);
    PhaseNames.Sort();
    for (const FString& PhaseName : PhaseNames)
    {
        const FSimPhaseTimingAccumulator* Acc = PhaseTimingByName.Find(PhaseName);
        if (!Acc || Acc->Samples <= 0)
        {
            continue;
        }
        const FString Suffix = MakePhaseSummarySuffix(PhaseName);
        const double AverageMs = Acc->TotalMs / static_cast<double>(Acc->Samples);
        TimingHeader += FString::Printf(TEXT(",avg_%s_ms,max_%s_ms,samples_%s"),
            *Suffix, *Suffix, *Suffix);
        TimingValues += FString::Printf(TEXT(",%.6f,%.6f,%d"),
            AverageMs, Acc->MaxMs, Acc->Samples);
        PhaseSummaryCsv += FString::Printf(TEXT("%s,%d,%.6f,%.6f,%.6f\n"),
            *SanitizePhaseCsvToken(PhaseName),
            Acc->Samples,
            Acc->TotalMs,
            AverageMs,
            Acc->MaxMs);
        bHasPhaseSummary = true;
    }
    if (bHasPhaseSummary)
    {
        WriteToFile(PhaseSummaryPath, PhaseSummaryCsv);
    }

    FString SummaryHeader =
        TEXT("first_tick,final_tick,total_observations,total_positive_fire_obs,total_positive_person_obs,partition_passes,avg_mean_staleness,last_coverage,total_persons,persons_detected,detection_fraction,mean_detection_latency_ticks,recall_at_T,median_latency,total_reacquisitions,tracking_gap_count,total_tracking_gap_ticks,max_tracking_gap_ticks,total_untracked_danger_ticks,total_untracked_exposure,fire_discovery_tick,mean_staleness,max_staleness,coverage_frac,mean_belief_error,total_distance,workload_imbalance,mean_tick_ms,yolo_frames,yolo_true_positives,yolo_false_positives,yolo_missed,yolo_persons_in_view,yolo_nonperson_detections,yolo_precision,yolo_recall");
    SummaryHeader += TimingHeader;
    const FString SummaryValues = FString::Printf(TEXT("%d,%d,%lld,%lld,%lld,%d,%.4f,%.6f,%d,%d,%.6f,%.2f,%.6f,%.2f,%d,%d,%d,%d,%d,%.6f,%d,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%d,%lld,%lld,%lld,%lld,%lld,%.6f,%.6f"),
        FirstRecordedTick, FinalTick,
        TotalObservations, TotalPositiveFireObs, TotalPositivePersonObs,
        NumPartitionPasses,
        (NumGridStatSamples > 0) ? (SumMeanStaleness / NumGridStatSamples) : 0.0,
        LastCoverageFraction,
        PersonCount, PersonsDetected,
        (PersonCount > 0) ? (static_cast<float>(PersonsDetected) / PersonCount) : 0.0f,
        (PersonsDetected > 0) ? (static_cast<double>(SumDetectionLatency) / PersonsDetected) : 0.0,
        (PersonCount > 0) ? (static_cast<float>(RecallHits) / PersonCount) : 0.0f,
        MedianLatency,
        TotalReacquisitions,
        TotalTrackingGaps,
        TotalTrackingGapTicks,
        MaxTrackingGapTicks,
        TotalUntrackedDangerTicks,
        TotalWeightedExposure,
        FireDiscoveryTick,
        (NumGridStatSamples > 0) ? (SumMeanStaleness / NumGridStatSamples) : 0.0,
        LastMaxStaleness,
        LastCoverageFraction,
        MeanBeliefError,
        TotalDistanceCells,
        LastWorkloadImbalance,
        (NumTickMsSamples > 0) ? (SumTickMs / NumTickMsSamples) : 0.0,
        YoloFrames,
        YoloTruePositives, YoloFalsePositives, YoloMissed, YoloPersonsInView, YoloNonPersonDetections,
        (YoloTruePositives + YoloFalsePositives > 0) ? (static_cast<double>(YoloTruePositives) / static_cast<double>(YoloTruePositives + YoloFalsePositives)) : 0.0,
        (YoloTruePositives + YoloMissed > 0) ? (static_cast<double>(YoloTruePositives) / static_cast<double>(YoloTruePositives + YoloMissed)) : 0.0);
    SummaryHeader += TEXT(",yolo_false_negatives,yolo_f1,yolo_metric_version");
    const double YoloF1 = (2 * YoloTruePositives + YoloFalsePositives + YoloMissed > 0)
        ? (2.0 * YoloTruePositives / (2 * YoloTruePositives + YoloFalsePositives + YoloMissed)) : 0.0;
    const FString YoloMetricValues = FString::Printf(TEXT(",%lld,%.6f,geometric_alive_one_to_one_cell_neighborhood_v2"), YoloMissed, YoloF1);
    const FString Summary = SummaryHeader + TEXT("\n") + SummaryValues + TimingValues + YoloMetricValues + TEXT("\n");
    WriteToFile(SummaryPath, Summary);

    if (bMetricsWriteFailed)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis metrics: run ended at tick %d with a write failure; see %s."),
            FinalTick, *MetricsWriteFailurePath);
    }
    else
    {
        UE_LOG(LogTemp, Log, TEXT("Thesis metrics: run ended at tick %d, summary -> %s"), FinalTick, *SummaryPath);
    }
    bActive = false;
}

void FSimMetricsRecorder::Flush()
{
    if (!TicksBuffer.IsEmpty() && AppendToFile(TicksPath, TicksBuffer)) TicksBuffer.Reset();
    if (!DronesBuffer.IsEmpty() && AppendToFile(DronesPath, DronesBuffer)) DronesBuffer.Reset();
    if (!EventsBuffer.IsEmpty() && AppendToFile(EventsPath, EventsBuffer)) EventsBuffer.Reset();
    if (!PhaseTimingsBuffer.IsEmpty() && AppendToFile(PhaseTimingsPath, PhaseTimingsBuffer)) PhaseTimingsBuffer.Reset();
    if (!YoloFramesBuffer.IsEmpty() && AppendToFile(YoloFramesPath, YoloFramesBuffer)) YoloFramesBuffer.Reset();
    if (!PersonGeometricEventsBuffer.IsEmpty())
    {
        if (AppendToFile(PersonGeometricEventsPath, PersonGeometricEventsBuffer)) PersonGeometricEventsBuffer.Reset();
    }
    if (!MixedObservationEventsBuffer.IsEmpty())
    {
        if (AppendToFile(MixedObservationEventsPath, MixedObservationEventsBuffer)) MixedObservationEventsBuffer.Reset();
    }
    if (!IdealWitnessEventsBuffer.IsEmpty())
    {
        if (AppendToFile(IdealWitnessEventsPath, IdealWitnessEventsBuffer)) IdealWitnessEventsBuffer.Reset();
    }
    if (!bMetricsWriteFailed) TicksSinceFlush = 0;
}

bool FSimMetricsRecorder::AppendToFile(const FString& FilePath, const FString& Content)
{
    return WriteToFile(FilePath, Content, true);
}

bool FSimMetricsRecorder::WriteToFile(const FString& FilePath, const FString& Content, bool bAppend)
{
    if (bMetricsWriteFailed)
    {
        return false;
    }

    // Allow concurrent readers (the study controller tails metrics files) and
    // check the actual append result before the caller releases its buffer.
    const uint32 WriteFlags = (bAppend ? EFileWrite::FILEWRITE_Append : EFileWrite::FILEWRITE_None) |
        EFileWrite::FILEWRITE_AllowRead;
    const bool bSaved = FFileHelper::SaveStringToFile(Content, *FilePath,
        FFileHelper::EEncodingOptions::AutoDetect,
        &IFileManager::Get(),
        WriteFlags);
    if (!bSaved)
    {
        RecordWriteFailure(FilePath, Content.Len());
        return false;
    }
    return true;
}

void FSimMetricsRecorder::RecordWriteFailure(const FString& FilePath, int32 PendingCharacters)
{
    if (bMetricsWriteFailed)
    {
        return;
    }

    bMetricsWriteFailed = true;
    bActive = false;
    const FString Marker = FString::Printf(
        TEXT("status=failed\nfile=%s\npending_characters=%d\nreason=append_failed\n"),
        *FilePath, PendingCharacters);
    const bool bMarkerSaved = FFileHelper::SaveStringToFile(
        Marker, *MetricsWriteFailurePath,
        FFileHelper::EEncodingOptions::AutoDetect,
        &IFileManager::Get(), EFileWrite::FILEWRITE_None | EFileWrite::FILEWRITE_AllowRead);
    UE_LOG(LogTemp, Error,
        TEXT("Thesis metrics: append failed for '%s' (%d pending characters); run metrics are invalid. Failure marker %s."),
        *FilePath, PendingCharacters, bMarkerSaved ? *MetricsWriteFailurePath : TEXT("could not be written"));
}
