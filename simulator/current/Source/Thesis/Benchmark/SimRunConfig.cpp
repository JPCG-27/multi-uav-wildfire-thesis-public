#include "SimRunConfig.h"
#include "SimulationManager.h"
#include "../Drone/PerfectDroneSensor.h"
#include "../Drone/LawnmowerV2Planner.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Policies/PrettyJsonPrintPolicy.h"
#include "Serialization/JsonWriter.h"
#include "UObject/UnrealType.h"

namespace
{
    // Tiny helpers: apply only when the key is present.
    void GetIf(const TSharedPtr<FJsonObject>& O, const TCHAR* Key, int32& Out)
    {
        if (O->HasTypedField<EJson::Number>(Key)) Out = static_cast<int32>(O->GetNumberField(Key));
    }
    void GetIf(const TSharedPtr<FJsonObject>& O, const TCHAR* Key, float& Out)
    {
        if (O->HasTypedField<EJson::Number>(Key)) Out = static_cast<float>(O->GetNumberField(Key));
    }
    void GetIf(const TSharedPtr<FJsonObject>& O, const TCHAR* Key, bool& Out)
    {
        if (O->HasTypedField<EJson::Boolean>(Key)) Out = O->GetBoolField(Key);
    }
    void GetIf(const TSharedPtr<FJsonObject>& O, const TCHAR* Key, FString& Out)
    {
        if (O->HasTypedField<EJson::String>(Key)) Out = O->GetStringField(Key);
    }

    bool GetIntArrayIfPresent(const TSharedPtr<FJsonObject>& O, const TCHAR* Key,
                              bool& bWasPresent, TArray<int32>& Out)
    {
        bWasPresent = O->HasField(Key);
        if (!bWasPresent) return true;
        const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
        if (!O->TryGetArrayField(Key, Values) || !Values) return false;
        Out.Reset(Values->Num());
        for (const TSharedPtr<FJsonValue>& Value : *Values)
        {
            if (!Value.IsValid() || Value->Type != EJson::Number) return false;
            const double Number = Value->AsNumber();
            if (!FMath::IsFinite(Number) || Number < 0.0 || Number > MAX_int32 ||
                Number != FMath::FloorToDouble(Number)) return false;
            Out.Add(static_cast<int32>(Number));
        }
        return true;
    }

    bool GetCellPositionArrayIfPresent(const TSharedPtr<FJsonObject>& O, const TCHAR* Key,
                                      bool& bWasPresent, TArray<FVector2D>& Out)
    {
        bWasPresent = O->HasField(Key);
        if (!bWasPresent) return true;
        const TArray<TSharedPtr<FJsonValue>>* Values = nullptr;
        if (!O->TryGetArrayField(Key, Values) || !Values) return false;
        Out.Reset(Values->Num());
        for (const TSharedPtr<FJsonValue>& Value : *Values)
        {
            if (!Value.IsValid() || Value->Type != EJson::Array) return false;
            const TArray<TSharedPtr<FJsonValue>>& Pair = Value->AsArray();
            if (Pair.Num() != 2 || !Pair[0].IsValid() || !Pair[1].IsValid() ||
                Pair[0]->Type != EJson::Number || Pair[1]->Type != EJson::Number) return false;
            const double X = Pair[0]->AsNumber();
            const double Y = Pair[1]->AsNumber();
            if (!FMath::IsFinite(X) || !FMath::IsFinite(Y)) return false;
            Out.Emplace(static_cast<float>(X), static_cast<float>(Y));
        }
        return true;
    }
}

FString FSimRunConfig::ResolveConfigPath(const FString& InEditorPath)
{
    FString CmdLinePath;
    if (FParse::Value(FCommandLine::Get(), TEXT("SimConfig="), CmdLinePath) && !CmdLinePath.IsEmpty())
    {
        return CmdLinePath;
    }
    return InEditorPath;
}

bool FSimRunConfig::LoadAndApply(const FString& Path, ASimulationManager& M)
{
    FString Resolved = Path;
    if (FPaths::IsRelative(Resolved))
    {
        Resolved = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Resolved);
    }

    FString JsonText;
    if (!FFileHelper::LoadFileToString(JsonText, *Resolved))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis run config: cannot read %s"), *Resolved);
        return false;
    }

    TSharedPtr<FJsonObject> Root;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(JsonText);
    if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis run config: parse error in %s"), *Resolved);
        return false;
    }

    // Missing profile denotes the original sensor, including in batched runs.
    FString RenderProfile = TEXT("legacy");
    GetIf(Root, TEXT("camera_render_profile"), RenderProfile);
    if (RenderProfile != TEXT("legacy") && RenderProfile != TEXT("fast_v1") &&
        RenderProfile != TEXT("fast_unshadowed_v1") && RenderProfile != TEXT("fast_unshadowed_v2") &&
        RenderProfile != TEXT("fast_shader_v3"))
    {
        UE_LOG(LogTemp, Fatal, TEXT("Unknown camera_render_profile '%s'."), *RenderProfile);
        return false;
    }
    M.CameraRenderProfile = RenderProfile;

    // ---- Run / benchmark control ----
    GetIf(Root, TEXT("run_id"), M.MetricsRunId);
    GetIf(Root, TEXT("benchmark_run_subdir"), M.MetricsRunSubdir);
    FString MethodId;
    GetIf(Root, TEXT("method_id"), MethodId);
    if (!MethodId.IsEmpty())
    {
        M.ApplyMethodId(MethodId);
    }
    GetIf(Root, TEXT("max_sim_ticks"), M.MaxSimTicks);
    GetIf(Root, TEXT("exit_on_max_ticks"), M.bExitOnMaxTicks);
    GetIf(Root, TEXT("record_metrics"), M.bRecordMetrics);
    GetIf(Root, TEXT("metrics_detail_mode"), M.MetricsDetailMode);
    GetIf(Root, TEXT("grid_stats_every_n_ticks"), M.GridStatsEveryNTicks);
    GetIf(Root, TEXT("grid_stats_readback_lag_ticks"), M.GridStatsReadbackLagTicks);
    GetIf(Root, TEXT("oracle_lookahead_ticks"), M.OracleLookaheadTicks);
    GetIf(Root, TEXT("recall_deadline_ticks"), M.RecallDeadlineTicks);
    GetIf(Root, TEXT("danger_exposure_threshold"), M.DangerExposureThreshold);
    GetIf(Root, TEXT("tracking_window_ticks"), M.TrackingWindowTicks);
    GetIf(Root, TEXT("record_wall_clock_timing"), M.bRecordWallClockTiming);

    // ---- Grid / sim ----
    GetIf(Root, TEXT("grid_width"), M.GridWidth);
    GetIf(Root, TEXT("grid_height"), M.GridHeight);
    GetIf(Root, TEXT("cell_size_uu"), M.CellSizeUU);
    GetIf(Root, TEXT("max_ticks_per_second"), M.MaxTicksPerSecond);
    GetIf(Root, TEXT("suppress_benchmark_viewport"), M.bSuppressBenchmarkViewport);
    GetIf(Root, TEXT("use_nanite_for_live_view"), M.bUseNaniteForLiveView);
    GetIf(Root, TEXT("seconds_per_tick"), M.SecondsPerTick);

    // ---- Fire ----
    FString FireCsv;
    GetIf(Root, TEXT("fire_csv_path"), FireCsv);
    if (!FireCsv.IsEmpty()) M.FirstFireTickCSVPath.FilePath = FireCsv;
    FString GeneratedFireCsv;
    GetIf(Root, TEXT("generated_fire_csv_output_path"), GeneratedFireCsv);
    if (!GeneratedFireCsv.IsEmpty()) M.GeneratedFireCsvOutputPath.FilePath = GeneratedFireCsv;
    GetIf(Root, TEXT("reuse_generated_scenario_csvs"), M.bReuseGeneratedScenarioCsvs);
    GetIf(Root, TEXT("fire_seed"), M.FireGeneratorSeed);
    GetIf(Root, TEXT("generate_fire_csv"), M.bGenerateFireCsvOnBeginPlay);
    GetIf(Root, TEXT("arrival_minutes_per_tick"), M.ArrivalMinutesPerTick);
    GetIf(Root, TEXT("burn_duration_ticks"), M.BurnDurationTicks);
    GetIf(Root, TEXT("fire_generator_ignition_count"), M.FireGeneratorIgnitionCount);
    GetIf(Root, TEXT("fire_generator_num_focus_points"), M.FireGeneratorNumFocusPoints);
    GetIf(Root, TEXT("fire_generator_spread_variant"), M.FireGeneratorSpreadVariant);
    GetIf(Root, TEXT("fire_generator_first_ignition_tick"), M.FireGeneratorFirstIgnitionTick);
    GetIf(Root, TEXT("fire_generator_additional_ignition_start_min_tick"), M.FireGeneratorAdditionalIgnitionStartMinTick);
    GetIf(Root, TEXT("fire_generator_additional_ignition_start_max_tick"), M.FireGeneratorAdditionalIgnitionStartMaxTick);
    GetIf(Root, TEXT("fire_generator_burn_percentage"), M.FireGeneratorBurnPercentage);
    GetIf(Root, TEXT("fire_generator_max_minutes"), M.FireGeneratorMaxMinutes);
    GetIf(Root, TEXT("fire_generator_spread_minutes_per_cell"), M.FireGeneratorSpreadMinutesPerCell);
    GetIf(Root, TEXT("fire_generator_spread_ticks_per_cell"), M.FireGeneratorSpreadTicksPerCell);
    GetIf(Root, TEXT("fire_generator_growth_acceleration"), M.FireGeneratorGrowthAcceleration);
    GetIf(Root, TEXT("fire_generator_terrain_cost_influence"), M.FireGeneratorTerrainCostInfluence);
    GetIf(Root, TEXT("fire_generator_spotting_chance"), M.FireGeneratorSpottingChance);
    GetIf(Root, TEXT("fire_generator_default_duration_ticks"), M.FireGeneratorDefaultDurationTicks);
    GetIf(Root, TEXT("fire_generator_auto_duration_ticks_per_burning_cell"), M.FireGeneratorAutoDurationTicksPerBurningCell);
    GetIf(Root, TEXT("fire_generator_duration_fill_fraction"), M.FireGeneratorDurationFillFraction);

    // ---- Terrain ----
    GetIf(Root, TEXT("auto_generate_terrain"), M.bAutoGenerateTerrain);
    GetIf(Root, TEXT("terrain_slope_intensity"), M.TerrainSlopeIntensity);
    GetIf(Root, TEXT("terrain_detail_density"), M.TerrainDetailDensity);
    GetIf(Root, TEXT("terrain_presentation_margin_meters"), M.TerrainPresentationMarginMeters);
    GetIf(Root, TEXT("render_terrain_visuals"), M.bRenderTerrainVisuals);
    FString GeneratedTerrainCsv;
    GetIf(Root, TEXT("generated_terrain_csv_output_path"), GeneratedTerrainCsv);
    if (!GeneratedTerrainCsv.IsEmpty()) M.GeneratedTerrainCsvOutputPath.FilePath = GeneratedTerrainCsv;
    FString GeneratedFuelCsv;
    GetIf(Root, TEXT("generated_fuel_csv_output_path"), GeneratedFuelCsv);
    if (!GeneratedFuelCsv.IsEmpty()) M.GeneratedFuelCsvOutputPath.FilePath = GeneratedFuelCsv;
    FString TerrainProfileStr;
    GetIf(Root, TEXT("terrain_profile"), TerrainProfileStr);
    if (TerrainProfileStr.Equals(TEXT("flat"), ESearchCase::IgnoreCase))           M.TerrainProfile = ETerrainProfile::Flat;
    else if (TerrainProfileStr.Equals(TEXT("rolling"), ESearchCase::IgnoreCase) ||
             TerrainProfileStr.Equals(TEXT("rolling_hills"), ESearchCase::IgnoreCase)) M.TerrainProfile = ETerrainProfile::RollingHills;
    else if (TerrainProfileStr.Equals(TEXT("foothills"), ESearchCase::IgnoreCase)) M.TerrainProfile = ETerrainProfile::Foothills;
    else if (TerrainProfileStr.Equals(TEXT("mountains"), ESearchCase::IgnoreCase)) M.TerrainProfile = ETerrainProfile::Mountains;

    // ---- Humans ----
    FString HumansCsv;
    GetIf(Root, TEXT("humans_csv_path"), HumansCsv);
    if (!HumansCsv.IsEmpty())
    {
        M.HumansCSVPath.FilePath = HumansCsv;
        // An explicit scenario was supplied — don't overwrite it with auto-generation.
        M.bAutoGenerateScenarioOnBeginPlay = false;
    }
    FString GeneratedHumansCsv;
    GetIf(Root, TEXT("generated_humans_csv_output_path"), GeneratedHumansCsv);
    if (!GeneratedHumansCsv.IsEmpty()) M.GeneratedHumansCsvOutputPath.FilePath = GeneratedHumansCsv;
    GetIf(Root, TEXT("auto_generate_scenario"), M.bAutoGenerateScenarioOnBeginPlay);
    GetIf(Root, TEXT("human_count"), M.HumanGeneratorCount);
    GetIf(Root, TEXT("human_max_ticks"), M.HumanGeneratorMaxTicks);
    GetIf(Root, TEXT("human_seed"), M.HumanGeneratorSeed);
    GetIf(Root, TEXT("human_mesh_scale"), M.HumanMeshScale);
    GetIf(Root, TEXT("human_visual_mesh_set"), M.HumanVisualMeshSet);
    GetIf(Root, TEXT("human_pose_mode"), M.HumanPoseMode);

    // ---- Vision / captures ----
    GetIf(Root, TEXT("enable_drone_captures"), M.bEnableDroneCaptures);
    GetIf(Root, TEXT("enable_camera_state_audit"), M.bEnableCameraStateAudit);
    GetIf(Root, TEXT("enable_yolo_dataset_export"), M.bEnableYoloDatasetExport);
    GetIf(Root, TEXT("drone_capture_clear_air"), M.bDroneCaptureClearAir);
    GetIf(Root, TEXT("yolo_dataset_id"), M.YoloDatasetId);
    GetIf(Root, TEXT("scenario_family_id"), M.ScenarioFamilyId);
    GetIf(Root, TEXT("yolo_dataset_output_dir"), M.YoloDatasetOutputDir);
    GetIf(Root, TEXT("yolo_view_source"), M.YoloViewSource);
    GetIf(Root, TEXT("num_real_camera_drones"), M.NumRealCameraDrones);
    GetIf(Root, TEXT("capture_every_n_ticks"), M.DroneCaptureEveryNTicks);
    GetIf(Root, TEXT("capture_resolution"), M.DroneCaptureResolution);
    GetIf(Root, TEXT("save_drone_capture_pngs"), M.bSaveDroneCapturePngs);
    GetIf(Root, TEXT("drone_capture_pitch_degrees"), M.DroneCapturePitchDegrees);
    GetIf(Root, TEXT("drone_capture_auto_exposure"), M.bDroneCaptureAutoExposure);
    GetIf(Root, TEXT("drone_capture_exposure_bias"), M.DroneCaptureExposureBias);
    GetIf(Root, TEXT("drone_capture_manual_ev100"), M.DroneCaptureManualEV100);
    GetIf(Root, TEXT("drone_capture_bloom_intensity"), M.DroneCaptureBloomIntensity);
    GetIf(Root, TEXT("fire_visuals_only_for_yolo_captures"), M.bFireVisualsOnlyForYoloCaptures);
    GetIf(Root, TEXT("enable_fire_visuals"), M.bEnableFireVisuals);
    GetIf(Root, TEXT("fire_visuals_yolo_capture_radius_uu"), M.FireVisualsYoloCaptureRadiusUU);
    GetIf(Root, TEXT("fire_visuals_flame_width_scale"), M.FireVisualsFlameWidthScaleMultiplier);
    GetIf(Root, TEXT("fire_visuals_flame_height_scale"), M.FireVisualsFlameHeightScaleMultiplier);
    GetIf(Root, TEXT("fire_visuals_flame_sprite_size_scale"), M.FireVisualsBundledFlameSpriteSizeMultiplier);
    GetIf(Root, TEXT("fire_visuals_smoke_scale"), M.FireVisualsSmokeVfxScaleMultiplier);
    GetIf(Root, TEXT("fire_visuals_smoke_layers"), M.FireVisualsSmokeLayerCount);
    GetIf(Root, TEXT("fire_visuals_share_smoke_across_cell_clusters"), M.bFireVisualsShareSmokeAcrossCellClusters);
    GetIf(Root, TEXT("fire_visuals_enable_bundled_heat_distortion"), M.bFireVisualsEnableBundledHeatDistortion);
    GetIf(Root, TEXT("fire_visuals_connected_fire_front"), M.bFireVisualsConnectedFireFront);
    GetIf(Root, TEXT("fire_visuals_max_lights"), M.FireVisualsMaxLights);
    GetIf(Root, TEXT("fire_visuals_light_radius_uu"), M.FireVisualsLightRadiusUU);
    GetIf(Root, TEXT("fire_visuals_max_shadow_casting_lights"), M.FireVisualsMaxShadowCastingLights);
    GetIf(Root, TEXT("fire_visuals_smoke_z_offset_uu"), M.FireVisualsSmokeZOffsetUU);
    GetIf(Root, TEXT("fire_visuals_smoke_layer_z_step_uu"), M.FireVisualsSmokeLayerZStepUU);
    GetIf(Root, TEXT("enable_vision_bridge"), M.bEnableVisionBridge);
    M.bEnableCachedDetectionChanceNull = false;
    M.bCachedDetectionChanceNullPilot = true;
    M.CachedDetectionChanceNullCohort = TEXT("mixed1");
    GetIf(Root, TEXT("enable_cached_detection_chance_null"), M.bEnableCachedDetectionChanceNull);
    GetIf(Root, TEXT("cached_detection_chance_null_pilot"), M.bCachedDetectionChanceNullPilot);
    GetIf(Root, TEXT("cached_detection_chance_null_cohort"), M.CachedDetectionChanceNullCohort);
    if (M.bEnableCachedDetectionChanceNull &&
        M.CachedDetectionChanceNullCohort != TEXT("mixed1") &&
        M.CachedDetectionChanceNullCohort != TEXT("all_yolo"))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis run config: cached_detection_chance_null_cohort must be 'mixed1' or 'all_yolo'."));
        return false;
    }
    GetIf(Root, TEXT("vision_server_host"), M.VisionServerHost);
    GetIf(Root, TEXT("vision_server_port"), M.VisionServerPort);
    GetIf(Root, TEXT("vision_cache_dir"), M.VisionCacheDirName);
    FString CacheModeStr;
    GetIf(Root, TEXT("vision_cache_mode"), CacheModeStr);
    if (CacheModeStr.Equals(TEXT("off"), ESearchCase::IgnoreCase))         M.VisionCacheMode = EVisionCacheMode::Off;
    else if (CacheModeStr.Equals(TEXT("record"), ESearchCase::IgnoreCase)) M.VisionCacheMode = EVisionCacheMode::Record;
    else if (CacheModeStr.Equals(TEXT("replay"), ESearchCase::IgnoreCase)) M.VisionCacheMode = EVisionCacheMode::Replay;
    GetIf(Root, TEXT("save_yolo_debug_images"), M.bSaveYoloDebugImages);
    GetIf(Root, TEXT("yolo_max_error_images"), M.YoloMaxErrorImages);
    GetIf(Root, TEXT("yolo_max_correct_images"), M.YoloMaxCorrectImages);
    GetIf(Root, TEXT("yolo_person_confidence_threshold"), M.YoloPersonConfidenceThreshold);

    // ---- Drones ----
    GetIf(Root, TEXT("drone_count"), M.AutoDroneCount);
    M.bBenchmarkRecordIdealSensorWitnesses = false;
    GetIf(Root, TEXT("record_ideal_sensor_witnesses"), M.bBenchmarkRecordIdealSensorWitnesses);
    M.BenchmarkDroneLogicalIds.Reset();
    M.BenchmarkDroneSpawnCellPositions.Reset();
    bool bHasLogicalIds = false;
    bool bHasSpawnCells = false;
    if (!GetIntArrayIfPresent(Root, TEXT("drone_logical_ids"), bHasLogicalIds, M.BenchmarkDroneLogicalIds) ||
        !GetCellPositionArrayIfPresent(Root, TEXT("drone_spawn_cell_positions"), bHasSpawnCells, M.BenchmarkDroneSpawnCellPositions))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis run config: drone_logical_ids must be nonnegative integers and drone_spawn_cell_positions must be finite [x,y] pairs."));
        return false;
    }
    if ((bHasLogicalIds || bHasSpawnCells) &&
        (M.BenchmarkDroneLogicalIds.Num() != M.AutoDroneCount ||
         M.BenchmarkDroneSpawnCellPositions.Num() != M.AutoDroneCount))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis run config: drone_logical_ids and drone_spawn_cell_positions must both contain exactly drone_count entries."));
        return false;
    }
    if (bHasLogicalIds)
    {
        TSet<int32> UniqueIds;
        for (const int32 Id : M.BenchmarkDroneLogicalIds)
        {
            if (UniqueIds.Contains(Id))
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis run config: drone_logical_ids contains duplicate ID %d."), Id);
                return false;
            }
            UniqueIds.Add(Id);
        }

        TSet<FIntPoint> UniqueSpawnCells;
        for (const FVector2D& Cell : M.BenchmarkDroneSpawnCellPositions)
        {
            if (Cell.X < 0.5 || Cell.Y < 0.5 ||
                Cell.X > static_cast<double>(M.GridWidth) - 0.5 ||
                Cell.Y > static_cast<double>(M.GridHeight) - 0.5)
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis run config: identity-mapped spawn cell (%.6f, %.6f) is outside the grid."), Cell.X, Cell.Y);
                return false;
            }
            const FIntPoint Quantized(FMath::RoundToInt(Cell.X * 1000000.0),
                                      FMath::RoundToInt(Cell.Y * 1000000.0));
            if (UniqueSpawnCells.Contains(Quantized))
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis run config: identity-mapped spawn positions must be unique."));
                return false;
            }
            UniqueSpawnCells.Add(Quantized);
        }
    }
    GetIf(Root, TEXT("drone_speed_uu_per_tick"), M.DroneSpeedUUPerTick);
    GetIf(Root, TEXT("parallel_drone_processing"), M.bParallelDroneProcessing);
    FString DroneProcessingMode;
    GetIf(Root, TEXT("drone_processing_mode"), DroneProcessingMode);
    if (DroneProcessingMode.Equals(TEXT("parallel_cpu"), ESearchCase::IgnoreCase))
    {
        M.bParallelDroneProcessing = true;
    }
    else if (DroneProcessingMode.Equals(TEXT("sequential"), ESearchCase::IgnoreCase))
    {
        M.bParallelDroneProcessing = false;
    }
    // Async drone-local GPU priority is fence-timed and not bit-reproducible; it defaults
    // off. Exposed here so the determinism gate / benchmark runs can force the deterministic
    // CPU fallback explicitly. The manager also force-disables it whenever parallel
    // processing is active (worker threads cannot consume the GPU readback).
    GetIf(Root, TEXT("use_drone_local_gpu_priority"), M.bDroneUseLocalGpuPriority);
    GetIf(Root, TEXT("drone_fov_degrees"), M.DroneHorizontalFOVDegrees);
    GetIf(Root, TEXT("drone_max_view_distance_uu"), M.DroneMaxViewDistanceUU);
    GetIf(Root, TEXT("drone_replan_cadence_ticks"), M.DroneReplanCadenceTicks);
    GetIf(Root, TEXT("drone_commitment_margin"), M.DroneCommitmentMargin);
    FString PlannerScoringMode = M.DronePlannerScoringMode;
    GetIf(Root, TEXT("drone_planner_scoring_mode"), PlannerScoringMode);
    if (PlannerScoringMode.Equals(TEXT("full_fov"), ESearchCase::IgnoreCase))
    {
        M.DronePlannerScoringMode = TEXT("full_fov");
    }
    else if (PlannerScoringMode.Equals(TEXT("target_cell"), ESearchCase::IgnoreCase))
    {
        M.DronePlannerScoringMode = TEXT("target_cell");
    }
    else
    {
        UE_LOG(LogTemp, Fatal, TEXT("Unknown drone_planner_scoring_mode '%s' (expected 'full_fov' or 'target_cell')."), *PlannerScoringMode);
        return false;
    }
    FString PlannerStrategy;
    GetIf(Root, TEXT("planner_strategy"), PlannerStrategy);
    if (!PlannerStrategy.IsEmpty()) M.PlannerStrategyName = PlannerStrategy;

    // ---- Partition ----
    FString RegionRegime;
    GetIf(Root, TEXT("region_regime"), RegionRegime);
    if (!RegionRegime.IsEmpty()) M.RegionRegimeName = RegionRegime;
    FString InformationSource;
    GetIf(Root, TEXT("information_source"), InformationSource);
    if (!InformationSource.IsEmpty()) M.InformationSourceName = InformationSource;
    FString PartitionStrategy;
    GetIf(Root, TEXT("partition_strategy"), PartitionStrategy);
    if (!PartitionStrategy.IsEmpty()) M.PartitionStrategyName = PartitionStrategy;
    GetIf(Root, TEXT("partition_recompute_every_n_ticks"), M.PartitionRecomputeEveryNTicks);
    GetIf(Root, TEXT("confirmed_detection_threshold"), M.ConfirmedDetectionThreshold);
    GetIf(Root, TEXT("partition_weight_confirmed"), M.PartitionWeightConfirmed);
    GetIf(Root, TEXT("partition_weight_expected"), M.PartitionWeightExpected);
    GetIf(Root, TEXT("partition_weight_normal"), M.PartitionWeightNormal);
    GetIf(Root, TEXT("background_tile_fraction"), M.BackgroundTileFraction);
    GetIf(Root, TEXT("max_fire_zone_fraction"), M.MaxFireZoneFraction);

    // ---- Priority model ----
    GetIf(Root, TEXT("priority_weight_staleness"), M.PriorityWeightStaleness);
    GetIf(Root, TEXT("priority_weight_danger"), M.PriorityWeightDanger);
    GetIf(Root, TEXT("priority_weight_person"), M.PriorityWeightPerson);
    GetIf(Root, TEXT("priority_staleness_saturation_ticks"), M.PriorityStalenessSaturationTicks);
    GetIf(Root, TEXT("priority_unseen_exploration_bonus"), M.PriorityUnseenExplorationBonus);
    GetIf(Root, TEXT("load_weight_staleness"), M.LoadWeightStaleness);
    GetIf(Root, TEXT("load_weight_danger"), M.LoadWeightDanger);
    GetIf(Root, TEXT("load_weight_person"), M.LoadWeightPerson);

    // ---- Sensor LLRs ----
    GetIf(Root, TEXT("sensor_fire_positive_llr"), M.SensorFirePositiveLLR);
    GetIf(Root, TEXT("sensor_fire_negative_llr"), M.SensorFireNegativeLLR);
    GetIf(Root, TEXT("sensor_person_positive_llr"), M.SensorPersonPositiveLLR);
    GetIf(Root, TEXT("sensor_person_negative_llr"), M.SensorPersonNegativeLLR);

    FString FloraDensity;
    GetIf(Root, TEXT("flora_density"), FloraDensity);
    if (FloraDensity.Equals(TEXT("none"), ESearchCase::IgnoreCase) ||
        FloraDensity.Equals(TEXT("zero"), ESearchCase::IgnoreCase))
    {
        M.ForestCoverage = 0.0f;
        M.ForestMaxTreesPerCell = 0;
        M.ForestMaxBushesPerCell = 0;
        M.ForestGroundCoverPerCell = 0;
        M.ForestSurfaceDebrisPerCell = 0;
        M.bForestAutoPopulate = false;
    }
    else if (FloraDensity.Equals(TEXT("sparse"), ESearchCase::IgnoreCase))
    {
        M.ForestCoverage = 0.55f;
        M.ForestMaxTreesPerCell = 1;
        M.ForestMaxBushesPerCell = 2;
        M.ForestGroundCoverPerCell = 2;
        M.ForestSurfaceDebrisPerCell = 1;
    }
    else if (FloraDensity.Equals(TEXT("medium"), ESearchCase::IgnoreCase))
    {
        M.ForestCoverage = 0.78f;
        M.ForestMaxTreesPerCell = 1;
        M.ForestMaxBushesPerCell = 3;
        M.ForestGroundCoverPerCell = 3;
        M.ForestSurfaceDebrisPerCell = 2;
    }
    else if (FloraDensity.Equals(TEXT("dense"), ESearchCase::IgnoreCase))
    {
        M.ForestCoverage = 0.92f;
        M.ForestMaxTreesPerCell = 2;
        M.ForestMaxBushesPerCell = 5;
        M.ForestGroundCoverPerCell = 5;
        M.ForestSurfaceDebrisPerCell = 3;
    }

    M.RefreshMethodSpecFromNames();

    UE_LOG(LogTemp, Log, TEXT("Thesis run config: applied %s"), *Resolved);
    return true;
}

FString FSimRunConfig::MakeConfigSnapshot(const ASimulationManager& M)
{
    const TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();

    Root->SetStringField(TEXT("run_id"), M.MetricsRunId);
    Root->SetStringField(TEXT("benchmark_run_subdir"), M.MetricsRunSubdir);
    Root->SetStringField(TEXT("method_id"), M.MethodId);
    Root->SetStringField(TEXT("information_source"), M.InformationSourceName);
    Root->SetStringField(TEXT("region_regime"), M.RegionRegimeName);
    Root->SetStringField(TEXT("planner_strategy"), M.PlannerStrategyName);
    if (M.PlannerStrategyName.Equals(TEXT("LawnmowerV1Aligned"), ESearchCase::IgnoreCase))
    {
        Root->SetNumberField(TEXT("lawnmower_v1_aligned_algorithm_version"), 1);
        Root->SetNumberField(TEXT("lawnmower_v1_aligned_lane_spacing_cells"), 1);
        Root->SetStringField(TEXT("lawnmower_v1_aligned_scan_order"), TEXT("spawn_side_inward_one_cell_serpentine"));
        Root->SetStringField(TEXT("lawnmower_v1_aligned_initial_waypoint"), TEXT("owned_spawn_cell_direct_flight"));
        Root->SetStringField(TEXT("lawnmower_v1_aligned_waypoint_policy"), TEXT("arrival_only_direct_flight_v1_progress"));
        Root->SetStringField(TEXT("lawnmower_v1_aligned_static_partition"), TEXT("SpawnAwareUniform"));
        Root->SetStringField(TEXT("sensor_class"), M.DroneSensorClass
            ? M.DroneSensorClass->GetPathName()
            : UPerfectDroneSensor::StaticClass()->GetPathName());
    }
    if (M.PlannerStrategyName.Equals(TEXT("LawnmowerV1"), ESearchCase::IgnoreCase))
    {
        Root->SetNumberField(TEXT("lawnmower_v1_algorithm_version"), 1);
        Root->SetNumberField(TEXT("lawnmower_v1_lane_spacing_cells"), 1);
        Root->SetStringField(TEXT("lawnmower_v1_scan_order"), TEXT("rows_ascending_x_alternating"));
        Root->SetStringField(TEXT("lawnmower_v1_initial_waypoint"), TEXT("first_owned_cell_in_scan_order"));
        Root->SetStringField(TEXT("lawnmower_v1_waypoint_policy"), TEXT("arrival_only_direct_flight"));
        Root->SetStringField(TEXT("sensor_class"), M.DroneSensorClass
            ? M.DroneSensorClass->GetPathName()
            : UPerfectDroneSensor::StaticClass()->GetPathName());
    }
    if (M.PlannerStrategyName.Equals(TEXT("LawnmowerV2"), ESearchCase::IgnoreCase))
    {
        Root->SetNumberField(TEXT("lawnmower_v2_algorithm_version"), 2);
        Root->SetNumberField(TEXT("lawnmower_v2_lane_spacing_cells"), ULawnmowerV2Planner::ComputeLaneStride(
            M.DroneMaxViewDistanceUU / FMath::Max(1.0f, M.CellSizeUU), M.DroneHorizontalFOVDegrees, M.GridWidth, M.GridHeight));
        Root->SetStringField(TEXT("lawnmower_v2_lane_spacing_rule"), TEXT("max1_twice_floor_range_cells_sin_half_fov"));
        Root->SetStringField(TEXT("lawnmower_v2_scan_order"), TEXT("spawn_side_inward_global_lanes_serpentine"));
        Root->SetStringField(TEXT("lawnmower_v2_progress_policy"), TEXT("shared_completed_waypoints_server_slot_global_epoch"));
        Root->SetStringField(TEXT("lawnmower_v2_initial_waypoint"), TEXT("nearest_unfinished_owned_lane_direct_flight"));
        Root->SetStringField(TEXT("sensor_class"), M.DroneSensorClass
            ? M.DroneSensorClass->GetPathName() : UPerfectDroneSensor::StaticClass()->GetPathName());
    }
    Root->SetNumberField(TEXT("max_sim_ticks"), M.MaxSimTicks);
    Root->SetNumberField(TEXT("recall_deadline_ticks"), M.RecallDeadlineTicks);
    Root->SetNumberField(TEXT("danger_exposure_threshold"), M.DangerExposureThreshold);
    Root->SetNumberField(TEXT("tracking_window_ticks"), M.TrackingWindowTicks);
    Root->SetNumberField(TEXT("oracle_lookahead_ticks"), M.OracleLookaheadTicks);
    Root->SetNumberField(TEXT("oracle_policy_version"), M.OraclePolicyVersion);
    Root->SetBoolField(TEXT("record_wall_clock_timing"), M.bRecordWallClockTiming);
    Root->SetBoolField(TEXT("suppress_benchmark_viewport"), M.bSuppressBenchmarkViewport);
    Root->SetBoolField(TEXT("use_nanite_for_live_view"), M.bUseNaniteForLiveView);
    Root->SetStringField(TEXT("metrics_detail_mode"), M.MetricsDetailMode);
    Root->SetNumberField(TEXT("grid_width"), M.GridWidth);
    Root->SetNumberField(TEXT("grid_height"), M.GridHeight);
    Root->SetNumberField(TEXT("cell_size_uu"), M.CellSizeUU);
    Root->SetNumberField(TEXT("seconds_per_tick"), M.SecondsPerTick);
    Root->SetNumberField(TEXT("arrival_minutes_per_tick"), M.ArrivalMinutesPerTick);
    Root->SetStringField(TEXT("fire_csv_path"), M.FirstFireTickCSVPath.FilePath);
    Root->SetStringField(TEXT("humans_csv_path"), M.HumansCSVPath.FilePath);
    Root->SetStringField(TEXT("generated_fire_csv_output_path"), M.GeneratedFireCsvOutputPath.FilePath);
    Root->SetStringField(TEXT("generated_humans_csv_output_path"), M.GeneratedHumansCsvOutputPath.FilePath);
    Root->SetStringField(TEXT("generated_terrain_csv_output_path"), M.GeneratedTerrainCsvOutputPath.FilePath);
    Root->SetStringField(TEXT("generated_fuel_csv_output_path"), M.GeneratedFuelCsvOutputPath.FilePath);
    Root->SetBoolField(TEXT("reuse_generated_scenario_csvs"), M.bReuseGeneratedScenarioCsvs);
    Root->SetNumberField(TEXT("human_mesh_scale"), M.HumanMeshScale);
    Root->SetStringField(TEXT("human_visual_mesh_set"), M.HumanVisualMeshSet);
    Root->SetStringField(TEXT("human_pose_mode"), M.HumanPoseMode);
    // Actual compiled visual contract, including manually supplied configs.
    // Keep aligned with Tools/run_benchmarks.py when captured pixels change.
    Root->SetNumberField(TEXT("vision_sensor_contract_version"), 6);
    Root->SetStringField(TEXT("yolo_online_metric_version"), TEXT("geometric_alive_one_to_one_cell_neighborhood_v2"));
    Root->SetNumberField(TEXT("environment_presentation_version"), 12);
    Root->SetNumberField(TEXT("terrain_presentation_margin_meters"), M.TerrainPresentationMarginMeters);
    Root->SetNumberField(TEXT("fire_seed"), M.FireGeneratorSeed);
    Root->SetNumberField(TEXT("fire_generator_ignition_count"), M.FireGeneratorIgnitionCount);
    Root->SetNumberField(TEXT("fire_generator_num_focus_points"), M.FireGeneratorNumFocusPoints);
    Root->SetNumberField(TEXT("fire_generator_spread_variant"), M.FireGeneratorSpreadVariant);
    Root->SetNumberField(TEXT("fire_generator_first_ignition_tick"), M.FireGeneratorFirstIgnitionTick);
    Root->SetNumberField(TEXT("fire_generator_additional_ignition_start_min_tick"), M.FireGeneratorAdditionalIgnitionStartMinTick);
    Root->SetNumberField(TEXT("fire_generator_additional_ignition_start_max_tick"), M.FireGeneratorAdditionalIgnitionStartMaxTick);
    Root->SetNumberField(TEXT("fire_generator_burn_percentage"), M.FireGeneratorBurnPercentage);
    Root->SetNumberField(TEXT("fire_generator_max_minutes"), M.FireGeneratorMaxMinutes);
    Root->SetNumberField(TEXT("fire_generator_spread_minutes_per_cell"), M.FireGeneratorSpreadMinutesPerCell);
    Root->SetNumberField(TEXT("fire_generator_spread_ticks_per_cell"), M.FireGeneratorSpreadTicksPerCell);
    Root->SetNumberField(TEXT("fire_generator_growth_acceleration"), M.FireGeneratorGrowthAcceleration);
    Root->SetNumberField(TEXT("fire_generator_terrain_cost_influence"), M.FireGeneratorTerrainCostInfluence);
    Root->SetNumberField(TEXT("fire_generator_spotting_chance"), M.FireGeneratorSpottingChance);
    Root->SetNumberField(TEXT("fire_generator_default_duration_ticks"), M.FireGeneratorDefaultDurationTicks);
    Root->SetNumberField(TEXT("fire_generator_auto_duration_ticks_per_burning_cell"), M.FireGeneratorAutoDurationTicksPerBurningCell);
    Root->SetNumberField(TEXT("fire_generator_duration_fill_fraction"), M.FireGeneratorDurationFillFraction);
    Root->SetBoolField(TEXT("render_terrain_visuals"), M.bRenderTerrainVisuals);
    Root->SetNumberField(TEXT("drone_count"), M.AutoDroneCount);
    if (M.bBenchmarkRecordIdealSensorWitnesses)
    {
        Root->SetBoolField(TEXT("record_ideal_sensor_witnesses"), true);
    }
    if (M.BenchmarkDroneLogicalIds.Num() > 0)
    {
        TArray<TSharedPtr<FJsonValue>> IdValues;
        for (const int32 Id : M.BenchmarkDroneLogicalIds)
            IdValues.Add(MakeShared<FJsonValueNumber>(Id));
        Root->SetArrayField(TEXT("drone_logical_ids"), IdValues);

        TArray<TSharedPtr<FJsonValue>> SpawnValues;
        for (const FVector2D& Cell : M.BenchmarkDroneSpawnCellPositions)
        {
            TArray<TSharedPtr<FJsonValue>> Pair;
            Pair.Add(MakeShared<FJsonValueNumber>(Cell.X));
            Pair.Add(MakeShared<FJsonValueNumber>(Cell.Y));
            SpawnValues.Add(MakeShared<FJsonValueArray>(Pair));
        }
        Root->SetArrayField(TEXT("drone_spawn_cell_positions"), SpawnValues);
    }
    Root->SetNumberField(TEXT("num_real_camera_drones"), M.NumRealCameraDrones);
    Root->SetBoolField(TEXT("enable_drone_captures"), M.bEnableDroneCaptures);
    Root->SetBoolField(TEXT("enable_camera_state_audit"), M.bEnableCameraStateAudit);
    Root->SetBoolField(TEXT("enable_vision_bridge"), M.bEnableVisionBridge);
    if (M.bEnableCachedDetectionChanceNull)
    {
        Root->SetBoolField(TEXT("enable_cached_detection_chance_null"), true);
        Root->SetBoolField(TEXT("cached_detection_chance_null_pilot"), M.bCachedDetectionChanceNullPilot);
        Root->SetStringField(TEXT("cached_detection_chance_null_cohort"), M.CachedDetectionChanceNullCohort);
    }
    Root->SetNumberField(TEXT("capture_every_n_ticks"), M.DroneCaptureEveryNTicks);
    Root->SetStringField(TEXT("vision_cache_mode"), M.VisionCacheMode == EVisionCacheMode::Replay ? TEXT("replay") :
        M.VisionCacheMode == EVisionCacheMode::Record ? TEXT("record") : TEXT("off"));
    Root->SetNumberField(TEXT("capture_resolution"), M.DroneCaptureResolution);
    if (M.CameraRenderProfile != TEXT("legacy"))
        Root->SetStringField(TEXT("camera_render_profile"), M.CameraRenderProfile);
    Root->SetBoolField(TEXT("enable_yolo_dataset_export"), M.bEnableYoloDatasetExport);
    Root->SetBoolField(TEXT("drone_capture_clear_air"), M.bDroneCaptureClearAir);
    Root->SetStringField(TEXT("yolo_dataset_id"), M.YoloDatasetId);
    Root->SetStringField(TEXT("scenario_family_id"), M.ScenarioFamilyId);
    Root->SetStringField(TEXT("yolo_dataset_output_dir"), M.YoloDatasetOutputDir);
    Root->SetStringField(TEXT("yolo_view_source"), M.YoloViewSource);
    Root->SetNumberField(TEXT("drone_capture_manual_ev100"), M.DroneCaptureManualEV100);
    Root->SetBoolField(TEXT("fire_visuals_only_for_yolo_captures"), M.bFireVisualsOnlyForYoloCaptures);
    Root->SetBoolField(TEXT("enable_fire_visuals"), M.bEnableFireVisuals);
    Root->SetBoolField(TEXT("save_drone_capture_pngs"), M.bSaveDroneCapturePngs);
    Root->SetNumberField(TEXT("fire_visuals_yolo_capture_radius_uu"), M.FireVisualsYoloCaptureRadiusUU);
    Root->SetNumberField(TEXT("fire_visuals_flame_width_scale"), M.FireVisualsFlameWidthScaleMultiplier);
    Root->SetNumberField(TEXT("fire_visuals_flame_height_scale"), M.FireVisualsFlameHeightScaleMultiplier);
    Root->SetNumberField(TEXT("fire_visuals_flame_sprite_size_scale"), M.FireVisualsBundledFlameSpriteSizeMultiplier);
    Root->SetNumberField(TEXT("fire_visuals_smoke_scale"), M.FireVisualsSmokeVfxScaleMultiplier);
    Root->SetNumberField(TEXT("fire_visuals_smoke_layers"), M.FireVisualsSmokeLayerCount);
    Root->SetBoolField(TEXT("fire_visuals_share_smoke_across_cell_clusters"), M.bFireVisualsShareSmokeAcrossCellClusters);
    Root->SetBoolField(TEXT("fire_visuals_enable_bundled_heat_distortion"), M.bFireVisualsEnableBundledHeatDistortion);
    Root->SetBoolField(TEXT("fire_visuals_connected_fire_front"), M.bFireVisualsConnectedFireFront);
    Root->SetNumberField(TEXT("fire_visuals_max_lights"), M.FireVisualsMaxLights);
    Root->SetNumberField(TEXT("fire_visuals_light_radius_uu"), M.FireVisualsLightRadiusUU);
    Root->SetNumberField(TEXT("fire_visuals_max_shadow_casting_lights"), M.FireVisualsMaxShadowCastingLights);
    Root->SetNumberField(TEXT("fire_visuals_smoke_z_offset_uu"), M.FireVisualsSmokeZOffsetUU);
    Root->SetNumberField(TEXT("fire_visuals_smoke_layer_z_step_uu"), M.FireVisualsSmokeLayerZStepUU);
    Root->SetNumberField(TEXT("drone_capture_pitch_degrees"), M.DroneCapturePitchDegrees);
    Root->SetBoolField(TEXT("drone_capture_auto_exposure"), M.bDroneCaptureAutoExposure);
    Root->SetNumberField(TEXT("drone_capture_exposure_bias"), M.DroneCaptureExposureBias);
    Root->SetNumberField(TEXT("drone_capture_bloom_intensity"), M.DroneCaptureBloomIntensity);
    Root->SetBoolField(TEXT("save_yolo_debug_images"), M.bSaveYoloDebugImages);
    Root->SetNumberField(TEXT("yolo_max_error_images"), M.YoloMaxErrorImages);
    Root->SetNumberField(TEXT("yolo_max_correct_images"), M.YoloMaxCorrectImages);
    Root->SetNumberField(TEXT("yolo_person_confidence_threshold"), M.YoloPersonConfidenceThreshold);
    Root->SetNumberField(TEXT("drone_speed_uu_per_tick"), M.DroneSpeedUUPerTick);
    Root->SetBoolField(TEXT("parallel_drone_processing"), M.bParallelDroneProcessing);
    Root->SetStringField(TEXT("drone_processing_mode"), M.bParallelDroneProcessing ? TEXT("parallel_cpu") : TEXT("sequential"));
    Root->SetBoolField(TEXT("use_drone_local_gpu_priority"), M.bDroneUseLocalGpuPriority);
    Root->SetNumberField(TEXT("drone_fov_degrees"), M.DroneHorizontalFOVDegrees);
    Root->SetNumberField(TEXT("drone_max_view_distance_uu"), M.DroneMaxViewDistanceUU);
    Root->SetNumberField(TEXT("drone_replan_cadence_ticks"), M.DroneReplanCadenceTicks);
    Root->SetNumberField(TEXT("drone_commitment_margin"), M.DroneCommitmentMargin);
    Root->SetStringField(TEXT("drone_planner_scoring_mode"), M.DronePlannerScoringMode);
    Root->SetStringField(TEXT("partition_strategy"), M.PartitionStrategyName);
    Root->SetNumberField(TEXT("partition_recompute_every_n_ticks"), M.PartitionRecomputeEveryNTicks);
    Root->SetNumberField(TEXT("grid_stats_every_n_ticks"), M.GridStatsEveryNTicks);
    Root->SetNumberField(TEXT("grid_stats_readback_lag_ticks"), M.GridStatsReadbackLagTicks);
    Root->SetNumberField(TEXT("priority_weight_staleness"), M.PriorityWeightStaleness);
    Root->SetNumberField(TEXT("priority_weight_danger"), M.PriorityWeightDanger);
    Root->SetNumberField(TEXT("priority_weight_person"), M.PriorityWeightPerson);
    Root->SetNumberField(TEXT("load_weight_danger"), M.LoadWeightDanger);
    Root->SetNumberField(TEXT("load_weight_person"), M.LoadWeightPerson);
    Root->SetNumberField(TEXT("sensor_fire_positive_llr"), M.SensorFirePositiveLLR);
    Root->SetNumberField(TEXT("sensor_fire_negative_llr"), M.SensorFireNegativeLLR);
    Root->SetNumberField(TEXT("sensor_person_positive_llr"), M.SensorPersonPositiveLLR);
    Root->SetNumberField(TEXT("sensor_person_negative_llr"), M.SensorPersonNegativeLLR);

    // Keep an exhaustive text snapshot of resolved manager settings alongside
    // the stable named fields above. This covers newly added planner/sensor and
    // runtime tunables without silently omitting them from audit provenance.
    const TSharedRef<FJsonObject> ReflectedSettings = MakeShared<FJsonObject>();
    for (TFieldIterator<FProperty> It(M.GetClass()); It; ++It)
    {
        const FProperty* Property = *It;
        if (!Property || Property->HasAnyPropertyFlags(CPF_Transient | CPF_Deprecated)) continue;
        if (CastField<FObjectPropertyBase>(Property) || CastField<FClassProperty>(Property) ||
            CastField<FDelegateProperty>(Property)) continue;
        if (const FArrayProperty* ArrayProperty = CastField<FArrayProperty>(Property))
        {
            if (CastField<FObjectPropertyBase>(ArrayProperty->Inner) || CastField<FClassProperty>(ArrayProperty->Inner)) continue;
        }
        FString ExportedValue;
        Property->ExportText_InContainer(0, ExportedValue, &M, nullptr, const_cast<ASimulationManager*>(&M), PPF_None);
        ReflectedSettings->SetStringField(Property->GetName(), ExportedValue);
    }
    Root->SetObjectField(TEXT("resolved_manager_properties"), ReflectedSettings);

    FString Out;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
        TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Out);
    FJsonSerializer::Serialize(Root, Writer);
    return Out;
}
