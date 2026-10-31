#pragma once

#include "CoreMinimal.h"

class ASimulationManager;

/**
 * JSON run configuration for benchmark runs.
 *
 * One JSON file = one run. Every field is OPTIONAL — only keys present in the
 * file override the manager's editor-configured values, so an empty file is a
 * no-op and current behaviour is preserved exactly.
 *
 * Load order (BeginPlay, before the grid is created):
 *   1. -SimConfig=<path> on the command line (wins), else
 *   2. ASimulationManager::SimConfigPath if set in the editor.
 *
 * Supported keys (see ApplyJsonToManager for the authoritative list):
 *   run_id, benchmark_run_subdir, max_sim_ticks, exit_on_max_ticks,
 *   record_metrics, metrics_detail_mode,
 *   grid_width, grid_height, cell_size_uu, max_ticks_per_second,
 *   seconds_per_tick,
 *   render_terrain_visuals,
 *   fire_csv_path, humans_csv_path, generated_fire_csv_output_path,
 *   generated_humans_csv_output_path, generated_terrain_csv_output_path,
 *   generated_fuel_csv_output_path, reuse_generated_scenario_csvs,
 *   fire_seed, generate_fire_csv, fire_generator_*,
 *   drone_count, num_real_camera_drones, drone_speed_uu_per_tick,
 *   parallel_drone_processing,
 *   fire_visuals_only_for_yolo_captures, fire_visuals_yolo_capture_radius_uu,
 *   method_id, planner_strategy, drone_planner_scoring_mode, region_regime, information_source,
 *   oracle_policy_version,
 *   partition_strategy, partition_recompute_every_n_ticks,
 *   grid_stats_every_n_ticks, grid_stats_readback_lag_ticks,
 *   plus the priority/partition weight tunables (snake_case of the UPROPERTY name).
 */
class THESIS_API FSimRunConfig
{
public:
    /** Resolves the config path (command line beats InEditorPath). Empty = none. */
    static FString ResolveConfigPath(const FString& InEditorPath);

    /** Loads the JSON at Path and applies present keys to the manager.
     *  Returns false on missing file / parse error (manager left untouched). */
    static bool LoadAndApply(const FString& Path, ASimulationManager& Manager);

    /** Serializes the manager's resolved run-relevant settings to a JSON string
     *  (provenance snapshot written next to the metrics). */
    static FString MakeConfigSnapshot(const ASimulationManager& Manager);
};
