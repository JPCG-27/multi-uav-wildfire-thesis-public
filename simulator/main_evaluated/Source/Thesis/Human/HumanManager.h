#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HumanTypes.h"
#include "HumanTrajectoryImporter.h"
#include "HumanVisualPose.h"
#include "HumanManager.generated.h"

class Grid;
class AProceduralTerrain;
class USkeletalMesh;
class UPoseableMeshComponent;

/** Read-only description of the last pose actually published to the renderer. */
struct THESIS_API FHumanVisualCaptureRecord
{
    int32 PersonId = INDEX_NONE;
    uint8 State = 0;
    FString Pose;
    FString MeshPath;
    FTransform WorldTransform = FTransform::Identity;
    const UPoseableMeshComponent* Component = nullptr;
};

/**
 * Replays precomputed human trajectories (Tools/generate_humans.py) as
 * ground truth.
 *
 * Each sim tick, ASimulationManager calls ApplyTick from the single-threaded
 * server slot (tick anatomy step 4 — after the drone phase). ApplyTick:
 *   1. Zeroes the human counts it wrote last tick (dirty-cell list — no grid scan).
 *   2. Scans the keyframe range for the tick and rebuilds per-person snapshots.
 *   3. Writes per-cell alive-person counts into Grid::real_priority_and_numHumans
 *      (the ground-truth field UPerfectDroneSensor reads). Dead persons remain
 *      visible in snapshots (bodies render) but do not count as persons.
 *
 * INVARIANT (ground-truth/belief wall): this class only writes real_* ground
 * truth. Server/drone belief is only ever updated through observations.
 * INVARIANT (determinism): replay state is a pure function of the loaded CSV
 * and the tick — reset-safe with no per-run state beyond the dirty list.
 *
 * Replaceability: a future live human simulator replaces this actor behind the
 * same ApplyTick(Tick, Grid*) call site.
 *
 * After MaxTick, persons hold their final pose (the fire has long passed).
 */
UCLASS(Blueprintable)
class THESIS_API AHumanManager : public AActor
{
    GENERATED_BODY()

public:
    AHumanManager();

    /** Load humans.csv. Safe to call again (re-import). */
    bool LoadTrajectories(const FString& CsvPath);

    /** Apply ground truth for Tick. Called from the server slot only. */
    void ApplyTick(int32 Tick, Grid* SharedGrid);

    /** Clears replay write-state (not the loaded trajectories). */
    void ResetReplay(Grid* SharedGrid);

    bool HasTrajectories() const { return Data.Keyframes.Num() > 0; }
    int32 GetNumPersons() const { return Data.NumPersons; }
    int32 GetMaxTick() const { return Data.MaxTick; }

    /** Tick the person first appears at (-1 for invalid ids). */
    int32 GetAppearTick(int32 PersonId) const
    {
        return Data.AppearTick.IsValidIndex(PersonId) ? Data.AppearTick[PersonId] : -1;
    }

    /** Snapshots for the tick last applied (debug draw / visuals / metrics). */
    const TArray<FHumanSnapshot>& GetCurrentSnapshots() const { return Current; }
    /** Actual simulation tick last requested by the visual pose updater. */
    int32 GetLastVisualUpdateRequestTick() const { return LastVisualUpdateRequestTick; }

    /** Returns presentation metadata for the pose meshes currently published.
     *  OutPoseTick is the trajectory tick used by the last visual update. */
    void GetLastVisualCaptureRecords(TArray<FHumanVisualCaptureRecord>& OutRecords, int32& OutPoseTick) const;

    /** Read-only trajectory access for the oracle benchmark path. */
    const FHumanTrajectoryImporter::FResult& GetTrajectoryDataForOracle() const { return Data; }

    /** Rebuild snapshots for Tick without changing replay state. */
    void GetSnapshotsForTick(int32 Tick, TArray<FHumanSnapshot>& OutSnapshots) const;

    // ---------- Visual representation ----------

    /** Optional single skeletal mesh override for every person. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    TObjectPtr<USkeletalMesh> HumanMesh;

    /** Optional mesh pool. When set, persons pick HumanMeshes[PersonId % Num].
     *  This is ideal for imported Mixamo characters. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    TArray<TObjectPtr<USkeletalMesh>> HumanMeshes;

    /** Auto-load meshes imported by Tools/import_mixamo_characters.py into
     *  /Game/Mixamo/Characters before falling back to the mannequin. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    bool bAutoUseMixamoIfAvailable = true;

    /** When HumanMesh is empty, auto-load the Third Person mannequin (added via
     *  Content Drawer -> Add -> Add Feature or Content Pack -> Third Person). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    bool bAutoUseMannequinIfAvailable = true;

    /** mixed, mixamo, or mannequin. Explicit HumanMesh/HumanMeshes still win. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    FString VisualMeshSet = TEXT("mixed");

    /** automatic, varied, or a pose name (standing/walking/running/waving/crouching/seated/lying). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    FString VisualPoseMode = TEXT("automatic");

    float VisualSecondsPerTick = 1.0f;
    int32 VisualSeed = 0;

    /** Render person meshes only inside drone captures. Off by default so you
     *  can see the people; a dozen skeletal meshes cost little. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals")
    bool bHumanMeshesCaptureOnly = false;

    /** Skip the per-tick skeletal-mesh update (component creation + repositioning)
     *  when nothing renders them (headless / no-capture runs). The ground-truth
     *  grid update in ApplyTick still runs; only the visual update is gated. */
    bool bUpdateVisualMeshes = true;

    /** In suppressed-viewport capture benchmarks, build the pose lazily from
     *  the last applied human state immediately before its first camera render.
     *  Disable for legacy per-tick pose updates (A/B capture parity). */
    bool bDeferVisualUpdatesUntilCapture = false;

    /** Ensure render components represent the latest state published by ApplyTick. */
    void UpdateVisualPoseForCapture();

    /** Runtime view-mode helper used by ASimulationManager. */
    void SetVisualsCaptureOnly(bool bCaptureOnly);


    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Visuals", meta = (ClampMin = "0.1"))
    float HumanMeshScale = 1.0f;

    /** Terrain for ground-height placement (null = flat at grid origin Z). */
    void SetTerrain(const AProceduralTerrain* InTerrain) { Terrain = InTerrain; }

    // ---------- Debug visualization ----------

    // Debug spheres around persons removed; humans are shown via their
    // skeletal-mesh actors only. Flip to true to restore the debug overlay.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Debug")
    bool bDrawDebugHumans = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Debug", meta = (ClampMin = "1.0"))
    float DebugSphereRadiusUU = 60.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Thesis|Humans|Debug")
    float DebugSphereZOffsetUU = 100.0f;

    /** Set by SimulationManager so debug draw maps cell space to world. */
    void SetGridFrame(const FVector& InGridOriginWorld, float InCellSizeUU)
    {
        GridOriginWorld = InGridOriginWorld;
        CellSizeUU = InCellSizeUU;
    }

protected:
    virtual void Tick(float DeltaSeconds) override;   // debug draw only

private:
    FHumanTrajectoryImporter::FResult Data;

    TArray<FHumanSnapshot> Current;        // indexed by PersonId
    TArray<int32> LastWrittenCells;        // cells whose counts we wrote last ApplyTick
    TArray<bool> AppearRecorded;           // per person: appear logged once

    FVector GridOriginWorld = FVector::ZeroVector;
    float CellSizeUU = 1000.0f;

    TWeakObjectPtr<const AProceduralTerrain> Terrain;

    UPROPERTY()
    TArray<TObjectPtr<UPoseableMeshComponent>> PersonMeshes;
    TArray<FString> CurrentVisualPoseNames;
    TArray<FString> CurrentVisualMeshPaths;
    int32 LastVisualPoseTick = INDEX_NONE;
    int32 LastVisualUpdateRequestTick = INDEX_NONE;
    int32 LastAppliedTick = INDEX_NONE;

    // Presentation caches only; never consumed by trajectories, truth or planners.
    TMap<USkeletalMesh*, FHumanVisualRig> VisualMeshPoses;
    TArray<TArray<int32>> PersonKeyframes;
    TArray<float> KeyframeHeadings;
    TArray<float> KeyframeDistancesPerTick;
    bool bTriedAutomaticMeshes = false;

    const FHumanVisualRig& GetVisualMeshPose(USkeletalMesh* Mesh);
    void BuildVisualTrajectoryIndex();
    void ResetVisualReplay();

    /** World-space ground Z at fractional cell coords. */
    float GroundZAt(float CellX, float CellY) const;

    /** Creates/updates one skeletal mesh per person (poses, lying for immobile/dead). */
    void UpdatePersonMeshes(int32 Tick);

    /** Loads the first available mesh set from imported project assets. */
    bool TryAutoLoadHumanMeshes();

    /** Deterministic visual mesh selection for a person id. */
    USkeletalMesh* MeshForPerson(int32 PersonId) const;
};
