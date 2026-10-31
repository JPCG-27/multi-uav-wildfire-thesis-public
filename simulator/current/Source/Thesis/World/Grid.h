#pragma once

#include "CoreMinimal.h"


// Inside Grid.h or a new file DroneTypes.h
struct FObservation;


/**
 * Grid — the 2D simulation substrate. Plain C++ (non-UObject) by design:
 * single instance, owned by ASimulationManager via TUniquePtr.
 *
 * Indexing convention: y * Width + x. Row-major; matches GPU texture layout
 * and Cell2Fire CSV row order.
 *
 * Now contains:
 *   - Ground truth arrays (for evaluation)
 *   - Drone-side belief arrays (fire log-odds, person log-odds, last observed tick)
 *   - Ownership array (who currently owns each cell)
 */
class THESIS_API Grid
{
public:
    Grid(int32 InWidth, int32 InHeight);
    ~Grid();

    Grid(const Grid&) = delete;
    Grid& operator=(const Grid&) = delete;

    void Initialize();
    void Initialize(const FString& FirstFireTickCSV);

    int32 Width;
    int32 Height;

    // --- CSV path for fire data ---
    void SetFirstFireTickCSVPath(const FString& FilePath);
    FString GetFirstFireTickCSVPath() const;
    void SetFirstFireTickArray(const TArray<int32>& Source);

    // --- Ground truth access (evaluation only, never used by drones for planning) ---
    float getFloraDensity(int32 x, int32 y) const;
    int32 getFirstFireTick(int32 x, int32 y) const;   // -1 = never burns
    int32 getRealPriorityAndNumHumans(int32 x, int32 y) const; // still packed: priority & numHumans
    void  setRealPriorityAndNumHumans(int32 x, int32 y, int32 priority, int32 numHumans); // server-only

    // --- Drone-side belief (written only by the owning drone) ---
    float getFireBelief(int32 x, int32 y) const;
    float getPersonBelief(int32 x, int32 y) const;
    int32 getLastObservedTick(int32 x, int32 y) const;
    int32 getLastPersonObservedTick(int32 x, int32 y) const;
    void  setFireBelief(int32 x, int32 y, float value);
    void  setPersonBelief(int32 x, int32 y, float value);
    void  setLastObservedTick(int32 x, int32 y, int32 tick);
    void  setLastPersonObservedTick(int32 x, int32 y, int32 tick);
    // Capture ticks may precede ingestion. Keep the newest actual evidence tick.
    void  RecordPersonObservation(const FObservation& Observation);

    // First tick at which this drone saw positive fire/person evidence at (x,y). -1 = never.
    // Drives the drone-side discovery delay so the discoverer keeps sweeping before being pulled back.
    int32 getFirstFireDetectionTick  (int32 x, int32 y) const;
    int32 getFirstPersonDetectionTick(int32 x, int32 y) const;
    void  setFirstFireDetectionTick  (int32 x, int32 y, int32 tick);
    void  setFirstPersonDetectionTick(int32 x, int32 y, int32 tick);

    // --- Ownership (read/write, maintained by partition apply) ---
    char  getOwner(int32 x, int32 y) const;
    void  setOwner(int32 x, int32 y, char value);     // 'N' = unowned, 0..253 = drone index

    // --- Flora and fire (ground truth, set once) ---
    void  setFloraDensity(int32 x, int32 y, float value);
    void  setFirstFireTick(int32 x, int32 y, int32 value);

private:
    bool InBounds(int32 x, int32 y) const;
    int32 Index(int32 x, int32 y) const;

    void LoadAndSetFirstFireTickFromCSV(const FString& FilePath);

    // Ground truth
    float* floraDensity = nullptr;
    int32* firstFireTick = nullptr;
    int32* real_priority_and_numHumans = nullptr;   // packed: low 16 priority, high 16 humans

    // Drone-side belief
    float* fireBelief = nullptr;
    float* personBelief = nullptr;
    int32* lastObservedTick = nullptr;
    int32* lastPersonObservedTick = nullptr;
    int32* firstFireDetectionTick   = nullptr;
    int32* firstPersonDetectionTick = nullptr;

    // Ownership (updated by SimulationManager partition apply)
    char*  owner = nullptr;

    int32  TotalCells = 0;
    FString FirstFireTickCSVPath;
};
