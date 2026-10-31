#pragma once

#include "CoreMinimal.h"
#include "VisionTypes.h"

class FSocket;

struct FVisionRequestTimings
{
    double SendSeconds = 0.0;
    double ReceiveSeconds = 0.0;
    double ParseAndCacheSeconds = 0.0;
};

/**
 * Blocking TCP client for Tools/vision_server.py (protocol documented there),
 * plus the detection cache that makes benchmark runs reproducible.
 *
 * Threading: every method is called from the single-threaded server slot of
 * ASimulationManager::Tick. Sends/receives block the game thread — same
 * philosophy as the GPU fences (slow frame, correct sim).
 *
 * Failure policy: Detect returns false for failed inference, read/write errors,
 * or incomplete replay data. The manager must pause/fail the run, never treat
 * failure as an empty valid observation.
 */
class THESIS_API FVisionBridge
{
public:
    ~FVisionBridge();

    void Configure(const FString& Host, int32 Port,
                   EVisionCacheMode InCacheMode, const FString& CacheDirName);

    /** Detections for a capture. Record/Off: sends pixels to the server
     *  (connecting lazily); Record also writes the response to the cache.
     *  Replay: reads the cache only — Pixels may be empty. */
    bool Detect(int32 DroneIndex, int32 Tick, int32 Width, int32 Height,
                const TArray<FColor>& PixelsBGRA, TArray<FVisionDetection>& OutDetections);

    EVisionCacheMode GetCacheMode() const { return CacheMode; }
    bool IsDegraded() const { return bConnectionFailed; }
    const FVisionRequestTimings& GetLastRequestTimings() const { return LastRequestTimings; }

private:
    FString ServerHost = TEXT("127.0.0.1");
    int32 ServerPort = 8765;
    EVisionCacheMode CacheMode = EVisionCacheMode::Record;
    FString CacheDir;

    FSocket* Socket = nullptr;
    bool bConnectionFailed = false;
    bool bLegacyTransfer = false;
    FVisionRequestTimings LastRequestTimings;
    TArray<uint8> RequestHeaderBuffer;
    TArray<uint8> ResponseBuffer;

    bool EnsureConnected();
    void CloseSocket();
    bool SendAll(const uint8* Data, int32 Len);
    bool RecvAll(uint8* Data, int32 Len);

    FString CachePath(int32 DroneIndex, int32 Tick) const;
    static bool ParseDetectionsJson(const FString& Json, TArray<FVisionDetection>& Out, bool bStrictValidation);
};
