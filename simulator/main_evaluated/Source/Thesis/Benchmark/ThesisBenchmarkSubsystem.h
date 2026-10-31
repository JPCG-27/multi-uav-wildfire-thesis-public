#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "ThesisBenchmarkSubsystem.generated.h"

/**
 * Holds the in-process scenario batch (-SimConfigManifest) across level reloads.
 *
 * GameInstance subsystems survive UGameplayStatics::OpenLevel, so the batch list
 * and the current index persist while each scenario runs in a FRESH level. That
 * isolates actor state while amortising editor/RHI startup. Fresh-process parity
 * must still be verified because renderer and engine caches survive level loads.
 */
UCLASS()
class THESIS_API UThesisBenchmarkSubsystem : public UGameInstanceSubsystem
{
    GENERATED_BODY()

public:
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;

    bool IsBatch() const { return ScenarioConfigs.Num() > 0; }
    int32 NumScenarios() const { return ScenarioConfigs.Num(); }
    int32 CurrentIndex() const { return Index; }

    FString CurrentConfigPath() const
    {
        return ScenarioConfigs.IsValidIndex(Index) ? ScenarioConfigs[Index] : FString();
    }

    /** Advance to the next scenario; returns true if one exists. */
    bool AdvanceAndHasMore();

private:
    TArray<FString> ScenarioConfigs;
    int32 Index = 0;
    FString StopFilePath;
};
