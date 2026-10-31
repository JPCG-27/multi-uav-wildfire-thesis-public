#include "ThesisBenchmarkSubsystem.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

void UThesisBenchmarkSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);
    FParse::Value(FCommandLine::Get(), TEXT("SimBatchStopFile="), StopFilePath);

    // Parse the manifest ONCE, before any level loads. The process command line
    // is unchanged by OpenLevel, but we cache the list here so we never re-read it.
    FString ManifestPath;
    if (FParse::Value(FCommandLine::Get(), TEXT("SimConfigManifest="), ManifestPath))
    {
        FString SingleConfig;
        if (FParse::Value(FCommandLine::Get(), TEXT("SimConfig="), SingleConfig))
        {
            UE_LOG(LogTemp, Fatal, TEXT("Use either SimConfigManifest or SimConfig, not both."));
            return;
        }
        FString Text;
        if (FFileHelper::LoadFileToString(Text, *ManifestPath))
        {
            TArray<FString> Lines;
            Text.ParseIntoArrayLines(Lines);
            for (FString& Line : Lines)
            {
                Line.TrimStartAndEndInline();
                if (!Line.IsEmpty())
                {
                    ScenarioConfigs.Add(Line);
                }
            }
        }
        if (ScenarioConfigs.IsEmpty())
        {
            UE_LOG(LogTemp, Fatal, TEXT("Thesis benchmark batch: unreadable or empty manifest %s"), *ManifestPath);
            return;
        }
        UE_LOG(LogTemp, Log,
               TEXT("Thesis benchmark batch: %d scenario(s) from manifest %s"),
               ScenarioConfigs.Num(), *ManifestPath);
    }
}

bool UThesisBenchmarkSubsystem::AdvanceAndHasMore()
{
    if (!StopFilePath.IsEmpty() && FPaths::FileExists(StopFilePath))
    {
        UE_LOG(LogTemp, Log, TEXT("Thesis batch: stop requested; leaving remaining scenarios pending."));
        return false;
    }
    ++Index;
    return ScenarioConfigs.IsValidIndex(Index);
}
