#include "HumanTrajectoryImporter.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

bool FHumanTrajectoryImporter::ImportFromCSV(const FString& FilePath, FResult& Out)
{
    Out.Keyframes.Reset();
    Out.TickStartIndex.Reset();
    Out.MaxTick = -1;
    Out.NumPersons = 0;

    FString Text;
    if (!FFileHelper::LoadFileToString(Text, *FilePath))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis humans: cannot read %s"), *FilePath);
        return false;
    }

    TArray<FString> Lines;
    Text.ParseIntoArrayLines(Lines, /*bCullEmpty=*/true);
    if (Lines.Num() < 1 || !Lines[0].StartsWith(TEXT("tick,person,x,y,group,state")))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis humans: bad header in %s"), *FilePath);
        return false;
    }

    Out.Keyframes.Reserve(Lines.Num() - 1);

    int32 PrevTick = -1;
    TSet<int32> PersonIds;
    TArray<FString> Cols;
    for (int32 i = 1; i < Lines.Num(); ++i)
    {
        Cols.Reset();
        Lines[i].ParseIntoArray(Cols, TEXT(","), /*bCullEmpty=*/false);
        if (Cols.Num() < 6)
        {
            UE_LOG(LogTemp, Warning, TEXT("Thesis humans: skipping malformed line %d"), i + 1);
            continue;
        }

        FHumanKeyframe K;
        K.Tick     = FCString::Atoi(*Cols[0]);
        K.PersonId = FCString::Atoi(*Cols[1]);
        K.X        = FCString::Atof(*Cols[2]);
        K.Y        = FCString::Atof(*Cols[3]);
        K.GroupId  = FCString::Atoi(*Cols[4]);
        K.State    = static_cast<uint8>(FCString::Atoi(*Cols[5]));

        if (K.Tick < PrevTick)
        {
            UE_LOG(LogTemp, Error,
                   TEXT("Thesis humans: non-monotonic tick at line %d (%d < %d) — aborting import."),
                   i + 1, K.Tick, PrevTick);
            return false;
        }
        PrevTick = K.Tick;

        PersonIds.Add(K.PersonId);
        Out.Keyframes.Add(K);
    }

    if (Out.Keyframes.Num() == 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis humans: no keyframes in %s"), *FilePath);
        return false;
    }

    Out.MaxTick = Out.Keyframes.Last().Tick;
    Out.NumPersons = PersonIds.Num();

    // Per-person appearance tick = tick of the person's first keyframe.
    Out.AppearTick.Init(-1, Out.NumPersons);
    for (const FHumanKeyframe& K : Out.Keyframes)
    {
        if (K.PersonId >= 0 && K.PersonId < Out.NumPersons && Out.AppearTick[K.PersonId] < 0)
        {
            Out.AppearTick[K.PersonId] = K.Tick;
        }
    }

    // Per-tick ranges (half-open, sentinel at end). Ticks with no rows get the
    // next tick's start index, i.e. an empty range.
    Out.TickStartIndex.Init(0, Out.MaxTick + 2);
    {
        int32 Cursor = 0;
        for (int32 T = 0; T <= Out.MaxTick; ++T)
        {
            while (Cursor < Out.Keyframes.Num() && Out.Keyframes[Cursor].Tick < T)
            {
                ++Cursor;
            }
            Out.TickStartIndex[T] = Cursor;
        }
        Out.TickStartIndex[Out.MaxTick + 1] = Out.Keyframes.Num();
        // TickStartIndex[T] = first row with Tick >= T, so
        // [TickStartIndex[T], TickStartIndex[T+1]) is exactly tick T's rows
        // (empty range for ticks with no rows).
    }

    UE_LOG(LogTemp, Log,
           TEXT("Thesis humans: imported %d keyframes, %d persons, ticks 0..%d from %s"),
           Out.Keyframes.Num(), Out.NumPersons, Out.MaxTick, *FilePath);
    return true;
}
