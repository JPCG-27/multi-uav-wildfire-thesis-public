#include "Grid.h"
#include "../Drone/DroneTypes.h"
#include "HAL/UnrealMemory.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

Grid::Grid(int32 InWidth, int32 InHeight)
    : Width(InWidth)
    , Height(InHeight)
    , TotalCells(InWidth * InHeight)
{
    floraDensity                = new float[TotalCells];
    firstFireTick               = new int32[TotalCells];
    real_priority_and_numHumans = new int32[TotalCells];
    fireBelief                  = new float[TotalCells];
    personBelief                = new float[TotalCells];
    lastObservedTick            = new int32[TotalCells];
    lastPersonObservedTick      = new int32[TotalCells];
    firstFireDetectionTick      = new int32[TotalCells];
    firstPersonDetectionTick    = new int32[TotalCells];
    owner                       = new char[TotalCells];

    FMemory::Memzero(floraDensity,                sizeof(float) * TotalCells);
    // -1 means this cell never burns. Do not zero this array: zero means
    // "ignites at tick 0" in the fire model.
    FMemory::Memset(firstFireTick, 0xFF, sizeof(int32) * TotalCells);
    FMemory::Memzero(real_priority_and_numHumans, sizeof(int32) * TotalCells);
    FMemory::Memzero(fireBelief,                  sizeof(float) * TotalCells);
    FMemory::Memzero(personBelief,                sizeof(float) * TotalCells);
    // lastObservedTick / firstFireDetectionTick / firstPersonDetectionTick: -1 means never
    FMemory::Memset(lastObservedTick, 0xFF,         sizeof(int32) * TotalCells);
    FMemory::Memset(lastPersonObservedTick, 0xFF,   sizeof(int32) * TotalCells);
    FMemory::Memset(firstFireDetectionTick, 0xFF,   sizeof(int32) * TotalCells);
    FMemory::Memset(firstPersonDetectionTick, 0xFF, sizeof(int32) * TotalCells);
    FMemory::Memset(owner, 'N', sizeof(char) * TotalCells);
}

Grid::~Grid()
{
    delete[] floraDensity;                floraDensity = nullptr;
    delete[] firstFireTick;               firstFireTick = nullptr;
    delete[] real_priority_and_numHumans; real_priority_and_numHumans = nullptr;
    delete[] fireBelief;                  fireBelief = nullptr;
    delete[] personBelief;                personBelief = nullptr;
    delete[] lastObservedTick;            lastObservedTick = nullptr;
    delete[] lastPersonObservedTick;      lastPersonObservedTick = nullptr;
    delete[] firstFireDetectionTick;      firstFireDetectionTick = nullptr;
    delete[] firstPersonDetectionTick;    firstPersonDetectionTick = nullptr;
    delete[] owner;                       owner = nullptr;
}

// ---------- Inline helpers ----------
bool Grid::InBounds(int32 x, int32 y) const
{
    return x >= 0 && x < Width && y >= 0 && y < Height;
}

int32 Grid::Index(int32 x, int32 y) const
{
    return y * Width + x;
}

// ---------- Ground truth access ----------
float Grid::getFloraDensity(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return floraDensity[Index(x, y)];
}

int32 Grid::getFirstFireTick(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return firstFireTick[Index(x, y)];
}

int32 Grid::getRealPriorityAndNumHumans(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return real_priority_and_numHumans[Index(x, y)];
}

void Grid::setRealPriorityAndNumHumans(int32 x, int32 y, int32 priority, int32 numHumans)
{
    checkSlow(InBounds(x, y));
    real_priority_and_numHumans[Index(x, y)] = (priority & 0xFFFF) | ((numHumans & 0xFFFF) << 16);
}

void Grid::setFloraDensity(int32 x, int32 y, float value)
{
    checkSlow(InBounds(x, y));
    floraDensity[Index(x, y)] = value;
}

void Grid::setFirstFireTick(int32 x, int32 y, int32 value)
{
    checkSlow(InBounds(x, y));
    firstFireTick[Index(x, y)] = value;
}

// ---------- Drone-side belief ----------
float Grid::getFireBelief(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return fireBelief[Index(x, y)];
}

float Grid::getPersonBelief(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return personBelief[Index(x, y)];
}

int32 Grid::getLastObservedTick(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return lastObservedTick[Index(x, y)];
}

void Grid::setFireBelief(int32 x, int32 y, float value)
{
    // INVARIANT: only the owning drone may write this
    checkSlow(InBounds(x, y));
    fireBelief[Index(x, y)] = value;
}

void Grid::setPersonBelief(int32 x, int32 y, float value)
{
    checkSlow(InBounds(x, y));
    personBelief[Index(x, y)] = value;
}

void Grid::setLastObservedTick(int32 x, int32 y, int32 tick)
{
    checkSlow(InBounds(x, y));
    lastObservedTick[Index(x, y)] = tick;
}

int32 Grid::getLastPersonObservedTick(int32 x, int32 y) const
{
    return InBounds(x, y) ? lastPersonObservedTick[Index(x, y)] : -1;
}

void Grid::setLastPersonObservedTick(int32 x, int32 y, int32 tick)
{
    if (InBounds(x, y)) lastPersonObservedTick[Index(x, y)] = tick;
}

void Grid::RecordPersonObservation(const FObservation& Observation)
{
    if (Observation.bHasPersonEvidence && Observation.Tick >= 0 &&
        Observation.CellIndex >= 0 && Observation.CellIndex < TotalCells)
    {
        int32& LastTick = lastPersonObservedTick[Observation.CellIndex];
        LastTick = FMath::Max(LastTick, Observation.Tick);
    }
}

int32 Grid::getFirstFireDetectionTick(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return firstFireDetectionTick[Index(x, y)];
}

int32 Grid::getFirstPersonDetectionTick(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return firstPersonDetectionTick[Index(x, y)];
}

void Grid::setFirstFireDetectionTick(int32 x, int32 y, int32 tick)
{
    checkSlow(InBounds(x, y));
    firstFireDetectionTick[Index(x, y)] = tick;
}

void Grid::setFirstPersonDetectionTick(int32 x, int32 y, int32 tick)
{
    checkSlow(InBounds(x, y));
    firstPersonDetectionTick[Index(x, y)] = tick;
}

// ---------- Ownership ----------
char Grid::getOwner(int32 x, int32 y) const
{
    checkSlow(InBounds(x, y));
    return owner[Index(x, y)];
}

void Grid::setOwner(int32 x, int32 y, char value)
{
    checkSlow(InBounds(x, y));
    owner[Index(x, y)] = value;
}

// ---------- CSV loading ----------
void Grid::SetFirstFireTickCSVPath(const FString& FilePath)
{
    FirstFireTickCSVPath = FilePath;
}

FString Grid::GetFirstFireTickCSVPath() const
{
    return FirstFireTickCSVPath;
}

void Grid::SetFirstFireTickArray(const TArray<int32>& Source)
{
    // Preserve the invariant that missing cells are "never burns" rather than
    // "burns at tick 0".
    FMemory::Memset(firstFireTick, 0xFF, sizeof(int32) * TotalCells);

    const int32 N = FMath::Min(Source.Num(), TotalCells);
    if (N > 0)
    {
        FMemory::Memcpy(firstFireTick, Source.GetData(), sizeof(int32) * N);
    }
}

void Grid::LoadAndSetFirstFireTickFromCSV(const FString& FilePath)
{
    FString FileContents;
    if (!FFileHelper::LoadFileToString(FileContents, *FilePath))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to load CSV: %s"), *FilePath);
        return;
    }

    TArray<FString> Lines;
    FileContents.ParseIntoArrayLines(Lines);

    for (int32 y = 0; y < Lines.Num() && y < Height; ++y)
    {
        TArray<FString> Cells;
        Lines[y].ParseIntoArray(Cells, TEXT(","), false);

        for (int32 x = 0; x < Cells.Num() && x < Width; ++x)
        {
            const int32 Value = FCString::Atoi(*Cells[x].TrimStartAndEnd());
            setFirstFireTick(x, y, Value);
        }
    }
}

void Grid::Initialize()
{
    for (int32 y = 0; y < Height; ++y)
    {
        for (int32 x = 0; x < Width; ++x)
        {
            setFloraDensity(x, y, 0.0f);
            setFirstFireTick(x, y, -1);
            setFireBelief(x, y, 0.0f);
            setPersonBelief(x, y, 0.0f);
            setLastObservedTick(x, y, -1);
            setLastPersonObservedTick(x, y, -1);
            setFirstFireDetectionTick(x, y, -1);
            setFirstPersonDetectionTick(x, y, -1);
            setOwner(x, y, 'N');
        }
    }

    if (!FirstFireTickCSVPath.IsEmpty())
    {
        LoadAndSetFirstFireTickFromCSV(FirstFireTickCSVPath);
    }
}

void Grid::Initialize(const FString& FirstFireTickCSV)
{
    FirstFireTickCSVPath = FirstFireTickCSV;
    Initialize();
}
