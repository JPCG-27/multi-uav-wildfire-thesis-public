#include "ChanceNullScorer.h"

TArray<ChanceNullScorer::FOffset> ChanceNullScorer::BuildOffsets(int32 FrameCount, bool bPilot)
{
    TArray<FOffset> Result;
    if (FrameCount <= 0)
    {
        return Result;
    }

    Result.Add({0, 0});
    const int32 MinLag = FMath::CeilToInt(static_cast<float>(FrameCount) * 0.10f);
    const int32 StepPercent = bPilot ? 40 : 5;
    const int32 StartPercent = bPilot ? 10 : 10;
    const int32 EndPercent = bPilot ? 90 : 90;
    for (int32 Percent = StartPercent; Percent <= EndPercent; Percent += StepPercent)
    {
        const int32 Offset = FMath::RoundToInt(static_cast<float>(FrameCount) * Percent / 100.0f);
        if (Offset <= 0 || Offset >= FrameCount || FMath::Min(Offset, FrameCount - Offset) < MinLag)
        {
            continue;
        }
        bool bDuplicateOffset = false;
        for (const FOffset& Existing : Result)
        {
            if (Existing.Frames == Offset)
            {
                bDuplicateOffset = true;
                break;
            }
        }
        if (!bDuplicateOffset)
        {
            Result.Add({Percent, Offset});
        }
    }
    return Result;
}

ChanceNullScorer::FFrameScore ChanceNullScorer::ScoreFrame(
    const FFrame& Destination,
    const FFrame& Source,
    int32 OffsetFrames,
    int32 GridWidth,
    float PersonConfidenceThreshold,
    const TFunction<int32(const FVisionDetection&, bool&)>& ProjectDetection)
{
    FFrameScore Result;
    Result.CameraId = Destination.CameraId;
    Result.OffsetFrames = OffsetFrames;
    Result.SourceCameraId = Source.CameraId;
    Result.SourceCaptureTick = Source.CaptureTick;
    Result.CaptureTick = Destination.CaptureTick;
    Result.ProcessingTick = Destination.ProcessingTick;
    Result.DetectionCount = Source.Detections.Num();
    Result.SourceDetections = Source.Detections;
    Result.PersonsInView = Destination.VisiblePeople.Num();
    Result.MatchedPersonIds.Init(INDEX_NONE, Source.Detections.Num());
    Result.DetectionCells.Init(INDEX_NONE, Source.Detections.Num());

    TArray<float> Confidence;
    Confidence.Reserve(Source.Detections.Num());
    TArray<int32> PersonCells;
    PersonCells.Reserve(Destination.VisiblePeople.Num());
    for (const FPerson& Person : Destination.VisiblePeople)
    {
        PersonCells.Add(Person.Cell);
    }

    for (int32 DetectionIndex = 0; DetectionIndex < Source.Detections.Num(); ++DetectionIndex)
    {
        const FVisionDetection& Detection = Source.Detections[DetectionIndex];
        Confidence.Add(Detection.Confidence);
        const bool bPersonClaim = Detection.ClassId == 0 && Detection.Confidence >= PersonConfidenceThreshold;
        if (!bPersonClaim)
        {
            ++Result.NonPerson;
            continue;
        }

        ++Result.PersonClaimCount;
        bool bOffGrid = false;
        const int32 Cell = ProjectDetection(Detection, bOffGrid);
        Result.DetectionCells[DetectionIndex] = Cell;
        if (bOffGrid)
        {
            ++Result.OffGridRejections;
        }
    }

    const TArray<int32> MatchedPersonIndices = YoloGeometricMetrics::MatchPersonIndices(
        Result.DetectionCells, Confidence, PersonCells, GridWidth);
    for (int32 DetectionIndex = 0; DetectionIndex < MatchedPersonIndices.Num(); ++DetectionIndex)
    {
        const int32 PersonIndex = MatchedPersonIndices[DetectionIndex];
        if (PersonIndex != INDEX_NONE)
        {
            Result.MatchedPersonIds[DetectionIndex] = Destination.VisiblePeople[PersonIndex].Id;
            ++Result.TruePositives;
        }
    }
    Result.FalsePositives = Result.PersonClaimCount - Result.TruePositives;
    Result.Missed = Result.PersonsInView - Result.TruePositives;
    return Result;
}

bool ChanceNullScorer::AggregatePeople(
    const TArray<FFrame>& Frames,
    const TArray<FFrameScore>& Scores,
    const TArray<FPerson>& AllPeople,
    TArray<FPersonScore>& OutPeople)
{
    OutPeople.Reset();
    if (Frames.Num() != Scores.Num())
    {
        return false;
    }
    TMap<int32, FPersonScore> ById;
    for (const FPerson& Person : AllPeople)
    {
        FPersonScore& Row = ById.Add(Person.Id);
        Row.PersonId = Person.Id;
        Row.AppearanceTick = Person.AppearanceTick;
    }

    for (int32 FrameIndex = 0; FrameIndex < Frames.Num(); ++FrameIndex)
    {
        const FFrame& Frame = Frames[FrameIndex];
        const FFrameScore& Score = Scores[FrameIndex];
        if (Score.CameraId != Frame.CameraId || Score.CaptureTick != Frame.CaptureTick ||
            Score.ProcessingTick != Frame.ProcessingTick)
        {
            return false;
        }
        for (const int32 PersonId : Score.MatchedPersonIds)
        {
            if (PersonId == INDEX_NONE)
            {
                continue;
            }
            FPersonScore* Row = ById.Find(PersonId);
            if (!Row)
            {
                return false;
            }
            ++Row->MatchCount;
            if (Row->FirstMatchProcessingTick == INDEX_NONE || Frame.ProcessingTick < Row->FirstMatchProcessingTick)
            {
                Row->FirstMatchProcessingTick = Frame.ProcessingTick;
            }
        }
    }

    ById.GenerateValueArray(OutPeople);
    OutPeople.Sort([](const FPersonScore& A, const FPersonScore& B) { return A.PersonId < B.PersonId; });
    return true;
}

bool ChanceNullScorer::MatchesOriginal(const FFrame& Destination, const FFrameScore& Score)
{
    auto SameDetections = [](const TArray<FVisionDetection>& A, const TArray<FVisionDetection>& B)
    {
        if (A.Num() != B.Num()) return false;
        for (int32 Index = 0; Index < A.Num(); ++Index)
        {
            if (A[Index].ClassId != B[Index].ClassId || A[Index].Confidence != B[Index].Confidence ||
                A[Index].X0 != B[Index].X0 || A[Index].Y0 != B[Index].Y0 ||
                A[Index].X1 != B[Index].X1 || A[Index].Y1 != B[Index].Y1)
            {
                return false;
            }
        }
        return true;
    };
    if (Score.CameraId != Destination.CameraId || Score.OffsetFrames != 0 ||
        Score.SourceCameraId != Destination.CameraId || Score.SourceCaptureTick != Destination.CaptureTick ||
        !SameDetections(Score.SourceDetections, Destination.Detections) ||
        Score.DetectionCount != Destination.Detections.Num() ||
        Score.DetectionCells.Num() != Destination.Detections.Num() ||
        Score.MatchedPersonIds.Num() != Destination.Detections.Num() ||
        Score.CaptureTick != Destination.CaptureTick ||
        Score.ProcessingTick != Destination.ProcessingTick ||
        Score.DetectionCells != Destination.OriginalDetectionCells ||
        Score.MatchedPersonIds != Destination.OriginalMatchedPersonIds ||
        Score.TruePositives != Destination.OriginalTruePositives ||
        Score.FalsePositives != Destination.OriginalFalsePositives ||
        Score.Missed != Destination.OriginalMissed ||
        Score.NonPerson != Destination.OriginalNonPerson ||
        Score.PersonClaimCount != Destination.OriginalPersonClaimCount ||
        Score.PersonsInView != Destination.VisiblePeople.Num() ||
        Destination.OriginalMissed != Destination.VisiblePeople.Num() - Destination.OriginalTruePositives ||
        Score.OffGridRejections != Destination.OriginalOffGridRejections)
    {
        return false;
    }

    return true;
}
