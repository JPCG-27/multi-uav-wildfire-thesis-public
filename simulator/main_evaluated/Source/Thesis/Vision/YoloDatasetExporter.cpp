#include "YoloDatasetExporter.h"

#include "Engine/TextureRenderTarget2D.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Policies/PrettyJsonPrintPolicy.h"

namespace
{
    constexpr float InvalidDepthCeilingUU = 100000000.0f;

    void IncludePoint(FIntRect& BBox, int64& Count, const FIntPoint& P)
    {
        if (Count == 0)
        {
            BBox = FIntRect(P, FIntPoint(P.X + 1, P.Y + 1));
            ++Count;
            return;
        }
        BBox.Min.X = FMath::Min(BBox.Min.X, P.X);
        BBox.Min.Y = FMath::Min(BBox.Min.Y, P.Y);
        BBox.Max.X = FMath::Max(BBox.Max.X, P.X + 1);
        BBox.Max.Y = FMath::Max(BBox.Max.Y, P.Y + 1);
        ++Count;
    }

    FString StateName(uint8 State)
    {
        switch (State)
        {
        case EHumanState::Healthy: return TEXT("healthy");
        case EHumanState::Injured: return TEXT("injured");
        case EHumanState::Immobile: return TEXT("immobile");
        case EHumanState::Dead: return TEXT("dead");
        default: return TEXT("unknown");
        }
    }

    TSharedPtr<FJsonObject> Vec3(const FVector& V)
    {
        TSharedPtr<FJsonObject> O = MakeShared<FJsonObject>();
        O->SetNumberField(TEXT("x"), V.X);
        O->SetNumberField(TEXT("y"), V.Y);
        O->SetNumberField(TEXT("z"), V.Z);
        return O;
    }

    TArray<TSharedPtr<FJsonValue>> BBoxValues(const FIntRect& B)
    {
        TArray<TSharedPtr<FJsonValue>> A;
        A.Add(MakeShared<FJsonValueNumber>(B.Min.X));
        A.Add(MakeShared<FJsonValueNumber>(B.Min.Y));
        A.Add(MakeShared<FJsonValueNumber>(B.Max.X));
        A.Add(MakeShared<FJsonValueNumber>(B.Max.Y));
        return A;
    }
}

bool FYoloDatasetExporter::SaveBytesAtomic(const FString& Path, const TArray64<uint8>& Bytes)
{
    const FString TempPath = Path + TEXT(".partial");
    IFileManager::Get().Delete(*TempPath, false, true, true);
    if (!FFileHelper::SaveArrayToFile(Bytes, *TempPath)) return false;
    if (!IFileManager::Get().Move(*Path, *TempPath, false, true, false, true))
    {
        IFileManager::Get().Delete(*TempPath, false, true, true);
        return false;
    }
    return true;
}

bool FYoloDatasetExporter::SaveTextAtomic(const FString& Path, const FString& Text, bool bReplaceExisting)
{
    const FString TempPath = Path + TEXT(".partial");
    IFileManager::Get().Delete(*TempPath, false, true, true);
    if (!FFileHelper::SaveStringToFile(Text, *TempPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM)) return false;
    if (!IFileManager::Get().Move(*Path, *TempPath, bReplaceExisting, true, false, true))
    {
        IFileManager::Get().Delete(*TempPath, false, true, true);
        return false;
    }
    return true;
}

bool FYoloDatasetExporter::BeginRun(const FString& OutputRoot, const FString& InDatasetId,
    const FString& InScenarioFamilyId, const TMap<FString, FString>& InSourceHashes, FString& OutError)
{
    OutError.Empty();
    if (bActive)
    {
        OutError = TEXT("An exporter run is already active.");
        return false;
    }
    if (InDatasetId.IsEmpty() || FPaths::GetCleanFilename(InDatasetId) != InDatasetId ||
        InDatasetId.Contains(TEXT("..")) || InScenarioFamilyId.IsEmpty())
    {
        OutError = TEXT("Dataset ID must be one path segment and scenario_family_id is required.");
        return false;
    }
    RunDirectory = FPaths::Combine(OutputRoot, InDatasetId);
    if (IFileManager::Get().DirectoryExists(*RunDirectory) || IFileManager::Get().FileExists(*FPaths::Combine(RunDirectory, TEXT("run.json"))))
    {
        OutError = FString::Printf(TEXT("Refusing to overwrite existing dataset directory: %s"), *RunDirectory);
        return false;
    }
    if (!IFileManager::Get().MakeDirectory(*RunDirectory, true))
    {
        OutError = FString::Printf(TEXT("Could not create dataset directory: %s"), *RunDirectory);
        return false;
    }

    ScenarioFamilyId = InScenarioFamilyId;
    DatasetId = InDatasetId;
    SourceHashes = InSourceHashes;
    FramePaths.Reset();
    bActive = true;

    TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("schema_version"), TEXT("sar-export-v1"));
    Root->SetBoolField(TEXT("complete"), false);
    Root->SetStringField(TEXT("status"), TEXT("in_progress"));
    Root->SetStringField(TEXT("dataset_id"), DatasetId);
    Root->SetStringField(TEXT("scenario_family_id"), ScenarioFamilyId);
    TSharedPtr<FJsonObject> Hashes = MakeShared<FJsonObject>();
    for (const TPair<FString, FString>& Pair : SourceHashes) Hashes->SetStringField(Pair.Key, Pair.Value);
    Root->SetObjectField(TEXT("source_hashes"), Hashes);
    Root->SetArrayField(TEXT("frames"), {});
    FString Json;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Json);
    FJsonSerializer::Serialize(Root.ToSharedRef(), Writer);
    if (!SaveTextAtomic(FPaths::Combine(RunDirectory, TEXT("run.json")), Json))
    {
        bActive = false;
        OutError = TEXT("Could not create initial incomplete run manifest.");
        return false;
    }
    return true;
}

bool FYoloDatasetExporter::BuildVisibleMask(const FYoloDatasetFrame& Frame, TArray<uint32>& OutMask,
    TArray<FYoloMaskPersonStats>& OutPersonStats, FString& OutError)
{
    OutError.Empty();
    const int64 PixelCount = static_cast<int64>(Frame.Width) * Frame.Height;
    if (Frame.Width <= 0 || Frame.Height <= 0 || PixelCount > MAX_int32 ||
        Frame.Rgb.Num() != PixelCount || Frame.OpaqueSceneDepthUU.Num() != PixelCount ||
        Frame.PersonDepthUU.Num() != Frame.Persons.Num())
    {
        OutError = TEXT("Frame dimensions or per-person depth arrays are inconsistent.");
        return false;
    }
    if (!(Frame.DepthToleranceUU >= 0.0f) || !FMath::IsFinite(Frame.DepthToleranceUU))
    {
        OutError = TEXT("Depth tolerance must be finite and nonnegative.");
        return false;
    }

    OutMask.Init(0, static_cast<int32>(PixelCount));
    OutPersonStats.Reset();
    OutPersonStats.SetNum(Frame.Persons.Num());
    TArray<float> NearestDepth;
    NearestDepth.Init(InvalidDepthCeilingUU, static_cast<int32>(PixelCount));
    TMap<int32, int32> PersonIndexById;

    for (const float SceneZ : Frame.OpaqueSceneDepthUU)
    {
        if (!FMath::IsFinite(SceneZ) || SceneZ <= 0.0f)
        {
            OutError = TEXT("Opaque scene depth contains a non-finite or non-positive sample.");
            return false;
        }
    }

    int32 PreviousPersonId = INDEX_NONE;
    for (int32 PersonIndex = 0; PersonIndex < Frame.Persons.Num(); ++PersonIndex)
    {
        const int32 PersonId = Frame.Persons[PersonIndex].PersonId;
        const TArray<float>& PersonDepth = Frame.PersonDepthUU[PersonIndex];
        if (PersonId < 0 || static_cast<uint32>(PersonId) == MAX_uint32 || PersonId <= PreviousPersonId ||
            PersonDepth.Num() != PixelCount)
        {
            OutError = TEXT("Person IDs must be unique, ascending, and each depth image must match frame size.");
            return false;
        }
        PreviousPersonId = PersonId;
        OutPersonStats[PersonIndex].PersonId = PersonId;
        PersonIndexById.Add(PersonId, PersonIndex);

        for (int32 Pixel = 0; Pixel < PixelCount; ++Pixel)
        {
            const float PersonZ = PersonDepth[Pixel];
            if (!FMath::IsFinite(PersonZ) || PersonZ <= 0.0f)
            {
                OutError = TEXT("Person depth contains a non-finite or non-positive sample.");
                return false;
            }
            if (PersonZ >= InvalidDepthCeilingUU) continue;
            const int32 X = Pixel % Frame.Width;
            const int32 Y = Pixel / Frame.Width;
            FYoloMaskPersonStats& Stats = OutPersonStats[PersonIndex];
            IncludePoint(Stats.AmodalBBoxXYXY, Stats.AmodalPixels, FIntPoint(X, Y));

            const float SceneZ = Frame.OpaqueSceneDepthUU[Pixel];
            const bool bVisiblePastOpaqueDepth = SceneZ >= InvalidDepthCeilingUU ||
                SceneZ + Frame.DepthToleranceUU >= PersonZ;
            if (!bVisiblePastOpaqueDepth) continue;

            if (PersonZ < NearestDepth[Pixel] ||
                (PersonZ == NearestDepth[Pixel] &&
                 (OutMask[Pixel] == 0 || PersonId < static_cast<int32>(OutMask[Pixel] - 1))))
            {
                NearestDepth[Pixel] = PersonZ;
                OutMask[Pixel] = static_cast<uint32>(PersonId) + 1u;
            }
        }
    }

    for (int32 Pixel = 0; Pixel < OutMask.Num(); ++Pixel)
    {
        const uint32 Id = OutMask[Pixel];
        if (Id == 0) continue;
        const int32* PersonIndex = PersonIndexById.Find(static_cast<int32>(Id - 1));
        if (!PersonIndex) continue;
        FYoloMaskPersonStats& Stats = OutPersonStats[*PersonIndex];
        IncludePoint(Stats.VisibleBBoxXYXY, Stats.VisiblePixels,
            FIntPoint(Pixel % Frame.Width, Pixel / Frame.Width));
    }
    return true;
}

bool FYoloDatasetExporter::WriteFrame(const FYoloDatasetFrame& Frame, FString& OutError)
{
    OutError.Empty();
    if (!bActive || Frame.ScenarioFamilyId != ScenarioFamilyId || Frame.Tick < 0 || Frame.PoseTick < 0 || Frame.CameraId.IsEmpty() ||
        !FMath::IsFinite(Frame.HorizontalFovDegrees) || Frame.HorizontalFovDegrees <= 0.0f || Frame.HorizontalFovDegrees >= 179.0f ||
        !FMath::IsFinite(Frame.SecondsPerTick) || Frame.SecondsPerTick <= 0.0f)
    {
        OutError = TEXT("No active run or invalid tick/camera/scenario identity.");
        return false;
    }
    const FString Stem = FString::Printf(TEXT("%s_t%06d"), *Frame.CameraId, Frame.Tick);
    const FString FrameRel = FString::Printf(TEXT("frames/%s.json"), *Stem);
    if (FramePaths.Contains(FrameRel) || IFileManager::Get().FileExists(*FPaths::Combine(RunDirectory, FrameRel)))
    {
        OutError = TEXT("Refusing to overwrite a frame already written for this camera and tick.");
        return false;
    }
    TArray<uint32> Mask;
    TArray<FYoloMaskPersonStats> Stats;
    if (!BuildVisibleMask(Frame, Mask, Stats, OutError)) return false;

    const FString ImageRel = FString::Printf(TEXT("images/%s.png"), *Stem);
    const FString MaskRel = FString::Printf(TEXT("masks/%s.u32"), *Stem);
    IFileManager::Get().MakeDirectory(*FPaths::Combine(RunDirectory, TEXT("images")), true);
    IFileManager::Get().MakeDirectory(*FPaths::Combine(RunDirectory, TEXT("masks")), true);
    IFileManager::Get().MakeDirectory(*FPaths::Combine(RunDirectory, TEXT("frames")), true);

    TArray64<uint8> Png;
    FImageUtils::PNGCompressImageArray(Frame.Width, Frame.Height, TArrayView64<const FColor>(Frame.Rgb), Png);
    if (Png.IsEmpty() || !SaveBytesAtomic(FPaths::Combine(RunDirectory, ImageRel), Png))
    {
        OutError = TEXT("RGB PNG write failed.");
        return false;
    }

    static_assert(PLATFORM_LITTLE_ENDIAN, "YOLO native masks are specified as uint32 little-endian.");
    const int64 PixelCount = static_cast<int64>(Frame.Width) * Frame.Height;
    TArray64<uint8> MaskBytes;
    MaskBytes.SetNumUninitialized(PixelCount * sizeof(uint32));
    FMemory::Memcpy(MaskBytes.GetData(), Mask.GetData(), PixelCount * sizeof(uint32));
    if (!SaveBytesAtomic(FPaths::Combine(RunDirectory, MaskRel), MaskBytes))
    {
        OutError = TEXT("Instance mask write failed.");
        return false;
    }

    TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("schema_version"), TEXT("sar-frame-v1"));
    Root->SetNumberField(TEXT("sensor_contract_version"), 5);
    Root->SetNumberField(TEXT("environment_presentation_version"), 12);
    Root->SetNumberField(TEXT("tick"), Frame.Tick);
    Root->SetNumberField(TEXT("pose_tick"), Frame.PoseTick);
    Root->SetNumberField(TEXT("pose_seed"), Frame.HumanVisualSeed);
    Root->SetNumberField(TEXT("pose_sim_seconds"), static_cast<double>(Frame.PoseTick) * Frame.SecondsPerTick);
    Root->SetStringField(TEXT("camera_id"), Frame.CameraId);
    Root->SetStringField(TEXT("scenario_family_id"), Frame.ScenarioFamilyId);
    Root->SetStringField(TEXT("view_source"), Frame.ViewSource);
    Root->SetNumberField(TEXT("width"), Frame.Width);
    Root->SetNumberField(TEXT("height"), Frame.Height);
    Root->SetStringField(TEXT("capture_source"), TEXT("SCS_FinalColorLDR"));
    Root->SetStringField(TEXT("depth_source"), TEXT("SCS_SceneDepth"));
    Root->SetStringField(TEXT("depth_encoding"), TEXT("linear_scene_depth_uu"));
    Root->SetNumberField(TEXT("depth_tolerance_uu"), Frame.DepthToleranceUU);
    Root->SetNumberField(TEXT("max_valid_depth_uu"), InvalidDepthCeilingUU);
    Root->SetStringField(TEXT("image_path"), ImageRel);
    Root->SetStringField(TEXT("mask_path"), MaskRel);
    TArray<TSharedPtr<FJsonValue>> Exclusions;
    for (const FString& E : Frame.QualityExclusions) Exclusions.Add(MakeShared<FJsonValueString>(E));
    Root->SetArrayField(TEXT("quality_exclusions"), Exclusions);

    const FVector Location = Frame.CameraTransform.GetLocation();
    const FRotator Rotation = Frame.CameraTransform.Rotator();
    TSharedPtr<FJsonObject> Camera = MakeShared<FJsonObject>();
    Camera->SetObjectField(TEXT("location_uu"), Vec3(Location));
    TSharedPtr<FJsonObject> RotJson = MakeShared<FJsonObject>();
    RotJson->SetNumberField(TEXT("pitch"), Rotation.Pitch);
    RotJson->SetNumberField(TEXT("yaw"), Rotation.Yaw);
    RotJson->SetNumberField(TEXT("roll"), Rotation.Roll);
    Camera->SetObjectField(TEXT("rotation_degrees"), RotJson);
    Camera->SetNumberField(TEXT("horizontal_fov_degrees"), Frame.HorizontalFovDegrees);
    const double TanHalfHorizontalFov = FMath::Tan(FMath::DegreesToRadians(static_cast<double>(Frame.HorizontalFovDegrees) * 0.5));
    const double FocalX = Frame.Width / (2.0 * TanHalfHorizontalFov);
    const double FocalY = Frame.Height / (2.0 * TanHalfHorizontalFov * Frame.Height / Frame.Width);
    TSharedPtr<FJsonObject> Intrinsics = MakeShared<FJsonObject>();
    Intrinsics->SetNumberField(TEXT("fx_px"), FocalX);
    Intrinsics->SetNumberField(TEXT("fy_px"), FocalY);
    Intrinsics->SetNumberField(TEXT("cx_px"), (Frame.Width - 1) * 0.5);
    Intrinsics->SetNumberField(TEXT("cy_px"), (Frame.Height - 1) * 0.5);
    Camera->SetObjectField(TEXT("intrinsics"), Intrinsics);
    Root->SetObjectField(TEXT("camera"), Camera);

    TArray<TSharedPtr<FJsonValue>> PersonsJson;
    for (int32 I = 0; I < Frame.Persons.Num(); ++I)
    {
        const FHumanVisualCaptureRecord& P = Frame.Persons[I];
        const FYoloMaskPersonStats& S = Stats[I];
        TSharedPtr<FJsonObject> Person = MakeShared<FJsonObject>();
        Person->SetNumberField(TEXT("person_id"), P.PersonId);
        Person->SetStringField(TEXT("state"), StateName(P.State));
        Person->SetStringField(TEXT("pose"), P.Pose);
        Person->SetStringField(TEXT("mesh"), P.MeshPath);
        Person->SetObjectField(TEXT("world_location_uu"), Vec3(P.WorldTransform.GetLocation()));
        const FRotator PersonRotation = P.WorldTransform.Rotator();
        TSharedPtr<FJsonObject> PersonRotationJson = MakeShared<FJsonObject>();
        PersonRotationJson->SetNumberField(TEXT("pitch"), PersonRotation.Pitch);
        PersonRotationJson->SetNumberField(TEXT("yaw"), PersonRotation.Yaw);
        PersonRotationJson->SetNumberField(TEXT("roll"), PersonRotation.Roll);
        Person->SetObjectField(TEXT("world_rotation_degrees"), PersonRotationJson);
        if (S.AmodalPixels > 0) Person->SetArrayField(TEXT("amodal_bbox_xyxy"), BBoxValues(S.AmodalBBoxXYXY));
        else Person->SetField(TEXT("amodal_bbox_xyxy"), MakeShared<FJsonValueNull>());
        if (S.VisiblePixels > 0) Person->SetArrayField(TEXT("visible_bbox_xyxy"), BBoxValues(S.VisibleBBoxXYXY));
        else Person->SetField(TEXT("visible_bbox_xyxy"), MakeShared<FJsonValueNull>());
        Person->SetNumberField(TEXT("amodal_pixels"), static_cast<double>(S.AmodalPixels));
        Person->SetNumberField(TEXT("visible_pixels"), static_cast<double>(S.VisiblePixels));
        const int64 Support = S.AmodalPixels;
        Person->SetNumberField(TEXT("occlusion_ratio"), Support > 0 ? 1.0 - static_cast<double>(S.VisiblePixels) / Support : 1.0);
        Person->SetBoolField(TEXT("truncated"), S.AmodalPixels > 0 &&
            (S.AmodalBBoxXYXY.Min.X == 0 || S.AmodalBBoxXYXY.Min.Y == 0 || S.AmodalBBoxXYXY.Max.X == Frame.Width || S.AmodalBBoxXYXY.Max.Y == Frame.Height));
        Person->SetNumberField(TEXT("mesh_scale"), P.WorldTransform.GetScale3D().X);
        PersonsJson.Add(MakeShared<FJsonValueObject>(Person));
    }
    Root->SetArrayField(TEXT("persons"), PersonsJson);

    FString Json;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Json);
    FJsonSerializer::Serialize(Root.ToSharedRef(), Writer);
    if (!SaveTextAtomic(FPaths::Combine(RunDirectory, FrameRel), Json))
    {
        OutError = TEXT("Frame metadata write failed.");
        return false;
    }
    FramePaths.Add(FrameRel);
    return true;
}

bool FYoloDatasetExporter::CompleteRun(const TSharedPtr<FJsonObject>& Provenance, FString& OutError)
{
    OutError.Empty();
    if (!bActive || FramePaths.IsEmpty())
    {
        OutError = TEXT("No active dataset export run with captured frames to complete.");
        return false;
    }
    TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
    Root->SetStringField(TEXT("schema_version"), TEXT("sar-export-v1"));
    Root->SetStringField(TEXT("dataset_id"), DatasetId);
    Root->SetBoolField(TEXT("complete"), true);
    Root->SetStringField(TEXT("status"), TEXT("complete"));
    Root->SetStringField(TEXT("scenario_family_id"), ScenarioFamilyId);
    TArray<TSharedPtr<FJsonValue>> Frames;
    for (const FString& Path : FramePaths) Frames.Add(MakeShared<FJsonValueString>(Path));
    Root->SetArrayField(TEXT("frames"), Frames);
    TSharedPtr<FJsonObject> Hashes = MakeShared<FJsonObject>();
    for (const TPair<FString, FString>& Pair : SourceHashes) Hashes->SetStringField(Pair.Key, Pair.Value);
    Root->SetObjectField(TEXT("source_hashes"), Hashes);
    if (Provenance.IsValid()) Root->SetObjectField(TEXT("provenance"), Provenance);
    FString Json;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&Json);
    FJsonSerializer::Serialize(Root.ToSharedRef(), Writer);
    if (!SaveTextAtomic(FPaths::Combine(RunDirectory, TEXT("run.json")), Json, true))
    {
        OutError = TEXT("Final run manifest could not be committed.");
        return false;
    }
    bActive = false;
    return true;
}

void FYoloDatasetExporter::AbortRun()
{
    if (bActive)
    {
        bActive = false;
        // Keep the run.json incomplete marker. Dataset assembly rejects it.
    }
}
