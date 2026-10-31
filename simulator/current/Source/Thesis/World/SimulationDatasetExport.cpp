#include "SimulationManager.h"
#include "DroneCaptureComponent.h"
#include "HumanManager.h"
#include "ProceduralTerrain.h"
#include "SimRunConfig.h"
#include "Components/PoseableMeshComponent.h"
#include "Components/ExponentialHeightFogComponent.h"
#include "Components/VolumetricCloudComponent.h"
#include "Particles/ParticleSystemComponent.h"
#include "NiagaraComponent.h"
#include "Materials/MaterialInterface.h"
#include "MaterialShared.h"
#include "UObject/UObjectIterator.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/SecureHash.h"
#include "Misc/EngineVersion.h"
#include "HAL/PlatformMisc.h"
#include "HAL/IConsoleManager.h"
#include "RHI.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

bool ASimulationManager::FailDatasetExport(const FString& Error)
{
    UE_LOG(LogTemp, Error, TEXT("Dataset export failed: %s"), *Error);
    if (DatasetExporter) DatasetExporter->AbortRun();
    State = ESimState::Stopped;
    if (bExitOnMaxTicks) FPlatformMisc::RequestExitWithStatus(false, 1);
    return false;
}

bool ASimulationManager::InitializeDatasetExport()
{
    // Ground truth can only enter this isolated offline output path.
    if (bEnableVisionBridge || !bEnableDroneCaptures || NumRealCameraDrones <= 0 ||
        DroneCaptureEveryNTicks <= 0 || MaxSimTicks <= 0 || bEnableFireVisuals ||
        bDroneCaptureAutoExposure || HumanMeshScale != 1.0f || !HumanManager)
        return FailDatasetExport(TEXT("Requires bridge off, cameras on, a finite run, fire visuals off, manual exposure, scale 1 and loaded human manager."));
    if (!YoloViewSource.Equals(TEXT("planner_trajectory"), ESearchCase::IgnoreCase) &&
        !YoloViewSource.Equals(TEXT("scripted_coverage"), ESearchCase::IgnoreCase))
        return FailDatasetExport(TEXT("yolo_view_source must be planner_trajectory or scripted_coverage."));

    TMap<FString, FString> Hashes;
    const TPair<FString, FString> Inputs[] = {
        {TEXT("humans"), HumansCSVPath.FilePath}, {TEXT("fire"), FirstFireTickCSVPath.FilePath},
        {TEXT("terrain"), GeneratedTerrainCsvOutputPath.FilePath}, {TEXT("fuel"), GeneratedFuelCsvOutputPath.FilePath}};
    for (const auto& Input : Inputs)
    {
        FString Path = Input.Value;
        if (FPaths::IsRelative(Path)) Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Path);
        TArray<uint8> Bytes;
        if (!FFileHelper::LoadFileToArray(Bytes, *Path))
            return FailDatasetExport(FString::Printf(TEXT("Cannot hash %s source: %s"), *Input.Key, *Path));
        FSHAHash Hash;
        FSHA1::HashBuffer(Bytes.GetData(), Bytes.Num(), Hash.Hash);
        Hashes.Add(Input.Key, TEXT("sha1:") + Hash.ToString());
    }
    FString Root = YoloDatasetOutputDir.IsEmpty() ? FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Datasets")) : YoloDatasetOutputDir;
    if (FPaths::IsRelative(Root)) Root = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Root);
    DatasetExporter = MakeUnique<FYoloDatasetExporter>();
    FString Error;
    if (!DatasetExporter->BeginRun(Root, YoloDatasetId, ScenarioFamilyId, Hashes, Error)) return FailDatasetExport(Error);
    UE_LOG(LogTemp, Display, TEXT("Dataset export started: %s/%s (family %s)"), *Root, *YoloDatasetId, *ScenarioFamilyId);
    return true;
}

bool ASimulationManager::SetupScriptedDatasetView(UDroneCaptureComponent* Capture, int32 Tick, int32 DroneIndex,
                                                    FTransform& OutOriginalTransform, float& OutOriginalFov)
{
    if (!DatasetExporter || !Capture || !YoloViewSource.Equals(TEXT("scripted_coverage"), ESearchCase::IgnoreCase))
        return false;

    OutOriginalTransform = Capture->GetComponentTransform();
    OutOriginalFov = Capture->FOVAngle;

    // Every fifth sample is a deliberate scene/background frame. Remaining
    // samples target a stable person snapshot selected in person-ID order.
    const int32 CaptureOrdinal = FMath::Max(0, Tick / FMath::Max(1, DroneCaptureEveryNTicks));
    const bool bBackground = ((CaptureOrdinal + DroneIndex) % 5) == 4;
    float BackgroundCellX = (GridWidth - 1) * 0.5f;
    float BackgroundCellY = (GridHeight - 1) * 0.5f;
    if (bBackground)
    {
        const FIntPoint Corners[] = {
            FIntPoint(FMath::Clamp(1, 0, GridWidth - 1), FMath::Clamp(1, 0, GridHeight - 1)),
            FIntPoint(FMath::Clamp(GridWidth - 2, 0, GridWidth - 1), FMath::Clamp(1, 0, GridHeight - 1)),
            FIntPoint(FMath::Clamp(1, 0, GridWidth - 1), FMath::Clamp(GridHeight - 2, 0, GridHeight - 1)),
            FIntPoint(FMath::Clamp(GridWidth - 2, 0, GridWidth - 1), FMath::Clamp(GridHeight - 2, 0, GridHeight - 1))};
        float BestNearestPersonDistanceSq = -1.0f;
        for (const FIntPoint& Corner : Corners)
        {
            float NearestPersonDistanceSq = TNumericLimits<float>::Max();
            for (const FHumanSnapshot& Snapshot : HumanManager->GetCurrentSnapshots())
            {
                if (Snapshot.PersonId < 0) continue;
                const float DX = static_cast<float>(Corner.X) - Snapshot.PosCell.X;
                const float DY = static_cast<float>(Corner.Y) - Snapshot.PosCell.Y;
                NearestPersonDistanceSq = FMath::Min(NearestPersonDistanceSq, DX * DX + DY * DY);
            }
            if (NearestPersonDistanceSq > BestNearestPersonDistanceSq)
            {
                BestNearestPersonDistanceSq = NearestPersonDistanceSq;
                BackgroundCellX = static_cast<float>(Corner.X);
                BackgroundCellY = static_cast<float>(Corner.Y);
            }
        }
    }
    const float BackgroundGroundZ = TerrainActor
        ? TerrainActor->GetHeightAtCell(BackgroundCellX, BackgroundCellY) : GridOriginWorld.Z;
    FVector Target(GridOriginWorld.X + BackgroundCellX * CellSizeUU,
                   GridOriginWorld.Y + BackgroundCellY * CellSizeUU, BackgroundGroundZ);
    if (!bBackground)
    {
        TArray<FHumanSnapshot> Candidates;
        for (const FHumanSnapshot& Snapshot : HumanManager->GetCurrentSnapshots())
            if (Snapshot.PersonId >= 0) Candidates.Add(Snapshot);
        Candidates.Sort([](const FHumanSnapshot& A, const FHumanSnapshot& B) { return A.PersonId < B.PersonId; });
        if (!Candidates.IsEmpty())
        {
            const int32 CandidateIndex = (CaptureOrdinal + DroneIndex) % Candidates.Num();
            const FHumanSnapshot& Subject = Candidates[CandidateIndex];
            const float CX = FMath::Clamp(Subject.PosCell.X, 0.0f, static_cast<float>(GridWidth - 1));
            const float CY = FMath::Clamp(Subject.PosCell.Y, 0.0f, static_cast<float>(GridHeight - 1));
            const float GroundZ = TerrainActor ? TerrainActor->GetHeightAtCell(CX, CY) : GridOriginWorld.Z;
            Target = FVector(GridOriginWorld.X + CX * CellSizeUU, GridOriginWorld.Y + CY * CellSizeUU, GroundZ);
        }
    }

    const uint32 SeedBits = static_cast<uint32>(FireGeneratorSeed) ^
        (static_cast<uint32>(Tick) * 73856093u) ^ (static_cast<uint32>(DroneIndex) * 19349663u);
    FRandomStream Random(static_cast<int32>(SeedBits));
    // The optimized-profile qualification corpus must use the operational camera.
    // Retain the historical randomized curriculum for existing legacy exports.
    const bool bOperationalCamera = CameraRenderProfile == TEXT("fast_shader_v3");
    const float Altitude = bOperationalCamera ? DroneCruiseAltitudeAboveGroundUU : Random.FRandRange(1500.0f, 6000.0f);
    const float PitchMagnitude = bOperationalCamera ? FMath::Abs(DroneCapturePitchDegrees) : Random.FRandRange(32.0f, 72.0f);
    const float Yaw = Random.FRandRange(-180.0f, 180.0f);
    const float HorizontalDistance = Altitude / FMath::Tan(FMath::DegreesToRadians(PitchMagnitude));
    const FVector Forward(FMath::Cos(FMath::DegreesToRadians(Yaw)), FMath::Sin(FMath::DegreesToRadians(Yaw)), 0.0f);
    const FVector CameraLocation = Target - Forward * HorizontalDistance + FVector(0.0f, 0.0f, Altitude);
    const FRotator CameraRotation = (Target - CameraLocation).Rotation();
    const float NewFov = bOperationalCamera ? OutOriginalFov : Random.FRandRange(28.0f, 58.0f);
    Capture->SetWorldLocationAndRotation(CameraLocation, CameraRotation);
    Capture->FOVAngle = NewFov;
    return true;
}

bool ASimulationManager::ExportDatasetCapture(UDroneCaptureComponent* Capture)
{
    if (!DatasetExporter || !Capture || Capture->CaptureSource != SCS_FinalColorLDR ||
        Capture->ProjectionType != ECameraProjectionMode::Perspective || Capture->bUseCustomProjectionMatrix ||
        Capture->PrimitiveRenderMode == ESceneCapturePrimitiveRenderMode::PRM_UseShowOnlyList)
        return FailDatasetExport(TEXT("Exporter requires an ordinary perspective FinalColorLDR camera."));

    FYoloDatasetFrame Frame;
    Frame.CameraId = Capture->GetOwner()->GetName();
    Frame.ScenarioFamilyId = ScenarioFamilyId;
    Frame.ViewSource = YoloViewSource;
    Frame.CameraTransform = Capture->GetComponentTransform();
    Frame.HorizontalFovDegrees = Capture->FOVAngle;
    Frame.HumanVisualSeed = HumanManager->VisualSeed;
    Frame.SecondsPerTick = SecondsPerTick;
    if (!Capture->ReadLastCapture(Frame.Rgb, Frame.Width, Frame.Height, Frame.Tick) || Frame.Tick != CurrentTick)
        return FailDatasetExport(TEXT("RGB readback is missing or belongs to another tick."));
    for (FColor& Pixel : Frame.Rgb) Pixel.A = 255;
    HumanManager->GetLastVisualCaptureRecords(Frame.Persons, Frame.PoseTick);
    Frame.Persons.Sort([](const FHumanVisualCaptureRecord& A, const FHumanVisualCaptureRecord& B) { return A.PersonId < B.PersonId; });
    for (const FHumanSnapshot& Snapshot : HumanManager->GetCurrentSnapshots())
    {
        if (Snapshot.PersonId >= 0 && !Frame.Persons.ContainsByPredicate(
            [&](const FHumanVisualCaptureRecord& P) { return P.PersonId == Snapshot.PersonId; }))
            Frame.QualityExclusions.AddUnique(TEXT("missing_human_visual_metadata"));
    }

    // Conservative global checks: do not silently call people behind translucent
    // smoke visible. Opaque depth cannot establish transmittance. No world flags
    // or production materials are changed by these checks.
    for (TObjectIterator<USceneComponent> It; It; ++It)
    {
        if (It->GetWorld() != GetWorld() || !It->IsRegistered() || !It->IsVisible()) continue;
        const USceneComponent* Component = *It;
        if (const UExponentialHeightFogComponent* Fog = Cast<UExponentialHeightFogComponent>(Component))
        {
            if (Fog->FogDensity > 0 && Capture->ShowFlags.Fog) Frame.QualityExclusions.AddUnique(TEXT("unvalidated_fog_visibility"));
        }
        if (Cast<UVolumetricCloudComponent>(Component) && Capture->ShowFlags.Cloud) Frame.QualityExclusions.AddUnique(TEXT("unvalidated_cloud_visibility"));
        if ((Cast<UNiagaraComponent>(Component) || Cast<UParticleSystemComponent>(Component)) && Component->IsActive())
            Frame.QualityExclusions.AddUnique(TEXT("unvalidated_particle_visibility"));
        if (const UPrimitiveComponent* Primitive = Cast<UPrimitiveComponent>(Component))
        {
            for (int32 Index = 0; Index < Primitive->GetNumMaterials(); ++Index)
            {
                const UMaterialInterface* Material = Primitive->GetMaterial(Index);
                if (Material && IsTranslucentBlendMode(Material->GetBlendMode()))
                    Frame.QualityExclusions.AddUnique(TEXT("unvalidated_translucent_geometry"));
            }
        }
    }
    Frame.QualityExclusions.Sort();
    int32 W = 0, H = 0;
    if (!Capture->ReadDepthPass(nullptr, Frame.OpaqueSceneDepthUU, W, H) || W != Frame.Width || H != Frame.Height)
        return FailDatasetExport(TEXT("Opaque-scene depth readback failed."));
    Frame.PersonDepthUU.SetNum(Frame.Persons.Num());
    for (int32 I = 0; I < Frame.Persons.Num(); ++I)
    {
        const UPoseableMeshComponent* Mesh = Frame.Persons[I].Component;
        if (!Mesh || !Capture->ReadDepthPass(Mesh, Frame.PersonDepthUU[I], W, H) || W != Frame.Width || H != Frame.Height)
            return FailDatasetExport(TEXT("Isolated person depth readback failed."));
    }
    FString Error;
    if (!DatasetExporter->WriteFrame(Frame, Error)) return FailDatasetExport(Error);
    UE_LOG(LogTemp, Display, TEXT("Dataset frame tick=%d pose_tick=%d camera=%s people=%d exclusions=%d"),
        Frame.Tick, Frame.PoseTick, *Frame.CameraId, Frame.Persons.Num(), Frame.QualityExclusions.Num());
    return true;
}

bool ASimulationManager::CompleteDatasetExport()
{
    if (!DatasetExporter) return FailDatasetExport(TEXT("Exporter was not initialized."));
    TSharedPtr<FJsonObject> Provenance = MakeShared<FJsonObject>();
    Provenance->SetStringField(TEXT("engine_version"), FEngineVersion::Current().ToString());
    Provenance->SetStringField(TEXT("gpu"), GRHIAdapterName);
    Provenance->SetStringField(TEXT("view_source"), YoloViewSource);
    Provenance->SetNumberField(TEXT("seconds_per_tick"), SecondsPerTick);
    TSharedPtr<FJsonObject> Settings;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(FSimRunConfig::MakeConfigSnapshot(*this));
    if (!FJsonSerializer::Deserialize(Reader, Settings)) return FailDatasetExport(TEXT("Could not serialize capture provenance."));
    // Source files are identified by content hashes; output locations must not
    // change the identity of a repeated export on the same machine.
    TArray<FString> Keys;
    Settings->Values.GetKeys(Keys);
    for (const FString& Key : Keys)
    {
        if (Key.EndsWith(TEXT("_path")) || Key.EndsWith(TEXT("_dir")) || Key.EndsWith(TEXT("_subdir")) ||
            Key == TEXT("run_id") || Key == TEXT("yolo_dataset_id")) Settings->RemoveField(Key);
    }
    Settings->SetNumberField(TEXT("capture_every_n_ticks"), DroneCaptureEveryNTicks);
    Provenance->SetObjectField(TEXT("settings"), Settings);
    TSharedPtr<FJsonObject> Render = MakeShared<FJsonObject>();
    const TCHAR* Names[] = { TEXT("r.Nanite"), TEXT("r.TextureStreaming"), TEXT("foliage.ForceLOD"), TEXT("r.Shadow.Virtual.SMRT.RayCountLocal") };
    for (const TCHAR* Name : Names)
        if (const IConsoleVariable* Var = IConsoleManager::Get().FindConsoleVariable(Name)) Render->SetStringField(Name, Var->GetString());
    Provenance->SetObjectField(TEXT("renderer"), Render);
    FString Error;
    if (!DatasetExporter->CompleteRun(Provenance, Error)) return FailDatasetExport(Error);
    return true;
}
