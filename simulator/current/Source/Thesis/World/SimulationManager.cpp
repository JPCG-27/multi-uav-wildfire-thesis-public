#include "SimulationManager.h"
#include "LiveViewPresentation.h"
#include "LiveRenderPolicy.h"
#include "Misc/App.h"
#include "ProfilingDebugging/CsvProfiler.h"
#include "GameFramework/PlayerController.h"
#include "Components/InputComponent.h"
#include "InputCoreTypes.h"
#include "Grid.h"
#include "PriorityModel.h"
#include "FireDataImporter.h"
#include "GpuBeliefManager.h"
#include "Drone.h"
#include "DronePlannerBase.h"
#include "DronePlannerFactory.h"
#include "LawnmowerV2Planner.h"
#include "OracleAssignmentSolver.h"
#include "Engine/World.h"
#include "Kismet/GameplayStatics.h"
#include "DrawDebugHelpers.h"
#include "GameFramework/Pawn.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "UObject/UObjectIterator.h"
#include "UObject/UnrealType.h"
#include "Components/SkyLightComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "RenderingThread.h"
#include "SceneView.h"
#include "SceneViewExtension.h"
#include "GameTime.h"
#include "AssetCompilingManager.h"
#include "ShaderCompiler.h"
#include "ContentStreaming.h"
#include "Engine/Texture2D.h"
#include "SimulationHUD.h"
#include "SimRunConfig.h"
#include "ThesisBenchmarkSubsystem.h"
#include "HumanManager.h"
#include "ForestDresser.h"
#include "ProceduralTerrain.h"
#include "VisionProjection.h"
#include "YoloGeometricMetrics.h"
#include "FireVisualsManager.h"
#include "DroneCaptureComponent.h"
#include "VisionBridge.h"
#include "Engine/TextureCube.h"
#include "PipelineStateCache.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "HAL/FileManager.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformMisc.h"
#include "GenericPlatform/GenericPlatformMisc.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Policies/PrettyJsonPrintPolicy.h"
#include "Materials/MaterialInterface.h"
#include "Async/ParallelFor.h"

ASimulationManager* ASimulationManager::StaticInstance = nullptr;

namespace
{
    FString CameraAuditCsv(const FString& Value)
    {
        FString Escaped = Value;
        Escaped.ReplaceInline(TEXT("\""), TEXT("\"\""));
        return TEXT("\"") + Escaped + TEXT("\"");
    }

    FString CameraAuditNumber(double Value)
    {
        return FString::Printf(TEXT("%.17g"), Value);
    }

    FString CameraAuditVector(const FVector& Value)
    {
        return FString::Printf(TEXT("%.17g,%.17g,%.17g"), Value.X, Value.Y, Value.Z);
    }

    FString CameraAuditQuat(const FQuat& Value)
    {
        return FString::Printf(TEXT("%.17g,%.17g,%.17g,%.17g"), Value.X, Value.Y, Value.Z, Value.W);
    }

    FString CameraAuditPixelHash(const TArray<FColor>* Pixels, int32 Width, int32 Height)
    {
        if (!Pixels || Width <= 0 || Height <= 0 || Pixels->Num() != Width * Height)
            return FString();

        // FNV-1a over explicit BGRA bytes, independent of struct padding.
        uint64 Hash = 14695981039346656037ull;
        for (const FColor& Pixel : *Pixels)
        {
            const uint8 Channels[] = { Pixel.B, Pixel.G, Pixel.R, Pixel.A };
            for (const uint8 Byte : Channels)
            {
                Hash ^= Byte;
                Hash *= 1099511628211ull;
            }
        }
        return FString::Printf(TEXT("fnv1a64-bgra8:%016llx"), static_cast<unsigned long long>(Hash));
    }

    FString CameraAuditVisionCacheKey(const FString& CacheDirectory, int32 DroneIndex, int32 Tick)
    {
        return FPaths::ConvertRelativePathToFull(FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("VisionCache"),
            CacheDirectory, FString::Printf(TEXT("d%d_t%06d.json"), DroneIndex, Tick)));
    }

    void CameraAuditSetVector(const TSharedRef<FJsonObject>& Object, const TCHAR* Prefix, const FVector& Value)
    {
        Object->SetNumberField(FString::Printf(TEXT("%s_x"), Prefix), Value.X);
        Object->SetNumberField(FString::Printf(TEXT("%s_y"), Prefix), Value.Y);
        Object->SetNumberField(FString::Printf(TEXT("%s_z"), Prefix), Value.Z);
    }

    void CameraAuditSetTransform(const TSharedRef<FJsonObject>& Object, const TCHAR* Prefix, const FTransform& Transform)
    {
        const FVector Location = Transform.GetLocation();
        const FQuat Rotation = Transform.GetRotation();
        const FVector Scale = Transform.GetScale3D();
        CameraAuditSetVector(Object, *FString::Printf(TEXT("%s_position"), Prefix), Location);
        Object->SetNumberField(FString::Printf(TEXT("%s_quat_x"), Prefix), Rotation.X);
        Object->SetNumberField(FString::Printf(TEXT("%s_quat_y"), Prefix), Rotation.Y);
        Object->SetNumberField(FString::Printf(TEXT("%s_quat_z"), Prefix), Rotation.Z);
        Object->SetNumberField(FString::Printf(TEXT("%s_quat_w"), Prefix), Rotation.W);
        CameraAuditSetVector(Object, *FString::Printf(TEXT("%s_scale"), Prefix), Scale);
    }

    TSharedRef<FJsonObject> CameraAuditResolvedProperties(const UObject* Object)
    {
        const TSharedRef<FJsonObject> Settings = MakeShared<FJsonObject>();
        if (!Object) return Settings;
        for (TFieldIterator<FProperty> It(Object->GetClass()); It; ++It)
        {
            const FProperty* Property = *It;
            if (!Property || Property->HasAnyPropertyFlags(CPF_Transient | CPF_Deprecated) ||
                CastField<FObjectPropertyBase>(Property) || CastField<FClassProperty>(Property) ||
                CastField<FDelegateProperty>(Property)) continue;
            if (const FArrayProperty* ArrayProperty = CastField<FArrayProperty>(Property))
            {
                if (CastField<FObjectPropertyBase>(ArrayProperty->Inner) || CastField<FClassProperty>(ArrayProperty->Inner)) continue;
            }
            const void* ValuePtr = Property->ContainerPtrToValuePtr<void>(Object);
            if (const FBoolProperty* BoolProperty = CastField<FBoolProperty>(Property))
            {
                Settings->SetBoolField(Property->GetName(), BoolProperty->GetPropertyValue(ValuePtr));
            }
            else if (const FNumericProperty* NumericProperty = CastField<FNumericProperty>(Property))
            {
                if (NumericProperty->IsFloatingPoint())
                {
                    Settings->SetNumberField(Property->GetName(), NumericProperty->GetFloatingPointPropertyValue(ValuePtr));
                }
                else if (NumericProperty->IsInteger())
                {
                    const bool bUnsignedInteger = CastField<FByteProperty>(NumericProperty) ||
                        CastField<FUInt16Property>(NumericProperty) || CastField<FUInt32Property>(NumericProperty) ||
                        CastField<FUInt64Property>(NumericProperty);
                    const int64 IntegerValue = bUnsignedInteger
                        ? static_cast<int64>(NumericProperty->GetUnsignedIntPropertyValue(ValuePtr))
                        : NumericProperty->GetSignedIntPropertyValue(ValuePtr);
                    Settings->SetNumberField(Property->GetName(), static_cast<double>(IntegerValue));
                }
                else
                {
                    FString ExportedValue;
                    Property->ExportText_InContainer(0, ExportedValue, Object, nullptr, const_cast<UObject*>(Object), PPF_None);
                    Settings->SetStringField(Property->GetName(), ExportedValue);
                }
            }
            else
            {
                FString ExportedValue;
                Property->ExportText_InContainer(0, ExportedValue, Object, nullptr, const_cast<UObject*>(Object), PPF_None);
                Settings->SetStringField(Property->GetName(), ExportedValue);
            }
        }
        return Settings;
    }

    FString CameraAuditProjectionState(const UDroneCaptureComponent* Capture)
    {
        if (!Capture) return TEXT("{}");
        FString Json;
        const TSharedRef<TJsonWriter<TCHAR>> Writer = TJsonWriterFactory<TCHAR>::Create(&Json);
        FJsonSerializer::Serialize(CameraAuditResolvedProperties(Capture), Writer);
        return Json;
    }

    // Skylight recapture creates its own six views, independent of the drone
    // camera's extension. Scope this registration to the synchronous recapture
    // so animated sky materials cannot bake wall-clock time into sensor light.
    class FSensorSkyCaptureTimeExtension final : public FWorldSceneViewExtension
    {
    public:
        FSensorSkyCaptureTimeExtension(const FAutoRegister& AutoRegister, UWorld* World,
            int32 Tick, float SecondsPerTick)
            : FWorldSceneViewExtension(AutoRegister, World)
            , CaptureTime(FGameTime::CreateUndilated(static_cast<double>(Tick) * SecondsPerTick, SecondsPerTick))
            , CaptureFrame(static_cast<uint32>(Tick))
        {
        }

        virtual void SetupViewFamily(FSceneViewFamily& Family) override
        {
            if (!Family.Views.IsEmpty() && Family.Views[0]->bIsReflectionCapture)
            {
                Family.Time = CaptureTime;
                Family.FrameNumber = CaptureFrame;
                Family.FrameCounter = CaptureFrame;
                Family.EngineShowFlags.SetDistanceFieldAO(false);
            }
        }

    private:
        const FGameTime CaptureTime;
        const uint32 CaptureFrame;
    };

    static constexpr const TCHAR* ThesisTerrainMaterialObjectPath =
        TEXT("/Game/Thesis/Materials/M_ThesisTerrain.M_ThesisTerrain");

    static FColor ColorForDroneIndex(int32 Idx)
    {
        static const FColor Palette[] = {
            FColor::Red, FColor::Green, FColor::Blue, FColor::Yellow,
            FColor::Cyan, FColor::Magenta, FColor::Orange, FColor::Purple,
        };
        if (Idx >= 0 && Idx < UE_ARRAY_COUNT(Palette))
        {
            return Palette[Idx];
        }
        const uint32 H = GetTypeHash(Idx);
        return FColor(uint8(H >> 16), uint8(H >> 8), uint8(H), 255);
    }

    static FColor PriorityToColor(float Priority, float MaxValue)
    {
        const float t = (MaxValue > KINDA_SMALL_NUMBER)
            ? FMath::Clamp(Priority / MaxValue, 0.0f, 1.0f)
            : 0.0f;
        return FLinearColor::LerpUsingHSV(FLinearColor::Blue, FLinearColor::Red, t).ToFColorSRGB();
    }

    static uint32 SensorSkySHFloatBits(float Value)
    {
        uint32 Bits = 0;
        FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
        return Bits;
    }

    static void LogSensorSkySHBits(const TArray<USkyLightComponent*>& SkyComponents, int32 Tick)
    {
        for (USkyLightComponent* Sky : SkyComponents)
        {
            if (!IsValid(Sky))
            {
                continue;
            }

            // This is the component's source SH snapshot after readiness flush, not the
            // effective/blended FSkyLightSceneProxy coefficients or cubemap contents.
            const FSHVectorRGB3 Irradiance = Sky->GetIrradianceEnvironmentMap();
            for (int32 Coefficient = 0; Coefficient < 9; ++Coefficient)
            {
                UE_LOG(LogTemp, Display,
                    TEXT("ThesisSkySH phase=%s tick=%d component=%s coefficient=%d Rbits=%08x Gbits=%08x Bbits=%08x"),
                    TEXT("post_readiness_flush"), Tick, *Sky->GetPathName(), Coefficient,
                    SensorSkySHFloatBits(Irradiance.R.V[Coefficient]),
                    SensorSkySHFloatBits(Irradiance.G.V[Coefficient]),
                    SensorSkySHFloatBits(Irradiance.B.V[Coefficient]));
            }
        }
    }

    struct FPythonLaunchCandidate
    {
        FString Executable;
        FString ArgsPrefix;
        FString Label;
    };

    static bool IsExecutableFileCandidate(const FString& Path)
    {
        return !Path.IsEmpty() && FPaths::FileExists(Path);
    }

    static FString NormalizeOptionalFullPath(FString Path)
    {
        Path.TrimStartAndEndInline();
        if (Path.IsEmpty())
        {
            return Path;
        }

        if (FPaths::IsRelative(Path))
        {
            Path = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir(), Path);
        }
        FPaths::NormalizeFilename(Path);
        return Path;
    }

    static void AddUniquePythonCandidate(TArray<FPythonLaunchCandidate>& Candidates, const FString& Executable, const FString& ArgsPrefix, const FString& Label)
    {
        FString CandidateExecutable = Executable;
        CandidateExecutable.TrimStartAndEndInline();
        if (CandidateExecutable.IsEmpty())
        {
            return;
        }

        for (const FPythonLaunchCandidate& Existing : Candidates)
        {
            if (Existing.Executable.Equals(CandidateExecutable, ESearchCase::IgnoreCase) && Existing.ArgsPrefix == ArgsPrefix)
            {
                return;
            }
        }

        Candidates.Add({ CandidateExecutable, ArgsPrefix, Label });
    }

    static TArray<FPythonLaunchCandidate> BuildPythonLaunchCandidates(const FString& ConfiguredPythonExecutable)
    {
        TArray<FPythonLaunchCandidate> Candidates;

        FString Configured = ConfiguredPythonExecutable;
        Configured.TrimStartAndEndInline();
        const bool bConfiguredIsAuto = Configured.IsEmpty()
            || Configured.Equals(TEXT("auto"), ESearchCase::IgnoreCase)
            || Configured.Equals(TEXT("python"), ESearchCase::IgnoreCase)
            || Configured.Equals(TEXT("python.exe"), ESearchCase::IgnoreCase);

        if (!bConfiguredIsAuto)
        {
            FString ConfiguredPath = Configured;
            if (ConfiguredPath.Contains(TEXT("/")) || ConfiguredPath.Contains(TEXT("\\")))
            {
                ConfiguredPath = NormalizeOptionalFullPath(ConfiguredPath);
            }
            AddUniquePythonCandidate(Candidates, ConfiguredPath, FString(), TEXT("configured PythonExecutablePath"));
        }

        // Prefer Unreal's bundled Python on Windows when available. This avoids the
        // Microsoft Store python.exe alias, which returns 9009 and does not run scripts.
        const FString EnginePythonWin64 = FPaths::ConvertRelativePathToFull(
            FPaths::EngineDir(),
            TEXT("Binaries/ThirdParty/Python3/Win64/python.exe"));
        if (IsExecutableFileCandidate(EnginePythonWin64))
        {
            AddUniquePythonCandidate(Candidates, EnginePythonWin64, FString(), TEXT("Unreal bundled Python"));
        }

        const FString ProjectVenvPythonWin64 = FPaths::ConvertRelativePathToFull(
            FPaths::ProjectDir(),
            TEXT(".venv/Scripts/python.exe"));
        if (IsExecutableFileCandidate(ProjectVenvPythonWin64))
        {
            AddUniquePythonCandidate(Candidates, ProjectVenvPythonWin64, FString(), TEXT("project .venv Python"));
        }

        // Windows Python launcher. It usually exists when Python was installed from python.org.
        AddUniquePythonCandidate(Candidates, TEXT("py"), TEXT("-3"), TEXT("Windows py launcher"));

        // PATH fallbacks. These are tried last because on Windows `python` may be the Store alias.
        AddUniquePythonCandidate(Candidates, TEXT("python3"), FString(), TEXT("python3 from PATH"));
        AddUniquePythonCandidate(Candidates, TEXT("python"), FString(), TEXT("python from PATH"));

        return Candidates;
    }

    static bool RunPythonScriptWithCandidates(
        const TArray<FPythonLaunchCandidate>& Candidates,
        const FString& ScriptAndArgs,
        FString& OutUsedCommand,
        FString& OutStdOut,
        FString& OutStdErr,
        int32& OutReturnCode)
    {
        FString FailureSummary;
        for (const FPythonLaunchCandidate& Candidate : Candidates)
        {
            const FString FullArgs = Candidate.ArgsPrefix.IsEmpty()
                ? ScriptAndArgs
                : Candidate.ArgsPrefix + TEXT(" ") + ScriptAndArgs;

            int32 ReturnCode = -1;
            FString StdOut;
            FString StdErr;
            const bool bLaunched = FPlatformProcess::ExecProcess(
                *Candidate.Executable,
                *FullArgs,
                &ReturnCode,
                &StdOut,
                &StdErr);

            if (bLaunched && ReturnCode == 0)
            {
                OutUsedCommand = Candidate.Executable + TEXT(" ") + FullArgs;
                OutStdOut = StdOut;
                OutStdErr = StdErr;
                OutReturnCode = ReturnCode;
                return true;
            }

            FailureSummary += FString::Printf(
                TEXT("[%s] Launched=%d ReturnCode=%d\nCommand: %s %s\nStdOut:\n%s\nStdErr:\n%s\n"),
                *Candidate.Label,
                bLaunched ? 1 : 0,
                ReturnCode,
                *Candidate.Executable,
                *FullArgs,
                *StdOut,
                *StdErr);
        }

        OutUsedCommand = TEXT("<all python candidates failed>");
        OutStdOut.Empty();
        OutStdErr = FailureSummary;
        OutReturnCode = -1;
        return false;
    }

}

ASimulationManager::~ASimulationManager() = default;

bool ASimulationManager::IsCameraStateAuditEnabled() const
{
    return bEnableCameraStateAudit || FParse::Param(FCommandLine::Get(), TEXT("ThesisCameraStateAudit"));
}

void ASimulationManager::BeginCameraStateAudit()
{
    CameraStateAuditCsvWriter.Reset();
    CameraStateAuditDirectory.Empty();
    CameraStateAuditCsvPath.Empty();
    bCameraStateAuditFailed = false;
    CameraStateAuditFailure.Empty();
    CameraAuditCaptureStates.Reset();
    if (!IsCameraStateAuditEnabled()) return;

    FString BaseDirectory;
    if (MetricsRecorder.IsActive())
    {
        BaseDirectory = MetricsRecorder.GetRunDirectory();
    }
    else
    {
        const FString RunLabel = MetricsRunId.IsEmpty()
            ? FString::Printf(TEXT("seed%d_%dx%d"), FireGeneratorSeed, GridWidth, GridHeight)
            : FPaths::MakeValidFileName(MetricsRunId);
        BaseDirectory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"),
            TEXT("camera_state_validation_1000_20261023"), RunLabel);
    }
    CameraStateAuditDirectory = FPaths::Combine(BaseDirectory, TEXT("camera_audit"));
    if (!IFileManager::Get().MakeDirectory(*CameraStateAuditDirectory, true))
    {
        FailCameraStateAudit(TEXT("Could not create camera audit output directory"));
        return;
    }
    CameraStateAuditCsvPath = FPaths::Combine(CameraStateAuditDirectory, TEXT("camera_state.csv"));
    if (FPaths::FileExists(CameraStateAuditCsvPath))
    {
        FailCameraStateAudit(TEXT("Camera audit CSV already exists; refusing to append to a prior run"));
        return;
    }
    CameraStateAuditCsvWriter.Reset(IFileManager::Get().CreateFileWriter(*CameraStateAuditCsvPath, FILEWRITE_AllowRead));
    if (!CameraStateAuditCsvWriter)
    {
        FailCameraStateAudit(FString::Printf(TEXT("Could not open camera audit CSV for shared-read writing: %s"), *CameraStateAuditCsvPath));
        return;
    }
    const FString Header = TEXT("event,engine_frame,capture_tick,consume_tick,dense_slot,logical_id,drone_name,cache_mode,rendered,capture_state_missing,cache_key,capture_position_xyz,capture_quaternion_xyzw,current_position_xyz,current_quaternion_xyzw,capture_fov_deg,current_fov_deg,capture_width,capture_height,readback_width,readback_height,projection_state_json,pixel_hash_algorithm,pixel_hash\n");
    if (!WriteCameraStateAuditCsv(Header)) return;

    const FString BaseConfig = FSimRunConfig::MakeConfigSnapshot(*this);
    TSharedPtr<FJsonObject> Config;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(BaseConfig);
    if (!FJsonSerializer::Deserialize(Reader, Config) || !Config.IsValid())
        Config = MakeShared<FJsonObject>();
    Config->SetNumberField(TEXT("camera_audit_schema_version"), 1);
    Config->SetBoolField(TEXT("camera_audit_enabled"), true);
    Config->SetStringField(TEXT("camera_audit_hash"), TEXT("FNV-1a 64-bit over explicit BGRA8 bytes; replay hash unavailable"));
    Config->SetBoolField(TEXT("gpu_audit_raw_snapshot_requested"), FParse::Param(FCommandLine::Get(), TEXT("ThesisCameraAuditRawGpuSnapshots")));
    Config->SetBoolField(TEXT("gpu_audit_raw_snapshot_enabled"), true);
    Config->SetStringField(TEXT("gpu_audit_snapshot_phase"), TEXT("server_slot_after_gpu_dispatch_swap_and_partition_before_vision_and_drone_updates"));
    TArray<TSharedPtr<FJsonValue>> GpuCheckpointTicks;
    for (const int32 CheckpointTick : { 0, 100, 500, 2000, 8000 })
        GpuCheckpointTicks.Add(MakeShared<FJsonValueNumber>(CheckpointTick));
    Config->SetArrayField(TEXT("gpu_audit_checkpoint_ticks"), GpuCheckpointTicks);
    Config->SetBoolField(TEXT("gpu_audit_also_capture_final_tick"), true);
    Config->SetStringField(TEXT("camera_audit_output_directory"), CameraStateAuditDirectory);
    const FString ResolvedCacheDirectory = VisionCacheDirName.IsEmpty()
        ? FString::Printf(TEXT("seed%d_%dx%d"), FireGeneratorSeed, GridWidth, GridHeight)
        : VisionCacheDirName;
    Config->SetStringField(TEXT("vision_cache_directory"), FPaths::ConvertRelativePathToFull(
        FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("VisionCache"), ResolvedCacheDirectory)));
    CameraAuditSetTransform(Config.ToSharedRef(), TEXT("simulation_manager"), GetActorTransform());
    CameraAuditSetVector(Config.ToSharedRef(), TEXT("grid_origin_world"), GridOriginWorld);
    Config->SetNumberField(TEXT("grid_width_resolved"), GridWidth);
    Config->SetNumberField(TEXT("grid_height_resolved"), GridHeight);
    Config->SetNumberField(TEXT("grid_cell_size_uu_resolved"), CellSizeUU);
    Config->SetNumberField(TEXT("world_tick_zero"), 0);

    TArray<TSharedPtr<FJsonValue>> DroneRecords;
    for (const TObjectPtr<ADrone>& DronePtr : RegisteredDrones)
    {
        const ADrone* Drone = DronePtr.Get();
        if (!Drone) continue;
        const TSharedRef<FJsonObject> Record = MakeShared<FJsonObject>();
        Record->SetNumberField(TEXT("dense_slot"), Drone->GetDroneIndex());
        Record->SetNumberField(TEXT("logical_id"), Drone->GetLogicalDroneId());
        Record->SetStringField(TEXT("actor_name"), Drone->GetName());
        Record->SetStringField(TEXT("planner_class"), Drone->PlannerClass ? Drone->PlannerClass->GetPathName() : TEXT("<default>"));
        Record->SetStringField(TEXT("sensor_class"), Drone->SensorClass ? Drone->SensorClass->GetPathName() : TEXT("<default>"));
        Record->SetBoolField(TEXT("target_cell_scoring_enabled"), Drone->bUseTargetCellScoring);
        Record->SetObjectField(TEXT("resolved_drone_properties"), CameraAuditResolvedProperties(Drone));
        Record->SetObjectField(TEXT("resolved_sensor_properties"), CameraAuditResolvedProperties(Drone->GetSensorInstance()));
        Record->SetObjectField(TEXT("resolved_planner_properties"), CameraAuditResolvedProperties(Drone->GetPlannerInstance()));
        CameraAuditSetTransform(Record, TEXT("drone_initial"), Drone->GetActorTransform());
        if (const UDroneCaptureComponent* Capture = Drone->GetCaptureComponent())
        {
            CameraAuditSetTransform(Record, TEXT("camera_initial"), Capture->GetComponentTransform());
            Record->SetNumberField(TEXT("camera_initial_fov_degrees"), Capture->FOVAngle);
            Record->SetNumberField(TEXT("camera_resolution_width"), FMath::Max(16, DroneCaptureResolution));
            Record->SetNumberField(TEXT("camera_resolution_height"), FMath::Max(16, DroneCaptureResolution));
            Record->SetObjectField(TEXT("resolved_capture_properties"), CameraAuditResolvedProperties(Capture));
        }
        DroneRecords.Add(MakeShared<FJsonValueObject>(Record));
    }
    Config->SetArrayField(TEXT("initial_drones"), DroneRecords);

    FString ConfigText;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
        TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&ConfigText);
    FJsonSerializer::Serialize(Config.ToSharedRef(), Writer);
    const FString ConfigPath = FPaths::Combine(CameraStateAuditDirectory, TEXT("resolved_config.json"));
    if (!FFileHelper::SaveStringToFile(ConfigText, *ConfigPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
    {
        UE_LOG(LogTemp, Error, TEXT("Camera state audit could not write resolved config: %s"), *ConfigPath);
        FailCameraStateAudit(FString::Printf(TEXT("Could not write resolved config: %s"), *ConfigPath));
        return;
    }
    if (UWorld* World = GetWorld())
    {
        if (UGpuBeliefManager* BeliefManager = World->GetSubsystem<UGpuBeliefManager>())
        {
            RecordGpuStateAudit(BeliefManager, 0);
        }
    }
    UE_LOG(LogTemp, Display, TEXT("Camera state audit enabled: %s"), *CameraStateAuditDirectory);
}

bool ASimulationManager::WriteCameraStateAuditCsv(const FString& Text)
{
    if (bCameraStateAuditFailed || !CameraStateAuditCsvWriter) return false;
    FTCHARToUTF8 Utf8(*Text);
    CameraStateAuditCsvWriter->Serialize(const_cast<ANSICHAR*>(Utf8.Get()), Utf8.Length());
    if (CameraStateAuditCsvWriter->IsError())
    {
        FailCameraStateAudit(TEXT("Camera audit CSV serialization failed"));
        return false;
    }
    CameraStateAuditCsvWriter->Flush();
    if (CameraStateAuditCsvWriter->IsError())
    {
        FailCameraStateAudit(TEXT("Camera audit CSV flush failed"));
        return false;
    }
    return true;
}

void ASimulationManager::FailCameraStateAudit(const FString& Reason)
{
    if (bCameraStateAuditFailed) return;
    bCameraStateAuditFailed = true;
    CameraStateAuditFailure = Reason;
    if (CameraStateAuditCsvWriter)
    {
        CameraStateAuditCsvWriter->Flush();
        CameraStateAuditCsvWriter.Reset();
    }
    const FString FailurePath = FPaths::Combine(CameraStateAuditDirectory, TEXT("camera_audit_failure.json"));
    TSharedRef<FJsonObject> Failure = MakeShared<FJsonObject>();
    Failure->SetBoolField(TEXT("camera_audit_failed"), true);
    Failure->SetStringField(TEXT("reason"), Reason);
    Failure->SetNumberField(TEXT("tick"), CurrentTick);
    Failure->SetNumberField(TEXT("engine_frame"), GFrameCounter);
    FString FailureText;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer = TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&FailureText);
    FJsonSerializer::Serialize(Failure, Writer);
    if (!CameraStateAuditDirectory.IsEmpty() &&
        !FFileHelper::SaveStringToFile(FailureText, *FailurePath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
    {
        UE_LOG(LogTemp, Error, TEXT("Camera state audit failed (%s); failure marker write also failed: %s"), *Reason, *FailurePath);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Camera state audit failed at tick %d: %s. Marker: %s"), CurrentTick, *Reason, *FailurePath);
    }
}

void ASimulationManager::RecordCameraCaptureAudit(const ADrone* Drone, const UDroneCaptureComponent* Capture,
    int32 CaptureTick, bool bRendered)
{
    if (CameraStateAuditCsvPath.IsEmpty() || bCameraStateAuditFailed || !Drone || !Capture) return;
    const int32 Slot = Drone->GetDroneIndex();
    FCameraAuditCaptureState& StateRecord = CameraAuditCaptureStates.FindOrAdd(Slot);
    StateRecord.CaptureTick = CaptureTick;
    StateRecord.EngineFrame = GFrameCounter;
    StateRecord.bRendered = bRendered;
    StateRecord.CameraTransform = Capture->GetComponentTransform();
    StateRecord.FOVDegrees = Capture->FOVAngle;
    StateRecord.Width = FMath::Max(16, DroneCaptureResolution);
    StateRecord.Height = FMath::Max(16, DroneCaptureResolution);
    StateRecord.ProjectionStateJson = CameraAuditProjectionState(Capture);

    const FString CacheDirectory = VisionCacheDirName.IsEmpty()
        ? FString::Printf(TEXT("seed%d_%dx%d"), FireGeneratorSeed, GridWidth, GridHeight)
        : VisionCacheDirName;
    const FString CacheKey = bEnableVisionBridge
        ? CameraAuditVisionCacheKey(CacheDirectory, Slot, CaptureTick)
        : FString();
    const FTransform& Pose = StateRecord.CameraTransform;
    const FString Row = FString::Join(TArray<FString>{
        CameraAuditCsv(TEXT("capture")), FString::Printf(TEXT("%llu"), static_cast<unsigned long long>(StateRecord.EngineFrame)), CameraAuditNumber(CaptureTick), TEXT("-1"),
        CameraAuditNumber(Slot), CameraAuditNumber(Drone->GetLogicalDroneId()), CameraAuditCsv(Drone->GetName()),
        CameraAuditCsv(VisionCacheMode == EVisionCacheMode::Replay ? TEXT("replay") :
            (VisionCacheMode == EVisionCacheMode::Record ? TEXT("record") : TEXT("off"))),
        bRendered ? TEXT("true") : TEXT("false"), TEXT("false"), CameraAuditCsv(CacheKey), CameraAuditCsv(CameraAuditVector(Pose.GetLocation())),
        CameraAuditCsv(CameraAuditQuat(Pose.GetRotation())), CameraAuditCsv(CameraAuditVector(Pose.GetLocation())), CameraAuditCsv(CameraAuditQuat(Pose.GetRotation())),
        CameraAuditNumber(StateRecord.FOVDegrees), CameraAuditNumber(StateRecord.FOVDegrees),
        CameraAuditNumber(StateRecord.Width), CameraAuditNumber(StateRecord.Height), TEXT("0"), TEXT("0"),
        CameraAuditCsv(StateRecord.ProjectionStateJson), TEXT(""), TEXT("")
    }, TEXT(",")) + TEXT("\n");
    WriteCameraStateAuditCsv(Row);
}

void ASimulationManager::RecordCameraConsumeAudit(const ADrone* Drone, const UDroneCaptureComponent* Capture,
    int32 CaptureTick, int32 ConsumeTick, bool bReplay, const TArray<FColor>* Pixels, int32 Width, int32 Height)
{
    if (CameraStateAuditCsvPath.IsEmpty() || bCameraStateAuditFailed || !Drone || !Capture) return;
    const FCameraAuditCaptureState* SavedState = CameraAuditCaptureStates.Find(Drone->GetDroneIndex());
    const bool bCaptureStateMissing = !SavedState || SavedState->CaptureTick != CaptureTick;
    if (bCaptureStateMissing)
    {
        UE_LOG(LogTemp, Error, TEXT("Camera state audit missing capture state: drone=%d capture_tick=%d consume_tick=%d."),
            Drone->GetDroneIndex(), CaptureTick, ConsumeTick);
    }
    const FTransform CurrentPose = Capture->GetComponentTransform();
    const FString CacheDirectory = VisionCacheDirName.IsEmpty()
        ? FString::Printf(TEXT("seed%d_%dx%d"), FireGeneratorSeed, GridWidth, GridHeight)
        : VisionCacheDirName;
    const FString CacheKey = bEnableVisionBridge
        ? CameraAuditVisionCacheKey(CacheDirectory, Drone->GetDroneIndex(), CaptureTick)
        : FString();
    const FString Hash = bReplay ? FString() : CameraAuditPixelHash(Pixels, Width, Height);
    const FString CapturePosition = bCaptureStateMissing
        ? FString() : CameraAuditCsv(CameraAuditVector(SavedState->CameraTransform.GetLocation()));
    const FString CaptureRotation = bCaptureStateMissing
        ? FString() : CameraAuditCsv(CameraAuditQuat(SavedState->CameraTransform.GetRotation()));
    const FString CaptureRendered = bCaptureStateMissing
        ? TEXT("unknown") : (SavedState->bRendered ? TEXT("true") : TEXT("false"));
    const FString CaptureFov = bCaptureStateMissing ? FString() : CameraAuditNumber(SavedState->FOVDegrees);
    const FString CaptureWidth = bCaptureStateMissing ? TEXT("0") : CameraAuditNumber(SavedState->Width);
    const FString CaptureHeight = bCaptureStateMissing ? TEXT("0") : CameraAuditNumber(SavedState->Height);
    const FString Row = FString::Join(TArray<FString>{
        CameraAuditCsv(TEXT("consume")), FString::Printf(TEXT("%llu"), static_cast<unsigned long long>(GFrameCounter)), CameraAuditNumber(CaptureTick), CameraAuditNumber(ConsumeTick),
        CameraAuditNumber(Drone->GetDroneIndex()), CameraAuditNumber(Drone->GetLogicalDroneId()), CameraAuditCsv(Drone->GetName()),
        CameraAuditCsv(bReplay ? TEXT("replay") :
            (VisionCacheMode == EVisionCacheMode::Record ? TEXT("record") : TEXT("off"))),
        CaptureRendered, bCaptureStateMissing ? TEXT("true") : TEXT("false"), CameraAuditCsv(CacheKey),
        CapturePosition, CaptureRotation,
        CameraAuditCsv(CameraAuditVector(CurrentPose.GetLocation())), CameraAuditCsv(CameraAuditQuat(CurrentPose.GetRotation())),
        CaptureFov, CameraAuditNumber(Capture->FOVAngle),
        CaptureWidth, CaptureHeight, CameraAuditNumber(bReplay ? 0 : Width),
        CameraAuditNumber(bReplay ? 0 : Height), CameraAuditCsv(bCaptureStateMissing ? FString() : SavedState->ProjectionStateJson),
        CameraAuditCsv(Hash.IsEmpty() ? TEXT("") : TEXT("fnv1a64-bgra8")), CameraAuditCsv(Hash)
    }, TEXT(",")) + TEXT("\n");
    WriteCameraStateAuditCsv(Row);
    CameraAuditCaptureStates.Remove(Drone->GetDroneIndex());
}

void ASimulationManager::RecordGpuStateAudit(UGpuBeliefManager* BeliefManager, int32 SourceTick)
{
    if (CameraStateAuditDirectory.IsEmpty() || !BeliefManager) return;

    TArray<FGpuAuditTextureSnapshot> Fields;
    FString CaptureError;
    const bool bCaptureSucceeded = BeliefManager->CaptureAuditSnapshot(SourceTick, Fields, CaptureError);
    const TSharedRef<FJsonObject> Snapshot = MakeShared<FJsonObject>();
    Snapshot->SetNumberField(TEXT("checkpoint_tick"), SourceTick);
    Snapshot->SetBoolField(TEXT("capture_succeeded"), bCaptureSucceeded);
    Snapshot->SetStringField(TEXT("capture_error"), CaptureError);
    Snapshot->SetNumberField(TEXT("width"), GridWidth);
    Snapshot->SetNumberField(TEXT("height"), GridHeight);
    Snapshot->SetStringField(TEXT("storage_order"), TEXT("row_major_y_then_x"));
    Snapshot->SetStringField(TEXT("snapshot_phase"), TEXT("server_slot_after_gpu_dispatch_swap_and_partition_before_vision_and_drone_updates"));
    // Save canonical bytes so a supported host-side SHA-256 implementation can
    // verify them after the run. Do not call FPlatformMisc::GetSHA256Signature:
    // several UE platform implementations deliberately assert as unsupported.
    Snapshot->SetBoolField(TEXT("raw_bytes_saved"), true);
    Snapshot->SetStringField(TEXT("hash_policy"), TEXT("sha256 computed by post-run Python validator from canonical raw bytes"));
    TArray<TSharedPtr<FJsonValue>> FieldRecords;

    for (const FGpuAuditTextureSnapshot& Field : Fields)
    {
        const TSharedRef<FJsonObject> Record = MakeShared<FJsonObject>();
        Record->SetStringField(TEXT("name"), Field.Name);
        Record->SetStringField(TEXT("format"), Field.Format);
        Record->SetNumberField(TEXT("width"), Field.Width);
        Record->SetNumberField(TEXT("height"), Field.Height);
        Record->SetNumberField(TEXT("source_tick"), Field.SourceTick);
        Record->SetBoolField(TEXT("available"), Field.bAvailable);
        Record->SetStringField(TEXT("unavailable_reason"), Field.UnavailableReason);
        const FString ReasonLower = Field.UnavailableReason.ToLower();
        Record->SetStringField(TEXT("availability_class"), Field.bAvailable ? TEXT("available") :
            (ReasonLower.Contains(TEXT("no gpu owner texture")) ? TEXT("unsupported_gpu_source") :
             (ReasonLower.Contains(TEXT("not dispatched")) ? TEXT("producer_inactive") : TEXT("resource_or_readback_failure"))));
        if (Field.bAvailable)
        {
            Record->SetNumberField(TEXT("byte_count"), Field.Bytes.Num());
            const FString SafeName = FPaths::MakeValidFileName(Field.Name);
            const FString RawPath = FPaths::Combine(CameraStateAuditDirectory,
                FString::Printf(TEXT("gpu_%s_t%06d.raw"), *SafeName, SourceTick));
            const bool bSaved = FFileHelper::SaveArrayToFile(Field.Bytes, *RawPath);
            Record->SetStringField(TEXT("data_file"), FPaths::GetCleanFilename(RawPath));
            Record->SetBoolField(TEXT("data_saved"), bSaved);
            Record->SetBoolField(TEXT("sha256_available"), false);
            if (!bSaved) Record->SetStringField(TEXT("save_error"), TEXT("SaveArrayToFile failed"));
        }
        FieldRecords.Add(MakeShared<FJsonValueObject>(Record));
    }
    Snapshot->SetArrayField(TEXT("gpu_fields"), FieldRecords);

    // Owner is a CPU partition result in this build; no GPU owner texture is
    // fabricated. UINT16 little-endian cells use 65535 for unowned.
    const FString OwnerPath = FPaths::Combine(CameraStateAuditDirectory,
        FString::Printf(TEXT("cpu_partition_owner_t%06d.u16le"), SourceTick));
    TArray<uint8> OwnerBytes;
    OwnerBytes.Reserve(GridWidth * GridHeight * sizeof(uint16));
    for (int32 Y = 0; Y < GridHeight; ++Y)
    {
        for (int32 X = 0; X < GridWidth; ++X)
        {
            const int32 CellOwner = DebugGetOwner(X, Y);
            const uint16 Encoded = CellOwner < 0 ? MAX_uint16 : static_cast<uint16>(CellOwner);
            OwnerBytes.Add(static_cast<uint8>(Encoded & 0xff));
            OwnerBytes.Add(static_cast<uint8>((Encoded >> 8) & 0xff));
        }
    }
    const bool bOwnerSaved = FFileHelper::SaveArrayToFile(OwnerBytes, *OwnerPath);
    Snapshot->SetStringField(TEXT("owner_source"), TEXT("CPU WorldGrid owner; GPU owner texture unavailable in this build"));
    Snapshot->SetStringField(TEXT("owner_data_file"), FPaths::GetCleanFilename(OwnerPath));
    Snapshot->SetStringField(TEXT("owner_format"), TEXT("uint16 little-endian; 65535=unowned"));
    Snapshot->SetBoolField(TEXT("owner_data_saved"), bOwnerSaved);
    Snapshot->SetBoolField(TEXT("owner_sha256_available"), false);

    FString SnapshotText;
    const TSharedRef<TJsonWriter<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>> Writer =
        TJsonWriterFactory<TCHAR, TPrettyJsonPrintPolicy<TCHAR>>::Create(&SnapshotText);
    FJsonSerializer::Serialize(Snapshot, Writer);
    const FString SnapshotPath = FPaths::Combine(CameraStateAuditDirectory,
        FString::Printf(TEXT("gpu_snapshot_t%06d.json"), SourceTick));
    if (!FFileHelper::SaveStringToFile(SnapshotText, *SnapshotPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM))
    {
        UE_LOG(LogTemp, Error, TEXT("Camera state audit could not write GPU snapshot metadata: %s"), *SnapshotPath);
    }
}

ASimulationManager::ASimulationManager()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    PrimaryActorTick.TickGroup = TG_PrePhysics;
    bAllowTickBeforeBeginPlay = false;

    FireGeneratorScriptPath.FilePath = TEXT("Tools/generate_fire_csv.py");
    GeneratedFireCsvOutputPath.FilePath = TEXT("Saved/GeneratedFire.csv");
    GeneratedHumansCsvOutputPath.FilePath = TEXT("Saved/GeneratedHumans.csv");
    GeneratedTerrainCsvOutputPath.FilePath = TEXT("Saved/GeneratedTerrain.csv");
    GeneratedFuelCsvOutputPath.FilePath = TEXT("Saved/GeneratedFuel.csv");
}


FString ASimulationManager::ResolveProjectRelativePath(const FFilePath& Path, const FString& DefaultRelativePath) const
{
    FString Resolved = Path.FilePath.IsEmpty() ? DefaultRelativePath : Path.FilePath;
    Resolved.TrimStartAndEndInline();

    if (Resolved.IsEmpty())
    {
        return FString();
    }

    const FString FullProjectDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectDir());
    if (FPaths::IsRelative(Resolved))
    {
        Resolved = FPaths::Combine(FullProjectDir, Resolved);
    }

    Resolved = FPaths::ConvertRelativePathToFull(Resolved);
    FPaths::NormalizeFilename(Resolved);
    return Resolved;
}

void ASimulationManager::ApplyMethodId(const FString& InMethodId)
{
    FMethodSpec Spec;
    if (!FMethodSpec::FromMethodId(InMethodId, Spec))
    {
        UE_LOG(LogTemp, Warning, TEXT("Thesis method: unknown method_id '%s'; keeping current method config."), *InMethodId);
        return;
    }

    MethodId = InMethodId;
    ActiveMethodSpec = Spec;
    InformationSourceName = FMethodSpec::ToConfigString(Spec.Info);
    RegionRegimeName = FMethodSpec::ToConfigString(Spec.Regime);
    PartitionStrategyName = Spec.PartitionStrategy.ToString();
    PlannerStrategyName = Spec.PlannerStrategy.ToString();
}

void ASimulationManager::RefreshMethodSpecFromNames()
{
    FMethodSpec Spec = ActiveMethodSpec;

    if (InformationSourceName.Equals(TEXT("oracle"), ESearchCase::IgnoreCase))
    {
        Spec.Info = EInformationSource::GroundTruthOracle;
    }
    else
    {
        Spec.Info = EInformationSource::BeliefFromSensors;
        InformationSourceName = TEXT("belief");
    }

    if (RegionRegimeName.Equals(TEXT("static_random"), ESearchCase::IgnoreCase))
    {
        Spec.Regime = ERegionRegime::StaticRandom;
        RegionRegimeName = TEXT("static_random");
    }
    else if (RegionRegimeName.Equals(TEXT("static_uniform"), ESearchCase::IgnoreCase) ||
        RegionRegimeName.Equals(TEXT("static"), ESearchCase::IgnoreCase))
    {
        Spec.Regime = ERegionRegime::StaticUniform;
        RegionRegimeName = TEXT("static_uniform");
    }
    else
    {
        Spec.Regime = ERegionRegime::Dynamic;
        RegionRegimeName = TEXT("dynamic");
    }

    Spec.PartitionStrategy = FName(*PartitionStrategyName);
    Spec.PlannerStrategy = FName(*PlannerStrategyName);
    if (Spec.Regime == ERegionRegime::StaticRandom)
    {
        Spec.PartitionStrategy = FName(TEXT("FullyRandomV1"));
        PartitionStrategyName = TEXT("FullyRandomV1");
    }
    if (Spec.Regime == ERegionRegime::StaticUniform)
    {
        const bool bSpawnAwareLawnmower = Spec.PlannerStrategy == FName(TEXT("LawnmowerV2")) ||
            Spec.PlannerStrategy == FName(TEXT("LawnmowerV1Aligned"));
        Spec.PartitionStrategy = FName(bSpawnAwareLawnmower ? TEXT("SpawnAwareUniform") : TEXT("Uniform"));
        PartitionStrategyName = bSpawnAwareLawnmower ? TEXT("SpawnAwareUniform") : TEXT("Uniform");
    }

    ActiveMethodSpec = Spec;
    EnsurePlannerStrategy();
}

void ASimulationManager::EnsurePlannerStrategy()
{
    const FName Wanted(*PlannerStrategyName);
    TSubclassOf<UDronePlannerBase> WantedClass = FDronePlannerFactory::CreatePlannerClass(Wanted);
    if (!WantedClass)
    {
        FString Known;
        for (const FName& Name : FDronePlannerFactory::RegisteredNames())
        {
            Known += Name.ToString() + TEXT(" ");
        }
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis: unknown planner strategy '%s' (registered: %s) - falling back to '%s'."),
               *PlannerStrategyName, *Known,
               *FDronePlannerFactory::DefaultStrategyName().ToString());
        PlannerStrategyName = FDronePlannerFactory::DefaultStrategyName().ToString();
        WantedClass = FDronePlannerFactory::CreatePlannerClass(FDronePlannerFactory::DefaultStrategyName());
    }

    DronePlannerClass = WantedClass;
    ActiveMethodSpec.PlannerStrategy = FName(*PlannerStrategyName);
}

void ASimulationManager::ApplyManagerConfigurationToDrone(ADrone* Drone) const
{
    if (!Drone)
    {
        return;
    }

    Drone->ApplyManagerConfiguration(
        DroneSpeedUUPerTick,
        DroneHorizontalFOVDegrees,
        DroneMaxViewDistanceUU,
        DroneReplanCadenceTicks,
        DroneCommitmentMargin,
        DroneDistancePenaltyPerCell,
        DroneTopKPathBonusCandidates,
        DroneArrivalRadiusUU,
        DroneFallbackReplanTicks,
        DroneDetourBudgetFraction,
        bDroneUseFastRoutePlanner,
        DroneFastRouteFOVGainWeight,
        DronePlannerScoringMode.Equals(TEXT("target_cell"), ESearchCase::IgnoreCase),
        // Parallel CPU drone processing cannot consume the async GPU readback on
        // worker threads, and the result is non-deterministic anyway; force the
        // deterministic CPU priority fallback whenever parallel is active so the
        // sequential and parallel paths compute identical results.
        bDroneUseLocalGpuPriority && !bParallelDroneProcessing,
        DroneLocalPriorityRequestIntervalTicks,
        DroneMaxLocalPriorityCacheAgeTicks,
        DroneNeighborPriorityWeight,
        DroneSensorClass,
        DronePlannerClass,
        SensorFirePositiveLLR,
        SensorFireNegativeLLR,
        SensorPersonPositiveLLR,
        SensorPersonNegativeLLR,
        PriorityWeightStaleness,
        PriorityWeightDanger,
        PriorityWeightPerson,
        PriorityStalenessSaturationTicks,
        PriorityProbabilityScale,
        PriorityMonitoringUrgencyTau,
        PriorityUnseenExplorationBonus,
        PriorityFireRampMultiplier,
        PriorityFireMonitoringUrgencyTau,
        PriorityFireMinimumUrgency,
        PriorityFireDiscoveryDelayTicks,
        PriorityFireDiscoveryRampTicks,
        PriorityPersonDiscoveryDelayTicks,
        PriorityPersonDiscoveryRampTicks,
        LoadWeightStaleness,
        LoadWeightDanger,
        LoadWeightPerson);

    Drone->ConfigureTerrainFlight(
        TerrainActor.Get(),
        DroneMinimumAltitudeAboveGroundUU,
        DroneCruiseAltitudeAboveGroundUU,
        DroneCameraForwardOffsetUU,
        DroneCameraDownOffsetUU);

    const bool bRealCameraDrone = IsDroneVisionCameraEnabled(Drone->GetDroneIndex());
    // Camera diagnostics alone retain the paper's ideal sensor. With the
    // vision bridge enabled, only external detections supply person evidence.
    Drone->SetExternalPersonSensing(bEnableVisionBridge && bRealCameraDrone);
    Drone->SetSeparatePersonFreshness(bEnableVisionBridge);
    Drone->ConfigureCapture(
        bRealCameraDrone,
        DroneCaptureResolution,
        DroneCaptureResolution,
        DroneHorizontalFOVDegrees,
        DroneCapturePitchDegrees,
        bSaveDroneCapturePngs);

    if (UDroneCaptureComponent* Cap = Drone->GetCaptureComponent())
    {
        Cap->SetExposure(bDroneCaptureAutoExposure, DroneCaptureExposureBias, DroneCaptureBloomIntensity, DroneCaptureManualEV100);
        Cap->SetFastUnshadowedRendering(CameraRenderProfile == TEXT("fast_unshadowed_v1") ||
            CameraRenderProfile == TEXT("fast_unshadowed_v2") || CameraRenderProfile == TEXT("fast_shader_v3"));
        Cap->SetDeterministicFirstCaptureWarmup(CameraRenderProfile == TEXT("fast_shader_v3"));
        Cap->ShowFlags.SetFog(!bDroneCaptureClearAir);
        Cap->ShowFlags.SetVolumetricFog(!bDroneCaptureClearAir);
        Cap->ShowFlags.SetCloud(!bDroneCaptureClearAir);
        Cap->DebugOutputDirectory = MetricsRunSubdir.IsEmpty() ? FString()
            : FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"), MetricsRunSubdir, TEXT("captures"));
    }

    // Spectator camera mirrors the capture transform (same FOV and pitch) so
    // "what the viewer sees" matches "what the vision model sees".
    Drone->ConfigureViewCamera(DroneHorizontalFOVDegrees, DroneCapturePitchDegrees);
}

bool ASimulationManager::IsDroneVisionCameraEnabled(int32 DroneIdx) const
{
    return bEnableDroneCaptures && DroneIdx >= 0 && DroneIdx < NumRealCameraDrones;
}

bool ASimulationManager::GenerateFireCsvFromManagerSettings()
{
    const FString ScriptPath = ResolveProjectRelativePath(
        FireGeneratorScriptPath,
        TEXT("Tools/generate_fire_csv.py"));

    FString OutputPath = ResolveProjectRelativePath(
        GeneratedFireCsvOutputPath,
        TEXT("Saved/GeneratedFire.csv"));

    if (ScriptPath.IsEmpty() || !FPaths::FileExists(ScriptPath))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis fire generator: script not found: %s"), *ScriptPath);
        return false;
    }

    if (OutputPath.IsEmpty())
    {
        OutputPath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("GeneratedFire.csv"));
        FPaths::NormalizeFilename(OutputPath);
    }

    IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutputPath), true);

    const int32 SafeIgnitionCount = FMath::Max(1,
        FireGeneratorNumFocusPoints > 0 ? FireGeneratorNumFocusPoints : FireGeneratorIgnitionCount);
    FireGeneratorIgnitionCount = SafeIgnitionCount;
    FireGeneratorNumFocusPoints = SafeIgnitionCount;
    const float SafeMinIgnitionDistance = FMath::Max(0.0f, FireGeneratorMinIgnitionDistance);
    const float SafeBurnPercentage = FMath::Clamp(FireGeneratorBurnPercentage, 0.0f, 100.0f);
    const int32 SafeFirstIgnitionTick = FMath::Max(1, FireGeneratorFirstIgnitionTick);
    const int32 SafeAdditionalIgnitionStartMinTick = FMath::Max(SafeFirstIgnitionTick, FireGeneratorAdditionalIgnitionStartMinTick);
    const int32 SafeAdditionalIgnitionStartMaxTick = FMath::Max(SafeAdditionalIgnitionStartMinTick, FireGeneratorAdditionalIgnitionStartMaxTick);

    // The importer treats <= 0 as "never burns", so generated fire times must be positive.
    const float SafeStartMin = FMath::Max(0.001f, FireGeneratorStartMin);
    const float SafeStartMax = FMath::Max(SafeStartMin + 0.001f, FireGeneratorStartMax);

    // 0 = auto-scale to grid size (handled by the script's --max-minutes <= 0 path).
    const float SafeMaxMinutes = FMath::Max(0.0f, FireGeneratorMaxMinutes);
    const float SafeSpreadMinutesPerCell = FMath::Max(0.001f, FireGeneratorSpreadMinutesPerCell);
    const float SafeSpreadTicksPerCell = FMath::Max(0.001f, FireGeneratorSpreadTicksPerCell);
    const float SafeGrowthAcceleration = FMath::Max(0.0f, FireGeneratorGrowthAcceleration);
    const float SafeTerrainCostInfluence = FMath::Max(0.0f, FireGeneratorTerrainCostInfluence);
    const float SafeWindStrength = FMath::Clamp(FireGeneratorWindStrength, 0.0f, 1.0f);
    const float SafeSpottingChance = FMath::Clamp(FireGeneratorSpottingChance, 0.0f, 1.0f);
    const float SafeMoisture = FMath::Clamp(FireGeneratorMoisture, 0.0f, 1.0f);
    const int32 SafePrecision = FMath::Max(0, FireGeneratorPrecision);
    const float SafeArrivalMinutesPerTick = FMath::Max(0.000001f, ArrivalMinutesPerTick);
    const int32 EstimatedBurningCells = FMath::Max(
        1,
        FMath::RoundToInt(static_cast<float>(FMath::Max(1, GridWidth) * FMath::Max(1, GridHeight)) * (SafeBurnPercentage / 100.0f)));
    const int32 AutoDurationTicks = FMath::Min(
        FMath::Max(1, FireGeneratorDefaultDurationTicks),
        FMath::Max(1, FMath::RoundToInt(static_cast<float>(EstimatedBurningCells) * FMath::Max(1.0f, FireGeneratorAutoDurationTicksPerBurningCell))));
    const int32 TargetDurationTicks = (MaxSimTicks > 0)
        ? MaxSimTicks
        : AutoDurationTicks;
    const float SafeDurationFillFraction = FMath::Clamp(FireGeneratorDurationFillFraction, 0.01f, 1.0f);

    FString ScriptAndArgs = FString::Printf(
        TEXT("\"%s\" --width %d --height %d --output \"%s\" --seed %d --ignition-count %d --num-focus-points %d --spread-variant %d --min-ignition-distance %.4f --burn-percentage %.4f --start-min %.4f --start-max %.4f --first-ignition-tick %d --additional-ignition-start-min-tick %d --additional-ignition-start-max-tick %d --max-minutes %.4f --spread-minutes-per-cell %.4f --spread-ticks-per-cell %.4f --growth-acceleration %.4f --terrain-cost-influence %.4f --wind-strength %.4f --spotting-chance %.6f --moisture %.4f --precision %d --target-duration-ticks %d --arrival-minutes-per-tick %.6f --duration-fill-fraction %.4f --cell-size-uu %.4f"),
        *ScriptPath,
        GridWidth,
        GridHeight,
        *OutputPath,
        FireGeneratorSeed,
        SafeIgnitionCount,
        SafeIgnitionCount,
        FMath::Clamp(FireGeneratorSpreadVariant, 0, 2),
        SafeMinIgnitionDistance,
        SafeBurnPercentage,
        SafeStartMin,
        SafeStartMax,
        SafeFirstIgnitionTick,
        SafeAdditionalIgnitionStartMinTick,
        SafeAdditionalIgnitionStartMaxTick,
        SafeMaxMinutes,
        SafeSpreadMinutesPerCell,
        SafeSpreadTicksPerCell,
        SafeGrowthAcceleration,
        SafeTerrainCostInfluence,
        SafeWindStrength,
        SafeSpottingChance,
        SafeMoisture,
        SafePrecision,
        TargetDurationTicks,
        SafeArrivalMinutesPerTick,
        SafeDurationFillFraction,
        FMath::Max(1.0f, CellSizeUU));

    if (!bFireGeneratorUseRandomWindDirection)
    {
        ScriptAndArgs += FString::Printf(
            TEXT(" --wind-angle-degrees %.4f"),
            FireGeneratorWindAngleDegrees);
    }

    const FString FuelCsv = ResolveProjectRelativePath(
        GeneratedFuelCsvOutputPath, TEXT("Saved/GeneratedFuel.csv"));
    if (FPaths::FileExists(FuelCsv))
    {
        ScriptAndArgs += FString::Printf(TEXT(" --fuel-csv \"%s\""), *FuelCsv);
    }

    const FString TerrainCsv = ResolveProjectRelativePath(
        GeneratedTerrainCsvOutputPath, TEXT("Saved/GeneratedTerrain.csv"));
    if (TerrainActor && FPaths::FileExists(TerrainCsv))
    {
        ScriptAndArgs += FString::Printf(TEXT(" --terrain-csv \"%s\""), *TerrainCsv);
    }

    const TArray<FPythonLaunchCandidate> PythonCandidates = BuildPythonLaunchCandidates(PythonExecutablePath);

    FString UsedCommand;
    FString StdOut;
    FString StdErr;
    int32 ReturnCode = -1;
    const bool bSuccess = RunPythonScriptWithCandidates(
        PythonCandidates,
        ScriptAndArgs,
        UsedCommand,
        StdOut,
        StdErr,
        ReturnCode);

    if (!bSuccess)
    {
        UE_LOG(LogTemp, Error,
               TEXT("Thesis fire generator failed. ReturnCode=%d\nLast/summary command: %s\nTried candidates:\n%s"),
               ReturnCode,
               *UsedCommand,
               *StdErr);
        UE_LOG(LogTemp, Error,
               TEXT("Fix: leave Python Executable empty/Auto to use Unreal's bundled Python, or set it to a real python.exe path such as C:/Users/<you>/AppData/Local/Programs/Python/Python312/python.exe. The Microsoft Store python alias is not enough."));
        return false;
    }

    UE_LOG(LogTemp, Log,
           TEXT("Thesis fire generator wrote %s\nCommand: %s\n%s"),
           *OutputPath,
           *UsedCommand,
           *StdOut);

    if (!FPaths::FileExists(OutputPath))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis fire generator reported success, but output CSV does not exist: %s"), *OutputPath);
        return false;
    }

    if (bLoadGeneratedFireCsvAfterRun)
    {
        FirstFireTickCSVPath.FilePath = OutputPath;
        // Reset only for in-session regeneration (editor button while running);
        // during BeginPlay (State == Stopped) the import right after handles it.
        if (WorldGrid.IsValid() && State != ESimState::Stopped)
        {
            ResetSimulation();
        }
    }

    return true;
}

bool ASimulationManager::GenerateHumansCsvFromManagerSettings()
{
    const FString ScriptPath = FPaths::ConvertRelativePathToFull(
        FPaths::ProjectDir(), TEXT("Tools/generate_humans.py"));

    const FString FireCsv = ResolveProjectRelativePath(FirstFireTickCSVPath, FString());
    if (ScriptPath.IsEmpty() || !FPaths::FileExists(ScriptPath))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis human generator: script not found: %s"), *ScriptPath);
        return false;
    }
    if (FireCsv.IsEmpty() || !FPaths::FileExists(FireCsv))
    {
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis human generator: no fire CSV at '%s' — humans need a fire to flee. Skipping."),
               *FireCsv);
        return false;
    }

    FString OutputPath = ResolveProjectRelativePath(
        GeneratedHumansCsvOutputPath, TEXT("Saved/GeneratedHumans.csv"));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(OutputPath), true);

    // Auto-scale person count with grid area unless set explicitly.
    const int32 NumPersons = (HumanGeneratorCount > 0)
        ? HumanGeneratorCount
        : FMath::Max(4, (GridWidth * GridHeight) / 750);
    const int32 Seed = (HumanGeneratorSeed != 0) ? HumanGeneratorSeed : FireGeneratorSeed;
    const int32 TrajectoryTicks = FMath::Max(1, HumanGeneratorMaxTicks);

    FString ScriptAndArgs = FString::Printf(
        TEXT("\"%s\" --fire-csv \"%s\" --width %d --height %d --arrival-minutes-per-tick %.4f --burn-duration-ticks %d --max-ticks %d --seed %d --num-persons %d --output \"%s\""),
        *ScriptPath,
        *FireCsv,
        GridWidth,
        GridHeight,
        FMath::Max(0.001f, ArrivalMinutesPerTick),
        FMath::Max(1, BurnDurationTicks),
        TrajectoryTicks,
        Seed,
        NumPersons,
        *OutputPath);

    const FString TerrainCsv = ResolveProjectRelativePath(
        GeneratedTerrainCsvOutputPath, TEXT("Saved/GeneratedTerrain.csv"));
    if (TerrainActor && FPaths::FileExists(TerrainCsv))
    {
        ScriptAndArgs += FString::Printf(TEXT(" --terrain-csv \"%s\""), *TerrainCsv);
    }

    const TArray<FPythonLaunchCandidate> PythonCandidates = BuildPythonLaunchCandidates(PythonExecutablePath);

    FString UsedCommand, StdOut, StdErr;
    int32 ReturnCode = -1;
    const bool bSuccess = RunPythonScriptWithCandidates(
        PythonCandidates, ScriptAndArgs, UsedCommand, StdOut, StdErr, ReturnCode);

    if (!bSuccess || !FPaths::FileExists(OutputPath))
    {
        UE_LOG(LogTemp, Error,
               TEXT("Thesis human generator failed. ReturnCode=%d\nCommand: %s\nStdErr:\n%s"),
               ReturnCode, *UsedCommand, *StdErr);
        return false;
    }

    UE_LOG(LogTemp, Log, TEXT("Thesis human generator wrote %s (%d persons, %d ticks, seed %d)\n%s"),
           *OutputPath, NumPersons, TrajectoryTicks, Seed, *StdOut);

    HumansCSVPath.FilePath = OutputPath;

    // If the human manager already exists (e.g. regenerating mid-session), reload.
    if (HumanManager)
    {
        HumanManager->HumanMeshScale = FMath::Max(0.1f, HumanMeshScale);
        HumanManager->VisualMeshSet = HumanVisualMeshSet;
        HumanManager->VisualPoseMode = HumanPoseMode;
        HumanManager->VisualSecondsPerTick = SecondsPerTick;
        HumanManager->VisualSeed = HumanGeneratorSeed != 0 ? HumanGeneratorSeed : FireGeneratorSeed;
        // Skip the per-tick skeletal-mesh update when nothing renders it (headless
        // -game with no captures); ground-truth human counts are written regardless.
        HumanManager->bUpdateVisualMeshes = bEnableDroneCaptures || (FApp::CanEverRender() && !bSuppressBenchmarkViewport);
        // Preserve the live viewport and legacy behavior. Only suppressed-viewport
        // runs with scheduled sensor renders can defer pose construction until a
        // camera actually needs it. The command-line opt-out supports same-build A/B.
        HumanManager->bDeferVisualUpdatesUntilCapture = bEnableDroneCaptures &&
            DroneCaptureEveryNTicks > 0 && bSuppressBenchmarkViewport &&
            !FParse::Param(FCommandLine::Get(), TEXT("ThesisLegacyHumanVisualUpdates"));
        HumanManager->LoadTrajectories(OutputPath);
        if (WorldGrid.IsValid())
        {
            HumanManager->ResetReplay(WorldGrid.Get());
            HumanManager->ApplyTick(CurrentTick, WorldGrid.Get());
        }
    }
    return true;
}

void ASimulationManager::BeginPlay()
{
    Super::BeginPlay();

    StaticInstance = this;

    // In-process scenario batching (opt-in): the GameInstance subsystem parsed
    // -SimConfigManifest once and survives level reloads. Each scenario runs in a
    // fresh level (OpenLevel at MaxSimTicks). Actor state is rebuilt; fresh-process
    // parity tests verify that surviving engine/RHI caches do not change outputs.
    if (UWorld* BatchWorld = GetWorld())
    {
        if (UGameInstance* GI = BatchWorld->GetGameInstance())
        {
            if (UThesisBenchmarkSubsystem* Bench = GI->GetSubsystem<UThesisBenchmarkSubsystem>())
            {
                if (Bench->IsBatch())
                {
                    SimConfigPath.FilePath = Bench->CurrentConfigPath();
                    UE_LOG(LogTemp, Log, TEXT("Thesis batch: scenario %d/%d -> %s"),
                           Bench->CurrentIndex() + 1, Bench->NumScenarios(), *SimConfigPath.FilePath);
                }
            }
        }
    }

    // Apply the optional JSON run config before anything depends on the values
    // (grid creation, fire generation, drone spawning). Command line wins over
    // the editor-set path; absence of both = pure editor-configured behaviour.
    {
        const FString ConfigPath = FSimRunConfig::ResolveConfigPath(SimConfigPath.FilePath);
        if (!ConfigPath.IsEmpty())
        {
            if (!FSimRunConfig::LoadAndApply(ConfigPath, *this))
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis: refusing to start with invalid run config %s."), *ConfigPath);
                State = ESimState::Stopped;
                return;
            }
        }
    }

    if (bSuppressBenchmarkViewport && GetWorld() && GetWorld()->IsGameWorld())
    {
        if (UGameViewportClient* Viewport = GetWorld()->GetGameViewport())
        {
            bPreviousDisableWorldRendering = Viewport->bDisableWorldRendering;
            Viewport->bDisableWorldRendering = true;
            bViewportSuppressionApplied = true;
        }
    }

    LiveRenderPolicy::Register(this);
    NormalizeEnvironmentLighting();
    LiveViewPresentation = FSceneViewExtensions::NewExtension<FLiveViewPresentationExtension>(GetWorld(), this);

    RefreshMethodSpecFromNames();
    EnsurePartitionStrategy();

    auto BeginSetupPhase = [this]() -> double
    {
        return bRecordWallClockTiming ? FPlatformTime::Seconds() : 0.0;
    };
    auto EndSetupPhase = [this](const TCHAR* PhaseName, double StartSeconds)
    {
        RecordWallClockPhaseTiming(-1, PhaseName, StartSeconds);
    };

    if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
    {
        EnableInput(PC);
        SetupDebugKeyBindings();

        // HUD hit-box buttons (drone view switching) need cursor + click events.
        PC->bShowMouseCursor = true;
        PC->bEnableClickEvents = true;
        PC->bEnableMouseOverEvents = true;

        // Ensure the custom HUD is active for canvas overlay
        if (!PC->GetHUD() || !PC->GetHUD()->IsA(ASimulationHUD::StaticClass()))
        {
            FActorSpawnParameters SpawnParams;
            SpawnParams.Owner = PC;
            ASimulationHUD* CustomHUD = GetWorld()->SpawnActor<ASimulationHUD>(
                ASimulationHUD::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
            if (CustomHUD)
            {
                PC->MyHUD = CustomHUD;
            }
        }
    }

    GridOriginWorld = GetActorLocation();
    GridOriginWorld.Z = 0.0f;

    {
        const double PhaseStart = BeginSetupPhase();
        WorldGrid = MakeUnique<Grid>(GridWidth, GridHeight);
        WorldGrid->Initialize();
        EndSetupPhase(TEXT("setup.grid_initialize"), PhaseStart);
    }

    // Terrain + forest BEFORE fire generation: the forest writes floraDensity,
    // which is exported as the fire generator's fuel map — so the fire burns
    // along the woods you actually see and stalls at clearings.
    {
        const double PhaseStart = BeginSetupPhase();
        BuildTerrainIfEnabled();
        EndSetupPhase(TEXT("setup.build_terrain"), PhaseStart);
    }
    {
        const double PhaseStart = BeginSetupPhase();
        BuildForestIfPresent();
        EndSetupPhase(TEXT("setup.build_forest"), PhaseStart);
    }
    {
        const double PhaseStart = BeginSetupPhase();
        ExportFuelCsvFromGrid();
        EndSetupPhase(TEXT("setup.export_fuel_csv"), PhaseStart);
    }

    if (bGenerateFireCsvOnBeginPlay || bAutoGenerateScenarioOnBeginPlay)
    {
        const double PhaseStart = BeginSetupPhase();
        const FString CachedFireCsv = ResolveProjectRelativePath(
            GeneratedFireCsvOutputPath, TEXT("Saved/GeneratedFire.csv"));
        if (bReuseGeneratedScenarioCsvs && FPaths::FileExists(CachedFireCsv))
        {
            FirstFireTickCSVPath.FilePath = CachedFireCsv;
            UE_LOG(LogTemp, Log, TEXT("Thesis fire generator: reusing cached scenario CSV %s"), *CachedFireCsv);
        }
        else
        {
            GenerateFireCsvFromManagerSettings();
        }
        EndSetupPhase(TEXT("setup.generate_fire_csv"), PhaseStart);
    }

    bool bFireLoaded = false;
    const double FireImportPhaseStart = BeginSetupPhase();
    const FString& CsvPath = FirstFireTickCSVPath.FilePath;
    if (!CsvPath.IsEmpty())
    {
        TArray<int32> Imported;
        if (UFireDataImporter::ImportFirstFireTicksFromCSV(
                CsvPath, GridWidth, GridHeight,
                ArrivalMinutesPerTick, BurnDurationTicks, Imported))
        {
            WorldGrid->SetFirstFireTickArray(Imported);
            bFireLoaded = true;
        }
    }

    if (!bFireLoaded && bUseFallbackFireIfNoCSV)
    {
        TArray<int32> Fallback;
        UFireDataImporter::GenerateFallbackCentreFire(
            GridWidth, GridHeight,
            FallbackIgnitionRadiusCells,
            FallbackSpreadTicksPerCell,
            FallbackMaxSpreadCells,
            Fallback);
        WorldGrid->SetFirstFireTickArray(Fallback);
        UE_LOG(LogTemp, Log, TEXT("Thesis: no Cell2Fire CSV — using fallback centre ignition (radius=%d, spread=%d/cell)."),
               FallbackIgnitionRadiusCells, FallbackSpreadTicksPerCell);
    }

    EndSetupPhase(TEXT("setup.import_fire_csv"), FireImportPhaseStart);

    if (bAutoGenerateScenarioOnBeginPlay && bFireLoaded)
    {
        const double PhaseStart = BeginSetupPhase();
        const FString CachedHumansCsv = ResolveProjectRelativePath(
            GeneratedHumansCsvOutputPath, TEXT("Saved/GeneratedHumans.csv"));
        if (bReuseGeneratedScenarioCsvs && FPaths::FileExists(CachedHumansCsv))
        {
            HumansCSVPath.FilePath = CachedHumansCsv;
            UE_LOG(LogTemp, Log, TEXT("Thesis human generator: reusing cached scenario CSV %s"), *CachedHumansCsv);
        }
        else
        {
            GenerateHumansCsvFromManagerSettings();
        }
        EndSetupPhase(TEXT("setup.generate_humans_csv"), PhaseStart);
    }

    // Fire visuals need the imported fire plan + flora + terrain.
    {
        const double PhaseStart = BeginSetupPhase();
        SetupFireVisuals();
        EndSetupPhase(TEXT("setup.setup_fire_visuals"), PhaseStart);
    }

    if (UWorld* World = GetWorld())
    {
        if (UGpuBeliefManager* BeliefMgr = World->GetSubsystem<UGpuBeliefManager>())
        {
            const double PhaseStart = BeginSetupPhase();
            BeliefMgr->InitializeTextures(GridWidth, GridHeight);
            EndSetupPhase(TEXT("setup.gpu_initialize_textures"), PhaseStart);
        }

        const double DroneSetupPhaseStart = BeginSetupPhase();
        TArray<AActor*> Found;
        UGameplayStatics::GetAllActorsOfClass(World, ADrone::StaticClass(), Found);
        Found.Sort([](const AActor& A, const AActor& B)
        {
            return A.GetUniqueID() < B.GetUniqueID();
        });
        if (BenchmarkDroneLogicalIds.Num() > 0 && Found.Num() > AutoDroneCount)
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis identity-mapped run has %d pre-placed drones but config permits %d; refusing to leave extra agents active."),
                   Found.Num(), AutoDroneCount);
            State = ESimState::Stopped;
            return;
        }
        RegisteredDrones.Reset();
        for (AActor* A : Found)
        {
            if (ADrone* D = Cast<ADrone>(A))
            {
                RegisterDrone(D);
                if (BenchmarkDroneSpawnCellPositions.IsValidIndex(D->GetDroneIndex()))
                {
                    const FVector2D SpawnCell = BenchmarkDroneSpawnCellPositions[D->GetDroneIndex()];
                    const float GroundZ = TerrainActor
                        ? TerrainActor->GetHeightAtCell(SpawnCell.X, SpawnCell.Y)
                        : GridOriginWorld.Z;
                    D->SetActorLocation(FVector(
                        GridOriginWorld.X + SpawnCell.X * CellSizeUU,
                        GridOriginWorld.Y + SpawnCell.Y * CellSizeUU,
                        GroundZ + DroneCruiseAltitudeAboveGroundUU), false, nullptr, ETeleportType::TeleportPhysics);
                    D->SetActorRotation(FRotator::ZeroRotator, ETeleportType::TeleportPhysics);
                }
                D->InitFromManager(WorldGrid.Get(), D->GetDroneIndex(), GridOriginWorld, CellSizeUU);
                ApplyManagerConfigurationToDrone(D);
            }
        }

        const int32 PreExisting = RegisteredDrones.Num();
        if (bAutoSpawnDronesIfEmpty && PreExisting < AutoDroneCount)
        {
            const int32 NumToSpawn = AutoDroneCount - PreExisting;
            UClass* ClassToSpawn = AutoDroneClass ? AutoDroneClass.Get() : ADrone::StaticClass();

            // Place drones on the perimeter of the grid, at the centres of edge cells.
            const float HalfCellUU = CellSizeUU * 0.5f;

            // The centre-line perimeter length (following cell centres along the edges)
            const float PerimeterUU =
                2.0f * ((GridWidth - 1) * CellSizeUU + (GridHeight - 1) * CellSizeUU);

            const int32 TotalSlots = FMath::Max(AutoDroneCount, 1);
            const float StepDist = PerimeterUU / TotalSlots;

            for (int32 i = 0; i < NumToSpawn; ++i)
            {
                // Treat all AutoDroneCount positions as a ring, continuing from where
                // manually placed drones would sit (they are assumed to occupy the first
                // PreExisting slots).
                const int32 SlotIndex = (PreExisting + i) % TotalSlots;
                float DistAlong = SlotIndex * StepDist;

                // Walk along the four edges clockwise, starting at the bottom‑left cell centre.
                FVector Loc;
                Loc.Z = GridOriginWorld.Z + FMath::Max(AutoDroneSpawnZ, DroneCruiseAltitudeAboveGroundUU);

                // 1) Bottom edge (left → right)
                const float BottomLen = (GridWidth - 1) * CellSizeUU;
                if (DistAlong < BottomLen)
                {
                    Loc.X = GridOriginWorld.X + HalfCellUU + DistAlong;
                    Loc.Y = GridOriginWorld.Y + HalfCellUU;
                }
                else
                {
                    DistAlong -= BottomLen;
                    // 2) Right edge (bottom → top)
                    const float RightLen = (GridHeight - 1) * CellSizeUU;
                    if (DistAlong < RightLen)
                    {
                        Loc.X = GridOriginWorld.X + (GridWidth - 1 + 0.5f) * CellSizeUU;
                        Loc.Y = GridOriginWorld.Y + HalfCellUU + DistAlong;
                    }
                    else
                    {
                        DistAlong -= RightLen;
                        // 3) Top edge (right → left)
                        const float TopLen = (GridWidth - 1) * CellSizeUU;
                        if (DistAlong < TopLen)
                        {
                            Loc.X = GridOriginWorld.X + (GridWidth - 1 + 0.5f) * CellSizeUU - DistAlong;
                            Loc.Y = GridOriginWorld.Y + (GridHeight - 1 + 0.5f) * CellSizeUU;
                        }
                        else
                        {
                            DistAlong -= TopLen;
                            // 4) Left edge (top → bottom)
                            Loc.X = GridOriginWorld.X + HalfCellUU;
                            Loc.Y = GridOriginWorld.Y + (GridHeight - 1 + 0.5f) * CellSizeUU - DistAlong;
                        }
                    }
                }

                if (BenchmarkDroneSpawnCellPositions.IsValidIndex(SlotIndex))
                {
                    const FVector2D SpawnCell = BenchmarkDroneSpawnCellPositions[SlotIndex];
                    Loc.X = GridOriginWorld.X + SpawnCell.X * CellSizeUU;
                    Loc.Y = GridOriginWorld.Y + SpawnCell.Y * CellSizeUU;
                }

                if (TerrainActor)
                {
                    const float CellX = (Loc.X - GridOriginWorld.X) / CellSizeUU;
                    const float CellY = (Loc.Y - GridOriginWorld.Y) / CellSizeUU;
                    Loc.Z = TerrainActor->GetHeightAtCell(CellX, CellY) + DroneCruiseAltitudeAboveGroundUU;
                }

                FActorSpawnParameters Params;
                Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                if (ADrone* D = World->SpawnActor<ADrone>(ClassToSpawn, Loc, FRotator::ZeroRotator, Params))
                {
                    RegisterDrone(D);
                    D->InitFromManager(WorldGrid.Get(), D->GetDroneIndex(), GridOriginWorld, CellSizeUU);
                    ApplyManagerConfigurationToDrone(D);
                }
            }
            UE_LOG(LogTemp, Log,
                   TEXT("Thesis: auto-spawned %d drones (target=%d, pre-existing=%d) along grid perimeter."),
                   NumToSpawn, AutoDroneCount, PreExisting);
        }
        else
        {
            UE_LOG(LogTemp, Log, TEXT("Thesis: discovered %d drones in level."),
                   RegisteredDrones.Num());
        }
        if (BenchmarkDroneLogicalIds.Num() > 0 && RegisteredDrones.Num() != AutoDroneCount)
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis identity-mapped run expected %d active drones after setup, but has %d; refusing to start."),
                   AutoDroneCount, RegisteredDrones.Num());
            State = ESimState::Stopped;
            return;
        }
        EndSetupPhase(TEXT("setup.drone_discovery_spawn"), DroneSetupPhaseStart);
    }

    {
        const double PhaseStart = BeginSetupPhase();
        ApplyManagerSettingsToAllDrones();
        EndSetupPhase(TEXT("setup.apply_drone_settings"), PhaseStart);
    }

    // Remember canonical spawn positions so in-process scenario swaps can restore
    // them (same-grid batches => identical spawn layout every scenario).
    CaptureInitialDroneSpawnLocations();

    if (bAutoStart && RegisteredDrones.Num() > 0)
    {
        const double PhaseStart = BeginSetupPhase();
        if (UWorld* World = GetWorld())
        {
            if (UGpuBeliefManager* BeliefMgr = World->GetSubsystem<UGpuBeliefManager>())
            {
                TArray<FVector2f> InitialPositions;
                InitialPositions.Reserve(RegisteredDrones.Num());
                for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
                {
                    if (ADrone* D = Drone.Get())
                    {
                        const FVector W = D->GetActorLocation();
                        InitialPositions.Emplace(
                            static_cast<float>((W.X - GridOriginWorld.X) / CellSizeUU),
                            static_cast<float>((W.Y - GridOriginWorld.Y) / CellSizeUU));
                    }
                }
                // Initial adaptive-zone partition. Partitioning is computed CPU-side
                // by the selected IPartitionStrategy.
                TArray<TObjectPtr<ADrone>> ActiveDronesInit;
                ActiveDronesInit.Reserve(RegisteredDrones.Num());
                for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
                {
                    if (Drone.Get()) ActiveDronesInit.Add(Drone);
                }
                TArray<uint16> InitOwners = ComputeBalancedPartition(ActiveDronesInit, InitialPositions, 0);
                ApplyPartitionOwnersToGrid(InitOwners);
            }
        }
        EndSetupPhase(TEXT("setup.initial_partition"), PhaseStart);
    }

    {
        const double PhaseStart = BeginSetupPhase();
        EnsureHumanManager();
        EndSetupPhase(TEXT("setup.ensure_human_manager"), PhaseStart);
    }
    if (HumanManager)
    {
        const double PhaseStart = BeginSetupPhase();
        // Ground truth for tick 0 so the first drone sensing pass sees humans.
        HumanManager->ApplyTick(0, WorldGrid.Get());
        EndSetupPhase(TEXT("setup.humans_apply_tick0"), PhaseStart);
    }

    if (bEnableYoloDatasetExport && !InitializeDatasetExport()) return;

    if (bAutoPositionCamera)
    {
        const FVector GridCentre(
            GridOriginWorld.X + GridWidth  * 0.5f * CellSizeUU,
            GridOriginWorld.Y + GridHeight * 0.5f * CellSizeUU,
            CameraOverviewHeightUU);
        const FRotator LookDown(-60.0f, 0.0f, 0.0f);

        if (APlayerController* PC = GetWorld()->GetFirstPlayerController())
        {
            PC->SetInitialLocationAndRotation(GridCentre, LookDown);
            if (APawn* Pawn = PC->GetPawn())
            {
                Pawn->TeleportTo(GridCentre, LookDown);
            }
        }
    }

    if (bAutoStart)
    {
        const double PhaseStart = BeginSetupPhase();
        StartSimulation();
        EndSetupPhase(TEXT("setup.start_simulation"), PhaseStart);
    }
}


void ASimulationManager::ApplyManagerSettingsToAllDrones()
{
    RefreshMethodSpecFromNames();
    for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
    {
        ApplyManagerConfigurationToDrone(Drone.Get());
    }
}

void ASimulationManager::RegisterDrone(ADrone* Drone)
{
    if (!Drone) return;
    Drone->DroneIndex = RegisteredDrones.Num();
    Drone->SetLogicalDroneId(BenchmarkDroneLogicalIds.IsValidIndex(Drone->GetDroneIndex())
        ? BenchmarkDroneLogicalIds[Drone->GetDroneIndex()]
        : Drone->GetDroneIndex());
    RegisteredDrones.Add(Drone);
}

void ASimulationManager::NormalizeEnvironmentLighting()
{
    // Prefer a complete authored sun/sky rig when the level
    // also contains standalone environment lights. Two suns
    // claiming atmosphere slot zero can otherwise illuminate
    // surfaces and atmospheric scattering at different scales.
    TArray<UDirectionalLightComponent*> AtmosphereSuns;
    for (TObjectIterator<UDirectionalLightComponent> It; It; ++It)
    {
        if (It->GetWorld() == GetWorld() && It->IsRegistered() &&
            It->IsVisible() && It->bAtmosphereSunLight && It->Intensity > 0.0f)
        {
            AtmosphereSuns.Add(*It);
        }
    }
    AtmosphereSuns.Sort([](const UDirectionalLightComponent& A, const UDirectionalLightComponent& B)
    {
        return A.Intensity != B.Intensity ? A.Intensity > B.Intensity : A.GetPathName() < B.GetPathName();
    });
    AActor* EnvironmentRig = AtmosphereSuns.IsEmpty() ? nullptr : AtmosphereSuns[0]->GetOwner();
    if (EnvironmentRig && EnvironmentRig->FindComponentByClass<USkyLightComponent>() &&
        EnvironmentRig->FindComponentByClass<USkyAtmosphereComponent>())
    {
        for (UDirectionalLightComponent* Sun : AtmosphereSuns)
        {
            if (Sun != AtmosphereSuns[0] && Sun->AtmosphereSunLightIndex == AtmosphereSuns[0]->AtmosphereSunLightIndex)
            {
                SensorOriginalAtmosphereSun.FindOrAdd(Sun, Sun->bAtmosphereSunLight);
                SensorOriginalVisibility.FindOrAdd(Sun, Sun->IsVisible());
                Sun->SetAtmosphereSunLight(false);
                Sun->SetVisibility(false);
            }
        }
        for (TObjectIterator<USkyAtmosphereComponent> It; It; ++It)
        {
            if (It->GetWorld() == GetWorld() && It->GetOwner() != EnvironmentRig)
            {
                SensorOriginalVisibility.FindOrAdd(*It, It->IsVisible());
                It->SetVisibility(false);
            }
        }
        for (TObjectIterator<USkyLightComponent> It; It; ++It)
        {
            if (It->GetWorld() == GetWorld() && It->GetOwner() != EnvironmentRig)
            {
                SensorOriginalVisibility.FindOrAdd(*It, It->IsVisible());
                It->SetVisibility(false);
            }
        }
        // Re-register the selected atmosphere sun after the
        // duplicate has vacated its scene slot.
        AtmosphereSuns[0]->MarkRenderStateDirty();
        UE_LOG(LogTemp, Display, TEXT("Simulation environment rig: %s"), *EnvironmentRig->GetPathName());
    }
}

void ASimulationManager::RestoreSensorResources()
{
    for (const auto& Entry : SensorTextureResidency)
    {
        if (UTexture2D* Texture = Entry.Key.Get())
        {
            Texture->bForceMiplevelsToBeResident = Entry.Value;
        }
    }
    SensorTextureResidency.Reset();
    for (const auto& Entry : SensorOriginalVisibility)
        if (USceneComponent* Component = Entry.Key.Get()) Component->SetVisibility(Entry.Value);
    for (const auto& Entry : SensorOriginalAtmosphereSun)
        if (UDirectionalLightComponent* Sun = Entry.Key.Get()) Sun->SetAtmosphereSunLight(Entry.Value);
    for (const auto& Entry : SensorOriginalRealtimeSky)
        if (USkyLightComponent* Sky = Entry.Key.Get()) Sky->SetRealTimeCaptureEnabled(Entry.Value);
    for (const auto& Entry : SensorOriginalSkyCubemap)
        if (USkyLightComponent* Sky = Entry.Key.Get())
        {
            if (const int32* Source = SensorOriginalSkySource.Find(Sky))
                Sky->SourceType = static_cast<ESkyLightSourceType>(*Source);
            Sky->SetCubemap(Entry.Value.Get());
        }
    SensorOriginalVisibility.Reset();
    SensorOriginalAtmosphereSun.Reset();
    SensorOriginalRealtimeSky.Reset();
    SensorOriginalSkySource.Reset();
    SensorOriginalSkyCubemap.Reset();
    bSensorSkyPrepared = false;
    bSensorTexturesPrepared = false;
}

void ASimulationManager::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Release the detector session before a batched level opens its replacement.
    // Actor destruction/GC may happen later than EndPlay.
    VisionBridge.Reset();
    LiveRenderPolicy::Unregister(this);
    LiveViewPresentation.Reset();
    RestoreSensorResources();
    if (bViewportSuppressionApplied && GetWorld())
    {
        if (UGameViewportClient* Viewport = GetWorld()->GetGameViewport())
        {
            Viewport->bDisableWorldRendering = bPreviousDisableWorldRendering;
        }
        bViewportSuppressionApplied = false;
    }
    if (StaticInstance == this)
    {
        StaticInstance = nullptr;
    }

    MetricsRecorder.EndRun(CurrentTick);
    WorldGrid.Reset();
    Super::EndPlay(EndPlayReason);
}

bool ASimulationManager::CanUseParallelDroneProcessing() const
{
    if (!bParallelDroneProcessing)
    {
        return false;
    }

    // Oracle targeting reaches back into manager/human ground truth during
    // replanning, so keep it on the original game-thread path for now.
    if (PlannerStrategyName.Equals(TEXT("OracleTarget"), ESearchCase::IgnoreCase))
    {
        return false;
    }

    for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
    {
        const ADrone* D = Drone.Get();
        if (D && !D->SupportsParallelCpuTick())
        {
            return false;
        }
    }

    return true;
}

void ASimulationManager::RecordDroneProcessingModeEvent()
{
    if (!MetricsRecorder.IsActive())
    {
        return;
    }

    const bool bEffectiveParallel = CanUseParallelDroneProcessing();
    FString Detail = FString::Printf(
        TEXT("requested=%s,effective=%s"),
        bParallelDroneProcessing ? TEXT("parallel_cpu") : TEXT("sequential"),
        bEffectiveParallel ? TEXT("parallel_cpu") : TEXT("sequential"));

    if (bParallelDroneProcessing &&
        PlannerStrategyName.Equals(TEXT("OracleTarget"), ESearchCase::IgnoreCase))
    {
        Detail += TEXT(",fallback=oracle_planner");
    }

    MetricsRecorder.RecordEvent(0, TEXT("drone_processing_mode"), Detail);
}

void ASimulationManager::TickDronesSequential(int32 AtTick)
{
    TArray<FDroneTickProfile> Profiles;
    if (bRecordWallClockTiming)
    {
        Profiles.Reserve(RegisteredDrones.Num());
    }

    // Sensor-only ground truth for world-space person gating. Re-injected each
    // tick so scenario resets / sensor recreation can never leave a stale
    // pointer. Cheap (N drones), single-threaded here, and the snapshot array
    // is immutable until the human update after the drone phase.
    {
        const TArray<FHumanSnapshot>* Snaps =
            HumanManager ? &HumanManager->GetCurrentSnapshots() : nullptr;
        for (TObjectPtr<ADrone>& Drone : RegisteredDrones)
        {
            if (ADrone* D = Drone.Get())
            {
                D->SetPersonGroundTruth(Snaps);
            }
        }
    }

    for (TObjectPtr<ADrone>& Drone : RegisteredDrones)
    {
        if (ADrone* D = Drone.Get())
        {
            D->TickDrone(AtTick, bRecordWallClockTiming);
            if (bRecordWallClockTiming)
            {
                Profiles.Add(D->GetLastTickProfile());
            }
            D->AppendPrivateObservationsTo(PendingObservations);
        }
    }

    RecordDroneTickProfileTimings(AtTick, TEXT("drone.sequential"), Profiles);
}

void ASimulationManager::TickDronesParallelCpu(int32 AtTick)
{
    const int32 NumSlots = RegisteredDrones.Num();
    ParallelDroneScratchDrones.SetNumZeroed(NumSlots, EAllowShrinking::No);
    ParallelDroneScratchLocations.SetNum(NumSlots, EAllowShrinking::No);
    ParallelDroneScratchResults.SetNum(NumSlots, EAllowShrinking::No);

    // Sensor-only ground truth for world-space person gating (see the
    // sequential path). Injected on the game thread BEFORE the ParallelFor;
    // read-only during the parallel phase, so this is race-free.
    {
        const TArray<FHumanSnapshot>* Snaps =
            HumanManager ? &HumanManager->GetCurrentSnapshots() : nullptr;
        for (TObjectPtr<ADrone>& Drone : RegisteredDrones)
        {
            if (ADrone* D = Drone.Get())
            {
                D->SetPersonGroundTruth(Snaps);
            }
        }
    }

    for (int32 I = 0; I < NumSlots; ++I)
    {
        ParallelDroneScratchDrones[I] = RegisteredDrones[I].Get();
        ParallelDroneScratchResults[I] = FDroneParallelTickResult();
        if (ParallelDroneScratchDrones[I])
        {
            ParallelDroneScratchLocations[I] = ParallelDroneScratchDrones[I]->GetActorLocation();
        }
    }

    const bool bUseRandomPlanner = PlannerStrategyName.Equals(TEXT("Random"), ESearchCase::IgnoreCase);
    const bool bUseFullyRandomPlanner = PlannerStrategyName.Equals(TEXT("FullyRandomV1"), ESearchCase::IgnoreCase);
    const bool bUseLawnmowerPlanner = PlannerStrategyName.Equals(TEXT("LawnmowerV1"), ESearchCase::IgnoreCase);
    const int32 RunSeed = FireGeneratorSeed;
    const bool bRecordIdealSensorWitnesses = MetricsRecorder.IsIdealSensorWitnessRecordingEnabled();

    ParallelFor(NumSlots, [this, AtTick, bUseRandomPlanner, bUseFullyRandomPlanner, bUseLawnmowerPlanner, RunSeed, bRecordIdealSensorWitnesses](int32 I)
    {
        if (ADrone* D = ParallelDroneScratchDrones[I])
        {
            D->TickDroneParallelCpu(
                AtTick,
                ParallelDroneScratchLocations[I],
                bUseRandomPlanner,
                bUseFullyRandomPlanner,
                bUseLawnmowerPlanner,
                RunSeed,
                bRecordWallClockTiming,
                bRecordIdealSensorWitnesses,
                ParallelDroneScratchResults[I]);
        }
    });

    TArray<FDroneTickProfile> Profiles;
    if (bRecordWallClockTiming)
    {
        Profiles.Reserve(NumSlots);
    }

    for (int32 I = 0; I < NumSlots; ++I)
    {
        if (ADrone* D = ParallelDroneScratchDrones[I])
        {
            D->ApplyParallelCpuTickResult(ParallelDroneScratchResults[I]);
            if (bRecordWallClockTiming)
            {
                Profiles.Add(ParallelDroneScratchResults[I].Profile);
            }
            for (const FIdealSensorWitnessEvent& Event : ParallelDroneScratchResults[I].IdealSensorWitnessEvents)
            {
                MetricsRecorder.RecordIdealSensorWitness(Event.SensingTick, Event.SnapshotTick,
                    Event.DroneIndex, Event.PersonId, Event.CellIndex);
            }
            D->AppendPrivateObservationsTo(PendingObservations);
        }
    }

    RecordDroneTickProfileTimings(AtTick, TEXT("drone.parallel"), Profiles);
}

void ASimulationManager::RecordWallClockPhaseTiming(int32 Tick, const TCHAR* PhaseName, double StartSeconds)
{
    if (!bRecordWallClockTiming || !PhaseName)
    {
        return;
    }

    const float ElapsedMs = static_cast<float>((FPlatformTime::Seconds() - StartSeconds) * 1000.0);
    if (MetricsRecorder.IsActive())
    {
        MetricsRecorder.RecordPhaseTiming(Tick, FString(PhaseName), ElapsedMs);
        return;
    }

    FPendingWallClockPhaseTiming& Pending = PendingWallClockPhaseTimings.AddDefaulted_GetRef();
    Pending.Tick = Tick;
    Pending.PhaseName = PhaseName;
    Pending.ElapsedMs = FMath::Max(0.0f, ElapsedMs);
}

void ASimulationManager::FlushPendingWallClockPhaseTimings()
{
    if (!MetricsRecorder.IsActive() || PendingWallClockPhaseTimings.Num() == 0)
    {
        return;
    }

    for (const FPendingWallClockPhaseTiming& Pending : PendingWallClockPhaseTimings)
    {
        MetricsRecorder.RecordPhaseTiming(Pending.Tick, Pending.PhaseName, Pending.ElapsedMs);
    }
    PendingWallClockPhaseTimings.Reset();
}

void ASimulationManager::RecordDroneTickProfileTimings(int32 Tick, const TCHAR* Prefix, const TArray<FDroneTickProfile>& Profiles)
{
    if (!bRecordWallClockTiming || !Prefix || !MetricsRecorder.IsActive() || Profiles.Num() == 0)
    {
        return;
    }

    struct FPhaseAggregate
    {
        float SumMs = 0.0f;
        float MaxMs = 0.0f;

        void Add(float Value)
        {
            const float SafeValue = FMath::Max(0.0f, Value);
            SumMs += SafeValue;
            MaxMs = FMath::Max(MaxMs, SafeValue);
        }
    };

    FPhaseAggregate Validate;
    FPhaseAggregate Sense;
    FPhaseAggregate LocalBelief;
    FPhaseAggregate ReplanDecision;
    FPhaseAggregate Replan;
    FPhaseAggregate Move;
    FPhaseAggregate WorkerTotal;

    int32 ValidCount = 0;
    for (const FDroneTickProfile& Profile : Profiles)
    {
        if (!Profile.bValid)
        {
            continue;
        }

        ++ValidCount;
        Validate.Add(Profile.ValidateMs);
        Sense.Add(Profile.SenseMs);
        LocalBelief.Add(Profile.LocalBeliefMs);
        ReplanDecision.Add(Profile.ReplanDecisionMs);
        Replan.Add(Profile.ReplanMs);
        Move.Add(Profile.MoveMs);
        WorkerTotal.Add(Profile.TotalMs);
    }

    if (ValidCount == 0)
    {
        return;
    }

    const FString PrefixString(Prefix);
    auto RecordAggregate = [this, Tick, &PrefixString](const TCHAR* Name, const FPhaseAggregate& Aggregate)
    {
        MetricsRecorder.RecordPhaseTiming(Tick, PrefixString + TEXT(".") + Name + TEXT("_sum"), Aggregate.SumMs);
        MetricsRecorder.RecordPhaseTiming(Tick, PrefixString + TEXT(".") + Name + TEXT("_max"), Aggregate.MaxMs);
    };

    RecordAggregate(TEXT("validate"), Validate);
    RecordAggregate(TEXT("sense"), Sense);
    RecordAggregate(TEXT("local_belief"), LocalBelief);
    RecordAggregate(TEXT("replan_decision"), ReplanDecision);
    RecordAggregate(TEXT("replan"), Replan);
    RecordAggregate(TEXT("move"), Move);
    RecordAggregate(TEXT("worker_total"), WorkerTotal);
}

void ASimulationManager::UpdateLawnmowerV2Progress()
{
    if (!PlannerStrategyName.Equals(TEXT("LawnmowerV2"), ESearchCase::IgnoreCase)) return;
    const int32 Total = GridWidth * GridHeight;
    if (GridWidth <= 0 || GridHeight <= 0) return;
    const int32 Stride = ULawnmowerV2Planner::ComputeLaneStride(
        DroneMaxViewDistanceUU / FMath::Max(1.0f, CellSizeUU), DroneHorizontalFOVDegrees, GridWidth, GridHeight);
    const auto IsLane = [this, Stride](int32 Cell)
    {
        const int32 Row = Cell / GridWidth;
        return Row % Stride == 0 || Row == GridHeight - 1;
    };
    if (CurrentTick == 1 || LawnmowerV2CompletedCells.Num() != Total)
    {
        LawnmowerV2CompletedCells.Init(0, Total);
        LawnmowerV2CompletedLaneCells = 0;
        LawnmowerV2CoverageEpoch = 0;
        for (const auto& Drone : RegisteredDrones)
            if (ADrone* D = Drone.Get()) D->ResetLawnmowerV2State();
    }
    // Stable drone-index order. No workers are active in this server slot.
    for (const auto& Drone : RegisteredDrones)
    {
        if (ADrone* D = Drone.Get())
        {
            TArray<int32> Completed;
            D->DrainLawnmowerV2CompletedCells(Completed);
            for (const int32 Cell : Completed)
            {
                if (LawnmowerV2CompletedCells.IsValidIndex(Cell) && !LawnmowerV2CompletedCells[Cell])
                {
                    LawnmowerV2CompletedCells[Cell] = 1;
                    if (IsLane(Cell)) ++LawnmowerV2CompletedLaneCells;
                }
            }
        }
    }
    const int32 LaneRows = (GridHeight - 1) / Stride + 1 + ((GridHeight - 1) % Stride != 0 ? 1 : 0);
    if (LawnmowerV2CompletedLaneCells >= LaneRows * GridWidth)
    {
        LawnmowerV2CompletedCells.Init(0, Total);
        LawnmowerV2CompletedLaneCells = 0;
        ++LawnmowerV2CoverageEpoch;
        for (const auto& Drone : RegisteredDrones)
            if (ADrone* D = Drone.Get()) D->ResetLawnmowerV2ProgressEpoch();
        MetricsRecorder.RecordEvent(CurrentTick, TEXT("lawnmower_v2_epoch"), FString::FromInt(LawnmowerV2CoverageEpoch));
    }
    for (const auto& Drone : RegisteredDrones)
        if (ADrone* D = Drone.Get()) D->ConfigureLawnmowerV2Progress(&LawnmowerV2CompletedCells);
}

void ASimulationManager::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    // A runtime switch to sensor captures must restore the calibrated renderer
    // before any capture is submitted in this tick.
    LiveRenderPolicy::Update();

    // Measured sim rate for the debug HUD (wall-clock window of 1s).
    TickRateWindowSeconds += DeltaSeconds;
    if (TickRateWindowSeconds >= 1.0f)
    {
        MeasuredTicksPerSecond = TicksThisSecond / TickRateWindowSeconds;
        TicksThisSecond = 0;
        TickRateWindowSeconds = 0.0f;
    }

    if (State == ESimState::Running)
    {
        TickAccumulator += DeltaSeconds;
        const float StepDuration = (MaxTicksPerSecond > 0.0f) ? (1.0f / MaxTicksPerSecond) : 0.0f;

        if (StepDuration <= 0.0f || TickAccumulator >= StepDuration)
        {
            const double TickStartSeconds = FPlatformTime::Seconds();
            auto BeginTickPhase = [this]() -> double
            {
                return bRecordWallClockTiming ? FPlatformTime::Seconds() : 0.0;
            };
            auto EndTickPhase = [this](const TCHAR* PhaseName, double StartSeconds)
            {
                RecordWallClockPhaseTiming(CurrentTick, PhaseName, StartSeconds);
            };
            if (StepDuration > 0.0f)
            {
                TickAccumulator -= StepDuration;
            }

            ++CurrentTick;
            ++TicksThisSecond;
            UpdateLawnmowerV2Progress();
            {
                const double PhaseStart = BeginTickPhase();
                OnTickAdvanced.Broadcast(CurrentTick);
                EndTickPhase(TEXT("tick.on_tick_advanced"), PhaseStart);
            }

            if (UWorld* World = GetWorld())
            {
                if (UGpuBeliefManager* BeliefMgr = World->GetSubsystem<UGpuBeliefManager>())
                {
                    // ---- 1. Advance staleness (every cell +1) ----
                    // StalenessTexture is read only by the server priority field
                    // (debug overlay) and the grid-stats reduction. When neither is
                    // active it is never read, so skip the per-tick update. Belief
                    // evolution does not depend on it, and observation ingest still
                    // resets observed-cell staleness on its own.
                    if (DebugPrioritySource == EDebugPrioritySource::ServerPriority || GridStatsEveryNTicks > 0)
                    {
                        const double PhaseStart = BeginTickPhase();
                        BeliefMgr->DispatchStalenessUpdate(CurrentTick);
                        EndTickPhase(TEXT("gpu.staleness_update_dispatch"), PhaseStart);
                    }

                    // ---- 2. Ingest observations from the previous tick ----
                    // This overwrites staleness = 0 for observed cells and adds LLRs.
                    if (PendingObservations.Num() > 0)
                    {
                        {
                            const double PhaseStart = BeginTickPhase();
                            BeliefMgr->DispatchObservationIngest(PendingObservations);
                            EndTickPhase(TEXT("gpu.observation_ingest_dispatch"), PhaseStart);
                        }

                        // Mirror the same updates into WorldGrid's CPU belief. WorldGrid
                        // is the server-side CPU aggregate that drones sync from when
                        // they get newly-owned cells via SetOwnedCells — so a cell
                        // already observed by a previous owner is not re-treated as
                        // never-seen when the new owner takes it.
                        if (WorldGrid.IsValid())
                        {
                            const double PhaseStart = BeginTickPhase();
                            for (const FObservation& O : PendingObservations)
                            {
                                const int32 X = O.CellIndex % GridWidth;
                                const int32 Y = O.CellIndex / GridWidth;
                                if (X < 0 || X >= GridWidth || Y < 0 || Y >= GridHeight) continue;
                                constexpr float MinLogOdds = -8.0f;
                                constexpr float MaxLogOdds =  8.0f;
                                WorldGrid->setFireBelief(
                                    X, Y,
                                    FMath::Clamp(WorldGrid->getFireBelief(X, Y) + O.FireLogOddsUpdate,
                                                 MinLogOdds,
                                                 MaxLogOdds));
                                WorldGrid->setPersonBelief(
                                    X, Y,
                                    FMath::Clamp(WorldGrid->getPersonBelief(X, Y) + O.PersonLogOddsUpdate,
                                                 MinLogOdds,
                                                 MaxLogOdds));
                                WorldGrid->setLastObservedTick(X, Y, O.Tick);
                                WorldGrid->RecordPersonObservation(O);
                                if (WorldGrid->getFireBelief(X, Y) > 0.0f &&
                                    WorldGrid->getFirstFireDetectionTick(X, Y) < 0)
                                    WorldGrid->setFirstFireDetectionTick(X, Y, O.Tick);
                                if (WorldGrid->getPersonBelief(X, Y) > 0.0f &&
                                    WorldGrid->getFirstPersonDetectionTick(X, Y) < 0)
                                    WorldGrid->setFirstPersonDetectionTick(X, Y, O.Tick);
                            }
                            EndTickPhase(TEXT("server.cpu_observation_mirror"), PhaseStart);
                        }

                        PendingObservations.Reset();
                    }

                    // ---- 3. Spread fire belief (uses ReadIndex, which now includes ingested data) ----
                    {
                        const double PhaseStart = BeginTickPhase();
                        BeliefMgr->DispatchSpread(CurrentTick);
                        EndTickPhase(TEXT("gpu.spread_dispatch"), PhaseStart);
                    }

                    // ---- 4. Priority (server field) ----
                    // The server PriorityTexture is consumed only by the debug
                    // overlay (see the bNeedDebugPriority-gated readback below).
                    // Drones plan from their own local priority and the partition
                    // reads drone belief, so in benchmark/headless runs this
                    // dispatch writes a texture nobody reads. Gate it on the same
                    // condition as its only consumer. DispatchPriority only reads
                    // belief/staleness and writes Priority, so skipping it has no
                    // effect on belief evolution or any recorded metric.
                    if (DebugPrioritySource == EDebugPrioritySource::ServerPriority)
                    {
                        const double PhaseStart = BeginTickPhase();
                        BeliefMgr->DispatchPriority(CurrentTick, bEnableVisionBridge);
                        EndTickPhase(TEXT("gpu.priority_dispatch"), PhaseStart);
                    }

                    // ---- 5. Swap ping-pong for next tick ----
                    {
                        const double PhaseStart = BeginTickPhase();
                        BeliefMgr->SwapPingPong();
                        EndTickPhase(TEXT("gpu.swap_ping_pong"), PhaseStart);
                    }

                    // ---- Adaptive CPU partition ----
                    const bool bShouldRecomputePartition =
                        !IsStaticPartitionRegime() &&
                        PartitionRecomputeEveryNTicks > 0 &&
                        (CurrentTick % PartitionRecomputeEveryNTicks) == 0 &&
                        RegisteredDrones.Num() > 0;

                    if (bShouldRecomputePartition)
                    {
                        const double PhaseStart = BeginTickPhase();
                        TArray<FVector2f> DronePosCellSpace;
                        TArray<TObjectPtr<ADrone>> ActiveDrones;
                        DronePosCellSpace.Reserve(RegisteredDrones.Num());
                        ActiveDrones.Reserve(RegisteredDrones.Num());
                        for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
                        {
                            if (ADrone* D = Drone.Get())
                            {
                                const FVector W = D->GetActorLocation();
                                DronePosCellSpace.Emplace(
                                    static_cast<float>((W.X - GridOriginWorld.X) / CellSizeUU),
                                    static_cast<float>((W.Y - GridOriginWorld.Y) / CellSizeUU));
                                ActiveDrones.Add(Drone);
                            }
                        }

                        TArray<uint16> Owners = ComputeBalancedPartition(ActiveDrones, DronePosCellSpace, CurrentTick);
                        ApplyPartitionOwnersToGrid(Owners);

                        if (MetricsRecorder.IsActive())
                        {
                            TArray<int32> OwnedCounts;
                            OwnedCounts.Reserve(RegisteredDrones.Num());
                            for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
                            {
                                const ADrone* D = Drone.Get();
                                OwnedCounts.Add(D ? D->GetOwnedCellCount() : 0);
                            }
                            MetricsRecorder.RecordPartitionPass(CurrentTick, OwnedCounts);
                        }
                        EndTickPhase(TEXT("partition.compute_apply"), PhaseStart);
                    }

                    // ---- Grid stats (GPU reduction, deterministic cadence) ----
                    // Dispatch at fixed ticks; consume (fence-blocking) exactly
                    // GridStatsReadbackLagTicks later so the metrics CSV rows are
                    // identical across same-seed runs.
                    if (MetricsRecorder.IsActive() && GridStatsEveryNTicks > 0)
                    {
                        const int32 PendingTick = BeliefMgr->GetPendingGridStatsTick();
                        if (PendingTick >= 0 && CurrentTick >= PendingTick + GridStatsReadbackLagTicks)
                        {
                            const double PhaseStart = BeginTickPhase();
                            float MeanStaleness = 0.0f, MaxStaleness = 0.0f, Coverage = 0.0f;
                            int32 SourceTick = -1;
                            if (BeliefMgr->ConsumeGridStats(MeanStaleness, MaxStaleness, Coverage, SourceTick))
                            {
                                MetricsRecorder.RecordGridStats(SourceTick, MeanStaleness, MaxStaleness, Coverage);
                            }
                            EndTickPhase(TEXT("grid_stats.consume_readback"), PhaseStart);
                        }

                        if ((CurrentTick % GridStatsEveryNTicks) == 0 &&
                            BeliefMgr->GetPendingGridStatsTick() < 0)
                        {
                            const double PhaseStart = BeginTickPhase();
                            BeliefMgr->DispatchGridStats(CurrentTick);
                            EndTickPhase(TEXT("grid_stats.dispatch"), PhaseStart);
                        }
                    }

                    // Server priority readback is debug-only. Drone planning deliberately
                    // does not consume this cache; each drone computes priority from its own
                    // local belief state so future server-failure experiments remain valid.
                    const bool bNeedDebugPriority =
                        (DebugPrioritySource == EDebugPrioritySource::ServerPriority);

                    if (bNeedDebugPriority)
                    {
                        const double PhaseStart = BeginTickPhase();
                        ++TicksSinceLastPriorityReadback;

                        if (TicksSinceLastPriorityReadback >= FMath::Max(1, DebugPriorityReadbackInterval))
                        {
                            BeliefMgr->RequestPriorityReadback(CurrentTick);
                            TicksSinceLastPriorityReadback = 0;
                        }

                        TArray<float> PrioData;
                        int32 PrioTick = -1;
                        if (BeliefMgr->TryGetPriorityReadback(PrioData, PrioTick))
                        {
                            CachedServerPriority = MoveTemp(PrioData);
                            CachedServerPriorityTick = PrioTick;
                        }
                        EndTickPhase(TEXT("debug.priority_readback"), PhaseStart);
                    }

                    // Fixed sparse checkpoint: full texture readbacks are
                    // opt-in and block this tick at a documented boundary.
                    // Missing priority/staleness passes remain unavailable in
                    // metadata; the audit never forces extra compute dispatches.
                    const bool bCameraAuditCheckpoint = CurrentTick == 100 || CurrentTick == 500 ||
                        CurrentTick == 2000 || CurrentTick == 8000 ||
                        (MaxSimTicks > 0 && CurrentTick >= MaxSimTicks);
                    if (bCameraAuditCheckpoint && !CameraStateAuditDirectory.IsEmpty())
                    {
                        const double PhaseStart = BeginTickPhase();
                        RecordGpuStateAudit(BeliefMgr, CurrentTick);
                        EndTickPhase(TEXT("audit.gpu_state_snapshot"), PhaseStart);
                    }
                }
            }

            // Vision step (server slot): captures taken at the END of earlier
            // ticks are turned into observations now — fixed K>=1 tick lag,
            // deterministic cadence. Replay cache mode never touches sockets
            // or pixel readbacks.
            {
                const double PhaseStart = BeginTickPhase();
                if (!ProcessVisionCaptures())
                {
                    // Failed perception is not a negative observation. Keep an
                    // interactive run paused; batch runs fail rather than emit
                    // a valid-looking completed experiment.
                    State = ESimState::Paused;
                    // Windows' graceful RequestExitWithStatus only posts WM_QUIT;
                    // GuardedMain can still return success. A failed batch must
                    // terminate nonzero and must not finalize a successful run.
                    // The platform implementation flushes the log before exit.
                    if (bExitOnMaxTicks) FPlatformMisc::RequestExitWithStatus(true, 2);
                    return;
                }
                EndTickPhase(TEXT("vision.process_captures"), PhaseStart);
            }

            // Snapshot every oracle drone before any one of them moves.  This
            // makes the global assignment independent of sequential tick order.
            if (IsOracleInformationEnabled())
            {
                const double PhaseStart = BeginTickPhase();
                UpdateOracleAssignments(CurrentTick);
                EndTickPhase(TEXT("oracle.global_assignment"), PhaseStart);
            }

            // Drone update + collect observations. The optional parallel path
            // keeps server ingestion in drone-index order after the barrier.
            if (CanUseParallelDroneProcessing())
            {
                const double PhaseStart = BeginTickPhase();
                TickDronesParallelCpu(CurrentTick);
                EndTickPhase(TEXT("drones.parallel_cpu"), PhaseStart);
            }
            else
            {
                const double PhaseStart = BeginTickPhase();
                TickDronesSequential(CurrentTick);
                EndTickPhase(TEXT("drones.sequential"), PhaseStart);
            }

            // Fire truth/scorch is cheap and deterministic; expensive VFX can
            // stay demand-loaded until a YOLO capture actually renders.
            if (FireVisualsActor && bFireVisualsOnlyForYoloCaptures)
            {
                const double PhaseStart = BeginTickPhase();
                FireVisualsActor->AdvanceFireState(CurrentTick);
                EndTickPhase(TEXT("fire_visuals.advance_state"), PhaseStart);
            }

            // Drone captures (Phase C): explicit renders at a fixed, staggered
            // cadence. Poses are this tick's (movement already applied above).
            if (bEnableDroneCaptures && DroneCaptureEveryNTicks > 0)
            {
                const double PhaseStart = BeginTickPhase();
                for (TObjectPtr<ADrone>& Drone : RegisteredDrones)
                {
                    if (ADrone* D = Drone.Get())
                    {
                        if (!IsDroneVisionCameraEnabled(D->GetDroneIndex()) || !D->HasActiveCapture())
                        {
                            continue;
                        }
                        if (((CurrentTick + D->GetDroneIndex()) % DroneCaptureEveryNTicks) == 0)
                        {
                            if (UDroneCaptureComponent* Cap = D->GetCaptureComponent())
                            {
                                FTransform OriginalCaptureTransform = FTransform::Identity;
                                float OriginalCaptureFov = Cap->FOVAngle;
                                const bool bScriptedDatasetView = bEnableYoloDatasetExport &&
                                    SetupScriptedDatasetView(Cap, CurrentTick, D->GetDroneIndex(),
                                                             OriginalCaptureTransform, OriginalCaptureFov);
                                const bool bRenderCapture = !bEnableVisionBridge || VisionCacheMode != EVisionCacheMode::Replay;
                                if (bRenderCapture && HumanManager && HumanManager->bDeferVisualUpdatesUntilCapture)
                                {
                                    const double HumanVisualStart = BeginTickPhase();
                                    HumanManager->UpdateVisualPoseForCapture();
                                    EndTickPhase(TEXT("camera.prepare_humans"), HumanVisualStart);
                                }
                                if (bRenderCapture && FireVisualsActor && bFireVisualsOnlyForYoloCaptures)
                                {
                                    const double PrepareStart = BeginTickPhase();
                                    FireVisualsActor->PrepareForYoloCapture(
                                        CurrentTick,
                                        Cap->GetComponentLocation(),
                                        FireVisualsYoloCaptureRadiusUU);
                                    EndTickPhase(TEXT("camera.prepare_fire_smoke"), PrepareStart);
                                }
                                if (bRenderCapture && !bSensorTexturesPrepared)
                                {
                                    const double ResidencyStart = BeginTickPhase();
                                    FlushAsyncLoading();
                                    FAssetCompilingManager::Get().FinishAllCompilation();
                                    if (GShaderCompilingManager) GShaderCompilingManager->FinishAllCompilation();
                                    // Pin only textures used by this world. No timed residency
                                    // expiry: disk speed must not choose a sensor's mip level.
                                    for (TObjectIterator<UPrimitiveComponent> It; It; ++It)
                                    {
                                        if (It->GetWorld() != GetWorld() || !It->IsRegistered()) continue;
                                        TArray<UMaterialInterface*> Materials;
                                        It->GetUsedMaterials(Materials);
                                        for (UMaterialInterface* Material : Materials)
                                        {
                                            if (!Material) continue;
                                            TArray<UTexture*> Textures;
                                            Material->GetUsedTextures(Textures);
                                            for (UTexture* Used : Textures)
                                            {
                                                if (UTexture2D* Texture = Cast<UTexture2D>(Used))
                                                {
                                                    if (!SensorTextureResidency.Contains(Texture))
                                                        SensorTextureResidency.Add(Texture, Texture->bForceMiplevelsToBeResident);
                                                    Texture->bForceMiplevelsToBeResident = true;
                                                }
                                            }
                                        }
                                    }
                                    IStreamingManager::Get().UpdateResourceStreaming(0.0f, true);
                                    IStreamingManager::Get().BlockTillAllRequestsFinished(0.0f, true);
                                    FlushRenderingCommands();
                                    for (const auto& Entry : SensorTextureResidency)
                                    {
                                        if (UTexture2D* Texture = Entry.Key.Get())
                                        {
                                            // First-tick assets may finish initialization after
                                            // the global streamer pass. This waits for init,
                                            // submits this asset's forced-residency request,
                                            // then waits for that request before validation.
                                            Texture->WaitForStreaming();
                                            if (!Texture->IsFullyStreamedIn())
                                            {
                                                UE_LOG(LogTemp, Fatal, TEXT("Sensor texture residency incomplete: %s"), *Texture->GetPathName());
                                            }
                                        }
                                    }
                                    UE_LOG(LogTemp, Display, TEXT("Sensor textures ready: %d resident textures"), SensorTextureResidency.Num());
                                    bSensorTexturesPrepared = true;
                                    EndTickPhase(TEXT("camera.startup_texture_residency"), ResidencyStart);
                                }
                                if (bRenderCapture && !bSensorSkyPrepared)
                                {
                                    const double SkyStart = BeginTickPhase();
                                    // A skylight may otherwise capture temporary shaders or
                                    // meshes and recapture later on a wall-clock schedule.
                                    // Finish scene preparation before its one authoritative
                                    // ambient capture, without advancing simulation ticks.
                                    FlushAsyncLoading();
                                    FAssetCompilingManager::Get().FinishAllCompilation();
                                    if (GShaderCompilingManager)
                                    {
                                        GShaderCompilingManager->FinishAllCompilation();
                                    }
                                    // Realtime skylight capture depends on overview rendering.
                                    // Capture the static scenario sky once so offscreen sensor
                                    // views have the same ambient illumination as visible runs.
                                    NormalizeEnvironmentLighting();
                                    TArray<USkyLightComponent*> SkyComponents;
                                    // SunSky and other Blueprint actors embed skylights;
                                    // searching only ASkyLight actors misses those lights.
                                    for (TObjectIterator<USkyLightComponent> It; It; ++It)
                                    {
                                        if (It->GetWorld() == GetWorld() && It->IsRegistered() && It->IsVisible())
                                        {
                                            SkyComponents.Add(*It);
                                        }
                                    }
                                    SkyComponents.Sort([](const USkyLightComponent& A, const USkyLightComponent& B)
                                    {
                                        return A.GetPathName() < B.GetPathName();
                                    });
                                    for (USkyLightComponent* Sky : SkyComponents)
                                    {
                                        SensorOriginalRealtimeSky.Add(Sky, Sky->IsRealTimeCaptureEnabled());
                                        Sky->SetRealTimeCaptureEnabled(false);
                                        if (CameraRenderProfile == TEXT("fast_shader_v3"))
                                        {
                                            // Scene skylight recapture retains renderer-dependent
                                            // differences after OpenLevel. This versioned profile
                                            // instead uses one immutable daylight environment.
                                            UTextureCube* Daylight = LoadObject<UTextureCube>(nullptr,
                                                TEXT("/Engine/MapTemplates/Sky/DaylightAmbientCubemap.DaylightAmbientCubemap"));
                                            if (!Daylight)
                                                UE_LOG(LogTemp, Fatal, TEXT("fast_shader_v3 daylight cubemap is unavailable."));
                                            SensorOriginalSkySource.Add(Sky, static_cast<int32>(Sky->SourceType.GetValue()));
                                            SensorOriginalSkyCubemap.Add(Sky, Sky->Cubemap.Get());
                                            Sky->SourceType = SLS_SpecifiedCubemap;
                                            Sky->SetCubemap(Daylight);
                                        }
                                        Sky->RecaptureSky();
                                    }
                                    GetWorld()->SendAllEndOfFrameUpdates();
                                    {
                                        const auto SkyCaptureTime = FSceneViewExtensions::NewExtension<FSensorSkyCaptureTimeExtension>(
                                            GetWorld(), CurrentTick, SecondsPerTick);
                                        USkyLightComponent::UpdateSkyCaptureContents(GetWorld());
                                        FlushRenderingCommands();
                                    }
                                    // Recapture marks the light's proxy dirty. Publish that
                                    // new irradiance before the first sensor render as well.
                                    GetWorld()->SendAllEndOfFrameUpdates();
                                    FlushRenderingCommands();
                                    if (CameraRenderProfile == TEXT("fast_shader_v3"))
                                    {
                                        // A specified cubemap does not render six scene views
                                        // to incidentally finish PSO precaching. Wait explicitly
                                        // so initial sensor pixels cannot use delayed proxies.
                                        ENQUEUE_RENDER_COMMAND(ThesisCaptureWaitForPipelines)(
                                            [](FRHICommandListImmediate&) { PipelineStateCache::WaitForAllTasks(); });
                                        FlushRenderingCommands();
                                        GetWorld()->SendAllEndOfFrameUpdates();
                                        FlushRenderingCommands();
                                    }
                                    for (USkyLightComponent* Sky : SkyComponents)
                                    {
                                        const FSHVectorRGB3 Irradiance = Sky->GetIrradianceEnvironmentMap();
                                        UE_LOG(LogTemp, Display, TEXT("Sensor sky %s: mobility=%d position=%s intensity=%g source=%d texture=%d irradiance0=(%g,%g,%g)"),
                                            *Sky->GetPathName(), static_cast<int32>(Sky->Mobility.GetValue()),
                                            *Sky->GetComponentLocation().ToString(), Sky->Intensity,
                                            static_cast<int32>(Sky->SourceType.GetValue()), Sky->GetProcessedSkyTexture() != nullptr,
                                            Irradiance.R.V[0], Irradiance.G.V[0], Irradiance.B.V[0]);
                                    }
                                    if (FParse::Param(FCommandLine::Get(), TEXT("ThesisSensorDiagnosticSkySH")))
                                    {
                                        LogSensorSkySHBits(SkyComponents, CurrentTick);
                                    }
                                    bSensorSkyPrepared = true;
                                    EndTickPhase(TEXT("camera.startup_sky"), SkyStart);
                                }
                                const double CaptureStart = BeginTickPhase();
                                Cap->CaptureTick(CurrentTick, bRenderCapture, SecondsPerTick);
                                RecordCameraCaptureAudit(D, Cap, CurrentTick, bRenderCapture);
                                EndTickPhase(TEXT("camera.submit_capture"), CaptureStart);
                                const double DatasetStart = BeginTickPhase();
                                const bool bExportedDatasetCapture = !bEnableYoloDatasetExport || ExportDatasetCapture(Cap);
                                if (bEnableYoloDatasetExport)
                                    EndTickPhase(TEXT("camera.dataset_export"), DatasetStart);
                                if (bRenderCapture && IsCameraStateAuditEnabled() && !bEnableVisionBridge)
                                {
                                    const TArray<FColor>* AuditPixels = nullptr;
                                    int32 AuditWidth = 0, AuditHeight = 0, AuditTick = INDEX_NONE;
                                    if (Cap->ReadLastCaptureView(AuditPixels, AuditWidth, AuditHeight, AuditTick) && AuditTick == CurrentTick)
                                    {
                                        RecordCameraConsumeAudit(D, Cap, AuditTick, CurrentTick, false,
                                            AuditPixels, AuditWidth, AuditHeight);
                                    }
                                }
                                if (bScriptedDatasetView)
                                {
                                    Cap->SetWorldTransform(OriginalCaptureTransform);
                                    Cap->FOVAngle = OriginalCaptureFov;
                                }
                                if (!bExportedDatasetCapture) return;
                                if (bRenderCapture && FireVisualsActor && bFireVisualsOnlyForYoloCaptures)
                                {
                                    const double CleanupStart = BeginTickPhase();
                                    FireVisualsActor->EndYoloCapture();
                                    EndTickPhase(TEXT("camera.hide_fire_smoke"), CleanupStart);
                                }
                            }
                        }
                    }
                }
                EndTickPhase(TEXT("drone_captures.capture_tick"), PhaseStart);
            }

            // Human update (tick anatomy step 4): replayed ground truth.
            if (HumanManager)
            {
                const double PhaseStart = BeginTickPhase();
                HumanManager->ApplyTick(CurrentTick, WorldGrid.Get());
                EndTickPhase(TEXT("humans.apply_tick"), PhaseStart);
            }

            // Fire front visuals (flames/smoke/lights follow the replay).
            if (FireVisualsActor && !bFireVisualsOnlyForYoloCaptures)
            {
                const double PhaseStart = BeginTickPhase();
                FireVisualsActor->ApplyTick(CurrentTick);
                EndTickPhase(TEXT("fire_visuals.apply_tick"), PhaseStart);
            }
            if (EditorFirePreviewActor && IsFireEditorPreviewEnabled())
            {
                const double PhaseStart = BeginTickPhase();
                CSV_SCOPED_TIMING_STAT_EXCLUSIVE(FireVisualsPreviewApplyTick);
                EditorFirePreviewActor->ApplyTick(CurrentTick);
                EndTickPhase(TEXT("fire_visuals.preview_apply_tick"), PhaseStart);
            }

            // Primary metric: first detection of each person by server belief.
            {
                const double PhaseStart = BeginTickPhase();
                UpdatePersonDetectionMetrics();
                EndTickPhase(TEXT("metrics.update_person_detection"), PhaseStart);
            }

            // Metrics (read-only: never influences the simulation).
            const float TickComputeMs = static_cast<float>((FPlatformTime::Seconds() - TickStartSeconds) * 1000.0);
            {
                const double PhaseStart = BeginTickPhase();
                RecordTickMetrics(bRecordWallClockTiming ? TickComputeMs : 0.0f);
                EndTickPhase(TEXT("metrics.record_tick"), PhaseStart);
            }
            EndTickPhase(TEXT("tick.total"), TickStartSeconds);

            // Benchmark stop condition.
            if (MaxSimTicks > 0 && CurrentTick >= MaxSimTicks)
            {
                if (bEnableYoloDatasetExport && !CompleteDatasetExport()) return;
                UE_LOG(LogTemp, Log, TEXT("Thesis: reached MaxSimTicks=%d — stopping."), MaxSimTicks);
                MetricsRecorder.RecordBeliefError(ComputeMeanBeliefError());
                if (bEnableCachedDetectionChanceNull && !WriteCachedDetectionChanceNull())
                {
                    State = ESimState::Paused;
                    if (bExitOnMaxTicks) FPlatformMisc::RequestExitWithStatus(true, 2);
                    return;
                }
                MetricsRecorder.EndRun(CurrentTick);
                State = ESimState::Stopped;
                // In-process batching: reload the level for the next scenario, which
                // tears down actors and re-runs BeginPlay. Only exit once the
                // batch is finished or its stop marker is present.
                bool bBatchContinued = false;
                if (UWorld* BatchWorld = GetWorld())
                {
                    if (UGameInstance* GI = BatchWorld->GetGameInstance())
                    {
                        if (UThesisBenchmarkSubsystem* Bench = GI->GetSubsystem<UThesisBenchmarkSubsystem>())
                        {
                            if (Bench->IsBatch() && Bench->AdvanceAndHasMore())
                            {
                                const FString LevelName = UGameplayStatics::GetCurrentLevelName(BatchWorld, true);
                                UE_LOG(LogTemp, Log, TEXT("Thesis batch: reloading level '%s' for scenario %d/%d."),
                                       *LevelName, Bench->CurrentIndex() + 1, Bench->NumScenarios());
                                UGameplayStatics::OpenLevel(BatchWorld, FName(*LevelName));
                                bBatchContinued = true;
                            }
                        }
                    }
                }
                if (!bBatchContinued && bExitOnMaxTicks)
                {
                    FPlatformMisc::RequestExit(/*Force=*/false);
                }
            }
        }
    }

    // Always draw debug visuals (no flicker)
    if (!bSuppressBenchmarkViewport)
    {
        DrawDebugVisuals();
    }
}

void ASimulationManager::DrawDebugVisuals() const
{
    UWorld* World = GetWorld();
    if (!World) return;

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            0, 0.0f, FColor::White,
            FString::Printf(TEXT("Tick: %d  |  State: %s  |  Drones: %d  |  Sim: %.1f ticks/s"),
                CurrentTick,
                (State == ESimState::Running ? TEXT("Running") :
                 State == ESimState::Paused  ? TEXT("Paused")  : TEXT("Stopped")),
                RegisteredDrones.Num(),
                MeasuredTicksPerSecond));

        for (int32 i = 0; i < RegisteredDrones.Num(); ++i)
        {
            const ADrone* D = RegisteredDrones[i].Get();
            if (!D) continue;

            const int32 Target = D->GetTargetCellIndex();
            const float  Score  = D->GetCurrentTargetScore();
            const FColor C      = ColorForDroneIndex(i);
            FString Info;

            if (Target == INDEX_NONE)
            {
                Info = FString::Printf(TEXT("D%d  target: --   score: --"), i);
            }
            else
            {
                const int32 TX = Target % GridWidth;
                const int32 TY = Target / GridWidth;
                Info = FString::Printf(TEXT("D%d  target: (%d,%d)  score: %.4f"), i, TX, TY, Score);
            }
            GEngine->AddOnScreenDebugMessage(i + 1, 0.0f, C, Info);
        }
    }

    // Grid bounds
    if (bDrawGridBounds && GridWidth > 0 && GridHeight > 0)
    {
        const float GW = GridWidth  * CellSizeUU;
        const float GH = GridHeight * CellSizeUU;
        const float Z0 = GridOriginWorld.Z;
        const float Zt = Z0 + GridDebugHeightUU;
        const FVector O  = GridOriginWorld;
        const FColor BC(100, 220, 100);
        const uint8  D  = 0;
        const float  T  = 8.0f;

        DrawDebugLine(World, O + FVector(0,  0,  Z0), O + FVector(GW, 0,  Z0), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(GW, 0,  Z0), O + FVector(GW, GH, Z0), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(GW, GH, Z0), O + FVector(0,  GH, Z0), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(0,  GH, Z0), O + FVector(0,  0,  Z0), BC, false, -1, D, T);

        DrawDebugLine(World, O + FVector(0,  0,  Z0), O + FVector(0,  0,  Zt), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(GW, 0,  Z0), O + FVector(GW, 0,  Zt), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(GW, GH, Z0), O + FVector(GW, GH, Zt), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(0,  GH, Z0), O + FVector(0,  GH, Zt), BC, false, -1, D, T);

        DrawDebugLine(World, O + FVector(0,  0,  Zt), O + FVector(GW, 0,  Zt), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(GW, 0,  Zt), O + FVector(GW, GH, Zt), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(GW, GH, Zt), O + FVector(0,  GH, Zt), BC, false, -1, D, T);
        DrawDebugLine(World, O + FVector(0,  GH, Zt), O + FVector(0,  0,  Zt), BC, false, -1, D, T);
    }

    // Drone spheres / targets
    if (bDrawDrones)
    {
        for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
        {
            if (const ADrone* D = Drone.Get())
            {
                const FVector Loc = D->GetActorLocation();
                const FColor C = ColorForDroneIndex(D->GetDroneIndex());
                DrawDebugSphere(World, Loc, DroneDebugRadiusUU, 12, C, false, -1.0f, 0, 4.0f);
                DrawDebugString(World, Loc + FVector(0, 0, DroneDebugRadiusUU + 50.0f),
                                FString::Printf(TEXT("D%d"), D->GetDroneIndex()),
                                nullptr, C, 0.0f, true, 1.2f);

                if (D->GetTargetCellIndex() != INDEX_NONE)
                {
                    const int32 TX = D->GetTargetCellIndex() % GridWidth;
                    const int32 TY = D->GetTargetCellIndex() / GridWidth;
                    const FVector TargetLoc(
                        GridOriginWorld.X + (TX + 0.5f) * CellSizeUU,
                        GridOriginWorld.Y + (TY + 0.5f) * CellSizeUU,
                        Loc.Z);
                    if (FVector::Dist2D(Loc, TargetLoc) > DroneDebugRadiusUU)
                    {
                        DrawDebugLine(World, Loc, TargetLoc, FColor::White, false, -1.0f, 0, 2.0f);
                        DrawDebugSphere(World, TargetLoc, 80.0f, 8, FColor::Silver, false, -1.0f, 0, 2.0f);
                    }
                }
            }
        }
    }

    // Cell ownership
    // Territory boundaries only: a line is drawn where the owner changes between
    // neighbouring cells (and on owned outer edges). On large grids this is
    // O(boundary length) lines instead of O(cells)*4 — the per-cell rectangle
    // version drew ~40k debug lines per frame on a 100x100 grid and tanked the
    // editor frame rate, which made the sim (1 tick per frame budget) look frozen.
    if (bDrawCellOwnership && WorldGrid.IsValid())
    {
        const float ZFloor = GridOriginWorld.Z + 5.0f;
        const uint8 D = 0;
        const float T = 8.0f;

        auto OwnerAt = [this](int32 CX, int32 CY) -> int32
        {
            if (CX < 0 || CY < 0 || CX >= GridWidth || CY >= GridHeight) return -1;
            const char C = WorldGrid->getOwner(CX, CY);
            return (C == 'N') ? -1 : static_cast<int32>(static_cast<uint8>(C));
        };

        for (int32 CY = 0; CY < GridHeight; ++CY)
        {
            for (int32 CX = 0; CX < GridWidth; ++CX)
            {
                const int32 Own = OwnerAt(CX, CY);
                if (Own < 0) continue;

                const FColor FC = ColorForDroneIndex(Own);
                const float X0 = GridOriginWorld.X + CX * CellSizeUU;
                const float Y0 = GridOriginWorld.Y + CY * CellSizeUU;
                const float X1 = X0 + CellSizeUU;
                const float Y1 = Y0 + CellSizeUU;

                // East edge (also drawn when the neighbour is the grid border).
                if (OwnerAt(CX + 1, CY) != Own)
                    DrawDebugLine(World, FVector(X1, Y0, ZFloor), FVector(X1, Y1, ZFloor), FC, false, -1, D, T);
                // West edge only on border/unowned (interior west edges are the
                // neighbour's east edge with a different owner — drawn by them too,
                // so both territory colours appear side by side).
                if (OwnerAt(CX - 1, CY) != Own)
                    DrawDebugLine(World, FVector(X0, Y0, ZFloor), FVector(X0, Y1, ZFloor), FC, false, -1, D, T);
                // South / north edges.
                if (OwnerAt(CX, CY + 1) != Own)
                    DrawDebugLine(World, FVector(X0, Y1, ZFloor), FVector(X1, Y1, ZFloor), FC, false, -1, D, T);
                if (OwnerAt(CX, CY - 1) != Own)
                    DrawDebugLine(World, FVector(X0, Y0, ZFloor), FVector(X1, Y0, ZFloor), FC, false, -1, D, T);
            }
        }
    }

    // Priority & fire heatmap
    DrawPriorityHeatmap();
}

float ASimulationManager::ComputeGroundTruthPriority(int32 X, int32 Y, int32 Tick) const
{
    if (!WorldGrid.IsValid()) return 0.0f;

    const int32 RealNumHumans = (WorldGrid->getRealPriorityAndNumHumans(X, Y) >> 16) & 0xFFFF;
    const bool bPersonHere = (RealNumHumans > 0);
    const int32 FirstFire = WorldGrid->getFirstFireTick(X, Y);
    const bool bFireHere = (FirstFire >= 0) && (Tick >= FirstFire);

    const float FireLogOdds   = bFireHere   ?  3.0f : -0.3f;
    const float PersonLogOdds = bPersonHere ?  2.5f : -0.2f;

    const float FireTerm = ThesisPriority::PositiveEvidence(FireLogOdds) * PriorityProbabilityScale;
    const float PersonTerm = ThesisPriority::PositiveEvidence(PersonLogOdds) * PriorityProbabilityScale;
    return PriorityWeightDanger * FireTerm + PriorityWeightPerson * PersonTerm;
}

int32 ASimulationManager::GetFirstFireTickByCellIndex(int32 CellIndex) const
{
    if (!WorldGrid.IsValid() || CellIndex < 0 || CellIndex >= GridWidth * GridHeight)
    {
        return -1;
    }
    return WorldGrid->getFirstFireTick(CellIndex % GridWidth, CellIndex / GridWidth);
}

int32 ASimulationManager::GetRealPersonCountByCellIndex(int32 CellIndex) const
{
    if (!WorldGrid.IsValid() || CellIndex < 0 || CellIndex >= GridWidth * GridHeight)
    {
        return 0;
    }
    return (WorldGrid->getRealPriorityAndNumHumans(CellIndex % GridWidth, CellIndex / GridWidth) >> 16) & 0xFFFF;
}

float ASimulationManager::ComputeOracleCellLoad(int32 CellIndex, int32 Tick, int32 LookaheadTicks) const
{
    if (!WorldGrid.IsValid() || CellIndex < 0 || CellIndex >= GridWidth * GridHeight)
    {
        return 0.0f;
    }

    const int32 FirstFire = GetFirstFireTickByCellIndex(CellIndex);
    float Load = 1.0f;
    if (FirstFire >= 0)
    {
        if (Tick >= FirstFire)
        {
            Load += LoadWeightDanger * PriorityProbabilityScale;
        }
        else if (LookaheadTicks > 0 && FirstFire <= Tick + LookaheadTicks)
        {
            const float Urgency = 1.0f - (static_cast<float>(FirstFire - Tick) / static_cast<float>(LookaheadTicks));
            Load += LoadWeightDanger * PriorityProbabilityScale * FMath::Clamp(Urgency, 0.0f, 1.0f);
        }
    }

    const int32 Persons = GetRealPersonCountByCellIndex(CellIndex);
    if (Persons > 0)
    {
        Load += LoadWeightPerson * PriorityProbabilityScale * static_cast<float>(Persons);
    }
    return Load;
}

bool ASimulationManager::FindOracleTargetForDrone(const ADrone& Drone, int32 AtTick, int32& OutCell, float& OutScore) const
{
    OutCell = INDEX_NONE;
    OutScore = -FLT_MAX;
    if (!IsOracleInformationEnabled() || OracleAssignmentTick != AtTick)
    {
        return false;
    }

    const int32 DroneIndex = Drone.GetDroneIndex();
    if (!OracleAssignedCellByDrone.IsValidIndex(DroneIndex) ||
        OracleAssignedCellByDrone[DroneIndex] == INDEX_NONE)
    {
        return false;
    }

    OutCell = OracleAssignedCellByDrone[DroneIndex];
    OutScore = OracleAssignedScoreByDrone.IsValidIndex(DroneIndex)
        ? OracleAssignedScoreByDrone[DroneIndex]
        : 0.0f;
    return true;
}

void ASimulationManager::RebuildOracleTrajectoryCache()
{
    OracleTrajectoryByPerson.Reset();
    OracleCachedKeyframeCount = 0;
    if (!HumanManager)
    {
        return;
    }

    const FHumanTrajectoryImporter::FResult& Data = HumanManager->GetTrajectoryDataForOracle();
    OracleTrajectoryByPerson.SetNum(Data.NumPersons);
    for (const FHumanKeyframe& Keyframe : Data.Keyframes)
    {
        if (OracleTrajectoryByPerson.IsValidIndex(Keyframe.PersonId))
        {
            OracleTrajectoryByPerson[Keyframe.PersonId].Add(Keyframe);
        }
    }
    OracleCachedKeyframeCount = Data.Keyframes.Num();
}

void ASimulationManager::UpdateOracleAssignments(int32 AtTick)
{
    if (!IsOracleInformationEnabled() || !WorldGrid.IsValid())
    {
        return;
    }
    if (OracleAssignmentTick == AtTick)
    {
        return;
    }

    const FHumanTrajectoryImporter::FResult* TrajectoryData =
        HumanManager ? &HumanManager->GetTrajectoryDataForOracle() : nullptr;
    const int32 KeyframeCount = TrajectoryData ? TrajectoryData->Keyframes.Num() : 0;
    if (AtTick == 0 || OracleCachedKeyframeCount != KeyframeCount)
    {
        RebuildOracleTrajectoryCache();
        OracleFirstFireCell = INDEX_NONE;
        OracleFirstFireTick = INDEX_NONE;
    }

    TArray<ADrone*> Drones;
    for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
    {
        if (ADrone* D = Drone.Get())
        {
            Drones.Add(D);
        }
    }
    Drones.Sort([](const ADrone& A, const ADrone& B)
    {
        return A.GetDroneIndex() < B.GetDroneIndex();
    });

    int32 MaxDroneIndex = INDEX_NONE;
    for (const ADrone* Drone : Drones)
    {
        MaxDroneIndex = FMath::Max(MaxDroneIndex, Drone->GetDroneIndex());
    }
    OracleAssignedCellByDrone.Init(INDEX_NONE, MaxDroneIndex + 1);
    OracleAssignedScoreByDrone.Init(0.0f, MaxDroneIndex + 1);

    if (Drones.Num() == 0)
    {
        OracleAssignmentTick = AtTick;
        return;
    }

    auto CellPosition = [this](int32 Cell) -> FVector2f
    {
        return FVector2f(static_cast<float>(Cell % GridWidth) + 0.5f,
                         static_cast<float>(Cell / GridWidth) + 0.5f);
    };
    auto DronePosition = [this](const ADrone& Drone) -> FVector2f
    {
        const FVector P = Drone.GetActorLocation();
        return FVector2f(static_cast<float>((P.X - GridOriginWorld.X) / CellSizeUU),
                         static_cast<float>((P.Y - GridOriginWorld.Y) / CellSizeUU));
    };
    auto VantageCell = [this](const FVector2f& From, const FVector2f& Subject, float ViewRangeCells) -> int32
    {
        FVector2f Direction = (Subject - From).GetSafeNormal();
        if (Direction.IsNearlyZero())
        {
            Direction = FVector2f(1.0f, 0.0f);
        }
        const float Offset = FMath::Max(1.0f, ViewRangeCells * 0.5f);
        FVector2f Vantage = Subject + Direction * Offset;
        int32 X = FMath::Clamp(FMath::FloorToInt(Vantage.X), 0, GridWidth - 1);
        int32 Y = FMath::Clamp(FMath::FloorToInt(Vantage.Y), 0, GridHeight - 1);
        int32 Cell = Y * GridWidth + X;
        const int32 SubjectX = FMath::Clamp(FMath::FloorToInt(Subject.X), 0, GridWidth - 1);
        const int32 SubjectY = FMath::Clamp(FMath::FloorToInt(Subject.Y), 0, GridHeight - 1);
        if (Cell == SubjectY * GridWidth + SubjectX)
        {
            Vantage = Subject - Direction * Offset;
            X = FMath::Clamp(FMath::FloorToInt(Vantage.X), 0, GridWidth - 1);
            Y = FMath::Clamp(FMath::FloorToInt(Vantage.Y), 0, GridHeight - 1);
            Cell = Y * GridWidth + X;
        }
        return Cell;
    };
    auto KeyframeAtTick = [](const TArray<FHumanKeyframe>& Frames, int32 Tick) -> const FHumanKeyframe*
    {
        if (Frames.Num() == 0)
        {
            return nullptr;
        }
        int32 Low = 0;
        int32 High = Frames.Num();
        while (Low < High)
        {
            const int32 Mid = Low + (High - Low) / 2;
            if (Frames[Mid].Tick <= Tick)
            {
                Low = Mid + 1;
            }
            else
            {
                High = Mid;
            }
        }
        return &Frames[FMath::Clamp(Low - 1, 0, Frames.Num() - 1)];
    };

    // Reserve exactly one drone just in time for the first ignition.  This is
    // the fire-discovery ceiling without parking a drone there for the entire
    // pre-ignition period.
    if (OracleFirstFireTick == INDEX_NONE)
    {
        for (int32 Cell = 0; Cell < GridWidth * GridHeight; ++Cell)
        {
            const int32 FireTick = GetFirstFireTickByCellIndex(Cell);
            if (FireTick >= 0 &&
                (OracleFirstFireTick == INDEX_NONE || FireTick < OracleFirstFireTick ||
                 (FireTick == OracleFirstFireTick && Cell < OracleFirstFireCell)))
            {
                OracleFirstFireTick = FireTick;
                OracleFirstFireCell = Cell;
            }
        }
    }

    TSet<int32> ReservedDroneRows;
    if (OracleFirstFireCell != INDEX_NONE)
    {
        const int32 FireX = OracleFirstFireCell % GridWidth;
        const int32 FireY = OracleFirstFireCell / GridWidth;
        const bool bFireObserved = WorldGrid->getFirstFireDetectionTick(FireX, FireY) >= 0;
        int32 BestRow = INDEX_NONE;
        int32 BestTravelTicks = MAX_int32;
        for (int32 Row = 0; !bFireObserved && Row < Drones.Num(); ++Row)
        {
            const ADrone& Drone = *Drones[Row];
            const float RangeCells = Drone.MaxViewDistanceUU / FMath::Max(1.0f, CellSizeUU);
            const float DistanceCells = FVector2f::Distance(DronePosition(Drone), CellPosition(OracleFirstFireCell));
            const float TravelUU = FMath::Max(0.0f, DistanceCells - RangeCells * 0.75f) * CellSizeUU;
            const int32 TravelTicks = FMath::CeilToInt(TravelUU / FMath::Max(1.0f, Drone.SpeedUUPerTick));
            if (TravelTicks < BestTravelTicks ||
                (TravelTicks == BestTravelTicks && Drone.GetDroneIndex() < Drones[BestRow]->GetDroneIndex()))
            {
                BestTravelTicks = TravelTicks;
                BestRow = Row;
            }
        }
        if (BestRow != INDEX_NONE && AtTick + BestTravelTicks >= OracleFirstFireTick - 1)
        {
            ADrone& Drone = *Drones[BestRow];
            const float RangeCells = Drone.MaxViewDistanceUU / FMath::Max(1.0f, CellSizeUU);
            OracleAssignedCellByDrone[Drone.GetDroneIndex()] =
                VantageCell(DronePosition(Drone), CellPosition(OracleFirstFireCell), RangeCells);
            OracleAssignedScoreByDrone[Drone.GetDroneIndex()] = 1.0e9f;
            ReservedDroneRows.Add(BestRow);
        }
    }

    TArray<int32> CandidatePersonIds;
    for (int32 PersonId = 0; PersonId < OracleTrajectoryByPerson.Num(); ++PersonId)
    {
        if (OracleTrajectoryByPerson[PersonId].Num() > 0 &&
            (!PersonFirstDetectedTick.IsValidIndex(PersonId) || PersonFirstDetectedTick[PersonId] < 0))
        {
            CandidatePersonIds.Add(PersonId);
        }
    }

    TArray<int32> SolverRowToDroneRow;
    for (int32 Row = 0; Row < Drones.Num(); ++Row)
    {
        if (!ReservedDroneRows.Contains(Row))
        {
            SolverRowToDroneRow.Add(Row);
        }
    }

    TArray<TArray<int64>> Costs;
    TArray<TArray<int32>> TargetCells;
    Costs.SetNum(SolverRowToDroneRow.Num());
    TargetCells.SetNum(SolverRowToDroneRow.Num());
    for (int32 SolverRow = 0; SolverRow < SolverRowToDroneRow.Num(); ++SolverRow)
    {
        ADrone& Drone = *Drones[SolverRowToDroneRow[SolverRow]];
        Costs[SolverRow].Init(FOracleAssignmentSolver::InfeasibleCost, CandidatePersonIds.Num());
        TargetCells[SolverRow].Init(INDEX_NONE, CandidatePersonIds.Num());
        const FVector2f Start = DronePosition(Drone);
        const float ViewRangeCells = Drone.MaxViewDistanceUU / FMath::Max(1.0f, CellSizeUU);
        const float Speed = FMath::Max(1.0f, Drone.SpeedUUPerTick);

        for (int32 Task = 0; Task < CandidatePersonIds.Num(); ++Task)
        {
            const int32 PersonId = CandidatePersonIds[Task];
            const TArray<FHumanKeyframe>& Frames = OracleTrajectoryByPerson[PersonId];
            int32 InterceptTick = FMath::Max(AtTick, Frames[0].Tick);
            const int32 LatestTick = MaxSimTicks > 0 ? MaxSimTicks : Frames.Last().Tick;
            const FHumanKeyframe* Intercept = nullptr;
            for (int32 Iteration = 0; Iteration < 8; ++Iteration)
            {
                Intercept = KeyframeAtTick(Frames, InterceptTick);
                if (!Intercept || Intercept->State == EHumanState::Dead)
                {
                    break;
                }
                const FVector2f Subject(Intercept->X, Intercept->Y);
                const float DistanceCells = FVector2f::Distance(Start, Subject);
                const float TravelUU = FMath::Max(0.0f, DistanceCells - ViewRangeCells * 0.75f) * CellSizeUU;
                const int32 NextTick = FMath::Max(Frames[0].Tick, AtTick + FMath::CeilToInt(TravelUU / Speed));
                if (NextTick == InterceptTick)
                {
                    break;
                }
                InterceptTick = NextTick;
            }

            Intercept = KeyframeAtTick(Frames, InterceptTick);
            if (!Intercept || Intercept->State == EHumanState::Dead || InterceptTick > LatestTick)
            {
                continue;
            }

            const int32 DetectionLatency = FMath::Max(0, InterceptTick - Frames[0].Tick);
            const int64 DeadlinePenalty = DetectionLatency > RecallDeadlineTicks ? 1000000000000LL : 0LL;
            Costs[SolverRow][Task] = DeadlinePenalty + static_cast<int64>(DetectionLatency) * 1000000LL + PersonId;
            TargetCells[SolverRow][Task] =
                VantageCell(Start, FVector2f(Intercept->X, Intercept->Y), ViewRangeCells);
        }
    }

    const FOracleAssignmentSolver::FResult Assignment = FOracleAssignmentSolver::Solve(Costs);
    TSet<int32> AssignedPersonIds;
    TArray<int32> UnassignedSolverRows;
    for (int32 SolverRow = 0; SolverRow < Assignment.TaskByDrone.Num(); ++SolverRow)
    {
        const int32 Task = Assignment.TaskByDrone[SolverRow];
        if (Task == INDEX_NONE || !TargetCells[SolverRow].IsValidIndex(Task))
        {
            UnassignedSolverRows.Add(SolverRow);
            continue;
        }
        ADrone& Drone = *Drones[SolverRowToDroneRow[SolverRow]];
        OracleAssignedCellByDrone[Drone.GetDroneIndex()] = TargetCells[SolverRow][Task];
        OracleAssignedScoreByDrone[Drone.GetDroneIndex()] = -static_cast<float>(Costs[SolverRow][Task] / 1000000LL);
        AssignedPersonIds.Add(CandidatePersonIds[Task]);
    }

    // Spare drones monitor already-found people, prioritising anyone currently
    // exposed to fire.  This keeps the upper-bound contract meaningful for the
    // tracking metrics after first detection, not only for recall/latency.
    TArray<int32> TrackingPersonIds;
    for (int32 PersonId = 0; PersonId < OracleTrajectoryByPerson.Num(); ++PersonId)
    {
        if (AssignedPersonIds.Contains(PersonId) ||
            !PersonFirstDetectedTick.IsValidIndex(PersonId) ||
            PersonFirstDetectedTick[PersonId] < 0)
        {
            continue;
        }
        const FHumanKeyframe* Frame = KeyframeAtTick(OracleTrajectoryByPerson[PersonId], AtTick);
        if (Frame && Frame->Tick <= AtTick && Frame->State != EHumanState::Dead)
        {
            TrackingPersonIds.Add(PersonId);
        }
    }

    TArray<TArray<int64>> TrackingCosts;
    TArray<TArray<int32>> TrackingCells;
    TrackingCosts.SetNum(UnassignedSolverRows.Num());
    TrackingCells.SetNum(UnassignedSolverRows.Num());
    for (int32 Row = 0; Row < UnassignedSolverRows.Num(); ++Row)
    {
        ADrone& Drone = *Drones[SolverRowToDroneRow[UnassignedSolverRows[Row]]];
        const FVector2f Start = DronePosition(Drone);
        const float ViewRangeCells = Drone.MaxViewDistanceUU / FMath::Max(1.0f, CellSizeUU);
        TrackingCosts[Row].Init(FOracleAssignmentSolver::InfeasibleCost, TrackingPersonIds.Num());
        TrackingCells[Row].Init(INDEX_NONE, TrackingPersonIds.Num());
        for (int32 Task = 0; Task < TrackingPersonIds.Num(); ++Task)
        {
            const int32 PersonId = TrackingPersonIds[Task];
            const FHumanKeyframe* Frame = KeyframeAtTick(OracleTrajectoryByPerson[PersonId], AtTick);
            if (!Frame)
            {
                continue;
            }
            const FVector2f Subject(Frame->X, Frame->Y);
            const int32 X = FMath::Clamp(FMath::FloorToInt(Frame->X), 0, GridWidth - 1);
            const int32 Y = FMath::Clamp(FMath::FloorToInt(Frame->Y), 0, GridHeight - 1);
            const bool bInDanger = IsFireActive(X, Y, AtTick);
            const int64 SafePenalty = bInDanger ? 0LL : 1000000000000LL;
            const int64 DistanceCost = FMath::RoundToInt64(FVector2f::Distance(Start, Subject) * 1000.0f);
            TrackingCosts[Row][Task] = SafePenalty + DistanceCost * 1000LL + PersonId;
            TrackingCells[Row][Task] = VantageCell(Start, Subject, ViewRangeCells);
        }
    }

    const FOracleAssignmentSolver::FResult TrackingAssignment =
        FOracleAssignmentSolver::Solve(TrackingCosts);
    for (int32 Row = 0; Row < TrackingAssignment.TaskByDrone.Num(); ++Row)
    {
        const int32 Task = TrackingAssignment.TaskByDrone[Row];
        if (Task == INDEX_NONE)
        {
            continue;
        }
        ADrone& Drone = *Drones[SolverRowToDroneRow[UnassignedSolverRows[Row]]];
        OracleAssignedCellByDrone[Drone.GetDroneIndex()] = TrackingCells[Row][Task];
        OracleAssignedScoreByDrone[Drone.GetDroneIndex()] =
            -static_cast<float>(TrackingCosts[Row][Task] / 1000000LL);
    }

    OracleAssignmentTick = AtTick;
}

void ASimulationManager::DrawPriorityHeatmap() const
{
    if (DebugPrioritySource == EDebugPrioritySource::None) return;

    UWorld* World = GetWorld();
    if (!World || !WorldGrid.IsValid()) return;

    const int32 Step = FMath::Max(DebugPriorityDisplayStep, 1);
    const float HalfSize = CellSizeUU * Step * 0.45f;
    const float Z = GridOriginWorld.Z + FMath::Max(DebugPriorityZOffset, 20.0f);
    constexpr uint8 DepthPriority = 100;

    int32 BoxCount = 0;

    // Per-mode max for color normalization. Without this every value clamps to "red" because
    // drone priority caps near 4 (StalenessTerm=1 + UnseenExplorationBonus=3) and server caps
    // near 1. Tuned so the visible gradient spans the actual range of each mode.
    float ColorMaxValue = 1.0f;
    switch (DebugPrioritySource)
    {
        case EDebugPrioritySource::DroneLocalPriority:
            ColorMaxValue = 1.0f + PriorityUnseenExplorationBonus;  // ~4.0
            break;
        case EDebugPrioritySource::ServerPriority:
        case EDebugPrioritySource::GroundTruthPriority:
            ColorMaxValue = 5.0f;  // staleness saturated (1.0) + danger evidence headroom
            break;
        case EDebugPrioritySource::FireGroundTruth:
            ColorMaxValue = 1.0f;
            break;
        default: break;
    }

    for (int32 CY = 0; CY < GridHeight; CY += Step)
    {
        for (int32 CX = 0; CX < GridWidth; CX += Step)
        {
            float Priority = 0.0f;
            bool  bSkipIfZero = false;

            switch (DebugPrioritySource)
            {
                case EDebugPrioritySource::GroundTruthPriority:
                    Priority = ComputeGroundTruthPriority(CX, CY, CurrentTick);
                    bSkipIfZero = true;  // ground truth is sparse; show only "interesting" cells
                    break;

                case EDebugPrioritySource::DroneLocalPriority:
                {
                    // Each cell is coloured by its OWNER drone's view of that cell — so the
                    // overlay reflects "what the responsible drone currently knows" rather than
                    // forcing one fixed drone's belief across the whole grid. Cells with no
                    // owner ('N' — transient, before the first partition push) render at 0.
                    if (WorldGrid.IsValid())
                    {
                        const char OwnerChar = WorldGrid->getOwner(CX, CY);
                        if (OwnerChar != 'N')
                        {
                            const int32 OwnerIdx = static_cast<int32>(static_cast<uint8>(OwnerChar));
                            if (RegisteredDrones.IsValidIndex(OwnerIdx))
                            {
                                if (ADrone* D = RegisteredDrones[OwnerIdx].Get())
                                {
                                    Priority = D->ComputeCellPriority(CY * GridWidth + CX, CurrentTick);
                                }
                            }
                        }
                    }
                    // Render every cell — observed cells (Priority near 0) show in blue,
                    // unobserved cells (high) show in red. This is what reveals the changing field.
                    bSkipIfZero = false;
                    break;
                }

                case EDebugPrioritySource::FireGroundTruth:
                {
                    if (IsFireActive(CX, CY, CurrentTick))
                        Priority = 1.0f;
                    else
                        continue;
                    break;
                }

                case EDebugPrioritySource::ServerPriority:
                {
                    if (CachedServerPriority.Num() == GridWidth * GridHeight)
                    {
                        Priority = CachedServerPriority[CY * GridWidth + CX];
                    }
                    bSkipIfZero = false;  // same reasoning as DroneLocal
                    break;
                }

                default: break;
            }

            if (bSkipIfZero && Priority <= 0.0f) continue;

            const FVector Center(
                GridOriginWorld.X + (CX + 0.5f * Step) * CellSizeUU,
                GridOriginWorld.Y + (CY + 0.5f * Step) * CellSizeUU,
                Z);

            // Wireframe box only – no 3D text
            DrawDebugBox(World, Center, FVector(HalfSize, HalfSize, 2.0f),
                         PriorityToColor(Priority, ColorMaxValue), false, -1.0f, DepthPriority, 4.0f);

            ++BoxCount;
        }
    }

    if (GEngine)
    {
        GEngine->AddOnScreenDebugMessage(
            -1, 0.0f, FColor::Cyan,
            FString::Printf(TEXT("Heatmap: %s  |  Boxes: %d  |  Step: %d"),
                (DebugPrioritySource == EDebugPrioritySource::None) ? TEXT("None") :
                (DebugPrioritySource == EDebugPrioritySource::GroundTruthPriority) ? TEXT("GroundTruth") :
                (DebugPrioritySource == EDebugPrioritySource::DroneLocalPriority) ? TEXT("DroneLocal") :
                (DebugPrioritySource == EDebugPrioritySource::ServerPriority) ? TEXT("Server") :
                (DebugPrioritySource == EDebugPrioritySource::FireGroundTruth) ? TEXT("Fire") : TEXT("??"),
                BoxCount, Step));
    }
}

bool ASimulationManager::IsFireActive(int32 X, int32 Y, int32 Tick) const
{
    if (!WorldGrid.IsValid()) return false;
    const int32 First = WorldGrid->getFirstFireTick(X, Y);
    return (First >= 0) && (Tick >= First);
}

void ASimulationManager::ApplyPartitionOwnersToGrid(const TArray<uint16>& Owners)
{
    if (!WorldGrid.IsValid()) return;
    if (Owners.Num() != GridWidth * GridHeight)
    {
        UE_LOG(LogTemp, Warning, TEXT("Thesis: partition owner size %d != %d, skipping apply."),
               Owners.Num(), GridWidth * GridHeight);
        return;
    }

    // Build per-drone owned-cell lists in a single pass while writing the shared grid.
    TArray<TArray<int32>> PerDroneCells;
    TArray<TArray<int32>> PerDroneNewlyOwnedCells;
    PerDroneCells.SetNum(RegisteredDrones.Num());
    PerDroneNewlyOwnedCells.SetNum(RegisteredDrones.Num());
    const int32 EstimatedPerDrone = (GridWidth * GridHeight) / FMath::Max(RegisteredDrones.Num(), 1);
    for (int32 i = 0; i < RegisteredDrones.Num(); ++i)
    {
        PerDroneCells[i].Reserve(EstimatedPerDrone);
        PerDroneNewlyOwnedCells[i].Reserve(FMath::Max(16, EstimatedPerDrone / 4));
    }

    for (int32 y = 0; y < GridHeight; ++y)
    {
        for (int32 x = 0; x < GridWidth; ++x)
        {
            const int32 Flat = y * GridWidth + x;
            const char PreviousOwner = WorldGrid->getOwner(x, y);
            const uint16 Idx = Owners[Flat];
            const char Encoded = (Idx == 0xFFFF) ? 'N' : static_cast<char>(FMath::Min<uint16>(Idx, 254));
            WorldGrid->setOwner(x, y, Encoded);

            if (Idx != 0xFFFF && PerDroneCells.IsValidIndex(static_cast<int32>(Idx)))
            {
                const int32 DroneIdx = static_cast<int32>(Idx);
                PerDroneCells[DroneIdx].Add(Flat);
                if (PreviousOwner != Encoded)
                {
                    PerDroneNewlyOwnedCells[DroneIdx].Add(Flat);
                }
            }
        }
    }

    for (int32 i = 0; i < RegisteredDrones.Num(); ++i)
    {
        if (ADrone* D = RegisteredDrones[i].Get())
        {
            D->SetOwnedCells(MoveTemp(PerDroneCells[i]), PerDroneNewlyOwnedCells[i]);
        }
    }
}

void ASimulationManager::EnsurePartitionStrategy()
{
    const FName Wanted(*PartitionStrategyName);
    if (PartitionStrategy.IsValid() && PartitionStrategy->GetStrategyName() == Wanted)
    {
        return;
    }

    PartitionStrategy = FPartitionStrategyFactory::Create(Wanted);
    if (!PartitionStrategy.IsValid())
    {
        FString Known;
        for (const FName& Name : FPartitionStrategyFactory::RegisteredNames())
        {
            Known += Name.ToString() + TEXT(" ");
        }
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis: unknown partition strategy '%s' (registered: %s) — falling back to '%s'."),
               *PartitionStrategyName, *Known,
               *FPartitionStrategyFactory::DefaultStrategyName().ToString());
        PartitionStrategy = FPartitionStrategyFactory::Create(FPartitionStrategyFactory::DefaultStrategyName());
        PartitionStrategyName = FPartitionStrategyFactory::DefaultStrategyName().ToString();
    }
}

FPartitionParams ASimulationManager::MakePartitionParams() const
{
    FPartitionParams Params;
    Params.GridWidth = GridWidth;
    Params.GridHeight = GridHeight;
    Params.ConfirmedDetectionThreshold = ConfirmedDetectionThreshold;
    Params.WeightConfirmed = PartitionWeightConfirmed;
    Params.WeightExpected = PartitionWeightExpected;
    Params.WeightNormal = PartitionWeightNormal;
    Params.BackgroundTileFraction = BackgroundTileFraction;
    Params.MaxFireZoneFraction = MaxFireZoneFraction;
    Params.RunSeed = FireGeneratorSeed;
    Params.OracleLookaheadTicks = OracleLookaheadTicks;
    Params.Oracle = IsOracleInformationEnabled() ? static_cast<const IGroundTruthOracle*>(this) : nullptr;
    return Params;
}

TArray<uint16> ASimulationManager::ComputeBalancedPartition(
    const TArray<TObjectPtr<ADrone>>& ActiveDrones,
    const TArray<FVector2f>& DronePositionsCellSpace,
    int32 Tick)
{
    EnsurePartitionStrategy();
    return PartitionStrategy->ComputePartition(
        ActiveDrones, DronePositionsCellSpace, Tick, MakePartitionParams());
}

int32 ASimulationManager::DebugGetOwner(int32 X, int32 Y) const
{
    if (!WorldGrid.IsValid()) return -1;
    if (X < 0 || Y < 0 || X >= GridWidth || Y >= GridHeight) return -1;
    const char O = WorldGrid->getOwner(X, Y);
    return (O == 'N') ? -1 : static_cast<int32>(static_cast<uint8>(O));
}

void ASimulationManager::BuildTerrainIfEnabled()
{
    if (!TerrainActor)
    {
        if (UWorld* World = GetWorld())
        {
            TArray<AActor*> Found;
            UGameplayStatics::GetAllActorsOfClass(World, AProceduralTerrain::StaticClass(), Found);
            if (Found.Num() > 0)
            {
                TerrainActor = Cast<AProceduralTerrain>(Found[0]);
            }
            else if (bAutoGenerateTerrain)
            {
                FActorSpawnParameters Params;
                Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                TerrainActor = World->SpawnActor<AProceduralTerrain>(
                    AProceduralTerrain::StaticClass(), GridOriginWorld, FRotator::ZeroRotator, Params);
            }
        }
    }

    if (!TerrainActor)
    {
        return;
    }

    // Push all manager-owned settings before building — applies to both
    // level-placed and auto-spawned terrain; no need to set these on the actor.
    TerrainActor->Profile                   = TerrainProfile;
    TerrainActor->SlopeIntensity            = TerrainSlopeIntensity;
    TerrainActor->DetailDensity             = TerrainDetailDensity;
    TerrainActor->SeedOverride              = TerrainSeedOverride;
    TerrainActor->VerticesPerCell           = TerrainVerticesPerCell;
    TerrainActor->PresentationMarginMeters = TerrainPresentationMarginMeters;
    TerrainActor->SlopeMovementCostFactor   = TerrainSlopeMovementCostFactor;
    TerrainActor->bVisibleOnlyToDroneCaptures = bTerrainVisibleOnlyToDroneCaptures;
    TerrainActor->bRenderVisuals            = bRenderTerrainVisuals;

    // The manager owns the generated terrain's visual baseline. Level-placed
    // terrain actors can otherwise keep stale/white material overrides from
    // older look-dev passes, which makes the floor render as an untextured slab.
    if (UMaterialInterface* TerrainMat = LoadObject<UMaterialInterface>(
        nullptr, ThesisTerrainMaterialObjectPath))
    {
        if (TerrainActor->TerrainMaterial != TerrainMat)
        {
            const FString PreviousPath = TerrainActor->TerrainMaterial
                ? TerrainActor->TerrainMaterial->GetPathName()
                : TEXT("<none>");
            UE_LOG(LogTemp, Log,
                   TEXT("Thesis terrain: assigning Geres terrain material %s (was %s)."),
                   *TerrainMat->GetPathName(), *PreviousPath);
        }
        TerrainActor->TerrainMaterial = TerrainMat;
    }
    else if (!TerrainActor->TerrainMaterial)
    {
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis terrain: %s is missing; using AProceduralTerrain fallback material."),
               ThesisTerrainMaterialObjectPath);
    }

    TerrainActor->BuildTerrain(GridWidth, GridHeight, GridOriginWorld, CellSizeUU, FireGeneratorSeed);
    ApplyViewVisualMode();

    const FString TerrainCsv = ResolveProjectRelativePath(
        GeneratedTerrainCsvOutputPath, TEXT("Saved/GeneratedTerrain.csv"));
    TerrainActor->ExportMovementCostCsv(TerrainCsv);

    // Keep drones safely above local terrain; this is especially important for
    // vision captures because soil/rocks close to the camera drown YOLO input.
    if (DroneCruiseAltitudeAboveGroundUU < FMath::Max(DroneMinimumAltitudeAboveGroundUU, CellSizeUU))
    {
        UE_LOG(LogTemp, Warning,
               TEXT("Thesis terrain: relief amplitude %.0f UU is close to/above drone altitude %.0f UU — raise AutoDroneSpawnZ or lower TerrainSlopeIntensity."),
               DroneCruiseAltitudeAboveGroundUU, CellSizeUU);
    }
}

void ASimulationManager::BuildForestIfPresent()
{
    if (!WorldGrid.IsValid())
    {
        return;
    }

    if (!ForestDresserActor)
    {
        if (UWorld* World = GetWorld())
        {
            TArray<AActor*> Found;
            UGameplayStatics::GetAllActorsOfClass(World, AForestDresser::StaticClass(), Found);
            if (Found.Num() > 0)
            {
                ForestDresserActor = Cast<AForestDresser>(Found[0]);
            }
            else
            {
                // Zero-config: spawn one (it auto-populates meshes from
                // /Game/ThesisFlora when its lists are empty).
                FActorSpawnParameters Params;
                Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                ForestDresserActor = World->SpawnActor<AForestDresser>(
                    AForestDresser::StaticClass(), GridOriginWorld, FRotator::ZeroRotator, Params);
            }
        }
    }

    if (ForestDresserActor)
    {
        // Push all manager-owned settings before building — applies to both
        // level-placed and auto-spawned forest; no need to set these on the actor.
        ForestDresserActor->ForestCoverage               = ForestCoverage;
        ForestDresserActor->NoiseScaleCells              = ForestNoiseScaleCells;
        ForestDresserActor->MaxTreesPerCell              = ForestMaxTreesPerCell;
        ForestDresserActor->MaxBushesPerCell             = ForestMaxBushesPerCell;
        ForestDresserActor->GroundCoverPerCell           = ForestGroundCoverPerCell;
        ForestDresserActor->SurfaceDebrisPerCell         = ForestSurfaceDebrisPerCell;
        ForestDresserActor->SurfaceDebrisEmbedUU         = ForestSurfaceDebrisEmbedUU;
        ForestDresserActor->SparseRockEmbedUU            = ForestSparseRockEmbedUU;
        ForestDresserActor->TreeScaleMin                 = ForestTreeScaleMin;
        ForestDresserActor->TreeScaleMax                 = ForestTreeScaleMax;
        ForestDresserActor->TreeZOffsetUU                = ForestTreeZOffsetUU;
        ForestDresserActor->SeedOverride                 = ForestSeedOverride;
        ForestDresserActor->bAutoPopulateFromThesisFlora = bForestAutoPopulate;
        ForestDresserActor->bVisibleOnlyToDroneCaptures  = bForestVisibleOnlyToDroneCaptures;
        ForestDresserActor->bDisableFoliageWind          = bForestDisableFoliageWind;
        // A visible standalone game also needs foliage. Only skip geometry for
        // genuinely headless/suppressed runs with no active sensor captures.
        ForestDresserActor->bSpawnVisualInstances = bEnableDroneCaptures ||
            (FApp::CanEverRender() && !bSuppressBenchmarkViewport);
        ForestDresserActor->BuildForest(WorldGrid.Get(), GridOriginWorld, CellSizeUU, FireGeneratorSeed,
                                        TerrainActor.Get());
        ApplyViewVisualMode();
    }
}

void ASimulationManager::ExportFuelCsvFromGrid()
{
    if (!WorldGrid.IsValid())
    {
        return;
    }

    const FString FuelCsv = ResolveProjectRelativePath(
        GeneratedFuelCsvOutputPath, TEXT("Saved/GeneratedFuel.csv"));
    IFileManager::Get().MakeDirectory(*FPaths::GetPath(FuelCsv), true);

    FString Csv;
    Csv.Reserve(GridWidth * GridHeight * 6);
    for (int32 Y = 0; Y < GridHeight; ++Y)
    {
        for (int32 X = 0; X < GridWidth; ++X)
        {
            Csv += FString::Printf(TEXT("%.3f"), WorldGrid->getFloraDensity(X, Y));
            Csv += (X + 1 < GridWidth) ? TEXT(",") : TEXT("\n");
        }
    }
    FFileHelper::SaveStringToFile(Csv, *FuelCsv);
}

bool ASimulationManager::IsFireEditorPreviewEnabled() const
{
    return bPreviewFireInEditor && bFireVisualsOnlyForYoloCaptures && GetWorld()
        && GetWorld()->WorldType == EWorldType::PIE && !bSuppressBenchmarkViewport;
}

void ASimulationManager::SetupFireVisuals()
{
    if (!bEnableFireVisuals || !WorldGrid.IsValid())
    {
        return;
    }

    if (!FireVisualsActor)
    {
        if (UWorld* World = GetWorld())
        {
            TArray<AActor*> Found;
            UGameplayStatics::GetAllActorsOfClass(World, AFireVisualsManager::StaticClass(), Found);
            if (Found.Num() > 0)
            {
                FireVisualsActor = Cast<AFireVisualsManager>(Found[0]);
            }
            else
            {
                FActorSpawnParameters Params;
                Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                FireVisualsActor = World->SpawnActor<AFireVisualsManager>(
                    AFireVisualsManager::StaticClass(), GridOriginWorld, FRotator::ZeroRotator, Params);
            }
        }
    }

    if (FireVisualsActor)
    {
        auto Configure = [&](AFireVisualsManager* Visuals, bool bPreview)
        {
            // Push all manager-owned settings before Init — applies to both
            // level-placed and auto-spawned fire visuals manager.
            // Do not silently replace visible editor values. The legacy showcase
            // flag remains serialized for old maps but no longer owns hidden tuning.
            Visuals->VisualProfile              = FireVisualProfile;
            Visuals->MaxFireVfx               = FireVisualsMaxVfx;
            // fast_unshadowed_v2 is an appearance experiment: keep one
            // connected-front flame support per selected cell while retaining
            // the separately configured shared smoke layers below.
            Visuals->FireClustersPerCell      = !bPreview &&
                (CameraRenderProfile == TEXT("fast_unshadowed_v2") || CameraRenderProfile == TEXT("fast_shader_v3"))
                ? 1 : FireVisualsClustersPerCell;
            Visuals->FireClusterRadiusUU      = FireVisualsClusterRadiusUU;
            Visuals->MaxFireLights            = FireVisualsMaxLights;
            Visuals->MaxShadowCastingFireLights = FireVisualsMaxShadowCastingLights;
            Visuals->LightRadiusUU            = FireVisualsLightRadiusUU;
            Visuals->LightIntensity           = FireVisualsLightIntensity;
            Visuals->ReassignEveryNTicks      = FireVisualsReassignEveryNTicks;
            Visuals->bAutoLoadDefaultVfxAssets = bFireVisualsAutoLoadDefaultVfx;
            Visuals->bScaleWithFuel           = bFireVisualsScaleWithFuel;
            Visuals->VfxScaleMultiplier       = FireVisualsVfxScaleMultiplier;
            Visuals->FlameWidthScaleMultiplier = FireVisualsFlameWidthScaleMultiplier;
            Visuals->FlameHeightScaleMultiplier = FireVisualsFlameHeightScaleMultiplier;
            Visuals->BundledFlameSpriteSizeMultiplier = FireVisualsBundledFlameSpriteSizeMultiplier;
            Visuals->bEnableBundledHeatDistortion = bFireVisualsEnableBundledHeatDistortion;
            Visuals->bConnectedFireFront = bFireVisualsConnectedFireFront;
            Visuals->OuterFlameScaleMultiplier = FireVisualsOuterFlameScaleMultiplier;
            Visuals->EmberScaleMultiplier     = FireVisualsEmberScaleMultiplier;
            Visuals->SmokeVfxScaleMultiplier  = FireVisualsSmokeVfxScaleMultiplier;
            Visuals->SmokeLayerCount          = FireVisualsSmokeLayerCount;
            Visuals->bShareSmokeAcrossCellClusters = bFireVisualsShareSmokeAcrossCellClusters;
            Visuals->SmokeLayerRadiusUU       = FireVisualsSmokeLayerRadiusUU;
            Visuals->SmokeLayerZStepUU        = FireVisualsSmokeLayerZStepUU;
            Visuals->FlameZOffsetUU           = FireVisualsFlameZOffsetUU;
            Visuals->SmokeZOffsetUU           = FireVisualsSmokeZOffsetUU;
            Visuals->LightZOffsetUU           = FireVisualsLightZOffsetUU;
            Visuals->LightSourceRadiusUU      = FireVisualsLightSourceRadiusUU;
            Visuals->LightVolumetricScatteringIntensity = FireVisualsLightVolumetricScatteringIntensity;
            Visuals->bIncrementalNiagaraPreviewStepping = bPreview;
            Visuals->bFastDeterministicCapture = !bPreview && CameraRenderProfile != TEXT("legacy");
            Visuals->bReuseDeterministicCaptureState = !bPreview && CameraRenderProfile == TEXT("fast_unshadowed_v2");
            Visuals->bUseFastShaderV3 = !bPreview && CameraRenderProfile == TEXT("fast_shader_v3");
            Visuals->bUseRealtimeNiagaraForPreview = bPreview;
            Visuals->bHideFromSceneCaptures = bPreview;
            Visuals->bLoadVfxOnlyForYoloCapture = !bPreview && bFireVisualsOnlyForYoloCaptures;
            Visuals->bVfxOnlyInDroneCaptures = !bPreview
                && (bFireVisualsOnlyForYoloCaptures || bFireVisualsVfxOnlyInDroneCaptures);
            // Preview illumination and scorch must not feed back into sensor views.
            Visuals->bFireLights = !bPreview && bFireLightsEnabled;
            Visuals->bScorchedGroundFeedback = !bPreview && bFireVisualsScorchedGround;
            Visuals->ScorchFlushEveryNTicks   = FireVisualsScorchFlushEveryNTicks;
            Visuals->Init(WorldGrid.Get(), GridOriginWorld, CellSizeUU,
                                   BurnDurationTicks, TerrainActor.Get(),
                                   ForestDresserActor.Get());
        };
        Configure(FireVisualsActor, false);
        if (IsFireEditorPreviewEnabled())
        {
            if (!EditorFirePreviewActor)
            {
                FActorSpawnParameters Params;
                Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                EditorFirePreviewActor = GetWorld()->SpawnActor<AFireVisualsManager>(
                    AFireVisualsManager::StaticClass(), GridOriginWorld, FRotator::ZeroRotator, Params);
            }
            if (EditorFirePreviewActor) Configure(EditorFirePreviewActor, true);
        }
        else if (EditorFirePreviewActor)
        {
            EditorFirePreviewActor->Destroy();
            EditorFirePreviewActor = nullptr;
        }
        ApplyViewVisualMode();
    }
}

void ASimulationManager::EnsureHumanManager()
{
    const FString CsvPath = ResolveProjectRelativePath(HumansCSVPath, FString());
    if (CsvPath.IsEmpty())
    {
        return;   // humans disabled for this run
    }

    if (!HumanManager)
    {
        if (UWorld* World = GetWorld())
        {
            TArray<AActor*> Found;
            UGameplayStatics::GetAllActorsOfClass(World, AHumanManager::StaticClass(), Found);
            if (Found.Num() > 0)
            {
                HumanManager = Cast<AHumanManager>(Found[0]);
            }
            else
            {
                FActorSpawnParameters Params;
                Params.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
                HumanManager = World->SpawnActor<AHumanManager>(
                    AHumanManager::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, Params);
            }
        }
    }

    if (HumanManager)
    {
        HumanManager->HumanMeshScale = FMath::Max(0.1f, HumanMeshScale);
        HumanManager->VisualMeshSet = HumanVisualMeshSet;
        HumanManager->VisualPoseMode = HumanPoseMode;
        HumanManager->VisualSecondsPerTick = SecondsPerTick;
        HumanManager->VisualSeed = HumanGeneratorSeed != 0 ? HumanGeneratorSeed : FireGeneratorSeed;
        // Skip the per-tick skeletal-mesh update when nothing renders it (headless
        // -game with no captures); ground-truth human counts are written regardless.
        HumanManager->bUpdateVisualMeshes = bEnableDroneCaptures || (FApp::CanEverRender() && !bSuppressBenchmarkViewport);
        HumanManager->bDeferVisualUpdatesUntilCapture = bEnableDroneCaptures &&
            DroneCaptureEveryNTicks > 0 && bSuppressBenchmarkViewport &&
            !FParse::Param(FCommandLine::Get(), TEXT("ThesisLegacyHumanVisualUpdates"));
        HumanManager->SetGridFrame(GridOriginWorld, CellSizeUU);
        HumanManager->SetTerrain(TerrainActor.Get());
        if (!HumanManager->LoadTrajectories(CsvPath))
        {
            UE_LOG(LogTemp, Warning, TEXT("Thesis: failed to load humans CSV %s — humans disabled."), *CsvPath);
        }
        ApplyViewVisualMode();
    }
}

bool ASimulationManager::ProjectRayToTerrain(const FVector& Origin, const FVector& Dir, FVector& OutHit) const
{
    if (Dir.Z >= -KINDA_SMALL_NUMBER)
    {
        return false;
    }

    if (!TerrainActor || CellSizeUU <= 0.0f || GridWidth <= 0 || GridHeight <= 0)
    {
        const float T = (GridOriginWorld.Z - Origin.Z) / Dir.Z;
        if (T <= 0.0f)
        {
            return false;
        }
        OutHit = Origin + Dir * T;
        return true;
    }

    auto HeightDeltaAt = [&](float T) -> float
    {
        const FVector P = Origin + Dir * T;
        const float CellX = (P.X - GridOriginWorld.X) / CellSizeUU;
        const float CellY = (P.Y - GridOriginWorld.Y) / CellSizeUU;
        return P.Z - TerrainActor->GetHeightAtCell(CellX, CellY);
    };

    const float LowestExpectedTerrain =
        GridOriginWorld.Z - FMath::Max(CellSizeUU, TerrainActor->BuiltAmplitudeUU + CellSizeUU);
    const float TMax = FMath::Clamp(
        (Origin.Z - LowestExpectedTerrain) / FMath::Max(-Dir.Z, KINDA_SMALL_NUMBER),
        CellSizeUU,
        CellSizeUU * FMath::Max(GridWidth, GridHeight) * 2.0f);

    float PrevT = 0.0f;
    float PrevDelta = HeightDeltaAt(PrevT);
    if (PrevDelta <= 0.0f)
    {
        OutHit = Origin;
        return true;
    }

    constexpr int32 MarchSteps = 96;
    for (int32 Step = 1; Step <= MarchSteps; ++Step)
    {
        const float T = TMax * (static_cast<float>(Step) / static_cast<float>(MarchSteps));
        const float Delta = HeightDeltaAt(T);
        if (Delta <= 0.0f)
        {
            float Lo = PrevT;
            float Hi = T;
            for (int32 Iter = 0; Iter < 10; ++Iter)
            {
                const float Mid = 0.5f * (Lo + Hi);
                if (HeightDeltaAt(Mid) > 0.0f)
                {
                    Lo = Mid;
                }
                else
                {
                    Hi = Mid;
                }
            }
            OutHit = Origin + Dir * Hi;
            return true;
        }
        PrevT = T;
        PrevDelta = Delta;
    }

    return false;
}

int32 ASimulationManager::ProjectImagePointToCell(const UDroneCaptureComponent* Capture, float U, float V) const
{
    if (!Capture)
    {
        return INDEX_NONE;
    }

    const FTransform CamXf = Capture->GetComponentTransform();
    const FVector Origin = CamXf.GetLocation();
    const float TanHalfFOV = FMath::Tan(FMath::DegreesToRadians(Capture->FOVAngle * 0.5f));

    // Square image: vertical FOV == horizontal. U,V in [0,1], V down.
    const FVector Dir = (CamXf.GetUnitAxis(EAxis::X)
                       + CamXf.GetUnitAxis(EAxis::Y) * ((U * 2.0f - 1.0f) * TanHalfFOV)
                       - CamXf.GetUnitAxis(EAxis::Z) * ((V * 2.0f - 1.0f) * TanHalfFOV)).GetSafeNormal();

    FVector P;
    if (!ProjectRayToTerrain(Origin, Dir, P))
    {
        return INDEX_NONE;
    }

    const int32 CX = FMath::FloorToInt((P.X - GridOriginWorld.X) / CellSizeUU);
    const int32 CY = FMath::FloorToInt((P.Y - GridOriginWorld.Y) / CellSizeUU);
    if (CX < 0 || CX >= GridWidth || CY < 0 || CY >= GridHeight)
    {
        return INDEX_NONE;
    }
    return CY * GridWidth + CX;
}

int32 ASimulationManager::ProjectImagePointToCell(const FTransform& CamXf, float CameraFOV,
                                                   float U, float V, bool& bOutOfGrid) const
{
    bOutOfGrid = false;

    const FVector Origin = CamXf.GetLocation();
    const float TanHalfFOV = FMath::Tan(FMath::DegreesToRadians(CameraFOV * 0.5f));

    // Square image: vertical FOV == horizontal. U,V in [0,1], V down.
    const FVector Dir = (CamXf.GetUnitAxis(EAxis::X)
                       + CamXf.GetUnitAxis(EAxis::Y) * ((U * 2.0f - 1.0f) * TanHalfFOV)
                       - CamXf.GetUnitAxis(EAxis::Z) * ((V * 2.0f - 1.0f) * TanHalfFOV)).GetSafeNormal();

    FVector P;
    if (!ProjectRayToTerrain(Origin, Dir, P))
    {
        return INDEX_NONE;
    }

    const int32 CX = FMath::FloorToInt((P.X - GridOriginWorld.X) / CellSizeUU);
    const int32 CY = FMath::FloorToInt((P.Y - GridOriginWorld.Y) / CellSizeUU);
    if (CX < 0 || CX >= GridWidth || CY < 0 || CY >= GridHeight)
    {
        bOutOfGrid = true;
        return INDEX_NONE;
    }
    return CY * GridWidth + CX;
}

bool ASimulationManager::ProjectWorldPointToImage(const UDroneCaptureComponent* Capture, const FVector& WorldPoint,
                                                  float& OutU, float& OutV) const
{
    OutU = 0.0f;
    OutV = 0.0f;
    if (!Capture)
    {
        return false;
    }

    const FTransform CamXf = Capture->GetComponentTransform();
    return ThesisVisionProjection::ProjectWorldPointToImage(
        CamXf.GetLocation(), CamXf.GetUnitAxis(EAxis::X), CamXf.GetUnitAxis(EAxis::Y),
        CamXf.GetUnitAxis(EAxis::Z), Capture->FOVAngle, WorldPoint, OutU, OutV);
}

namespace
{
    // Draws a hollow rectangle (thickness Thick px) into a BGRA FColor buffer.
    void DrawRectBGRA(TArray<FColor>& Px, int32 W, int32 H,
                      int32 X0, int32 Y0, int32 X1, int32 Y1, FColor C, int32 Thick)
    {
        if (X1 < X0) { Swap(X0, X1); }
        if (Y1 < Y0) { Swap(Y0, Y1); }
        auto Plot = [&](int32 X, int32 Y)
        {
            if (X >= 0 && X < W && Y >= 0 && Y < H) { Px[Y * W + X] = C; }
        };
        for (int32 t = 0; t < Thick; ++t)
        {
            for (int32 X = X0; X <= X1; ++X) { Plot(X, Y0 + t); Plot(X, Y1 - t); }
            for (int32 Y = Y0; Y <= Y1; ++Y) { Plot(X0 + t, Y); Plot(X1 - t, Y); }
        }
    }

    void DrawCrossBGRA(TArray<FColor>& Px, int32 W, int32 H,
                       int32 X, int32 Y, int32 Radius, FColor C, int32 Thick)
    {
        auto Plot = [&](int32 PX, int32 PY)
        {
            if (PX >= 0 && PX < W && PY >= 0 && PY < H) { Px[PY * W + PX] = C; }
        };
        for (int32 t = -Thick / 2; t <= Thick / 2; ++t)
        {
            for (int32 D = -Radius; D <= Radius; ++D)
            {
                Plot(X + D, Y + t);
                Plot(X + t, Y + D);
            }
        }
    }

    void DrawNoPersonDetectionMarkerBGRA(TArray<FColor>& Px, int32 W, int32 H)
    {
        DrawRectBGRA(Px, W, H, 1, 1, W - 2, H - 2, FColor(230, 40, 40, 255), 4);
        const int32 Size = FMath::Clamp(FMath::Min(W, H) / 18, 12, 36);
        const int32 Pad = FMath::Clamp(FMath::Min(W, H) / 60, 4, 12);
        for (int32 D = 0; D <= Size; ++D)
        {
            for (int32 T = 0; T < 3; ++T)
            {
                const int32 X0 = Pad + D;
                const int32 Y0 = Pad + D + T;
                const int32 X1 = Pad + Size - D;
                const int32 Y1 = Pad + D + T;
                if (X0 >= 0 && X0 < W && Y0 >= 0 && Y0 < H) { Px[Y0 * W + X0] = FColor(230, 40, 40, 255); }
                if (X1 >= 0 && X1 < W && Y1 >= 0 && Y1 < H) { Px[Y1 * W + X1] = FColor(230, 40, 40, 255); }
            }
        }
    }
}

bool ASimulationManager::CellOrNeighborsHaveRealPerson(int32 CellIndex) const
{
    if (CellIndex < 0 || CellIndex >= GridWidth * GridHeight)
    {
        return false;
    }
    const int32 CX = CellIndex % GridWidth;
    const int32 CY = CellIndex / GridWidth;
    for (int32 Dy = -1; Dy <= 1; ++Dy)
    {
        for (int32 Dx = -1; Dx <= 1; ++Dx)
        {
            const int32 X = CX + Dx;
            const int32 Y = CY + Dy;
            if (X < 0 || X >= GridWidth || Y < 0 || Y >= GridHeight)
            {
                continue;
            }
            if (GetRealPersonCountByCellIndex(Y * GridWidth + X) > 0)
            {
                return true;
            }
        }
    }
    return false;
}

int32 ASimulationManager::CollectCaptureFootprint(const UDroneCaptureComponent* Capture, TArray<int32>& OutCells) const
{
    OutCells.Reset();
    if (!Capture)
    {
        return 0;
    }
    const float Us[5] = { 0.05f, 0.95f, 0.05f, 0.95f, 0.5f };
    const float Vs[5] = { 0.05f, 0.05f, 0.95f, 0.95f, 0.5f };
    int32 MinX = GridWidth, MinY = GridHeight, MaxX = -1, MaxY = -1;
    for (int32 K = 0; K < 5; ++K)
    {
        const int32 Cell = ProjectImagePointToCell(Capture, Us[K], Vs[K]);
        if (Cell == INDEX_NONE)
        {
            continue;
        }
        const int32 X = Cell % GridWidth;
        const int32 Y = Cell / GridWidth;
        MinX = FMath::Min(MinX, X);
        MaxX = FMath::Max(MaxX, X);
        MinY = FMath::Min(MinY, Y);
        MaxY = FMath::Max(MaxY, Y);
    }
    // The projected image corners only bound the footprint. The quadrilateral
    // may be rotated or skewed by camera tilt, so retain a cell only when its
    // terrain-height centre actually projects into the image.
    for (int32 Y = MinY; Y <= MaxY; ++Y)
    {
        for (int32 X = MinX; X <= MaxX; ++X)
        {
            const int32 Cell = Y * GridWidth + X;
            const float CellX = static_cast<float>(X) + 0.5f;
            const float CellY = static_cast<float>(Y) + 0.5f;
            const float GroundZ = TerrainActor
                ? TerrainActor->GetHeightAtCell(CellX, CellY)
                : GridOriginWorld.Z;
            const FVector CellCentre(
                GridOriginWorld.X + CellX * CellSizeUU,
                GridOriginWorld.Y + CellY * CellSizeUU,
                GroundZ);
            float U = 0.0f, V = 0.0f;
            if (ProjectWorldPointToImage(Capture, CellCentre, U, V))
            {
                OutCells.Add(Cell);
            }
        }
    }

    // Count ground-truth people by their actual projected anchors, not by
    // occupied cells in a rectangular AABB. This is geometric frustum
    // membership only; terrain/building occlusion is not modelled here.
    int32 GtPersons = 0;
    if (HumanManager)
    {
        for (const FHumanSnapshot& Snapshot : HumanManager->GetCurrentSnapshots())
        {
            if (Snapshot.PersonId == INDEX_NONE || Snapshot.State == EHumanState::Dead)
            {
                continue;
            }
            const FVector Anchor(
                GridOriginWorld.X + Snapshot.PosCell.X * CellSizeUU,
                GridOriginWorld.Y + Snapshot.PosCell.Y * CellSizeUU,
                (TerrainActor
                    ? TerrainActor->GetHeightAtCell(Snapshot.PosCell.X, Snapshot.PosCell.Y)
                    : GridOriginWorld.Z) + 100.0f);
            float U = 0.0f, V = 0.0f;
            if (ProjectWorldPointToImage(Capture, Anchor, U, V))
            {
                ++GtPersons;
            }
        }
    }
    return GtPersons;
}

void ASimulationManager::SaveYoloImagePair(const TCHAR* Tag, int32 DroneIndex, int32 Tick, int32 W, int32 H,
                                           const TArray<FColor>& Pixels, const TArray<FVisionDetection>& PersonDets,
                                           const TArray<bool>& IsTruePositive,
                                           const UDroneCaptureComponent* Capture) const
{
    if (W <= 0 || H <= 0 || Pixels.Num() < W * H)
    {
        return;
    }
    const FString BaseDir = MetricsRecorder.IsActive()
        ? MetricsRecorder.GetRunDirectory()
        : FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"));
    const FString Dir = FPaths::Combine(BaseDir, TEXT("vision"));
    IFileManager::Get().MakeDirectory(*Dir, true);
    const FString Base = FString::Printf(TEXT("%s_drone%d_t%06d"), Tag, DroneIndex, Tick);

    // Raw frame.
    {
        TArray<FColor> Raw = Pixels;
        for (FColor& P : Raw) { P.A = 255; }
        TArray64<uint8> Png;
        FImageUtils::PNGCompressImageArray(W, H, Raw, Png);
        FFileHelper::SaveArrayToFile(Png, *FPaths::Combine(Dir, Base + TEXT("_raw.png")));
    }

    // Annotated frame: green box = true positive, red box = false positive.
    // Miss-only frames have no YOLO boxes, so draw a red frame/cross marker to
    // make the "YOLO output" image visually distinct from the raw capture.
    {
        TArray<FColor> Annot = Pixels;
        for (FColor& P : Annot) { P.A = 255; }
        for (int32 K = 0; K < PersonDets.Num(); ++K)
        {
            const FVisionDetection& Det = PersonDets[K];
            const bool bTP = IsTruePositive.IsValidIndex(K) && IsTruePositive[K];
            const FColor BoxColor = bTP ? FColor(40, 220, 40, 255) : FColor(230, 40, 40, 255);
            const int32 X0 = FMath::Clamp(FMath::RoundToInt(Det.X0 * W), 0, W - 1);
            const int32 X1 = FMath::Clamp(FMath::RoundToInt(Det.X1 * W), 0, W - 1);
            const int32 Y0 = FMath::Clamp(FMath::RoundToInt(Det.Y0 * H), 0, H - 1);
            const int32 Y1 = FMath::Clamp(FMath::RoundToInt(Det.Y1 * H), 0, H - 1);
            DrawRectBGRA(Annot, W, H, X0, Y0, X1, Y1, BoxColor, 2);
        }
        if (PersonDets.Num() == 0)
        {
            DrawNoPersonDetectionMarkerBGRA(Annot, W, H);
        }
        TArray64<uint8> Png;
        FImageUtils::PNGCompressImageArray(W, H, Annot, Png);
        FFileHelper::SaveArrayToFile(Png, *FPaths::Combine(Dir, Base + TEXT("_annotated.png")));
    }

    // Expected frame: yellow boxes/crosses mark alive ground-truth persons
    // projected into this capture. This is a benchmark diagnostic only; it is
    // never fed back into the server belief path.
    {
        TArray<FColor> Expected = Pixels;
        for (FColor& P : Expected) { P.A = 255; }

        int32 ExpectedMarkersDrawn = 0;
        if (HumanManager && Capture && GridWidth > 0 && GridHeight > 0 && CellSizeUU > 0.0f)
        {
            const TArray<FHumanSnapshot>& Snapshots = HumanManager->GetCurrentSnapshots();
            const FColor ExpectedColor(255, 220, 30, 255);
            const float MarkerHeightUU = 100.0f;

            auto WorldAtCell = [&](float CellX, float CellY, float ZOffsetUU) -> FVector
            {
                const float GroundZ = TerrainActor
                    ? TerrainActor->GetHeightAtCell(CellX, CellY)
                    : GridOriginWorld.Z;
                return FVector(
                    GridOriginWorld.X + CellX * CellSizeUU,
                    GridOriginWorld.Y + CellY * CellSizeUU,
                    GroundZ + ZOffsetUU);
            };

            for (const FHumanSnapshot& S : Snapshots)
            {
                if (S.PersonId == INDEX_NONE || S.State == EHumanState::Dead)
                {
                    continue;
                }

                float U = 0.0f, V = 0.0f;
                const FVector MarkerWorld = WorldAtCell(S.PosCell.X, S.PosCell.Y, MarkerHeightUU);
                if (!ProjectWorldPointToImage(Capture, MarkerWorld, U, V))
                {
                    continue;
                }

                const int32 PX = FMath::Clamp(FMath::RoundToInt(U * static_cast<float>(W - 1)), 0, W - 1);
                const int32 PY = FMath::Clamp(FMath::RoundToInt(V * static_cast<float>(H - 1)), 0, H - 1);
                DrawCrossBGRA(Expected, W, H, PX, PY, FMath::Clamp(FMath::Min(W, H) / 45, 8, 18), ExpectedColor, 3);
                ++ExpectedMarkersDrawn;

                const int32 CellX = FMath::Clamp(FMath::FloorToInt(S.PosCell.X), 0, GridWidth - 1);
                const int32 CellY = FMath::Clamp(FMath::FloorToInt(S.PosCell.Y), 0, GridHeight - 1);
                const float CornersX[4] = {
                    static_cast<float>(CellX), static_cast<float>(CellX + 1),
                    static_cast<float>(CellX), static_cast<float>(CellX + 1)
                };
                const float CornersY[4] = {
                    static_cast<float>(CellY), static_cast<float>(CellY),
                    static_cast<float>(CellY + 1), static_cast<float>(CellY + 1)
                };

                int32 MinPX = W, MinPY = H, MaxPX = -1, MaxPY = -1;
                for (int32 C = 0; C < 4; ++C)
                {
                    float CornerU = 0.0f, CornerV = 0.0f;
                    if (!ProjectWorldPointToImage(Capture, WorldAtCell(CornersX[C], CornersY[C], 0.0f), CornerU, CornerV))
                    {
                        continue;
                    }
                    const int32 CPX = FMath::Clamp(FMath::RoundToInt(CornerU * static_cast<float>(W - 1)), 0, W - 1);
                    const int32 CPY = FMath::Clamp(FMath::RoundToInt(CornerV * static_cast<float>(H - 1)), 0, H - 1);
                    MinPX = FMath::Min(MinPX, CPX);
                    MinPY = FMath::Min(MinPY, CPY);
                    MaxPX = FMath::Max(MaxPX, CPX);
                    MaxPY = FMath::Max(MaxPY, CPY);
                }
                if (MaxPX >= MinPX && MaxPY >= MinPY)
                {
                    DrawRectBGRA(Expected, W, H, MinPX, MinPY, MaxPX, MaxPY, ExpectedColor, 2);
                }
            }
        }
        if (ExpectedMarkersDrawn == 0)
        {
            const FColor ExpectedColor(255, 220, 30, 255);
            DrawRectBGRA(Expected, W, H, 1, 1, W - 2, H - 2, ExpectedColor, 4);
            const int32 Radius = FMath::Clamp(FMath::Min(W, H) / 36, 10, 24);
            const int32 Pad = FMath::Clamp(FMath::Min(W, H) / 45, 8, 18);
            DrawCrossBGRA(Expected, W, H, W - Pad - Radius, Pad + Radius, Radius, ExpectedColor, 3);
        }

        TArray64<uint8> Png;
        FImageUtils::PNGCompressImageArray(W, H, Expected, Png);
        FFileHelper::SaveArrayToFile(Png, *FPaths::Combine(Dir, Base + TEXT("_expected.png")));
    }
}

bool ASimulationManager::ProcessVisionCaptures()
{
    if (!bEnableVisionBridge || !bEnableDroneCaptures)
    {
        return true;
    }

    if (!VisionBridge.IsValid())
    {
        VisionBridge = MakeUnique<FVisionBridge>();
        const FString CacheDir = VisionCacheDirName.IsEmpty()
            ? FString::Printf(TEXT("seed%d_%dx%d"), FireGeneratorSeed, GridWidth, GridHeight)
            : VisionCacheDirName;
        VisionBridge->Configure(VisionServerHost, VisionServerPort, VisionCacheMode, CacheDir);
    }

    if (VisionProcessedTick.Num() < RegisteredDrones.Num())
    {
        VisionProcessedTick.Init(-1, RegisteredDrones.Num());
    }

    for (int32 i = 0; i < RegisteredDrones.Num(); ++i)
    {
        ADrone* D = RegisteredDrones[i].Get();
        if (!D) continue;
        if (!IsDroneVisionCameraEnabled(i) || !D->HasActiveCapture()) continue;

        UDroneCaptureComponent* Cap = D->GetCaptureComponent();
        if (!Cap) continue;

        const int32 CapTick = Cap->GetLastCaptureTick();
        if (CapTick < 0 || VisionProcessedTick[i] >= CapTick)
        {
            continue;   // nothing new for this drone
        }

        // Pixels only exist with live readback (Record/Off). Replay has none, but
        // classification/counting still runs from the cached detections.
        TArray<FColor> OwnedPixels;
        const TArray<FColor>* PixelBuffer = &OwnedPixels;
        int32 W = 0, H = 0;
        TArray<FVisionDetection> Detections;
        if (VisionBridge->GetCacheMode() == EVisionCacheMode::Replay)
        {
            static const TArray<FColor> NoPixels;
            if (!VisionBridge->Detect(i, CapTick, 0, 0, NoPixels, Detections)) return false;
        }
        else
        {
            int32 SrcTick = -1;
            const double ReadbackStart = bRecordWallClockTiming ? FPlatformTime::Seconds() : 0.0;
            const bool bLegacyConsume = FParse::Param(FCommandLine::Get(), TEXT("ThesisLegacyCaptureConsume"));
            const bool bRead = bLegacyConsume
                ? Cap->ReadLastCapture(OwnedPixels, W, H, SrcTick)
                : Cap->ReadLastCaptureView(PixelBuffer, W, H, SrcTick);
            if (!bRead)
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis vision: capture readback failed for drone %d tick %d."), i, CapTick);
                return false;
            }
            RecordWallClockPhaseTiming(CurrentTick, TEXT("camera.consume_rgb_readback"), ReadbackStart);
            const double InferenceStart = bRecordWallClockTiming ? FPlatformTime::Seconds() : 0.0;
            if (SrcTick != CapTick || !VisionBridge->Detect(i, CapTick, W, H, *PixelBuffer, Detections)) return false;
            RecordWallClockPhaseTiming(CurrentTick, TEXT("vision.detect_request"), InferenceStart);
            if (bRecordWallClockTiming)
            {
                const FVisionRequestTimings& Timings = VisionBridge->GetLastRequestTimings();
                MetricsRecorder.RecordPhaseTiming(CurrentTick, TEXT("vision.transport_send"), Timings.SendSeconds * 1000.0);
                MetricsRecorder.RecordPhaseTiming(CurrentTick, TEXT("vision.response_wait"), Timings.ReceiveSeconds * 1000.0);
                MetricsRecorder.RecordPhaseTiming(CurrentTick, TEXT("vision.parse_and_cache"), Timings.ParseAndCacheSeconds * 1000.0);
            }
        }
        const TArray<FColor>& Pixels = *PixelBuffer;
        VisionProcessedTick[i] = CapTick;
        RecordCameraConsumeAudit(D, Cap, CapTick, CurrentTick,
            VisionBridge->GetCacheMode() == EVisionCacheMode::Replay,
            VisionBridge->GetCacheMode() == EVisionCacheMode::Replay ? nullptr : &Pixels, W, H);

        ChanceNullScorer::FFrame ChanceNullFrame;
        const bool bCollectChanceNullFrame = bEnableCachedDetectionChanceNull;
        if (bCollectChanceNullFrame)
        {
            ChanceNullFrame.CameraId = i;
            ChanceNullFrame.CaptureTick = CapTick;
            ChanceNullFrame.ProcessingTick = CurrentTick;
            ChanceNullFrame.ProcessingCameraTransform = Cap->GetComponentTransform();
            ChanceNullFrame.ProcessingCameraFOV = Cap->FOVAngle;
            ChanceNullFrame.Detections = Detections;
        }

        // Classify every detection against ground truth.
        TArray<FObservation> VisionObs;
        VisionObs.Reserve(Detections.Num());
        TArray<FVisionDetection> PersonDets;
        TArray<bool> IsTruePositive;
        // Measurement only: keep one-to-one matches out of observation creation.
        // This operational metric is geometric, not an opaque-visible IoU metric.
        struct FMetricPerson { int32 Id; int32 Cell; };
        TArray<FMetricPerson> MetricPersons;
        if (HumanManager)
        {
            for (const FHumanSnapshot& Snapshot : HumanManager->GetCurrentSnapshots())
            {
                if (Snapshot.PersonId == INDEX_NONE || Snapshot.State == EHumanState::Dead) continue;
                const FVector Anchor(GridOriginWorld.X + Snapshot.PosCell.X * CellSizeUU,
                    GridOriginWorld.Y + Snapshot.PosCell.Y * CellSizeUU,
                    (TerrainActor ? TerrainActor->GetHeightAtCell(Snapshot.PosCell.X, Snapshot.PosCell.Y) : GridOriginWorld.Z) + 100.0f);
                float U = 0.0f, V = 0.0f;
                if (!ProjectWorldPointToImage(Cap, Anchor, U, V)) continue;
                const int32 X = FMath::Clamp(FMath::FloorToInt(Snapshot.PosCell.X), 0, GridWidth - 1);
                const int32 Y = FMath::Clamp(FMath::FloorToInt(Snapshot.PosCell.Y), 0, GridHeight - 1);
                MetricPersons.Add({Snapshot.PersonId, Y * GridWidth + X});
            }
        }
        MetricPersons.Sort([](const FMetricPerson& A, const FMetricPerson& B) { return A.Id < B.Id; });
        if (bCollectChanceNullFrame)
        {
            ChanceNullFrame.VisiblePeople.Reserve(MetricPersons.Num());
            for (const FMetricPerson& Person : MetricPersons)
            {
                ChanceNullScorer::FPerson& SavedPerson = ChanceNullFrame.VisiblePeople.AddDefaulted_GetRef();
                SavedPerson.Id = Person.Id;
                SavedPerson.Cell = Person.Cell;
                SavedPerson.AppearanceTick = HumanManager ? HumanManager->GetAppearTick(Person.Id) : INDEX_NONE;
            }
        }
        TArray<int32> MetricPersonCells, MetricDetectionCells;
        TArray<float> MetricDetectionConfidence;
        TArray<bool> MetricDetectionOffGrid;
        for (const FMetricPerson& Person : MetricPersons) MetricPersonCells.Add(Person.Cell);
        for (const FVisionDetection& Det : Detections)
        {
            const bool bPersonClaim = Det.ClassId == 0 && Det.Confidence >= YoloPersonConfidenceThreshold;
            bool bOffGrid = false;
            const float U = (Det.X0 + Det.X1) * 0.5f;
            const float V = Det.Y1;
            const int32 Cell = bPersonClaim ? ProjectImagePointToCell(Cap, U, V) : INDEX_NONE;
            if (bCollectChanceNullFrame && bPersonClaim)
            {
                const int32 ReplayCell = ProjectImagePointToCell(
                    ChanceNullFrame.ProcessingCameraTransform, ChanceNullFrame.ProcessingCameraFOV, U, V, bOffGrid);
                if (ReplayCell != Cell)
                {
                    bCachedDetectionNullProjectionMismatch = true;
                    UE_LOG(LogTemp, Error,
                           TEXT("Chance-null projection parity failed: camera %d capture %d processing %d detection %d original=%d replay=%d."),
                           i, CapTick, CurrentTick, MetricDetectionCells.Num(), Cell, ReplayCell);
                }
            }
            MetricDetectionCells.Add(Cell);
            MetricDetectionOffGrid.Add(bOffGrid);
            MetricDetectionConfidence.Add(Det.Confidence);
        }
        const TArray<int32> MetricMatchedPersonIndex = YoloGeometricMetrics::MatchPersonIndices(
            MetricDetectionCells, MetricDetectionConfidence, MetricPersonCells, GridWidth);
        int32 FrameTP = 0, FrameFP = 0, FrameNonPerson = 0;
        for (int32 DetectionIndex = 0; DetectionIndex < Detections.Num(); ++DetectionIndex)
        {
            const FVisionDetection& Det = Detections[DetectionIndex];
            const bool bPersonClaim = (Det.ClassId == 0) && (Det.Confidence >= YoloPersonConfidenceThreshold);
            if (!bPersonClaim)
            {
                ++FrameNonPerson;
                continue;
            }
            const int32 Cell = MetricDetectionCells[DetectionIndex];
            const int32 MatchedPersonIndex = MetricMatchedPersonIndex[DetectionIndex];
            const bool bTruePositive = MatchedPersonIndex != INDEX_NONE;
            if (bTruePositive) { ++FrameTP; } else { ++FrameFP; }
            PersonDets.Add(Det);
            IsTruePositive.Add(bTruePositive);

            if (bTruePositive && MetricsRecorder.IsActive())
            {
                const FMetricPerson& MatchedPerson = MetricPersons[MatchedPersonIndex];
                MetricsRecorder.RecordPersonGeometricMatch(
                    CurrentTick, CapTick, i, MatchedPerson.Id, DetectionIndex, Det.Confidence,
                    Cell, MatchedPerson.Cell);
            }

            if (Cell != INDEX_NONE)
            {
                FObservation O;
                O.CellIndex = Cell;
                O.Tick = CapTick;
                O.FireLogOddsUpdate = 0.0f;
                // Confidence-scaled positive LLR; calibrated against the perfect
                // sensor's PersonPositiveLLR so belief dynamics stay comparable.
                O.bHasPersonEvidence = true;
                O.PersonLogOddsUpdate = SensorPersonPositiveLLR * FMath::Clamp(Det.Confidence, 0.0f, 1.0f);
                VisionObs.Add(O);
            }
        }

        if (bCollectChanceNullFrame)
        {
            ChanceNullFrame.OriginalDetectionCells = MetricDetectionCells;
            ChanceNullFrame.OriginalMatchedPersonIds.Init(INDEX_NONE, MetricMatchedPersonIndex.Num());
            for (int32 DetectionIndex = 0; DetectionIndex < MetricMatchedPersonIndex.Num(); ++DetectionIndex)
            {
                const int32 PersonIndex = MetricMatchedPersonIndex[DetectionIndex];
                if (PersonIndex != INDEX_NONE)
                {
                    ChanceNullFrame.OriginalMatchedPersonIds[DetectionIndex] = MetricPersons[PersonIndex].Id;
                }
            }
            ChanceNullFrame.OriginalTruePositives = FrameTP;
            ChanceNullFrame.OriginalFalsePositives = FrameFP;
            ChanceNullFrame.OriginalPersonClaimCount = PersonDets.Num();
            ChanceNullFrame.OriginalNonPerson = FrameNonPerson;
            ChanceNullFrame.OriginalMissed = ChanceNullFrame.VisiblePeople.Num() - FrameTP;
            for (const bool bOffGrid : MetricDetectionOffGrid)
            {
                ChanceNullFrame.OriginalOffGridRejections += bOffGrid ? 1 : 0;
            }
            if (CachedDetectionNullFrames.Num() <= i)
            {
                CachedDetectionNullFrames.SetNum(i + 1);
            }
            CachedDetectionNullFrames[i].Add(MoveTemp(ChanceNullFrame));
        }

        // Ground-truth persons within the camera footprint (misses + fire check).
        TArray<int32> FootprintCells;
        CollectCaptureFootprint(Cap, FootprintCells);
        const int32 GtPersonsInView = MetricPersons.Num();
        const int32 FrameMissed = GtPersonsInView - FrameTP;

        // Diagnostic: shows whether the server returned anything, whether people
        // were actually in view, and whether live pixels were available. If this
        // logs "0 raw det" with "GT-in-view > 0", YOLO saw the frame but found no
        // person (image too bright/small, wrong backend, or no human meshes). If
        // "pixels 0", you are in replay (no images possible).
        UE_LOG(LogTemp, Log,
               TEXT("Thesis YOLO: drone %d tick %d - %d raw det, %d person-claim (TP %d, FP %d), %d nonperson, GT-in-view %d, pixels %d"),
               i, CapTick, Detections.Num(), PersonDets.Num(), FrameTP, FrameFP, FrameNonPerson, GtPersonsInView, Pixels.Num());

        if (MetricsRecorder.IsActive())
        {
            MetricsRecorder.RecordYoloFrame(FrameTP, FrameFP, FrameMissed, GtPersonsInView, FrameNonPerson, CapTick, i);
        }

        // Save example images — only possible when live pixels were read back.
        if (bSaveYoloDebugImages && Pixels.Num() > 0 && W > 0 && H > 0)
        {
            const bool bMissedFrame = (GtPersonsInView > 0 && PersonDets.Num() == 0); // total miss: people in view, no YOLO person boxes
            const bool bPartialMissFrame = (GtPersonsInView > FrameTP && FrameTP > 0);
            const bool bFalsePositiveFrame = (FrameFP > 0);
            if (bMissedFrame || bPartialMissFrame || bFalsePositiveFrame)
            {
                if (YoloMaxErrorImages <= 0 || YoloErrorImagePairsSaved < YoloMaxErrorImages)
                {
                    const TCHAR* ErrorTag = bFalsePositiveFrame
                        ? TEXT("false_positive")
                        : (bMissedFrame ? TEXT("missed") : TEXT("partial_miss"));
                    SaveYoloImagePair(ErrorTag,
                                      i, CapTick, W, H, Pixels, PersonDets, IsTruePositive, Cap);
                    ++YoloErrorImagePairsSaved;
                }
            }
            if (FrameTP > 0 && YoloMaxCorrectImages > 0 && YoloCorrectImagePairsSaved < YoloMaxCorrectImages)
            {
                SaveYoloImagePair(TEXT("correct"), i, CapTick, W, H, Pixels, PersonDets, IsTruePositive, Cap);
                ++YoloCorrectImagePairsSaved;
            }
        }

        if (VisionObs.Num() > 0)
        {
            // Server belief: ingested next tick with the regular pipeline.
            PendingObservations.Append(VisionObs);
            // Capturing drone's local belief: applied now (server slot).
            D->ApplyExternalObservations(VisionObs, CurrentTick);
        }
    }
    return true;
}

bool ASimulationManager::WriteCachedDetectionChanceNull()
{
    if (!bEnableCachedDetectionChanceNull)
    {
        return true;
    }
    if (VisionCacheMode != EVisionCacheMode::Replay || !MetricsRecorder.IsActive() || CachedDetectionNullFrames.IsEmpty())
    {
        UE_LOG(LogTemp, Error, TEXT("Chance-null output requires recorded Replay frames and an active metrics run."));
        return false;
    }
    if (bCachedDetectionNullProjectionMismatch)
    {
        UE_LOG(LogTemp, Error, TEXT("Chance-null had a processing-time camera projection mismatch; refusing null output."));
        return false;
    }

    struct FCameraScores
    {
        int32 CameraId = INDEX_NONE;
        TArray<ChanceNullScorer::FFrame> Frames;
        TArray<ChanceNullScorer::FOffset> Offsets;
        TMap<int32, TArray<ChanceNullScorer::FFrameScore>> ScoresByFraction;
    };
    TArray<FCameraScores> Cameras;
    TArray<int32> CommonFractions;
    for (int32 CameraId = 0; CameraId < CachedDetectionNullFrames.Num(); ++CameraId)
    {
        if (CachedDetectionNullFrames[CameraId].IsEmpty())
        {
            continue;
        }
        FCameraScores& Camera = Cameras.AddDefaulted_GetRef();
        Camera.CameraId = CameraId;
        Camera.Frames = CachedDetectionNullFrames[CameraId];
        Camera.Offsets = ChanceNullScorer::BuildOffsets(Camera.Frames.Num(), bCachedDetectionChanceNullPilot);
        TArray<int32> Fractions;
        for (const ChanceNullScorer::FOffset& Offset : Camera.Offsets)
        {
            Fractions.Add(Offset.FractionPercent);
        }
        if (CommonFractions.IsEmpty())
        {
            CommonFractions = Fractions;
        }
        else if (Fractions != CommonFractions)
        {
            UE_LOG(LogTemp, Error,
                   TEXT("Chance-null cameras have different supported fraction ranks; refusing partial all-camera replicate coverage."));
            return false;
        }
    }
    if (Cameras.Num() != NumRealCameraDrones)
    {
        UE_LOG(LogTemp, Error,
               TEXT("Chance-null collected %d camera streams; expected %d. Refusing partial camera coverage."),
               Cameras.Num(), NumRealCameraDrones);
        return false;
    }
    for (int32 ExpectedCameraId = 0; ExpectedCameraId < NumRealCameraDrones; ++ExpectedCameraId)
    {
        if (!Cameras.ContainsByPredicate([ExpectedCameraId](const FCameraScores& Camera)
            { return Camera.CameraId == ExpectedCameraId; }))
        {
            UE_LOG(LogTemp, Error, TEXT("Chance-null is missing expected camera stream %d."), ExpectedCameraId);
            return false;
        }
    }
    if (CachedDetectionChanceNullCohort.IsEmpty() ||
        FPaths::GetCleanFilename(CachedDetectionChanceNullCohort) != CachedDetectionChanceNullCohort ||
        CachedDetectionChanceNullCohort.Contains(TEXT("..")))
    {
        UE_LOG(LogTemp, Error, TEXT("Chance-null cohort must be a nonempty path-safe identifier."));
        return false;
    }
    if (Cameras.IsEmpty() || CommonFractions.Num() < 2 || CommonFractions[0] != 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Chance-null has no usable camera frame sequence or no nonzero offset support."));
        return false;
    }

    // Validate the entire zero-shift replay before any output is written.
    for (FCameraScores& Camera : Cameras)
    {
        const ChanceNullScorer::FOffset* ZeroOffset = Camera.Offsets.FindByPredicate(
            [](const ChanceNullScorer::FOffset& Offset) { return Offset.FractionPercent == 0; });
        if (!ZeroOffset || ZeroOffset->Frames != 0)
        {
            UE_LOG(LogTemp, Error, TEXT("Chance-null camera %d is missing its zero-shift baseline."), Camera.CameraId);
            return false;
        }
        TArray<ChanceNullScorer::FFrameScore>& ZeroScores = Camera.ScoresByFraction.Add(0);
        ZeroScores.Reserve(Camera.Frames.Num());
        for (int32 DestinationIndex = 0; DestinationIndex < Camera.Frames.Num(); ++DestinationIndex)
        {
            const ChanceNullScorer::FFrame& Destination = Camera.Frames[DestinationIndex];
            const ChanceNullScorer::FFrame& Source = Camera.Frames[DestinationIndex];
            const auto Project = [this, &Destination](const FVisionDetection& Detection, bool& bOffGrid)
            {
                return ProjectImagePointToCell(Destination.ProcessingCameraTransform,
                    Destination.ProcessingCameraFOV,
                    (Detection.X0 + Detection.X1) * 0.5f, Detection.Y1, bOffGrid);
            };
            ChanceNullScorer::FFrameScore Score = ChanceNullScorer::ScoreFrame(
                Destination, Source, 0, GridWidth, YoloPersonConfidenceThreshold, Project);
            Score.ReplicateIndex = 0;
            if (!ChanceNullScorer::MatchesOriginal(Destination, Score))
            {
                UE_LOG(LogTemp, Error,
                       TEXT("Chance-null zero-shift gate failed for camera %d capture %d processing %d; no null files written."),
                       Camera.CameraId, Destination.CaptureTick, Destination.ProcessingTick);
                return false;
            }
            ZeroScores.Add(MoveTemp(Score));
        }
    }

    // Score every supported fraction using each camera's own actual processed-frame count.
    for (FCameraScores& Camera : Cameras)
    {
        for (const ChanceNullScorer::FOffset& Offset : Camera.Offsets)
        {
            if (Offset.FractionPercent == 0)
            {
                continue;
            }
            TArray<ChanceNullScorer::FFrameScore>& Scores = Camera.ScoresByFraction.Add(Offset.FractionPercent);
            Scores.Reserve(Camera.Frames.Num());
            for (int32 DestinationIndex = 0; DestinationIndex < Camera.Frames.Num(); ++DestinationIndex)
            {
                const ChanceNullScorer::FFrame& Destination = Camera.Frames[DestinationIndex];
                const int32 SourceIndex = (DestinationIndex + Offset.Frames) % Camera.Frames.Num();
                const ChanceNullScorer::FFrame& Source = Camera.Frames[SourceIndex];
                const auto Project = [this, &Destination](const FVisionDetection& Detection, bool& bOffGrid)
                {
                    return ProjectImagePointToCell(Destination.ProcessingCameraTransform,
                        Destination.ProcessingCameraFOV,
                        (Detection.X0 + Detection.X1) * 0.5f, Detection.Y1, bOffGrid);
                };
                ChanceNullScorer::FFrameScore Score = ChanceNullScorer::ScoreFrame(
                    Destination, Source, Offset.Frames, GridWidth, YoloPersonConfidenceThreshold, Project);
                Score.ReplicateIndex = Offset.FractionPercent;
                Scores.Add(MoveTemp(Score));
            }
        }
    }

    TArray<ChanceNullScorer::FPerson> AllPeople;
    if (HumanManager)
    {
        AllPeople.Reserve(HumanManager->GetNumPersons());
        for (int32 PersonId = 0; PersonId < HumanManager->GetNumPersons(); ++PersonId)
        {
            ChanceNullScorer::FPerson& Person = AllPeople.AddDefaulted_GetRef();
            Person.Id = PersonId;
            Person.AppearanceTick = HumanManager->GetAppearTick(PersonId);
        }
    }
    if (AllPeople.IsEmpty())
    {
        UE_LOG(LogTemp, Error, TEXT("Chance-null requires the loaded scenario person roster."));
        return false;
    }

    FString FrameCsv = TEXT("run_id,cohort,camera_id,fraction_percent,offset_frames,capture_tick,processing_tick,source_capture_tick,detection_count,person_claim_count,true_positives,false_positives,persons_in_view,off_grid_rejections\n");
    FString PersonCsv = TEXT("run_id,cohort,fraction_percent,person_id,appearance_tick,first_match_processing_tick,match_count\n");
    FString EventCsv = TEXT("run_id,cohort,camera_id,fraction_percent,offset_frames,capture_tick,processing_tick,detection_index,confidence,detection_cell,person_id,person_cell\n");
    FString SupportCsv = TEXT("run_id,cohort,camera_id,processed_frame_count,fraction_percent,offset_frames,min_circular_lag_frames,nominal_lag_ticks,pilot\n");
    int32 TotalFrameRows = 0;
    const FString RunId = MetricsRunId.IsEmpty()
        ? FPaths::GetCleanFilename(MetricsRecorder.GetRunDirectory())
        : MetricsRunId;
    const FString CsvRunId = RunId.Replace(TEXT("\""), TEXT("\"\""));

    for (const int32 FractionPercent : CommonFractions)
    {
        TArray<ChanceNullScorer::FFrame> CombinedFrames;
        TArray<ChanceNullScorer::FFrameScore> CombinedScores;
        for (const FCameraScores& Camera : Cameras)
        {
            const TArray<ChanceNullScorer::FFrameScore>* CameraScores = Camera.ScoresByFraction.Find(FractionPercent);
            if (!CameraScores || CameraScores->Num() != Camera.Frames.Num())
            {
                UE_LOG(LogTemp, Error, TEXT("Chance-null output has incomplete camera coverage at fraction %d."), FractionPercent);
                return false;
            }
            const ChanceNullScorer::FOffset* Offset = Camera.Offsets.FindByPredicate(
                [FractionPercent](const ChanceNullScorer::FOffset& Entry) { return Entry.FractionPercent == FractionPercent; });
            if (!Offset)
            {
                return false;
            }
            const int32 MinCircularLagFrames = FMath::Min(Offset->Frames, Camera.Frames.Num() - Offset->Frames);
            SupportCsv += FString::Printf(TEXT("\"%s\",\"%s\",%d,%d,%d,%d,%d,%d,%d\n"),
                *CsvRunId, *CachedDetectionChanceNullCohort, Camera.CameraId, Camera.Frames.Num(),
                FractionPercent, Offset->Frames, MinCircularLagFrames,
                MinCircularLagFrames * DroneCaptureEveryNTicks, bCachedDetectionChanceNullPilot ? 1 : 0);
            for (int32 FrameIndex = 0; FrameIndex < Camera.Frames.Num(); ++FrameIndex)
            {
                const ChanceNullScorer::FFrame& Frame = Camera.Frames[FrameIndex];
                const ChanceNullScorer::FFrameScore& Score = (*CameraScores)[FrameIndex];
                const int32 SourceIndex = (FrameIndex + Offset->Frames) % Camera.Frames.Num();
                FrameCsv += FString::Printf(TEXT("\"%s\",\"%s\",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n"),
                    *CsvRunId, *CachedDetectionChanceNullCohort, Camera.CameraId, FractionPercent, Offset->Frames,
                    Score.CaptureTick, Score.ProcessingTick, Camera.Frames[SourceIndex].CaptureTick, Score.DetectionCount,
                    Score.PersonClaimCount, Score.TruePositives, Score.FalsePositives,
                    Score.PersonsInView, Score.OffGridRejections);
                CombinedFrames.Add(Frame);
                CombinedScores.Add(Score);
                for (int32 DetectionIndex = 0; DetectionIndex < Score.MatchedPersonIds.Num(); ++DetectionIndex)
                {
                    const int32 PersonId = Score.MatchedPersonIds[DetectionIndex];
                    if (PersonId == INDEX_NONE) continue;
                    const ChanceNullScorer::FPerson* MatchedPerson = Frame.VisiblePeople.FindByPredicate(
                        [PersonId](const ChanceNullScorer::FPerson& Candidate) { return Candidate.Id == PersonId; });
                    if (!MatchedPerson)
                    {
                        UE_LOG(LogTemp, Error, TEXT("Chance-null matched person %d is absent from destination snapshot."), PersonId);
                        return false;
                    }
                    EventCsv += FString::Printf(TEXT("\"%s\",\"%s\",%d,%d,%d,%d,%d,%d,%.8f,%d,%d,%d\n"),
                        *CsvRunId, *CachedDetectionChanceNullCohort, Camera.CameraId, FractionPercent,
                        Offset->Frames, Score.CaptureTick, Score.ProcessingTick, DetectionIndex,
                        Score.SourceDetections[DetectionIndex].Confidence, Score.DetectionCells[DetectionIndex],
                        PersonId, MatchedPerson->Cell);
                }
                ++TotalFrameRows;
            }
        }
        // Fixed order makes simultaneous multi-camera matches deterministic.
        TArray<int32> PairOrder;
        PairOrder.SetNum(CombinedFrames.Num());
        for (int32 Index = 0; Index < PairOrder.Num(); ++Index) PairOrder[Index] = Index;
        PairOrder.Sort([&CombinedFrames](int32 A, int32 B)
        {
            const ChanceNullScorer::FFrame& Left = CombinedFrames[A];
            const ChanceNullScorer::FFrame& Right = CombinedFrames[B];
            return Left.ProcessingTick == Right.ProcessingTick
                ? Left.CameraId < Right.CameraId
                : Left.ProcessingTick < Right.ProcessingTick;
        });
        TArray<ChanceNullScorer::FFrame> SortedFrames;
        TArray<ChanceNullScorer::FFrameScore> SortedScores;
        SortedFrames.Reserve(PairOrder.Num());
        SortedScores.Reserve(PairOrder.Num());
        for (const int32 Index : PairOrder)
        {
            SortedFrames.Add(MoveTemp(CombinedFrames[Index]));
            SortedScores.Add(MoveTemp(CombinedScores[Index]));
        }
        TArray<ChanceNullScorer::FPersonScore> PersonScores;
        if (!ChanceNullScorer::AggregatePeople(SortedFrames, SortedScores, AllPeople, PersonScores))
        {
            UE_LOG(LogTemp, Error, TEXT("Chance-null person aggregation failed for fraction %d."), FractionPercent);
            return false;
        }
        for (const ChanceNullScorer::FPersonScore& Person : PersonScores)
        {
            PersonCsv += FString::Printf(TEXT("\"%s\",\"%s\",%d,%d,%d,%d,%d\n"), *CsvRunId,
                *CachedDetectionChanceNullCohort, FractionPercent, Person.PersonId, Person.AppearanceTick,
                Person.FirstMatchProcessingTick, Person.MatchCount);
        }
    }

    const FString ControlsDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"), TEXT("supervisor-controls"));
    const FString OutputDir = FPaths::Combine(ControlsDir, TEXT("chance-null"), CachedDetectionChanceNullCohort, RunId);
    IFileManager::Get().MakeDirectory(*OutputDir, true);
    const FString FramePath = FPaths::Combine(OutputDir, TEXT("chance_null_frame_counts.csv"));
    const FString PersonPath = FPaths::Combine(OutputDir, TEXT("chance_null_person_matches.csv"));
    const FString SupportPath = FPaths::Combine(OutputDir, TEXT("chance_null_shift_support.csv"));
    const FString EventPath = FPaths::Combine(OutputDir, TEXT("chance_null_match_events.csv"));
    if (!FFileHelper::SaveStringToFile(FrameCsv, *FramePath) ||
        !FFileHelper::SaveStringToFile(PersonCsv, *PersonPath) ||
        !FFileHelper::SaveStringToFile(SupportCsv, *SupportPath) ||
        !FFileHelper::SaveStringToFile(EventCsv, *EventPath))
    {
        UE_LOG(LogTemp, Error, TEXT("Chance-null failed to save sidecar outputs under %s."), *OutputDir);
        return false;
    }
    UE_LOG(LogTemp, Display, TEXT("Chance-null wrote %d cameras, %d fraction ranks, %d processed frames to %s."),
        Cameras.Num(), CommonFractions.Num(), TotalFrameRows, *OutputDir);
    return true;
}

void ASimulationManager::RecordTickMetrics(float TickComputeMs)
{
    if (!MetricsRecorder.IsActive())
    {
        return;
    }

    TArray<FDroneTickSample> Samples;
    Samples.Reserve(RegisteredDrones.Num());
    for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
    {
        const ADrone* D = Drone.Get();
        if (!D) continue;

        FDroneTickSample S;
        S.DroneIndex = D->GetLogicalDroneId();
        const FVector W = D->GetActorLocation();
        S.PosCellX = static_cast<float>((W.X - GridOriginWorld.X) / CellSizeUU);
        S.PosCellY = static_cast<float>((W.Y - GridOriginWorld.Y) / CellSizeUU);
        S.TargetCellIndex = D->GetTargetCellIndex();
        S.TargetScore = D->GetCurrentTargetScore();
        S.OwnedCellCount = D->GetOwnedCellCount();
        Samples.Add(S);
    }

    // PendingObservations holds exactly this tick's new observations here
    // (collected from the drones above, ingested next tick).
    int32 NumPositiveFire = 0;
    int32 NumPositivePerson = 0;
    for (const FObservation& O : PendingObservations)
    {
        if (O.FireLogOddsUpdate > 0.0f) ++NumPositiveFire;
        if (O.PersonLogOddsUpdate > 0.0f) ++NumPositivePerson;
    }

    MetricsRecorder.RecordTick(CurrentTick, Samples,
                               PendingObservations.Num(),
                               NumPositiveFire, NumPositivePerson,
                               TickComputeMs);
}

void ASimulationManager::UpdatePersonDetectionMetrics()
{
    if (!HumanManager || !WorldGrid.IsValid())
    {
        return;
    }

    const TArray<FHumanSnapshot>& Snapshots = HumanManager->GetCurrentSnapshots();
    if (PersonFirstDetectedTick.Num() < HumanManager->GetNumPersons())
    {
        PersonFirstDetectedTick.Init(-1, HumanManager->GetNumPersons());
    }

    for (const FHumanSnapshot& S : Snapshots)
    {
        if (S.PersonId == INDEX_NONE ||
            !PersonFirstDetectedTick.IsValidIndex(S.PersonId))
        {
            continue;
        }

        const int32 X = FMath::Clamp(static_cast<int32>(S.PosCell.X), 0, GridWidth - 1);
        const int32 Y = FMath::Clamp(static_cast<int32>(S.PosCell.Y), 0, GridHeight - 1);
        const int32 AppearTick = HumanManager->GetAppearTick(S.PersonId);

        // Detected = positive server person belief at the person's current cell,
        // observed no earlier than the person's appearance. (WorldGrid mirrors
        // the GPU server belief — see the observation-ingest block in Tick.)
        const int32 LastObservedTick = bEnableVisionBridge
            ? WorldGrid->getLastPersonObservedTick(X, Y) : WorldGrid->getLastObservedTick(X, Y);
        const bool bTracked =
            WorldGrid->getPersonBelief(X, Y) > 0.0f &&
            LastObservedTick >= AppearTick &&
            CurrentTick - LastObservedTick <= TrackingWindowTicks;
        const float Danger = IsFireActive(X, Y, CurrentTick) ? 1.0f : 0.0f;

        MetricsRecorder.RecordPersonState(
            CurrentTick,
            S.PersonId,
            AppearTick,
            bTracked,
            Danger,
            DangerExposureThreshold);

        if (PersonFirstDetectedTick[S.PersonId] < 0 && bTracked)
        {
            PersonFirstDetectedTick[S.PersonId] = CurrentTick;
            MetricsRecorder.RecordPersonDetection(CurrentTick, S.PersonId, AppearTick);
        }
    }
}

float ASimulationManager::ComputeMeanBeliefError() const
{
    if (!WorldGrid.IsValid() || GridWidth <= 0 || GridHeight <= 0)
    {
        return 0.0f;
    }

    double ErrorSum = 0.0;
    const int32 NumCells = GridWidth * GridHeight;
    for (int32 Cell = 0; Cell < NumCells; ++Cell)
    {
        const int32 X = Cell % GridWidth;
        const int32 Y = Cell / GridWidth;
        const float FireTruth = IsFireActive(X, Y, CurrentTick) ? 1.0f : 0.0f;
        const float PersonTruth = (((WorldGrid->getRealPriorityAndNumHumans(X, Y) >> 16) & 0xFFFF) > 0) ? 1.0f : 0.0f;
        const float FireBelief = FMath::Clamp(ThesisPriority::PositiveEvidence(WorldGrid->getFireBelief(X, Y)), 0.0f, 1.0f);
        const float PersonBelief = FMath::Clamp(ThesisPriority::PositiveEvidence(WorldGrid->getPersonBelief(X, Y)), 0.0f, 1.0f);
        ErrorSum += 0.5 * (FMath::Abs(FireBelief - FireTruth) + FMath::Abs(PersonBelief - PersonTruth));
    }

    return static_cast<float>(ErrorSum / static_cast<double>(FMath::Max(1, NumCells)));
}

void ASimulationManager::StartSimulation()
{
    const bool bMixedRecoveryRequested = FParse::Param(FCommandLine::Get(), TEXT("ThesisMixedObservationRecovery"));
    if (bEnableCachedDetectionChanceNull && (!bRecordMetrics || !bEnableVisionBridge ||
        VisionCacheMode != EVisionCacheMode::Replay || !bEnableDroneCaptures || NumRealCameraDrones <= 0))
    {
        UE_LOG(LogTemp, Error, TEXT("Cached-detection chance null requires metrics, VisionBridge Replay, captures, and at least one camera drone."));
        State = ESimState::Stopped;
        return;
    }
    if (bMixedRecoveryRequested && (!bRecordMetrics || !bEnableVisionBridge ||
        VisionCacheMode != EVisionCacheMode::Replay || !bEnableDroneCaptures ||
        NumRealCameraDrones != 1 || !CanUseParallelDroneProcessing()))
    {
        UE_LOG(LogTemp, Error, TEXT("Mixed observation recovery requires metrics, VisionBridge Replay, captures, exactly one camera drone, and supported parallel CPU processing."));
        State = ESimState::Stopped;
        return;
    }
    if (bRecordMetrics && !MetricsRecorder.IsActive())
    {
        CachedDetectionNullFrames.Reset();
        bCachedDetectionNullProjectionMismatch = false;
        const FString RunId = MetricsRunId.IsEmpty()
            ? FString::Printf(TEXT("run_%s"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")))
            : MetricsRunId;
        MetricsRecorder.SetDetailMode(
            MetricsDetailMode.Equals(TEXT("summary"), ESearchCase::IgnoreCase)
                ? ESimMetricsDetailMode::Summary
                : ESimMetricsDetailMode::Full);
        MetricsRecorder.BeginRun(RunId, FSimRunConfig::MakeConfigSnapshot(*this), MetricsRunSubdir);
        RecordInitialDroneIdentityMap();
        MetricsRecorder.SetPersonGeometricRecoveryEnabled(
            bEnableVisionBridge && VisionCacheMode == EVisionCacheMode::Replay &&
            (FParse::Param(FCommandLine::Get(), TEXT("ThesisPersonGeometricRecovery")) || bMixedRecoveryRequested ||
             bEnableCachedDetectionChanceNull));
        MetricsRecorder.SetMixedObservationRecoveryEnabled(bMixedRecoveryRequested);
        MetricsRecorder.SetIdealSensorWitnessRecordingEnabled(
            bBenchmarkRecordIdealSensorWitnesses || bMixedRecoveryRequested);
        FlushPendingWallClockPhaseTimings();
        RecordDroneProcessingModeEvent();
        if (bSaveYoloDebugImages)
        {
            IFileManager::Get().DeleteDirectory(
                *FPaths::Combine(MetricsRecorder.GetRunDirectory(), TEXT("vision")),
                /*RequireExists=*/false,
                /*Tree=*/true);
        }
        MetricsRecorder.SetRecallDeadlineTicks(RecallDeadlineTicks);
        YoloErrorImagePairsSaved = 0;
        YoloCorrectImagePairsSaved = 0;
        PersonFirstDetectedTick.Reset();
        if (HumanManager)
        {
            PersonFirstDetectedTick.Init(-1, HumanManager->GetNumPersons());
            MetricsRecorder.SetPersonCount(HumanManager->GetNumPersons());
        }
    }
    BeginCameraStateAudit();
    State = ESimState::Running;
}
void ASimulationManager::PauseSimulation()
{
    if (State == ESimState::Running) State = ESimState::Paused;
}
void ASimulationManager::ResumeSimulation()
{
    if (State == ESimState::Paused) State = ESimState::Running;
}

void ASimulationManager::ToggleSimulationPause()
{
    if (State == ESimState::Running)
    {
        State = ESimState::Paused;
        UE_LOG(LogTemp, Log, TEXT("Thesis simulation paused."));
    }
    else if (State == ESimState::Paused)
    {
        State = ESimState::Running;
        UE_LOG(LogTemp, Log, TEXT("Thesis simulation resumed."));
    }
    else if (State == ESimState::Stopped)
    {
        StartSimulation();
        UE_LOG(LogTemp, Log, TEXT("Thesis simulation started."));
    }
}

void ASimulationManager::CaptureInitialDroneSpawnLocations()
{
    InitialDroneSpawnLocations.Reset();
    InitialDroneSpawnLocations.Reserve(RegisteredDrones.Num());
    for (const TObjectPtr<ADrone>& Drone : RegisteredDrones)
    {
        InitialDroneSpawnLocations.Add(Drone.Get() ? Drone->GetActorLocation() : FVector::ZeroVector);
    }
}

void ASimulationManager::RecordInitialDroneIdentityMap()
{
    if (BenchmarkDroneLogicalIds.Num() == 0 || !MetricsRecorder.IsActive()) return;
    const int32 Count = FMath::Min(RegisteredDrones.Num(), InitialDroneSpawnLocations.Num());
    for (int32 Slot = 0; Slot < Count; ++Slot)
    {
        const ADrone* Drone = RegisteredDrones[Slot].Get();
        if (!Drone) continue;
        const FVector SpawnWorld = Drone->GetActorLocation();
        const float CellX = (SpawnWorld.X - GridOriginWorld.X) / CellSizeUU;
        const float CellY = (SpawnWorld.Y - GridOriginWorld.Y) / CellSizeUU;
        MetricsRecorder.RecordDroneIdentity(Slot, Drone->GetLogicalDroneId(), CellX, CellY,
                                            SpawnWorld, Drone->GetActorRotation());
    }
}

bool ASimulationManager::SwapToNextScenario()
{
    const bool bMixedRecoveryRequested = FParse::Param(FCommandLine::Get(), TEXT("ThesisMixedObservationRecovery"));
    if (bMixedRecoveryRequested && !bRecordMetrics)
    {
        UE_LOG(LogTemp, Error, TEXT("Mixed observation recovery requires metrics recording; stopping batch."));
        State = ESimState::Stopped;
        return false;
    }
    // Only active with an in-process manifest, and only while scenarios remain.
    if (ScenarioManifestPaths.Num() == 0 ||
        CurrentScenarioIndex + 1 >= ScenarioManifestPaths.Num())
    {
        return false;
    }

    ++CurrentScenarioIndex;
    RestoreSensorResources();
    NormalizeEnvironmentLighting();
    const FString NextConfig = ScenarioManifestPaths[CurrentScenarioIndex];
    SimConfigPath.FilePath = NextConfig;

    const int32 PrevGridW = GridWidth;
    const int32 PrevGridH = GridHeight;

    // Apply the next scenario's config (seed, CSV paths, terrain profile, metrics
    // run_id/subdir, tunables). The metrics run for the PREVIOUS scenario was
    // already ended by the caller, so we never EndRun here (no double-finalise).
    if (!FSimRunConfig::LoadAndApply(NextConfig, *this))
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis in-process: refusing to continue after invalid next run config %s."), *NextConfig);
        State = ESimState::Stopped;
        return false;
    }
    LiveRenderPolicy::Update();

    // Same-grid batching invariant: drone spawn layout + GPU texture sizes are
    // constant within a batch. A grid change would invalidate the restored
    // positions, so refuse rather than silently produce non-identical results.
    if (GridWidth != PrevGridW || GridHeight != PrevGridH)
    {
        UE_LOG(LogTemp, Error,
               TEXT("Thesis in-process: scenario %d changes grid %dx%d -> %dx%d; batches MUST share grid size. Aborting batch."),
               CurrentScenarioIndex, PrevGridW, PrevGridH, GridWidth, GridHeight);
        return false;
    }

    if (BenchmarkDroneLogicalIds.Num() > 0)
    {
        if (RegisteredDrones.Num() != AutoDroneCount ||
            BenchmarkDroneLogicalIds.Num() != RegisteredDrones.Num() ||
            BenchmarkDroneSpawnCellPositions.Num() != RegisteredDrones.Num())
        {
            UE_LOG(LogTemp, Error, TEXT("Thesis identity-mapped batch changed active drone count or identity/spawn map; refusing an unmatched continuation."));
            State = ESimState::Stopped;
            return false;
        }
        for (int32 Slot = 0; Slot < RegisteredDrones.Num(); ++Slot)
        {
            const ADrone* Drone = RegisteredDrones[Slot].Get();
            if (!Drone || Drone->GetLogicalDroneId() != BenchmarkDroneLogicalIds[Slot] ||
                !InitialDroneSpawnLocations.IsValidIndex(Slot))
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis identity-mapped batch changed drone identity/order; refusing an unmatched continuation."));
                State = ESimState::Stopped;
                return false;
            }
            const FVector2D Expected = BenchmarkDroneSpawnCellPositions[Slot];
            const FVector OriginalSpawn = InitialDroneSpawnLocations[Slot];
            const float ActualX = (OriginalSpawn.X - GridOriginWorld.X) / CellSizeUU;
            const float ActualY = (OriginalSpawn.Y - GridOriginWorld.Y) / CellSizeUU;
            if (!FMath::IsNearlyEqual(ActualX, Expected.X, 0.001f) ||
                !FMath::IsNearlyEqual(ActualY, Expected.Y, 0.001f))
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis identity-mapped batch changed drone spawn layout; refusing an unmatched continuation."));
                State = ESimState::Stopped;
                return false;
            }
        }
    }
    else
    {
        for (int32 Slot = 0; Slot < RegisteredDrones.Num(); ++Slot)
        {
            const ADrone* Drone = RegisteredDrones[Slot].Get();
            if (Drone && Drone->GetLogicalDroneId() != Slot)
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis in-process: cannot switch from an identity-mapped control to legacy drone IDs without a fresh process."));
                State = ESimState::Stopped;
                return false;
            }
        }
    }

    RefreshMethodSpecFromNames();
    EnsurePartitionStrategy();

    // ---- Re-initialise world state (mirrors ResetSimulation, minus EndRun) ----
    CurrentTick = 0;
    TickAccumulator = 0.0f;
    OracleAssignmentTick = INDEX_NONE;
    OracleCachedKeyframeCount = -1;
    PendingObservations.Reset();
    CachedServerPriority.Reset();
    CachedServerPriorityTick = -1;
    TicksSinceLastPriorityReadback = 9999;

    if (!WorldGrid.IsValid())
    {
        WorldGrid = MakeUnique<Grid>(GridWidth, GridHeight);
    }
    WorldGrid->Initialize();

    // Terrain + forest FIRST: BuildForest writes floraDensity (the fuel map), which
    // GPU InitializeTextures uploads and drones snapshot. Order matches BeginPlay;
    // doing GPU/fire before the forest uploaded a stale (zeroed) fuel map.
    BuildTerrainIfEnabled();
    BuildForestIfPresent();

    bool bFireLoaded = false;
    const FString& CsvPath = FirstFireTickCSVPath.FilePath;
    if (!CsvPath.IsEmpty())
    {
        TArray<int32> Imported;
        if (UFireDataImporter::ImportFirstFireTicksFromCSV(
                CsvPath, GridWidth, GridHeight, ArrivalMinutesPerTick, BurnDurationTicks, Imported))
        {
            WorldGrid->SetFirstFireTickArray(Imported);
            bFireLoaded = true;
        }
    }
    if (!bFireLoaded && bUseFallbackFireIfNoCSV)
    {
        TArray<int32> Fallback;
        UFireDataImporter::GenerateFallbackCentreFire(
            GridWidth, GridHeight, FallbackIgnitionRadiusCells,
            FallbackSpreadTicksPerCell, FallbackMaxSpreadCells, Fallback);
        WorldGrid->SetFirstFireTickArray(Fallback);
    }

    // Load the NEW scenario's human trajectories (ResetReplay alone keeps the old file).
    if (HumanManager)
    {
        if (!HumansCSVPath.FilePath.IsEmpty())
        {
            HumanManager->LoadTrajectories(HumansCSVPath.FilePath);
        }
        HumanManager->ResetReplay(WorldGrid.Get());
        HumanManager->ApplyTick(0, WorldGrid.Get());
    }

    SetupFireVisuals();

    if (UWorld* World = GetWorld())
    {
        if (UGpuBeliefManager* BeliefMgr = World->GetSubsystem<UGpuBeliefManager>())
        {
            BeliefMgr->InitializeTextures(GridWidth, GridHeight);
            BeliefMgr->ResetTextures();
        }
    }

    // Restore canonical drone spawn positions (XY is identical for same-grid
    // batches; Z is recomputed from terrain on the first move), then re-init state.
    TArray<TObjectPtr<ADrone>> ActiveDrones;
    TArray<FVector2f> DronePositionsCellSpace;
    ActiveDrones.Reserve(RegisteredDrones.Num());
    DronePositionsCellSpace.Reserve(RegisteredDrones.Num());
    for (int32 i = 0; i < RegisteredDrones.Num(); ++i)
    {
        ADrone* D = RegisteredDrones[i].Get();
        if (!D) continue;
        if (InitialDroneSpawnLocations.IsValidIndex(i))
        {
            if (BenchmarkDroneSpawnCellPositions.IsValidIndex(i))
            {
                const FVector2D Cell = BenchmarkDroneSpawnCellPositions[i];
                const float GroundZ = TerrainActor ? TerrainActor->GetHeightAtCell(Cell.X, Cell.Y) : GridOriginWorld.Z;
                D->SetActorLocation(FVector(
                    GridOriginWorld.X + Cell.X * CellSizeUU,
                    GridOriginWorld.Y + Cell.Y * CellSizeUU,
                    GroundZ + DroneCruiseAltitudeAboveGroundUU), false, nullptr, ETeleportType::TeleportPhysics);
                D->SetActorRotation(FRotator::ZeroRotator, ETeleportType::TeleportPhysics);
            }
            else
            {
                D->SetActorLocation(InitialDroneSpawnLocations[i]);
            }
        }
        D->InitFromManager(WorldGrid.Get(), D->GetDroneIndex(), GridOriginWorld, CellSizeUU);
        ApplyManagerConfigurationToDrone(D);
        const FVector W = D->GetActorLocation();
        DronePositionsCellSpace.Emplace(
            static_cast<float>((W.X - GridOriginWorld.X) / CellSizeUU),
            static_cast<float>((W.Y - GridOriginWorld.Y) / CellSizeUU));
        ActiveDrones.Add(RegisteredDrones[i]);
    }
    ApplyManagerSettingsToAllDrones();

    if (ActiveDrones.Num() > 0)
    {
        TArray<uint16> Owners = ComputeBalancedPartition(ActiveDrones, DronePositionsCellSpace, 0);
        ApplyPartitionOwnersToGrid(Owners);
    }

    // Begin the next scenario's own metrics run (run_id / subdir came from its config).
    State = ESimState::Running;
    if (bRecordMetrics)
    {
        if (bMixedRecoveryRequested && (!bEnableVisionBridge || VisionCacheMode != EVisionCacheMode::Replay ||
            !bEnableDroneCaptures || NumRealCameraDrones != 1 || !CanUseParallelDroneProcessing()))
        {
            UE_LOG(LogTemp, Error, TEXT("Mixed observation recovery scenario is incompatible with required replay/capture/parallel settings; stopping batch."));
            State = ESimState::Stopped;
            return false;
        }
        MetricsRecorder.SetDetailMode(
            MetricsDetailMode.Equals(TEXT("summary"), ESearchCase::IgnoreCase)
                ? ESimMetricsDetailMode::Summary
                : ESimMetricsDetailMode::Full);
        const FString RunId = MetricsRunId.IsEmpty()
            ? FString::Printf(TEXT("run_%d"), CurrentScenarioIndex)
            : MetricsRunId;
        MetricsRecorder.BeginRun(RunId, FSimRunConfig::MakeConfigSnapshot(*this), MetricsRunSubdir);
        RecordInitialDroneIdentityMap();
        MetricsRecorder.SetPersonGeometricRecoveryEnabled(
            bEnableVisionBridge && VisionCacheMode == EVisionCacheMode::Replay &&
            (FParse::Param(FCommandLine::Get(), TEXT("ThesisPersonGeometricRecovery")) || bMixedRecoveryRequested));
        MetricsRecorder.SetMixedObservationRecoveryEnabled(bMixedRecoveryRequested);
        MetricsRecorder.SetIdealSensorWitnessRecordingEnabled(
            bBenchmarkRecordIdealSensorWitnesses || bMixedRecoveryRequested);
        FlushPendingWallClockPhaseTimings();
        RecordDroneProcessingModeEvent();
        MetricsRecorder.SetRecallDeadlineTicks(RecallDeadlineTicks);
        if (HumanManager)
        {
            MetricsRecorder.SetPersonCount(HumanManager->GetNumPersons());
        }
    }

    UE_LOG(LogTemp, Log, TEXT("Thesis in-process: started scenario %d/%d (%s)."),
           CurrentScenarioIndex + 1, ScenarioManifestPaths.Num(), *NextConfig);
    return true;
}

void ASimulationManager::ResetSimulation()
{
    const bool bMixedRecoveryRequested = FParse::Param(FCommandLine::Get(), TEXT("ThesisMixedObservationRecovery"));
    if (bMixedRecoveryRequested && (!bRecordMetrics || !bEnableVisionBridge ||
        VisionCacheMode != EVisionCacheMode::Replay || !bEnableDroneCaptures ||
        NumRealCameraDrones != 1 || !CanUseParallelDroneProcessing()))
    {
        UE_LOG(LogTemp, Error, TEXT("Mixed observation recovery reset is incompatible with required metrics/replay/capture/parallel settings."));
        State = ESimState::Stopped;
        return;
    }
    RestoreSensorResources();
    NormalizeEnvironmentLighting();
    // NOTE: ResetSimulation does NOT regenerate the fire CSV — it reloads the
    // existing one. If the forest/fuel map changed since the fire was generated
    // (e.g. ForestDresser settings edited mid-session), the fire plan no longer
    // matches the vegetation. Restart PIE or call
    // GenerateFireCsvFromManagerSettings() to regenerate.
    UE_LOG(LogTemp, Log,
           TEXT("Thesis: ResetSimulation reuses the existing fire CSV (fuel changes since generation are NOT reflected)."));

    // Close the current metrics run; a new one starts on the next StartSimulation
    // (or immediately below if the sim is already running).
    MetricsRecorder.EndRun(CurrentTick);

    CurrentTick = 0;
    TickAccumulator = 0.0f;
    OracleAssignmentTick = INDEX_NONE;
    OracleCachedKeyframeCount = -1;
    PendingObservations.Reset();
    CachedServerPriority.Reset();
    CachedServerPriorityTick = -1;
    TicksSinceLastPriorityReadback = 9999;

    if (!WorldGrid.IsValid())
    {
        WorldGrid = MakeUnique<Grid>(GridWidth, GridHeight);
    }

    WorldGrid->Initialize();

    bool bFireLoaded = false;
    const FString& CsvPath = FirstFireTickCSVPath.FilePath;
    if (!CsvPath.IsEmpty())
    {
        TArray<int32> Imported;
        if (UFireDataImporter::ImportFirstFireTicksFromCSV(
                CsvPath, GridWidth, GridHeight,
                ArrivalMinutesPerTick, BurnDurationTicks, Imported))
        {
            WorldGrid->SetFirstFireTickArray(Imported);
            bFireLoaded = true;
        }
    }

    if (!bFireLoaded && bUseFallbackFireIfNoCSV)
    {
        TArray<int32> Fallback;
        UFireDataImporter::GenerateFallbackCentreFire(
            GridWidth, GridHeight,
            FallbackIgnitionRadiusCells,
            FallbackSpreadTicksPerCell,
            FallbackMaxSpreadCells,
            Fallback);
        WorldGrid->SetFirstFireTickArray(Fallback);
    }

    if (UWorld* World = GetWorld())
    {
        if (UGpuBeliefManager* BeliefMgr = World->GetSubsystem<UGpuBeliefManager>())
        {
            BeliefMgr->InitializeTextures(GridWidth, GridHeight);
            BeliefMgr->ResetTextures();
        }
    }

    if (HumanManager)
    {
        HumanManager->ResetReplay(WorldGrid.Get());
        HumanManager->ApplyTick(0, WorldGrid.Get());
    }

    // Grid::Initialize() wiped floraDensity — rebuild terrain + forest before
    // drones re-copy floraDensity (grid size may have changed in the editor).
    BuildTerrainIfEnabled();
    BuildForestIfPresent();
    SetupFireVisuals();

    TArray<TObjectPtr<ADrone>> ActiveDrones;
    TArray<FVector2f> DronePositionsCellSpace;
    ActiveDrones.Reserve(RegisteredDrones.Num());
    DronePositionsCellSpace.Reserve(RegisteredDrones.Num());

    for (int32 Slot = 0; Slot < RegisteredDrones.Num(); ++Slot)
    {
        TObjectPtr<ADrone>& Drone = RegisteredDrones[Slot];
        if (ADrone* D = Drone.Get())
        {
            if (BenchmarkDroneSpawnCellPositions.IsValidIndex(Slot))
            {
                const FVector2D Cell = BenchmarkDroneSpawnCellPositions[Slot];
                const float GroundZ = TerrainActor ? TerrainActor->GetHeightAtCell(Cell.X, Cell.Y) : GridOriginWorld.Z;
                D->SetActorLocation(FVector(
                    GridOriginWorld.X + Cell.X * CellSizeUU,
                    GridOriginWorld.Y + Cell.Y * CellSizeUU,
                    GroundZ + DroneCruiseAltitudeAboveGroundUU), false, nullptr, ETeleportType::TeleportPhysics);
                D->SetActorRotation(FRotator::ZeroRotator, ETeleportType::TeleportPhysics);
            }
            D->InitFromManager(WorldGrid.Get(), D->GetDroneIndex(), GridOriginWorld, CellSizeUU);
            ApplyManagerConfigurationToDrone(D);

            const FVector W = D->GetActorLocation();
            DronePositionsCellSpace.Emplace(
                static_cast<float>((W.X - GridOriginWorld.X) / CellSizeUU),
                static_cast<float>((W.Y - GridOriginWorld.Y) / CellSizeUU));
            ActiveDrones.Add(Drone);
        }
    }

    if (ActiveDrones.Num() > 0)
    {
        TArray<uint16> Owners = ComputeBalancedPartition(ActiveDrones, DronePositionsCellSpace, 0);
        ApplyPartitionOwnersToGrid(Owners);
    }

    // If the sim is mid-run, start a fresh metrics run for the new episode.
    if (State == ESimState::Running && bRecordMetrics)
    {
        const FString RunId = MetricsRunId.IsEmpty()
            ? FString::Printf(TEXT("run_%s"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")))
            : MetricsRunId;
        MetricsRecorder.SetDetailMode(
            MetricsDetailMode.Equals(TEXT("summary"), ESearchCase::IgnoreCase)
                ? ESimMetricsDetailMode::Summary
                : ESimMetricsDetailMode::Full);
        MetricsRecorder.BeginRun(RunId, FSimRunConfig::MakeConfigSnapshot(*this), MetricsRunSubdir);
        RecordInitialDroneIdentityMap();
        MetricsRecorder.SetPersonGeometricRecoveryEnabled(
            bEnableVisionBridge && VisionCacheMode == EVisionCacheMode::Replay &&
            (FParse::Param(FCommandLine::Get(), TEXT("ThesisPersonGeometricRecovery")) || bMixedRecoveryRequested));
        MetricsRecorder.SetMixedObservationRecoveryEnabled(bMixedRecoveryRequested);
        MetricsRecorder.SetIdealSensorWitnessRecordingEnabled(
            bBenchmarkRecordIdealSensorWitnesses || bMixedRecoveryRequested);
        FlushPendingWallClockPhaseTimings();
        RecordDroneProcessingModeEvent();
        if (bSaveYoloDebugImages)
        {
            IFileManager::Get().DeleteDirectory(
                *FPaths::Combine(MetricsRecorder.GetRunDirectory(), TEXT("vision")),
                /*RequireExists=*/false,
                /*Tree=*/true);
        }
        MetricsRecorder.SetRecallDeadlineTicks(RecallDeadlineTicks);
        YoloErrorImagePairsSaved = 0;
        YoloCorrectImagePairsSaved = 0;
        PersonFirstDetectedTick.Reset();
        if (HumanManager)
        {
            PersonFirstDetectedTick.Init(-1, HumanManager->GetNumPersons());
            MetricsRecorder.SetPersonCount(HumanManager->GetNumPersons());
        }
    }
}

int32 ASimulationManager::DebugGetFirstFireTick(int32 X, int32 Y) const
{
    if (!WorldGrid.IsValid()) return -1;
    if (X < 0 || Y < 0 || X >= GridWidth || Y >= GridHeight) return -1;
    return WorldGrid->getFirstFireTick(X, Y);
}

void ASimulationManager::SetDebugPrioritySource(EDebugPrioritySource NewSource)
{
    DebugPrioritySource = NewSource;
}

void ASimulationManager::CycleDebugPrioritySource()
{
    switch (DebugPrioritySource)
    {
        case EDebugPrioritySource::None:
            DebugPrioritySource = EDebugPrioritySource::GroundTruthPriority;
            break;
        case EDebugPrioritySource::GroundTruthPriority:
            DebugPrioritySource = EDebugPrioritySource::DroneLocalPriority;
            break;
        case EDebugPrioritySource::DroneLocalPriority:
            DebugPrioritySource = EDebugPrioritySource::ServerPriority;
            break;
        case EDebugPrioritySource::ServerPriority:
        case EDebugPrioritySource::FireGroundTruth:
            DebugPrioritySource = EDebugPrioritySource::None;
            break;
        default:
            DebugPrioritySource = EDebugPrioritySource::None;
            break;
    }
}

void ASimulationManager::SetDebugPriorityDroneIndex(int32 NewIndex)
{
    DebugPriorityDroneIndex = NewIndex;
}

void ASimulationManager::SetupDebugKeyBindings()
{
    if (InputComponent)
    {
        InputComponent->BindKey(EKeys::One,   IE_Pressed, this, &ASimulationManager::OnKey_One);
        InputComponent->BindKey(EKeys::Two,   IE_Pressed, this, &ASimulationManager::OnKey_Two);
        InputComponent->BindKey(EKeys::Three, IE_Pressed, this, &ASimulationManager::OnKey_Three);
        InputComponent->BindKey(EKeys::Zero,  IE_Pressed, this, &ASimulationManager::OnKey_Zero);
        InputComponent->BindKey(EKeys::Tab,   IE_Pressed, this, &ASimulationManager::OnKey_Tab);
        InputComponent->BindKey(EKeys::Four,  IE_Pressed, this, &ASimulationManager::OnKey_Four);
        InputComponent->BindKey(EKeys::SpaceBar, IE_Pressed, this, &ASimulationManager::OnKey_Space);
        InputComponent->BindKey(EKeys::V, IE_Pressed, this, &ASimulationManager::OnKey_V);
        InputComponent->BindKey(EKeys::LeftBracket,  IE_Pressed, this, &ASimulationManager::OnKey_LeftBracket);
        InputComponent->BindKey(EKeys::RightBracket, IE_Pressed, this, &ASimulationManager::OnKey_RightBracket);
    }
}

void ASimulationManager::OnKey_One()  { SetDebugPrioritySource(EDebugPrioritySource::GroundTruthPriority); }
void ASimulationManager::OnKey_Two()  { SetDebugPrioritySource(EDebugPrioritySource::DroneLocalPriority); }
void ASimulationManager::OnKey_Three(){ SetDebugPrioritySource(EDebugPrioritySource::ServerPriority); }
void ASimulationManager::OnKey_Zero() { SetDebugPrioritySource(EDebugPrioritySource::None); }
void ASimulationManager::OnKey_Tab()  { CycleDebugPrioritySource(); }
void ASimulationManager::OnKey_Four() { SetDebugPrioritySource(EDebugPrioritySource::FireGroundTruth); }
void ASimulationManager::OnKey_Space() { ToggleSimulationPause(); }
void ASimulationManager::OnKey_V()            { ToggleDroneView(); }
void ASimulationManager::OnKey_LeftBracket()  { ViewPreviousDrone(); }
void ASimulationManager::OnKey_RightBracket() { ViewNextDrone(); }

void ASimulationManager::ApplyViewVisualMode()
{
    const bool bOverviewGridOnly = bOverviewShowsGridOnly && ViewedDroneIndex == INDEX_NONE;

    if (TerrainActor)
    {
        TerrainActor->SetVisualsEnabled(bRenderTerrainVisuals);
        TerrainActor->SetVisualsCaptureOnly(bOverviewGridOnly || bTerrainVisibleOnlyToDroneCaptures);
    }
    if (ForestDresserActor)
    {
        ForestDresserActor->SetVisualsCaptureOnly(bOverviewGridOnly || bForestVisibleOnlyToDroneCaptures);
    }
    if (HumanManager)
    {
        HumanManager->SetVisualsCaptureOnly(bOverviewGridOnly);
    }
    if (FireVisualsActor)
    {
        FireVisualsActor->SetVfxCaptureOnly(bFireVisualsOnlyForYoloCaptures || bFireVisualsVfxOnlyInDroneCaptures);
    }
}

void ASimulationManager::ViewDrone(int32 DroneIdx)
{
    if (!RegisteredDrones.IsValidIndex(DroneIdx) || !RegisteredDrones[DroneIdx])
    {
        return;
    }
    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC) return;

    ViewedDroneIndex = DroneIdx;
    LastViewedDroneIndex = DroneIdx;
    ApplyViewVisualMode();
    PC->SetViewTargetWithBlend(RegisteredDrones[DroneIdx].Get(), DroneViewBlendTime, VTBlend_Cubic);
}

void ASimulationManager::ViewOverview()
{
    APlayerController* PC = GetWorld()->GetFirstPlayerController();
    if (!PC) return;

    ViewedDroneIndex = INDEX_NONE;
    ApplyViewVisualMode();
    AActor* OverviewTarget = PC->GetPawn() ? static_cast<AActor*>(PC->GetPawn())
                                           : static_cast<AActor*>(PC);
    PC->SetViewTargetWithBlend(OverviewTarget, DroneViewBlendTime, VTBlend_Cubic);
}

void ASimulationManager::ToggleDroneView()
{
    if (ViewedDroneIndex == INDEX_NONE)
    {
        ViewDrone(RegisteredDrones.IsValidIndex(LastViewedDroneIndex) ? LastViewedDroneIndex : 0);
    }
    else
    {
        ViewOverview();
    }
}

void ASimulationManager::ViewNextDrone()
{
    const int32 N = RegisteredDrones.Num();
    if (N == 0) return;
    ViewDrone(ViewedDroneIndex == INDEX_NONE ? 0 : (ViewedDroneIndex + 1) % N);
}

void ASimulationManager::ViewPreviousDrone()
{
    const int32 N = RegisteredDrones.Num();
    if (N == 0) return;
    ViewDrone(ViewedDroneIndex == INDEX_NONE ? N - 1 : (ViewedDroneIndex - 1 + N) % N);
}
