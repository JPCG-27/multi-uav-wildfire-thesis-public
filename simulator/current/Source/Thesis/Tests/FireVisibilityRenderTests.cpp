#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/FireVisualsManager.h"
#include "../World/FireVisualProfile.h"
#include "../World/Grid.h"
#include "../Vision/DroneCaptureComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraComponent.h"
#include "NiagaraLightRendererProperties.h"
#include "NiagaraEmitterInstance.h"
#include "NiagaraSystemInstance.h"
#include "NiagaraSystemInstanceController.h"
#include "NiagaraDataSet.h"
#include "NiagaraParameterStore.h"
#include "NiagaraDataInterface.h"
#include "Components/PrimitiveComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "Misc/Paths.h"
#include "RenderingThread.h"
#include "UObject/UnrealType.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FFireVisibilityRenderTest, "Thesis.Fire.VisibilityRendering",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FFireVisibilityRenderTest::RunTest(const FString& Parameters)
{
    const UWorld::InitializationValues Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, FName(TEXT("FireVisibilityFixture")),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(World->WorldType).SetCurrentWorld(World);
    Grid Cells(1, 1);
    Cells.setFirstFireTick(0, 0, 0);
    Cells.setFloraDensity(0, 0, 1.0f);
    AFireVisualsManager* Fire = World->SpawnActor<AFireVisualsManager>();
    Fire->bConnectedFireFront = false; // Legacy baselines remain explicit as the actor default changes.
    Fire->bAutoLoadDefaultVfxAssets = false;
    Fire->bUseProceduralFallbackVfx = false;
    Fire->bLoadVfxOnlyForYoloCapture = false;
    Fire->MaxFireVfx = 1;
    Fire->FireClustersPerCell = 1;
    Fire->FireClusterRadiusUU = 0;
    Fire->bFireLights = false;
    AActor* CameraActor = World->SpawnActor<AActor>();
    UDroneCaptureComponent* Camera = NewObject<UDroneCaptureComponent>(CameraActor);
    CameraActor->SetRootComponent(Camera);
    Camera->RegisterComponent();
    Camera->InitCapture(512, 512, 60, 0);
    Camera->SetWorldLocation(FVector(-800, -400, 600));
    Camera->SetWorldRotation((FVector(500, 500, 150) - Camera->GetComponentLocation()).Rotation());
    Camera->SetExposure(false, 0, 0.15f, 14);
    Camera->bSavePngForDebug = true;
    Camera->DebugOutputDirectory = FPaths::ProjectSavedDir() / TEXT("FireQualityQA/reference_balance");

    struct FFlameFixtureAsset { const TCHAR* Name; const TCHAR* Path; };
    const FFlameFixtureAsset Assets[] = {
        { TEXT("NS_Fire_Small_Simple"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Small_Simple.NS_Fire_Small_Simple") },
        { TEXT("NS_Fire_Simple"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Simple.NS_Fire_Simple") },
        { TEXT("NS_Fire_Medium"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Medium.NS_Fire_Medium") },
        { TEXT("NS_Fire_Floor_01_Simple"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_01_Simple.NS_Fire_Floor_01_Simple") },
        { TEXT("NS_Fire_Floor_02_Simple"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_02_Simple.NS_Fire_Floor_02_Simple") },
        { TEXT("NS_Fire_Big_Simple"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Big_Simple.NS_Fire_Big_Simple") },
        { TEXT("NS_Fire_Floor_01"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_01.NS_Fire_Floor_01") },
        { TEXT("NS_Fire_Floor_02"), TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Floor_02.NS_Fire_Floor_02") },
        { TEXT("NS_Fire_Examples"), TEXT("/Game/NiagaraExamples/FX_Misc/NS_Fire.NS_Fire") },
    };
    for (int32 Index = 0; Index < UE_ARRAY_COUNT(Assets); ++Index)
    {
        const FString Label = Assets[Index].Name;
        Fire->FlameSystem = LoadObject<UNiagaraSystem>(nullptr, Assets[Index].Path);
        if (!TestNotNull(Assets[Index].Path, Fire->FlameSystem.Get())) continue;
        Fire->Init(&Cells, FVector::ZeroVector, 1000, 1000, nullptr, nullptr);
#if WITH_EDITOR
        FAssetCompilingManager::Get().FinishAllCompilation();
#endif
        Fire->PrepareForYoloCapture(30, Camera->GetComponentLocation(), 10000);
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->CaptureTick(30 + Index, true, 0.1f);
        TArray<FColor> Pixels;
        int32 W, H, Tick;
        TestTrue(TEXT("Read flame render"), Camera->ReadLastCapture(Pixels, W, H, Tick));
        int32 WarmPixels = 0;
        int32 PeakRed = 0;
        for (const FColor& Pixel : Pixels)
        {
            PeakRed = FMath::Max(PeakRed, int32(Pixel.R));
            if (Pixel.R > 30 && Pixel.R > Pixel.B * 2) ++WarmPixels;
        }
        AddInfo(FString::Printf(TEXT("%s EV14: warm pixels=%d peak red=%d"), *Label, WarmPixels, PeakRed));
        if (Label == TEXT("NS_Fire_Examples"))
        {
            TArray<UNiagaraComponent*> Components;
            Fire->GetComponents(Components);
            int32 ParticleCount = 0;
            auto LogStaticMeshInterfaces = [this](const FNiagaraParameterStore& Store, const FString& Scope)
            {
                const TArray<UNiagaraDataInterface*>& Interfaces = Store.GetDataInterfaces();
                for (int32 Index = 0; Index < Interfaces.Num(); ++Index)
                {
                    UNiagaraDataInterface* DataInterface = Interfaces[Index];
                    if (!DataInterface || !DataInterface->GetClass()->GetName().Contains(TEXT("StaticMesh"))) continue;
                    const FNiagaraVariableBase* Variable = Store.FindVariableFromDataInterfaceIndex(Index);
                    FString Details = FString::Printf(TEXT("%s %s var=%s"), *Scope,
                        *DataInterface->GetClass()->GetName(), Variable ? *Variable->GetName().ToString() : TEXT("<none>"));
                    if (FObjectPropertyBase* DefaultMeshProperty = CastField<FObjectPropertyBase>(
                            FindFProperty<FProperty>(DataInterface->GetClass(), TEXT("DefaultMesh"))))
                    {
                        UObject* DefaultMesh = DefaultMeshProperty->GetObjectPropertyValue_InContainer(DataInterface);
                        Details += FString::Printf(TEXT(" default=%s"), DefaultMesh ? *DefaultMesh->GetPathName() : TEXT("<null>"));
                    }
                    if (FObjectPropertyBase* SourceComponentProperty = CastField<FObjectPropertyBase>(
                            FindFProperty<FProperty>(DataInterface->GetClass(), TEXT("SourceComponent"))))
                    {
                        UObject* SourceComponent = SourceComponentProperty->GetObjectPropertyValue_InContainer(DataInterface);
                        Details += FString::Printf(TEXT(" source=%s"), SourceComponent ? *SourceComponent->GetPathName() : TEXT("<null>"));
                    }
                    AddInfo(Details);
                }
            };
            for (UNiagaraComponent* Component : Components)
            {
                UNiagaraSystem* RuntimeSystem = Component ? Component->GetAsset() : nullptr;
                if (!RuntimeSystem) continue;
                const FNiagaraVariable FlameScaleVariable(FNiagaraTypeDefinition::GetFloatDef(), FName(TEXT("User.Flame Scale")));
                if (RuntimeSystem->GetExposedParameters().IndexOf(FlameScaleVariable) == INDEX_NONE) continue;
                AddInfo(FString::Printf(TEXT("Epic runtime system %s state=%s"), *RuntimeSystem->GetPathName(),
                    *UEnum::GetValueAsString(Component->GetExecutionState())));
                for (const FNiagaraEmitterHandle& Handle : RuntimeSystem->GetEmitterHandles())
                    AddInfo(FString::Printf(TEXT("Epic emitter config %s enabled=%d"),
                        *Handle.GetName().ToString(), Handle.GetIsEnabled() ? 1 : 0));
                LogStaticMeshInterfaces(RuntimeSystem->GetExposedParameters(), TEXT("system"));

                const auto Controller = Component->GetSystemInstanceController();
                FNiagaraSystemInstance* Instance = Controller.IsValid() ? Controller->GetSystemInstance_Unsafe() : nullptr;
                if (!Instance) continue;
                LogStaticMeshInterfaces(Instance->GetInstanceParameters(), TEXT("instance"));
                for (const FNiagaraEmitterInstanceRef& Emitter : Instance->GetEmitters())
                {
                    const int32 EmitterParticles = Emitter->GetNumParticles();
                    ParticleCount += EmitterParticles;
                    const UEnum* StateEnum = StaticEnum<ENiagaraExecutionState>();
                    const FString StateName = StateEnum
                        ? StateEnum->GetNameStringByValue(static_cast<int64>(Emitter->GetExecutionState()))
                        : TEXT("Unknown");
                    AddInfo(FString::Printf(TEXT("Epic runtime emitter %s state=%s sim=%s particles=%d totalSpawned=%d"),
                        *Emitter->GetEmitterHandle().GetName().ToString(), *StateName,
                        Emitter->GetSimTarget() == ENiagaraSimTarget::CPUSim ? TEXT("CPU") : TEXT("GPU"),
                        EmitterParticles, Emitter->GetTotalSpawnedParticles()));
                }
            }
            AddInfo(FString::Printf(TEXT("Epic candidate live particles: %d"), ParticleCount));
            TArray<FNiagaraVariable> Variables;
            Fire->FlameSystem->GetExposedParameters().GetParameters(Variables);
            for (const FNiagaraVariable& Variable : Variables)
                if (Variable.GetType() == FNiagaraTypeDefinition::GetFloatDef())
                    AddInfo(FString::Printf(TEXT("Epic candidate %s = %g"), *Variable.GetName().ToString(),
                        Fire->FlameSystem->GetExposedParameters().GetParameterValue<float>(Variable)));
        }
        if (Label == TEXT("NS_Fire_Simple"))
            TestTrue(TEXT("NS_Fire_Simple renders flame at scene exposure"), WarmPixels > 100);
        Fire->EndYoloCapture();
    }
    // Confirm that the authored asset actually responds to the morphology
    // controls, rather than just checking component transforms.
    Fire->FlameSystem = LoadObject<UNiagaraSystem>(nullptr,
        TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Simple.NS_Fire_Simple"));
    int32 PreviousWarmPixels = 0;
    int32 PreviousHeight = 0;
    for (int32 Variant = 0; Variant < 2; ++Variant)
    {
        Fire->FlameWidthScaleMultiplier = Variant == 0 ? 1.0f : 1.55f;
        Fire->FlameHeightScaleMultiplier = Variant == 0 ? 1.0f : 2.4f;
        Fire->BundledFlameSpriteSizeMultiplier = Variant == 0 ? 1.0f : 3.0f;
        Fire->Init(&Cells, FVector::ZeroVector, 1000, 1000, nullptr, nullptr);
        Fire->PrepareForYoloCapture(30, Camera->GetComponentLocation(), 10000);
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->DebugOutputDirectory = FPaths::ProjectSavedDir() /
            (Variant == 0 ? TEXT("FireQualityQA/reference_balance/before") : TEXT("FireQualityQA/reference_balance/after"));
        Camera->CaptureTick(30, true, 0.1f);
        TArray<FColor> Pixels;
        int32 W, H, Tick;
        TestTrue(TEXT("Read flame morphology render"), Camera->ReadLastCapture(Pixels, W, H, Tick));
        int32 WarmPixels = 0, MinY = H, MaxY = -1;
        for (int32 I = 0; I < Pixels.Num(); ++I)
        {
            const FColor& Pixel = Pixels[I];
            if (Pixel.R > 30 && Pixel.R > Pixel.B * 2)
            {
                ++WarmPixels;
                MinY = FMath::Min(MinY, I / W);
                MaxY = FMath::Max(MaxY, I / W);
            }
        }
        const int32 Height = MaxY >= MinY ? MaxY - MinY + 1 : 0;
        AddInfo(FString::Printf(TEXT("Flame morphology %s: warm pixels=%d height=%d"),
            Variant == 0 ? TEXT("before") : TEXT("after"), WarmPixels, Height));
        if (Variant == 0) { PreviousWarmPixels = WarmPixels; PreviousHeight = Height; }
        else
        {
            TestTrue(TEXT("Flame morphology increases visible coverage"), WarmPixels > PreviousWarmPixels * 1.3f);
            TestTrue(TEXT("Flame morphology increases visible height"), Height > PreviousHeight * 1.2f);
        }
        Fire->EndYoloCapture();
    }

    // Compare the current broad sprites with a narrower component footprint,
    // taller particle trajectory and smaller quads. This tests silhouette
    // shape without increasing Niagara system count or sprite area.
    Fire->FlameSystem = LoadObject<UNiagaraSystem>(nullptr,
        TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Simple.NS_Fire_Simple"));
    int32 BaselineAspectWidth = 0;
    int32 BaselineAspectHeight = 0;
    int32 BaselineAspectPixels = 0;
    for (const bool bNarrowTall : { false, true })
    {
        Fire->FlameWidthScaleMultiplier = bNarrowTall ? 1.2f : 1.55f;
        Fire->FlameHeightScaleMultiplier = bNarrowTall ? 4.0f : 2.4f;
        Fire->BundledFlameSpriteSizeMultiplier = bNarrowTall ? 2.4f : 3.0f;
        Fire->Init(&Cells, FVector::ZeroVector, 1000, 1000, nullptr, nullptr);
        Fire->PrepareForYoloCapture(30, Camera->GetComponentLocation(), 10000);
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->DebugOutputDirectory = FPaths::ProjectSavedDir() /
            (bNarrowTall ? TEXT("FireQualityQA/narrow_tall_flame/narrow_tall") :
                           TEXT("FireQualityQA/narrow_tall_flame/current"));
        Camera->CaptureTick(30, true, 0.1f);
        TArray<FColor> Pixels;
        int32 W, H, Tick;
        TestTrue(TEXT("Read flame shape render"), Camera->ReadLastCapture(Pixels, W, H, Tick));
        int32 WarmPixels = 0, MinX = W, MaxX = -1, MinY = H, MaxY = -1;
        for (int32 I = 0; I < Pixels.Num(); ++I)
        {
            const FColor& Pixel = Pixels[I];
            if (Pixel.R > 30 && Pixel.R > Pixel.B * 2)
            {
                const int32 X = I % W;
                const int32 Y = I / W;
                ++WarmPixels;
                MinX = FMath::Min(MinX, X);
                MaxX = FMath::Max(MaxX, X);
                MinY = FMath::Min(MinY, Y);
                MaxY = FMath::Max(MaxY, Y);
            }
        }
        const int32 FlameWidthPixels = MaxX >= MinX ? MaxX - MinX + 1 : 0;
        const int32 FlameHeightPixels = MaxY >= MinY ? MaxY - MinY + 1 : 0;
        AddInfo(FString::Printf(TEXT("Flame shape %s: warm pixels=%d bbox=%dx%d"),
            bNarrowTall ? TEXT("narrow/tall") : TEXT("current"), WarmPixels,
            FlameWidthPixels, FlameHeightPixels));
        if (!bNarrowTall)
        {
            BaselineAspectPixels = WarmPixels;
            BaselineAspectWidth = FlameWidthPixels;
            BaselineAspectHeight = FlameHeightPixels;
        }
        else
        {
            TestTrue(TEXT("Narrow/tall flame remains visible"), WarmPixels > 100);
            TestTrue(TEXT("Narrow/tall flame increases rendered height-to-width ratio"),
                FlameWidthPixels > 0 && BaselineAspectWidth > 0
                && static_cast<float>(FlameHeightPixels) / FlameWidthPixels
                    > (static_cast<float>(BaselineAspectHeight) / BaselineAspectWidth) * 1.15f);
            AddInfo(FString::Printf(TEXT("Warm-pixel area ratio narrow/tall/current=%.3f"),
                BaselineAspectPixels > 0 ? static_cast<float>(WarmPixels) / BaselineAspectPixels : 0.0f));
        }
        Fire->EndYoloCapture();
    }

    // Regression: even a saved finite-lifetime profile cannot extinguish a
    // simulator-burning cell. Check capture/preview paths and rewind on pixels.
    UFireVisualProfile* LegacyProfile = NewObject<UFireVisualProfile>(Fire);
    LegacyProfile->bUseFiniteVisualLifetime = true;
    LegacyProfile->FlameDurationOverrideTicks = 2;
    LegacyProfile->SmoulderDurationTicks = 1;
    Fire->VisualProfile = LegacyProfile;
    Cells.setFirstFireTick(0, 0, 5);
    Camera->bSavePngForDebug = false;
    for (const bool bPreview : { false, true })
    {
        Fire->bIncrementalNiagaraPreviewStepping = bPreview;
        Fire->Init(&Cells, FVector::ZeroVector, 1000, 2, nullptr, nullptr);
        for (const int32 TargetTick : { 4, 10, 500, 4, 10 })
        {
            if (bPreview) Fire->ApplyTick(TargetTick);
            else Fire->PrepareForYoloCapture(TargetTick, Camera->GetComponentLocation(), 10000);
            World->SendAllEndOfFrameUpdates();
            FlushRenderingCommands();
            Camera->CaptureTick(TargetTick, true, 0.1f);
            TArray<FColor> Pixels;
            int32 W, H, CapturedTick;
            TestTrue(TEXT("Read persistent flame render"), Camera->ReadLastCapture(Pixels, W, H, CapturedTick));
            int32 WarmPixels = 0;
            for (const FColor& Pixel : Pixels)
                if (Pixel.R > 30 && Pixel.R > Pixel.B * 2) ++WarmPixels;
            TestTrue(FString::Printf(TEXT("%s flame follows ignition at tick %d (warm pixels %d)"),
                bPreview ? TEXT("Preview") : TEXT("Capture"), TargetTick, WarmPixels),
                TargetTick < 5 ? WarmPixels == 0 : WarmPixels > 100);
            if (!bPreview) Fire->EndYoloCapture();
        }
    }
    Fire->VisualProfile = nullptr;
    Fire->bIncrementalNiagaraPreviewStepping = false;

    // Compare update cost on the same bounded 32-cluster workload. Timings are
    // diagnostic only; they never feed the simulation or make a flaky assertion.
    Grid ManyCells(4, 2);
    for (int32 Y = 0; Y < 2; ++Y)
        for (int32 X = 0; X < 4; ++X)
        {
            ManyCells.setFirstFireTick(X, Y, 0);
            ManyCells.setFloraDensity(X, Y, 1);
        }
    UNiagaraSystem* SimpleFire = LoadObject<UNiagaraSystem>(nullptr,
        TEXT("/Game/Vefects/Free_Fire/Shared/Particles/NS_Fire_Simple.NS_Fire_Simple"));
    Fire->FlameSystem = SimpleFire;
    Fire->MaxFireVfx = 32;
    Fire->FireClustersPerCell = 4;
    Fire->bIncrementalNiagaraPreviewStepping = false;
    Fire->Init(&ManyCells, FVector::ZeroVector, 1000, 1000, nullptr, nullptr);
    Fire->ApplyTick(0);

    // Preview uses its own hidden-from-capture actor. Keep the sensor actor
    // canonical and exercise incremental updates only on the preview actor.
    AFireVisualsManager* PreviewFire = World->SpawnActor<AFireVisualsManager>();
    PreviewFire->bConnectedFireFront = false;
    PreviewFire->bAutoLoadDefaultVfxAssets = false;
    PreviewFire->bUseProceduralFallbackVfx = false;
    PreviewFire->bLoadVfxOnlyForYoloCapture = false;
    PreviewFire->bFireLights = false;
    PreviewFire->bHideFromSceneCaptures = true;
    PreviewFire->MaxFireVfx = 32;
    PreviewFire->FireClustersPerCell = 4;
    PreviewFire->FlameSystem = SimpleFire;
    PreviewFire->SmokeSystem = LoadObject<UNiagaraSystem>(nullptr,
        TEXT("/Game/NiagaraExamples/FX_Smoke/NS_Smoke_Plume.NS_Smoke_Plume"));

    Camera->bSavePngForDebug = false;
    // Same 32 flame clusters and same smoke assets; only plume ownership changes.
    PreviewFire->bIncrementalNiagaraPreviewStepping = true;
    for (const bool bShareSmoke : { false, true })
    {
        PreviewFire->bShareSmokeAcrossCellClusters = bShareSmoke;
        PreviewFire->Init(&ManyCells, FVector::ZeroVector, 1000, 1000, nullptr, nullptr);
        PreviewFire->ApplyTick(0);
        const double Start = FPlatformTime::Seconds();
        for (int32 T = 1; T <= 20; ++T) PreviewFire->ApplyTick(T);
        const double Milliseconds = (FPlatformTime::Seconds() - Start) * 1000.0 / 20;
        TArray<UNiagaraComponent*> Components;
        PreviewFire->GetComponents(Components);
        int32 VisibleSystems = 0;
        for (const UNiagaraComponent* Component : Components)
            if (Component->IsVisible()) ++VisibleSystems;
        // This fixture has exactly 32 flame systems and no ember/outer layers.
        // Transient asset names change after Init, so never classify by name.
        const int32 VisibleSmoke = VisibleSystems - 32;
        TestEqual(TEXT("One two-layer plume per cell when sharing, per cluster otherwise"),
            VisibleSmoke, bShareSmoke ? 16 : 64);
        AddInfo(FString::Printf(TEXT("32-cluster smoke %s: %d visible systems, %.3f ms/tick"),
            bShareSmoke ? TEXT("shared") : TEXT("per-cluster"), VisibleSmoke, Milliseconds));
    }
    for (const bool bIncremental : { false, true })
    {
        PreviewFire->bIncrementalNiagaraPreviewStepping = bIncremental;
        PreviewFire->Init(&ManyCells, FVector::ZeroVector, 1000, 1000, nullptr, nullptr);
        PreviewFire->ApplyTick(0);
        const double Start = FPlatformTime::Seconds();
        for (int32 T = 1; T <= 20; ++T) PreviewFire->ApplyTick(T);
        AddInfo(FString::Printf(TEXT("32-cluster Niagara update (%s): %.3f ms/tick"),
            bIncremental ? TEXT("incremental") : TEXT("reconstruct"),
            (FPlatformTime::Seconds() - Start) * 1000.0 / 20));
    }
    auto CapturePixels = [&](int32 CaptureTick)
    {
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->CaptureTick(CaptureTick, true, 0.1f);
        TArray<FColor> Pixels;
        int32 W, H, Tick;
        TestTrue(TEXT("Read canonical fire capture"), Camera->ReadLastCapture(Pixels, W, H, Tick));
        return Pixels;
    };
    auto MeasurePixelDelta = [](const TArray<FColor>& A, const TArray<FColor>& B,
                                int32& OutChangedPixels, int32& OutMaxChannelDelta)
    {
        OutChangedPixels = 0;
        OutMaxChannelDelta = 0;
        if (A.Num() != B.Num())
        {
            OutChangedPixels = FMath::Max(A.Num(), B.Num());
            OutMaxChannelDelta = 255;
            return;
        }
        for (int32 Index = 0; Index < A.Num(); ++Index)
        {
            const int32 Delta = FMath::Max(
                FMath::Max(FMath::Abs(int32(A[Index].R) - int32(B[Index].R)),
                           FMath::Abs(int32(A[Index].G) - int32(B[Index].G))),
                FMath::Max(FMath::Abs(int32(A[Index].B) - int32(B[Index].B)),
                           FMath::Abs(int32(A[Index].A) - int32(B[Index].A))));
            if (Delta > 0) ++OutChangedPixels;
            OutMaxChannelDelta = FMath::Max(OutMaxChannelDelta, Delta);
        }
    };

    // Include lit opaque geometry so stray Niagara particle lights, shadows,
    // reflections, or indirect-lighting contributions cannot hide in an
    // all-black background when preview is warmed below.
    AActor* FloorActor = World->SpawnActor<AActor>();
    UStaticMeshComponent* CaptureFloor = NewObject<UStaticMeshComponent>(FloorActor);
    FloorActor->SetRootComponent(CaptureFloor);
    CaptureFloor->SetStaticMesh(LoadObject<UStaticMesh>(nullptr, TEXT("/Engine/BasicShapes/Cube.Cube")));
    TestNotNull(TEXT("Lit opaque capture floor asset"), CaptureFloor->GetStaticMesh().Get());
    CaptureFloor->RegisterComponent();
    CaptureFloor->SetWorldLocation(FVector(2000.0f, 1000.0f, -55.0f));
    CaptureFloor->SetWorldScale3D(FVector(40.0f, 30.0f, 0.1f));
    CaptureFloor->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // Prepare once. Repeated captures at one canonical tick should be byte
    // identical before and after the separate live preview advances in time.
    Fire->PrepareForYoloCapture(30, Camera->GetComponentLocation(), 10000);
    const TArray<FColor> BaselinePixelsA = CapturePixels(30);
    const TArray<FColor> BaselinePixelsB = CapturePixels(30);
    TestTrue(TEXT("Canonical same-tick captures are pixel-identical before preview advances"),
        BaselinePixelsA.Num() > 0 && BaselinePixelsA == BaselinePixelsB);
    int32 BaselineFirePixels = 0;
    for (const FColor& Pixel : BaselinePixelsA)
        if (Pixel.R > 30 && Pixel.R > Pixel.B * 2) ++BaselineFirePixels;
    TestTrue(TEXT("Baseline capture contains visible fire pixels over lit geometry"), BaselineFirePixels > 0);

    auto SnapshotSensorParticles = [&]()
    {
        TArray<uint8> State;
        auto Append = [&State](const void* Data, int32 NumBytes)
        {
            const int32 Offset = State.AddUninitialized(NumBytes);
            if (NumBytes > 0) FMemory::Memcpy(State.GetData() + Offset, Data, NumBytes);
        };
        auto AppendValue = [&Append](const auto& Value) { Append(&Value, sizeof(Value)); };

        TArray<UNiagaraComponent*> Components;
        Fire->GetComponents(Components);
        Components.Sort([](const UNiagaraComponent& A, const UNiagaraComponent& B)
        {
            return A.GetName() < B.GetName();
        });
        AppendValue(Components.Num());
        for (UNiagaraComponent* Component : Components)
        {
            const auto Controller = Component->GetSystemInstanceController();
            const float Age = Controller.IsValid() ? Controller->GetAge() : -1.0f;
            AppendValue(Age);
            FNiagaraSystemInstance* Instance = Controller.IsValid()
                ? Controller->GetSystemInstance_Unsafe() : nullptr;
            const int32 EmitterCount = Instance ? Instance->GetEmitters().Num() : -1;
            AppendValue(EmitterCount);
            if (!Instance) continue;

            for (const FNiagaraEmitterInstanceRef& Emitter : Instance->GetEmitters())
            {
                const FNiagaraDataSet& DataSet = Emitter->GetParticleData();
                const FNiagaraDataBuffer* Data = DataSet.GetCurrentData();
                const uint32 NumParticles = Data ? Data->GetNumInstances() : 0;
                AppendValue(NumParticles);
                if (!Data) continue;
                const uint32 NumFloatComponents = DataSet.GetNumFloatComponents();
                const uint32 NumIntComponents = DataSet.GetNumInt32Components();
                AppendValue(NumFloatComponents);
                AppendValue(NumIntComponents);
                for (uint32 ComponentIndex = 0; ComponentIndex < NumFloatComponents; ++ComponentIndex)
                    for (uint32 ParticleIndex = 0; ParticleIndex < NumParticles; ++ParticleIndex)
                    {
                        const float Value = *Data->GetInstancePtrFloat(ComponentIndex, ParticleIndex);
                        AppendValue(Value);
                    }
                for (uint32 ComponentIndex = 0; ComponentIndex < NumIntComponents; ++ComponentIndex)
                    for (uint32 ParticleIndex = 0; ParticleIndex < NumParticles; ++ParticleIndex)
                    {
                        const int32 Value = *Data->GetInstancePtrInt32(ComponentIndex, ParticleIndex);
                        AppendValue(Value);
                    }
            }
        }
        return State;
    };

    const TArray<uint8> SensorStateBeforePreview = SnapshotSensorParticles();
    // Control: advance native world time with all preview systems inactive. If
    // canonical state changes here, world-managed Niagara history is leaking
    // wall-clock ticks even without a running editor preview.
    PreviewFire->SetActorTickEnabled(false);
    TArray<UNiagaraComponent*> PreviewBeforeControl;
    PreviewFire->GetComponents(PreviewBeforeControl);
    for (UNiagaraComponent* Component : PreviewBeforeControl)
    {
        Component->SetVisibility(false);
        Component->Deactivate();
        Component->SetComponentTickEnabled(false);
    }
    const float PreviewDeltas[] = { 0.016f, 0.125f, 0.033f, 0.20f, 0.050f };
    for (const float DeltaSeconds : PreviewDeltas)
    {
        World->Tick(LEVELTICK_All, DeltaSeconds);
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
    }
    const TArray<FColor> PixelsAfterInactiveControl = CapturePixels(30);
    const TArray<uint8> SensorStateAfterInactiveControl = SnapshotSensorParticles();
    int32 ControlChangedPixels = 0;
    int32 ControlMaxChannelDelta = 0;
    MeasurePixelDelta(BaselinePixelsA, PixelsAfterInactiveControl,
        ControlChangedPixels, ControlMaxChannelDelta);
    AddInfo(FString::Printf(TEXT("No-preview world-tick control: changed pixels=%d, max channel delta=%d"),
        ControlChangedPixels, ControlMaxChannelDelta));
    TestTrue(TEXT("Inactive preview world ticks preserve exact same-tick sensor pixels"),
        BaselinePixelsA == PixelsAfterInactiveControl);
    TestTrue(TEXT("Inactive preview world ticks preserve canonical Niagara payload"),
        SensorStateBeforePreview == SensorStateAfterInactiveControl);

    // Re-establish the exact canonical capture state after the control before
    // measuring the independent live preview path.
    Fire->PrepareForYoloCapture(30, Camera->GetComponentLocation(), 10000);
    const TArray<FColor> BaselinePixelsForPreview = CapturePixels(30);
    const TArray<uint8> SensorStateBeforeRealtimePreview = SnapshotSensorParticles();
    PreviewFire->bUseRealtimeNiagaraForPreview = true;
    PreviewFire->bIncrementalNiagaraPreviewStepping = false;
    PreviewFire->ReassignEveryNTicks = 10;
    PreviewFire->Init(&ManyCells, FVector::ZeroVector, 1000, 1000, nullptr, nullptr);
    PreviewFire->ApplyTick(30);
    TArray<UNiagaraComponent*> PreviewNiagara;
    PreviewFire->GetComponents(PreviewNiagara);
    int32 ActiveRealtimePreviewSystems = 0;
    int32 ActivePreviewTickEnabledSystems = 0;
    bool bSourceHasParticleLightRenderer = false;
    for (const FNiagaraEmitterHandle& Handle : SimpleFire->GetEmitterHandles())
    {
        if (const FVersionedNiagaraEmitterData* Data = Handle.GetEmitterData())
            for (const UNiagaraRendererProperties* Renderer : Data->GetRenderers())
                bSourceHasParticleLightRenderer |= Cast<UNiagaraLightRendererProperties>(Renderer) != nullptr;
    }
    TestTrue(TEXT("Fixture fire source includes a Niagara particle-light renderer"), bSourceHasParticleLightRenderer);
    bool bSourceParticleLightRendererEnabled = false;
    for (const FNiagaraEmitterHandle& Handle : SimpleFire->GetEmitterHandles())
        if (const FVersionedNiagaraEmitterData* Data = Handle.GetEmitterData())
            for (const UNiagaraRendererProperties* Renderer : Data->GetRenderers())
                if (const UNiagaraLightRendererProperties* Light = Cast<UNiagaraLightRendererProperties>(Renderer))
                    bSourceParticleLightRendererEnabled |= Light->GetIsEnabled();
    TestTrue(TEXT("Authored fire asset particle light stays enabled for canonical effects"),
        bSourceParticleLightRendererEnabled);
    bool bPreviewLightRendererFound = false;
    UNiagaraComponent* AgeProbe = nullptr;
    float PreviewAgeBefore = -1.0f;
    for (const UNiagaraComponent* Component : PreviewNiagara)
    {
        if (Component->IsVisible())
        {
            ++ActiveRealtimePreviewSystems;
            TestTrue(FString::Printf(TEXT("Visible preview component %s is active"), *Component->GetName()),
                Component->IsActive());
            if (Component->IsComponentTickEnabled()) ++ActivePreviewTickEnabledSystems;
            if (!AgeProbe)
            {
                AgeProbe = const_cast<UNiagaraComponent*>(Component);
                const auto Controller = Component->GetSystemInstanceController();
                if (Controller.IsValid()) PreviewAgeBefore = Controller->GetAge();
            }
            if (UNiagaraSystem* Asset = Component->GetAsset())
            {
                for (const FNiagaraEmitterHandle& Handle : Asset->GetEmitterHandles())
                {
                    if (const FVersionedNiagaraEmitterData* Data = Handle.GetEmitterData())
                    {
                        for (const UNiagaraRendererProperties* Renderer : Data->GetRenderers())
                        {
                            if (const UNiagaraLightRendererProperties* Light = Cast<UNiagaraLightRendererProperties>(Renderer))
                            {
                                bPreviewLightRendererFound = true;
                                TestFalse(TEXT("Preview-local particle-light renderer is disabled"), Light->GetIsEnabled());
                            }
                        }
                    }
                }
            }
        }
        else
        {
            TestFalse(FString::Printf(TEXT("Hidden preview component %s is not ticking"), *Component->GetName()),
                Component->IsComponentTickEnabled());
        }
    }
    TestTrue(TEXT("Realtime preview fixture contains active flame/smoke systems"), ActiveRealtimePreviewSystems > 0);
    AddInfo(FString::Printf(TEXT("Realtime preview active systems=%d, component ticks enabled=%d"),
        ActiveRealtimePreviewSystems, ActivePreviewTickEnabledSystems));
    TestTrue(TEXT("Preview copy contains a checked particle-light renderer"), bPreviewLightRendererFound);

    // Advancing real world time by different frame deltas changes only preview
    // particle history; sensor time remains pinned to capture tick 30.
    int32 PreviewTick = 31;
    for (const float DeltaSeconds : PreviewDeltas)
    {
        PreviewFire->ApplyTick(PreviewTick++);
        World->Tick(LEVELTICK_All, DeltaSeconds);
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
    }
    if (AgeProbe)
    {
        const auto Controller = AgeProbe->GetSystemInstanceController();
        TestTrue(TEXT("Preview Niagara age advances under native world ticks"),
            Controller.IsValid() && PreviewAgeBefore >= 0.0f && Controller->GetAge() > PreviewAgeBefore);
    }
    TArray<UPrimitiveComponent*> PreviewPrimitives;
    PreviewFire->GetComponents(PreviewPrimitives);
    for (const UPrimitiveComponent* Primitive : PreviewPrimitives)
    {
        TestTrue(FString::Printf(TEXT("Preview component %s is hidden from scene captures"), *Primitive->GetName()),
            Primitive->bHiddenInSceneCapture);
        TestFalse(FString::Printf(TEXT("Preview component %s is hidden from reflection captures"), *Primitive->GetName()),
            Primitive->bVisibleInReflectionCaptures);
        TestFalse(FString::Printf(TEXT("Preview component %s is hidden from realtime sky captures"), *Primitive->GetName()),
            Primitive->bVisibleInRealTimeSkyCaptures);
        TestFalse(FString::Printf(TEXT("Preview component %s is excluded from ray tracing"), *Primitive->GetName()),
            Primitive->bVisibleInRayTracing);
        TestFalse(FString::Printf(TEXT("Preview component %s cannot cast hidden shadows"), *Primitive->GetName()),
            Primitive->bCastHiddenShadow);
    }
    TArray<UPointLightComponent*> PreviewLights;
    PreviewFire->GetComponents(PreviewLights);
    TestEqual(TEXT("Preview actor has no lights that can affect sensor captures"), PreviewLights.Num(), 0);

    const TArray<FColor> PixelsAfterPreview = CapturePixels(30);
    int32 PreviewChangedPixels = 0;
    int32 PreviewMaxChannelDelta = 0;
    MeasurePixelDelta(BaselinePixelsForPreview, PixelsAfterPreview,
        PreviewChangedPixels, PreviewMaxChannelDelta);
    AddInfo(FString::Printf(TEXT("Realtime preview world-tick comparison: changed pixels=%d, max channel delta=%d"),
        PreviewChangedPixels, PreviewMaxChannelDelta));
    TestTrue(TEXT("Realtime preview and variable frame deltas do not change canonical same-tick pixels"),
        BaselinePixelsForPreview == PixelsAfterPreview);
    const TArray<uint8> SensorStateAfterPreview = SnapshotSensorParticles();
    TestTrue(TEXT("Canonical sensor Niagara age and live particle payload are independent of preview updates"),
        SensorStateBeforeRealtimePreview == SensorStateAfterPreview);

    // Rewind to a non-cadence tick: resetting pool assignments must immediately
    // rebuild visible fire instead of waiting until the next multiple of ten.
    PreviewFire->ApplyTick(29);
    int32 RewoundVisiblePreviewSystems = 0;
    for (const UNiagaraComponent* Component : PreviewNiagara)
        if (Component && Component->IsVisible() && Component->IsActive())
            ++RewoundVisiblePreviewSystems;
    TestTrue(TEXT("Realtime preview rewind to non-cadence tick immediately restores fire visuals"),
        RewoundVisiblePreviewSystems > 0);
    AddInfo(FString::Printf(TEXT("Non-cadence rewind tick 29 restored %d visible preview systems"),
        RewoundVisiblePreviewSystems));
    Fire->EndYoloCapture();

    // A dense adjacent burn must cover every primary cell before spending the
    // bounded pool on extra cluster detail. This 96-cell test uses 128 slots,
    // four requested clusters per cell, and no forest anchors.
    Grid ConnectedCells(12, 8);
    for (int32 Y = 0; Y < 8; ++Y)
        for (int32 X = 0; X < 12; ++X)
        {
            ConnectedCells.setFirstFireTick(X, Y, 0);
            ConnectedCells.setFloraDensity(X, Y, 1.0f);
        }
    Fire->FlameSystem = SimpleFire;
    Fire->SmokeSystem = LoadObject<UNiagaraSystem>(nullptr,
        TEXT("/Game/NiagaraExamples/FX_Smoke/NS_Smoke_Plume.NS_Smoke_Plume"));
    Fire->bConnectedFireFront = true;
    Fire->bFireLights = false;
    Fire->bLoadVfxOnlyForYoloCapture = false;
    Fire->MaxFireVfx = 128;
    Fire->FireClustersPerCell = 4;
    Fire->FireClusterRadiusUU = 0.0f;
    Fire->SmokeLayerCount = 1;
    Fire->bShareSmokeAcrossCellClusters = true;
    Fire->Init(&ConnectedCells, FVector::ZeroVector, 300.0f, 1000, nullptr, nullptr);
    Fire->ApplyTick(0);
    TArray<UNiagaraComponent*> ConnectedFlames;
    Fire->GetComponents(ConnectedFlames);
    TSet<int32> RepresentedCells;
    int32 VisibleFlameComponents = 0;
    int32 VisibleSmokeComponents = 0;
    for (const UNiagaraComponent* Component : ConnectedFlames)
    {
        if (!Component || !Component->IsVisible() || !Component->GetAsset()) continue;
        bool bHasFlameEmitter = false;
        for (const FNiagaraEmitterHandle& Handle : Component->GetAsset()->GetEmitterHandles())
            if (Handle.GetName().ToString().StartsWith(TEXT("NE_Flame")) && Handle.GetIsEnabled())
                bHasFlameEmitter = true;
        if (bHasFlameEmitter)
        {
            ++VisibleFlameComponents;
            const FVector Location = Component->GetComponentLocation();
            const int32 X = FMath::FloorToInt(Location.X / 300.0f);
            const int32 Y = FMath::FloorToInt(Location.Y / 300.0f);
            if (X >= 0 && X < 12 && Y >= 0 && Y < 8)
            {
                RepresentedCells.Add(Y * 12 + X);
            }
            continue;
        }
        TArray<FNiagaraVariable> ExposedVariables;
        Component->GetAsset()->GetExposedParameters().GetParameters(ExposedVariables);
        for (const FNiagaraVariable& Parameter : ExposedVariables)
            if (Parameter.GetName() == FName(TEXT("User.SpawnRate")))
            {
                ++VisibleSmokeComponents;
                break;
            }
    }
    TestEqual(TEXT("Connected fire gives all96 adjacent cells a primary center support"), RepresentedCells.Num(), 96);
    TestEqual(TEXT("Connected fire fills only the existing flame pool"), VisibleFlameComponents, 128);
    TestEqual(TEXT("Two-by-two cell tiles share one smoke layer"), VisibleSmokeComponents, 24);
    AddInfo(FString::Printf(TEXT("Connected footprint fixture: %d/%d cell centers represented; %d flame and %d smoke systems"),
        RepresentedCells.Num(), 96, VisibleFlameComponents, VisibleSmokeComponents));

    // Compare the same adjacent burn with the old random cluster allocation
    // and the connected-front allocation. With smoke/lights off, warm-pixel
    // islands measure whether rendered flame tongues actually join.
    Fire->SmokeSystem = nullptr;
    Fire->SmokeLayerCount = 0;
    Fire->MaxFireVfx = 128;
    Fire->FireClustersPerCell = 4;
    Fire->FlameWidthScaleMultiplier = 1.2f;
    Fire->FlameHeightScaleMultiplier = 4.0f;
    Fire->BundledFlameSpriteSizeMultiplier = 2.4f;
    Camera->SetWorldLocation(FVector(1800, 1200, 4500));
    Camera->SetWorldRotation(FRotator(-90, 0, 0));
    Camera->SetExposure(false, 0, 0.15f, 14);
    Camera->bSavePngForDebug = true;
    Camera->DebugOutputDirectory = FPaths::ProjectSavedDir() / TEXT("FireQualityQA/connected_pixel_ab");
    struct FWarmPixelMetrics { int32 Islands = 0; int32 Total = 0; int32 Largest = 0; };
    auto MeasureWarmIslands = [](const TArray<FColor>& Pixels, int32 Width, int32 Height)
    {
        FWarmPixelMetrics Metrics;
        TArray<uint8> Visited;
        Visited.Init(0, Pixels.Num());
        TArray<int32> Queue;
        for (int32 PixelIdx = 0; PixelIdx < Pixels.Num(); ++PixelIdx)
        {
            const FColor& Pixel = Pixels[PixelIdx];
            if (Visited[PixelIdx] || Pixel.R <= 30 || Pixel.R <= Pixel.B * 2) continue;
            ++Metrics.Islands;
            Visited[PixelIdx] = 1;
            Queue.Reset();
            Queue.Add(PixelIdx);
            for (int32 Head = 0; Head < Queue.Num(); ++Head)
            {
                const int32 X = Queue[Head] % Width;
                const int32 Y = Queue[Head] / Width;
                for (int32 DY = -1; DY <= 1; ++DY)
                    for (int32 DX = -1; DX <= 1; ++DX)
                    {
                        const int32 NX = X + DX;
                        const int32 NY = Y + DY;
                        if (NX < 0 || NX >= Width || NY < 0 || NY >= Height) continue;
                        const int32 Next = NY * Width + NX;
                        if (Visited[Next]) continue;
                        const FColor& Candidate = Pixels[Next];
                        if (Candidate.R <= 30 || Candidate.R <= Candidate.B * 2) continue;
                        Visited[Next] = 1;
                        Queue.Add(Next);
                    }
            }
            Metrics.Largest = FMath::Max(Metrics.Largest, Queue.Num());
        }
        Metrics.Total = 0;
        for (const FColor& Pixel : Pixels)
            if (Pixel.R > 30 && Pixel.R > Pixel.B * 2) ++Metrics.Total;
        return Metrics;
    };
    FWarmPixelMetrics WarmMetrics[2];
    for (int32 Mode = 0; Mode < 2; ++Mode)
    {
        Fire->bConnectedFireFront = (Mode == 1);
        Fire->Init(&ConnectedCells, FVector::ZeroVector, 300.0f, 1000, nullptr, nullptr);
        Fire->PrepareForYoloCapture(30, Camera->GetComponentLocation(), 10000);
        World->SendAllEndOfFrameUpdates();
        FlushRenderingCommands();
        Camera->CaptureTick(600 + Mode, true, 0.1f);
        TArray<FColor> Pixels;
        int32 Width = 0, Height = 0, CapturedTick = INDEX_NONE;
        TestTrue(TEXT("Read connected-front morphology render"), Camera->ReadLastCapture(Pixels, Width, Height, CapturedTick));
        WarmMetrics[Mode] = MeasureWarmIslands(Pixels, Width, Height);
        const float LargestFraction = WarmMetrics[Mode].Total > 0
            ? float(WarmMetrics[Mode].Largest) / WarmMetrics[Mode].Total : 0.0f;
        AddInfo(FString::Printf(TEXT("Connected-front pixel AB %s: %d warm islands, %d warm pixels"),
            Mode == 0 ? TEXT("legacy") : TEXT("connected"), WarmMetrics[Mode].Islands, WarmMetrics[Mode].Total));
        AddInfo(FString::Printf(TEXT("Connected-front pixel AB %s: largest warm island %d pixels (%.3f of warm area)"),
            Mode == 0 ? TEXT("legacy") : TEXT("connected"), WarmMetrics[Mode].Largest, LargestFraction));
        Fire->EndYoloCapture();
    }
    TestTrue(TEXT("Legacy render contains warm flame pixels"), WarmMetrics[0].Total > 0);
    TestTrue(TEXT("Connected render contains warm flame pixels"), WarmMetrics[1].Total > 0);
    const float LegacyLargestFraction = WarmMetrics[0].Total > 0
        ? float(WarmMetrics[0].Largest) / WarmMetrics[0].Total : 0.0f;
    const float ConnectedLargestFraction = WarmMetrics[1].Total > 0
        ? float(WarmMetrics[1].Largest) / WarmMetrics[1].Total : 0.0f;
    TestTrue(TEXT("Connected mode grows the largest joined warm-flame area fraction"),
        ConnectedLargestFraction > LegacyLargestFraction);
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    FlushRenderingCommands();
    return true;
}
#endif
