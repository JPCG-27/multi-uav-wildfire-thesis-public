#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../Human/HumanVisualPose.h"
#include "Engine/SkeletalMesh.h"
#include "../Human/HumanManager.h"
#include "../World/Grid.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/TextureRenderTarget2D.h"
#include "TextureResource.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "RenderingThread.h"
#if WITH_EDITOR
#include "AssetCompilingManager.h"
#endif

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHumanVisualPoseTest, "Thesis.Humans.VisualPoses",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FHumanVisualPoseTest::RunTest(const FString& Parameters)
{
    const TCHAR* Meshes[] = {
        TEXT("/Game/Mixamo/Characters/SK_Mixamo_00.SK_Mixamo_00"),
        TEXT("/Game/Mixamo/Characters/SK_Mixamo_01.SK_Mixamo_01"),
        TEXT("/Game/Mixamo/Characters/SK_Mixamo_02.SK_Mixamo_02"),
        TEXT("/Game/NiagaraExamples/Gallery/SkeletalMesh/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple"),
        TEXT("/Game/NiagaraExamples/Gallery/SkeletalMesh/Mannequins/Meshes/SKM_Quinn_Simple.SKM_Quinn_Simple") };
    for (const TCHAR* Path : Meshes)
    {
        USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, Path);
        if (!TestNotNull(Path, Mesh)) continue;
        FHumanVisualRig Rig;
        Rig.Initialize(Mesh->GetRefSkeleton());
        TestTrue(FString::Printf(TEXT("%s supported limb rig"), *Mesh->GetName()), Rig.bSupported);
        if (!Rig.bSupported) continue;
        for (uint8 P = 0; P < static_cast<uint8>(EHumanVisualPose::Count); ++P)
        {
            const EHumanVisualPose Pose = static_cast<EHumanVisualPose>(P);
            TArray<FTransform> First, Repeated, Unrelated;
            FBox Bounds, RepeatedBounds, OtherBounds;
            Rig.Evaluate(Pose, 3.6, 9340, First, Bounds);
            Rig.Evaluate(EHumanVisualPose::Running, 19.8, 19, Unrelated, OtherBounds);
            Rig.Evaluate(Pose, 3.6, 9340, Repeated, RepeatedBounds);
            TestEqual(TEXT("Preserve skeleton bone count"), First.Num(), Rig.Reference.Num());
            TestTrue(TEXT("Valid contact bounds"), Bounds.IsValid != 0);
            for (int32 I = 0; I < First.Num(); ++I)
            {
                TestFalse(TEXT("Finite transforms"), First[I].ContainsNaN());
                TestTrue(TEXT("Tick replay independent of evaluation history"), First[I].Equals(Repeated[I], 0.0));
                TestTrue(TEXT("No bone scaling"), First[I].GetScale3D().Equals(Rig.Reference[I].GetScale3D(), 0.0));
            }
            const FTransform& Arm = First[Rig.UpperArm[0]];
            TestFalse(TEXT("Pose is not the reference T/A pose"), Arm.Equals(Rig.Reference[Rig.UpperArm[0]], 1.e-5));
        }
        TArray<FTransform> Standing, Crouched, WalkingA, WalkingB;
        FBox StandingBounds, CrouchBounds, A, B;
        Rig.Evaluate(EHumanVisualPose::Standing, 0, 1, Standing, StandingBounds);
        Rig.Evaluate(EHumanVisualPose::Crouching, 0, 1, Crouched, CrouchBounds);
        Rig.Evaluate(EHumanVisualPose::Walking, 0.1, 1, WalkingA, A);
        Rig.Evaluate(EHumanVisualPose::Walking, 0.4, 1, WalkingB, B);
        TestTrue(TEXT("Crouch reduces ground-to-head height"), CrouchBounds.GetSize().Z < StandingBounds.GetSize().Z);
        TestFalse(TEXT("Walking phase changes the leg"), WalkingA[Rig.Thigh[0]].Equals(WalkingB[Rig.Thigh[0]], 1.e-5));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHumanPoseGalleryTest, "Thesis.Humans.PoseGallery",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FHumanPoseGalleryTest::RunTest(const FString& Parameters)
{
    const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("HumanPoseGallery"));
    IFileManager::Get().MakeDirectory(*Directory, true);
    const FString Csv = FPaths::Combine(Directory, TEXT("human.csv"));
    FFileHelper::SaveStringToFile(TEXT("tick,person,x,y,group,state\n0,0,1,1,0,0\n1,0,1,1,0,0\n"), *Csv);
    const UWorld::InitializationValues Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, FName(TEXT("HumanPoseGallery")),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    for (int32 I = 0; I < 2; ++I)
    {
        ADirectionalLight* Light = World->SpawnActor<ADirectionalLight>();
        Light->SetActorRotation(FRotator(-35.0, I == 0 ? 150.0 : -40.0, 0));
        Light->GetLightComponent()->SetIntensity(I == 0 ? 4.0f : 1.5f);
    }
    AActor* Camera = World->SpawnActor<AActor>();
    USceneCaptureComponent2D* Capture = NewObject<USceneCaptureComponent2D>(Camera);
    Camera->SetRootComponent(Capture);
    Capture->RegisterComponent();
    Capture->bCaptureEveryFrame = false;
    Capture->bCaptureOnMovement = false;
    Capture->CaptureSource = SCS_FinalColorLDR;
    Capture->FOVAngle = 35;
    Capture->ShowFlags.SetTemporalAA(false);
    Capture->ShowFlags.SetMotionBlur(false);
    Capture->PostProcessSettings.bOverride_AutoExposureMethod = true;
    Capture->PostProcessSettings.AutoExposureMethod = AEM_Manual;
    Capture->PostProcessSettings.bOverride_AutoExposureApplyPhysicalCameraExposure = true;
    Capture->PostProcessSettings.AutoExposureApplyPhysicalCameraExposure = false;
    Capture->TextureTarget = NewObject<UTextureRenderTarget2D>(Capture);
    Capture->TextureTarget->InitCustomFormat(512, 512, PF_B8G8R8A8, false);
    Capture->TextureTarget->UpdateResourceImmediate(true);
    const FVector CameraPosition(930, 360, 200);
    Capture->SetWorldLocationAndRotation(CameraPosition, (FVector(300, 300, 85) - CameraPosition).Rotation());
    Grid Truth(4, 4);
    const TCHAR* Meshes[] = {
        TEXT("/Game/Mixamo/Characters/SK_Mixamo_00.SK_Mixamo_00"),
        TEXT("/Game/Mixamo/Characters/SK_Mixamo_01.SK_Mixamo_01"),
        TEXT("/Game/Mixamo/Characters/SK_Mixamo_02.SK_Mixamo_02"),
        TEXT("/Game/NiagaraExamples/Gallery/SkeletalMesh/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple"),
        TEXT("/Game/NiagaraExamples/Gallery/SkeletalMesh/Mannequins/Meshes/SKM_Quinn_Simple.SKM_Quinn_Simple") };
    for (const TCHAR* MeshPath : Meshes)
    {
        USkeletalMesh* LoadedMesh = LoadObject<USkeletalMesh>(nullptr, MeshPath);
#if WITH_EDITOR
        FAssetCompilingManager::Get().FinishAllCompilation();
#endif
        for (uint8 P = 0; P < static_cast<uint8>(EHumanVisualPose::Count); ++P)
        {
            AHumanManager* Manager = World->SpawnActor<AHumanManager>();
            Manager->HumanMesh = LoadedMesh;
            Manager->SetGridFrame(FVector::ZeroVector, 300.0f);
            if (!TestNotNull(TEXT("Gallery mesh"), Manager->HumanMesh.Get()) || !TestTrue(TEXT("Gallery trajectory"), Manager->LoadTrajectories(Csv)))
            {
                Manager->Destroy();
                continue;
            }
            Manager->VisualPoseMode = FHumanVisualRig::Name(static_cast<EHumanVisualPose>(P));
            Manager->ApplyTick(1, &Truth);
            World->SendAllEndOfFrameUpdates();
            Capture->CaptureScene();
            FlushRenderingCommands();
            TArray<FColor> Pixels;
            TestTrue(TEXT("Gallery pixels"), Capture->TextureTarget->GameThread_GetRenderTargetResource()->ReadPixels(Pixels));
            if (Pixels.Num() != 512 * 512)
            {
                Manager->Destroy();
                continue;
            }
            TArray64<uint8> Png;
            FImageUtils::PNGCompressImageArray(512, 512, Pixels, Png);
            const FString Path = FPaths::Combine(Directory, Manager->HumanMesh->GetName() + TEXT("_") + Manager->VisualPoseMode + TEXT(".png"));
            TestTrue(TEXT("Gallery saved"), FFileHelper::SaveArrayToFile(Png, *Path));
            Manager->Destroy();
            World->SendAllEndOfFrameUpdates();
            FlushRenderingCommands();
        }
    }
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHumanVisualPoseDeferralTest, "Thesis.Humans.VisualPoseDeferral",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FHumanVisualPoseDeferralTest::RunTest(const FString& Parameters)
{
    USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr,
        TEXT("/Game/Mixamo/Characters/SK_Mixamo_00.SK_Mixamo_00"));
    if (!Mesh)
        Mesh = LoadObject<USkeletalMesh>(nullptr,
            TEXT("/Game/NiagaraExamples/Gallery/SkeletalMesh/Mannequins/Meshes/SKM_Manny_Simple.SKM_Manny_Simple"));
    if (!TestNotNull(TEXT("Human pose test mesh"), Mesh)) return true;

    const FString Directory = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("HumanPoseDeferralTest"));
    IFileManager::Get().MakeDirectory(*Directory, true);
    const FString FirstCsv = FPaths::Combine(Directory, TEXT("first.csv"));
    const FString SecondCsv = FPaths::Combine(Directory, TEXT("second.csv"));
    FFileHelper::SaveStringToFile(TEXT("tick,person,x,y,group,state\n0,0,1,1,0,0\n1,0,2,1,0,0\n"), *FirstCsv);
    FFileHelper::SaveStringToFile(TEXT("tick,person,x,y,group,state\n0,0,1,1,0,0\n1,0,4,1,0,0\n"), *SecondCsv);

    const UWorld::InitializationValues Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, FName(TEXT("HumanPoseDeferralTest")),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    Grid Truth(8, 8);

    AHumanManager* Direct = World->SpawnActor<AHumanManager>();
    Direct->HumanMesh = Mesh;
    Direct->SetGridFrame(FVector::ZeroVector, 300.0f);
    TestTrue(TEXT("Direct trajectory loaded"), Direct->LoadTrajectories(FirstCsv));
    TestFalse(TEXT("Standalone manager defaults to per-tick visuals"), Direct->bDeferVisualUpdatesUntilCapture);
    Direct->ApplyTick(0, &Truth);
    TestEqual(TEXT("Direct path updates at tick zero"), Direct->GetLastVisualUpdateRequestTick(), 0);
    Direct->ApplyTick(1, &Truth);
    Direct->ApplyTick(3, &Truth); // Beyond Data.MaxTick: held final state, actual request tick remains 3.
    TestEqual(TEXT("Direct path records actual post-trajectory tick"), Direct->GetLastVisualUpdateRequestTick(), 3);

    AHumanManager* Deferred = World->SpawnActor<AHumanManager>();
    Grid DeferredTruth(8, 8);
    Deferred->HumanMesh = Mesh;
    Deferred->SetGridFrame(FVector::ZeroVector, 300.0f);
    Deferred->bDeferVisualUpdatesUntilCapture = true;
    TestTrue(TEXT("Deferred trajectory loaded"), Deferred->LoadTrajectories(FirstCsv));
    Deferred->ApplyTick(0, &DeferredTruth);
    TestEqual(TEXT("Deferred path leaves pose untouched before capture"), Deferred->GetLastVisualUpdateRequestTick(), INDEX_NONE);
    Deferred->UpdateVisualPoseForCapture();
    TestEqual(TEXT("Capture publishes the currently applied state"), Deferred->GetLastVisualUpdateRequestTick(), 0);
    Deferred->ApplyTick(1, &DeferredTruth);
    TestEqual(TEXT("ApplyTick still leaves deferred pose alone"), Deferred->GetLastVisualUpdateRequestTick(), 0);
    Deferred->UpdateVisualPoseForCapture();
    TestEqual(TEXT("Capture publishes the next applied state"), Deferred->GetLastVisualUpdateRequestTick(), 1);
    Deferred->ApplyTick(3, &DeferredTruth);
    Deferred->UpdateVisualPoseForCapture();
    TestEqual(TEXT("Deferred path tracks requested tick after trajectory end"), Deferred->GetLastVisualUpdateRequestTick(), 3);
    Deferred->UpdateVisualPoseForCapture();
    TestEqual(TEXT("Repeated same-tick camera does not republish pose"), Deferred->GetLastVisualUpdateRequestTick(), 3);

    TArray<FHumanVisualCaptureRecord> DirectRecords, DeferredRecords;
    int32 DirectPoseTick = INDEX_NONE, DeferredPoseTick = INDEX_NONE;
    Direct->GetLastVisualCaptureRecords(DirectRecords, DirectPoseTick);
    Deferred->GetLastVisualCaptureRecords(DeferredRecords, DeferredPoseTick);
    TestTrue(TEXT("Direct and deferred renders use the same clamped trajectory pose"),
        DirectRecords.Num() == 1 && DeferredRecords.Num() == 1 &&
        DirectPoseTick == DeferredPoseTick && DirectRecords[0].Pose == DeferredRecords[0].Pose &&
        DirectRecords[0].WorldTransform.Equals(DeferredRecords[0].WorldTransform, 0.0f));

    TArray<FHumanVisualCaptureRecord> BeforeReset, AfterReset;
    int32 BeforePoseTick = INDEX_NONE, AfterPoseTick = INDEX_NONE;
    Deferred->GetLastVisualCaptureRecords(BeforeReset, BeforePoseTick);
    TestTrue(TEXT("Deferred pose record exists before reset"), BeforeReset.Num() == 1);
    TestTrue(TEXT("Reload updated"), Deferred->LoadTrajectories(SecondCsv));
    Deferred->ResetReplay(&DeferredTruth);
    Deferred->ApplyTick(1, &DeferredTruth);
    TestEqual(TEXT("Reset clears prior requested visual tick"), Deferred->GetLastVisualUpdateRequestTick(), INDEX_NONE);
    Deferred->UpdateVisualPoseForCapture();
    Deferred->GetLastVisualCaptureRecords(AfterReset, AfterPoseTick);
    TestTrue(TEXT("Same-tick reset publishes the new trajectory pose"), AfterReset.Num() == 1 &&
        BeforeReset.Num() == 1 && !AfterReset[0].WorldTransform.GetLocation().Equals(
            BeforeReset[0].WorldTransform.GetLocation(), 1.0f));

    Direct->Destroy();
    Deferred->Destroy();
    World->SendAllEndOfFrameUpdates();
    FlushRenderingCommands();
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    return true;
}
#endif
