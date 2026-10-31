#include "HumanManager.h"
#include "Grid.h"
#include "ProceduralTerrain.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "Engine/SkeletalMesh.h"
#include "Components/PoseableMeshComponent.h"
#include "Misc/PackageName.h"

AHumanManager::AHumanManager()
{
    PrimaryActorTick.bCanEverTick = true;     // debug draw only — sim state changes happen in ApplyTick
    PrimaryActorTick.bStartWithTickEnabled = true;

    USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    SetRootComponent(Root);
}

float AHumanManager::GroundZAt(float CellX, float CellY) const
{
    if (const AProceduralTerrain* T = Terrain.Get())
    {
        return T->GetHeightAtCell(CellX, CellY);
    }
    return GridOriginWorld.Z;
}

void AHumanManager::SetVisualsCaptureOnly(bool bCaptureOnly)
{
    bHumanMeshesCaptureOnly = bCaptureOnly;
    for (const TObjectPtr<UPoseableMeshComponent>& C : PersonMeshes)
    {
        if (C)
        {
            C->SetVisibleInSceneCaptureOnly(bCaptureOnly);
        }
    }
}

void AHumanManager::UpdateVisualPoseForCapture()
{
    if (bUpdateVisualMeshes && LastAppliedTick != INDEX_NONE && LastVisualUpdateRequestTick != LastAppliedTick)
    {
        UpdatePersonMeshes(LastAppliedTick);
    }
}

bool AHumanManager::TryAutoLoadHumanMeshes()
{
    if (HumanMeshes.Num() > 0 || HumanMesh) return true; // Explicit overrides win.
    if (bTriedAutomaticMeshes) return false;
    bTriedAutomaticMeshes = true;
    if (VisualMeshSet != TEXT("mixed") && VisualMeshSet != TEXT("mixamo") && VisualMeshSet != TEXT("mannequin"))
    {
        UE_LOG(LogTemp, Fatal, TEXT("Invalid human_visual_mesh_set '%s': use mixed, mixamo, or mannequin."), *VisualMeshSet);
        return false;
    }
    auto TryMesh = [&](const FString& Package) -> bool
    {
        if (!FPackageName::DoesPackageExist(Package)) return false;
        const FString Path = Package + TEXT(".") + FPackageName::GetShortName(Package);
        if (USkeletalMesh* Mesh = LoadObject<USkeletalMesh>(nullptr, *Path))
        {
            HumanMeshes.AddUnique(Mesh);
            return true;
        }
        return false;
    };
    if (bAutoUseMixamoIfAvailable && VisualMeshSet != TEXT("mannequin"))
    {
        for (int32 I = 0; I < 32; ++I)
            TryMesh(FString::Printf(TEXT("/Game/Mixamo/Characters/SK_Mixamo_%02d"), I));
    }
    if (bAutoUseMannequinIfAvailable && VisualMeshSet != TEXT("mixamo"))
    {
        // Support both the Third Person pack and the UE mannequins already
        // shipped in this project's Niagara examples. Never duplicate a type.
        if (!TryMesh(TEXT("/Game/Characters/Mannequins/Meshes/SKM_Manny")))
            TryMesh(TEXT("/Game/NiagaraExamples/Gallery/SkeletalMesh/Mannequins/Meshes/SKM_Manny_Simple"));
        if (!TryMesh(TEXT("/Game/Characters/Mannequins/Meshes/SKM_Quinn")))
            TryMesh(TEXT("/Game/NiagaraExamples/Gallery/SkeletalMesh/Mannequins/Meshes/SKM_Quinn_Simple"));
    }
    UE_LOG(LogTemp, Log, TEXT("Thesis humans: visual set=%s loaded %d meshes; pose mode=%s."),
        *VisualMeshSet, HumanMeshes.Num(), *VisualPoseMode);
    if (HumanMeshes.IsEmpty())
        UE_LOG(LogTemp, Error, TEXT("No human meshes available for '%s'. Supply HumanMesh/HumanMeshes or install the character content."), *VisualMeshSet);
    return !HumanMeshes.IsEmpty();
}

USkeletalMesh* AHumanManager::MeshForPerson(int32 PersonId) const
{
    if (HumanMeshes.Num() > 0)
    {
        const int32 MeshIdx = FMath::Abs(PersonId) % HumanMeshes.Num();
        return HumanMeshes[MeshIdx].Get();
    }
    return HumanMesh.Get();
}

const FHumanVisualRig& AHumanManager::GetVisualMeshPose(USkeletalMesh* Mesh)
{
    if (const FHumanVisualRig* Existing = VisualMeshPoses.Find(Mesh)) return *Existing;
    FHumanVisualRig Rig;
    Rig.Initialize(Mesh->GetRefSkeleton());
    if (!Rig.bSupported)
        UE_LOG(LogTemp, Warning, TEXT("Human pose rig %s has missing limb bones; unsupported limbs retain reference transforms."), *Mesh->GetPathName());
    return VisualMeshPoses.Add(Mesh, MoveTemp(Rig));
}

void AHumanManager::BuildVisualTrajectoryIndex()
{
    PersonKeyframes.SetNum(Data.NumPersons);
    for (TArray<int32>& Indices : PersonKeyframes) Indices.Reset();
    KeyframeHeadings.SetNum(Data.Keyframes.Num());
    KeyframeDistancesPerTick.Init(0.0f, Data.Keyframes.Num());
    for (int32 I = 0; I < Data.Keyframes.Num(); ++I)
    {
        const FHumanKeyframe& K = Data.Keyframes[I];
        if (!PersonKeyframes.IsValidIndex(K.PersonId)) continue;
        TArray<int32>& Indices = PersonKeyframes[K.PersonId];
        float Heading = static_cast<float>((static_cast<uint32>(K.PersonId) * 47u + static_cast<uint32>(VisualSeed)) % 360u);
        if (!Indices.IsEmpty())
        {
            const int32 Previous = Indices.Last();
            const FHumanKeyframe& P = Data.Keyframes[Previous];
            Heading = KeyframeHeadings[Previous];
            const FVector2f Delta(K.X - P.X, K.Y - P.Y);
            if (Delta.SizeSquared() > UE_SMALL_NUMBER && K.Tick > P.Tick)
            {
                Heading = FMath::RadiansToDegrees(FMath::Atan2(Delta.Y, Delta.X));
                KeyframeDistancesPerTick[I] = Delta.Size() / (K.Tick - P.Tick);
            }
        }
        KeyframeHeadings[I] = Heading;
        Indices.Add(I);
    }
}

void AHumanManager::ResetVisualReplay()
{
    LastVisualPoseTick = INDEX_NONE;
    LastVisualUpdateRequestTick = INDEX_NONE;
    CurrentVisualPoseNames.Reset();
    CurrentVisualMeshPaths.Reset();
    for (const TObjectPtr<UPoseableMeshComponent>& C : PersonMeshes)
    {
        if (C) C->SetVisibility(false);
    }
}

void AHumanManager::UpdatePersonMeshes(int32 Tick)
{
    if (!TryAutoLoadHumanMeshes()) return;
    EHumanVisualPose ForcedPose = EHumanVisualPose::Standing;
    const bool bAutomatic = VisualPoseMode == TEXT("automatic");
    const bool bVaried = VisualPoseMode == TEXT("varied");
    if (!bAutomatic && !bVaried && !FHumanVisualRig::Parse(VisualPoseMode, ForcedPose))
    {
        UE_LOG(LogTemp, Fatal, TEXT("Invalid human_pose_mode '%s'."), *VisualPoseMode);
        return;
    }
    const int32 VisualTick = FMath::Clamp(Tick, 0, Data.MaxTick);
    const double SimSeconds = static_cast<double>(VisualTick) * VisualSecondsPerTick;
    LastVisualPoseTick = VisualTick;
    LastVisualUpdateRequestTick = Tick;
    CurrentVisualPoseNames.SetNum(Current.Num());
    CurrentVisualMeshPaths.SetNum(Current.Num());
    while (PersonMeshes.Num() < Current.Num())
    {
        UPoseableMeshComponent* C = NewObject<UPoseableMeshComponent>(this);
        C->RegisterComponent();
        C->AttachToComponent(GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        C->SetVisibleInSceneCaptureOnly(bHumanMeshesCaptureOnly);
        C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        C->SetCanEverAffectNavigation(false);
        // Bones are prepared synchronously below. No frame-time animation or
        // visibility-dependent animation update may change a sensor image.
        C->SetComponentTickEnabled(false);
        C->VisibilityBasedAnimTickOption = EVisibilityBasedAnimTickOption::AlwaysTickPoseAndRefreshBones;
        PersonMeshes.Add(C);
    }


    for (int32 i = 0; i < PersonMeshes.Num(); ++i)
    {
        UPoseableMeshComponent* C = PersonMeshes[i].Get();
        if (!C) continue;

        if (!Current.IsValidIndex(i) || Current[i].PersonId == INDEX_NONE)
        {
            C->SetVisibility(false);
            CurrentVisualPoseNames[i].Reset();
            CurrentVisualMeshPaths[i].Reset();
            continue;
        }

        const FHumanSnapshot& S = Current[i];
        USkeletalMesh* Mesh = MeshForPerson(S.PersonId);
        if (!Mesh)
        {
            C->SetVisibility(false);
            CurrentVisualPoseNames[i].Reset();
            CurrentVisualMeshPaths[i].Reset();
            continue;
        }
        const FHumanVisualRig& Rig = GetVisualMeshPose(Mesh);
        if (C->GetSkinnedAsset() != Mesh)
        {
            C->SetSkinnedAssetAndUpdate(Mesh);
        }

        FVector Pos(
            GridOriginWorld.X + S.PosCell.X * CellSizeUU,
            GridOriginWorld.Y + S.PosCell.Y * CellSizeUU,
            GroundZAt(S.PosCell.X, S.PosCell.Y));

        // Binary search the immutable replay index, not last-rendered state:
        // skipping captures, reset, and revisiting a tick cannot change a pose.
        const TArray<int32>& Indices = PersonKeyframes[S.PersonId];
        int32 Low = 0, High = Indices.Num();
        while (Low < High)
        {
            const int32 Mid = Low + (High - Low) / 2;
            if (Data.Keyframes[Indices[Mid]].Tick <= VisualTick) Low = Mid + 1;
            else High = Mid;
        }
        const int32 Key = Low > 0 ? Indices[Low - 1] : INDEX_NONE;
        const float Heading = Key != INDEX_NONE ? KeyframeHeadings[Key] : 0.0f;
        const float Speed = Key != INDEX_NONE
            ? KeyframeDistancesPerTick[Key] * CellSizeUU / FMath::Max(VisualSecondsPerTick, 0.001f) : 0.0f;
        const uint32 PersonSeed = static_cast<uint32>(VisualSeed) * 196613u + static_cast<uint32>(S.PersonId) * 3145739u;
        EHumanVisualPose Selected = ForcedPose;
        if (bVaried) Selected = static_cast<EHumanVisualPose>(static_cast<uint32>(S.PersonId) % static_cast<uint32>(EHumanVisualPose::Count));
        if (bAutomatic)
        {
            if (S.State == EHumanState::Dead) Selected = EHumanVisualPose::Lying;
            else if (S.State == EHumanState::Immobile) Selected = (PersonSeed % 2) ? EHumanVisualPose::Seated : EHumanVisualPose::Lying;
            else if (Speed > 1.0f) Selected = Speed > 180.0f && S.State == EHumanState::Healthy
                ? EHumanVisualPose::Running : EHumanVisualPose::Walking;
            else
            {
                const EHumanVisualPose Rest[] = { EHumanVisualPose::Standing, EHumanVisualPose::Waving,
                    EHumanVisualPose::Crouching, EHumanVisualPose::Seated };
                Selected = Rest[PersonSeed % UE_ARRAY_COUNT(Rest)];
            }
        }
        CurrentVisualPoseNames[i] = FHumanVisualRig::Name(Selected);
        CurrentVisualMeshPaths[i] = Mesh->GetPathName();
        FBox BoneBounds(ForceInit);
        Rig.Evaluate(Selected, SimSeconds, PersonSeed, C->BoneSpaceTransforms, BoneBounds);
        C->MarkRefreshTransformDirty();
        C->RefreshBoneTransforms();
        // The mesh may have been registered in this same engine frame. Its
        // initial reference-pose revision must not alias the posed GPU buffer.
        // Captures deliberately have no inter-frame skeletal motion blur.
        C->ClearMotionVector();
        const FQuat Facing(FVector::UpVector, FMath::DegreesToRadians(Heading - Rig.ForwardYaw));
        const FQuat Recline = Selected == EHumanVisualPose::Lying
            ? FQuat(Rig.Right, UE_PI / 2.0) : FQuat::Identity;
        const FQuat Rot = Facing * Recline;
        // Skeletal contact estimate, never used as a detector training label.
        const FBox ContactBounds = BoneBounds.IsValid ? BoneBounds.ExpandBy(8.0f) : Mesh->GetBounds().GetBox();
        const FBox RotatedBounds = ContactBounds.TransformBy(
            FTransform(Rot, FVector::ZeroVector, FVector(HumanMeshScale)));
        Pos.Z -= RotatedBounds.Min.Z;

        C->SetVisibility(true);
        C->SetWorldLocationAndRotation(Pos, Rot);
        C->SetWorldScale3D(FVector(HumanMeshScale));
        C->UpdateBounds();
        C->DoDeferredRenderUpdates_Concurrent();
    }
}

void AHumanManager::GetLastVisualCaptureRecords(TArray<FHumanVisualCaptureRecord>& OutRecords, int32& OutPoseTick) const
{
    OutRecords.Reset();
    OutPoseTick = LastVisualPoseTick;
    for (int32 PersonId = 0; PersonId < Current.Num(); ++PersonId)
    {
        const FHumanSnapshot& Snapshot = Current[PersonId];
        const UPoseableMeshComponent* Component = PersonMeshes.IsValidIndex(PersonId) ? PersonMeshes[PersonId].Get() : nullptr;
        if (Snapshot.PersonId == INDEX_NONE || !Component || !Component->IsVisible() ||
            !CurrentVisualPoseNames.IsValidIndex(PersonId) || CurrentVisualPoseNames[PersonId].IsEmpty())
        {
            continue;
        }
        FHumanVisualCaptureRecord& Record = OutRecords.AddDefaulted_GetRef();
        Record.PersonId = Snapshot.PersonId;
        Record.State = Snapshot.State;
        Record.Pose = CurrentVisualPoseNames[PersonId];
        Record.MeshPath = CurrentVisualMeshPaths.IsValidIndex(PersonId) ? CurrentVisualMeshPaths[PersonId] : FString();
        Record.WorldTransform = Component->GetComponentTransform();
        Record.Component = Component;
    }
}

bool AHumanManager::LoadTrajectories(const FString& CsvPath)
{
    FHumanTrajectoryImporter::FResult NewData;
    if (!FHumanTrajectoryImporter::ImportFromCSV(CsvPath, NewData))
    {
        return false;
    }

    Data = MoveTemp(NewData);
    BuildVisualTrajectoryIndex();
    ResetVisualReplay();
    LastAppliedTick = INDEX_NONE;
    Current.Reset();
    LastWrittenCells.Reset();
    AppearRecorded.Init(false, Data.NumPersons);
    return true;
}

void AHumanManager::GetSnapshotsForTick(int32 Tick, TArray<FHumanSnapshot>& OutSnapshots) const
{
    OutSnapshots.Reset();
    if (Data.Keyframes.Num() == 0 || Data.NumPersons <= 0 || Data.TickStartIndex.Num() == 0)
    {
        return;
    }

    OutSnapshots.SetNum(Data.NumPersons);
    const int32 ClampedTick = FMath::Clamp(Tick, 0, Data.MaxTick);

    // Start from the current replay cache when it is usable; then overlay the
    // requested tick's keyframes. This preserves held poses after the CSV ends.
    if (Current.Num() == Data.NumPersons)
    {
        OutSnapshots = Current;
    }

    const int32 RangeStart = Data.TickStartIndex[ClampedTick];
    const int32 RangeEnd = Data.TickStartIndex[ClampedTick + 1];
    for (int32 I = RangeStart; I < RangeEnd; ++I)
    {
        const FHumanKeyframe& K = Data.Keyframes[I];
        if (!OutSnapshots.IsValidIndex(K.PersonId))
        {
            continue;
        }
        FHumanSnapshot& S = OutSnapshots[K.PersonId];
        S.PersonId = K.PersonId;
        S.PosCell = FVector2f(K.X, K.Y);
        S.GroupId = K.GroupId;
        S.State = K.State;
    }
}

void AHumanManager::ResetReplay(Grid* SharedGrid)
{
    ResetVisualReplay();
    LastAppliedTick = INDEX_NONE;
    // Zero anything we wrote; Grid::Initialize() may already have wiped it,
    // but this keeps the manager correct regardless of call order.
    if (SharedGrid)
    {
        for (const int32 Cell : LastWrittenCells)
        {
            const int32 X = Cell % SharedGrid->Width;
            const int32 Y = Cell / SharedGrid->Width;
            SharedGrid->setRealPriorityAndNumHumans(X, Y, 0, 0);
        }
    }
    LastWrittenCells.Reset();
    Current.Reset();
    AppearRecorded.Init(false, Data.NumPersons);
}

void AHumanManager::ApplyTick(int32 Tick, Grid* SharedGrid)
{
    if (!SharedGrid || Data.Keyframes.Num() == 0)
    {
        return;
    }

    const int32 W = SharedGrid->Width;
    const int32 H = SharedGrid->Height;

    // 1. Clear last tick's counts (dirty list — never a grid scan).
    for (const int32 Cell : LastWrittenCells)
    {
        const int32 X = Cell % W;
        const int32 Y = Cell / W;
        SharedGrid->setRealPriorityAndNumHumans(X, Y, 0, 0);
    }
    LastWrittenCells.Reset();

    // 2. Keyframe range for this tick. After the trajectory ends, hold the
    //    final tick's poses (persons stay where they ended up).
    const int32 ClampedTick = FMath::Clamp(Tick, 0, Data.MaxTick);
    const int32 RangeStart = Data.TickStartIndex[ClampedTick];
    const int32 RangeEnd   = Data.TickStartIndex[ClampedTick + 1];

    if (RangeEnd > RangeStart)
    {
        if (Current.Num() < Data.NumPersons)
        {
            Current.SetNum(Data.NumPersons);
        }
        for (int32 i = RangeStart; i < RangeEnd; ++i)
        {
            const FHumanKeyframe& K = Data.Keyframes[i];
            if (K.PersonId < 0 || K.PersonId >= Data.NumPersons)
            {
                continue;
            }
            FHumanSnapshot& S = Current[K.PersonId];
            S.PersonId = K.PersonId;
            S.PosCell = FVector2f(K.X, K.Y);
            S.GroupId = K.GroupId;
            S.State = K.State;
        }
    }

    // 3. Write per-cell alive counts (ground truth). Deterministic: snapshot
    //    order is PersonId order; counts are order-independent anyway.
    //    Accumulate per cell first to write each cell once.
    TMap<int32, int32> CellCounts;
    CellCounts.Reserve(Current.Num());
    for (const FHumanSnapshot& S : Current)
    {
        if (S.PersonId == INDEX_NONE || S.State == EHumanState::Dead)
        {
            continue;   // bodies are not SAR person targets
        }
        const int32 X = FMath::Clamp(static_cast<int32>(S.PosCell.X), 0, W - 1);
        const int32 Y = FMath::Clamp(static_cast<int32>(S.PosCell.Y), 0, H - 1);
        CellCounts.FindOrAdd(Y * W + X) += 1;
    }

    LastWrittenCells.Reserve(CellCounts.Num());
    for (const TPair<int32, int32>& Pair : CellCounts)
    {
        const int32 X = Pair.Key % W;
        const int32 Y = Pair.Key / W;
        SharedGrid->setRealPriorityAndNumHumans(X, Y, 0, Pair.Value);
        LastWrittenCells.Add(Pair.Key);
    }

    // 4. Visual representation (skeletal meshes for drone captures). Skipped when
    //    nothing renders them (headless/no-capture runs) -- pure visual work.
    LastAppliedTick = Tick;
    if (bUpdateVisualMeshes && !bDeferVisualUpdatesUntilCapture)
    {
        UpdatePersonMeshes(LastAppliedTick);
    }
}

void AHumanManager::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);

    if (!bDrawDebugHumans || Current.Num() == 0)
    {
        return;
    }

    UWorld* World = GetWorld();
    if (!World)
    {
        return;
    }

    for (const FHumanSnapshot& S : Current)
    {
        if (S.PersonId == INDEX_NONE)
        {
            continue;
        }

        FColor Color;
        switch (S.State)
        {
            case EHumanState::Healthy:  Color = FColor::Green;  break;
            case EHumanState::Injured:  Color = FColor::Yellow; break;
            case EHumanState::Immobile: Color = FColor::Orange; break;
            default:                    Color = FColor::Black;  break;
        }

        const FVector Pos(
            GridOriginWorld.X + S.PosCell.X * CellSizeUU,
            GridOriginWorld.Y + S.PosCell.Y * CellSizeUU,
            GroundZAt(S.PosCell.X, S.PosCell.Y) + DebugSphereZOffsetUU);

        DrawDebugSphere(World, Pos, DebugSphereRadiusUU, 8, Color, false, -1.0f, 0, 2.0f);
    }
}
