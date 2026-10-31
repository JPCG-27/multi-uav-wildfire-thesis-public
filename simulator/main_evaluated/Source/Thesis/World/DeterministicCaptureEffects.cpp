#include "DeterministicCaptureEffects.h"

#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialExpressionVertexColor.h"
#include "Materials/MaterialExpressionScalarParameter.h"
#include "Materials/MaterialExpressionMultiply.h"
#include "Materials/MaterialExpressionConstant.h"
#include "ShaderCompiler.h"
#include "UObject/StrongObjectPtr.h"
#include "ProceduralMeshComponent.h"
#include "UObject/Package.h"

namespace
{
    uint32 MixBits(uint32 V)
    {
        V ^= V >> 16;
        V *= 0x7FEB352Du;
        V ^= V >> 15;
        V *= 0x846CA68Bu;
        return V ^ (V >> 16);
    }

    float UnitFromHash(uint32 V)
    {
        return static_cast<float>(MixBits(V) >> 8) * (1.0f / 16777215.0f);
    }

    UMaterial* CreateCaptureMaterial()
    {
#if WITH_EDITOR
        static TStrongObjectPtr<UMaterial> Cached;
        if (Cached.IsValid()) return Cached.Get();

        UMaterial* Material = NewObject<UMaterial>(GetTransientPackage(), NAME_None, RF_Transient);
        Material->BlendMode = BLEND_Translucent;
        Material->SetShadingModel(MSM_Unlit);
        Material->TwoSided = true;

        UMaterialExpressionVertexColor* VertexColor =
            NewObject<UMaterialExpressionVertexColor>(Material, NAME_None, RF_Transient);
        UMaterialExpressionScalarParameter* Intensity =
            NewObject<UMaterialExpressionScalarParameter>(Material, NAME_None, RF_Transient);
        Intensity->ParameterName = TEXT("CaptureIntensity");
        Intensity->DefaultValue = 1.0f;
        UMaterialExpressionConstant* FlameGain =
            NewObject<UMaterialExpressionConstant>(Material, NAME_None, RF_Transient);
        FlameGain->R = 5.0f;
        UMaterialExpressionMultiply* EmissiveScale =
            NewObject<UMaterialExpressionMultiply>(Material, NAME_None, RF_Transient);
        EmissiveScale->A.Connect(0, Intensity);
        EmissiveScale->B.Connect(0, FlameGain);
        UMaterialExpressionMultiply* Emissive =
            NewObject<UMaterialExpressionMultiply>(Material, NAME_None, RF_Transient);
        Emissive->A.Connect(0, VertexColor);
        Emissive->B.Connect(0, EmissiveScale);
        UMaterialExpressionMultiply* Opacity =
            NewObject<UMaterialExpressionMultiply>(Material, NAME_None, RF_Transient);
        Opacity->A.Connect(4, VertexColor);
        Opacity->B.Connect(0, Intensity);
        Material->GetEditorOnlyData()->EmissiveColor.Connect(0, Emissive);
        Material->GetEditorOnlyData()->Opacity.Connect(0, Opacity);
        Material->GetExpressionCollection().AddExpression(VertexColor);
        Material->GetExpressionCollection().AddExpression(Intensity);
        Material->GetExpressionCollection().AddExpression(FlameGain);
        Material->GetExpressionCollection().AddExpression(EmissiveScale);
        Material->GetExpressionCollection().AddExpression(Emissive);
        Material->GetExpressionCollection().AddExpression(Opacity);
        Material->StateId = FGuid(0x44657443, 0x61707475, 0x72654658, 0x76330001);
        Material->ForceRecompileForRendering();
        if (GShaderCompilingManager) GShaderCompilingManager->FinishAllCompilation();
        Cached.Reset(Material);
        return Material;
#else
        UE_LOG(LogTemp, Fatal, TEXT("fast_shader_v3 requires an editor build to compile its deterministic capture material."));
        return nullptr;
#endif
    }

    void AddTriangle(TArray<int32>& Indices, int32 A, int32 B, int32 C)
    {
        Indices.Append({ A, B, C });
    }

    void AddFlame(TArray<FVector>& V, TArray<int32>& I, TArray<FLinearColor>& C,
        TArray<FVector2D>& UV, float Height, float Radius, float Variation)
    {
        constexpr int32 Sides = 12;
        const int32 Base = V.Num();
        const float TipLean = (Variation - 0.5f) * Radius * 0.35f;
        for (int32 N = 0; N < Sides; ++N)
        {
            const float A = 2.0f * PI * static_cast<float>(N) / Sides;
            const float X = FMath::Cos(A), Y = FMath::Sin(A);
            V.Add(FVector(X * Radius, Y * Radius, 0));
            C.Add(FLinearColor(1.0f, 0.035f, 0.002f, 1.0f));
            UV.Add(FVector2D(static_cast<float>(N) / Sides, 0));
            V.Add(FVector(X * Radius * 0.63f, Y * Radius * 0.63f, Height * 0.55f));
            C.Add(FLinearColor(1.0f, 0.22f, 0.006f, 1.0f));
            UV.Add(FVector2D(static_cast<float>(N) / Sides, 0.55f));
        }
        const int32 Tip = V.Num();
        V.Add(FVector(TipLean, 0, Height));
        C.Add(FLinearColor(1.0f, 0.78f, 0.08f, 1.0f));
        UV.Add(FVector2D(0.5f, 1.0f));
        for (int32 N = 0; N < Sides; ++N)
        {
            const int32 Next = (N + 1) % Sides;
            AddTriangle(I, Base + N * 2, Base + Next * 2, Base + N * 2 + 1);
            AddTriangle(I, Base + N * 2 + 1, Base + Next * 2, Base + Next * 2 + 1);
            AddTriangle(I, Base + N * 2 + 1, Base + Next * 2 + 1, Tip);
        }
    }

    void AddSmokePuff(TArray<FVector>& V, TArray<int32>& I, TArray<FLinearColor>& C,
        TArray<FVector2D>& UV, const FVector& Center, const FVector& Radii,
        float Opacity, int32 Sides, int32 Rings)
    {
        const int32 Base = V.Num();
        for (int32 Ring = 0; Ring <= Rings; ++Ring)
        {
            const float T = static_cast<float>(Ring) / Rings;
            const float Z = (T * 2.0f - 1.0f) * Radii.Z;
            const float RingScale = FMath::Sqrt(FMath::Max(0.0f, 1.0f - FMath::Square(T * 2.0f - 1.0f)));
            const float Feather = FMath::Clamp((1.0f - FMath::Square(T * 2.0f - 1.0f)) * 0.88f, 0.0f, 1.0f);
            for (int32 Side = 0; Side <= Sides; ++Side)
            {
                const float A = 2.0f * PI * static_cast<float>(Side) / Sides;
                const float Noise = 0.91f + 0.09f * UnitFromHash(static_cast<uint32>(Side + Ring * 41));
                V.Add(Center + FVector(FMath::Cos(A) * Radii.X * RingScale * Noise,
                    FMath::Sin(A) * Radii.Y * RingScale * Noise, Z));
                const float Density = Opacity * Feather;
                C.Add(FLinearColor(0.095f, 0.105f, 0.115f, Density));
                UV.Add(FVector2D(static_cast<float>(Side) / Sides, T));
            }
        }
        const int32 Stride = Sides + 1;
        for (int32 Ring = 0; Ring < Rings; ++Ring)
            for (int32 Side = 0; Side < Sides; ++Side)
            {
                const int32 A = Base + Ring * Stride + Side;
                const int32 B = A + Stride;
                AddTriangle(I, A, A + 1, B);
                AddTriangle(I, A + 1, B + 1, B);
            }
    }

    void FillNormals(const TArray<FVector>& Vertices, TArray<FVector>& Normals)
    {
        Normals.Init(FVector::UpVector, Vertices.Num());
    }
}

UDeterministicCaptureEffectsComponent::UDeterministicCaptureEffectsComponent()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void UDeterministicCaptureEffectsComponent::Initialize(const FVector& FlameWorldLocation,
    const FVector& SmokeWorldLocation,
    int32 InCellId, int32 InSeed, float InFuel, int32 InIgnitionTick, int32 CurrentTick,
    const FVector2D& WindDirection, float WindSpeedUUPerSecond,
    float FlameScale, float SmokeScale, float Intensity, int32 SmokeLayerCount,
    bool bEnableFlame, bool bEnableSmoke)
{
    CellId = InCellId;
    Seed = InSeed;
    Fuel = FMath::Max(0.0f, InFuel);
    IgnitionTick = InIgnitionTick;
    ConfiguredSmokeLayerCount = FMath::Clamp(SmokeLayerCount, 1, 4);
    FlameLocation = FlameWorldLocation;
    SmokeLocation = SmokeWorldLocation;
    EnsureGeometry();
    ApplyTick(FlameWorldLocation, SmokeWorldLocation, CurrentTick, WindDirection, WindSpeedUUPerSecond, FlameScale, SmokeScale,
        Intensity, SmokeLayerCount, bEnableFlame, bEnableSmoke);
}

void UDeterministicCaptureEffectsComponent::EnsureGeometry()
{
    if (bGeometryCreated || !GetOwner()) return;
    bGeometryCreated = true;

    auto MakeMesh = [this](const TCHAR* Name) -> UProceduralMeshComponent*
    {
        const FName UniqueName = MakeUniqueObjectName(this, UProceduralMeshComponent::StaticClass(), FName(Name));
        UProceduralMeshComponent* Mesh = NewObject<UProceduralMeshComponent>(this, UniqueName);
        if (!Mesh) return nullptr;
        Mesh->SetupAttachment(GetOwner()->GetRootComponent());
        Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        Mesh->SetCanEverAffectNavigation(false);
        Mesh->bUseAsyncCooking = false;
        Mesh->RegisterComponent();
        Mesh->SetVisibleInSceneCaptureOnly(bCaptureOnly);
        Mesh->SetCastShadow(false);
        Mesh->bVisibleInReflectionCaptures = false;
        Mesh->bVisibleInRayTracing = false;
        return Mesh;
    };

    FlameMesh = MakeMesh(TEXT("DeterministicCaptureFlame"));
    SmokeMesh = MakeMesh(TEXT("DeterministicCaptureSmoke"));

    constexpr float Variation = 0.5f;
    if (FlameMesh)
    {
        TArray<FVector> V, N; TArray<int32> I; TArray<FLinearColor> C; TArray<FVector2D> UV;
        AddFlame(V, I, C, UV, 650.0f, 320.0f, Variation);
        FillNormals(V, N);
        FlameMesh->CreateMeshSection_LinearColor(0, V, I, N, UV, C, TArray<FProcMeshTangent>(), false);
    }
    if (SmokeMesh)
    {
        TArray<FVector> V, N; TArray<int32> I; TArray<FLinearColor> C; TArray<FVector2D> UV;
        const int32 Layers = ConfiguredSmokeLayerCount;
        for (int32 Layer = 0; Layer < Layers; ++Layer)
        {
            const float HeightAlpha = Layers > 1 ? static_cast<float>(Layer) / (Layers - 1) : 0.0f;
            const float PuffSize = 230.0f + HeightAlpha * 180.0f;
            const FVector Center((Layer % 2 == 0 ? -1.0f : 1.0f) * (70.0f + HeightAlpha * 60.0f),
                (Layer % 2 == 0 ? 1.0f : -1.0f) * 35.0f,
                250.0f + static_cast<float>(Layer) * (Layers > 1 ? 400.0f / (Layers - 1) : 0.0f));
            AddSmokePuff(V, I, C, UV, Center,
                FVector(PuffSize, PuffSize * 0.86f, PuffSize * 0.88f),
                0.48f - HeightAlpha * 0.18f, 12, 6);
        }
        FillNormals(V, N);
        SmokeMesh->CreateMeshSection_LinearColor(0, V, I, N, UV, C, TArray<FProcMeshTangent>(), false);
    }

    UMaterialInterface* BaseMaterial = FlameMaterial ? FlameMaterial.Get() : SmokeMaterial.Get();
    if (!BaseMaterial)
    {
        GeneratedMaterial = CreateCaptureMaterial();
        BaseMaterial = GeneratedMaterial;
    }
    if (BaseMaterial)
    {
        FlameMID = UMaterialInstanceDynamic::Create(FlameMaterial ? FlameMaterial.Get() : BaseMaterial, this);
        SmokeMID = UMaterialInstanceDynamic::Create(SmokeMaterial ? SmokeMaterial.Get() : BaseMaterial, this);
        if (FlameMesh) FlameMesh->SetMaterial(0, FlameMID);
        if (SmokeMesh) SmokeMesh->SetMaterial(0, SmokeMID);
    }
}

void UDeterministicCaptureEffectsComponent::UpdateAtTick(const FVector& FlameWorldLocation,
    const FVector& SmokeWorldLocation, int32 CurrentTick,
    const FVector2D& WindDirection, float WindSpeedUUPerSecond,
    float FlameScale, float SmokeScale, float Intensity, int32 SmokeLayerCount,
    bool bEnableFlame, bool bEnableSmoke)
{
    ApplyTick(FlameWorldLocation, SmokeWorldLocation, CurrentTick, WindDirection, WindSpeedUUPerSecond, FlameScale,
        SmokeScale, Intensity, SmokeLayerCount, bEnableFlame, bEnableSmoke);
}

void UDeterministicCaptureEffectsComponent::ApplyTick(const FVector& FlameWorldLocation,
    const FVector& SmokeWorldLocation, int32 CurrentTick,
    const FVector2D& WindDirection, float WindSpeedUUPerSecond,
    float FlameScale, float SmokeScale, float Intensity, int32 SmokeLayerCount, bool bEnableFlame, bool bEnableSmoke)
{
    const int32 AgeTicks = FMath::Max(0, CurrentTick - IgnitionTick);
    const uint32 Key = static_cast<uint32>(Seed) ^ static_cast<uint32>(CellId) * 0x9E3779B9u;
    const float Phase = static_cast<float>((AgeTicks + static_cast<int32>(MixBits(Key) % 97u)) % 97) / 97.0f;
    const float Flicker = 0.88f + 0.12f * FMath::Sin(2.0f * PI * Phase);
    const FVector2D Wind = WindDirection.GetSafeNormal() * FMath::Max(0.0f, WindSpeedUUPerSecond);
    const float SmokeAgeSeconds = FMath::Min(static_cast<float>(AgeTicks) * 0.1f, 30.0f);

    if (FlameMesh)
    {
        FlameLocation = FlameWorldLocation;
        FlameMesh->SetWorldLocation(FlameLocation);
        const float FuelScale = FMath::Clamp(0.55f + Fuel * 0.45f, 0.35f, 1.5f);
        FlameMesh->SetWorldScale3D(FVector(FMath::Max(0.0f, FlameScale) * Flicker * FuelScale));
        FlameMesh->SetVisibility(bEnableFlame && Intensity > 0.0f, true);
    }
    if (SmokeMesh)
    {
        const float DriftTime = SmokeAgeSeconds;
        const FVector Drift(Wind.X * DriftTime, Wind.Y * DriftTime,
            FMath::Min(1600.0f, DriftTime * 45.0f));
        SmokeLocation = SmokeWorldLocation;
        SmokeMesh->SetWorldLocation(SmokeLocation + Drift);
        SmokeMesh->SetWorldScale3D(FVector(FMath::Max(0.0f, SmokeScale)));
        SmokeMesh->SetVisibility(bEnableSmoke && Intensity > 0.0f, true);
    }
    if (FlameMID)
    {
        FlameMID->SetScalarParameterValue(TEXT("CaptureIntensity"), FMath::Max(0.0f, Intensity));
    }
    if (SmokeMID)
    {
        SmokeMID->SetScalarParameterValue(TEXT("CaptureIntensity"), FMath::Max(0.0f, Intensity));
    }
}

void UDeterministicCaptureEffectsComponent::SetVisibleInSceneCaptureOnly(bool bVisibleOnlyInCapture)
{
    bCaptureOnly = bVisibleOnlyInCapture;
    if (FlameMesh) FlameMesh->SetVisibleInSceneCaptureOnly(bVisibleOnlyInCapture);
    if (SmokeMesh) SmokeMesh->SetVisibleInSceneCaptureOnly(bVisibleOnlyInCapture);
}

void UDeterministicCaptureEffectsComponent::GetEffectPrimitives(TArray<UPrimitiveComponent*>& OutPrimitives) const
{
    if (FlameMesh) OutPrimitives.Add(FlameMesh);
    if (SmokeMesh) OutPrimitives.Add(SmokeMesh);
}

void UDeterministicCaptureEffectsComponent::OnComponentDestroyed(bool bDestroyingHierarchy)
{
    if (FlameMesh) FlameMesh->DestroyComponent();
    if (SmokeMesh) SmokeMesh->DestroyComponent();
    Super::OnComponentDestroyed(bDestroyingHierarchy);
}

void UDeterministicCaptureEffectsComponent::Deactivate()
{
    if (FlameMesh) FlameMesh->SetVisibility(false, true);
    if (SmokeMesh) SmokeMesh->SetVisibility(false, true);
}
