#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/ProceduralTerrain.h"
#include "ProceduralMeshComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Misc/CommandLine.h"

namespace
{
    struct FTerrainMeshSnapshot
    {
        FTransform ActorTransform = FTransform::Identity;
        FString HeightSignature;
        TArray<FProcMeshVertex> Vertices;
        TArray<uint32> Triangles;
        FBox LocalBounds = FBox(ForceInit);
    };

    bool CaptureMesh(AProceduralTerrain* Terrain, FTerrainMeshSnapshot& Out)
    {
        if (!Terrain || !Terrain->Mesh) return false;
        const FProcMeshSection* Section = Terrain->Mesh->GetProcMeshSection(0);
        if (!Section) return false;
        Out.ActorTransform = Terrain->GetActorTransform();
        Out.HeightSignature = Terrain->GetHeightFieldSignature();
        Out.Vertices = Section->ProcVertexBuffer;
        Out.Triangles = Section->ProcIndexBuffer;
        Out.LocalBounds = Section->SectionLocalBox;
        return true;
    }

    bool MeshesEqual(const FTerrainMeshSnapshot& A, const FTerrainMeshSnapshot& B)
    {
        if (!A.ActorTransform.Equals(B.ActorTransform, 0.0f) ||
            A.HeightSignature != B.HeightSignature || A.Vertices.Num() != B.Vertices.Num() ||
            A.Triangles != B.Triangles || !A.LocalBounds.Min.Equals(B.LocalBounds.Min, 0.0f) ||
            !A.LocalBounds.Max.Equals(B.LocalBounds.Max, 0.0f))
        {
            return false;
        }
        for (int32 I = 0; I < A.Vertices.Num(); ++I)
        {
            const FProcMeshVertex& X = A.Vertices[I];
            const FProcMeshVertex& Y = B.Vertices[I];
            if (!X.Position.Equals(Y.Position, 0.0f) || !X.Normal.Equals(Y.Normal, 0.0f) ||
                !X.Tangent.TangentX.Equals(Y.Tangent.TangentX, 0.0f) ||
                !X.UV0.Equals(Y.UV0, 0.0f) || X.Color != Y.Color)
            {
                return false;
            }
        }
        return true;
    }

    struct FScopedDisableSceneReuse
    {
        FString Original = FCommandLine::Get();
        explicit FScopedDisableSceneReuse(bool bDisable)
        {
            FString Line = Original;
            Line.ReplaceInline(TEXT("-ThesisDisableSceneReuse"), TEXT(""), ESearchCase::IgnoreCase);
            if (bDisable) Line += TEXT(" -ThesisDisableSceneReuse");
            FCommandLine::Set(*Line);
        }
        ~FScopedDisableSceneReuse() { FCommandLine::Set(*Original); }
    };

    struct FTerrainCase
    {
        int32 Width = 9;
        int32 Height = 8;
        FVector Origin = FVector(120.0, -240.0, 310.0);
        float CellSize = 1000.0f;
        int32 Seed = 78123;
        ETerrainProfile Profile = ETerrainProfile::Foothills;
        float Detail = 1.0f;
    };

    void Build(AProceduralTerrain* Terrain, const FTerrainCase& C)
    {
        Terrain->Profile = C.Profile;
        Terrain->DetailDensity = C.Detail;
        Terrain->SeedOverride = 0;
        Terrain->VerticesPerCell = 1;
        Terrain->PresentationMarginCells = 1;
        Terrain->PresentationMarginMeters = 0.0f;
        Terrain->RenderTileQuads = 0;
        Terrain->BuildTerrain(C.Width, C.Height, C.Origin, C.CellSize, C.Seed);
    }

}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTerrainSceneCacheTest, "Thesis.Terrain.SceneGeometryCache",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FTerrainSceneCacheTest::RunTest(const FString& Parameters)
{
    const auto Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("TerrainSceneCacheFixture"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    AProceduralTerrain* Terrain = World->SpawnActor<AProceduralTerrain>();
    Terrain->bRenderVisuals = true;
    Terrain->bVisibleOnlyToDroneCaptures = false;

    FTerrainCase Base;
    FTerrainMeshSnapshot Cold, Warm;
    {
        FScopedDisableSceneReuse DisableReuse(true);
        Build(Terrain, Base);
        TestTrue(TEXT("Cold uncached mesh captured"), CaptureMesh(Terrain, Cold));
    }
    Build(Terrain, Base);
    Build(Terrain, Base); // First enabled build fills the cache; this one is a warm hit.
    TestTrue(TEXT("Warm cached mesh captured"), CaptureMesh(Terrain, Warm));
    TestTrue(TEXT("Same config cold/warm mesh is bit-exact"), MeshesEqual(Cold, Warm));

    auto CompareCachedWithUncached = [&](const TCHAR* Label, const FTerrainCase& Changed, bool bExpectDifferentGeometry = true)
    {
        Build(Terrain, Base); // Put the old geometry back in the one-entry cache.
        FTerrainMeshSnapshot OldKey;
        CaptureMesh(Terrain, OldKey);
        Build(Terrain, Changed);
        FTerrainMeshSnapshot Cached;
        TestTrue(FString::Printf(TEXT("%s cached build captured"), Label), CaptureMesh(Terrain, Cached));
        if (bExpectDifferentGeometry)
        {
            TestFalse(FString::Printf(TEXT("%s changes the result"), Label), MeshesEqual(OldKey, Cached));
        }
        else
        {
            TestTrue(FString::Printf(TEXT("%s changes the world-space signature"), Label),
                OldKey.HeightSignature != Cached.HeightSignature &&
                !OldKey.ActorTransform.Equals(Cached.ActorTransform, 0.0f));
        }
        {
            FScopedDisableSceneReuse DisableReuse(true);
            Build(Terrain, Changed);
        }
        FTerrainMeshSnapshot Uncached;
        TestTrue(FString::Printf(TEXT("%s uncached reference captured"), Label), CaptureMesh(Terrain, Uncached));
        TestTrue(FString::Printf(TEXT("%s cached result matches uncached reference"), Label),
            MeshesEqual(Cached, Uncached));
    };

    FTerrainCase Changed = Base;
    Changed.Origin.Z += 725.0;
    CompareCachedWithUncached(TEXT("Origin Z"), Changed, false);
    Changed = Base;
    Changed.Profile = ETerrainProfile::Mountains;
    CompareCachedWithUncached(TEXT("Profile"), Changed);
    Changed = Base;
    Changed.Seed += 1;
    CompareCachedWithUncached(TEXT("Seed"), Changed);
    Changed = Base;
    Changed.Detail = 1.65f;
    CompareCachedWithUncached(TEXT("Detail density"), Changed);
    Changed = Base;
    Changed.Width += 2;
    CompareCachedWithUncached(TEXT("Grid size"), Changed);

    Build(Terrain, Base);
    Terrain->ScorchCell(3, 4);
    Terrain->FlushScorch();
    const int32 BurnMaskWidth = Base.Width + 2; // The fixture uses a one-cell apron.
    const int32 ScorchedIndex = (4 + 1) * BurnMaskWidth + (3 + 1);
    TestTrue(TEXT("Scorch updates the CPU burn-mask mirror"), Terrain->BurnMaskCPU.IsValidIndex(ScorchedIndex) &&
        Terrain->BurnMaskCPU[ScorchedIndex] == 255 && !Terrain->bBurnDirty);
    Build(Terrain, Base); // Geometry may be reused; the tick-varying mask must be fresh.
    TestTrue(TEXT("Warm geometry reuse clears tick-varying burn-mask state"),
        Terrain->BurnMaskCPU.IsValidIndex(ScorchedIndex) && Terrain->BurnMaskCPU[ScorchedIndex] == 0 &&
        !Terrain->bBurnDirty);
    Terrain->ScorchCell(3, 4);
    Terrain->FlushScorch();
    Terrain->ResetScorch();

    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    return true;
}
#endif
