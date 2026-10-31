#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/LiveRenderPolicy.h"
#include "../World/SimulationManager.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "HAL/IConsoleManager.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLiveRenderPolicyTest, "Thesis.Environment.LiveRenderPolicy",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FLiveRenderPolicyTest::RunTest(const FString& Parameters)
{
    TestTrue(TEXT("Presentation-only view can use Nanite"), LiveRenderPolicy::CanUseNanite(true, false, false));
    TestFalse(TEXT("Explicitly disabled live Nanite stays off"), LiveRenderPolicy::CanUseNanite(false, false, false));
    TestFalse(TEXT("Capture profile stays deterministic"), LiveRenderPolicy::CanUseNanite(true, true, false));
    TestFalse(TEXT("Suppressed viewport does not enable live graphics"), LiveRenderPolicy::CanUseNanite(true, false, true));

    IConsoleVariable* Nanite = IConsoleManager::Get().FindConsoleVariable(TEXT("r.Nanite"));
    IConsoleVariable* ProxyDelay = IConsoleManager::Get().FindConsoleVariable(
        TEXT("r.PSOPrecache.ProxyCreationWhenPSOReady"));
    if (!Nanite || !ProxyDelay)
    {
        AddError(TEXT("Render policy console variables are unavailable."));
        return false;
    }
    const int32 Original = Nanite->GetInt();
    const int32 OriginalProxyDelay = ProxyDelay->GetInt();
    const auto Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("RenderPolicyFixture"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    auto* Live = World->SpawnActor<ASimulationManager>();
    UWorld* SensorWorld = UWorld::CreateWorld(EWorldType::Game, false, TEXT("RenderPolicySensorWorld"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(SensorWorld);
    auto* Sensor = SensorWorld->SpawnActor<ASimulationManager>();
    Live->bEnableVisionBridge = true; // Matches the saved map: bridge checked, captures disabled.
    Live->bEnableYoloDatasetExport = true;
    Live->CameraRenderProfile = TEXT("legacy");
    LiveRenderPolicy::Register(Live);
    TestEqual(TEXT("Vision and dataset flags without drone capture enable Nanite"), Nanite->GetInt(), 1);
    Sensor->bEnableDroneCaptures = true;
    Sensor->CameraRenderProfile = TEXT("fast_shader_v3");
    LiveRenderPolicy::Register(Sensor);
    TestEqual(TEXT("Concurrent sensor manager takes precedence"), Nanite->GetInt(), 0);

    TestEqual(TEXT("Fast shader v3 enables immediate proxy creation"), ProxyDelay->GetInt(), 0);
    // Register/update a legacy capture manager after v3. It must not overwrite
    // the global PSO policy while a v3 capture manager is still registered.
    Live->bEnableDroneCaptures = true;
    LiveRenderPolicy::Update();
    TestEqual(TEXT("Legacy capture does not undo v3 immediate proxy creation"), ProxyDelay->GetInt(), 0);
    TestEqual(TEXT("Legacy capture still keeps deterministic Nanite policy"), Nanite->GetInt(), 0);

    LiveRenderPolicy::Unregister(Sensor);
    TestEqual(TEXT("Removing v3 restores the original proxy creation policy"), ProxyDelay->GetInt(), OriginalProxyDelay);
    TestEqual(TEXT("Remaining legacy capture keeps Nanite disabled"), Nanite->GetInt(), 0);

    Live->bEnableDroneCaptures = false;
    LiveRenderPolicy::Update();
    TestEqual(TEXT("Disabling captures returns to Nanite presentation"), Nanite->GetInt(), 1);
    LiveRenderPolicy::Unregister(Live);
    TestEqual(TEXT("Last manager restores previous editor setting"), Nanite->GetInt(), Original);
    TestEqual(TEXT("Last manager leaves proxy policy at its original value"), ProxyDelay->GetInt(), OriginalProxyDelay);
    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    GEngine->DestroyWorldContext(SensorWorld);
    SensorWorld->DestroyWorld(false);
    return true;
}
#endif
