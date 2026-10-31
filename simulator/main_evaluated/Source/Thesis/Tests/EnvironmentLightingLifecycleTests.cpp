#if WITH_DEV_AUTOMATION_TESTS
#include "Misc/AutomationTest.h"
#include "../World/SimulationManager.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/SkyLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FEnvironmentLightingLifecycleTest, "Thesis.Environment.LightingLifecycle",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FEnvironmentLightingLifecycleTest::RunTest(const FString& Parameters)
{
    const auto Initialization = UWorld::InitializationValues().AllowAudioPlayback(false)
        .CreatePhysicsScene(false).CreateNavigation(false).CreateAISystem(false).ShouldSimulatePhysics(false);
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false, TEXT("LightingFixture"),
        nullptr, true, ERHIFeatureLevel::Num, &Initialization);
    GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
    ASimulationManager* Manager = World->SpawnActor<ASimulationManager>();
    AActor* Rig = World->SpawnActor<AActor>();
    auto AddComponent = [](AActor* Owner, UActorComponent* Component)
    {
        Owner->AddInstanceComponent(Component);
        Component->RegisterComponent();
    };
    auto* MainSun = NewObject<UDirectionalLightComponent>(Rig);
    MainSun->SetIntensity(75000.0f);
    MainSun->SetAtmosphereSunLight(true);
    AddComponent(Rig, MainSun);
    AddComponent(Rig, NewObject<USkyLightComponent>(Rig));
    AddComponent(Rig, NewObject<USkyAtmosphereComponent>(Rig));
    AActor* Extra = World->SpawnActor<AActor>();
    auto* ExtraSun = NewObject<UDirectionalLightComponent>(Extra);
    ExtraSun->SetIntensity(4.2f);
    ExtraSun->SetAtmosphereSunLight(true);
    AddComponent(Extra, ExtraSun);
    auto* ExtraSky = NewObject<USkyLightComponent>(Extra);
    AddComponent(Extra, ExtraSky);

    Manager->NormalizeEnvironmentLighting();
    TestFalse(TEXT("Duplicate sun disabled without sensor initialization"), ExtraSun->IsVisible());
    TestFalse(TEXT("Duplicate skylight disabled"), ExtraSky->IsVisible());
    TestTrue(TEXT("Authored rig retained"), MainSun->IsVisible());
    Manager->NormalizeEnvironmentLighting();
    Manager->RestoreSensorResources();
    TestTrue(TEXT("Repeated cleanup preserves original sun visibility"), ExtraSun->IsVisible());
    TestTrue(TEXT("Repeated cleanup preserves original sky visibility"), ExtraSky->IsVisible());
    TestTrue(TEXT("Atmosphere flag restored"), ExtraSun->bAtmosphereSunLight);

    GEngine->DestroyWorldContext(World);
    World->DestroyWorld(false);
    return true;
}
#endif
