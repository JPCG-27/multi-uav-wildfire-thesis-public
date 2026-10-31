#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "RHI.h"
#include "RHICommandList.h"
#include "RenderingThread.h"

#include "../Drone/DroneTypes.h"
#include "../Gpu/GpuBeliefManager.h"
#include "../World/PriorityModel.h"

namespace
{
class FIsolatedGpuBeliefWorld
{
public:
    bool Create()
    {
        if (!GEngine)
        {
            return false;
        }

        const UWorld::InitializationValues InitValues = UWorld::InitializationValues()
            .AllowAudioPlayback(false)
            .CreatePhysicsScene(false)
            .RequiresHitProxies(false)
            .ShouldSimulatePhysics(false)
            .EnableTraceCollision(false)
            .CreateNavigation(false)
            .CreateAISystem(false);
        World = UWorld::CreateWorld(EWorldType::Game, false,
                                    FName(TEXT("GpuPersonFreshnessAutomation")),
                                    nullptr, false, ERHIFeatureLevel::Num, &InitValues);
        if (!World)
        {
            return false;
        }

        World->AddToRoot();
        Context = &GEngine->CreateNewWorldContext(EWorldType::Game);
        Context->SetCurrentWorld(World);
        // CreateWorld consumes InitValues while initializing the persistent
        // level. Calling InitializeNewWorld again duplicates WorldSettings.
        return true;
    }

    ~FIsolatedGpuBeliefWorld()
    {
        if (!World)
        {
            return;
        }

        World->DestroyWorld(false);
        if (GEngine && Context)
        {
            GEngine->DestroyWorldContext(World);
        }
        World->RemoveFromRoot();
    }

    UWorld* Get() const { return World; }

private:
    UWorld* World = nullptr;
    FWorldContext* Context = nullptr;
};

bool ReadPriorityAfterObservations(UGpuBeliefManager& BeliefManager,
                                   const TArray<FObservation>& PersonObservations,
                                   const TArray<FObservation>& FireOnlyObservations,
                                   int32 CurrentTick,
                                   bool bSeparatePersonFreshness,
                                   float& OutPriority,
                                   FString& OutFailure)
{
    // These must remain separate GPU ingests.  The second pass is the exact
    // fire-only update that must not overwrite the clock written by the first.
    BeliefManager.DispatchObservationIngest(PersonObservations);
    BeliefManager.DispatchObservationIngest(FireOnlyObservations);
    BeliefManager.DispatchSpread();
    BeliefManager.DispatchPriority(CurrentTick, bSeparatePersonFreshness);
    BeliefManager.RequestPriorityReadback(CurrentTick);

    // A render-command fence only proves submission to the RHI thread.  The
    // test is the explicit GPU consumer, so wait for completion here rather
    // than polling or depending on wall-clock timing.
    ENQUEUE_RENDER_COMMAND(ThesisGpuFreshnessTestWaitForGpu)(
        [](FRHICommandListImmediate& RHICmdList)
        {
            RHICmdList.SubmitAndBlockUntilGPUIdle();
        });
    FlushRenderingCommands();

    TArray<float> Readback;
    int32 SourceTick = INDEX_NONE;
    if (!BeliefManager.TryGetPriorityReadback(Readback, SourceTick))
    {
        OutFailure = TEXT("priority readback was not ready after the explicit GPU idle wait");
        return false;
    }
    if (SourceTick != CurrentTick)
    {
        OutFailure = FString::Printf(TEXT("priority readback source tick %d, expected %d"), SourceTick, CurrentTick);
        return false;
    }
    if (Readback.Num() != 1)
    {
        OutFailure = FString::Printf(TEXT("priority readback had %d cells, expected one"), Readback.Num());
        return false;
    }

    OutPriority = Readback[0];
    return true;
}

float ExpectedSeparatePersonPriority(float PersonLogOdds, int32 PersonStaleness)
{
    const float PersonEvidence = ThesisPriority::PositiveEvidence(PersonLogOdds);
    const float PersonUrgency = 1.0f - FMath::Exp(
        -static_cast<float>(PersonStaleness) / ThesisPriority::MonitoringUrgencyTau);
    return ThesisPriority::WeightPerson * PersonEvidence * ThesisPriority::ProbabilityScale * PersonUrgency;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FGpuPersonObservationFreshnessTest,
    "Thesis.Vision.GpuPersonObservationFreshness",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FGpuPersonObservationFreshnessTest::RunTest(const FString& Parameters)
{
    if (GUsingNullRHI)
    {
        AddWarning(TEXT("Skipping GPU freshness readback test because NullRHI cannot execute the shader passes."));
        return true;
    }

    FIsolatedGpuBeliefWorld Fixture;
    if (!TestTrue(TEXT("Creates an isolated world"), Fixture.Create()))
    {
        return false;
    }

    UGpuBeliefManager* BeliefManager = Fixture.Get()->GetSubsystem<UGpuBeliefManager>();
    if (!TestNotNull(TEXT("Creates the isolated GPU belief subsystem"), BeliefManager))
    {
        return false;
    }

    constexpr int32 PersonObservationTick = 10;
    constexpr int32 FireOnlyObservationTick = 20;
    constexpr float PositivePersonLogOdds = 2.5f;
    constexpr float AnalyticTolerance = 1.0e-4f;

    FObservation PositivePerson;
    PositivePerson.CellIndex = 0;
    PositivePerson.Tick = PersonObservationTick;
    PositivePerson.PersonLogOddsUpdate = PositivePersonLogOdds;
    PositivePerson.bHasPersonEvidence = true;

    FObservation FireOnly;
    FireOnly.CellIndex = 0;
    FireOnly.Tick = FireOnlyObservationTick;
    FireOnly.FireLogOddsUpdate = 0.0f;
    FireOnly.PersonLogOddsUpdate = 0.0f;
    FireOnly.bHasPersonEvidence = false;

    TArray<FObservation> PersonObservations;
    PersonObservations.Add(PositivePerson);
    TArray<FObservation> FireOnlyObservations;
    FireOnlyObservations.Add(FireOnly);

    // Separate freshness: the later fire-only observation resets generic
    // staleness but must retain tick 10 for person monitoring urgency.
    BeliefManager->InitializeTextures(1, 1);
    BeliefManager->ResetTextures();
    float SeparatePriority = 0.0f;
    FString ReadbackFailure;
    if (!TestTrue(TEXT("Reads separate-freshness GPU priority"),
                  ReadPriorityAfterObservations(*BeliefManager, PersonObservations, FireOnlyObservations,
                                                FireOnlyObservationTick, true, SeparatePriority, ReadbackFailure)))
    {
        AddError(ReadbackFailure);
        return false;
    }

    const float ExpectedSeparate = ExpectedSeparatePersonPriority(
        PositivePersonLogOdds, FireOnlyObservationTick - PersonObservationTick);
    TestTrue(*FString::Printf(
                 TEXT("Separate freshness preserves person urgency after fire-only sensing (actual %.8f, expected %.8f)"),
                 SeparatePriority, ExpectedSeparate),
             FMath::IsNearlyEqual(SeparatePriority, ExpectedSeparate, AnalyticTolerance));

    // Legacy / ideal-sensor behavior deliberately uses generic staleness.  The
    // same fire-only observation therefore leaves no urgency at this tick.
    BeliefManager->ResetTextures();
    float LegacyPriority = 0.0f;
    ReadbackFailure.Reset();
    if (!TestTrue(TEXT("Reads legacy GPU priority"),
                  ReadPriorityAfterObservations(*BeliefManager, PersonObservations, FireOnlyObservations,
                                                FireOnlyObservationTick, false, LegacyPriority, ReadbackFailure)))
    {
        AddError(ReadbackFailure);
        return false;
    }
    TestTrue(*FString::Printf(TEXT("Legacy mode retains generic-staleness priority behavior (actual %.8f)"), LegacyPriority),
             FMath::IsNearlyZero(LegacyPriority, AnalyticTolerance));

    // Reset must discard person belief: a fire-only update after a new scenario
    // cannot yield person priority from a pooled texture.
    BeliefManager->ResetTextures();
    TArray<FObservation> NoPersonObservations;
    float FireOnlyPriorityAfterReset = 0.0f;
    ReadbackFailure.Reset();
    if (!TestTrue(TEXT("Reads fire-only reset GPU priority"),
                  ReadPriorityAfterObservations(*BeliefManager, NoPersonObservations, FireOnlyObservations,
                                                FireOnlyObservationTick, true, FireOnlyPriorityAfterReset, ReadbackFailure)))
    {
        AddError(ReadbackFailure);
        return false;
    }
    TestTrue(*FString::Printf(
                 TEXT("Reset clears GPU person belief before fire-only sensing (actual %.8f)"),
                 FireOnlyPriorityAfterReset),
             FMath::IsNearlyZero(FireOnlyPriorityAfterReset, AnalyticTolerance));

    // Reset must also discard the modality clock.  The fresh tick deliberately
    // predates the old one: a retained tick 10 would win shader max() over 3.
    BeliefManager->ResetTextures();
    FObservation PersonAfterReset = PositivePerson;
    PersonAfterReset.Tick = 3;
    TArray<FObservation> PersonObservationsAfterReset;
    PersonObservationsAfterReset.Add(PersonAfterReset);
    float PriorityAfterReset = 0.0f;
    ReadbackFailure.Reset();
    if (!TestTrue(TEXT("Reads reset GPU priority"),
                  ReadPriorityAfterObservations(*BeliefManager, PersonObservationsAfterReset, FireOnlyObservations,
                                                FireOnlyObservationTick, true, PriorityAfterReset, ReadbackFailure)))
    {
        AddError(ReadbackFailure);
        return false;
    }
    const float ExpectedAfterReset = ExpectedSeparatePersonPriority(
        PositivePersonLogOdds, FireOnlyObservationTick - PersonAfterReset.Tick);
    TestTrue(*FString::Printf(
                 TEXT("Reset clears GPU person belief and accepts the new modality clock (actual %.8f, expected %.8f)"),
                 PriorityAfterReset, ExpectedAfterReset),
             FMath::IsNearlyEqual(PriorityAfterReset, ExpectedAfterReset, AnalyticTolerance));

    return true;
}

#endif
