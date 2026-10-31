#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

#include "../Drone/Drone.h"
#include "../World/Grid.h"
#include "../World/PriorityModel.h"

namespace
{
class FPersonFreshnessOwnershipWorld
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
                                    FName(TEXT("PersonFreshnessOwnershipAutomation")),
                                    nullptr, false, ERHIFeatureLevel::Num, &InitValues);
        if (!World)
        {
            return false;
        }

        World->AddToRoot();
        Context = &GEngine->CreateNewWorldContext(EWorldType::Game);
        Context->SetCurrentWorld(World);
        // CreateWorld consumes InitValues while initializing its persistent level.
        // Do not call InitializeNewWorld again: that duplicates WorldSettings.
        return true;
    }

    ~FPersonFreshnessOwnershipWorld()
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

void SetPersonBeliefCell(Grid& SharedGrid,
                        float PersonLogOdds,
                        int32 LastObservedTick,
                        int32 LastPersonObservedTick,
                        int32 FirstPersonDetectionTick)
{
    SharedGrid.setPersonBelief(1, 0, PersonLogOdds);
    SharedGrid.setLastObservedTick(1, 0, LastObservedTick);
    SharedGrid.setLastPersonObservedTick(1, 0, LastPersonObservedTick);
    SharedGrid.setFirstPersonDetectionTick(1, 0, FirstPersonDetectionTick);
}

float ExpectedDronePriority(float PersonLogOdds,
                            int32 CurrentTick,
                            int32 LastObservedTick,
                            int32 LastPersonObservedTick,
                            int32 FirstPersonDetectionTick,
                            bool bSeparatePersonFreshness)
{
    const float GeneralStaleness = static_cast<float>(CurrentTick - LastObservedTick);
    const float PersonStaleness = bSeparatePersonFreshness
        ? static_cast<float>(CurrentTick - LastPersonObservedTick)
        : GeneralStaleness;
    const float StalenessTerm = GeneralStaleness / ThesisPriority::StalenessSaturationTicks;
    const float PersonUrgency = ThesisPriority::MonitoringUrgency(PersonStaleness);
    const float PersonGate = ThesisPriority::DiscoveryGate(FirstPersonDetectionTick, CurrentTick);
    const float PersonTerm = ThesisPriority::PositiveEvidence(PersonLogOdds)
        * ThesisPriority::ProbabilityScale
        * PersonUrgency
        * PersonGate;

    return ThesisPriority::WeightStaleness * StalenessTerm
        + ThesisPriority::WeightPerson * PersonTerm;
}

bool TransferCellTo(ADrone& Drone, int32 CellIndex)
{
    TArray<int32> NewOwnedCells;
    NewOwnedCells.Add(CellIndex);
    TArray<int32> NewlyOwnedCells;
    NewlyOwnedCells.Add(CellIndex);
    Drone.SetOwnedCells(MoveTemp(NewOwnedCells), NewlyOwnedCells);
    return Drone.GetOwnedCellsForPlanning().Contains(CellIndex);
}

void ReleaseCellFrom(ADrone& Drone)
{
    TArray<int32> NoOwnedCells;
    const TArray<int32> NoNewCells;
    Drone.SetOwnedCells(MoveTemp(NoOwnedCells), NoNewCells);
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPersonFreshnessOwnershipTransferTest,
    "Thesis.Vision.PersonFreshnessOwnershipTransfer",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPersonFreshnessOwnershipTransferTest::RunTest(const FString& Parameters)
{
    // Declare the grid before the fixture so actors are destroyed while their
    // SharedGrid pointer is still valid.
    Grid SharedGrid(2, 1);
    FPersonFreshnessOwnershipWorld Fixture;
    if (!TestTrue(TEXT("Creates an isolated world"), Fixture.Create()))
    {
        return false;
    }

    ADrone* PreviousOwner = Fixture.Get()->SpawnActor<ADrone>();
    ADrone* NewOwner = Fixture.Get()->SpawnActor<ADrone>();
    if (!TestNotNull(TEXT("Spawns the previous owner"), PreviousOwner) ||
        !TestNotNull(TEXT("Spawns the new owner"), NewOwner))
    {
        return false;
    }

    PreviousOwner->InitFromManager(&SharedGrid, 0, FVector::ZeroVector, 1000.0f);
    NewOwner->InitFromManager(&SharedGrid, 1, FVector::ZeroVector, 1000.0f);
    PreviousOwner->SetSeparatePersonFreshness(true);
    NewOwner->SetSeparatePersonFreshness(true);

    constexpr int32 PersonCell = 1;
    constexpr int32 FirstCurrentTick = 40;
    constexpr float InitialPersonLogOdds = 2.5f;
    constexpr int32 InitialLastObservedTick = 35;
    constexpr int32 InitialLastPersonObservedTick = 10;
    constexpr int32 InitialFirstPersonDetectionTick = 0;
    constexpr float AnalyticTolerance = 1.0e-5f;

    SetPersonBeliefCell(SharedGrid, InitialPersonLogOdds,
                       InitialLastObservedTick, InitialLastPersonObservedTick,
                       InitialFirstPersonDetectionTick);
    TestTrue(TEXT("Initial owner acquires the cell"), TransferCellTo(*PreviousOwner, PersonCell));

    const float InitialExpected = ExpectedDronePriority(
        InitialPersonLogOdds, FirstCurrentTick, InitialLastObservedTick,
        InitialLastPersonObservedTick, InitialFirstPersonDetectionTick, true);
    const float InitialActual = PreviousOwner->ComputeCellPriority(PersonCell, FirstCurrentTick);
    TestTrue(*FString::Printf(TEXT("Initial owner uses person freshness (actual %.8f, expected %.8f)"),
                               InitialActual, InitialExpected),
             FMath::IsNearlyEqual(InitialActual, InitialExpected, AnalyticTolerance));

    // Simulate the server's atomic between-tick handover through the production
    // SetOwnedCells API. The new owner must receive the modality-specific tick.
    ReleaseCellFrom(*PreviousOwner);
    TestTrue(TEXT("New owner acquires the transferred cell"), TransferCellTo(*NewOwner, PersonCell));
    const float TransferredActual = NewOwner->ComputeCellPriority(PersonCell, FirstCurrentTick);
    TestTrue(*FString::Printf(TEXT("Handover copies person freshness (actual %.8f, expected %.8f)"),
                               TransferredActual, InitialExpected),
             FMath::IsNearlyEqual(TransferredActual, InitialExpected, AnalyticTolerance));

    NewOwner->SetSeparatePersonFreshness(false);
    const float LegacyExpected = ExpectedDronePriority(
        InitialPersonLogOdds, FirstCurrentTick, InitialLastObservedTick,
        InitialLastPersonObservedTick, InitialFirstPersonDetectionTick, false);
    const float LegacyActual = NewOwner->ComputeCellPriority(PersonCell, FirstCurrentTick);
    TestTrue(*FString::Printf(TEXT("Ideal/legacy mode still uses generic freshness (actual %.8f, expected %.8f)"),
                               LegacyActual, LegacyExpected),
             FMath::IsNearlyEqual(LegacyActual, LegacyExpected, AnalyticTolerance));
    TestTrue(TEXT("Separate YOLO freshness differs from legacy generic freshness"),
             InitialExpected > LegacyExpected);

    // Release and reacquire after the shared aggregate advances. Newly-owned
    // cells must refresh from the current shared belief, not retain stale local
    // values from the previous ownership period.
    ReleaseCellFrom(*NewOwner);
    constexpr int32 ReacquireCurrentTick = 45;
    constexpr float ReacquiredPersonLogOdds = 1.5f;
    constexpr int32 ReacquiredLastObservedTick = 39;
    constexpr int32 ReacquiredLastPersonObservedTick = 36;
    constexpr int32 ReacquiredFirstPersonDetectionTick = 5;
    SetPersonBeliefCell(SharedGrid, ReacquiredPersonLogOdds,
                       ReacquiredLastObservedTick, ReacquiredLastPersonObservedTick,
                       ReacquiredFirstPersonDetectionTick);
    PreviousOwner->SetSeparatePersonFreshness(true);
    TestTrue(TEXT("Previous owner reacquires the cell"), TransferCellTo(*PreviousOwner, PersonCell));

    const float ReacquiredExpected = ExpectedDronePriority(
        ReacquiredPersonLogOdds, ReacquireCurrentTick, ReacquiredLastObservedTick,
        ReacquiredLastPersonObservedTick, ReacquiredFirstPersonDetectionTick, true);
    const float ReacquiredActual = PreviousOwner->ComputeCellPriority(PersonCell, ReacquireCurrentTick);
    TestTrue(*FString::Printf(TEXT("Reacquisition copies current belief and person tick (actual %.8f, expected %.8f)"),
                               ReacquiredActual, ReacquiredExpected),
             FMath::IsNearlyEqual(ReacquiredActual, ReacquiredExpected, AnalyticTolerance));

    return true;
}

#endif
