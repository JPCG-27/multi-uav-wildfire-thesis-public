#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "../World/Grid.h"
#include "../Drone/DroneTypes.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(
    FPersonObservationFreshnessTest,
    "Thesis.Vision.PersonObservationFreshness",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FPersonObservationFreshnessTest::RunTest(const FString& Parameters)
{
    Grid Belief(2, 2);
    TestEqual(TEXT("No initial person evidence"), Belief.getLastPersonObservedTick(1, 0), -1);
    FObservation Observation;
    Observation.CellIndex = 1;
    Observation.Tick = 7;
    Observation.PersonLogOddsUpdate = 2.5f;
    Belief.RecordPersonObservation(Observation);
    TestEqual(TEXT("Capture tick, not later ingestion tick"), Belief.getLastPersonObservedTick(1, 0), 7);

    Observation.Tick = 50;
    Observation.bHasPersonEvidence = false;
    Observation.PersonLogOddsUpdate = 0.0f;
    Belief.setLastObservedTick(1, 0, 50);
    Belief.RecordPersonObservation(Observation);
    TestEqual(TEXT("Fire sensing retains legacy clock"), Belief.getLastObservedTick(1, 0), 50);
    TestEqual(TEXT("Fire sensing cannot refresh person tracking"), Belief.getLastPersonObservedTick(1, 0), 7);

    Observation.Tick = 40;
    Observation.bHasPersonEvidence = true;
    Observation.PersonLogOddsUpdate = -0.2f;
    Belief.RecordPersonObservation(Observation);
    TestEqual(TEXT("Actual negative evidence refreshes person clock"), Belief.getLastPersonObservedTick(1, 0), 40);
    Observation.Tick = 30;
    Belief.RecordPersonObservation(Observation);
    TestEqual(TEXT("Older evidence cannot rewind freshness"), Belief.getLastPersonObservedTick(1, 0), 40);

    Grid NewOwner(2, 2);
    NewOwner.setLastPersonObservedTick(1, 0, Belief.getLastPersonObservedTick(1, 0));
    TestEqual(TEXT("Ownership copy retains modality clock"), NewOwner.getLastPersonObservedTick(1, 0), 40);
    Belief.Initialize();
    TestEqual(TEXT("Reset clears person clock"), Belief.getLastPersonObservedTick(1, 0), -1);
    TestEqual(TEXT("Reset clears legacy clock"), Belief.getLastObservedTick(1, 0), -1);
    return true;
}

#endif
