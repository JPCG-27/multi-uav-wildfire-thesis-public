#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "SimMetricsRecorder.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

namespace
{
    static FString MakeMetricsWriterTestSubdir(const TCHAR* CaseName)
    {
        return FPaths::Combine(
            TEXT("MetricsWriterTests"),
            FString::Printf(TEXT("%s_%s"), CaseName, *FGuid::NewGuid().ToString(EGuidFormats::Digits)));
    }
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimMetricsRecorderCheckedAppendTest,
    "Thesis.Benchmark.MetricsRecorder.CheckedAppend",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FSimMetricsRecorderCheckedAppendTest::RunTest(const FString& Parameters)
{
    const FString Subdir = MakeMetricsWriterTestSubdir(TEXT("success"));
    const FString RunDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"), Subdir);
    FSimMetricsRecorder Recorder;
    Recorder.FlushEveryNTicks = 1;
    TestTrue(TEXT("BeginRun succeeds"), Recorder.BeginRun(TEXT("checked_append"), TEXT("{}"), Subdir));

    TArray<FDroneTickSample> Samples;
    FDroneTickSample Sample;
    Sample.DroneIndex = 0;
    Sample.PosCellX = 2.0f;
    Samples.Add(Sample);
    Recorder.RecordTick(17, Samples, 3, 1, 0, 0.0f);

    FString Ticks;
    FString Drones;
    TestTrue(TEXT("ticks row was appended"), FFileHelper::LoadFileToString(Ticks, *FPaths::Combine(RunDir, TEXT("ticks.csv"))));
    TestTrue(TEXT("drone row was appended"), FFileHelper::LoadFileToString(Drones, *FPaths::Combine(RunDir, TEXT("drones.csv"))));
    TestTrue(TEXT("ticks include exact sample"), Ticks.Contains(TEXT("17,3,1,0\n")));
    TestTrue(TEXT("drones include exact sample"), Drones.Contains(TEXT("17,0,2.0000,0.0000")));
    TestFalse(TEXT("successful writes leave no failure state"), Recorder.HasWriteFailure());
    TestFalse(TEXT("successful writes release their buffers"), Recorder.HasPendingWrites());
    Recorder.EndRun(17);
    FSimMetricsRecorder ReusedRecorder;
    AddExpectedError(TEXT("refusing to reuse non-empty run directory"),
        EAutomationExpectedErrorFlags::Contains, 1, false);
    TestFalse(TEXT("existing run directory cannot be reused"),
        ReusedRecorder.BeginRun(TEXT("checked_append_reuse"), TEXT("{}"), Subdir));
    FString PreservedTicks;
    TestTrue(TEXT("existing metrics remain readable after reuse is rejected"),
        FFileHelper::LoadFileToString(PreservedTicks, *FPaths::Combine(RunDir, TEXT("ticks.csv"))));
    TestTrue(TEXT("reuse rejection preserves the completed tick row"), PreservedTicks.Contains(TEXT("17,3,1,0\n")));
    IFileManager::Get().DeleteDirectory(*RunDir, false, true);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimMetricsRecorderAppendFailureTest,
    "Thesis.Benchmark.MetricsRecorder.AppendFailureIsRetainedAndReported",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FSimMetricsRecorderAppendFailureTest::RunTest(const FString& Parameters)
{
    const FString Subdir = MakeMetricsWriterTestSubdir(TEXT("failure"));
    const FString RunDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"), Subdir);
    FSimMetricsRecorder Recorder;
    Recorder.FlushEveryNTicks = 1;
    TestTrue(TEXT("BeginRun succeeds"), Recorder.BeginRun(TEXT("append_failure"), TEXT("{}"), Subdir));

    const FString TicksPath = FPaths::Combine(RunDir, TEXT("ticks.csv"));
    IFileManager::Get().Delete(*TicksPath, false, true, true);
    TestTrue(TEXT("replace output file with a directory to force an append failure"),
        IFileManager::Get().MakeDirectory(*TicksPath, true));
    AddExpectedError(TEXT("Thesis metrics: append failed"),
        EAutomationExpectedErrorFlags::Contains, 1, false);
    Recorder.RecordTick(7, TArray<FDroneTickSample>(), 0, 0, 0, 0.0f);

    TestTrue(TEXT("append failure is sticky"), Recorder.HasWriteFailure());
    TestTrue(TEXT("unwritten row remains buffered"), Recorder.HasPendingWrites());
    TestFalse(TEXT("failed recorder stops accepting metrics"), Recorder.IsActive());
    FString Marker;
    const FString MarkerPath = FPaths::Combine(RunDir, TEXT("metrics_write_failure.txt"));
    TestTrue(TEXT("failure marker exists"), FFileHelper::LoadFileToString(Marker, *MarkerPath));
    TestTrue(TEXT("marker identifies failed file"), Marker.Contains(TEXT("file=")) && Marker.Contains(TEXT("ticks.csv")));
    TestTrue(TEXT("marker identifies pending content and reason"),
        Marker.Contains(TEXT("pending_characters=8")) && Marker.Contains(TEXT("reason=append_failed")));

    IFileManager::Get().DeleteDirectory(*RunDir, false, true);
    return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
