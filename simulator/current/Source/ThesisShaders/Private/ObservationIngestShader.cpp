#include "ObservationIngestShader.h"

IMPLEMENT_GLOBAL_SHADER(
    FObservationIngestCS,
    "/ThesisShaders/ObservationIngest.usf",
    "MainCS",
    SF_Compute);