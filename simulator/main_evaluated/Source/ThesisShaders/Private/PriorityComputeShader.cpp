#include "PriorityComputeShader.h"

IMPLEMENT_GLOBAL_SHADER(
    FPriorityComputeCS,
    "/ThesisShaders/PriorityCompute.usf",
    "MainCS",
    SF_Compute);
