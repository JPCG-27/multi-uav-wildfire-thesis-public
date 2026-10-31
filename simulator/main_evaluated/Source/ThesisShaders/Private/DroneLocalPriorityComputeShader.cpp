#include "DroneLocalPriorityComputeShader.h"

IMPLEMENT_GLOBAL_SHADER(
    FDroneLocalPriorityComputeCS,
    "/ThesisShaders/DroneLocalPriorityCompute.usf",
    "MainCS",
    SF_Compute);
