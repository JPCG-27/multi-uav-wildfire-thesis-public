#include "StalenessUpdateShader.h"

IMPLEMENT_GLOBAL_SHADER(
    FStalenessUpdateCS,
    "/ThesisShaders/StalenessUpdate.usf",
    "MainCS",
    SF_Compute);