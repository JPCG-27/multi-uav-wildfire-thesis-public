#include "PixelBleedSpreadShader.h"

IMPLEMENT_GLOBAL_SHADER(
    FPixelBleedSpreadCS,
    "/ThesisShaders/PixelBleedSpread.usf",
    "MainCS",
    SF_Compute);
