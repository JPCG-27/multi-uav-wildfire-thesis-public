#include "GridStatsShader.h"

IMPLEMENT_GLOBAL_SHADER(
    FGridStatsColumnReduceCS,
    "/ThesisShaders/GridStatsReduce.usf",
    "ColumnReduceCS",
    SF_Compute);

IMPLEMENT_GLOBAL_SHADER(
    FGridStatsFinalReduceCS,
    "/ThesisShaders/GridStatsReduce.usf",
    "FinalReduceCS",
    SF_Compute);
