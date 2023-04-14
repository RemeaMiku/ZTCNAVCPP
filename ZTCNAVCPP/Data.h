#pragma once
#include "Ephemeris.h"
#include "OEMBESTPOS.h"
#include "OEMRange.h"

/// <summary>
/// ÎÀÐÇÐÇÀú
/// </summary>
inline std::map<Satellite, Ephemeris*> EphemerisDataOf {};
inline std::vector<OEMBESTPOS> BestPosDatas {};
inline int IterationMaxNum = 10;
inline double IterationThreshold = 1E-6;
inline double RangeInterval = 1;
inline CartCoordinate BasePos;