#include "Data.h"

//OEMRANGE RangeData{};
std::map<Satellite, Ephemeris*> EphemerisDataOf {};
std::vector<OEMBESTPOS> BestPosDatas {};
int IterationMaxNum = 10;
double IterationThreshold = 1E-6;
double RangeInterval = 1;
