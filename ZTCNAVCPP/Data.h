#pragma once
#include "Ephemeris.h"
#include "OEMBESTPOS.h"
#include "OEMRange.h"

///// <summary>
///// 读取的当前历元的观测值数据
///// </summary>
//extern OEMRANGE RangeData;
/// <summary>
/// 卫星星历
/// </summary>
extern std::map<Satellite, Ephemeris*> EphemerisDataOf;
/// <summary>
/// 读取的当前历元的定位数据
/// </summary>
extern std::vector<OEMBESTPOS> BestPosDatas;
extern int IterationMaxNum;
extern double IterationThreshold;
extern double RangeInterval;