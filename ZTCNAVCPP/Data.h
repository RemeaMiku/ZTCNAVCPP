#pragma once
#include "Ephemeris.h"
#include "OEMBESTPOS.h"
#include "OEMRange.h"

///// <summary>
///// ��ȡ�ĵ�ǰ��Ԫ�Ĺ۲�ֵ����
///// </summary>
//extern OEMRANGE RangeData;
/// <summary>
/// ��������
/// </summary>
extern std::map<Satellite, Ephemeris*> EphemerisDataOf;
/// <summary>
/// ��ȡ�ĵ�ǰ��Ԫ�Ķ�λ����
/// </summary>
extern std::vector<OEMBESTPOS> BestPosDatas;
extern int IterationMaxNum;
extern double IterationThreshold;
extern double RangeInterval;