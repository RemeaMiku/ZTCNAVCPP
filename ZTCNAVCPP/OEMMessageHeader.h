#pragma once
#pragma warning(disable:26495)
#include "GpsTime.h"

enum OEMMessageType
{
	RANGE = 43,
	GPSEPHEM = 7,
	BDSEPHEMERIS = 1696,
	BESTPOS = 42
};
class OEMMessageHeader
{
public:
	GpsTime Time;
	unsigned char HeaderLength;
	unsigned short MessageLength;
	OEMMessageType MessageType;
	OEMMessageHeader() = default;
	OEMMessageHeader(unsigned short week, int ms, unsigned char hdLen, unsigned short msgLen, unsigned short msgid);
};
