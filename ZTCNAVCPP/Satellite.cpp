#include "Satellite.h"
Satellite::Satellite(unsigned short prn, SatelliteSystem sys) :Id(prn), System(sys)
{}

Satellite::Satellite(const std::string& prn)
{
	auto pair { SystemNameOfCode.find(prn[0]) };
	if (pair != SystemNameOfCode.end())
	{
		System = pair->second;
	}
	else
	{
		System = Other;
	}
	Id = atoi(prn.substr(1, 2).c_str());
}

std::string Satellite::ToString() const
{
	return std::format("{}{:00}", SystemCodeOfName.at(System), Id);
}

const std::map<char, SatelliteSystem> Satellite::SystemNameOfCode
{
	{'G',GPS},
	{'C',BDS},
	{'R',GLONASS},
	{'E',Galileo},
	{'J',QZSS},
};

const std::map<SatelliteSystem, char> Satellite::SystemCodeOfName
{
	{GPS,'G'},
	{BDS,'C'},
	{GLONASS,'R'},
	{Galileo,'E'},
	{QZSS,'J'},
	{Other,'-'},
};