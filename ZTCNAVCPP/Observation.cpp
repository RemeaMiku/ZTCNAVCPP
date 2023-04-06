#include "Observation.h"

const std::map<SatelliteSystem, std::map<int, SignalType>> SignalTypesOfOEM
{
	{GPS,{{0,SignalType::L1CA},{9,SignalType::L2PY}}},
	{BDS,{{0,SignalType::B1I},{4,SignalType::B1I},{2,SignalType::B3I},{6,SignalType::B3I}}}
};

const std::map<SatelliteSystem, std::vector<SignalType>> SignalTypesOfSatelliteSystems
{
	{GPS,{SignalType::L1CA,SignalType::L2PY}},
	{BDS,{SignalType::B1I,SignalType::B3I}}
};

const std::map<SignalType, double> FrequencyOfSignalType
{
	{SignalType::L1CA,1575.42E6},
	{SignalType::L2PY,1227.60E6},
	{SignalType::B1I,1561.098E6},
	{SignalType::B3I,1268.52E6}
};

double SatelliteObservation::IFOfPseudorange(const SignalType& t1, const SignalType& t2) const
{
	auto& f_1 { FrequencyOfSignalType.at(t1) }, & f_2 { FrequencyOfSignalType.at(t2) };
	auto f_1_2 { pow(f_1, 2) }, f_2_2 { pow(f_2,2) };
	auto& P_1 { Data.at(t1).Pseudorange }, & P_2 { Data.at(t2).Pseudorange };
	auto pif { 1 / (f_1_2 - f_2_2) * (f_1_2 * P_1 - f_2_2 * P_2) };
	return pif;
}

double SatelliteObservation::Melbourne_W¨¹bbena(const SignalType& t1, const SignalType& t2) const
{
	auto& f_1 { FrequencyOfSignalType.at(t1) }, & f_2 { FrequencyOfSignalType.at(t2) };
	auto& obs1 { Data.at(t1) }, & obs2 { Data.at(t2) };
	auto& L_1 { obs1.CarrierPhase }, & L_2 { obs2.CarrierPhase }, & P_1 { obs1.Pseudorange }, & P_2 { obs2.Pseudorange };
	return 1 / (f_1 - f_2) * (f_1 * L_1 - f_2 * L_2) - 1 / (f_1 + f_2) * (f_1 * P_1 + f_2 * P_2);
}

double SatelliteObservation::GFOfCarrierPhase(const SignalType& t1, const SignalType& t2) const
{
	auto& L_1 { Data.at(t1).CarrierPhase }, & L_2 { Data.at(t2).CarrierPhase };
	return L_1 - L_2;
}

SignalType SatelliteObservation::GetSignalType(const SatelliteSystem& sys, int type)
{
	if (SignalTypesOfOEM.contains(sys))
	{
		auto& map { SignalTypesOfOEM.at(sys) };
		if (map.contains(type))
		{
			return map.at(type);
		}
		else
		{
			return SignalType::OTHERS;
		}
	}
	else
	{
		return SignalType::OTHERS;
	}
}