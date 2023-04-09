#pragma once
#include <iostream>
#include <fstream>
#include <bitset>
#include <concepts>
#include <optional>
#include "OEMBDSEPHEMRIS.h"
#include "OEMBESTPOS.h"
#include "OEMGPSEPHEM.h"
#include "OEMRANGE.h"
#include "Data.h"

template<typename Stream>
	requires std::derived_from<Stream, std::istream>
class OEMDecoder
{
	using uc = unsigned char;
	using us = unsigned short;
	using ui = unsigned int;
	using ul = unsigned long;
	using rg = OEMRANGE;
	using eph = Ephemeris;
	using gpseph = OEMGPSEPHEM;
	using bdseph = OEMBDSEPHEMRIS;
	using bt = OEMBESTPOS;
	using vec = std::vector<uc>;
private:
	inline static constexpr auto _polyCrc32 { 0xEDB88320u };
	OEMMessageHeader _header;
	std::vector<unsigned char> _message;
	/// <summary>
	/// 返回从message指定位置转换类型得到的字段
	/// </summary>
	/// <typeparam name="FieldType">约束为浮点或整型</typeparam>
	/// <param name="offset"></param>
	/// <returns></returns>
	template<typename FieldType>
		requires std::is_floating_point_v<FieldType> || std::is_integral_v<FieldType>
	inline FieldType ToDec(int offset)
	{
		return *reinterpret_cast<const FieldType*>(&_message[offset]);
	}
	/// <summary>
	/// 对流位置偏移
	/// </summary>
	/// <param name="stream"></param>
	/// <param name="offset"></param>
	inline void StreamOffset(Stream& stream, int offset)
	{
		stream.seekg((int)stream.tellg() + offset);
	}
	/// <summary>
	/// 从流中直接析出字段
	/// </summary>
	/// <typeparam name="FieldType">约束为浮点或整型</typeparam>
	/// <param name="stream"></param>
	/// <returns></returns>
	template<typename FieldType>
		requires std::is_floating_point_v<FieldType> || std::is_integral_v<FieldType>
	inline FieldType StreamRead(Stream & stream)
	{
		FieldType res {};
		stream.read((char*)&res, sizeof(FieldType));
		return res;
	}
	/// <summary>
	/// 返回理论32位CRC校验码
	/// </summary>
	/// <returns></returns>
	inline unsigned int Crc32Verification()
	{
		unsigned int crc { 0 };
		auto len { static_cast<int>(_message.size()) };
		for (int i = 0; i < len; ++i)
		{
			crc ^= _message[i];
			for (int j = 0; j < 8; ++j)
			{
				crc & 1 ? crc = (crc >> 1) ^ _polyCrc32 : crc >>= 1;
			}
		}
		return crc;
	}
public:
	OEMDecoder()
	{}
	/// <summary>
	/// 读取流。若message为RANGE，则返回true。若流读取完毕，则返回false
	/// </summary>
	/// <param name="stream"></param>
	/// <returns></returns>
	std::optional<OEMRANGE> Read(Stream& stream)
	{
		while (!stream.eof())
		{
			auto p { StreamRead<uc>(stream) };
			if (p == 0XAA)
			{
				p = StreamRead<uc>(stream);
				if (p == 0X44)
				{
					p = StreamRead<uc>(stream);
					if (p == 0X12)
					{
						auto hdLen { StreamRead<uc>(stream) };
						StreamOffset(stream, 4);
						auto msgLen { StreamRead<us>(stream) };
						StreamOffset(stream, -10);
						auto size { hdLen + msgLen };
						_message = vec(size);
						stream.read((char*)&_message[0], size);
						const auto originalCRC { StreamRead<ui>(stream) };
						if (originalCRC == Crc32Verification())
							[[likely]]
						{
							auto msgid { ToDec<us>(4) };
							_header = OEMMessageHeader { ToDec<us>(14),ToDec<int>(16),hdLen,msgLen,msgid };
							_message.erase(_message.begin(), _message.begin() + _header.HeaderLength);
							switch (msgid)
							{
								case RANGE:
									return ReadOEMRange();
								case GPSEPHEM:
									ReadOEMGPSEPHEM();
									break;
								case BDSEPHEMERIS:
									ReadOEMBDSEPHEMRIS();
									break;
								case BESTPOS:
									ReadOEMBESTPOS();
									break;
								default:
									break;
							}
							}
						}
					}
				}
		}
		return std::nullopt;
			};
private:
	void ReadOEMBDSEPHEMRIS()
	{
		auto ephemeris { OEMBDSEPHEMRIS{_header} };
		ephemeris.SatelliteOfEphemeris.System = BDS;
		ephemeris.SatelliteOfEphemeris.Id = static_cast<us>(ToDec<unsigned long>(0));
		auto week = ToDec<unsigned long>(4) + 1356;
		ephemeris.HealthStatus = ToDec<unsigned long>(16);
		ephemeris.TimeOfGroupDelay = ToDec<double>(20);
		ephemeris.ReferenceTimeForSatelliteClockCorrction = GpsTime { week, (double)ToDec<unsigned long>(40) + 14 };
		ephemeris.ClockJitter = ToDec<double>(44);
		ephemeris.ClockWander = ToDec<double>(52);
		ephemeris.ClockWanderSpeed = ToDec<double>(60);
		ephemeris.ReferenceTimeForEphemeris = GpsTime { week, (double)ToDec<unsigned long>(72) + 14 };
		ephemeris.Semi_majorAxis = pow(ToDec<double>(76), 2);
		ephemeris.Eccentricity = ToDec<double>(84);
		ephemeris.ArgumentOfPerigee = ToDec<double>(92);
		ephemeris.MeanAngularVelocityDifference = ToDec<double>(100);
		ephemeris.MeanAnomalyOfReferenceTime = ToDec<double>(108);
		ephemeris.RightAscension = ToDec<double>(116);
		ephemeris.RateOfRightAscension = ToDec<double>(124);
		ephemeris.InclinationAngleAtReferenceTime = ToDec<double>(132);
		ephemeris.RateOfInclinationAngle = ToDec<double>(140);
		ephemeris.Cos_CorrectionArgumentOfLatitude = ToDec<double>(148);
		ephemeris.Sin_CorrectionArgumentOfLatitude = ToDec<double>(156);
		ephemeris.Cos_CorrectionOrbitRadius = ToDec<double>(164);
		ephemeris.Sin_CorrectionOrbitRadius = ToDec<double>(172);
		ephemeris.Cos_CorrectionAngleOfInclination = ToDec<double>(180);
		ephemeris.Sin_CorrectionAngleOfInclination = ToDec<double>(188);
		if (EphemerisDataOf.contains(ephemeris.SatelliteOfEphemeris))
		{
			*EphemerisDataOf[ephemeris.SatelliteOfEphemeris] = ephemeris;
		}
		else
		{
			EphemerisDataOf[ephemeris.SatelliteOfEphemeris] = new OEMBDSEPHEMRIS { ephemeris };
		}
	}
	void ReadOEMBESTPOS()
	{
		auto bestPos { OEMBESTPOS{_header,
			Angle::ToRad(ToDec<double>(8)),
			Angle::ToRad(ToDec<double>(16)),
			ToDec<double>(24)} };
		bestPos.LatitudeStandardDeviation = ToDec<float>(40);
		bestPos.LongitudeStandardDeviation = ToDec<float>(44);
		bestPos.HeightStandardDeviation = ToDec<float>(48);
		auto p { bestPos.BaseStationId };
		for (int i = 0; i < 4; ++i)
		{
			*(p++) = ToDec<char>(52 + i);
		}
		bestPos.NumbersOfSatelliteTracked = ToDec<unsigned char>(64);
		bestPos.NumberOfSatelliteUsedInSolution = ToDec<unsigned char>(65);
		//BestPosDatas.push_back(bestPos);
	}
	void ReadOEMGPSEPHEM()
	{
		OEMGPSEPHEM ephemeris { _header };
		ephemeris.SatelliteOfEphemeris.System = GPS;
		ephemeris.SatelliteOfEphemeris.Id = static_cast<us>(ToDec<unsigned long>(0));
		ephemeris.HealthStatus = ToDec<unsigned long>(12);
		auto week { ToDec<unsigned long>(24) };
		ephemeris.ReferenceTimeForEphemeris = GpsTime { week,ToDec<double>(32) };
		ephemeris.Semi_majorAxis = ToDec<double>(40);
		ephemeris.MeanAngularVelocityDifference = ToDec<double>(48);
		ephemeris.MeanAnomalyOfReferenceTime = ToDec<double>(56);
		ephemeris.Eccentricity = ToDec<double>(64);
		ephemeris.ArgumentOfPerigee = ToDec<double>(72);
		ephemeris.Cos_CorrectionArgumentOfLatitude = ToDec<double>(80);
		ephemeris.Sin_CorrectionArgumentOfLatitude = ToDec<double>(88);
		ephemeris.Cos_CorrectionOrbitRadius = ToDec<double>(96);
		ephemeris.Sin_CorrectionOrbitRadius = ToDec<double>(104);
		ephemeris.Cos_CorrectionAngleOfInclination = ToDec<double>(112);
		ephemeris.Sin_CorrectionAngleOfInclination = ToDec<double>(120);
		ephemeris.InclinationAngleAtReferenceTime = ToDec<double>(128);
		ephemeris.RateOfInclinationAngle = ToDec<double>(136);
		ephemeris.RightAscension = ToDec<double>(144);
		ephemeris.RateOfRightAscension = ToDec<double>(152);
		ephemeris.ReferenceTimeForSatelliteClockCorrction = GpsTime { week,ToDec<double>(164) };
		ephemeris.ClockJitter = ToDec<double>(180);
		ephemeris.ClockWander = ToDec<double>(188);
		ephemeris.ClockWanderSpeed = ToDec<double>(196);
		if (EphemerisDataOf.contains(ephemeris.SatelliteOfEphemeris))
		{
			*EphemerisDataOf[ephemeris.SatelliteOfEphemeris] = ephemeris;
		}
		else
		{
			EphemerisDataOf[ephemeris.SatelliteOfEphemeris] = new OEMGPSEPHEM { ephemeris };
		}
	}
	OEMRANGE ReadOEMRange()
	{
		using namespace std;
		const auto n { ToDec<unsigned long>(0) };
		OEMRANGE range { _header,n };
		auto offset { 0 };
		for (ul i = 0; i < n; ++i)
		{
			bitset<32> status(ToDec<unsigned long>(44 + offset));
			bitset<3> bsys;
			bitset<5> btype;
			for (size_t j = 16; j < 19; ++j)
			{
				bsys[j - 16] = status[j];
			}
			for (size_t j = 21; j < 26; ++j)
			{
				btype[j - 21] = status[j];
			}
			auto sys { SatelliteSystem(bsys.to_ulong()) };
			auto type { SatelliteObservation::GetSignalType(sys,btype.to_ulong()) };
			auto parityFlag = (bool)status[11];
			if (type != SignalType::OTHERS)
			{
				Satellite sat { ToDec<us>(4 + offset),sys };
				range.ObservationOf[sat].Data[type] = ObservationData
				{
					ToDec<double>(8 + offset),
					ToDec<float>(16 + offset),
					SatelliteObservation::ToPhaseMeter(-ToDec<double>(20 + offset),FrequencyOfSignalType.at(type)),
					static_cast<float>(SatelliteObservation::ToPhaseMeter(ToDec<float>(28 + offset),FrequencyOfSignalType.at(type))),
					-static_cast<float>(SatelliteObservation::ToPhaseMeter(ToDec<float>(32 + offset),FrequencyOfSignalType.at(type))),
					ToDec<float>(36 + offset),
					parityFlag
				};
			}
			offset += 44;
		}
		return range;
	}
		};