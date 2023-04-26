#pragma once
#include <cmath>
#include <iostream>
#include <limits>
#include "Angle.hpp"
#include "CartCoordinate.h"
#include "Satellite.h"
#include "Ellipsoid.h"
#include "Vector.h"
#include "Observation.h"
#include "GpsTime.h"
#include "Data.h"
#include "Matrix.h"
#include "lambda.h"
namespace SinglePointPositioning
{
	using namespace Angle;
	inline constexpr double H_0 { 0 };
	inline constexpr double T_0 { 15 + 273.16 };
	inline constexpr double P_0 { 1013.25 };
	inline constexpr double RH_0 { 0.5 };
	inline constexpr double VerticalAngleLimit { ToRad((double)15) };

	/// <summary>
	/// HopeFiled模型对流层改正
	/// </summary>
	/// <param name="H">测站高</param>
	/// <param name="E">卫星高度角</param>
	/// <returns>对流层改正数</returns>
	double TroposphericCorrectionInHopeField(double H, double E)
	{
		if (abs(H) > 1E4)
		{
			return 0;
		}
		auto RH { RH_0 * exp(-0.0006396 * (H - H_0)) };
		auto p { P_0 * pow((1 - 0.0000226 * (H - H_0)),5.225) };
		auto T { T_0 - 0.0065 * (H - H_0) };
		auto e { RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * pow(T,2)) };
		double h_w { 11000 };
		auto h_d { 40136 + 148.72 * (T_0 - 273.16) };
		auto K_w { 155.2E-7 * 4810 / pow(T,2) * e * (h_w - H) };
		auto K_d { 155.2E-7 * p / T * (h_d - H) };
		return K_d / sin(ToRad(sqrt(pow(ToDegree(E), 2) + 6.25))) + K_w / sin(ToRad(sqrt(pow(ToDegree(E), 2) + 2.25)));
	}

	/// <summary>
	/// 计算卫星高度角
	/// </summary>
	/// <param name="obsPos"></param>
	/// <param name="satPositions"></param>
	/// <param name="e"></param>
	/// <returns></returns>
	double VerticalAngleOfSatellite(const CartCoordinate& obsPos, const CartCoordinate& satPos, const Ellipsoid& e)
	{
		//提取测站到卫星的向量
		auto obsToSat { Vector(satPos.ToArray()) - Vector(obsPos.ToArray()) };

		//通过改变大地坐标高程来提取测站当地法向量
		auto obsPos2 { obsPos.ToBLH(e) };
		obsPos2.H()++;
		auto obsPos3 { CartCoordinate::FromBLH(obsPos2,e) };
		auto normal { Vector(obsPos3.ToArray()) - Vector(obsPos.ToArray()) };

		//通过向量内积得到角度
		return PI / 2 - acos(normal * obsToSat / (normal.Norm() * obsToSat.Norm()));
	}

	/// <summary>
	/// 计算信号发射时刻Gps时
	/// </summary>
	/// <param name="t_obs"></param>
	/// <param name="P"></param>
	/// <param name="delta_t"></param>
	/// <returns></returns>
	inline GpsTime TimeOfSignalTransmission(const GpsTime& t_obs, double P, double delta_t)
	{
		return t_obs - P / LightSpeed - delta_t;
	}

	/// <summary>
	/// 对卫星位置进行地球自转改正
	/// </summary>
	/// <param name="omega_e"></param>
	/// <param name="satPositions"></param>
	/// <param name="delta_t"></param>
	/// <returns></returns>
	[[nodiscard]] inline std::vector<double> EarthRotationCorrection(double omega_e, const std::vector<double>& vec, double delta_t)
	{
		auto temp { omega_e * delta_t };
		auto sina { sin(temp) }, cosa { cos(temp) };
		return { cosa * vec[0] + sina * vec[1] ,-sina * vec[0] + cosa * vec[1] ,vec[2] };
	}

	struct SatelliteData
	{
		CartCoordinate Coord;
		Vector Velocity;
		double IFOfPseudorange;
		double Doppler;
		double ClockError;
		double ClockVelocityError;
		double Troposphere;
		double Angle;
		inline std::string ToString() const
		{
			return std::format("{}{} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}°", Coord.ToString(), Velocity.ToString(), IFOfPseudorange, Doppler, ClockError, ClockVelocityError, Troposphere, Angle::ToDegree(Angle));
		}
	};

	/// <summary>
	/// 定位结果结构
	/// 按STATUS Time CartCoord GeoCoord Velocity ClockVelocityError Pdop SPPSIGAMA StdVelocity SatelliteNumbers CLKERR输出
	/// </summary>
	struct SppResult
	{
		//指示解算状态
		bool State;

		//观测时刻GPST
		GpsTime Time;

		//测站ECEF坐标
		CartCoordinate CartCoord;

		//测站WGS-84坐标
		GeoCoordinate GeoCoord;

		//速度向量
		Vector Velocity;
		double Pdop;
		double StdPosition;
		double StdVelocity;

		//各系统卫星数
		std::map<SatelliteSystem, int> SatelliteNumbers;

		//各系统接收机钟差
		std::map<SatelliteSystem, double> ClockError;

		//接收机钟速
		double ClockVelocityError;

		//参与解算卫星数据
		std::map<Satellite, SatelliteData> SatelliteDatas;
		friend inline std::ostream& operator<<(std::ostream& os, const SppResult& obsData)
		{
			if (obsData.State)
			{
				os << 'T';
			}
			else
			{
				os << 'F';
				return os;
			}
			os << std::format(" {} {} {} {} {:.4f} {:.4f} {:.4f} {:.4f}", obsData.Time.ToString(), obsData.CartCoord.ToString(), obsData.GeoCoord.ToString(), obsData.Velocity.ToString(), obsData.ClockVelocityError, obsData.Pdop, obsData.StdPosition, obsData.StdVelocity);
			for (auto& [sys, num] : obsData.SatelliteNumbers)
			{
				os << std::format(" {}:{}", Satellite::SystemCodeOfName.at(sys), num);
			}
			os << ' ';
			for (auto& [sys, err] : obsData.ClockError)
			{
				os << std::format(" {}:{:.4f}", Satellite::SystemCodeOfName.at(sys), err);
			}
			return os;
		}
	};

	std::map<SatelliteSystem, int> satNums;
	std::map<Satellite, CartCoordinate> satPositions;
	std::map<Satellite, SatelliteData> satDatas;
	Matrix X, P, Q, V, B, W;

	inline CartCoordinate ObsPos()
	{
		return { X[0][0],X[1][0],X[2][0] };
	}

	inline int SatNum()
	{
		return (int)satDatas.size();
	}

	inline int ParaNum()
	{
		return 3 + (int)satNums.size();
	}

	void Filter(const GpsTime& time, const std::map<Satellite, SatelliteObservation>& satObs)
	{
		//auto satObs = range.Filter();
		//清空记录
		satNums.clear();
		satPositions.clear();
		satDatas.clear();
		if (satObs.size() <= 3)
		{
			return;
		}

		//遍历卫星观测值
		for (auto& [sat, obs] : satObs)
		{
			if (EphemerisDataOf.contains(sat))
			{
				auto& eph { EphemerisDataOf[sat] };
				auto clockError { 0.0 };
				const auto& t1 { SignalTypesOfSatelliteSystems.at(sat.System)[0] }, & t2 { SignalTypesOfSatelliteSystems.at(sat.System)[1] };

				//计算信号发射时刻t_tr
				auto& P_1 { obs.Data.at(SignalTypesOfSatelliteSystems.at(sat.System)[0]).Pseudorange };
				auto t_tr { TimeOfSignalTransmission(time, P_1, clockError) };
				clockError = eph->ClockError(t_tr);
				t_tr = TimeOfSignalTransmission(time, P_1, clockError);

				//检查星历是否可用
				if (eph->IsAvailable(t_tr))
				{
					//所属系统卫星数+1
					satNums[sat.System]++;
					auto satPos { eph->Position(t_tr) };
					satPositions[sat] = satPos;
					auto satVel { eph->Velocity(t_tr) };
					clockError = eph->ClockError(t_tr);
					auto clockVel { eph->ClockSpeedError(t_tr) };
					auto pif { obs.IFOfPseudorange(t1,t2) };

					//BDS群延迟改正
					if (sat.System == BDS)
					{
						pif += LightSpeed * SatelliteObservation::k13 * ((OEMBDSEPHEMRIS*)eph)->TimeOfGroupDelay / (1 - SatelliteObservation::k13);
					}
					satDatas[sat] = { satPos,satVel,pif,obs.Data.at(t1).InstantaneousCarrierDopplerFrequency,clockError,clockVel,0,0 };
					continue;
				}
			}
		}
	}

	bool LSOfSPP(int satNum, int paraNum)
	{
		//初始化矩阵
		B = { satNum,paraNum };
		P = { satNum,satNum };
		W = { satNum,1 };
		for (int j = 0; j < IterationMaxNum; j++)
		{
			int rowIndex = 0, columnIndex = 3;//列号标记，针对不同系统而言，其钟差对应列号不同
			auto lastSys { satDatas.begin()->first.System };//系统标记，记录上一个卫星的系统。因为map的排序特性得以实现

			//遍历卫星观测值
			for (auto& [sat, data] : satDatas)
			{
				//系统更换
				if (sat.System != lastSys)
				{
					columnIndex++;
				}
				auto& satPos { data.Coord };
				auto rho { CartCoordinate::Distance(ObsPos(),data.Coord) };

				//地球自转改正
				satPos = CartCoordinate(EarthRotationCorrection(EphemerisDataOf[sat]->Omega_e(), satPositions[sat].ToArray(), rho / LightSpeed));
				std::vector<double> row(paraNum, 0);
				for (int i = 0; i < 3; ++i)
				{
					row[i] = (ObsPos()[i] - satPos[i]) / rho;
				}
				B.SetRow(rowIndex, row);
				B[rowIndex][columnIndex] = 1;
				W[rowIndex][0] = data.IFOfPseudorange - rho - X[columnIndex][0] + LightSpeed * data.ClockError - data.Troposphere;
				P[rowIndex][rowIndex] = 1;
				rowIndex++;
				lastSys = sat.System;
			}
			auto Bt { B.Transpose() };
			Q = (Bt * P * B).Inverse();
			auto x { Q * Bt * P * W };
			X += x;
			if (Vector(x.GetColumn(0)).Norm() < IterationThreshold)
			{
				V = B * x - W;
				return true;
			}
		}
		return false;
	}

	void UpdateSatData()
	{
		for (auto it = satDatas.begin(); it != satDatas.end();)
		{
			auto& sat { it->first };
			auto& data { it->second };
			auto& satPos { data.Coord };
			auto rho { CartCoordinate::Distance(ObsPos(),satPos) };

			//EarthRotationCorrection(EphemerisDataOf[sat]->Omega_e(), satPositions, rho / LightSpeed);
			auto& e { Ellipsoid::Ellipsoids.at(sat.System) };
			data.Angle = VerticalAngleOfSatellite(ObsPos(), satPos, e);

			//检查高度角
			if (data.Angle > VerticalAngleLimit)
			{
				data.Troposphere = TroposphericCorrectionInHopeField(ObsPos().ToBLH(e).H(), data.Angle);
				it++;
			}
			else
			{
				//擦除数据
				satNums[sat.System]--;
				if (satNums[sat.System] == 0)
				{
					satNums.erase(sat.System);
				}
				satDatas.erase(it++);
			}
		}
	}

	SppResult Solve(const GpsTime& time, const std::map<Satellite, SatelliteObservation>& satObs)
	{
		Filter(time, satObs);
		if (SatNum() <= ParaNum())
		{
			return { false };
		}
		X = { ParaNum(),1 };
		if (!LSOfSPP(SatNum(), ParaNum()))
		{
			return { false };
		}
		UpdateSatData();
		if (SatNum() < ParaNum())
		{
			return { false };
		}
		if (X.RowsCount() != ParaNum())
		{
			Matrix temp { ParaNum(),1 };
			for (int j = 0; j < ParaNum(); j++)
			{
				temp[j][0] = X[j][0];
			}
			X = temp;
		}
		if (!LSOfSPP(SatNum(), ParaNum()))
		{
			return { false };
		}

		//UpdateSatData();
		std::map<SatelliteSystem, double> clkErrOf;
		int rowIndex { 3 };
		for (auto& [sys, num] : satNums)
		{
			clkErrOf[sys] = X[rowIndex++][0];
		}
		auto pdop { sqrt(Q[0][0] + Q[1][1] + Q[2][2]) };
		auto sppSigma { sqrt((V.Transpose() * P * V / (SatNum() - ParaNum()))[0][0]) };
		rowIndex = 0;
		B = { SatNum(),4 };
		B.SetColumn(3, std::vector<double>(SatNum(), 1));
		for (auto& [sat, data] : satDatas)
		{
			auto& satPos { data.Coord };
			auto obsPos { ObsPos() };
			auto& satVel { data.Velocity };
			auto rho { CartCoordinate::Distance(satPos,obsPos) };
			satPos = CartCoordinate(EarthRotationCorrection(EphemerisDataOf[sat]->Omega_e(), satPos.ToArray(), rho / LightSpeed));
			satVel = Vector(EarthRotationCorrection(EphemerisDataOf[sat]->Omega_e(), satVel.ToArray(), rho / LightSpeed));
			for (auto i = 0; i < 3; i++)
			{
				B[rowIndex][i] = (obsPos[i] - satPos[i]) / rho;
			}
			W[rowIndex++][0] = data.Doppler - ((satPos.X() - obsPos.X()) * satVel[0] + (satPos.Y() - obsPos.Y()) * satVel[1] + (satPos.Z() - obsPos.Z()) * satVel[2]) / rho + LightSpeed * data.ClockVelocityError;
		}
		auto Bt { B.Transpose() };
		auto x { (Bt * P * B).Inverse() * Bt * P * W };
		V = B * x - W;
		auto spvSigma { sqrt((V.Transpose() * P * V / (SatNum() - 4))[0][0]) };
		return { true,time,ObsPos(),ObsPos().ToBLH(WGS_84),Vector{std::vector<double>{x[0][0],x[1][0],x[2][0]}},pdop, sppSigma,spvSigma,satNums,clkErrOf,x[3][0],satDatas };
	}
}

namespace RealTimeKinematic
{
	std::map<Satellite, SatelliteObservation> BuildSingleDifferenceObservations(const std::map<Satellite, SinglePointPositioning::SatelliteData>& rovSatDatas, const std::map<Satellite, SinglePointPositioning::SatelliteData>& baseSatDatas, const std::map<Satellite, SatelliteObservation>& rovObservations, const std::map<Satellite, SatelliteObservation>& baseObservations)
	{
		std::map<Satellite, SatelliteObservation> singleDifferenceObservations;
		for (auto& [satellite, baseObs] : baseObservations)
		{
			if (!baseSatDatas.contains(satellite) || !rovSatDatas.contains(satellite))
				continue;
			if (!rovObservations.contains(satellite))
				continue;
			auto& rovObs = rovObservations.at(satellite);
			auto satObservation = SatelliteObservation {};
			auto& signalTypes = SignalTypesOfSatelliteSystems.at(satellite.System);
			auto isAvailable = true;
			for (auto& signalType : signalTypes)
			{
				auto& rovData = rovObs.Data.at(signalType);
				auto& baseData = baseObs.Data.at(signalType);
				if (rovData.ParityFlag == 0 || baseData.ParityFlag == 0)
				{
					isAvailable = false;
					break;
				}
				satObservation.Data[signalType] = ObservationData
				{
					rovData.Pseudorange - baseData.Pseudorange,
					0,
					rovData.CarrierPhase - baseData.CarrierPhase,
					0,
					0,
					std::min(rovData.C_No,baseData.C_No),
					true
				};
			}
			if (isAvailable)
				singleDifferenceObservations[satellite] = satObservation;
		}
		return singleDifferenceObservations;
	}

	std::map<SatelliteSystem, Satellite> FindReferenceSatellite(const std::map<Satellite, SatelliteObservation>& singleDifferenceObservations)
	{
		std::map<SatelliteSystem, Satellite> referenceSatellites;
		std::map<SatelliteSystem, std::vector<Satellite>> candidateSatellites;
		for (auto& [satellite, observation] : singleDifferenceObservations)
		{
			auto& signalTypes = SignalTypesOfSatelliteSystems.at(satellite.System);
			auto c_no1 = observation.Data.at(signalTypes[0]).C_No;
			auto c_no2 = observation.Data.at(signalTypes[1]).C_No;
			if (c_no1 <= 40 || c_no2 <= 28)
				continue;
			candidateSatellites[satellite.System].push_back(satellite);
		}
		for (auto& [system, satellites] : candidateSatellites)
		{
			float maxc_no = 0;
			std::optional<Satellite> maxSat { std::nullopt };
			for (auto& satellite : satellites)
			{
				auto& signalTypes = SignalTypesOfSatelliteSystems.at(satellite.System);
				auto c_no1 = singleDifferenceObservations.at(satellite).Data.at(signalTypes[0]).C_No;
				if (c_no1 > maxc_no)
				{
					maxc_no = c_no1;
					maxSat = satellite;
				}
			}
			if (maxSat.has_value())
				referenceSatellites[system] = maxSat.value();
		}
		return referenceSatellites;
	}

	Matrix B, W, P, a, Q;

	struct RtkResult
	{
		bool IsFixed;
		GpsTime Time;
		CartCoordinate RefPos;
		CartCoordinate RovPos;
		double Pdop;
		double Sigma;
		double Ratio;
		inline Vector BaseLine() const
		{
			Vector baseline { 3 };
			for (auto i = 0; i < 3; i++)
			{
				baseline(i) = RovPos(i) - RefPos(i);
			}
			return baseline;
		}
		friend inline std::ostream& operator<<(std::ostream& os, const RtkResult& res)
		{
			os << std::format("{} {} {} {} {}{:.4f} {:.4f} {:.4f}", res.IsFixed ? "FIXED" : "FLOAT", res.Time.ToString(), res.RovPos.ToString(), res.RovPos.ToBLH(WGS_84).ToString(), res.BaseLine().ToString(), res.Ratio, res.Pdop, res.Sigma);
			return os;
		}
	};

	std::optional<RtkResult> RtkFloat(std::map<Satellite, SatelliteObservation>& singleDifferenceObs, const SinglePointPositioning::SppResult& baseSppResult, const SinglePointPositioning::SppResult& rovSppResult, const std::map<SatelliteSystem, Satellite>& refSats)
	{
		int satNum = singleDifferenceObs.size() - refSats.size();
		if (satNum < 2)
			return std::nullopt;
		std::map<SatelliteSystem, int> ddObsNums;
		for (auto& [sat, sdObs] : singleDifferenceObs)
		{
			auto& refSat = refSats.at(sat.System);
			if (refSat == sat)
				continue;
			ddObsNums[sat.System] ++;
		}
		auto rovPos { rovSppResult.CartCoord };
		auto isFirst { true };
		auto& basePos = BasePos;
		B = { 4 * satNum,3 + 2 * satNum };
		W = { 4 * satNum,1 };
		P = { 4 * satNum,4 * satNum };
		a = { 2 * satNum,1 };
		Matrix X;
		for (size_t times = 0; times < 10; times++)
		{
			auto satIndex = 0, columnStartIndex = 0;
			auto lastSys = singleDifferenceObs.begin()->first.System;
			for (auto& [sat, sdObs] : singleDifferenceObs)
			{
				auto& refSat = refSats.at(sat.System);
				if (refSat == sat)
					continue;
				auto& rovRefSatPos = rovSppResult.SatelliteDatas.at(refSat).Coord;
				auto& rovSatPos = rovSppResult.SatelliteDatas.at(sat).Coord;
				auto rovSatDis = CartCoordinate::Distance(rovPos, rovSatPos);
				auto rovRefSatDis = CartCoordinate::Distance(rovPos, rovRefSatPos);
				auto l = (rovPos.X() - rovSatPos.X()) / rovSatDis - (rovPos.X() - rovRefSatPos.X()) / rovRefSatDis;
				auto m = (rovPos.Y() - rovSatPos.Y()) / rovSatDis - (rovPos.Y() - rovRefSatPos.Y()) / rovRefSatDis;
				auto n = (rovPos.Z() - rovSatPos.Z()) / rovSatDis - (rovPos.Z() - rovRefSatPos.Z()) / rovRefSatDis;
				auto& types = SignalTypesOfSatelliteSystems.at(sat.System);
				auto f1 = FrequencyOfSignalType.at(types[0]);
				auto f2 = FrequencyOfSignalType.at(types[1]);
				auto lambda1 = LightSpeed / f1;
				auto lambda2 = LightSpeed / f2;
				auto rowStartIndex = 4 * satIndex;
				for (size_t i = 0; i < 4; i++)
				{
					B(rowStartIndex + i, 0) = l;
					B(rowStartIndex + i, 1) = m;
					B(rowStartIndex + i, 2) = n;
				}
				B(rowStartIndex + 2, 3 + 2 * satIndex) = lambda1;
				B(rowStartIndex + 3, 4 + 2 * satIndex) = lambda2;
				auto& refSatSdObs = singleDifferenceObs.at(refSat);
				auto P1 = sdObs.Data.at(types[0]).Pseudorange - refSatSdObs.Data.at(types[0]).Pseudorange;
				auto P2 = sdObs.Data.at(types[1]).Pseudorange - refSatSdObs.Data.at(types[1]).Pseudorange;
				auto L1 = sdObs.Data.at(types[0]).CarrierPhase - refSatSdObs.Data.at(types[0]).CarrierPhase;
				auto L2 = sdObs.Data.at(types[1]).CarrierPhase - refSatSdObs.Data.at(types[1]).CarrierPhase;
				double N1, N2;
				if (isFirst)
				{
					N1 = (L1 - P1) / lambda1;
					N2 = (L2 - P2) / lambda2;
				}
				else
				{
					N1 = a(2 * satIndex, 0) + X(3 + 2 * satIndex, 0);
					N2 = a(1 + 2 * satIndex, 0) + X(4 + 2 * satIndex, 0);
				}
				a(2 * satIndex, 0) = N1;
				a(1 + 2 * satIndex, 0) = N2;
				auto& baseRefSatPos = baseSppResult.SatelliteDatas.at(refSat).Coord;
				auto& baseSatPos = baseSppResult.SatelliteDatas.at(sat).Coord;
				auto baseSatDis = CartCoordinate::Distance(basePos, baseSatPos);
				auto baseRefSatDis = CartCoordinate::Distance(basePos, baseRefSatPos);
				auto rho = (rovSatDis - rovRefSatDis) - (baseSatDis - baseRefSatDis);
				W(rowStartIndex, 0) = P1 - rho;
				W(rowStartIndex + 1, 0) = P2 - rho;
				W(rowStartIndex + 2, 0) = L1 - rho - lambda1 * N1;
				W(rowStartIndex + 3, 0) = L2 - rho - lambda2 * N2;
				auto sigmaL = sqrt(0.0005);
				auto sigmaP = sqrt(0.5);
				n = ddObsNums[sat.System];
				if (sat.System != lastSys)
				{
					columnStartIndex += 4 * ddObsNums.at(lastSys);
					lastSys = sat.System;
				}
				for (size_t i = 0; i < n; i++)
				{
					if (rowStartIndex == columnStartIndex + 4 * i)
					{
						P(rowStartIndex, rowStartIndex) = n / (2 * sigmaP * sigmaP * (n + 1));
						P(rowStartIndex + 1, rowStartIndex + 1) = n / (2 * sigmaP * sigmaP * (n + 1));
						P(rowStartIndex + 2, rowStartIndex + 2) = n / (2 * sigmaL * sigmaL * (n + 1));
						P(rowStartIndex + 3, rowStartIndex + 3) = n / (2 * sigmaL * sigmaL * (n + 1));
						continue;
					}
					P(rowStartIndex, columnStartIndex + 4 * i) = -1 / (2 * sigmaP * sigmaP * (n + 1));
					P(rowStartIndex + 1, columnStartIndex + 4 * i + 1) = -1 / (2 * sigmaP * sigmaP * (n + 1));
					P(rowStartIndex + 2, columnStartIndex + 4 * i + 2) = -1 / (2 * sigmaL * sigmaL * (n + 1));
					P(rowStartIndex + 3, columnStartIndex + 4 * i + 3) = -1 / (2 * sigmaL * sigmaL * (n + 1));
				}
				satIndex++;
			}
			auto Bt = B.Transpose();
			Q = (Bt * P * B).Inverse();
			X = Q * Bt * P * W;
			rovPos.X() += X(0, 0);
			rovPos.Y() += X(1, 0);
			rovPos.Z() += X(2, 0);
			if (Vector(X.GetColumn(0)).Norm() <= IterationThreshold)
			{
				Matrix Q2 = { 2 * satNum,2 * satNum };
				for (size_t i = 0; i < Q2.RowsCount(); i++)
				{
					for (size_t j = 0; j < Q2.ColumnsCount(); j++)
					{
						Q2(i, j) = Q(i + 3, j + 3);
					}
				}
				Q = Q2;
				RtkResult result;
				result.Time = baseSppResult.Time;
				result.IsFixed = false;
				result.Ratio = std::numeric_limits<double>::quiet_NaN();
				result.RefPos = basePos;
				result.RovPos = rovPos;
				result.Sigma = sqrt((W.Transpose() * P * W)(0, 0) / (2 * satNum - 3));
				result.Pdop = sqrt(Q2(0, 0) + Q2(1, 1) + Q2(2, 2));
				return result;
			}
			isFirst = false;
		}
		return std::nullopt;
	}

	void MatToPtr(Matrix& m, double* p)
	{
		for (int i = 0; i < m.RowsCount(); i++)
		{
			for (int j = 0; j < m.ColumnsCount(); j++)
			{
				p[i + j * m.RowsCount()] = m(i, j);
			}
		}
	}

	RtkResult RtkFixed(RtkResult& rtkResult, std::map<Satellite, SatelliteObservation>& singleDifferenceObs, const SinglePointPositioning::SppResult& baseSppResult, const SinglePointPositioning::SppResult& rovSppResult, const std::map<SatelliteSystem, Satellite>& refSats)
	{
		auto row = Q.RowsCount();
		double* QPtr = new double[row * row];
		double* aPtr = new double[row];
		MatToPtr(Q, QPtr);
		MatToPtr(a, aPtr);
		double* F = new double[row * 2];
		double* s = new double[2];
		if (lambda(row, 2, aPtr, QPtr, F, s) != 0)
		{
			delete[] QPtr, aPtr, F, s;
			return rtkResult;
		}
		auto ratio = s[1] / s[0];
		rtkResult.Ratio = ratio;
		if (ratio <= 3)
		{
			delete[] QPtr, aPtr, F, s;
			return rtkResult;
		}
		Matrix f { row,1 };
		for (size_t i = 0; i < row; i++)
		{
			f(i, 0) = lround(F[i]);
		}
		int satNum = row / 2;
		std::map<SatelliteSystem, int> ddObsNums;
		for (auto& [sat, sdObs] : singleDifferenceObs)
		{
			auto& refSat = refSats.at(sat.System);
			if (refSat == sat)
				continue;
			ddObsNums[sat.System] ++;
		}
		auto& rovPos = rtkResult.RovPos;
		auto& basePos = BasePos;
		B = { 2 * satNum,3 };
		W = { 2 * satNum,1 };
		P = { 2 * satNum,2 * satNum };
		Matrix X;
		for (auto times = 0; times < 5; times++)
		{
			auto satIndex = 0, columnStartIndex = 0;
			auto lastSys = singleDifferenceObs.begin()->first.System;
			for (auto& [sat, sdObs] : singleDifferenceObs)
			{
				auto& refSat = refSats.at(sat.System);
				if (refSat == sat)
					continue;
				auto& rovRefSatPos = rovSppResult.SatelliteDatas.at(refSat).Coord;
				auto& rovSatPos = rovSppResult.SatelliteDatas.at(sat).Coord;
				auto rovSatDis = CartCoordinate::Distance(rovPos, rovSatPos);
				auto rovRefSatDis = CartCoordinate::Distance(rovPos, rovRefSatPos);
				auto l = (rovPos.X() - rovSatPos.X()) / rovSatDis - (rovPos.X() - rovRefSatPos.X()) / rovRefSatDis;
				auto m = (rovPos.Y() - rovSatPos.Y()) / rovSatDis - (rovPos.Y() - rovRefSatPos.Y()) / rovRefSatDis;
				auto n = (rovPos.Z() - rovSatPos.Z()) / rovSatDis - (rovPos.Z() - rovRefSatPos.Z()) / rovRefSatDis;
				auto& types = SignalTypesOfSatelliteSystems.at(sat.System);
				auto f1 = FrequencyOfSignalType.at(types[0]);
				auto f2 = FrequencyOfSignalType.at(types[1]);
				auto lambda1 = LightSpeed / f1;
				auto lambda2 = LightSpeed / f2;
				auto rowStartIndex = 2 * satIndex;
				for (auto i = 0; i < 2; i++)
				{
					B(rowStartIndex + i, 0) = l;
					B(rowStartIndex + i, 1) = m;
					B(rowStartIndex + i, 2) = n;
				}
				auto& baseSdObs = singleDifferenceObs.at(refSat);
				auto L1 = sdObs.Data.at(types[0]).CarrierPhase - baseSdObs.Data.at(types[0]).CarrierPhase;
				auto L2 = sdObs.Data.at(types[1]).CarrierPhase - baseSdObs.Data.at(types[1]).CarrierPhase;
				auto N1 = f(2 * satIndex, 0);
				auto N2 = f(2 * satIndex + 1, 0);
				auto& baseRefSatPos = baseSppResult.SatelliteDatas.at(refSat).Coord;
				auto& baseSatPos = baseSppResult.SatelliteDatas.at(sat).Coord;
				auto baseSatDis = CartCoordinate::Distance(basePos, baseSatPos);
				auto baseRefSatDis = CartCoordinate::Distance(basePos, baseRefSatPos);
				auto rho = (rovSatDis - rovRefSatDis) - (baseSatDis - baseRefSatDis);
				W(rowStartIndex, 0) = L1 - rho - lambda1 * N1;
				W(rowStartIndex + 1, 0) = L2 - rho - lambda2 * N2;
				auto sigmaL = 1;
				n = ddObsNums[sat.System];
				if (sat.System != lastSys)
				{
					columnStartIndex += 2 * ddObsNums.at(lastSys);
					lastSys = sat.System;
				}
				for (auto i = 0; i < n; i++)
				{
					if (rowStartIndex == columnStartIndex + 2 * i)
					{
						P(rowStartIndex, rowStartIndex) = n / (2 * sigmaL * sigmaL * (n + 1));
						P(rowStartIndex + 1, rowStartIndex + 1) = n / (2 * sigmaL * sigmaL * (n + 1));
						continue;
					}
					P(rowStartIndex, columnStartIndex + 2 * i) = -1 / (2 * sigmaL * sigmaL * (n + 1));
					P(rowStartIndex + 1, columnStartIndex + 2 * i + 1) = -1 / (2 * sigmaL * sigmaL * (n + 1));
				}
				satIndex++;
			}
			auto Bt = B.Transpose();
			Q = (Bt * P * B).Inverse();
			X = Q * Bt * P * W;
			rovPos.X() += X(0, 0);
			rovPos.Y() += X(1, 0);
			rovPos.Z() += X(2, 0);
			if (Vector(X.GetColumn(0)).Norm() <= IterationThreshold)
			{
				rtkResult.IsFixed = true;
				rtkResult.Sigma = sqrt((W.Transpose() * P * W)(0, 0) / (2 * satNum - 3));
				rtkResult.Pdop = sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
				delete[] QPtr, aPtr, F, s;
				return rtkResult;
			}
		}
		delete[] QPtr, aPtr, F, s;
		return rtkResult;
	}

	std::optional<RtkResult> Solve(const std::map<Satellite, SatelliteObservation>& rovObservations, const std::map<Satellite, SatelliteObservation>& baseObservations, const SinglePointPositioning::SppResult& rovSppResult, const SinglePointPositioning::SppResult& baseSppResult)
	{
		auto sdObs = BuildSingleDifferenceObservations(rovSppResult.SatelliteDatas, baseSppResult.SatelliteDatas, rovObservations, baseObservations);
		static OutlierDetector sdObsDetector;
		sdObs = sdObsDetector.Filter(baseSppResult.Time, sdObs);
		if (rovSppResult.State == false || baseSppResult.State == false || sdObs.size() < 3)
			return std::nullopt;
		auto refSats = FindReferenceSatellite(sdObs);
		if (refSats.size() < 1)
			return std::nullopt;
		auto rtkResultOrNull = RtkFloat(sdObs, baseSppResult, rovSppResult, refSats);
		if (!rtkResultOrNull.has_value())
			return std::nullopt;
		return RtkFixed(rtkResultOrNull.value(), sdObs, baseSppResult, rovSppResult, refSats);
	}
}