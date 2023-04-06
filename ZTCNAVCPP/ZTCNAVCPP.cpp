// ZTCNAVCPP.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
// 编译时请将C++语言版本设置为C++20及以上

#include <filesystem>
#include "OEMDecoder.hpp"
#include "OutlierDetector.h"
#include "Positioning.hpp"
#include "Socket.hpp"

using namespace std;
using namespace std::filesystem;
using namespace std::chrono;
//读取文件路径
path inputPath;
//输出文件路径
path outputPath;
//网络缓冲区
unsigned char buf[20480];
//输出标题
string title { "NUMBER STATUS GPST_WN GPST_SOW POS_X POS_Y POS_Z POS_B POS_L POS_H VEL_X VEL_Y VEL_Z CLKVELERR PDOP SPPSIGAMA SPVSIGMA SYS:SATNUM SYS:CLKERR" };

void RunByFile();
void RunBySocket();


int IsSynchronous(const GpsTime& rovTime, const GpsTime& refTime)
{
	auto offSet = refTime - rovTime;
	if (offSet > 0.1)
		return 1;
	if (offSet < -0.1)
		return -1;
	return 0;
}



int main()
{
	//关闭stdio同步，打消缓存
	ios::sync_with_stdio(false);
	//关闭cin和cout绑定
	cin.tie(0);
	cout.tie(0);

	/*auto refPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\short-baseline\\oem719-202203170900-1.bin";
	auto rovPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\short-baseline\\oem719-202203170900-2.bin";*/
	auto refPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\Zero-baseline\\oem719-202203031500-1.bin";
	auto rovPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\Zero-baseline\\oem719-202203031500-2.bin";

	ifstream rovStream { rovPath,ios::binary | ios::in };
	ifstream refStream { refPath,ios::binary | ios::in };

	OEMDecoder<ifstream> decoder;
	OutlierDetector rovDetector;
	OutlierDetector refDetector;
	OutlierDetector singleDifferenceDetector;
	auto rtkNum = 0, sppNum = 0;
	auto rovRangeOrNull = decoder.Read(rovStream);
	auto refRangeOrNull = decoder.Read(refStream);
	auto state = 0;
	while (rovRangeOrNull.has_value())
	{
		auto& rovRange = rovRangeOrNull.value();
		if (state != 1)
			refRangeOrNull = decoder.Read(refStream);
		while (refRangeOrNull.has_value())
		{
			auto& refRange = refRangeOrNull.value();
			auto rovTime = rovRange.Header.Time;
			auto refTime = refRange.Header.Time;
			state = IsSynchronous(rovTime, refTime);
			if (state == 0)
			{
				cout << format("RTK:rovTime{}-refTime{}\n", rovTime.ToString(), refTime.ToString());
				auto rovObservations = rovDetector.Filter(rovTime, rovRange.ObservationOf);
				auto refObservations = refDetector.Filter(refTime, refRange.ObservationOf);
				auto rovSppResult = SinglePointPositioning::Solve(rovTime, rovObservations);
				auto refSppResult = SinglePointPositioning::Solve(refTime, refObservations);
				if (rovSppResult.State == false || refSppResult.State == false)
					break;
				auto singleDifferenceObservations = RealTimeKinematic::BuildSingleDifferenceObservations(rovSppResult.SatelliteDatas, refSppResult.SatelliteDatas, rovObservations, refObservations);
				singleDifferenceObservations = singleDifferenceDetector.Filter(refTime, singleDifferenceObservations);
				auto refSats = RealTimeKinematic::FindReferenceSatellite(singleDifferenceObservations);
				for (auto& [sys, sat] : refSats)
				{
					cout << format("{}{}\n", Satellite::SystemCodeOfName.at(sys), sat.Id);
				}
				auto rtkFloatResOrNull = RealTimeKinematic::RtkFloat(singleDifferenceObservations, refSppResult, rovSppResult, refSats);
				if (rtkFloatResOrNull.has_value())
				{
					cout << rtkFloatResOrNull.value().RovPos.ToString() << endl;
					cout << RealTimeKinematic::RtkFixed(rtkFloatResOrNull.value(), singleDifferenceObservations, refSppResult, rovSppResult, refSats).value() << endl;
				}
				/*cout << "rov:" << SinglePointPositioning::Solve(rovTime, rovObservations) << endl;
				cout << "ref:" << SinglePointPositioning::Solve(refTime, refObservations) << endl;*/
				rtkNum++;
				break;
			}
			if (state == 1)
			{
				/*cout << format("SPP:rovTime:{}\n", rovTime.ToString());
				cout << "rov:" << SinglePointPositioning::Solve(rovTime, rovDetector.Filter(rovRange)) << endl;*/
				sppNum++;
				break;
			}
			if (state == -1)
			{
				refRangeOrNull = decoder.Read(refStream);
			}
		}
		rovRangeOrNull = decoder.Read(rovStream);
	}
	cout << rtkNum << endl;
	cout << sppNum << endl;
	/*bool byFile;
	while (true)
	{
		cout << "输入文件路径。输入\"n\"则使用网络" << "\n";
		cin >> inputPath;
		if (inputPath != "n")
		{
			if (!exists(inputPath))
			{
				cerr << "路径不存在!" << "\n";
				continue;
			}
			byFile = true;
			break;
		}
		else
		{
			byFile = false;
			break;
		}
	}
	outputPath = format(".\\{:%Y%m%d%H%M%S}.txt", time_point_cast<seconds>(utc_clock::now()));
	if (byFile)
	{
		auto start { utc_clock::now() };
		RunByFile();
		auto end { utc_clock::now() };
		cout << duration_cast<duration<double>>(end - start) << "\n";
		system("pause");
	}
	else
	{
		RunBySocket();
	}*/
}

//void RunByFile()
//{
//	ifstream is { inputPath,ios::binary | ios::in };
//	ofstream os { outputPath,ios::out };
//	OEMDecoder<ifstream> decoder;
//	cout << title << "\n";
//	os << title << "\n";
//	auto i { 0 };
//	while (!is.eof())
//	{
//		if (auto range = decoder.Read(is); range.has_value())
//		{
//			i++;
//			auto res { SinglePointPositioning::Solve(range.value()) };
//			cout << format("{} ", i) << res << "\n";
//			os << format("{} ", i) << res << "\n";
//		}
//	}
//}
//
//void RunBySocket()
//{
//	ofstream os { outputPath,ios::out };
//	OEMDecoder<stringstream> decoder;
//	SOCKET listener;
//	OpenSocket(listener, "47.114.134.129", 7190);
//	cout << title << "\n";
//	os << title << "\n";
//	auto i { 0 };
//	while (true)
//	{
//		auto n = recv(listener, (char*)buf, 20480, 0);
//		if (n >= 0)
//		{
//			stringstream is;
//			for (int j = 0; j < n; j++)
//			{
//				is << buf[j];
//			}
//			while (!is.eof())
//			{
//				if (auto range = decoder.Read(is); range.has_value())
//				{
//					i++;
//					auto res { SinglePointPositioning::Solve(range.value()) };
//					cout << format("{} ", i) << res << "\n";
//					os << format("{} ", i) << res << "\n";
//				}
//			}
//		}
//	}
//}
