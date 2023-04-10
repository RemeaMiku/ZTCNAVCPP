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

//输出标题
string title { "NUMBER STATUS GPST_WN GPST_SOW POS_X POS_Y POS_Z POS_B POS_L POS_H VEL_X VEL_Y VEL_Z CLKVELERR PDOP SPPSIGAMA SPVSIGMA SYS:SATNUM SYS:CLKERR" };

void RunByFile();
void RunBySocket();

enum SynchronousState
{
	Synchronous,
	RefBehindRov,
	RovBehindRef
};

SynchronousState GetSynchronousState(const GpsTime& rovTime, const GpsTime& refTime)
{
	auto offSet = refTime - rovTime;
	if (offSet > 0.1)
		return RefBehindRov;
	if (offSet < -0.1)
		return RovBehindRef;
	return Synchronous;
}

int main()
{
	//关闭stdio同步，打消缓存
	ios::sync_with_stdio(false);
	//关闭cin和cout绑定
	cin.tie(0);
	cout.tie(0);

	RunBySocket();
}

void RunByFile()
{
	//auto refPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\short-baseline\\oem719-202203170900-1.bin";
	//auto rovPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\short-baseline\\oem719-202203170900-2.bin";
	auto refPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\Zero-baseline\\oem719-202203031500-1.bin";
	auto rovPath = "D:\\RemeaMiku study\\course in progress\\卫星导航算法与程序设计\\Zero-baseline\\oem719-202203031500-2.bin";

	ifstream rovStream { rovPath,ios::binary | ios::in };
	ifstream refStream { refPath,ios::binary | ios::in };

	OEMDecoder<ifstream> decoder;
	OutlierDetector rovDetector;
	OutlierDetector refDetector;
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
			state = GetSynchronousState(rovTime, refTime);
			if (state == 0)
			{
				auto rovObservations = rovDetector.Filter(rovTime, rovRange.ObservationOf);
				auto refObservations = refDetector.Filter(refTime, refRange.ObservationOf);
				auto rovSppResult = SinglePointPositioning::Solve(rovTime, rovObservations);
				auto refSppResult = SinglePointPositioning::Solve(refTime, refObservations);
				if (rovSppResult.State == false || refSppResult.State == false)
				{
					cout << "F" << endl;
					break;
				}
				auto rtkResultOrNull = RealTimeKinematic::Solve(rovObservations, refObservations, rovSppResult, refSppResult);
				if (!rtkResultOrNull.has_value())
				{
					cout << "F" << endl;
					break;
				}
				cout << rtkResultOrNull.value() << endl;
				break;
			}
			if (state == 1)
			{
				cout << "SPP" << endl;
				break;
			}
			if (state == -1)
			{
				refRangeOrNull = decoder.Read(refStream);
			}
		}
		rovRangeOrNull = decoder.Read(rovStream);
	}
}

unsigned char rovBuf[20480];
unsigned char refBuf[20480];

stringstream UCharBufferToStream(unsigned char* buf, int len)
{
	if (len <= 0)
		return{};
	stringstream stream;
	for (auto i = 0; i < len; i++)
	{
		stream << buf[i];
	}
	return stream;
}

void RunBySocket()
{
	SOCKET rovListener, refListener;
	OpenSocket(refListener, "47.114.134.129", 7190);
	OpenSocket(rovListener, "8.140.46.126", 5002);
	OEMDecoder<stringstream> decoder;
	OutlierDetector rovDetector;
	OutlierDetector refDetector;
	optional<OEMRANGE> refRangeOrNull = nullopt;
	optional<OEMRANGE> rovRangeOrNull = nullopt;
	optional<GpsTime> cachedRefTime = nullopt;
	optional<map<Satellite, SatelliteObservation>> cachedRefObs = nullopt;
	auto total = 0;
	auto solved = 0;
	while (true)
	{
		auto start = utc_clock::now();
		auto rovMsgLen = recv(rovListener, (char*)rovBuf, 20480, 0);
		auto refMsgLen = recv(refListener, (char*)refBuf, 20480, 0);
		cout << format("RovRecv:{}\n", rovMsgLen);
		cout << format("RefRecv:{}\n", refMsgLen);
		auto rovStream = UCharBufferToStream(rovBuf, rovMsgLen);
		auto refStream = UCharBufferToStream(refBuf, refMsgLen);
		rovRangeOrNull = decoder.Read(rovStream);
		refRangeOrNull = decoder.Read(refStream);
		total++;
		auto hasRov = false, hasRef = false;
		OEMRANGE rovRange, refRange;
		GpsTime rovTime, refTime;
		map<Satellite, SatelliteObservation> rovObs, refObs;
		SinglePointPositioning::SppResult rovSppRes, refSppRes;
		if (rovRangeOrNull.has_value())
		{
			hasRov = true;
			rovRange = rovRangeOrNull.value();
			rovTime = rovRange.Header.Time;
			rovObs = rovDetector.Filter(rovTime, rovRange.ObservationOf);
			rovSppRes = SinglePointPositioning::Solve(rovTime, rovObs);
			cout << "RovRange ";
		}
		if (refRangeOrNull.has_value())
		{
			hasRef = true;
			refRange = refRangeOrNull.value();
			refTime = refRange.Header.Time;
			refObs = refDetector.Filter(refTime, refRange.ObservationOf);
			refSppRes = SinglePointPositioning::Solve(refTime, refObs);
			cout << "RefRange ";
		}
		if (hasRov && hasRef)
		{
			auto state = GetSynchronousState(rovTime, refTime);
			switch (state)
			{
				case Synchronous:
					cachedRefTime = nullopt;
					cout << "TryRtk:";
					{
						auto rtkResultOrNull = RealTimeKinematic::Solve(rovObs, refObs, rovSppRes, refSppRes);
						if (!rtkResultOrNull.has_value())
						{
							cout << "F\n";
							break;
						}
						solved++;
						cout << rtkResultOrNull.value() << endl;
					}
					break;
				case RefBehindRov:
					cout << "RefBehind";
					if (!cachedRefTime.has_value())
					{
						cout << " TrySpp:\n";
						cachedRefTime = refTime;
						cachedRefObs = refObs;
						break;
					}
					{
						auto timeTemp = refTime;
						auto obsTemp = refObs;
						refTime = cachedRefTime.value();
						refObs = cachedRefObs.value();
						cachedRefTime = timeTemp;
						cachedRefObs = obsTemp;
						if (GetSynchronousState(rovTime, refTime) != Synchronous)
						{
							cout << "\n";
							break;
						}
						refSppRes = SinglePointPositioning::Solve(refTime, refObs);
						cout << " TryRtk:";
						{
							auto rtkResultOrNull = RealTimeKinematic::Solve(rovObs, refObs, rovSppRes, refSppRes);
							if (!rtkResultOrNull.has_value())
							{
								cout << "F\n";
								break;
							}
							solved++;
							cout << rtkResultOrNull.value() << endl;
						}
					}
					break;
				case RovBehindRef:
					cout << "RovBehind TrySpp:\n";
					cachedRefTime = nullopt;
					break;
				default:
					break;
			}
		}
		if (hasRov && !hasRef)
		{
			if (!cachedRefTime.has_value())
				cout << '\n';
			else
			{
				refTime = cachedRefTime.value();
				if (GetSynchronousState(rovTime, refTime) != Synchronous)
					cout << '\n';
				else
				{
					refObs = cachedRefObs.value();
					cachedRefTime = nullopt;
					refSppRes = SinglePointPositioning::Solve(refTime, refObs);
					cout << "TryRtk:";
					auto rtkResultOrNull = RealTimeKinematic::Solve(rovObs, refObs, rovSppRes, refSppRes);
					if (!rtkResultOrNull.has_value())
						cout << "F\n";
					else
					{
						solved++;
						cout << rtkResultOrNull.value() << endl;
					}
				}
			}
		}
		if (!hasRov && hasRef)
		{
			cachedRefTime = refTime;
			cachedRefObs = refObs;
		}
		if (!hasRov)
			cout << '\n';
		cout << format("{}/{}\n\n", solved, total);
		auto end = utc_clock::now();
		auto duration = duration_cast<milliseconds>(end - start).count();
		if (duration <= 1000)
			Sleep(1000 - duration);
	}
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
