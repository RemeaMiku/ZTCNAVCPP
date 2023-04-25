// ZTCNAVCPP.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
// 编译时请将C++语言版本设置为C++20及以上

#include "OEMDecoder.hpp"
#include "OutlierDetector.h"
#include "Positioning.hpp"
#include "Socket.hpp"
#include "Config.hpp"

using namespace std;
using namespace std::filesystem;
using namespace std::chrono;

enum SynchronousState
{
	Synchronous,
	BaseIsNewer,
	RoverIsNewer
};

SynchronousState GetSynchronousState(const GpsTime& rovTime, const GpsTime& baseTime)
{
	auto offSet = baseTime - rovTime;
	if (offSet > 0.1)
		return BaseIsNewer;
	if (offSet < -0.1)
		return RoverIsNewer;
	return Synchronous;
}

void DoRtkByFile(const path& basePath, const path& rovPath, path& resPath)
{
	if (!exists(basePath) || !exists(rovPath))
	{
		cout << "输入路径有误\n";
		return;
	}
	ifstream rovStream { rovPath,ios::binary | ios::in };
	ifstream baseStream { basePath,ios::binary | ios::in };
	ofstream resStream { resPath,ios::out,ios::trunc };
	if (!resStream)
	{
		cout << "输出路径有误\n";
		return;
	}
	OEMDecoder<ifstream> decoder;
	OutlierDetector rovDetector;
	OutlierDetector baseDetector;
	auto rovRangeOrNull = decoder.Read(rovStream);
	auto baseRangeOrNull = decoder.Read(baseStream);
	auto state = Synchronous;
	string title { "NO. MODE STATUS GPST_WN GPST_SOW ROV_X ROV_Y ROV_Z ROV_B ROV_L ROV_H BASELINE_X BASELINE_Y BASELINE_Z RATIO PDOP SIGMA" };
	cout << title << endl;
	resStream << title << endl;
	auto totalNum = 0;
	while (rovRangeOrNull.has_value())
	{
		totalNum++;
		auto& rovRange = rovRangeOrNull.value();
		auto rovTime = rovRange.Header.Time;
		auto rovObservations = rovDetector.Filter(rovTime, rovRange.ObservationOf);
		auto rovSppResult = SinglePointPositioning::Solve(rovTime, rovObservations);
		if (state != BaseIsNewer)
			baseRangeOrNull = decoder.Read(baseStream);
		while (baseRangeOrNull.has_value())
		{
			auto& baseRange = baseRangeOrNull.value();
			auto baseTime = baseRange.Header.Time;
			auto baseObservations = baseDetector.Filter(baseTime, baseRange.ObservationOf);
			auto baseSppResult = SinglePointPositioning::Solve(baseTime, baseObservations);
			state = GetSynchronousState(rovTime, baseTime);
			if (state == Synchronous)
			{
				auto rtkResultOrNull = RealTimeKinematic::Solve(rovObservations, baseObservations, rovSppResult, baseSppResult);
				if (!rtkResultOrNull.has_value())
				{
					cout << format("{} RTK NORES\n", totalNum);
					resStream << format("{} RTK NORES\n", totalNum);
					break;
				}
				auto& rtkRes = rtkResultOrNull.value();
				cout << format("{} RTK ", totalNum) << rtkRes << '\n';
				resStream << format("{} RTK ", totalNum) << rtkRes << '\n';
				break;
			}
			if (state == BaseIsNewer)
			{
				cout << format("{} SPP\n", totalNum);
				resStream << format("{} SPP\n", totalNum);
				break;
			}
			if (state == RoverIsNewer)
				baseRangeOrNull = decoder.Read(baseStream);
		}
		rovRangeOrNull = decoder.Read(rovStream);
	}
	cout << format("解算完成，共{}历元\n", totalNum);
}

unsigned char rovBuf[40960];
unsigned char baseBuf[40960];

void UCharBufferToStream(unsigned char* buf, int len, stringstream& stream)
{
	if (len <= 0)
		return;
	stream.clear();
	for (auto i = 0; i < len; i++)
		stream << buf[i];
	return;
}

tuple<string, unsigned short> StringToIpWithPort(const string& str)
{
	auto delimiter = ':';
	auto size = str.size();
	auto pos = str.find(delimiter);
	auto ip = str.substr(0, pos);
	auto port = stoi(str.substr(pos + 1, size - pos));
	return { ip,port };
}

void DoRtkBySocket(const string& baseAddress, const string& rovAddress, const path& resPath)
{
	auto [baseIp, basePort] = StringToIpWithPort(baseAddress);
	auto [rovIp, rovPort] = StringToIpWithPort(rovAddress);
	ofstream resStream { resPath,ios::out };
	if (!resStream)
	{
		cout << "输出路径有误\n";
		return;
	}
	string title { "NO. MODE STATUS GPST_WN GPST_SOW ROV_X ROV_Y ROV_Z ROV_B ROV_L ROV_H BASELINE_X BASELINE_Y BASELINE_Z RATIO PDOP SIGMA" };
	cout << title << '\n';
	resStream << title << '\n';
	stringstream rovStream, baseStream;
	SOCKET rovListener, baseListener;
	if (!OpenSocket(baseListener, baseIp.c_str(), basePort) || !OpenSocket(rovListener, rovIp.c_str(), rovPort))
	{
		cout << "输入地址有误\n";
		return;
	}
	OEMDecoder<stringstream> decoder;
	OutlierDetector rovDetector;
	OutlierDetector baseDetector;
	auto totalNum = 0;
	auto rtkSolvedNum = 0;
	auto state = Synchronous;
	optional<OEMRANGE> rovRangeOrNull = nullopt;
	optional<OEMRANGE> baseRangeOrNull = nullopt;
	while (true)
	{
		totalNum++;
		auto start = utc_clock::now();
		auto rovMsgLen = recv(rovListener, (char*)rovBuf, 40960, 0);
		auto baseMsgLen = recv(baseListener, (char*)baseBuf, 40960, 0);
		cout << format("RovRecv:{}\n", rovMsgLen);
		cout << format("RefRecv:{}\n", baseMsgLen);
		UCharBufferToStream(rovBuf, rovMsgLen, rovStream);
		UCharBufferToStream(baseBuf, baseMsgLen, baseStream);
		if (state != RoverIsNewer)
			rovRangeOrNull = decoder.Read(rovStream);
		if (state != BaseIsNewer)
			baseRangeOrNull = decoder.Read(baseStream);
		auto hasRov = false, hasBase = false;
		OEMRANGE rovRange, baseRange;
		GpsTime rovTime, baseTime;
		map<Satellite, SatelliteObservation> rovObs, baseObs;
		SinglePointPositioning::SppResult rovSppRes, baseSppRes;
		if (rovRangeOrNull.has_value())
		{
			hasRov = true;
			rovRange = rovRangeOrNull.value();
			rovTime = rovRange.Header.Time;
			rovObs = rovDetector.Filter(rovTime, rovRange.ObservationOf);
			rovSppRes = SinglePointPositioning::Solve(rovTime, rovObs);
			cout << "RovRange ";
		}
		if (baseRangeOrNull.has_value())
		{
			hasBase = true;
			baseRange = baseRangeOrNull.value();
			baseTime = baseRange.Header.Time;
			baseObs = baseDetector.Filter(baseTime, baseRange.ObservationOf);
			baseSppRes = SinglePointPositioning::Solve(baseTime, baseObs);
			cout << "RefRange ";
		}
		if (hasRov && hasBase)
		{
			state = GetSynchronousState(rovTime, baseTime);
			switch (state)
			{
				case Synchronous:
					cout << "TryRtk:";
					{
						auto rtkResultOrNull = RealTimeKinematic::Solve(rovObs, baseObs, rovSppRes, baseSppRes);
						if (!rtkResultOrNull.has_value())
						{
							cout << format("{} RTK NORES\n", totalNum);
							resStream << format("{} RTK NORES\n", totalNum);
							break;
						}
						rtkSolvedNum++;
						auto& rtkRes = rtkResultOrNull.value();
						cout << format("{} RTK ", totalNum) << rtkRes << '\n';
						resStream << format("{} RTK ", totalNum) << rtkRes << '\n';
					}
					break;
				case BaseIsNewer:
					cout << format("{} SPP\n", totalNum);
					resStream << format("{} SPP\n", totalNum);
					break;
				case RoverIsNewer:
					cout << format("{} SPP\n", totalNum);
					resStream << format("{} SPP\n", totalNum);
					break;
				default:
					break;
			}
		}
		else
		{
			if (hasRov)
			{
				resStream << format("{} SPP\n", totalNum);
			}
			else
			{
				resStream << format("{} NODATA\n", totalNum);
			}
		}
		if (totalNum % 200 == 0)
		{
			stringstream tempRovStream;
			stringstream tempBaseStream;
			tempRovStream << rovStream.rdbuf();
			tempBaseStream << baseStream.rdbuf();
			rovStream = move(tempRovStream);
			baseStream = move(tempBaseStream);
			system("cls");
			cout << title << '\n';
		}
		auto end = utc_clock::now();
		auto duration = duration_cast<milliseconds>(end - start).count();
		cout << format("\n{}ms,{}/{}={:.4f}\%\n\n", duration, rtkSolvedNum, totalNum, rtkSolvedNum * 100.0 / totalNum);
		if (duration < 1000)
			Sleep(1000 - duration);
	}
}

void PrintConfig(Config& config)
{
	cout << "配置信息如下\n";
	cout << format("SolutionMode:{}\n", config.SolutionMode);
	cout << format("Sources:{}\n", config.SourceMode);
	for (auto& dataSource : config.SourceDictionary)
		cout << format("{}:{}\n", dataSource.first, dataSource.second);
	cout << "Parameters\n";
	for (auto& dataSource : config.ParameterDictionary)
		cout << format("{}:{}\n", dataSource.first, dataSource.second);
	cout << "Targets\n";
	for (auto& dataSource : config.TargetDictionary)
		cout << format("{}:{}\n", dataSource.first, dataSource.second.string());
	system("pause");
}

int main()
{
	ios::sync_with_stdio(false);
	cin.tie(0);
	cout.tie(0);
	auto configOrNull = ReadConfigFromXml("config.xml");
	if (configOrNull == nullopt)
	{
		cerr << "配置文件读取失败\n";
		exit(1);
	}
	auto& config = configOrNull.value();
	PrintConfig(config);
	if (config.SolutionMode == "Rtk")
	{
		BasePos = { config.ParameterDictionary["Base ECEF X Coord"],config.ParameterDictionary["Base ECEF Y Coord"] ,config.ParameterDictionary["Base ECEF Z Coord"] };
		auto& baseSource = config.SourceDictionary["Base"];
		auto& rovSource = config.SourceDictionary["Rover"];
		auto& resTarget = config.TargetDictionary["Result"];
		auto& logTarget = config.SourceDictionary["Log"];
		if (resTarget == "Auto")
			resTarget = format("{:%Y%m%d%H%M%S}.csv", time_point_cast<seconds>(utc_clock::now()));
		if (config.SourceMode == "File")
			DoRtkByFile(baseSource, rovSource, resTarget);
		if (config.SourceMode == "Socket")
			DoRtkBySocket(baseSource, rovSource, resTarget);
		return 0;
	}
	if (config.SolutionMode == "Spp")
	{
		cout << "DoSpp\n";
		return 0;
	}
	cout << "解算模式有误\n";
}