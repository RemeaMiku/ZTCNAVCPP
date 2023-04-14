#pragma once
#include <set>
#include <map>
#include <string>
#include <optional>
#include <filesystem>
#include "tinyxml.h"

struct Config
{
public:
	std::string SolutionMode;
	std::string SourceMode;
	std::map<std::string, std::string> SourceDictionary;
	std::map<std::string, double> ParameterDictionary;
	std::map<std::string, std::filesystem::path> TargetDictionary;
	const std::set<std::string> ParameterKeys
	{
		{ "Satellite Altitude Angle Threshold" },
		{ "Base ECEF X Coord" },
		{ "Base ECEF Y Coord" },
		{ "Base ECEF Z Coord" },
	};
};

inline std::optional<Config> ReadConfigFromXml(std::string filePath)
{
	using namespace std;
	TiXmlDocument document;
	if (!document.LoadFile(filePath.c_str(), TIXML_ENCODING_UTF8))
		return nullopt;
	Config config;
	auto element = document.RootElement();
	if (element == nullptr || element->ValueTStr() != "Config")
		return nullopt;
	element = element->FirstChildElement();
	if (element == nullptr || element->ValueTStr() != "SolutionMode")
		return nullopt;
	config.SolutionMode = element->GetText();
	element = element->NextSiblingElement();
	if (element == nullptr || element->ValueTStr() != "SourceDictionary")
		return nullopt;
	auto mode = element->Attribute("Mode");
	if (mode == nullptr)
		return nullopt;
	config.SourceMode = mode;
	auto child = element->FirstChildElement();
	while (child != nullptr)
	{
		TiXmlString key { child->Attribute("Key") };
		if (key.empty() || (key != "Rover" && key != "Base"))
			return nullopt;
		config.SourceDictionary[key.data()] = child->GetText();
		child = child->NextSiblingElement();
	}
	element = element->NextSiblingElement();
	if (element == nullptr || element->ValueTStr() != "ParameterDictionary")
		return nullopt;
	child = element->FirstChildElement();
	while (child != nullptr)
	{
		TiXmlString key { child->Attribute("Key") };
		if (key.empty() || !config.ParameterKeys.contains(key.c_str()))
			return nullopt;
		config.ParameterDictionary[key.data()] = std::stod(child->GetText());
		child = child->NextSiblingElement();
	}
	element = element->NextSiblingElement();
	if (element == nullptr || element->ValueTStr() != "TargetDictionary")
		return nullopt;
	child = element->FirstChildElement();
	while (child != nullptr)
	{
		TiXmlString key { child->Attribute("Key") };
		if (key.empty() || (key != "Result" && key != "Log"))
			return nullopt;
		config.TargetDictionary[key.data()] = child->GetText();
		child = child->NextSiblingElement();
	}
	return config;
}