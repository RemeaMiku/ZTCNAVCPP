#include "OEMBESTPOS.h"
#pragma warning(disable:26495)
OEMBESTPOS::OEMBESTPOS(const OEMMessageHeader& hd, double lat, double lon, double hgt) :Header(hd), Position(lat, lon, hgt) {}
std::string OEMBESTPOS::ToString() const noexcept
{
	return std::format("{} {} {} {:.4f} {:.4f} {:.4f}", Header.Time.ToString(), BaseStationId, Position.ToString(), LatitudeStandardDeviation, LongitudeStandardDeviation, HeightStandardDeviation);
}