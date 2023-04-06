#include "OEMMessageHeader.h"

OEMMessageHeader::OEMMessageHeader(unsigned short week, int ms, unsigned char hdLen, unsigned short msgLen, unsigned short msgid) :Time(week, (double)ms / std::milli::den), HeaderLength(hdLen), MessageLength(msgLen), MessageType((OEMMessageType)msgid) {}