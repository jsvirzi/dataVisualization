//
// Created by jsvirzi on 4/1/17.
//

#ifndef PCAPREADER_H
#define PCAPREADER_H

class PcapReader {
	public:
	enum {
		TypeLidarDataPacket,
		TypeLidarPositionPacket,
		LidarPacketTypes,
	};
	PcapReader(const char *file);
	int readPacket(void **p);
	int fd;
#define NPOINTS (3600 * 16)
	LidarData pointCloud[NPOINTS];
	LidarPacketHeader lidarPacketHeader;
	LidarDataPacket lidarDataPacket;
	LidarPositionPacket lidarPositionPacket;
	PcapPacketHeader packetHeader;
};

#endif //DATAVISUALIZATION_PCAPREADER_H
