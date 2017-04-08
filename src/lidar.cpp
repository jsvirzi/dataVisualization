#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <csignal>

#include "lidar.h"

/*
 * http://www.microhowto.info/howto/listen_for_and_receive_udp_datagrams_in_c.html
 * https://www.pacificsimplicity.ca/blog/reading-packet-hex-dumps-manually-no-wireshark
 * http://www.paraview.org/Wiki/VeloView
 */

extern void stop(int sig);

#if 0

taken literally from

git clone https://github.com/Kitware/VeloView.git
filename: VeloView/VelodyneHDL/vtkPacketFileWriter.cxx

const unsigned short vtkPacketFileWriter::LidarPacketHeader[21] = {
    0xffff, 0xffff, 0xffff, 0x7660,
    0x0088, 0x0000, 0x0008, 0x0045,
    0xd204, 0x0000, 0x0040, 0x11ff,
    0xaab4, 0xa8c0, 0xc801, 0xffff, // checksum 0xa9b4 //source ip 0xa8c0, 0xc801 is 192.168.1.200
    0xffff, 0x4009, 0x4009, 0xbe04, 0x0000};

//--------------------------------------------------------------------------------
const unsigned short vtkPacketFileWriter::PositionPacketHeader[21] = {
    0xffff, 0xffff, 0xffff, 0x7660,
    0x0088, 0x0000, 0x0008, 0x0045,
    0xd204, 0x0000, 0x0040, 0x11ff,
    0xaab4, 0xa8c0, 0xc801, 0xffff, // checksum 0xa9b4 //source ip 0xa8c0, 0xc801 is 192.168.1.200
    0xffff, 0x7420, 0x7420, 0x0802, 0x0000};

#endif

static LidarPacketHeader dummyLidarPacketHeader = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x60, 0x76, 0x88, 0x00, 0x00,
	0x00, 0x08, 0x00, 0x45, 0x00, 0x04, 0xd2, 0x00, 0x00, 0x40, 0x00,
	0xff, 0x11, 0xb4, 0xaa, 0xc0, 0xa8, 0x01, 0xc8, 0xff, 0xff, 0xff,
	0xff, 0x09, 0x40, 0x09, 0x40, 0x04, 0xbe, 0x00, 0x00};

static PositionPacketHeader dummyPositionPacketHeader = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x60, 0x76, 0x88, 0x00, 0x00,
	0x00, 0x08, 0x00, 0x45, 0x00, 0x04, 0xd2, 0x00, 0x00, 0x40, 0x00,
	0xff, 0x11, 0xb4, 0xaa, 0xc0, 0xa8, 0x01, 0xc8, 0xff, 0xff, 0xff,
	0xff, 0x20, 0x74, 0x20, 0x74, 0x02, 0x08, 0x00, 0x00};

void saveLidarDataPacket(LidarDataPacket *lidarDataPacket, int fd);
void saveLidarPositionPacket(LidarPositionPacket *lidarPositionPacket, int fd);

const double laserPolarAngle[nLidarChannels] = {
	-15.0 * M_PI / 180.0, 1.0 * M_PI / 180.0,   -13.0 * M_PI / 180.0,
	3.0 * M_PI / 180.0,   -11.0 * M_PI / 180.0, 5.0 * M_PI / 180.0,
	-9.0 * M_PI / 180.0,  7.0 * M_PI / 180.0,   -7.0 * M_PI / 180.0,
	9.0 * M_PI / 180.0,   -5.0 * M_PI / 180.0,  11.0 * M_PI / 180.0,
	-3.0 * M_PI / 180.0,  13.0 * M_PI / 180.0,  -1.0 * M_PI / 180.0,
	15.0 * M_PI / 180.0};

int histogramIntensity[nLidarChannels][256];
int histogramAzimuth[nLidarChannels][256];
int histogramIntensityAzimuth[nLidarChannels][256 * 256];
void analyzePointCloud(LidarData *data, int nPoints);

int convertLidarPacketToLidarData(LidarDataPacket *lidarDataPacket, LidarData *lidarData, time_t timeBase) {
	int iBlock, iChannel, nPoints = 0;
	double azimuth[nLidarBlocks];

	/*
	 * find delta azimuth between blocks. in principle, one only needs to take the
	 * difference between two blocks
	 * we will get a better estimate by averaging over blocks in a packet
	 * TODO this will fail at phi = 0 (where the cable inserts into the lidar puck)
	 */
	double interpolatedDeltaAzimuth = 0.0;
	LidarDataBlock *lidarDataBlock = &lidarDataPacket->dataBlock[0];
	azimuth[0] = getAzimuth(lidarDataBlock);
	for (iBlock = 1; iBlock < nLidarBlocks; ++iBlock) {
		lidarDataBlock = &lidarDataPacket->dataBlock[iBlock];
		azimuth[iBlock] = getAzimuth(lidarDataBlock);
		double a = azimuth[iBlock] - azimuth[iBlock - 1];
		interpolatedDeltaAzimuth += (a * a);
	}
	interpolatedDeltaAzimuth = 0.5 * sqrt(interpolatedDeltaAzimuth / (nLidarBlocks - 1));

	int sequenceIndex = 0;
	for (iBlock = 0; iBlock < nLidarBlocks; ++iBlock) {
		LidarDataBlock *lidarDataBlock = &lidarDataPacket->dataBlock[iBlock];
		double azimuth0 = azimuth[iBlock];
		for (iChannel = 0; iChannel < nLidarChannels; ++iChannel) {
			LidarChannelDatum *datum = &lidarDataBlock->data[iChannel];
			double a = distanceUnit * datum->distance[1];
			lidarData->R = 256.0 * a + distanceUnit * datum->distance[0];
			lidarData->phi = azimuth0 + iChannel * cycleTimeBetweenFirings * radiansPerMicrosecond;
			lidarData->theta = laserPolarAngle[iChannel];
			lidarData->intensity = datum->reflectivity;
			lidarData->channel = iChannel;
			lidarData->timestamp = timeBase * 1000000; /* epoch seconds to microseconds */
			uint64_t daqTimestamp = cycleTimeBetweenBlocks * sequenceIndex + cycleTimeBetweenFirings * iChannel;
			lidarData->timestamp += daqTimestamp; /* daq timestamp since top of hour */
			++lidarData;
			++nPoints;
		}
		++sequenceIndex;
		azimuth0 += interpolatedDeltaAzimuth;
		for (iChannel = 0; iChannel < nLidarChannels; ++iChannel) {
			LidarChannelDatum *datum = &lidarDataBlock->data[nLidarChannels + iChannel];
			double a = distanceUnit * datum->distance[1];
			lidarData->R = 256.0 * a + distanceUnit * datum->distance[0];
			lidarData->phi = azimuth0 + iChannel * cycleTimeBetweenFirings * radiansPerMicrosecond;
			lidarData->theta = laserPolarAngle[iChannel];
			lidarData->intensity = datum->reflectivity;
			lidarData->channel = iChannel;
			lidarData->timestamp = timeBase * 1000000; /* epoch seconds to microseconds */
			uint64_t daqTimestamp = cycleTimeBetweenBlocks * sequenceIndex + cycleTimeBetweenFirings * iChannel;
			lidarData->timestamp += daqTimestamp; /* daq timestamp since top of hour */
			++lidarData;
			++nPoints;
		}
		++sequenceIndex;
	}
	return nPoints;
}

int isBeginningPacket(LidarDataPacket *packet) {
	LidarDataBlock *lidarDataBlock = &packet->dataBlock[0];
	double azimuth = getAzimuth(lidarDataBlock);
	double c = cos(azimuth), s = sin(azimuth);
	if ((fabs(c - 1.0) < 0.1) && (fabs(s - 0.0) < 0.1)) { return 1; }
	else { return 0; }
}

void dumpLidarPacket(LidarDataPacket *packet) {
	int iBlock;
	for (iBlock = 0; iBlock < nLidarBlocks; ++iBlock) {
		LidarDataBlock *lidarDataBlock = &packet->dataBlock[iBlock];
		double azimuth = getAzimuth(lidarDataBlock);
		printf("flag(%d) = 0x%x. azimuth = %.3f(rad) = %.3f(deg)\n", iBlock,
			   packet->dataBlock[iBlock].flag, azimuth, azimuth * 180.0 / M_PI);
		LidarChannelDatum *datum = lidarDataBlock->data;
		int iData;
		for (iData = 0; iData < 32; ++iData, ++datum) {
			printf("0x%2.2x ", datum->reflectivity);
		}
		printf("\n");
	}
	getchar();
}

/*
 * getAzimuth() returns a value between -PI and PI
 * the lidar system reports values in degrees between 0 and 359.99
 */
double getAzimuth(LidarDataBlock *block) {
	unsigned char azimuthLo = block->azimuthLo;
	unsigned char azimuthHi = block->azimuthHi;
	int azimuthData = azimuthHi;
	azimuthData = (azimuthData << 8) | azimuthLo;
	double azimuth = azimuthData * M_PI / 18000.0 - M_PI;  // azimuth is reported in 0.01 degrees. convert to radians
	return azimuth;
}
