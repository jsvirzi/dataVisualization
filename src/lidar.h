#ifndef LIDAR_H
#define LIDAR_H

#include <inttypes.h>
#include <pthread.h>
#include <math.h>

#define nLidarBlocks 12

typedef struct {
  uint8_t distance[2];
  uint8_t reflectivity;
} LidarChannelDatum;

typedef struct {
  uint16_t flag;
  uint8_t azimuthLo, azimuthHi;  // these need rearrangement
  LidarChannelDatum data[32];
} LidarDataBlock;

typedef struct {
  //    uint8 header[42];
  LidarDataBlock dataBlock[nLidarBlocks];
  uint8_t timestamp[4];
  uint8_t factoryField[2];
} LidarDataPacket;

typedef struct {
  //    unsigned char unused1[512];
  uint8_t unused1[198];
  uint8_t timestamp[4];
  uint8_t unused2[4];
  int8_t nmeaSentence[72];
  uint8_t unused3[234];
} LidarPositionPacket;

typedef struct {
    double R, theta, phi;  // R = distance, phi = azimuth, theta = altitude
    int intensity, channel;
    uint64_t timestamp;
} LidarData;

enum {
  StrongestReturn = 0x37,
  LastReturn = 0x38,
  DualReturn = 0x39
} ReturnModes;

double getAzimuth(LidarDataBlock *block);
int convertLidarPacketToLidarData(LidarDataPacket *lidarPacket, LidarData *lidarData, time_t timeBase);
void saveLidarToText(LidarDataPacket *lidarPacket, const char *filename);

typedef struct { unsigned char header[42]; } LidarPacketHeader;

typedef struct { unsigned char header[42]; } PositionPacketHeader;

const int nLidarChannels = 16;
const int lidarDataPacketSize = 1206;
const int lidarBlockSize = 100;
const int lidarDatumSize = 3;
const int lidarPositionPacketSize = 512;

const int VLP16Device = 0x22;
const double revolutionsPerMicrosecond = 20.0 * 1.0e-6;
const double radiansPerMicrosecond = 2.0 * M_PI * revolutionsPerMicrosecond;
const double cycleTimeBetweenFirings = 2.304;  // microseconds
const double rechargePeriod = 18.43;           // microseconds
const double cycleTimeBetweenBlocks = 55.296; // microseconds. according to doc, calculations = 55.294
const double distanceUnit = 0.002;             // 2 millimeters
const double radiansPerBlock = cycleTimeBetweenBlocks * 24 * radiansPerMicrosecond; /* 0.1667688 radians at 20Hz */

/*
 * https://wiki.wireshark.org/Development/LibpcapFileFormat
 */

typedef struct pcap_hdr_s {
  uint32_t magic_number;  /* magic number */
  uint16_t version_major; /* major version number */
  uint16_t version_minor; /* minor version number */
  int32_t thiszone;       /* GMT to local correction */
  uint32_t sigfigs;       /* accuracy of timestamps */
  uint32_t snaplen;       /* max length of captured packets, in octets */
  uint32_t network;       /* data link type */
} PcapGlobalHeader;

typedef struct pcaprec_hdr_s {
  uint32_t ts_sec;   /* timestamp seconds */
  uint32_t ts_usec;  /* timestamp microseconds */
  uint32_t incl_len; /* number of octets of packet saved in file */
  uint32_t orig_len; /* actual length of packet */
} PcapPacketHeader;

#endif
