#ifndef LIDAR_H
#define LIDAR_H

#include <inttypes.h>
#include <pthread.h>

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
    int intensity, channel, azimuth;
    uint64_t timestamp;
} LidarData;

enum {
  StrongestReturn = 0x37,
  LastReturn = 0x38,
  DualReturn = 0x39
} ReturnModes;

double getAzimuth(LidarDataBlock *block);
int convertLidarPacketToLidarData(LidarDataPacket *lidarPacket, LidarData *lidarData, time_t timeBase, double minAzimuth, double maxAzimuth);
void saveLidarToText(LidarDataPacket *lidarPacket, const char *filename);

typedef struct { unsigned char header[42]; } LidarPacketHeader;

typedef struct { unsigned char header[42]; } PositionPacketHeader;

const int nLidarChannels = 16;
const int lidarDataPacketSize = 1206;
const int lidarBlockSize = 100;
const int lidarDatumSize = 3;
const int lidarPositionPacketSize = 512;

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
