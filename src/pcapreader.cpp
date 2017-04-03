#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/stat.h>

#include "lidar.h"
#include "pcapreader.h"

PcapReader::PcapReader(const char *filename) {
    PcapGlobalHeader pcapGlobalHeader;
    fd = open(filename, O_RDONLY, S_IREAD);
    read(fd, &pcapGlobalHeader, sizeof(PcapGlobalHeader)); /* this is overhead at beginning of file */
}

int PcapReader::readPacket(void **p) {
    int type;
    memset(&packetHeader, 0, sizeof(packetHeader));
    memset(&lidarPacketHeader, 0, sizeof(lidarPacketHeader));
    int size = read(fd, &packetHeader, sizeof(packetHeader));
    size = packetHeader.orig_len;
    read(fd, &lidarPacketHeader, sizeof(LidarPacketHeader));
    if (size == (lidarDataPacketSize + sizeof(LidarPacketHeader))) {
        read(fd, &lidarDataPacket, lidarDataPacketSize);
        *p = (void *)&lidarDataPacket;
        type = TypeLidarDataPacket;
    } else if (size == (lidarPositionPacketSize + sizeof(LidarPacketHeader))) {
        read(fd, &lidarPositionPacket, lidarPositionPacketSize);
        *p = (void *)&lidarPositionPacket;
        type = TypeLidarPositionPacket;
    } else {
        *p = 0;
        type = LidarPacketTypes;
    }
    return type;
}

