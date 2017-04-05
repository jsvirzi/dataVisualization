#include <TCanvas.h>
#include <TGraph.h>
#include <TAxis.h>

#include <iostream>
#include <stdio.h>

#include <math.h>

/* ROOT include files */
#include <TFile.h>
#include <TTree.h>
#include <fcntl.h>
#include <sys/stat.h>

/* local include files */
#include "lidar.h"
#include "pcapreader.h"
#include "gpsReferenceWeekConverter.h"
#include "utils.h"

using namespace std;

int main(int argc, char **argv) {
	int i;
	std::string ifile, xfile, ofile, sfile, wdir, fourcc_string = "MJPG", lfile = "lidar.pcap";
	for(i=1;i<argc;++i) {
		if (strcmp(argv[i], "-d") == 0) wdir = argv[++i]; /* working directory for data */
		else if(strcmp(argv[i], "-i") == 0) ifile = argv[++i];
		else if(strcmp(argv[i], "-l") == 0) lfile = argv[++i]; /* lidar */
		else if(strcmp(argv[i], "-x") == 0) xfile = argv[++i];
		else if(strcmp(argv[i], "-s") == 0) sfile = argv[++i]; /* sensor/gps data file */
		else if(strcmp(argv[i], "-o") == 0) ofile = argv[++i];
	}

    wdir = "/Users/jsvirzi/Documents/rigData/26-03-2017-05-16-48-6661";
	wdir = "/home/jsvirzi/projects/data/rig/26-03-2017-05-22-26-930";
	wdir = "/home/jsvirzi/projects/data/rig/03-04-2017-05-53-33";
	wdir = "/home/jsvirzi/projects/mapping/data/03-04-2017-07-32-04";
	wdir = "/home/jsvirzi/projects/data/rig/03-04-2017-07-32-04";

	ofile = wdir + "/daq.root";
	TFile fpOut(ofile.c_str(), "recreate");

	TTree *treeLidar = new TTree("lidar", "lidar");

	/* setup lidar */
	TTree *tree = treeLidar;
	const int LidarRevolutionsPerSecond = 20;
	const int MaxLidarPacketsPerSecond = 754;
	int maxLidarPacketsPerEvent = (MaxLidarPacketsPerSecond + LidarRevolutionsPerSecond - 1) / LidarRevolutionsPerSecond;
	int maxLidarPoints = nLidarChannels * nLidarBlocks * 2 * maxLidarPacketsPerEvent;
//	uint64_t evtTime;
	uint64_t *daqTime = new uint64_t [maxLidarPoints];
	int *channel = new int [maxLidarPoints];
	int *intensity = new int [maxLidarPoints];
	double *R = new double [maxLidarPoints];
	double *theta = new double [maxLidarPoints];
	double *phi = new double [maxLidarPoints];
	double *dphi = new double [maxLidarPoints];
	int nLidarPoints;
	tree->Branch("nLidarPoints", &nLidarPoints, "nLidarPoints/I");
//	tree->Branch("evtTime", &evtTime, "evtTime/l");
	tree->Branch("daqTime", daqTime, "daqTime[nLidarPoints]/l");
	tree->Branch("channel", channel, "channel[nLidarPoints]/I");
//	tree->Branch("azimuth", azimuth, "azimuth[nLidarPoints]/I");
	tree->Branch("intensity", intensity, "intensity[nLidarPoints]/I");
	tree->Branch("R", R, "R[nLidarPoints]/D");
	tree->Branch("theta", theta, "theta[nLidarPoints]/D");
	tree->Branch("phi", phi, "phi[nLidarPoints]/D");
	tree->Branch("dphi", dphi, "dphi[nLidarPoints]/D");

	lfile = wdir + "/lidar.pcap";
	PcapReader pcapReader(lfile.c_str());
	int lidarPacketType;
	void *p;

	bool newEvent = false;
	int nDataPackets = 0, nPositionPackets = 0;
	uint32_t previousTimestamp = 0, timestamp = 0;
	time_t timeBase = 0;
	double phiPrevious = 0.0;
	while((lidarPacketType = pcapReader.readPacket(&p)) != pcapReader.LidarPacketTypes) {
		if(lidarPacketType == pcapReader.TypeLidarDataPacket) {
			LidarDataPacket *lidarDataPacket = (LidarDataPacket *)p;
			LidarData *lidarData = pcapReader.pointCloud;

			/* read data packet and convert data */
			int nPoints = convertLidarPacketToLidarData(lidarDataPacket, lidarData, timeBase, -M_PI, M_PI);

			for(i=0;i<nPoints;++i) {
				LidarData *p = &lidarData[i];
				if(newEvent == false) {
					if(p->phi >= 0.0) {
						if(nLidarPoints > 0) {
							treeLidar->Fill();
						}
						nLidarPoints = 0;
						newEvent = true;
					}
				} else {
					if(p->phi < 0.0) {
						newEvent = false;
					}
				}
				daqTime[nLidarPoints] = p->timestamp;
				channel[nLidarPoints] = p->channel;
				intensity[nLidarPoints] = p->intensity;
				R[nLidarPoints] = p->R;
				theta[nLidarPoints] = p->theta;
				phi[nLidarPoints] = p->phi;
				dphi[nLidarPoints] = p->phi - phiPrevious;
				phiPrevious = p->phi;
				++nLidarPoints;
			}

			++nDataPackets;
		} else if (lidarPacketType == pcapReader.TypeLidarPositionPacket) {
			LidarPositionPacket *lidarPositionPacket = (LidarPositionPacket *)p;
			char nmeaSentence[sizeof(lidarPositionPacket->nmeaSentence) + 1];
			snprintf(nmeaSentence, sizeof(nmeaSentence), "%s", lidarPositionPacket->nmeaSentence);
			previousTimestamp = timestamp;
			timestamp = lidarPositionPacket->timestamp[3];
			timestamp = (timestamp << 8) | lidarPositionPacket->timestamp[2];
			timestamp = (timestamp << 8) | lidarPositionPacket->timestamp[1];
			timestamp = (timestamp << 8) | lidarPositionPacket->timestamp[0];
			++nPositionPackets;
			unsigned char valid = lidarPositionPacket->timestamp[4];
			char timeStr[32], dateStr[32];
			readFieldFromCsv(nmeaSentence, 1, timeStr, sizeof(timeStr));
			readFieldFromCsv(nmeaSentence, 9, dateStr, sizeof(dateStr));
			timeBase = hourTimeBaseInSeconds(dateStr, timeStr);
			printf("NMEA=[%s]. TIME(0x%2.2x)=0x%8.8x=%d. deltaT=%d. D=%d/P=%d. DATE/TIME=[%s/%s]\n",
				   nmeaSentence, valid, timestamp, timestamp, timestamp - previousTimestamp, nDataPackets,
				   nPositionPackets, timeStr, dateStr);
		}
	}

	TTree *treeVideo = new TTree("video", "video");

	/* setup video */
	tree = treeVideo;
	int nFrames;
	const int maxFrames = 16; /* maximum number of frames per event/lidar revolution */
	unsigned int *ordinal = new unsigned int [maxFrames];
	unsigned int *fileSize = new unsigned int [maxFrames];
	unsigned int *cpuPhase = new unsigned int [maxFrames];
    unsigned int *shutter = new unsigned int [maxFrames];
	unsigned int *gain = new unsigned int [maxFrames];
	unsigned int *exposure = new unsigned int [maxFrames];
	unsigned int *frameCounter = new unsigned int [maxFrames];
	uint64_t *sensorTimestamp = new uint64_t [maxFrames];
	uint64_t deltaSensorTimestamp;
	tree->Branch("nFrames", &nFrames, "nFrames/I");
	tree->Branch("daqTime", daqTime, "daqTime[nFrames]/l"); /* same as lidar */
	tree->Branch("ordinal", ordinal, "ordinal[nFrames]/i");
	tree->Branch("frameCounter", frameCounter, "frameCounter[nFrames]/i");
	tree->Branch("fileSize", fileSize, "fileSize[nFrames]/i");
	tree->Branch("cpuPhase", cpuPhase, "cpuPhase[nFrames]/i");
	tree->Branch("sensorTimestamp", sensorTimestamp, "sensorTimestamp[nFrames]/l");
	tree->Branch("deltaSensorTimestamp", &deltaSensorTimestamp, "deltaSensorTimestamp/l");
    tree->Branch("shutter", shutter, "shutter[nFrames]/i");
    tree->Branch("gain", gain, "gain[nFrames]/i");
    tree->Branch("exposure", exposure, "exposure[nFrames]/i");

    TTree *treeImu = new TTree("imu", "imu");
    tree = treeImu;
    int nImuPoints;
    const int maxImuPoints = 100;

	ssize_t nRead;
    size_t maxLineDim = 1024;
    char *line = new char [maxLineDim];
	FILE *fp[4];
	ifile = wdir + "/video-0.csv";
    fp[0] = fopen(ifile.c_str(), "r");
	ifile = wdir + "/video-1.csv";
    fp[1] = fopen(ifile.c_str(), "r");
	ifile = wdir + "/video-2.csv";
    fp[2] = fopen(ifile.c_str(), "r");
	ifile = wdir + "/video-3.csv";
    fp[3] = fopen(ifile.c_str(), "r");
	char frameCounterStr[64];
	char ordinalStr[64];
	char sensorTimestampStr[64];
	char shutterStr[32];
	char exposureStr[32];
	char gainStr[32];
	bool filesOk = true;
	deltaSensorTimestamp = 0;
	uint64_t previousSensorTimestamp = 0;
	while(filesOk) {
		for(i=0;i<4;++i) {
			nFrames = 1;
			if ((nRead = getline(&line, &maxLineDim, fp[i])) != -1) {
				printf("line = [%s]\n", line);
				readFieldFromCsv(line, 4, sensorTimestampStr, sizeof(sensorTimestampStr));
				readFieldFromCsv(line, 5, ordinalStr, sizeof(ordinalStr));
				readFieldFromCsv(line, 6, frameCounterStr, sizeof(frameCounterStr));
				readFieldFromCsv(line, 7, shutterStr, sizeof(shutterStr));
				readFieldFromCsv(line, 8, gainStr, sizeof(gainStr));
				readFieldFromCsv(line, 9, exposureStr, sizeof(exposureStr));
				sscanf(sensorTimestampStr, "%lu", &sensorTimestamp[0]);
				sscanf(ordinalStr, "%d", &ordinal[0]);
				sscanf(frameCounterStr, "%d", &frameCounter[0]);
				sscanf(shutterStr, "%d", &shutter[0]);
				sscanf(gainStr, "%d", &gain[0]);
				sscanf(exposureStr, "%d", &exposure[0]);
				cpuPhase[0] = i;
				if(previousSensorTimestamp == 0) {
					deltaSensorTimestamp = 0;
				} else {
					deltaSensorTimestamp = sensorTimestamp[0] - previousSensorTimestamp;
				}
				previousSensorTimestamp = sensorTimestamp[0];
				printf("timestamp=%lu ord=%u fc=%d sh=%u gain=%u exp=%u\n", sensorTimestamp[0], ordinal[0], frameCounter[0], shutter[0], gain[0], exposure[0]);
				treeVideo->Fill();
			} else {
				filesOk = false;
			}
		}
	}

	fpOut.Write();
	fpOut.Close();

	delete [] daqTime;
	delete [] channel;
//	delete [] azimuth;
	delete [] intensity;
	delete [] R;
	delete [] theta;
	delete [] phi;
}