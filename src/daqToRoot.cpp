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

	TTree *treeLidar = new TTree("lidar", "lidar");

	/* setup lidar */
	TTree *tree = treeLidar;
	const int LidarBlocks = 24;
	const int LidarRevolutionsPerSecond = 20;
	const int MaxLidarPacketsPerSecond = 754;
	int maxLidarPacketsPerEvent = (MaxLidarPacketsPerSecond + LidarRevolutionsPerSecond - 1) / LidarRevolutionsPerSecond;
	int nLidar, maxLidarPoints = nLidarChannels * nLidarBlocks * 2 * maxLidarPacketsPerEvent;
	uint64_t *daqTime = new uint64_t [maxLidarPoints];
	int *channel = new int [maxLidarPoints];
	int *azimuth = new int [maxLidarPoints];
	int *intensity = new int [maxLidarPoints];
	float *R = new float [maxLidarPoints];
	float *theta = new float [maxLidarPoints];
	float *phi = new float [maxLidarPoints];
	int nLidarPoints;
	tree->Branch("nLidarPoints", &nLidarPoints, "nLidarPoints/I");
	tree->Branch("daqTime", daqTime, "daqTime[nLidarPoints]/l");
	tree->Branch("channel", channel, "channel[nLidarPoints]/I");
	tree->Branch("azimuth", azimuth, "azimuth[nLidarPoints]/I");
	tree->Branch("intensity", intensity, "intensity[nLidarPoints]/I");
	tree->Branch("R", R, "R[nLidarPoints]/F");
	tree->Branch("theta", theta, "theta[nLidarPoints]/F");
	tree->Branch("phi", phi, "phi[nLidarPoints]/F");

	TTree *treeVideo = new TTree("video", "video");

	/* setup video */
	tree = treeVideo;
	int nFrames;
	const int maxFrames = 16; /* maximum number of frames per event/lidar revolution */
	int *ordinal = new int [maxFrames];
	int *fileSize = new int [maxFrames];
	int *cpuPhase = new int [maxFrames];
    int *shutter = new int [maxFrames];
    int *gain = new int [maxFrames];
    int *exposure = new int [maxFrames];
	int *frameCounter = new int [maxFrames];
	uint64_t *sensorTimestamp = new uint64_t [maxFrames];
	tree->Branch("nFrames", &nFrames, "nFrames/I");
	tree->Branch("daqTime", daqTime, "daqTime[nFrames]/l"); /* same as lidar */
	tree->Branch("ordinal", ordinal, "ordinal[nFrames]/I");
	tree->Branch("frameCounter", frameCounter, "frameCounter[nFrames]/I");
	tree->Branch("fileSize", fileSize, "fileSize[nFrames]/I");
	tree->Branch("cpuPhase", cpuPhase, "cpuPhase[nFrames]/I");
	tree->Branch("sensorTimestamp", sensorTimestamp, "sensorTimestamp[nFrames]/l");
    tree->Branch("shutter", shutter, "shutter[nFrames]/I");
    tree->Branch("gain", gain, "gain[nFrames]/I");
    tree->Branch("exposure", exposure, "exposure[nFrames]/I");

    TTree *treeImu = new TTree("imu", "imu");
    tree = treeImu;
    int nImuPoints;
    const int maxImuPoints = 100;

	ssize_t nRead;
    size_t maxLineDim = 1024;
    char *line = new char [maxLineDim];
    ifile = wdir + "/video-0.csv";
    FILE *fp = fopen(ifile.c_str(), "r");
	char frameCounterStr[64];
	char ordinalStr[64];
	char sensorTimestampStr[64];
	char shutterStr[32];
	char exposureStr[32];
	char gainStr[32];
	nFrames = 1;
    while((nRead = getline(&line, &maxLineDim, fp)) != -1) {
        printf("line = [%s]\n", line);
		readFieldFromCsv(line, 4, sensorTimestampStr, sizeof(sensorTimestampStr));
		readFieldFromCsv(line, 5, ordinalStr, sizeof(ordinalStr));
		readFieldFromCsv(line, 6, frameCounterStr, sizeof(frameCounterStr));
		readFieldFromCsv(line, 7, shutterStr, sizeof(shutterStr));
		readFieldFromCsv(line, 8, gainStr, sizeof(gainStr));
		readFieldFromCsv(line, 9, exposureStr, sizeof(exposureStr));
		sscanf(line, "%lu", &sensorTimestamp[0]);
		sscanf(line, "%d", &ordinal[0]);
		sscanf(line, "%d", &frameCounter[0]);
		sscanf(line, "%d", &shutter[0]);
		sscanf(line, "%d", &gain[0]);
		sscanf(line, "%d", &exposure[0]);
		printf("timestamp=%lu ord=%d fc=%d sh=%d gain=%d exp=%d\n", sensorTimestamp[0], ordinal[0], frameCounter[0], shutter[0], gain[0], exposure[0]);
    }

	delete [] daqTime;
	delete [] channel;
	delete [] azimuth;
	delete [] intensity;
	delete [] R;
	delete [] theta;
	delete [] phi;
}