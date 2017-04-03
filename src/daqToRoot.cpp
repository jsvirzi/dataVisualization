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

	ofile = wdir + "/daq.root";
	TFile fpOut(ofile.c_str(), "recreate");

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
	delete [] azimuth;
	delete [] intensity;
	delete [] R;
	delete [] theta;
	delete [] phi;
}