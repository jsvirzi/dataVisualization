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
	TBranch *branchNLidarPoints = tree->Branch("nLidarPoints", &nLidarPoints, "nLidarPoints/I");
	TBranch *branchDaqTime = tree->Branch("daqTime", daqTime, "daqTime[nLidarPoints]/l");
	TBranch *branchChannel = tree->Branch("channel", channel, "channel[nLidarPoints]/I");
	TBranch *branchAzimuth = tree->Branch("azimuth", azimuth, "azimuth[nLidarPoints]/I");
	TBranch *branchIntensity = tree->Branch("intensity", intensity, "intensity[nLidarPoints]/I");
	TBranch *branchR = tree->Branch("R", R, "R[nLidarPoints]/F");
	TBranch *branchTheta = tree->Branch("theta", theta, "theta[nLidarPoints]/F");
	TBranch *branchPhi = tree->Branch("phi", phi, "phi[nLidarPoints]/F");

	TTree *treeVideo = new TTree("video", "video");
	tree = treeVideo;
	int nFrames;
	const int maxFrames = 16; /* maximum number of frames per event/lidar revolution */
	int *ordinal = new int [maxFrames];
	int *fileSize = new int [maxFrames];
	int *cpuPhase = new int [maxFrames];
	uint64_t *sensorTimestamp = new uint64_t [maxFrames];
	TBranch *branchDaqTime = tree->Branch("daqTime", daqTime, "daqTime[nFrames]/l"); /* same as lidar */
	TBranch *branchOrdinal = tree->Branch("ordinal", ordinal, "ordinal[nFrames]/I");
	TBranch *branchFileSize = tree->Branch("fileSize", fileSize, "fileSize[nFrames]/I");
	TBranch *branchCpuPhase = tree->Branch("cpuPhase", cpuPhase, "cpuPhase[nFrames]/I");
	TBranch *branchSensorTimestamp = tree->Branch("sensorTimestamp", sensorTimestamp, "sensorTimestamp[nFrames]/I");




	delete [] daqTime;
	delete [] channel;
	delete [] azimuth;
	delete [] intensity;
	delete [] R;
	delete [] theta;
	delete [] phi;
}