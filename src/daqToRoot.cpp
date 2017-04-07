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
//	wdir = "/home/jsvirzi/projects/data/rig/03-04-2017-07-32-04";

    /* output file */
	ofile = wdir + "/daq.root";
	TFile fpOut(ofile.c_str(), "recreate");

    /*** setup lidar ***/

	TTree *treeLidar = new TTree("lidar", "lidar");
	TTree *tree = treeLidar;
	const int LidarRevolutionsPerSecond = 20;
	const int MaxLidarPacketsPerSecond = 754;
	int maxLidarPacketsPerEvent = (MaxLidarPacketsPerSecond + LidarRevolutionsPerSecond - 1) / LidarRevolutionsPerSecond;
	int maxLidarPoints = nLidarChannels * nLidarBlocks * 2 * maxLidarPacketsPerEvent;
	uint64_t *daqTime = new uint64_t [maxLidarPoints];
	int *channel = new int [maxLidarPoints];
	int *intensity = new int [maxLidarPoints];
	double *R = new double [maxLidarPoints];
	double *theta = new double [maxLidarPoints];
	double *phi = new double [maxLidarPoints];
	double *dphi = new double [maxLidarPoints];
	int nLidarPoints, eventId = 0;
	tree->Branch("nLidarPoints", &nLidarPoints, "nLidarPoints/I");
	tree->Branch("eventId", &eventId, "eventId/I");
	tree->Branch("daqTime", daqTime, "daqTime[nLidarPoints]/l");
	tree->Branch("channel", channel, "channel[nLidarPoints]/I");
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
	LidarData newDatum, oldDatum;
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
						++eventId;
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
				phi[nLidarPoints] = asin(sin(p->phi));
				dphi[nLidarPoints] = asin(sin(p->phi - phiPrevious));
				newDatum = *p;
				if(dphi[nLidarPoints] > 0.5) {
					printf("dphi=%f = %f - %f\n", dphi[nLidarPoints], phi[nLidarPoints], phiPrevious);
				}
				oldDatum = newDatum;
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

    /*** setup video ***/

	TTree *treeVideo = new TTree("video", "video");
	tree = treeVideo;
	int nFrames, fileSize, maxJpegFileSize = 1024 * 1024;
	unsigned int ordinal, cpuPhase, shutter, gain, exposure, frameCounter;
	unsigned char *frameData = new unsigned char [maxJpegFileSize];
	uint64_t sensorTimestamp, deltaSensorTimestamp;
	tree->Branch("nFrames", &nFrames, "nFrames/I");
	tree->Branch("fileSize", &fileSize, "fileSize/i");
	tree->Branch("ordinal", &ordinal, "ordinal/i");
	tree->Branch("frameCounter", &frameCounter, "frameCounter/i");
	tree->Branch("cpuPhase", &cpuPhase, "cpuPhase/i");
	tree->Branch("sensorTimestamp", &sensorTimestamp, "sensorTimestamp/l");
	tree->Branch("deltaSensorTimestamp", &deltaSensorTimestamp, "deltaSensorTimestamp/l");
    tree->Branch("shutter", &shutter, "shutter/i");
    tree->Branch("gain", &gain, "gain/i");
    tree->Branch("exposure", &exposure, "exposure/i");
	tree->Branch("data", frameData, "data[fileSize]/b");

    ssize_t nRead;
    size_t maxLineDim = 1024;
    char *line = (char *)malloc(maxLineDim);
	FILE *fp[4];
	ifile = wdir + "/video-0.csv";
    fp[0] = fopen(ifile.c_str(), "r");
	ifile = wdir + "/video-1.csv";
    fp[1] = fopen(ifile.c_str(), "r");
	ifile = wdir + "/video-2.csv";
    fp[2] = fopen(ifile.c_str(), "r");
	ifile = wdir + "/video-3.csv";
    fp[3] = fopen(ifile.c_str(), "r");
	char filename[128];
	bool filesOk = true, saveVideoFiles = true;
	deltaSensorTimestamp = 0;
	uint64_t previousSensorTimestamp = 0;
	while(filesOk) {
		for(i=0;i<4;++i) {
			nFrames = 1;
			if ((nRead = getline(&line, &maxLineDim, fp[i])) != -1) {
				printf("line = [%s]\n", line);
				readFieldFromCsv(line, 1, filename, sizeof(filename));

                sensorTimestamp = readUint64FromCsv(line, 4);
                ordinal = readIntFromCsv(line, 5);
                frameCounter = readIntFromCsv(line, 6);
                shutter = readIntFromCsv(line, 7);
                gain = readIntFromCsv(line, 8);
                exposure = readIntFromCsv(line, 9);

				if(saveVideoFiles) {
					std::string vfile = wdir + "/";
					vfile += filename;
					FILE *fp = fopen(vfile.c_str(), "r");
					fseek(fp, 0L, SEEK_END);
					fileSize = ftell(fp);
					rewind(fp);
					fileSize = fread(frameData, sizeof(unsigned char), maxJpegFileSize, fp);
				} else {
					fileSize = 0;
				}
				cpuPhase = i;
				if(previousSensorTimestamp == 0) {
					deltaSensorTimestamp = 0;
				} else {
					deltaSensorTimestamp = sensorTimestamp - previousSensorTimestamp;
				}
				previousSensorTimestamp = sensorTimestamp;
				printf("timestamp=%lu ord=%u fc=%d sh=%u gain=%u exp=%u\n", sensorTimestamp, ordinal, frameCounter, shutter, gain, exposure);
				treeVideo->Fill();
			} else {
				filesOk = false;
			}
		}
	}

    fclose(fp[0]);
    fclose(fp[1]);
    fclose(fp[2]);
    fclose(fp[3]);

	fpOut.Write();
	fpOut.Close();

    /*** setup IMU ***/

    int nImuPoints;
    const int maxImuPoints = 100;
    double roll, pitch, yaw, time, lat, lon, deltaTime, previousTime = 0.0;
    double northVelocity, eastVelocity, upVelocity;    TTree *treeIns = new TTree("ins", "ins");
    treeIns->Branch("roll", &roll, "roll/D");
    treeIns->Branch("pitch", &pitch, "pitch/D");
    treeIns->Branch("northVelocity", &northVelocity, "northVelocity/D");
    treeIns->Branch("eastVelocity", &eastVelocity, "eastVelocity/D");
    treeIns->Branch("upVelocity", &upVelocity, "upVelocity/D");
    treeIns->Branch("yaw", &yaw, "yaw/D");
    treeIns->Branch("lat", &lat, "lat/D");
    treeIns->Branch("lon", &lon, "lon/D");
    treeIns->Branch("time", &time, "time/D");
    treeIns->Branch("deltaTime", &deltaTime, "deltaTime/D");

    int skip = 0, verbose = 0, debug = 0;
    line = 0;
    ifile = wdir + "/imugps.txt";
    FILE *fpi = fopen(ifile.c_str(), "r");
    for(i=0;i<skip;++i) { getline(&line, &maxLineDim, fpi); }
    char msgId[32];
    while(getline(&line, &maxLineDim, fpi) != -1) {
        int length = strlen(line);
        char ch = line[length-1];
        if((ch == '\n') || (ch == '\r')) { line[length-1] = 0; }
        readFieldFromCsv(line, 0, msgId, sizeof(msgId));
        if(strcmp(msgId, "#INSPVAXA") == 0) {
            if(verbose) { printf("line=[%s]\n", line); }

            time = readDoubleFromCsv(line, 6);
            lat = readDoubleFromCsv(line, 11);
            lon = readDoubleFromCsv(line, 12);
            northVelocity = readDoubleFromCsv(line, 15);
            eastVelocity = readDoubleFromCsv(line, 16);
            upVelocity = readDoubleFromCsv(line, 17);
            roll = readDoubleFromCsv(line, 18);
            pitch = readDoubleFromCsv(line, 19);
            yaw = readDoubleFromCsv(line, 20);

            deltaTime = time - previousTime;
            previousTime = time;
            treeIns->Fill();
            if(verbose) {
                printf("time = %lf deltaTime = %lf lat = %.12lf lon = %.12lf \n", time, deltaTime, lat, lon);
                printf("velocity = (N=%lf, E=%lf, U=%lf)\n", northVelocity, eastVelocity, upVelocity);
                printf("roll/pitch/yaw = (%lf, %lf, %lf)\n", roll, pitch, yaw);
                if(debug) { getchar(); }
            }
        }
    }

    treeLidar->Write();
    treeIns->Write();
    treeVideo->Write();

    fpOut.Write();
    fpOut.Close();

    delete [] daqTime;
	delete [] channel;
	delete [] intensity;
	delete [] R;
	delete [] theta;
	delete [] phi;
}