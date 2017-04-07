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

	int i, eventId;
    TTree *tree;
    char filename[128];
    uint64_t *daqTime;

    uint64_t t0; /* this is actual start time of session, expressed as number of microseconds since *the* epoch */
    const uint64_t eventDuration = 50000; /* 50ms = 50000 microseconds */
    int nEvents, iEvent;
    std::string ofile, wdir;
    bool storeVideo = false, verbose = false, debug = false;
	for(i=1;i<argc;++i) {
		if (strcmp(argv[i], "-d") == 0) wdir = argv[++i]; /* working directory for data */
		else if(strcmp(argv[i], "-o") == 0) ofile = argv[++i];
		else if(strcmp(argv[i], "-video") == 0) storeVideo = true;
		else if(strcmp(argv[i], "-verbose") == 0) verbose = true;
		else if(strcmp(argv[i], "-debug") == 0) debug = true;
	}

    wdir = "/Users/jsvirzi/Documents/rigData/26-03-2017-05-16-48-6661";
	wdir = "/home/jsvirzi/projects/data/rig/26-03-2017-05-22-26-930";
	wdir = "/home/jsvirzi/projects/data/rig/03-04-2017-05-53-33";
	wdir = "/home/jsvirzi/projects/mapping/data/03-04-2017-07-32-04";
//	wdir = "/home/jsvirzi/projects/data/rig/03-04-2017-07-32-04";

    /* output file */
    if(ofile.length() == 0) {
        ofile = wdir + "/daq.root";
    }
	TFile fpOut(ofile.c_str(), "recreate");

    /*** setup video ***/

    /* video goes first. by construction, video is fired at precisely t, t + 50ms, t + 100ms, ...
     * where t is actual integral number of seconds since *the* epoch */

    TTree *treeVideo = new TTree("video", "video");
    tree = treeVideo;
    int nFrames, fileSize, maxJpegFileSize = 1024 * 1024;
    unsigned int ordinal, cpuPhase, shutter, gain, exposure, frameCounter;
    unsigned char *frameData = new unsigned char [maxJpegFileSize];
    uint64_t sensorTimestamp, deltaSensorTimestamp;
    tree->Branch("nFrames", &nFrames, "nFrames/I");
    tree->Branch("eventId", &eventId, "eventId/I");
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
    FILE *fp[4]; /* one for each compression phase */
    snprintf(filename, sizeof(filename), "%s/video-0.csv", wdir.c_str());
    fp[0] = fopen(filename, "r");
    snprintf(filename, sizeof(filename), "%s/video-1.csv", wdir.c_str());
    fp[1] = fopen(filename, "r");
    snprintf(filename, sizeof(filename), "%s/video-2.csv", wdir.c_str());
    fp[2] = fopen(filename, "r");
    snprintf(filename, sizeof(filename), "%s/video-3.csv", wdir.c_str());
    fp[3] = fopen(filename, "r");
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

                if(storeVideo) {
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

    tree->Write();

    nEvents = tree->GetEntries();

    fclose(fp[0]);
    fclose(fp[1]);
    fclose(fp[2]);
    fclose(fp[3]);

    /*** setup IMU ***/

    /* TODO this is a big kludge */

    // 1943,181951.400 @0
    // 1943,181955.405 @17
    t0 = secondsAtStartOfReferenceWeek(1943) * 1e6 + 181955405000;

    int nImuPoints;
    const int maxImuPoints = 100;
    daqTime = new uint64_t [maxImuPoints];
    uint64_t endOfEvent; /* event start/stop */
    double roll[maxImuPoints], pitch[maxImuPoints], yaw[maxImuPoints];
    double time, previousTime = 0.0;
    double northVelocity[maxImuPoints], eastVelocity[maxImuPoints], upVelocity[maxImuPoints];
    double lat[maxImuPoints], lon[maxImuPoints];
    TTree *treeIns = new TTree("ins", "ins");
    tree = treeIns;
    tree->Branch("nImu", &nImuPoints, "nImu/I");
    tree->Branch("eventId", &eventId, "eventId/I");
    tree->Branch("roll", roll, "roll[nImu]/D");
    tree->Branch("yaw", yaw, "yaw[nImu]/D");
    tree->Branch("pitch", pitch, "pitch[nImu]/D");
    tree->Branch("northVelocity", northVelocity, "northVelocity[nImu]/D");
    tree->Branch("eastVelocity", eastVelocity, "eastVelocity[nImu]/D");
    tree->Branch("upVelocity", upVelocity, "upVelocity[nImu]/D");
    tree->Branch("lat", lat, "lat[nImu]/D");
    tree->Branch("lon", lon, "lon[nImu]/D");
    tree->Branch("daqTime", daqTime, "daqTime[nImu]/l");

    int skip = 17, week;
    snprintf(filename, sizeof(filename), "%s/gpsimu.txt", wdir.c_str());
    FILE *fpi = fopen(filename, "r");
    if(fpi == 0) {
        printf("unable to open file [%s]\n", filename);
        return -1;
    }
    for(i=0;i<skip;++i) { getline(&line, &maxLineDim, fpi); }
    char msgId[32];

    eventId = 0;
    nImuPoints = 0;
    endOfEvent = t0 + eventDuration;
    while(getline(&line, &maxLineDim, fpi) != -1) {
        int length = strlen(line);
        char ch = line[length-1];
        if((ch == '\n') || (ch == '\r')) { line[length-1] = 0; }
        readFieldFromCsv(line, 0, msgId, sizeof(msgId));
        if(strcmp(msgId, "#INSPVAXA") == 0) {
            if(verbose) { printf("line=[%s]\n", line); }
            week = readIntFromCsv(line, 5);
            time = readDoubleFromCsv(line, 6);
            uint64_t microSeconds = (uint64_t) (time * 1.0e6);
            uint64_t timeOfDatum = secondsAtStartOfReferenceWeek(week) * 1e6 + microSeconds;
            if(timeOfDatum >= endOfEvent) {
                if(nImuPoints > 0) {
                    tree->Fill();
                    endOfEvent += eventDuration;
                    nImuPoints = 0;
                }
                ++eventId;
            }
            daqTime[nImuPoints] = timeOfDatum;
            lat[nImuPoints] = readDoubleFromCsv(line, 11);
            lon[nImuPoints] = readDoubleFromCsv(line, 12);
            northVelocity[nImuPoints] = readDoubleFromCsv(line, 15);
            eastVelocity[nImuPoints] = readDoubleFromCsv(line, 16);
            upVelocity[nImuPoints] = readDoubleFromCsv(line, 17);
            roll[nImuPoints] = readDoubleFromCsv(line, 18);
            pitch[nImuPoints] = readDoubleFromCsv(line, 19);
            yaw[nImuPoints] = readDoubleFromCsv(line, 20);
            verbose = debug = true;
            if(verbose) {
                printf("line=[%s]\n", line);
                printf("time = %lf lat = %.12lf lon = %.12lf \n", timeOfDatum, lat[nImuPoints], lon[nImuPoints]);
                printf("velocity = (N=%lf, E=%lf, U=%lf)\n", northVelocity[nImuPoints], eastVelocity[nImuPoints], upVelocity[nImuPoints]);
                printf("roll/pitch/yaw = (%lf, %lf, %lf)\n", roll[nImuPoints], pitch[nImuPoints], yaw[nImuPoints]);
                if(debug) { getchar(); }
            }
            ++nImuPoints;
        }
    }

    tree->Write();

    delete[] daqTime;

    /*** setup lidar ***/

	TTree *treeLidar = new TTree("lidar", "lidar");
	tree = treeLidar;
	const int LidarRevolutionsPerSecond = 20;
	const int MaxLidarPacketsPerSecond = 754;
	int maxLidarPacketsPerEvent = (MaxLidarPacketsPerSecond + LidarRevolutionsPerSecond - 1) / LidarRevolutionsPerSecond;
	int maxLidarPoints = nLidarChannels * nLidarBlocks * 2 * maxLidarPacketsPerEvent;
	daqTime = new uint64_t [maxLidarPoints];
	int *channel = new int [maxLidarPoints];
	int *intensity = new int [maxLidarPoints];
	double *R = new double [maxLidarPoints];
	double *theta = new double [maxLidarPoints];
	double *phi = new double [maxLidarPoints];
	double *dphi = new double [maxLidarPoints];
	int nLidarPoints;
	tree->Branch("nLidar", &nLidarPoints, "nLidar/I");
	tree->Branch("eventId", &eventId, "eventId/I");
	tree->Branch("daqTime", daqTime, "daqTime[nLidar]/l");
	tree->Branch("channel", channel, "channel[nLidar]/I");
	tree->Branch("intensity", intensity, "intensity[nLidar]/I");
	tree->Branch("R", R, "R[nLidar]/D");
	tree->Branch("theta", theta, "theta[nLidar]/D");
	tree->Branch("phi", phi, "phi[nLidar]/D");
	tree->Branch("dphi", dphi, "dphi[nLidar]/D");

    snprintf(filename, sizeof(filename), "%s/lidar.pcap", wdir.c_str());
	PcapReader pcapReader(filename);
	int lidarPacketType;
	void *p;

    eventId = 0;
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

    tree->Write();

    /* clean up from lidar session */
    delete [] daqTime;
    delete [] channel;
    delete [] intensity;
    delete [] R;
    delete [] theta;
    delete [] phi;

    /* close out root file */
    fpOut.Write();
    fpOut.Close();
}