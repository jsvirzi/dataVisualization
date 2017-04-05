#include "opencv2/opencv.hpp"

#include <TCanvas.h>
#include <TGraph.h>
#include <TAxis.h>

#include <iostream>
#include <stdio.h>

#include <math.h>
#include <TFile.h>
#include <TTree.h>
#include <fcntl.h>
#include <sys/stat.h>

#include "lidar.h"
#include "pcapreader.h"
#include "utils.h"

using namespace cv;
using namespace std;

const long TimeBetweenGpsAndUnixEpochs = 315964782;

#define ESC 27

void prettyGraph(TGraph *graph, float xMin, float xMax, float yMin, float yMax) {
    float lineWidth = 2.0;
    int markerStyle = kFullDotMedium;
    int markerColor = kRed;
    graph->GetXaxis()->SetLabelOffset(999);
    graph->GetXaxis()->SetLabelSize(0);
    graph->GetYaxis()->SetLabelOffset(999);
    graph->GetYaxis()->SetLabelSize(0);
    graph->GetXaxis()->SetRangeUser(xMin, xMax);
    graph->GetYaxis()->SetNdivisions(0, 0);
    graph->SetMinimum(yMin);
    graph->SetMaximum(yMax);
    graph->SetLineColor(kBlue);
    graph->SetLineWidth(lineWidth);
    graph->SetMarkerStyle(markerStyle);
    graph->SetMarkerColor(markerColor);
}

void getMinMax(float *x, int n, float *xMin, float *xMax) {
	*xMin = 999.0;
	*xMax = -999.0;
	for(int i=0;i<n;++i) {
		float datum = x[i];
		if (datum < *xMin) { *xMin = datum; }
		if (datum > *xMax) { *xMax = datum; }
	}
}

int AccId = 2, GyrId = 3, MagId = 4;

int readFloatArrayFromCsv(const char *filename, int srcId, float *x, float *y, float *z, long *t, int max) {
	FILE *fp = fopen(filename, "r");
	if(fp == 0) return 0;
	char *line = 0;
	size_t len = 0;
	int bytes;
	bool flag;
	char result[128];
	float vx, vy, vz;
	long vt;
	int index = 0;
	while((bytes = getline(&line, &len, fp)) != -1) {
		if(line[bytes-1] == 0x0a) line[bytes-1] = 0;
		if(line[bytes-1] == 0x0d) line[bytes-1] = 0;
		// printf("line read = [%s]\n", line);
		// flag = readFieldFromCsv(line, 0, result, sizeof(result)); 
		// printf("field 0 = %s\n", result);
		flag = readFieldFromCsv(line, 1, result, sizeof(result)); 
		if(flag) {
			int id = atoi(result);
			if(id == srcId) {
				// printf("field 1 = %s\n", result);
				readFieldFromCsv(line, 2, result, sizeof(result)); 
				// printf("field 2 = %s\n", result);
				sscanf(result, "%f", &vx);
				readFieldFromCsv(line, 3, result, sizeof(result)); 
				// printf("field 3 = %s\n", result);
				sscanf(result, "%f", &vy);
				readFieldFromCsv(line, 4, result, sizeof(result)); 
				// printf("field 4 = %s\n", result);
				sscanf(result, "%f", &vz);
				readFieldFromCsv(line, 5, result, sizeof(result)); 
				// printf("field 5 = %s\n", result);
				sscanf(result, "%ld", &vt);

				// if((index % 10000) == 0 || (index > 400000)) { printf("index = %d\n", index); }

				if(index < max) {
					// printf("X=%f/Y=%f/Z=%f/T=%ld\n", vx, vy, vz, vt);
					x[index] = vx;
					y[index] = vy;
					z[index] = vz;
					t[index] = vt - t[0];
					++index;
				} else {
					break;
				}
				// printf("X=%f/Y=%f/Z=%f/T=%ld\n", vx, vy, vz, vt);
			}
			// flag = readFieldFromCsv(line, 3, result, sizeof(result)); 
			// flag = readFieldFromCsv(line, 4, result, sizeof(result)); 
			// flag = readFieldFromCsv(line, 5, result, sizeof(result)); 
		}
	}
	printf("read %d\n", index);
	free(line);
	fclose(fp);
	return index;
} 

#define MaxImu 1000000
float yawData[MaxImu], pitchData[MaxImu], rollData[MaxImu], sysTimeData[MaxImu], evtTimeData[MaxImu];

int main(int argc, char **argv) {

	int i, frameBufferSize = 8, skip = 0;
	bool is_color = true, smooth = false;
	std::string ifile, xfile, ofile, sfile, wdir, fourcc_string = "MJPG", lfile = "lidar.pcap";
	for(i=1;i<argc;++i) {
		if (strcmp(argv[i], "-d") == 0) wdir = argv[++i]; /* working directory for data */
		else if(strcmp(argv[i], "-i") == 0) ifile = argv[++i];
		else if(strcmp(argv[i], "-l") == 0) lfile = argv[++i]; /* lidar */
		else if(strcmp(argv[i], "-x") == 0) xfile = argv[++i];
		else if(strcmp(argv[i], "-s") == 0) sfile = argv[++i]; /* sensor/gps data file */
		else if(strcmp(argv[i], "-o") == 0) ofile = argv[++i];
		else if(strcmp(argv[i], "-n") == 0) frameBufferSize = atoi(argv[++i]);
		else if(strcmp(argv[i], "-skip") == 0) skip = atoi(argv[++i]);
		else if(strcmp(argv[i], "-smooth") == 0) smooth = true;
	}

    wdir = "/home/jsvirzi/projects/dataVisualization/data";
    sfile = "/home/jsvirzi/projects/mapping/data/gpsimu.root";
    /*** from sunday ***/
    wdir = "/home/jsvirzi/projects/mapping/data/26-03-2017-05-22-26";
    wdir = "/home/jsvirzi/projects/mapping/data/03-04-2017-05-53-33";
    wdir = "/home/jsvirzi/projects/mapping/data/03-04-2017-07-32-04";
    // sfile = "/home/jsvirzi/projects/mapping/data/26-03-2017-05-22-26/gpsimu.root";
    sfile = wdir + "/gpsimu.root";
    lfile = wdir + "/lidar.pcap";

    TFile fdS(sfile.c_str(), "read");

    PcapReader pcapReader(lfile.c_str());
    int lidarPacketType;
    void *p;

    int nDataPackets = 0, nPositionPackets = 0;
    uint32_t previousTimestamp = 0, timestamp = 0;
	time_t timeBase = 0;
    while((lidarPacketType = pcapReader.readPacket(&p)) != pcapReader.LidarPacketTypes) {
        if(lidarPacketType == pcapReader.TypeLidarDataPacket) {
            LidarDataPacket *lidarDataPacket = (LidarDataPacket *)p;
            LidarData *lidarData = pcapReader.pointCloud;
            convertLidarPacketToLidarData(lidarDataPacket, lidarData, timeBase, -M_PI, M_PI);
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

    getchar();

    double yaw, pitch, roll, time, time0;
    TTree *tree = (TTree *)fdS.Get("ins");
    TBranch *branchYaw = tree->GetBranch("yaw");
    branchYaw->SetAddress(&yaw);
    TBranch *branchRoll = tree->GetBranch("roll");
    branchRoll->SetAddress(&roll);
    TBranch *branchPitch = tree->GetBranch("pitch");
    branchPitch->SetAddress(&pitch);
    TBranch *branchTime = tree->GetBranch("time");
    branchTime->SetAddress(&time);
    int nAcc = 0, n = tree->GetEntries(), startEntry = 16, finalEntry = n;
    for(i=startEntry;i<finalEntry;++i) {
        branchYaw->GetEvent(i);
        branchRoll->GetEvent(i);
        branchPitch->GetEvent(i);
        branchTime->GetEvent(i);
        if(fabs(yaw) > 360.0) yaw = 0.0;
        if(fabs(roll) > 360.0) roll = 0.0;
        if(fabs(pitch) > 360.0) pitch = 0.0;
        yawData[nAcc] = yaw;
        rollData[nAcc] = roll;
        pitchData[nAcc] = pitch;
        if(i == startEntry) time0 = time;
        else evtTimeData[nAcc] = (time - time0);
        ++nAcc;
    }

	float yawMin = 999.0, yawMax = -999.0, rollMin = 999.0, rollMax = -999.0, pitchMin = 999.0, pitchMax = -999.0;

	getMinMax(yawData, nAcc, &yawMin, &yawMax);
	getMinMax(rollData, nAcc, &rollMin, &rollMax);
	getMinMax(pitchData, nAcc, &pitchMin, &pitchMax);

    printf("min/max yaw=%f/%f roll=%f/%f pitch=%f/%f", yawMin, yawMax, rollMin, rollMax, pitchMin, pitchMax);

    int nImages = 4, imageHead = 0, thumbnailWidth = 480, thumbnailHeight = 270, type = CV_8UC3;
    Size sizeThumbnail(thumbnailWidth, thumbnailHeight);
    Mat *thumbnails = new Mat [nImages];
    for(i=0;i<nImages;++i) {
        thumbnails[i] = Mat::zeros(sizeThumbnail, type);
    }

    int nGraphs = 3, graphHeight = 360, graphWidth = nImages * thumbnailWidth;
    int bigHeight = thumbnailHeight + nGraphs * graphHeight;
    Size sizeGraph(graphWidth, graphHeight);
    int bigWidth = nImages * thumbnailWidth;
    Mat bigMat = Mat::zeros(bigHeight, bigWidth, type);

    Size sizeCanvas(graphWidth, nGraphs * graphHeight);
    TCanvas canvas("canvas", "canvas", sizeCanvas.width, sizeCanvas.height);
    canvas.SetLeftMargin(0.0);
    canvas.SetRightMargin(0.0);
    canvas.SetTopMargin(0.0);
    canvas.SetBottomMargin(0.0);
    canvas.Divide(1, 3, 0.0, 0.0);
    canvas.SetBorderMode(0);

    int nGraphPoints = 200;
    float *timeData = new float [ nGraphPoints ];
    for(int j=0;j<nGraphPoints;++j) {
        timeData[j] = j;
    }

    char filename[256];
	namedWindow("main");
	for(i=0;i<3000;++i) {
		snprintf(filename, sizeof(filename), "%s/image-%010u-%1d.jpg", wdir.c_str(), i, i % 4);
        printf("filename = [%s]\n", filename);
		Mat mat = imread(filename);
        resize(mat, thumbnails[imageHead], sizeThumbnail);
        Mat tiles[nImages];
        for(int j=0;j<nImages;++j) {
            tiles[j] = Mat(bigMat, Rect(j * thumbnailWidth, 0, thumbnailWidth, thumbnailHeight));
            thumbnails[(imageHead + j) % nImages].copyTo(tiles[j]);
        }

        if(nGraphs > 0) {
            int imuPos = i * 10;
            TGraph graphYaw(nGraphPoints, timeData, &yawData[imuPos]);
            TGraph graphRoll(nGraphPoints, timeData, &rollData[imuPos]);
            TGraph graphPitch(nGraphPoints, timeData, &pitchData[imuPos]);
            prettyGraph(&graphYaw, timeData[0], timeData[nGraphPoints-1], yawMin, yawMax);
            prettyGraph(&graphRoll, timeData[0], timeData[nGraphPoints-1], rollMin, rollMax);
            prettyGraph(&graphPitch, timeData[0], timeData[nGraphPoints-1], pitchMin, pitchMax);

            TGraph *graph = &graphYaw;
            int smooth = 0;
            canvas.cd(1);
            gPad->SetLeftMargin(0.0);
            gPad->SetRightMargin(0.0);
            if(smooth) {
                graphYaw.Draw("AC");
            } else {
                graphYaw.Draw("AP");
            }
            canvas.cd(2);
            gPad->SetLeftMargin(0.0);
            gPad->SetRightMargin(0.0);
//        graph->GetXaxis()->SetLabelOffset(999);
//        graph->GetXaxis()->SetLabelSize(0);
            if(smooth) {
                graphRoll.Draw("AC");
            } else {
                graphRoll.Draw("AP");
            }
            canvas.cd(3);
            gPad->SetLeftMargin(0.0);
            gPad->SetRightMargin(0.0);
            if (smooth) {
                graphPitch.Draw("AC");
            } else {
                graphPitch.Draw("AP");
            }
            canvas.Update();
            canvas.Draw();
            canvas.SaveAs("canvas.png");
            Mat tMat, cMat = imread("canvas.png");
            Mat ctile = Mat(bigMat, Rect(0, thumbnailHeight, graphWidth, nGraphs * graphHeight));
            resize(cMat, tMat, sizeCanvas);
            tMat.copyTo(ctile);
        }

		imshow("main", bigMat);
        if(cv::waitKey(50) == 'a') {
            cv::waitKey(1000);
        }
        imageHead = (imageHead + 1) % nImages;
	}

#if 0
    int pos = (frameBufferHead + 1) % frameBufferSize;
    for (i = 0; i < frameBufferSize; ++i) {
        Mat tileA = Mat(tile, Rect(i * width, 0, width, height));
        Mat tileB = Mat(tile, Rect(i * width, height, width, height));
        internalThumbnail[pos].copyTo(tileA);
        externalThumbnail[pos].copyTo(tileB);
        pos = (pos + 1) % frameBufferSize;
    }
#endif


    return 0;

	bool single = true;
	VideoCapture icap(ifile.c_str()); /* open input stream - camera or file */
	VideoCapture xcap;
	if(!icap.isOpened()) return -1;
	if(xfile.length() > 0) {
		xcap = VideoCapture(xfile.c_str());
		if(!xcap.isOpened()) return -1;
		single = false;
	}

	double iwidth = icap.get(CV_CAP_PROP_FRAME_WIDTH);
	double iheight = icap.get(CV_CAP_PROP_FRAME_HEIGHT);
	double ifps = icap.get(CV_CAP_PROP_FPS);

	TGraph *graph;

	float twopi = 2.0 * M_PI, envelope;
	float thetaAcc = 0, thetaGyr = 0.0, thetaMag = 0.0, dthetaAcc = twopi / 2.5, dthetaGyr = twopi / 3.5, dthetaMag = twopi / 4.5;
//	for(i=0;i<nAcc;++i) {
//		tData[i] = i * 0.005; /* 5 ms. assume equally spaced data */

		// printf("X=%f/Y=%f/Z=%f/T=%ld\n", accData[0][i], accData[1][i], accData[2][i], accTime[i]);

//		 envelope = sin(i * twopi / 25.0);
//		 accData[0][i] = envelope * sin(thetaAcc + 0.0 * dthetaAcc);
//		 accData[1][i] = envelope * sin(thetaAcc + 1.0 * dthetaAcc);
//		 accData[2][i] = envelope * sin(thetaAcc + 2.0 * dthetaAcc);
//		 thetaAcc += dthetaAcc;
//		envelope = sin(i * twopi / 35.0);
//		gyrData[0][i] = envelope * sin(thetaGyr + 0.0 * dthetaGyr);
//		gyrData[1][i] = envelope * sin(thetaGyr + 1.0 * dthetaGyr);
//		gyrData[2][i] = envelope * sin(thetaGyr + 2.0 * dthetaGyr);
//		thetaGyr += dthetaGyr;
//		envelope = sin(i * twopi / 45.0);
//		magData[0][i] = envelope * sin(thetaMag + 0.0 * dthetaMag);
//		magData[1][i] = envelope * sin(thetaMag + 1.0 * dthetaMag);
//		magData[2][i] = envelope * sin(thetaMag + 2.0 * dthetaMag);
//		thetaMag += dthetaMag;
//	}

#if 0

	if(0.0 < ifps && ifps < 100.0) {
		printf("input frame rate = %.f\n", ifps);
	} else {
		ifps = 30.0;
		printf("unable to obtain input frame rate. setting manually to %.1f\n", ifps);
	}

	printf("input video parameters: WxH = %.fx%.f\n", iwidth, iheight);

	int cols = 1920, rows = 1080, type = CV_8UC3;
	int graphHeight = 120, numberOfGraphs = 3;
	int accPos = 0, gyrPos = 0, magPos = 0;
	Rect internalRoi(0, 0, cols, rows), externalRoi(cols, 0, cols, rows);
	int width = 320, height = 180;
	int tileWidth = frameBufferSize * width;
	int graphWidth = tileWidth;
	int tileHeight = 2 * height + numberOfGraphs * graphHeight;
	Size sizeThumbnail(width, height);
	Size sizeGraph(graphWidth, numberOfGraphs * graphHeight);
	Mat tile = Mat::zeros(tileHeight, tileWidth, type);
	Mat *frameBuffer = new Mat [ frameBufferSize ];
	Mat *internalMat = new Mat [ frameBufferSize ];
	Mat *externalMat = new Mat [ frameBufferSize ];
	Mat *internalThumbnail = new Mat [ frameBufferSize ];
	Mat *externalThumbnail = new Mat [ frameBufferSize ];

	VideoWriter *writer = 0;
	if(ofile.length()) {
		int fourcc = CV_FOURCC(fourcc_string[0], fourcc_string[1], fourcc_string[2], fourcc_string[3]);
		writer = new VideoWriter(ofile.c_str(), fourcc, ifps, cvSize(tileWidth, tileHeight), is_color);
		printf("output file = [%s] with FOURCC = [%s] => %4.4x\n", ofile.c_str(), fourcc_string.c_str(), fourcc);
	}

	printf("graph width = %d", graphWidth);

	TCanvas canvas("canvas", "canvas", graphWidth, 3 * graphHeight);
	canvas.SetLeftMargin(0.0);
	canvas.SetRightMargin(0.0);
	canvas.SetTopMargin(0.0);
	canvas.SetBottomMargin(0.0);
	canvas.Divide(1, 3, 0.0, 0.0);
	canvas.SetBorderMode(0);

	for(i=0;i<frameBufferSize;++i) {
		frameBuffer[i] = Mat::zeros(2 * cols, rows, type);
		internalMat[i] = Mat::zeros(cols, rows, type);
		externalMat[i] = Mat::zeros(cols, rows, type);
		internalThumbnail[i] = Mat::zeros(width, height, type);
		externalThumbnail[i] = Mat::zeros(width, height, type);
	}
	int frameBufferHead = 0, nFrames = 0;
	namedWindow("main", 1);
	for(;;) {

		if (single) {

			icap >> frameBuffer[frameBufferHead];
			Mat *frame = &frameBuffer[frameBufferHead];
			if (frame->data == NULL) {
				printf("frame data == NULL. terminating\n");
				break;
			}

			++nFrames;
			if ((skip > 0) && ((nFrames % (skip + 1)) != 0)) continue;

			internalMat[frameBufferHead] = frameBuffer[frameBufferHead](internalRoi);
			externalMat[frameBufferHead] = frameBuffer[frameBufferHead](externalRoi);
			resize(internalMat[frameBufferHead], internalThumbnail[frameBufferHead], sizeThumbnail);
			resize(externalMat[frameBufferHead], externalThumbnail[frameBufferHead], sizeThumbnail);

			int pos = (frameBufferHead + 1) % frameBufferSize;
			for (i = 0; i < frameBufferSize; ++i) {
				Mat tileA = Mat(tile, Rect(i * width, 0, width, height));
				Mat tileB = Mat(tile, Rect(i * width, height, width, height));
				internalThumbnail[pos].copyTo(tileA);
				externalThumbnail[pos].copyTo(tileB);
				pos = (pos + 1) % frameBufferSize;
			}

		} else {

			Mat *frame;

			icap >> internalMat[frameBufferHead];
			frame = &internalMat[frameBufferHead];
			if (frame->data == NULL) {
				printf("frame data == NULL. terminating\n");
				break;
			}

			xcap >> externalMat[frameBufferHead];
			frame = &externalMat[frameBufferHead];
			if (frame->data == NULL) {
				printf("frame data == NULL. terminating\n");
				break;
			}

			++nFrames;
			if ((skip > 0) && ((nFrames % (skip + 1)) != 0)) continue;

			resize(internalMat[frameBufferHead], internalThumbnail[frameBufferHead], sizeThumbnail);
			resize(externalMat[frameBufferHead], externalThumbnail[frameBufferHead], sizeThumbnail);

			int pos = (frameBufferHead + 1) % frameBufferSize;
			for (i = 0; i < frameBufferSize; ++i) {
				Mat tileA = Mat(tile, Rect(i * width, 0, width, height));
				Mat tileB = Mat(tile, Rect(i * width, height, width, height));
				internalThumbnail[pos].copyTo(tileA);
				externalThumbnail[pos].copyTo(tileB);
				pos = (pos + 1) % frameBufferSize;
			}

		}

		int nPoints = (skip + 1) * frameBufferSize * 6 + 1, lineWidth = 2.0;
		TGraph graphAccX(nPoints, &tData[accPos], &accData[0][accPos]);
		TGraph graphAccY(nPoints, &tData[accPos], &accData[1][accPos]);
		TGraph graphAccZ(nPoints, &tData[accPos], &accData[2][accPos]);
		TGraph graphGyrX(nPoints, &tData[gyrPos], &gyrData[0][gyrPos]);
		TGraph graphGyrY(nPoints, &tData[gyrPos], &gyrData[1][gyrPos]);
		TGraph graphGyrZ(nPoints, &tData[gyrPos], &gyrData[2][gyrPos]);
		TGraph graphMagX(nPoints, &tData[magPos], &magData[0][magPos]);
		TGraph graphMagY(nPoints, &tData[magPos], &magData[1][magPos]);
		TGraph graphMagZ(nPoints, &tData[magPos], &magData[2][magPos]);

		float xMin = tData[accPos], xMax = tData[accPos + nPoints - 1];
		float accMin = accXMin, gyrMin = gyrXMin, magMin = magXMin;

		if (accMin < accYMin) accMin = accYMin;
		if (gyrMin < gyrYMin) gyrMin = gyrYMin;
		if (magMin < magYMin) magMin = magYMin;

		if (accMin < accZMin) accMin = accZMin;
		if (gyrMin < gyrZMin) gyrMin = gyrZMin;
		if (magMin < magZMin) magMin = magZMin;

		float accMax = accXMax, gyrMax = gyrXMax, magMax = magXMax;

		if (accMax > accYMax) accMax = accYMax;
		if (gyrMax > gyrYMax) gyrMax = gyrYMax;
		if (magMax > magYMax) magMax = magYMax;

		if (accMax > accZMax) accMax = accZMax;
		if (gyrMax > gyrZMax) gyrMax = gyrZMax;
		if (magMax > magZMax) magMax = magZMax;

		int markerStyle = 1;

		graph = &graphAccX;
		graph->GetXaxis()->SetLabelOffset(999);
		graph->GetXaxis()->SetLabelSize(0);
		graph->GetYaxis()->SetLabelOffset(999);
		graph->GetYaxis()->SetLabelSize(0);
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->GetYaxis()->SetNdivisions(0, 0);
		graph->SetMinimum(accMin);
		graph->SetMaximum(accMax);
		graph->SetLineColor(kBlue);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphAccY;
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->SetLineColor(kGreen);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphAccZ;
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->SetLineColor(kRed);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphGyrX;
		graph->GetXaxis()->SetLabelOffset(999);
		graph->GetXaxis()->SetLabelSize(0);
		graph->GetYaxis()->SetLabelOffset(999);
		graph->GetYaxis()->SetLabelSize(0);
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->GetYaxis()->SetNdivisions(0, 0);
		graph->SetMinimum(gyrMin);
		graph->SetMaximum(gyrMax);
		graph->SetLineColor(kBlue);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphGyrY;
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->SetLineColor(kGreen);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphGyrZ;
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->SetLineColor(kRed);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphMagX;
		graph->GetXaxis()->SetLabelOffset(999);
		graph->GetXaxis()->SetLabelSize(0);
		graph->GetYaxis()->SetLabelOffset(999);
		graph->GetYaxis()->SetLabelSize(0);
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->GetYaxis()->SetNdivisions(0, 0);
		graph->SetMinimum(magMin);
		graph->SetMaximum(magMax);
		graph->SetLineColor(kBlue);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphMagY;
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->SetLineColor(kGreen);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		graph = &graphMagZ;
		graph->GetXaxis()->SetRangeUser(xMin, xMax);
		graph->SetLineColor(kRed);
		graph->SetLineWidth(lineWidth);
		graph->SetMarkerStyle(markerStyle);

		accPos += 6 * skip;
		gyrPos += 6 * skip;
		magPos += 6 * skip;
		canvas.cd(1);
		gPad->SetLeftMargin(0.0);
		gPad->SetRightMargin(0.0);
		if(smooth) {
			graphAccX.Draw("AC");
			graphAccY.Draw("C");
			graphAccZ.Draw("C");
		} else {
			graphAccX.Draw("AP");
			graphAccY.Draw("LP");
			graphAccZ.Draw("LP");
		}
		canvas.cd(2);
		gPad->SetLeftMargin(0.0);
		gPad->SetRightMargin(0.0);
		graph->GetXaxis()->SetLabelOffset(999);
		graph->GetXaxis()->SetLabelSize(0);
		if(smooth) {
			graphGyrX.Draw("AC");
			graphGyrY.Draw("C");
			graphGyrZ.Draw("C");
		} else {
			graphGyrX.Draw("AP");
			graphGyrY.Draw("LP");
			graphGyrZ.Draw("LP");
		}
		canvas.cd(3);
		gPad->SetLeftMargin(0.0);
		gPad->SetRightMargin(0.0);
		if (smooth) {
			graphMagX.Draw("AC");
			graphMagY.Draw("C");
			graphMagZ.Draw("C");
		} else {
			graphMagX.Draw("AP");
			graphMagY.Draw("LP");
			graphMagZ.Draw("LP");
		}
		canvas.Draw();
		canvas.Update();
		canvas.SaveAs("canvas.png");
		Mat tMat, cMat = imread("canvas.png");
		Mat ctile = Mat(tile, Rect(0, 2 * height, graphWidth, numberOfGraphs * graphHeight));
		resize(cMat, tMat, sizeGraph);
		tMat.copyTo(ctile);

		frameBufferHead = (frameBufferHead + 1) % frameBufferSize;

		if(writer) writer->write(tile);

		imshow("main", tile);
		int ch = waitKey(15);
		if(ch == ESC) { break; }
		else if(ch == 'p') { ch = waitKey(0); }
	}

	icap.release();
	if (!single) {
		xcap.release();
	}
	if(writer) delete writer;
	return 0;

#endif
}


