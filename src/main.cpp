#include "opencv2/opencv.hpp"

#include <TCanvas.h>
#include <TGraph.h>
#include <TAxis.h>

#include <iostream>
#include <stdio.h>

#include <math.h>

using namespace cv;
using namespace std;

#define ESC 27

bool readFieldFromCsv(const char *line, int field, char *res, int max) {
	int i, pos = 0, len = strlen(line), nCommas = 0, beginPos = 0, endPos = -1;
	for(pos=0;pos<=len;++pos) {
		char ch = line[pos];
		if((ch == ',') || (ch == 0)) {
			if(nCommas == field) {
				endPos = pos;
			} else {
				beginPos = pos + 1;
			}

			if(endPos >= 0) {
				int tLen = endPos - beginPos;
				if(tLen < max) {
					for(i=0;i<tLen;++i) {
						res[i] = line[beginPos + i];
					}
					res[i] = 0;
					return true;
				} else {
					return false;
				}
			}
			++nCommas;
		}
	}
	return false;
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
float accData[3][MaxImu], gyrData[3][MaxImu], magData[3][MaxImu], tData[MaxImu];
long accTime[MaxImu], gyrTime[MaxImu], magTime[MaxImu];

int main(int argc, char **argv) {

	int i, frameBufferSize = 8, skip = 0;
	bool is_color = true, smooth = false;
	std::string ifile, xfile, ofile, sfile, fourcc_string = "MJPG";
	for(i=1;i<argc;++i) {
		if(strcmp(argv[i], "-i") == 0) ifile = argv[++i];
		else if(strcmp(argv[i], "-x") == 0) xfile = argv[++i];
		else if(strcmp(argv[i], "-s") == 0) sfile = argv[++i];
		else if(strcmp(argv[i], "-o") == 0) ofile = argv[++i];
		else if(strcmp(argv[i], "-n") == 0) frameBufferSize = atoi(argv[++i]);
		else if(strcmp(argv[i], "-skip") == 0) skip = atoi(argv[++i]);
		else if(strcmp(argv[i], "-smooth") == 0) smooth = true;
	}

	int nAcc = readFloatArrayFromCsv(sfile.c_str(), AccId, accData[0], accData[1], accData[2], accTime, MaxImu);
	int nGyr = readFloatArrayFromCsv(sfile.c_str(), GyrId, gyrData[0], gyrData[1], gyrData[2], gyrTime, MaxImu);
	int nMag = readFloatArrayFromCsv(sfile.c_str(), MagId, magData[0], magData[1], magData[2], magTime, MaxImu);

	float accXMin = 999.0, accXMax = -999.0, accYMin = 999.0, accYMax = -999.0, accZMin = 999.0, accZMax = -999.0;
	float gyrXMin = 999.0, gyrXMax = -999.0, gyrYMin = 999.0, gyrYMax = -999.0, gyrZMin = 999.0, gyrZMax = -999.0;
	float magXMin = 999.0, magXMax = -999.0, magYMin = 999.0, magYMax = -999.0, magZMin = 999.0, magZMax = -999.0;

	getMinMax(accData[0], nAcc, &accXMin, &accXMax);
	getMinMax(accData[1], nAcc, &accYMin, &accYMax);
	getMinMax(accData[2], nAcc, &accZMin, &accZMax);

	getMinMax(gyrData[0], nGyr, &gyrXMin, &gyrXMax);
	getMinMax(gyrData[1], nGyr, &gyrYMin, &gyrYMax);
	getMinMax(gyrData[2], nGyr, &gyrZMin, &gyrZMax);

	getMinMax(magData[0], nMag, &magXMin, &magXMax);
	getMinMax(magData[1], nMag, &magYMin, &magYMax);
	getMinMax(magData[2], nMag, &magZMin, &magZMax);

//	for(i=0;i<nAcc;++i) {
//		if(accData[0][i] > accXMax) { accXMax = accData[0][i]; }
//		if(accData[0][i] < accXMin) { accXMin = accData[0][i]; }
//		if(accData[1][i] > accYMax) { accYMax = accData[1][i]; }
//		if(accData[1][i] < accYMin) { accYMin = accData[1][i]; }
//		if(accData[2][i] > accZMax) { accZMax = accData[2][i]; }
//		if(accData[2][i] < accZMin) { accZMin = accData[2][i]; }
//	}

	printf("%d %d %d elements read from ACC/GYR/MAG\n", nAcc, nGyr, nMag);
	printf("ACC min/max = %f/%f %f/%f %f/%f\n", accXMin, accXMax, accYMin, accYMax, accZMin, accZMax);
	printf("GYR min/max = %f/%f %f/%f %f/%f\n", gyrXMin, gyrXMax, gyrYMin, gyrYMax, gyrZMin, gyrZMax);
	printf("MAG min/max = %f/%f %f/%f %f/%f\n", magXMin, magXMax, magYMin, magYMax, magZMin, magZMax);

	bool single = true;
	// VideoCapture *cap;
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
	for(i=0;i<nAcc;++i) {
		tData[i] = i * 0.005; /* 5 ms. assume equally spaced data */

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
	}

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
}


