// CameraCallib.cpp : Defines the entry point for the console application.
//

#define DEBUG true

#include "stdafx.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"	
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.01905f; //meters
const float arucoSquareDimension = 0.1016f; //meters
const Size chessboardDimension = Size(6,9);

// Calculate real all checkboard corners position
void calculateKnownBoardPosition(Size boardSize, float boardEdgeLength, vector<Point3f>& corners) 
{
	for (int y = 0; y < boardSize.height; y++) 
	{
		for (int x = 0; x < boardSize.width; x++) 
		{
			corners.push_back(Point3f(x * boardEdgeLength, y * boardEdgeLength, 0.0f));
		}
	}
}

// Process input image, and find all corners in the image
void getChessboardCorners(Mat inputImage, Size chessboardDimension, vector<Vec2f>& foundCorners, Mat& outputImage)
{
	// Find corners
	bool found = findChessboardCorners(inputImage, chessboardDimension, foundCorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

	// Render found corners to output image
	inputImage.copyTo(outputImage);
	drawChessboardCorners(outputImage, chessboardDimension, foundCorners, found);
}

int main()
{
	Mat frame, drawToFrame;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoeff;
	vector<Mat> savedImages;
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	VideoCapture vid(0);

	if (!vid.isOpened())
		return -1;

	int fps = 20;

	namedWindow("OutputVideo", CV_WINDOW_AUTOSIZE);

	vector<Vec2f> foundCorners;
	char holdKey;
	while (true)
	{
		if (!vid.read(frame))
			break;

		// Clear vector
		foundCorners.clear();
		getChessboardCorners(frame, chessboardDimension, foundCorners, drawToFrame);

		// Stream to window
		imshow("OutputVideo", drawToFrame);

		holdKey = waitKey(1000 / fps);
	}


    return 0;
}

