// CameraCallib.cpp : Defines the entry point for the console application.
//

#define DEBUG true

#include "stdafx.h"
#include <iostream> 
#include <fstream>
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
bool getChessboardCorners(Mat inputImage, Size chessboardDimension, vector<Point2f>& foundCorners, Mat& outputImage)
{
	bool found = findChessboardCorners(inputImage, chessboardDimension, foundCorners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

	// Render found corners to output image
	inputImage.copyTo(outputImage);
	drawChessboardCorners(outputImage, chessboardDimension, foundCorners, found);

	return found;
}

// Camera calibration
void calibrateCamera(vector<Mat> inputImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distortionCoeff, Mat& rVecs, Mat& tVecs) 
{
	// Get corners points from captured images
	vector<vector<Point2f>> chessboardCorners;
	for (int i = 0; i < inputImages.size(); i++) 
	{
		vector<Point2f> pointBuff;
		findChessboardCorners(inputImages[i], chessboardDimension, pointBuff, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		chessboardCorners.push_back(pointBuff);
	}

	// Remap corner points in world plane
	vector<vector<Point3f>> chessboardCornersInWorldSpace(1);
	calculateKnownBoardPosition(boardSize, squareEdgeLength, chessboardCornersInWorldSpace[0]);
	chessboardCornersInWorldSpace.resize(chessboardCorners.size(), chessboardCornersInWorldSpace[0]);

	// Calibrate camera
	calibrateCamera(chessboardCornersInWorldSpace, chessboardCorners, boardSize, cameraMatrix, distortionCoeff, rVecs, tVecs);
}

// Write calibration result to disk
bool writeCalibrationResult(string filename, Mat cameraMatrix, Mat distortionCoeff, Mat rVecs, Mat tVecs) 
{
	ofstream outInStream(filename + "_intrinsic");
	ofstream outExStream(filename + "_extrinsic");

	if (outInStream)
	{
		// Write camera matrix
		for (int r = 0; r < cameraMatrix.rows; r++)
		{
			for (int c = 0; c < cameraMatrix.cols; c++)
			{
				outInStream << cameraMatrix.at<double>(r, c) << endl;
			}
		}

		// Write distortion coeff
		for (int r = 0; r < distortionCoeff.rows; r++)
		{
			for (int c = 0; c < distortionCoeff.cols; c++)
			{
				outInStream << distortionCoeff.at<double>(r, c) << endl;
			}
		}

		outInStream.close();
	}
	else
		return false;

	if (outExStream)
	{
		// Write rvecs
		for (int r = 0; r < rVecs.rows; r++)
		{
			for (int c = 0; c < rVecs.cols; c++)
			{
				outExStream << rVecs.at<double>(r, c) << endl;
			}
		}

		// Write tvecs
		for (int r = 0; r < tVecs.rows; r++)
		{
			for (int c = 0; c < tVecs.cols; c++)
			{
				outExStream << tVecs.at<double>(r, c) << endl;
			}
		}

		outExStream.close();
	}
	else
		return false;

	return true;
}

int main(int argc, char** argv)
{
	Mat frame, drawToFrame;
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distortionCoeff = Mat::zeros(8, 1, CV_64F);
	Mat tVecs, rVecs;
	vector<Mat> capturedFrames;
	vector<vector<Point2f>> markerCorners, rejectedCandidates;
	VideoCapture vid(0);

	if (!vid.isOpened())
		return -1;

	int fps = 20;

	namedWindow("OutputVideo", CV_WINDOW_AUTOSIZE);

	cout << "Program start ...";
	vector<Point2f> foundCorners;
	char holdKey;
	bool chessboardFound;
	while (true)
	{
		if (!vid.read(frame))
			break;

		// Clear vector
		foundCorners.clear();
		chessboardFound = getChessboardCorners(frame, chessboardDimension, foundCorners, drawToFrame);

		// Stream to window
		imshow("OutputVideo", drawToFrame);

		holdKey = waitKey(1000 / fps);

		// Key actions
		switch (holdKey)
		{
		case ' ':
			//capture frame
			if (chessboardFound)
				capturedFrames.push_back(frame.clone());
			else
				cout << "\nChessboard pattern not found!";
			break;
		case 13: // enter key
			//start calibration
			if (capturedFrames.size() > 15)
			{
				calibrateCamera(capturedFrames, chessboardDimension, calibrationSquareDimension, cameraMatrix, distortionCoeff, rVecs, tVecs);
				writeCalibrationResult("calibration-result.txt", cameraMatrix, distortionCoeff, rVecs, tVecs);
			}
			else
				cout << "\nNeed more frames! Captured frames : " << capturedFrames.size();
			break;
		case 27: // esc key
			//exit program
			return 0;
			break;
		}
	}


    return 0;
}

