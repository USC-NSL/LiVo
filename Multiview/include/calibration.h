//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#pragma once
// #include "stdafx.h"
#include "marker.h"
#include "utils.h"
#include <cmath>

#define _USE_MATH_DEFINES

vector<float> RotatePoint(vector<float> &point, std::vector<std::vector<float>> &R);
vector<float> InverseRotatePoint(vector<float> &point, std::vector<std::vector<float>> &R);
struct MarkerPose
{
	int markerId;
	float R[3][3];
	float t[3];
};

class MarkerPose2
{

public:
	int markerId;
	float R[3][3];
	float t[3];
	

	MarkerPose2()
	{
		markerId = 0;
		r[0] = 0.0;
		r[1] = 0.0;
		r[2] = 0.0;
		AffineTransform();
		UpdateRotationMatrix();
	}

	void SetOrientation(float X, float Y, float Z)
	{
		r[0] = X;
		r[1] = Y;
		r[2] = Z;

		UpdateRotationMatrix();
	}

	void setTranslation(float X, float Y, float Z)
	{
		t[0] = X;
		t[1] = Y;
		t[2] = Z;
	}

	void setMarkerId(int Id)
	{
		markerId = Id;
	}

private:
	float r[3];

	void AffineTransform()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (i == j)
					R[i][j] = 1;
				else
					R[i][j] = 0;
			}
			t[i] = 0;
		}
	}

	void UpdateRotationMatrix()
	{
		float radX = r[0] * (float)M_PI / 180.0f;
		float radY = r[1] * (float)M_PI / 180.0f;
		float radZ = r[2] * (float)M_PI / 180.0f;

		float c1 = cos(radZ);
		float c2 = cos(radY);
		float c3 = cos(radX);
		float s1 = sin(radZ);
		float s2 = sin(radY);
		float s3 = sin(radX);

		//Z Y X rotation
		R[0][0] = c1 * c2;
		R[0][1] = c1 * s2 * s3 - c3 * s1;
		R[0][2] = s1 * s3 + c1 * c3 * s2;
		R[1][0] = c2 * s1;
		R[1][1] = c1 * c3 + s1 * s2 * s3;
		R[1][2] = c3 * s1 * s2 - c1 * s3;
		R[2][0] = -s2;
		R[2][1] = c2 * s3;
		R[2][2] = c2 * c3;
	}
};

typedef struct instrinsic
{
	float fx;	// focal length x
	float fy;	// focal length y
	float cx;	// camera optical center x
	float cy;	// camera optical center y
	int width;
	int height;

	instrinsic()
	{
		fx = 0;
		fy = 0;
		cx = 0;
		cy = 0;
		width = 0;
		height = 0;
	}
} intrinsic_t;

class Calibration
{
public:

	intrinsic_t intrinsics;
	vector<float> worldT;
	vector<vector<float>> worldR;
	int iUsedMarkerId;

	vector<MarkerPose2> markerPoses;
	int nMarkers;

	bool bCalibrated;

	Calibration(int nSamples=30);
	Calibration(const Calibration &calibration);
	~Calibration();

	bool calibrate(RGB *pBuffer, Point3f *pCameraCoordinates, int cColorWidth, int cColorHeight);
	bool loadCalibration(const string &serialNumber, const string &path, bool loadInstrinsics = false);
	bool saveCalibration(const string &serialNumber, const string &path, bool saveInstrinsics = false);
	void setIntrinsics(float fx, float fy, float cx, float cy, int width, int height);
	void panoptic2point(const uint16_t &depth, const int &vertex_id, Point3f &point);
	void kinect2point(const xy_table_t *xy_table, const uint16_t &depth, const int &vertex_id, Point3f &point);
private:
	IMarker *pDetector;
	int nSampleCounter;
	int nRequiredSamples;

	vector<vector<Point3f>> marker3DSamples;

	void procrustes(MarkerInfo &marker, vector<Point3f> &markerInWorld, vector<float> &markerT, vector<vector<float>> &markerR);
	bool getMarkerCorners3D(vector<Point3f> &marker3D, MarkerInfo &marker, Point3f *pCameraCoordinates, int cColorWidth, int cColorHeight);
};

