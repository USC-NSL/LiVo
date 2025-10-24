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

// #include <fstream>
#include <boost/filesystem/fstream.hpp>
#include "calibration.h"
#include "opencv/cv.h"
#include "consts.h"

using namespace std;
namespace fs = boost::filesystem;

Calibration::Calibration(int nSamples)
{
	bCalibrated = false;
	iUsedMarkerId = -1;
	nSampleCounter = 0;
	nRequiredSamples = nSamples;

	worldT = vector<float>(3, 0.0f);
	for (int i = 0; i < 3; i++)
	{
		worldR.push_back(vector<float>(3, 0.0f));
		worldR[i][i] = 1.0f;
	}

	nMarkers = 2; // Rajrup To Do: Now only supports 2 marker
	markerPoses.resize(nMarkers);

	markerPoses[0].SetOrientation(0.0f, 0.0f, 0.0f);
	markerPoses[0].setTranslation(0.0f, 0.0f, 0.0f);
	markerPoses[0].setMarkerId(0);

	markerPoses[1].SetOrientation(0.0f, 180.0f, 0.0f);
	markerPoses[1].setTranslation(0.0f, 0.0f, -0.2f);
	markerPoses[1].setMarkerId(1);

	pDetector = new MarkerDetector();
}

Calibration::Calibration(const Calibration &calibration)
{
	for(int i = 0; i < 3; i++)
	{
		worldT[i] = calibration.worldT[i];
		for(int j = 0; j < 3; j++)
		{
			worldR[i][j] = calibration.worldR[i][j];
		}
	}

	intrinsics = calibration.intrinsics;

	bCalibrated = calibration.bCalibrated;
	iUsedMarkerId = calibration.iUsedMarkerId;
}

Calibration::~Calibration()
{
	if (pDetector != NULL)
	{
		delete pDetector;
		pDetector = NULL;
	}
}

void Calibration::setIntrinsics(float fx, float fy, float cx, float cy, int width, int height)
{
	intrinsics.fx = fx;
	intrinsics.fy = fy;
	intrinsics.cx = cx;
	intrinsics.cy = cy;
	intrinsics.width = width;
	intrinsics.height = height;
}

bool Calibration::calibrate(RGB *pBuffer, Point3f *pCameraCoordinates, int cColorWidth, int cColorHeight)
{
	MarkerInfo marker;

	bool res = pDetector->GetMarker(pBuffer, cColorHeight, cColorWidth, marker);
	if (!res)
		return false;

	int indexInPoses = -1;

	for (unsigned int j = 0; j < markerPoses.size(); j++)
	{
		if (marker.id == markerPoses[j].markerId)
		{
			indexInPoses = j;
			break;
		}
	}
	if (indexInPoses == -1)
		return false;

	MarkerPose2 markerPose = markerPoses[indexInPoses];
	iUsedMarkerId = markerPose.markerId;

	vector<Point3f> marker3D(marker.corners.size());
	bool success = getMarkerCorners3D(marker3D, marker, pCameraCoordinates, cColorWidth, cColorHeight);

	if (!success)
	{
		return false;
	}

	marker3DSamples.push_back(marker3D);
	nSampleCounter++;

	LOG(INFO) << "Marker: " << iUsedMarkerId << " nSamples: " << nSampleCounter;

	if (nSampleCounter < nRequiredSamples)
		return false;

	for (size_t i = 0; i < marker3D.size(); i++)
	{
		marker3D[i] = Point3f();
		for (int j = 0; j < nRequiredSamples; j++)
		{
			marker3D[i].X += marker3DSamples[j][i].X / (float)nRequiredSamples;
			marker3D[i].Y += marker3DSamples[j][i].Y / (float)nRequiredSamples;
			marker3D[i].Z += marker3DSamples[j][i].Z / (float)nRequiredSamples;
		}
	}

	procrustes(marker, marker3D, worldT, worldR);

	vector<vector<float>> Rcopy = worldR;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			worldR[i][j] = 0;

			for (int k = 0; k < 3; k++)
			{
				worldR[i][j] += markerPose.R[i][k] * Rcopy[k][j];
			}
		}
	}

	vector<float> translationIncr(3);
	translationIncr[0] = markerPose.t[0];
	translationIncr[1] = markerPose.t[1];
	translationIncr[2] = markerPose.t[2];;

	translationIncr = InverseRotatePoint(translationIncr, worldR);

	worldT[0] += translationIncr[0];
	worldT[1] += translationIncr[1];
	worldT[2] += translationIncr[2];

	bCalibrated = true;

	marker3DSamples.clear();
	nSampleCounter = 0;

	return true;
}

/**
 * @brief Load calibration from file
 * 
 * @param serialNumber Device serial number
 * @param path Folder to load calibration file
 * @param saveIntrinsics If true, loads intrinsics as well 
 */
bool Calibration::loadCalibration(const string &serialNumber, const string &path, bool loadIntrinsics)
{
	fs::path filepath{FORMAT(path << CALIBRATION_EXT_FILENAME << "_" << serialNumber << ".txt")};
	fs::ifstream file;
	file.open(filepath, ios::in);
	if (!file.is_open())
	{
		LOG(ERROR) << "Failed to load Calibration file: " << filepath;
		return false;
	}
		
	for (int i = 0; i < 3; i++)
		file >> worldT[i];
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			file >> worldR[i][j];
	}

	file >> iUsedMarkerId;
	file >> bCalibrated;

	file.close();
	// LOG(INFO) << "Extrinsic Calibration for device " << serialNumber << " loaded from " << filepath;

	if(loadIntrinsics)
	{
		fs::path filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_" << serialNumber << ".txt")};
		fs::ifstream file;
		file.open(filepath, ios::in);
		if (!file.is_open())
		{
			LOG(ERROR) << "Failed to load Calibration file: " << filepath;
			return false;
		}

		uint32_t calib_param;
		float calib_value;
		while(!file.eof())
		{
			file >> calib_param >> calib_value;
			switch(calib_param)
			{
				case RW:
					intrinsics.width = int(calib_value);
					break;
				case RH:
					intrinsics.height = int(calib_value);
					break;
				case FX:
					intrinsics.fx = calib_value;
					break;
				case FY:
					intrinsics.fy = calib_value;
					break;
				case CX:
					intrinsics.cx = calib_value;
					break;
				case CY:
					intrinsics.cy = calib_value;
					break;
				default:
					LOG(ERROR) << "Unknown calibration parameter: " << calib_param;
					return false;
			}
		}

		file.close();
		// LOG(INFO) << "Intrinsic Calibration for device " << serialNumber << " loaded from " << filepath;
	}

	return true;
}

/**
 * @brief Saves calibration to file
 * 
 * @param serialNumber Device serial number
 * @param path Folder to save calibration file
 * @param saveIntrinsics If true, saves intrinsics as well
 */
bool Calibration::saveCalibration(const string &serialNumber, const string &path, bool saveIntrinsics)
{
	fs::path filepath{FORMAT(path << CALIBRATION_EXT_FILENAME << "_" << serialNumber << ".txt")};
	fs::ofstream file;
	file.open(filepath, ios::out | ios::trunc);

	if (!file.is_open())
	{	
		LOG(ERROR) << "Failed to open file: " << filepath;
		return false;
	}

	for (int i = 0; i < 3; i++)
		file << worldT[i] << " ";
	file << endl;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			file << worldR[i][j] << " ";
		file << endl;
	}
	file << iUsedMarkerId << endl;
	file << bCalibrated << endl;

	file.close();
	LOG(INFO) << "Extrinsic Calibration for device " << serialNumber << " saved to " << filepath;

	if(saveIntrinsics)
	{
		fs::path filepath{FORMAT(path << CALIBRATION_INT_FILENAME << "_" << serialNumber << ".txt")};
		fs::ofstream file;
		file.open(filepath, ios::out | ios::trunc);

		if (!file.is_open())
		{	
			LOG(ERROR) << "Failed to open file: " << filepath;
			return false;
		}

		file << RW << " " << intrinsics.width << endl;
		file << RH << " " << intrinsics.height << endl;
		file << CX << " " << intrinsics.cx << endl;
		file << CY << " " << intrinsics.cy << endl;
		file << FX << " " << intrinsics.fx << endl;
		file << FY << " " << intrinsics.fy << endl;

		file.close();
		LOG(INFO) << "Intrinsic Calibration for device " << serialNumber << " saved to " << filepath;
	}

	return true;
}

void Calibration::procrustes(MarkerInfo &marker, vector<Point3f> &markerInWorld, vector<float> &worldToMarkerT, vector<vector<float>> &worldToMarkerR)
{
	int nVertices = marker.points.size();

	Point3f markerCenterInWorld;
	Point3f markerCenter;
	for (int i = 0; i < nVertices; i++)
	{
		markerCenterInWorld.X += markerInWorld[i].X / nVertices;
		markerCenterInWorld.Y += markerInWorld[i].Y / nVertices;
		markerCenterInWorld.Z += markerInWorld[i].Z / nVertices;

		markerCenter.X += marker.points[i].X / nVertices;
		markerCenter.Y += marker.points[i].Y / nVertices;
		markerCenter.Z += marker.points[i].Z / nVertices;
	}

	worldToMarkerT.resize(3);
	worldToMarkerT[0] = -markerCenterInWorld.X;
	worldToMarkerT[1] = -markerCenterInWorld.Y;
	worldToMarkerT[2] = -markerCenterInWorld.Z;

	vector<Point3f> markerInWorldTranslated(nVertices);
	vector<Point3f> markerTranslated(nVertices);
	for (int i = 0; i < nVertices; i++)
	{
		markerInWorldTranslated[i].X = markerInWorld[i].X + worldToMarkerT[0];
		markerInWorldTranslated[i].Y = markerInWorld[i].Y + worldToMarkerT[1];
		markerInWorldTranslated[i].Z = markerInWorld[i].Z + worldToMarkerT[2];

		markerTranslated[i].X = marker.points[i].X - markerCenter.X;
		markerTranslated[i].Y = marker.points[i].Y - markerCenter.Y;
		markerTranslated[i].Z = marker.points[i].Z - markerCenter.Z;
	}

	cv::Mat A(nVertices, 3, CV_64F);
	cv::Mat B(nVertices, 3, CV_64F);

	for (int i = 0; i < nVertices; i++)
	{
		A.at<double>(i, 0) = markerTranslated[i].X;
		A.at<double>(i, 1) = markerTranslated[i].Y;
		A.at<double>(i, 2) = markerTranslated[i].Z;

		B.at<double>(i, 0) = markerInWorldTranslated[i].X;
		B.at<double>(i, 1) = markerInWorldTranslated[i].Y;
		B.at<double>(i, 2) = markerInWorldTranslated[i].Z;
	}

	cv::Mat M = A.t() * B;

	cv::SVD svd;
	svd(M);
	cv::Mat R = svd.u * svd.vt;

	double det = cv::determinant(R);

	if (det < 0)
	{
		cv::Mat temp = cv::Mat::eye(3, 3, CV_64F);
		temp.at<double>(2, 2) = -1;
		R = svd.u * temp * svd.vt;
	}

	worldToMarkerR.resize(3);

	for (int i = 0; i < 3; i++)
	{
		worldToMarkerR[i].resize(3);
		for (int j = 0; j < 3; j++)
		{
			worldToMarkerR[i][j] = static_cast<float>(R.at<double>(i, j));
		}
	}
}

bool Calibration::getMarkerCorners3D(vector<Point3f> &marker3D, MarkerInfo &marker, Point3f *pCameraCoordinates, int cColorWidth, int cColorHeight)
{
	for (unsigned int i = 0; i < marker.corners.size(); i++)
	{
		int minX = static_cast<int>(marker.corners[i].X);
		int maxX = minX + 1;
		int minY = static_cast<int>(marker.corners[i].Y);
		int maxY = minY + 1;

		float dx = marker.corners[i].X - minX;
		float dy = marker.corners[i].Y - minY;

		Point3f pointMin = pCameraCoordinates[minX + minY * cColorWidth];
		Point3f pointXMaxYMin = pCameraCoordinates[maxX + minY * cColorWidth];
		Point3f pointXMinYMax = pCameraCoordinates[minX + maxY * cColorWidth];
		Point3f pointMax = pCameraCoordinates[maxX + maxY * cColorWidth];

		if (pointMin.Z <= 0 || pointXMaxYMin.Z <= 0 || pointXMinYMax.Z <= 0 || pointMax.Z <= 0)
			return false;

		marker3D[i].X = (1 - dx) * (1 - dy) * pointMin.X + dx * (1 - dy) * pointXMaxYMin.X + (1 - dx) * dy * pointXMinYMax.X + dx * dy * pointMax.X;
		marker3D[i].Y = (1 - dx) * (1 - dy) * pointMin.Y + dx * (1 - dy) * pointXMaxYMin.Y + (1 - dx) * dy * pointXMinYMax.Y + dx * dy * pointMax.Y;
		marker3D[i].Z = (1 - dx) * (1 - dy) * pointMin.Z + dx * (1 - dy) * pointXMaxYMin.Z + (1 - dx) * dy * pointXMinYMax.Z + dx * dy * pointMax.Z;
	}

	return true;
}

/**
 * @brief RAJRUP
 * TODO: Change these operations to matrix multiplication using Eigen library.
 */
void Calibration::panoptic2point(const uint16_t &depth, const int &vertex_id, Point3f &point)
{
	float x = (float)(vertex_id % intrinsics.width);
	float y = (float)(vertex_id / intrinsics.width);
	float z = (float)depth;

	Point3f temp;
	temp.X = ((intrinsics.cx - x) * z / intrinsics.fx) / 1000.0F;    // mm to m
	temp.Y = ((intrinsics.cy - y) * z / intrinsics.fy) / 1000.0F;    // mm to m
	temp.Z = z / 1000.0F;                      						 // mm to m

	// TODO: Rajrup: Remove 1.0F from here, when you are sure everything works fine.
	point.X = worldR[0][0] * temp.X + worldR[0][1] * temp.Y + worldR[0][2] * temp.Z + worldT[0];
	point.Y = worldR[1][0] * temp.X + worldR[1][1] * temp.Y + worldR[1][2] * temp.Z + worldT[1]; 	// No need to invert Y axis, since it will be taken care by the frustum inversion.
	point.Z = -(worldR[2][0] * temp.X + worldR[2][1] * temp.Y + worldR[2][2] * temp.Z + worldT[2]); 	// Open3D uses right handed coordinate system. So, Z is negative.
}

/**
 * @brief RAJRUP
 * TODO: Change these operations to matrix multiplication using Eigen library.
 */
void Calibration::kinect2point(const xy_table_t *xy_table, const uint16_t &depth, const int &vertex_id, Point3f &point)
{
	// First translate then rotate (in m)
	Point3f temp;
	temp.X = xy_table->x_table[vertex_id] * (float)depth/1000.0f + worldT[0];
	temp.Y = xy_table->y_table[vertex_id] * (float)depth/1000.0f + worldT[1];
	temp.Z = (float)depth/1000.0f + worldT[2];

	point.X = temp.X * worldR[0][0] + temp.Y * worldR[0][1] + temp.Z * worldR[0][2];
	point.Y = temp.X * worldR[1][0] + temp.Y * worldR[1][1] + temp.Z * worldR[1][2];
	point.Z = temp.X * worldR[2][0] + temp.Y * worldR[2][1] + temp.Z * worldR[2][2];
}

vector<float> InverseRotatePoint(vector<float> &point, std::vector<std::vector<float>> &R)
{
	vector<float> res(3);

	res[0] = point[0] * R[0][0] + point[1] * R[1][0] + point[2] * R[2][0];
	res[1] = point[0] * R[0][1] + point[1] * R[1][1] + point[2] * R[2][1];
	res[2] = point[0] * R[0][2] + point[1] * R[1][2] + point[2] * R[2][2];

	return res;
}

vector<float> RotatePoint(vector<float> &point, std::vector<std::vector<float>> &R)
{
	vector<float> res(3);

	res[0] = point[0] * R[0][0] + point[1] * R[0][1] + point[2] * R[0][2];
	res[1] = point[0] * R[1][0] + point[1] * R[1][1] + point[2] * R[1][2];
	res[2] = point[0] * R[2][0] + point[1] * R[2][1] + point[2] * R[2][2];

	return res;
}
