#include "frustum.h"
#include "consts.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>

namespace fs = boost::filesystem;

using namespace std;

const string getPlane2String(int enumPlane)
{
	return Plane2String[enumPlane];
}

Plane::Plane(const Vector3f &v1, const Vector3f &v2, const Vector3f &v3) 
{
	set3Points(v1, v2, v3);
}

Plane::Plane(const Plane &plane)
{
	this->normal = plane.normal;
	this->point = plane.point;
	this->d = plane.d;
}

Plane::Plane() {}

Plane::~Plane() {}

void Plane::set3Points(const Vector3f &v1, const Vector3f &v2, const Vector3f &v3)
{
	Vector3f aux1, aux2;

	aux1 = v1 - v2;
	aux2 = v3 - v2;

	normal = aux2.cross(aux1);

	// normal.normalize();
	point = v2;
	d = -(normal.dot(point));
	normalize();
}

void Plane::set3Points2(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3)
{
	Vector3f v1 = p2 - p1;
	Vector3f v2 = p3 - p1;

	float a1 = v1(0);
	float b1 = v1(1);
	float c1 = v1(2);

	float a2 = v2(0);
	float b2 = v2(1);
	float c2 = v2(2);

	normal = v1.cross(v2);
	normal.normalize();

	point = p1;
	float a = (b1 * c2) - (b2 * c1);
	float b = (a2 * c1) - (a1 * c2);
	float c = (a1 * b2) - (b1 * a2);
	d = ((-1.0 * a * p1(0)) - (b * p1(1)) - (c * p1(2)));
}

void Plane::setNormalAndPoint(const Vector3f &normal, const Vector3f &point) 
{
	this->normal = normal;
	this->normal.normalize();
	d = -(this->normal.dot(point));
}

/**
 * @brief Equation is of the form ax + by+ xz + d = 0. Check the sign of d when using this function.
 */
void Plane::setCoefficients(float a, float b, float c, float d)
{
	// set the normal vector
	normal << a, b, c;

	//compute the length of the vector
	float l = normal.squaredNorm();

	// normalize the vector
	normal << a/l, b/l, c/l;

	// and divide d by the length as well
	this->d = d/l;
}

void Plane::setCoefficients2(float a, float b, float c, float d) 
{
	// set the normal vector
	normal << a, b, c;
	this->d = d;

	normalize();
}

void Plane::normalize()
{
	//compute the length of the vector
	float l = normal.norm();

	// normalize the vector
	normal = normal/l;

	// and divide d by the length as well
	this->d = d/l;
}

float Plane::distance(const Vector3f &p) 
{
	return d + normal.dot(p);
}

// Change suggested by Christina
float Plane::distance2(const Vector3f &p) 
{
	return normal.dot(point - p);
}

void Plane::print() 
{
	cout << "Normal: " << normal << endl;
	cout << "d: " << d << endl;
}

/**
 * @brief Construct a new Frustum from default equation of planes.
 * Here the frusutm intrinsic parameters aren't well defined.
 */
Frustum::Frustum() 
{
	Matrix<float, 6, 4> samplePlanes;
	samplePlanes << -2.32018,	-3.08633,	0.139777,	21.1138,	
					3.04084,	1.83392,	-1.52258,	21.1138,	
					1.60987,	-1.50022,	0.751411,	21.1138,
					-0.889218,	0.247813,	-2.13421,	21.1138,
					1.35521,	-2.35517,	-2.60037,	27.5265,
					-0.634549,	1.10276,	1.21757,	14.701;
					
	for(int i = TOP; i <= FAR; i++) 
	{
		pl[i].setCoefficients2(samplePlanes(i, 0), samplePlanes(i, 1), samplePlanes(i, 2), samplePlanes(i, 3));
	}

	frameID = 0;
}

Frustum::Frustum(const Frustum &frustum)
{
	for(int i = TOP; i <= FAR; i++) 
		pl[i] = frustum.pl[i];

	ntl = frustum.ntl;
	ntr = frustum.ntr;
	nbl = frustum.nbl;
	nbr = frustum.nbr;
	ftl = frustum.ftl;
	ftr = frustum.ftr;
	fbl = frustum.fbl;
	fbr = frustum.fbr;

	nearD = frustum.nearD;
	farD = frustum.farD;
	ratio = frustum.ratio;
	angle = frustum.angle;
	tang = frustum.tang;
	
	nw = frustum.nw;
	nh = frustum.nh;
	fw = frustum.fw;
	fh = frustum.fh;

	pos = frustum.pos;
	rot = frustum.rot;
	quat = frustum.quat;
	lookAt = frustum.lookAt;
	up = frustum.up;

	frameID = frustum.frameID;
}

Frustum::~Frustum() {}

void Frustum::setCamInternals(const CamInt &cam) 
{
	this->ratio = cam.ratio;
	this->angle = cam.angle;
	this->nearD = cam.nearD;
	this->farD = cam.farD;

	//////////////////////////////////////////////////
	// Christina's fix
	tang = (float)tan(angle * 0.5);
	nh = 2 * nearD * tang;
	nw = nh * ratio; 
	fh = 2 * farD * tang;
	fw = fh * ratio;
	//////////////////////////////////////////////////

	// std::cout << "ratio: " << ratio << std::endl;
	// std::cout << "near height: " << nh << std::endl;
	// std::cout << "near width: " << nw << std::endl;
	// std::cout << "far height: " << fh << std::endl;
	// std::cout << "far width: " << fw << std::endl;
}

// void Frustum::setCamPlanes(const CamView &cam) 
// {
// 	Vector3f dir, nc, fc, X, Y, Z;

// 	Z = cam.p - cam.l;
// 	Z.normalize();

// 	X = cam.u.cross(Z);
// 	X.normalize();

// 	Y = Z.cross(X);
// 	Y.normalize();

// 	nc = cam.p - (Z * nearD);
// 	fc = cam.p - (Z * farD);

// 	ntl = nc + (Y * nh) - (X * nw);
// 	ntr = nc + (Y * nh) + (X * nw);
// 	nbl = nc - (Y * nh) - (X * nw);
// 	nbr = nc - (Y * nh) + (X * nw);

// 	ftl = fc + (Y * fh) - (X * fw);
// 	ftr = fc + (Y * fh) + (X * fw);
// 	fbl = fc - (Y * fh) - (X * fw);
// 	fbr = fc - (Y * fh) + (X * fw);

// 	pl[TOP].set3Points(ntr, ntl, ftl);
// 	pl[BOTTOM].set3Points(nbl, nbr, fbr);
// 	pl[LEFT].set3Points(ntl, nbl, fbl);
// 	pl[RIGHT].set3Points(nbr, ntr, fbr);
// 	pl[NEAR].set3Points(ntl, ntr, nbr);
// 	pl[FAR].set3Points(ftr, ftl, fbl);
// }

//////////////////////////////////////////////////
/**
 * @brief Calculate frustum planes from camera view.
 * Uses Christina's fix
 * @param cam CamView : Camera view [position, rotation, quaternion, lookAt, up]
 */
void Frustum::setCamPlanes(const CamView &cam) 
{
	Vector3f dir, nc, fc, X, Y, Z;

	// We don't need to calibrate camera coordinates.
	Z = cam.l;		// Camera direction is Z+ in this case
	Z.normalize();

	Y = cam.u;
	Y.normalize();

	X = Z.cross(Y);
	X.normalize();

	nc = cam.p + (Z * nearD);
	fc = cam.p + (Z * farD);

	/**
	 * @brief Negate the Y axis to align with the Unity's camera direction.
	 * This rotates the frustum wrt y-axis, thus the point cloud turns upside.
	 * Since Open3D uses right-hand coordinate system, the camera is opposite to Unity's camera along y and z axis.
	 * Point cloud is already rotated along z-axis. If we rotate the point cloud along y-axis, it will be upside but the frustum will not be rotated along y-axis.
	 * So we don't rotate the ptcl, rather rotate the frustum along y-axis.
	 */

	Matrix3f rotMat;
	rotMat << 1, 0, 0,
			  0, -1, 0,
			  0, 0, 1;

	ntl = rotMat * (nc + (Y * nh/2) - (X * nw/2));
	ntr = rotMat * (nc + (Y * nh/2) + (X * nw/2));
	nbl = rotMat * (nc - (Y * nh/2) - (X * nw/2));
	nbr = rotMat * (nc - (Y * nh/2) + (X * nw/2));

	ftl = rotMat * (fc + (Y * fh/2) - (X * fw/2));
	ftr = rotMat * (fc + (Y * fh/2) + (X * fw/2));
	fbl = rotMat * (fc - (Y * fh/2) - (X * fw/2));
	fbr = rotMat * (fc - (Y * fh/2) + (X * fw/2));

	pos = rotMat * cam.p;
	rot = cam.r;
	quat = cam.q;
	lookAt = rotMat * cam.l;
	up = rotMat * cam.u;

	pl[TOP].set3Points2(ntl, ntr, ftl);
	pl[BOTTOM].set3Points2(nbl, fbl, nbr);
	pl[LEFT].set3Points2(ntl, ftl, nbl);
	pl[RIGHT].set3Points2(ntr, nbr, ftr);
	pl[NEAR].set3Points2(ntl, nbl, nbr);
	pl[FAR].set3Points2(ftl, ftr, fbl);
}
//////////////////////////////////////////////////

/**
 * @brief Set frustum planes.
 */
void Frustum::setCamPlanes(const Plane &left, const Plane &right, const Plane &bottom, const Plane &top, const Plane &near, const Plane &far)
{
	pl[TOP] = top;
	pl[BOTTOM] = bottom;
	pl[LEFT] = left;
	pl[RIGHT] = right;
	pl[NEAR] = near;
	pl[FAR] = far;
}

// Change suggested by Christina
void Frustum::add_buffer(float buffer)
{
	for (int i = 0; i < 6; i++)
        pl[i].point = buffer * pl[i].normal + pl[i].point;
}

int Frustum::pointInFrustum(const Vector3f &p) 
{
	int result = INSIDE;
	for(int i = 0; i < 6; i++) 
	{
		if (pl[i].distance(p) < 0)
			return OUTSIDE;
	}
	return result;
}

// Change suggested by Christina
int Frustum::pointInFrustum2(const Vector3f &p) 
{
	int result = INSIDE;
	for(int i = 0; i < 6; i++) 
	{
		if (pl[i].distance2(p) > 0.0f)
			return OUTSIDE;
	}
	return result;
}

int Frustum::sphereInFrustum(const Vector3f &p, float ratio) 
{
	int result = INSIDE;
	float distance;

	for(int i = 0; i < 6; i++) {
		distance = pl[i].distance(p);
		if (distance < -ratio)
			return OUTSIDE;
		else if (distance < ratio)
			result = INTERSECT;
	}
	return result;
}

/**
 * @brief http://www.lighthouse3d.com/tutorials/view-frustum-culling/geometric-approach-testing-boxes/
 * TODO: RAJRUP - http://www.lighthouse3d.com/tutorials/view-frustum-culling/geometric-approach-testing-boxes-ii/
 * @param aabb 
 * @return int : INSIDE, OUTSIDE or INTERSECT
 */
int Frustum::aabbInFrustum(AABB &aabb)
{
	int result = INSIDE, out, in;

	// for each plane do ...
	for(int i = TOP; i <= FAR; i++) 
	{
		// reset counters for corners in and out
		out = 0;
		in = 0;

		// for each corner of the box do ...
		// get out of the cycle as soon as a box as corners
		// both inside and out of the frustum
		for (int k = 0; k < N_AABB_VERTEX && (in == 0 || out == 0); k++) // Has Early exit
		{
			// is the corner outside or inside
			if (pl[i].distance(aabb.getVertex(k).head<3>()) < 0.0f)
				out++;
			else
				in++;
		}
		//if all corners are out
		if (!in)
			return OUTSIDE;
		// if some corners are out and others are in
		else if (out)
			result = INTERSECT;
	}
	return result;
}

int Frustum::aabbInFrustum2(AABB &aabb)
{
	int result = INSIDE, out, in;

	// for each plane do ...
	for(int i = TOP; i <= FAR; i++) 
	{
		// reset counters for corners in and out
		out = 0;
		in = 0;

		// for each corner of the box do ...
		// get out of the cycle as soon as a box as corners
		// both inside and out of the frustum
		for (int k = 0; k < N_AABB_VERTEX && (in == 0 || out == 0); k++) // Has Early exit
		{
			// is the corner outside or inside
			if (pl[i].distance2(aabb.getVertex(k).head<3>()) > 0.0f)
				out++;
			else
				in++;
		}
		//if all corners are out
		if (!in)
			return OUTSIDE;
		// if some corners are out and others are in
		else if (out)
			result = INTERSECT;
	}
	return result;
}

void Frustum::getFrustumCubeEnclosedCorners(Vector3f &minCorner, Vector3f &maxCorner)
{
	// TODO: Rajrup: Change the corners to an array with enum values.
	Vector3f corners[8] = {ntl, ntr, nbl, nbr, ftl, ftr, fbl, fbr};

	for(int i = 0; i < 8; i++)
	{
		if(i == 0)
		{
			minCorner = corners[i];
			maxCorner = corners[i];
		}
		else
		{
			minCorner = minCorner.cwiseMin(corners[i]);
			maxCorner = maxCorner.cwiseMax(corners[i]);
		}
	}
}

bool Frustum::saveToFile(const string &path, const string &filename)
{
	// Creating a directory
	string frustum_path = FORMAT(path << "frustum/");
    if(!fs::exists(frustum_path))
    {
        if(!fs::create_directories(frustum_path))
        {
            cout << "ERROR: " << "Directory creation failed, path "<< frustum_path << endl;
            return -1;
        }
    }

	fs::path filepath{FORMAT(frustum_path << filename)};
	ofstream file;
	file.open(filepath.c_str(), ofstream::out | ofstream::trunc);

	if (!file.is_open())
	{	
		cout << "Error: Failed to open file: " << filepath << endl;
		return false;
	}

	cout << "Saving frustum to file: " << filename << endl;
	file << "---------------------------------------------------" << endl;
	for(int i = TOP; i <= FAR; i++) 
	{
		Vector3f normal = pl[i].normal;
		float d = pl[i].d;
		file << "Plane " << getPlane2String(i) << ": " << normal(0) << ", " << normal(1) << ", " << normal(2) << ", " << d << std::endl;
	}
	file << "---------------------------------------------------" << endl;
	cout << "Frame ID: " << frameID << endl;
	file << "ntl: " << ntl(0) << ", " << ntl(1) << ", " << ntl(2) << endl;
	file << "ntr: " << ntr(0) << ", " << ntr(1) << ", " << ntr(2) << endl;
	file << "nbl: " << nbl(0) << ", " << nbl(1) << ", " << nbl(2) << endl;
	file << "nbr: " << nbr(0) << ", " << nbr(1) << ", " << nbr(2) << endl;
	file << "ftl: " << ftl(0) << ", " << ftl(1) << ", " << ftl(2) << endl;
	file << "ftr: " << ftr(0) << ", " << ftr(1) << ", " << ftr(2) << endl;
	file << "fbl: " << fbl(0) << ", " << fbl(1) << ", " << fbl(2) << endl;
	file << "fbr: " << fbr(0) << ", " << fbr(1) << ", " << fbr(2) << endl;
	file << "nearD: " << nearD << endl;
	file << "farD: " << farD << endl;
	file << "ratio: " << ratio << endl;
	file << "angle: " << angle << endl;
	file << "tang: " << tang << endl;
	file << "nw: " << nw << endl;
	file << "nh: " << nh << endl;
	file << "fw: " << fw << endl;
	file << "fh: " << fh << endl;
	file << "---------------------------------------------------" << endl;

	file.close();
	return true;
}

void Frustum::print()
{
	cout << "---------------------------------------------------" << endl;
	cout << "Frame ID: " << frameID << endl;
	cout << "ntl: " << ntl(0) << ", " << ntl(1) << ", " << ntl(2) << endl;
	cout << "ntr: " << ntr(0) << ", " << ntr(1) << ", " << ntr(2) << endl;
	cout << "nbl: " << nbl(0) << ", " << nbl(1) << ", " << nbl(2) << endl;
	cout << "nbr: " << nbr(0) << ", " << nbr(1) << ", " << nbr(2) << endl;
	cout << "ftl: " << ftl(0) << ", " << ftl(1) << ", " << ftl(2) << endl;
	cout << "ftr: " << ftr(0) << ", " << ftr(1) << ", " << ftr(2) << endl;
	cout << "fbl: " << fbl(0) << ", " << fbl(1) << ", " << fbl(2) << endl;
	cout << "fbr: " << fbr(0) << ", " << fbr(1) << ", " << fbr(2) << endl;
	cout << "nearD: " << nearD << endl;
	cout << "farD: " << farD << endl;
	cout << "ratio: " << ratio << endl;
	cout << "angle: " << angle << endl;
	cout << "tang: " << tang << endl;
	cout << "nw: " << nw << endl;
	cout << "nh: " << nh << endl;
	cout << "fw: " << fw << endl;
	cout << "fh: " << fh << endl;
	cout << "Plane: #########################" << endl;
	for(int i = TOP; i <= FAR; i++) 
	{
		Vector3f normal = pl[i].normal;
		float d = pl[i].d;
		cout << "Plane " << i << ": " << normal(0) << ", " << normal(1) << ", " << normal(2) << ", " << d << std::endl;
	}
	cout << "---------------------------------------------------" << endl;
}

Plane4f::Plane4f() {}

Plane4f::~Plane4f() {}

Plane4f::Plane4f(const Vector4f &eq)
{
	this->eq = eq;
}

Plane4f::Plane4f(const Plane4f &plane)
{
	this->eq = plane.eq;
}

void Plane4f::setEquation(const Vector4f &eq)
{
	this->eq = eq;
}

float Plane4f::distance(const Vector4f &p)
{
	return eq.dot(p);
}

void Plane4f::normalize()
{
	float l = eq.head<3>().norm();
	eq = eq / l;
}

FrustumClip::FrustumClip() 
{
	Matrix<float, 6, 4> samplePlanes;
	samplePlanes << -2.32018,	-3.08633,	0.139777,	21.1138,	
					3.04084,	1.83392,	-1.52258,	21.1138,	
					1.60987,	-1.50022,	0.751411,	21.1138,
					-0.889218,	0.247813,	-2.13421,	21.1138,
					1.35521,	-2.35517,	-2.60037,	27.5265,
					-0.634549,	1.10276,	1.21757,	14.701;
					
	for(int i = TOP; i <= FAR; i++) 
	{
		pl[i] = Plane4f(Vector4f(samplePlanes.row(i)));
	}
}

FrustumClip::FrustumClip(const FrustumClip &frustum)
{
	for(int i = TOP; i <= FAR; i++) 
	{
		pl[i] = frustum.pl[i];
	}
}

FrustumClip::~FrustumClip() {}

void FrustumClip::setClipPlanes(const Matrix4d &camProjViewMat) // Camera PV matrix has to be Matrix4d format as deirved from PCL viewer
{
	Vector4f left = (camProjViewMat.row(3) + camProjViewMat.row(0)).cast<float>();
	Vector4f right = (camProjViewMat.row(3) - camProjViewMat.row(0)).cast<float>();
	Vector4f bottom = (camProjViewMat.row(3) + camProjViewMat.row(1)).cast<float>();
	Vector4f top = (camProjViewMat.row(3) - camProjViewMat.row(1)).cast<float>();
	Vector4f near = (camProjViewMat.row(3) + camProjViewMat.row(2)).cast<float>();
	Vector4f far = (camProjViewMat.row(3) - camProjViewMat.row(2)).cast<float>();

	pl[LEFT].setEquation(left);
	pl[RIGHT].setEquation(right);
	pl[BOTTOM].setEquation(bottom);
	pl[TOP].setEquation(top);
	pl[NEAR].setEquation(near);
	pl[FAR].setEquation(far);
}

int FrustumClip::pointInFrustum(const Vector4f &p) 
{
	int result = INSIDE;
	for(int i = TOP; i <= FAR; i++) 
	{
		if (pl[i].distance(p) < 0.0f)
			return OUTSIDE;
	}
	return result;
}

/**
 * @brief Finds where the point is wrt each of the frustum planes.
 * Early exit if a point is the left of any of the frustum planes
 * @param p 
 * @param p2Planes : Only calculate the distance to the planes pl[i] where p2Planes[i] = UNASSIGNED
 * @return int : INSIDE, OUTSIDE
 */
int FrustumClip::pointInFrustum(const Vector4f &p, int p2Planes[N_FRUSTRUM_PLANE])
{
	int sum = 0;
	for(int i = TOP; i <= FAR; i++) 
	{
		if(p2Planes[i] == LEFT_TO_PLANE)
			return OUTSIDE;
		else if(p2Planes[i] == UNASSIGNED)
		{
			if(pl[i].distance(p) < 0)
			{
				p2Planes[i] = LEFT_TO_PLANE;
				return OUTSIDE;
			}
			p2Planes[i] = RIGHT_TO_PLANE;
		}
	}
	return INSIDE;
}

/**
 * @brief http://www.lighthouse3d.com/tutorials/view-frustum-culling/geometric-approach-testing-boxes/
 * TODO: RAJRUP - http://www.lighthouse3d.com/tutorials/view-frustum-culling/geometric-approach-testing-boxes-ii/
 * @param aabb 
 * @return int : INSIDE, OUTSIDE or INTERSECT
 */
int FrustumClip::aabbInFrustum(AABB &aabb)
{
	int result = INSIDE, out, in;

	// for each plane do ...
	for(int i = TOP; i <= FAR; i++) 
	{
		// reset counters for corners in and out
		out = 0;
		in = 0;

		// for each corner of the box do ...
		// get out of the cycle as soon as a box as corners
		// both inside and out of the frustum
		for (int k = 0; k < N_AABB_VERTEX && (in == 0 || out == 0); k++) // Has Early exit
		{
			// is the corner outside or inside
			if (pl[i].distance(aabb.getVertex(k)) < 0.0f)
				out++;
			else
				in++;
		}
		//if all corners are out
		if (!in)
			return OUTSIDE;
		// if some corners are out and others are in
		else if (out)
			result = INTERSECT;
	}
	return result;
}

void FrustumClip::print()
{	
	std::string sep = "\n----------------------------------------\n";
	IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	for(int i = TOP; i <= FAR; i++) 
	{
		std::cout << "Plane " << i << ": " << pl[i].eq.transpose().format(CleanFmt) << std::endl;
	}
	std::cout << sep;
}

bool FrustumClip::saveToFile(const string &path, const string &filename)
{
	// Creating a directory
	string frustum_path = FORMAT(path << "frustum/");
    if(!fs::exists(frustum_path))
    {
        if(!fs::create_directories(frustum_path))
        {
            cout << "ERROR: " << "Directory creation failed, path "<< frustum_path << endl;
            return -1;
        }
    }

	fs::path filepath{FORMAT(frustum_path << filename)};
	ofstream file;
	file.open(filepath.c_str(), ofstream::out | ofstream::trunc);

	if (!file.is_open())
	{	
		cout << "Error: Failed to open file: " << filepath << endl;
		return false;
	}

	cout << "Saving frustum to file: " << filename << endl;
	for(int i = TOP; i <= FAR; i++) 
	{
		for(int j = 0; j < 4; j++)		// 4 elements in a plane equation represented as Vector4f
		{
			file << pl[i].eq(j) << "\t";
		}
		file << endl;
	}
	file.close();
	return true;
}

bool FrustumClip::loadFromFile(const string &path, const string &filename)
{
	string frustum_path = FORMAT(path << "frustum/");
	if(!fs::exists(frustum_path))
	{
		cout << "ERROR: " << "Directory does not exist, path "<< frustum_path << endl;
		return false;
	}

	fs::path filepath{FORMAT(frustum_path << filename)};
	ifstream file;
	file.open(filepath.c_str(), ios::in);

	if (!file.is_open())
	{	
		cout << "Error: Failed to open file: " << filepath << endl;
		return false;
	}

	cout << "Loading frustum from file: " << filename << endl;
	for(int i = TOP; i <= FAR; i++) 
	{
		for (int j = 0; j < 4; j++)
		{
			file >> pl[i].eq(j);
		}
	}
	file.close();
	return true;
}