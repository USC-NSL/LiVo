#pragma once
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

#define N_AABB_VERTEX 8

enum
{
	TopLeftFront = 0,
	TopRightFront,		// (maxX, maxY, maxZ)
	BottomRightFront,
	BottomLeftFront,
	TopLeftBack,
	TopRightBack,
	BottomRightBack,
	BottomLeftBack		// (minX, minY, minZ)
};

/** 
 * Axis-aligned bounding box
 *         Y
 * 		   |
 * 		   |
 * 		    -------- X
 * 		  /
 * 		 /
 * 	    Z
 */
class AABB
{

public:
	Vector4f mid, halfLength; 	// mid: (mid_x, mid_y, mid_z, 1.0)	halfLength: (hl_x, hl_y, hl_z, 1.0)

	AABB() : mid(Vector4f(0.0, 0.0, 0.0, 1.0)), halfLength(Vector4f(0.0, 0.0, 0.0, 1.0)) {};
	AABB(const Vector4f &mid, const Vector4f &halfLength) : mid(mid), halfLength(halfLength) {};
	~AABB() {};

	void setMid(const Vector4f &mid) { this->mid = mid; }
	void setHalfLength(const Vector4f &halfLength) { this->halfLength = halfLength; }

	Vector4f getVertex(int index)
	{
		return mid + halfLength.cwiseProduct(boundsOffsetTable[index]);
	}

	Vector4f getMin()
	{
		return mid + halfLength.cwiseProduct(boundsOffsetTable[BottomLeftBack]);
	}

	Vector4f getMax()
	{
		return mid + halfLength.cwiseProduct(boundsOffsetTable[TopRightFront]);
	}

	void print()
	{
		for (int i = TopLeftFront; i <= BottomLeftBack; i++)
		{
			switch(i)
			{
			case TopLeftFront:
				cout << "TopLeftFront: ";
				break;
			case TopRightFront:
				cout << "TopRightFront: ";
				break;
			case BottomRightFront:
				cout << "BottomRightFront: ";
				break;
			case BottomLeftFront:
				cout << "BottomLeftFront: ";
				break;
			case TopLeftBack:
				cout << "TopLeftBack: ";
				break;
			case TopRightBack:
				cout << "TopRightBack: ";
				break;
			case BottomRightBack:
				cout << "BottomRightBack: ";
				break;
			case BottomLeftBack:
				cout << "BottomLeftBack: ";
				break;
			default:
				break;
			}
			cout << getVertex(i).transpose() << endl;
		}
		
	}

private:
	Vector4f boundsOffsetTable[N_AABB_VERTEX] =
								{
									Vector4f(-1.0f, +1.0f, +1.0f, 0.0f), // Last element shouldn't be added
									Vector4f(+1.0f, +1.0f, +1.0f, 0.0f),
									Vector4f(+1.0f, -1.0f, +1.0f, 0.0f),
									Vector4f(-1.0f, -1.0f, +1.0f, 0.0f),
									Vector4f(-1.0f, +1.0f, -1.0f, 0.0f),
									Vector4f(+1.0f, +1.0f, -1.0f, 0.0f),
									Vector4f(+1.0f, -1.0f, -1.0f, 0.0f),
									Vector4f(-1.0f, -1.0f, -1.0f, 0.0f)
								};
};

#define N_FRUSTRUM_PLANE 6

enum 
{
	TOP = 0,
	BOTTOM,
	LEFT,
	RIGHT,
	NEAR,
	FAR
};
static const string Plane2String[] = { 
										"Top (t)", 
										"Bottom (b)",
										"Left (l)",
										"Right (r)",
										"Near (n)",
										"Far (f)"
									  };

enum
{
	LEFT_TO_PLANE = 0,
	RIGHT_TO_PLANE = 1,
	UNASSIGNED = -1
};

enum 
{
	OUTSIDE = 0, 
	INTERSECT, 
	INSIDE
};

class Plane
{

public:

	Vector3f normal, point;		// normal vector and point (point where vector from origin touches the plane) EQUATION: normal.p + d = 0 where p (p_x, p_y, p_z) is the point
	float d;					// distance of the plane from the origin

	Plane(const Vector3f &v1, const Vector3f &v2, const Vector3f &v3);
	Plane(const Plane &plane);
	Plane();
	~Plane();

	void normalize();
	void set3Points(const Vector3f &v1, const Vector3f &v2, const Vector3f &v3);
	void set3Points2(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3);
	void setNormalAndPoint(const Vector3f &normal, const Vector3f &point);
	void setCoefficients(float a, float b, float c, float d);
	void setCoefficients2(float a, float b, float c, float d);
	float distance(const Vector3f &p);
	float distance2(const Vector3f &p);

	void print();
};

class Plane4f
{
public:
	Vector4f eq; // eq_0*p_x + eq_1*p_y + eq_2*p_z + eq_3 = 0 where point p on the plane is represented by (p_x, p_y, p_z, 1)
	
	Plane4f(const Vector4f &eq);
	Plane4f(const Plane4f &plane);
	Plane4f();
	~Plane4f();

	void setEquation(const Vector4f &eq);
	float distance(const Vector4f &p);
	void normalize();

	void print();
};

typedef struct
{
	float angle;	// fovy in radian
	float ratio;	// aspect ratio
	float nearD;	// distance to near plane
	float farD;		// distance to far plane

} CamInt;

typedef struct
{
	Vector3f p; // camera position
	Vector3f r; // camera rotation (in radians)
	Vector4f q; // camera quaternion
	Vector3f l; // camera forward vector
	Vector3f u; // camera up vector

} CamView;

class Frustum
{

private:

public:

	Plane pl[N_FRUSTRUM_PLANE];

	Vector3f ntl, ntr, nbl, nbr, ftl, ftr, fbl, fbr;
	float nearD, farD, ratio, angle, tang;
	float nw, nh, fw, fh;
	Vector3f pos, rot;
	Vector4f quat;
	Vector3f lookAt, up;
	uint32_t frameID;

	Frustum();
	Frustum(const Frustum &frustum);
	~Frustum();

	void setCamInternals(const CamInt &cam);
	void setCamPlanes(const CamView &cam);
	void setCamPlanes(const Plane &left, const Plane &right, const Plane &bottom, const Plane &top, const Plane &near, const Plane &far);
	int pointInFrustum(const Vector3f &p);
	int pointInFrustum2(const Vector3f &p);
	int sphereInFrustum(const Vector3f &p, float ratio);
	int aabbInFrustum(AABB &aabb);
	int aabbInFrustum2(AABB &aabb);
	void add_buffer(float buffer_size);
	void getFrustumCubeEnclosedCorners(Vector3f &minCorner, Vector3f &maxCorner);
	bool saveToFile(const string &path, const string &filename);
	void print();
};

class FrustumClip
{

public:

	Plane4f pl[N_FRUSTRUM_PLANE];

	FrustumClip();
	FrustumClip(const FrustumClip &frustum);
	~FrustumClip();
	
	void setClipPlanes(const Matrix4d &camProjViewMat);
	int pointInFrustum(const Vector4f &p);
	int pointInFrustum(const Vector4f &p, int p2Planes[N_FRUSTRUM_PLANE]);
	int sphereInFrustum(const Vector4f &p, float ratio);
	int aabbInFrustum(AABB &aabb);
	void print();
	bool saveToFile(const string &path, const string &filename);
	bool loadFromFile(const string &path, const string &filename);

private:
};