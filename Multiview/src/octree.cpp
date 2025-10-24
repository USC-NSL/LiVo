// Implementation of Octree in c++
#include <iostream>
#include "octree.h"

#define ep 0.0
#define delta 1e-6

/**
 * Rajrup: Octree construction
 * TODO:
 * Change Pointf to Eigen::Vector3f
 */

// Constructor
Octree::Octree()
{
	// To declare empty node
	point = new Pointf();
}

// Constructor with three arguments
Octree::Octree(float x, float y, float z)
{
	// To declare point node
	point = new Pointf(x, y, z);
}

// Constructor with six arguments
Octree::Octree(float x1, float y1, float z1, float x2, float y2, float z2)
{
	// This use to construct Octree
	// with boundaries defined
	if (x2 < x1 || y2 < y1 || z2 < z1)
	{
		cout << "boundary poitns are not valid" << endl;
		return;
	}

	point = nullptr;
	topLeftFront = new Pointf(x1, y1, z1);
	bottomRightBack = new Pointf(x2, y2, z2);

	// Calculate the mid point of the cube
	mid = new Pointf((x1 + x2) / 2.0, (y1 + y2) / 2.0, (z1 + z2) / 2.0);
	halfLength = new Pointf((x2 - x1) / 2.0, (y2 - y1) / 2.0, (z2 - z1) / 2.0);

	// Assigning null to the children
	children.assign(8, nullptr);
	for (int i = TopLeftFront; i <= BottomLeftBack; i++)
		children[i] = new Octree(); // empty children
}

Octree::~Octree() {}

// Function to insert a point in the octree
void Octree::insert(float x, float y, float z)
{
	// If the point already exists in the octree
	// if (find(x, y, z)) 
	// {
	// 	cout << "Point already exist in the tree" << endl;
	// 	return;
	// }

	// If the point is out of bounds
	if (x < topLeftFront->X
		|| x > bottomRightBack->X
		|| y < topLeftFront->Y
		|| y > bottomRightBack->Y
		|| z < topLeftFront->Z
		|| z > bottomRightBack->Z) 
	{
		cout << "Point is out of bound" << endl;
		return;
	}

	// Binary search to insert the point
	float midx = (topLeftFront->X + bottomRightBack->X) / 2.0;
	float midy = (topLeftFront->Y + bottomRightBack->Y) / 2.0;
	float midz = (topLeftFront->Z + bottomRightBack->Z) / 2.0;

	int pos = -1;

	// Checking the octant of the point
	if (x <= midx) 
	{
		if (y <= midy) 
		{
			if (z <= midz)
				pos = TopLeftFront;
			else
				pos = TopLeftBack;
		}
		else 
		{
			if (z <= midz)
				pos = BottomLeftFront;
			else
				pos = BottomLeftBack;
		}
	}
	else 
	{
		if (y <= midy) 
		{
			if (z <= midz)
				pos = TopRightFront;
			else
				pos = TopRightBack;
		}
		else 
		{
			if (z <= midz)
				pos = BottomRightFront;
			else
				pos = BottomRightBack;
		}
	}

	// If an internal node is encountered
	if (children[pos]->point == nullptr) 
	{
		children[pos]->insert(x, y, z);
		return;
	}

	// If an empty node is encountered
	else if (children[pos]->point->X == -1.0) 
	{
		delete children[pos];
		children[pos] = new Octree(x, y, z);
		return;
	}
	else if (x == children[pos]->point->X
			&& y == children[pos]->point->Y
			&& z == children[pos]->point->Z)
	{
		cout << "Point already exist in the tree" << endl;
		return;
	}
	else 
	{
		float x_ = children[pos]->point->X,
			y_ = children[pos]->point->Y,
			z_ = children[pos]->point->Z;
		delete children[pos];
		children[pos] = nullptr;
		if (pos == TopLeftFront) 
		{
			children[pos] = new Octree(topLeftFront->X,
									topLeftFront->Y,
									topLeftFront->Z,
									midx,
									midy,
									midz);
		}

		else if (pos == TopRightFront) {
			children[pos] = new Octree(midx + ep,
									topLeftFront->Y,
									topLeftFront->Z,
									bottomRightBack->X,
									midy,
									midz);
		}
		else if (pos == BottomRightFront) {
			children[pos] = new Octree(midx + ep,
									midy + ep,
									topLeftFront->Z,
									bottomRightBack->X,
									bottomRightBack->Y,
									midz);
		}
		else if (pos == BottomLeftFront) {
			children[pos] = new Octree(topLeftFront->X,
									midy + ep,
									topLeftFront->Z,
									midx,
									bottomRightBack->Y,
									midz);
		}
		else if (pos == TopLeftBack) {
			children[pos] = new Octree(topLeftFront->X,
									topLeftFront->Y,
									midz + ep,
									midx,
									midy,
									bottomRightBack->Z);
		}
		else if (pos == TopRightBack) {
			children[pos] = new Octree(midx + ep,
									topLeftFront->Y,
									midz + ep,
									bottomRightBack->X,
									midy,
									bottomRightBack->Z);
		}
		else if (pos == BottomRightBack) {
			children[pos] = new Octree(midx + ep,
									midy + ep,
									midz + ep,
									bottomRightBack->X,
									bottomRightBack->Y,
									bottomRightBack->Z);
		}
		else if (pos == BottomLeftBack) {
			children[pos] = new Octree(topLeftFront->X,
									midy + ep,
									midz + ep,
									midx,
									bottomRightBack->Y,
									bottomRightBack->Z);
		}
		children[pos]->insert(x_, y_, z_);
		children[pos]->insert(x, y, z);
	}
}

// Function that returns true if the point
// (x, y, z) exists in the octree
bool Octree::find(float x, float y, float z)
{	
	// If point is out of bound
	if (x < topLeftFront->X
		|| x > bottomRightBack->X
		|| y < topLeftFront->Y
		|| y > bottomRightBack->Y
		|| z < topLeftFront->Z
		|| z > bottomRightBack->Z)
		return 0;

	// Otherwise perform binary search
	// for each ordinate
	float midx = (topLeftFront->X + bottomRightBack->X) / 2.0;
	float midy = (topLeftFront->Y + bottomRightBack->Y) / 2.0;
	float midz = (topLeftFront->Z + bottomRightBack->Z) / 2.0;

	int pos = -1;

	// Deciding the position
	// where to move
	if (x <= midx) 
	{
		if (y <= midy) 
		{
			if (z <= midz)
				pos = TopLeftFront;
			else
				pos = TopLeftBack;
		}
		else 
		{
			if (z <= midz)
				pos = BottomLeftFront;
			else
				pos = BottomLeftBack;
		}
	}
	else 
	{
		if (y <= midy) 
		{
			if (z <= midz)
				pos = TopRightFront;
			else
				pos = TopRightBack;
		}
		else 
		{
			if (z <= midz)
				pos = BottomRightFront;
			else
				pos = BottomRightBack;
		}
	}

	// If an internal node is encountered
	if (children[pos]->point == nullptr)
		return children[pos]->find(x, y, z);

	// If an empty node is encountered
	else if (children[pos]->point->X == -1.0)
		return 0;
	else 
	{
		// If node is found with
		// the given value
		if (x == children[pos]->point->X
			&& y == children[pos]->point->Y
			&& z == children[pos]->point->Z)
			return 1;
	}
	return 0;
}

// Constructor
OctreeEigen::OctreeEigen()
{
	// empty node
	point = nullptr; 
	isLeaf = true;
}

// Destructor
OctreeEigen::~OctreeEigen() 
{
	// Do not delete the point as it points to orginal point cloud data structure
	point = nullptr; 

	for (int i = 0; i < children.size(); i++)
	{
		delete children[i];
		children[i] = nullptr;
	}
}

// Constructor with three arguments
OctreeEigen::OctreeEigen(Vector3f *point)
{
	// To declare point node
	this->point = point;
	isLeaf = true;
}

// Constructor with six arguments
OctreeEigen::OctreeEigen(const Vector3f &topLeftFront, const Vector3f &bottomRightBack)
{
	// This use to construct Octree
	// with boundaries defined
	if (bottomRightBack[0] < topLeftFront[0] || bottomRightBack[1] < topLeftFront[1] || bottomRightBack[2] < topLeftFront[2])
	{
		cout << "boundary poitns are not valid" << endl;
		return;
	}

	isLeaf = false;
	this->topLeftFront = topLeftFront;
	this->bottomRightBack = bottomRightBack;

	// Calculate the mid point of the cube
	this->mid = (topLeftFront + bottomRightBack) / 2.0;
	this->halfLength = (bottomRightBack - topLeftFront) / 2.0;

	// Assigning null to the children
	children.assign(8, nullptr);
	for (int i = TopLeftFront; i <= BottomLeftBack; i++)
		children[i] = new OctreeEigen(); // empty children
}

// Function to insert a point in the octree
void OctreeEigen::insert(Vector3f *point)
{
	Vector3f point_ = *point;

	// If the point is out of bounds
	if (point_[0] < topLeftFront[0] 
		|| point_[0] > bottomRightBack[0] 
		|| point_[1] < topLeftFront[1] 
		|| point_[1] > bottomRightBack[1] 
		|| point_[2] < topLeftFront[2] 
		|| point_[2] > bottomRightBack[2])
	{
		cout << "Point is out of bound" << endl;
		return;
	}

	// Binary search to insert the point
	Vector3f midTemp = (topLeftFront + bottomRightBack) / 2.0;

	int pos = -1;

	// Checking the octant of the point
	if (point_[0] <= midTemp[0]) 
	{
		if (point_[1] <= midTemp[1]) 
		{
			if (point_[2] <= midTemp[2])
				pos = TopLeftFront;
			else
				pos = TopLeftBack;
		}
		else 
		{
			if (point_[2] <= midTemp[2])
				pos = BottomLeftFront;
			else
				pos = BottomLeftBack;
		}
	}
	else 
	{
		if (point_[1] <= midTemp[1]) 
		{
			if (point_[2] <= midTemp[2])
				pos = TopRightFront;
			else
				pos = TopRightBack;
		}
		else 
		{
			if (point_[2] <= midTemp[2])
				pos = BottomRightFront;
			else
				pos = BottomRightBack;
		}
	}

	// If an internal node is encountered
	if (children[pos]->isLeaf == false) 
	{
		children[pos]->insert(point);
		return;
	}

	// If an empty child node is encountered, it must be a leaf node
	else if (children[pos]->point == nullptr) 
	{
		// assert(children[pos]->isLeaf == true);
		children[pos]->point = point;
		return;
	}
	else if(*point == *(children[pos]->point)) // Same point cannot be added to octree
	{
		cout << "Point already exist in the tree" << endl;
		return;
	}
	else 
	{
		// assert(children[pos]->isLeaf == true);
		Vector3f *pointTemp = children[pos]->point;

		delete children[pos];
		children[pos] = nullptr;
		
		Vector3f topLeftFrontTemp, bottomRightBackTemp;
		if (pos == TopLeftFront) 
		{
			topLeftFrontTemp = Vector3f(topLeftFront[0], topLeftFront[1], topLeftFront[2]);
			bottomRightBackTemp = Vector3f(mid[0], mid[1], mid[2]);
		}
		else if (pos == TopRightFront) 
		{
			topLeftFrontTemp = Vector3f(mid[0] + ep, topLeftFront[1], topLeftFront[2]);
			bottomRightBackTemp = Vector3f(bottomRightBack[0], mid[1], mid[2]);
		}
		else if (pos == BottomRightFront) 
		{
			topLeftFrontTemp = Vector3f(mid[0] + ep, mid[1] + ep, topLeftFront[2]);
			bottomRightBackTemp = Vector3f(bottomRightBack[0], bottomRightBack[1], mid[2]);
		}
		else if (pos == BottomLeftFront) 
		{
			topLeftFrontTemp = Vector3f(topLeftFront[0], mid[1] + ep, topLeftFront[2]);
			bottomRightBackTemp = Vector3f(mid[0], bottomRightBack[1], mid[2]);
		}
		else if (pos == TopLeftBack) 
		{
			topLeftFrontTemp = Vector3f(topLeftFront[0], topLeftFront[1], mid[2] + ep);
			bottomRightBackTemp = Vector3f(mid[0], mid[1], bottomRightBack[2]);
		}
		else if (pos == TopRightBack) 
		{
			topLeftFrontTemp = Vector3f(mid[0] + ep, topLeftFront[1], mid[2] + ep);
			bottomRightBackTemp = Vector3f(bottomRightBack[0], mid[1], bottomRightBack[2]);
		}
		else if (pos == BottomRightBack) 
		{
			topLeftFrontTemp = Vector3f(mid[0] + ep, mid[1] + ep, mid[2] + ep);
			bottomRightBackTemp = Vector3f(bottomRightBack[0], bottomRightBack[1], bottomRightBack[2]);
		}
		else if (pos == BottomLeftBack) 
		{
			topLeftFrontTemp = Vector3f(topLeftFront[0], mid[1] + ep, mid[2] + ep);
			bottomRightBackTemp = Vector3f(mid[0], bottomRightBack[1], bottomRightBack[2]);
		}
		
		children[pos] = new OctreeEigen(topLeftFrontTemp, bottomRightBackTemp);
		children[pos]->insert(pointTemp);
		children[pos]->insert(point);
	}
}

// Function that returns true if the point
// (x, y, z) exists in the octree
bool OctreeEigen::find(const Vector3f &point)
{
	// If the point is out of bounds
	if (point[0] < topLeftFront[0] 
		|| point[0] > bottomRightBack[0] 
		|| point[1] < topLeftFront[1] 
		|| point[1] > bottomRightBack[1] 
		|| point[2] < topLeftFront[2] 
		|| point[2] > bottomRightBack[2])
		return false;

	// Otherwise perform binary search for each ordinate
	Vector3f midTemp = (topLeftFront + bottomRightBack) / 2.0;

	int pos = -1;

	// Deciding the position where to move
	if (point[0] <= midTemp[0]) 
	{
		if (point[1] <= midTemp[1]) 
		{
			if (point[2] <= midTemp[2])
				pos = TopLeftFront;
			else
				pos = TopLeftBack;
		}
		else 
		{
			if (point[2] <= midTemp[2])
				pos = BottomLeftFront;
			else
				pos = BottomLeftBack;
		}
	}
	else 
	{
		if (point[1] <= midTemp[1]) 
		{
			if (point[2] <= midTemp[2])
				pos = TopRightFront;
			else
				pos = TopRightBack;
		}
		else 
		{
			if (point[2] <= midTemp[2])
				pos = BottomRightFront;
			else
				pos = BottomRightBack;
		}
	}

	// If an internal node is encountered
	if (children[pos]->isLeaf == false) 
		return children[pos]->find(point);

	// If an empty node is encountered
	else if (children[pos]->point == nullptr)
		return false;
	else 
	{
		// If node is found with the given value
		if (point == *(children[pos]->point))
			return true;
	}
	return false;
}

int OctreeEigen::calcHeight()
{
	if (isLeaf)
		return 0;

	int maxHeight = 0;
	for (int i = 0; i < 8; i++)
	{
		assert(children[i] != nullptr);
		int height = children[i]->calcHeight();
		if (height > maxHeight)
			maxHeight = height;
	}
	return maxHeight + 1;
}

OctreeAABBCull::OctreeAABBCull() : isLeaf (true), aabb_pos (-1), aabb (nullptr), point (nullptr) {}

OctreeAABBCull::~OctreeAABBCull() 
{
	// Do not delete the point as it points to orginal point cloud data structure
	point = nullptr; 

	delete aabb;
	aabb = nullptr;
	
	for (int i = 0; i < children.size(); i++)
	{
		delete children[i];
		children[i] = nullptr;
	}
}

OctreeAABBCull::OctreeAABBCull(const Vector4f &min, const Vector4f &max, FrustumClip &frustum)
{
	// This use to construct Octree
	// with boundaries defined
	if (max[0] < min[0] || max[1] < min[1] || max[2] < min[2])
	{
		cout << "boundary points are not valid" << endl;
		return;
	}

	isLeaf = false;

	// Calculate the mid point of the cube
	aabb = new AABB((min + max) / 2.0, (max - min) / 2.0);

	// aabb->print();

	// Check if the AABB is inside/outside/intersecting the frustum
	aabb_pos = frustum.aabbInFrustum(*aabb);

	// Assigning null to the children
	children.assign(N_AABB_VERTEX, nullptr);
	for (int i = TopLeftFront; i <= BottomLeftBack; i++)
		children[i] = new OctreeAABBCull(); // empty children
}

/**
 * @brief Checks where the current AABB is wrt frustum.
 * The position of the point is same as that of the current AABB. 
 * If AABB is inside/outside frustum, return from the function. Else, divide the AABB into further levels for testing. 
 * @param point 
 * @return int : OUTSIDE = 0, INTERSECT = 1, INSIDE = 2, ERROR = -1 */
int8_t OctreeAABBCull::culling(Vector3f *point, FrustumClip &frustum)
{
	if(aabb_pos == OUTSIDE || aabb_pos == INSIDE)
		return aabb_pos;

	Vector3f point_ = *point;
	Vector4f min = aabb->getMin();
	Vector4f max = aabb->getMax();
	
	// If the point is out of bounds
	/**
	 * RAJRUP: Numerical approx error in the following condition.
	 * TODO: A hacky fix for now. Need an elegant solution later. Also fix the find function.
	 */
	if (point_[0] < min[0] - delta
		|| point_[0] > max[0] + delta
		|| point_[1] < min[1] - delta
		|| point_[1] > max[1] + delta
		|| point_[2] < min[2] - delta
		|| point_[2] > max[2] + delta)
	{
		// cout << "Point is out of bounds" << endl;
		return UNASSIGNED;
	}

	// Binary search to insert the point
	Vector4f mid = aabb->mid;

	int pos = -1;

	// Checking the octant of the point
	if (point_[0] <= mid[0]) 
	{
		if (point_[1] <= mid[1]) 
		{
			if (point_[2] <= mid[2])
				pos = BottomLeftBack;
			else
				pos = BottomLeftFront;
		}
		else 
		{
			if (point_[2] <= mid[2])
				pos = TopLeftBack;
			else
				pos = TopLeftFront;
		}
	}
	else 
	{
		if (point_[1] <= mid[1]) 
		{
			if (point_[2] <= mid[2])
				pos = BottomRightBack;
			else
				pos = BottomRightFront;
		}
		else 
		{
			if (point_[2] <= mid[2])
				pos = TopRightBack;
			else
				pos = TopRightFront;
		}
	}

	// If an internal node is encountered
	if (children[pos]->isLeaf == false) 
	{
		return children[pos]->culling(point, frustum);
		// return children[pos]->aabb_pos;
	}
	// If an empty child node is encountered, it must be a leaf node
	else if (children[pos]->point == nullptr) 
	{
		// assert(children[pos]->isLeaf == true);
		children[pos]->point = point;
		return aabb_pos;
	}
	else if(*point == *(children[pos]->point)) // Same point cannot be added to octree
	{
		cout << "Point already exist in the tree" << endl;
		return UNASSIGNED;
	}
	else 
	{
		// assert(children[pos]->isLeaf == true);
		Vector3f *pointTemp = children[pos]->point;

		delete children[pos];
		children[pos] = nullptr;
		
		Vector4f minTemp, maxTemp;

		switch (pos)
		{
			case TopLeftFront:
				minTemp = Vector4f(min[0], mid[1], mid[2], 1.0f);
				maxTemp = Vector4f(mid[0], max[1], max[2], 1.0f);
				break;
			case TopRightFront:
				minTemp = Vector4f(mid[0], mid[1], mid[2], 1.0f);
				maxTemp = Vector4f(max[0], max[1], max[2], 1.0f);
				break;
			case BottomRightFront:
				minTemp = Vector4f(mid[0], min[1], mid[2], 1.0f);
				maxTemp = Vector4f(max[0], mid[1], max[2], 1.0f);
				break;
			case BottomLeftFront:
				minTemp = Vector4f(min[0], min[1], mid[2], 1.0f);
				maxTemp = Vector4f(mid[0], mid[1], max[2], 1.0f);
				break;
			case TopLeftBack:
				minTemp = Vector4f(min[0], mid[1], min[2], 1.0f);
				maxTemp = Vector4f(mid[0], max[1], mid[2], 1.0f);
				break;
			case TopRightBack:
				minTemp = Vector4f(mid[0], mid[1], min[2], 1.0f);
				maxTemp = Vector4f(max[0], max[1], mid[2], 1.0f);
				break;
			case BottomRightBack:
				minTemp = Vector4f(mid[0], min[1], min[2], 1.0f);
				maxTemp = Vector4f(max[0], mid[1], mid[2], 1.0f);
				break;
			case BottomLeftBack:
				minTemp = Vector4f(min[0], min[1], min[2], 1.0f);
				maxTemp = Vector4f(mid[0], mid[1], mid[2], 1.0f);
				break;
			default:
				cout << "Error in the octree" << endl;
				return aabb_pos;
		}
		
		children[pos] = new OctreeAABBCull(minTemp, maxTemp, frustum);
		children[pos]->culling(pointTemp, frustum);
		return children[pos]->culling(point, frustum);
	}
	return aabb_pos;
}

// Function that returns true if the point
// (x, y, z) exists in the octree
bool OctreeAABBCull::find(const Vector3f &point)
{
	Vector4f min = aabb->getMin();
	Vector4f max = aabb->getMax();
	
	// If the point is out of bounds
	if (point[0] < min[0] - delta
		|| point[0] > max[0] + delta
		|| point[1] < min[1] - delta
		|| point[1] > max[1] + delta
		|| point[2] < min[2] - delta
		|| point[2] > max[2] + delta)
		return false;

	// Otherwise perform binary search for each ordinate
	Vector4f mid = aabb->mid;

	int pos = -1;

	// Deciding the position where to move
	if (point[0] <= mid[0]) 
	{
		if (point[1] <= mid[1]) 
		{
			if (point[2] <= mid[2])
				pos = BottomLeftBack;
			else
				pos = BottomLeftFront;
		}
		else 
		{
			if (point[2] <= mid[2])
				pos = TopLeftBack;
			else
				pos = TopLeftFront;
		}
	}
	else 
	{
		if (point[1] <= mid[1]) 
		{
			if (point[2] <= mid[2])
				pos = BottomRightBack;
			else
				pos = BottomRightFront;
		}
		else 
		{
			if (point[2] <= mid[2])
				pos = TopRightBack;
			else
				pos = TopRightFront;
		}
	}

	// If an internal node is encountered
	if (children[pos]->isLeaf == false) 
		return children[pos]->find(point);

	// If an empty node is encountered
	else if (children[pos]->point == nullptr)
		return false;
	else 
	{
		// If node is found with the given value
		if (point == *(children[pos]->point))
			return true;
	}
	return false;
}


int OctreeAABBCull::calcHeight()
{
	if (isLeaf)
		return 0;

	int maxHeight = 0;
	for (int i = TopLeftFront; i <= BottomLeftBack; i++)
	{
		assert(children[i] != nullptr);
		int height = children[i]->calcHeight();
		if (height > maxHeight)
			maxHeight = height;
	}
	return maxHeight + 1;
}
