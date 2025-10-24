// Implementation of Octree in c++
#include <iostream>
#include "octree.h"

// Driver code
int main()
{
	// -----------------------------------------------------
	Octree tree(1.0, 1.0, 1.0, 5.0, 5.0, 5.0);

	tree.insert(1.0, 2.0, 3.0);
	tree.insert(1.0, 2.0, 3.0);
	tree.insert(6.0, 5.0, 5.0);

	cout << (tree.find(1.0, 2.0, 3.0)
				? "Found\n"
				: "Not Found\n");

	cout << (tree.find(3.0, 4.0, 4.0)
				? "Found\n"
				: "Not Found\n");
	tree.insert(3.0, 4.0, 4.0);

	cout << (tree.find(3.0, 4.0, 4.0)
				? "Found\n"
				: "Not Found\n");

	// -----------------------------------------------------
	// Vector3f p1(1.0, 1.0, 1.0);
	// Vector3f p2(5.0, 5.0, 5.0);
	// OctreeEigen *treeEigen =  new OctreeEigen(p1, p2);

	// Vector3f *point = new Vector3f(1.0, 2.0, 3.0);
	// treeEigen->insert(point);

	// point = new Vector3f(1.0, 2.0, 3.0);
	// treeEigen->insert(point);

	// point = new Vector3f(6.0, 5.0, 5.0);
	// treeEigen->insert(point);
	
	// point = new Vector3f(1.0, 2.0, 3.0);
	// cout << (treeEigen->find(*point)
	// 			? "Found\n"
	// 			: "Not Found\n");

	// point = new Vector3f(3.0, 4.0, 4.0);
	// cout << (treeEigen->find(*point)
	// 			? "Found\n"
	// 			: "Not Found\n");

	// treeEigen->insert(point);

	// cout << (treeEigen->find(*point)
	// 			? "Found\n"
	// 			: "Not Found\n");
	// delete treeEigen;

	// -----------------------------------------------------
	Vector4f min(1.0, 1.0, 1.0, 1.0);
	Vector4f max(5.0, 5.0, 5.0, 1.0);
	FrustumClip frustum;
	OctreeAABBCull *treeAABB = new OctreeAABBCull(min, max, frustum);

	Vector3f *point = new Vector3f(1.0, 2.0, 3.0);
	treeAABB->culling(point, frustum);

	point = new Vector3f(1.0, 2.0, 3.0);
	treeAABB->culling(point, frustum);

	point = new Vector3f(6.0, 5.0, 5.0);
	treeAABB->culling(point, frustum);
	
	point = new Vector3f(1.0, 2.0, 3.0);
	cout << (treeAABB->find(*point)
				? "Found\n"
				: "Not Found\n");

	point = new Vector3f(3.0, 4.0, 4.0);
	cout << (treeAABB->find(*point)
				? "Found\n"
				: "Not Found\n");

	treeAABB->culling(point, frustum);

	cout << (treeAABB->find(*point)
				? "Found\n"
				: "Not Found\n");
	delete treeAABB;


	return 0;
}