/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"
#include "obstacles/triangulate.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

//Look at the GJK_EPA.h header file for documentation and instructions

// Solution based off of answer from http://stackoverflow.com/questions/471962/how-do-determine-if-a-polygon-is-complex-convex-nonconvex.
bool SteerLib::GJK_EPA::isConvex(const std::vector<Util::Vector>& shape) {
	if (shape.size() < 4) { // Assume nothing is collinear.
		return true;
	}
	bool sign = false;
	size_t numVertices = shape.size();
	for (size_t i = 0; i < numVertices; ++i) {
		const Util::Vector &a = shape[i];
		const Util::Vector &b = shape[(i + 1) % numVertices];
		const Util::Vector &c = shape[(i + 2) % numVertices];
		
		const float dx1 = a.x - b.x;
		const float dy1 = a.z - b.z;
		const float dx2 = c.x - b.x;
		const float dy2 = c.z - b.z;
		const float zcrossproduct = (dx1 * dy2) - (dy1 * dx2);
		if (i == 0) { // Set sign based on first pair.
			sign = (zcrossproduct > 0);
		} else if (sign != (zcrossproduct > 0)) { // Sign must match.
			return false;
		}
	}
	return true;
}

bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	if (isConvex(_shapeA) && isConvex(_shapeB)) {
		return intersectConvex(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB);
	}
	return intersectConcave(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB);
}

bool SteerLib::GJK_EPA::intersectConvex(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
    bool retval = gjk(_shapeA, _shapeB, simplex); 

    if (retval) {
    	// There is a collision
    	epa(_shapeA, _shapeB, simplex, return_penetration_depth, return_penetration_vector);
    } 

	return retval;
}

bool SteerLib::GJK_EPA::intersectConcave(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	std::vector<Util::Vector> trianglesA;
	std::vector<Util::Vector> trianglesB;
	triangulatePolygon(_shapeA, trianglesA);
	triangulatePolygon(_shapeB, trianglesB);
	Util::Vector maxPenetrationVec;
	float maxPenetrationDepth = 0;
	for (size_t indexA = 0; indexA < trianglesA.size(); indexA += 3) {
		std::vector<Util::Vector> triangleA(trianglesA.begin() + indexA, trianglesA.begin() + indexA + 3);

		for (size_t indexB = 0; indexB < trianglesB.size(); indexB += 3) {
			std::vector<Util::Vector> triangleB(trianglesB.begin() + indexB, trianglesB.begin() + indexB + 3);
			float penetration_depth;
			Util::Vector penetration_vector;
			if (intersectConvex(penetration_depth, penetration_vector, triangleA, triangleB) && penetration_depth > maxPenetrationDepth) {
				maxPenetrationDepth = penetration_depth;
				maxPenetrationVec = penetration_vector;
			}
		}
	}
	if (maxPenetrationDepth > 0) {
		return_penetration_depth = maxPenetrationDepth;
		return_penetration_vector = maxPenetrationVec;
		return true;
	}
	return false;
}

bool SteerLib::GJK_EPA::gjk(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, std::vector<Util::Vector>& simplex) {
	Util::Vector centerA = getShapeCenter(shapeA);
	Util::Vector centerB = getShapeCenter(shapeB);
	Util::Vector direction = centerB - centerA;
	simplex.push_back(getSimplexPointUsingDirection(shapeA, shapeB, direction));
	direction = -direction;
	while (true) {
		simplex.push_back(getSimplexPointUsingDirection(shapeA, shapeB, direction));
		if (dot(simplex.back(), direction) <= 0) return false;
		else if (simplexContainsOrigin(simplex, direction)) {
			if (simplex.size() < 3) {
				// The simplex will have 2 points in this case (2 points have already been added)
				simplex.push_back(getSimplexPointUsingDirection(shapeA, shapeB, direction));
			}
			return true;
		}
	}
}

bool SteerLib::GJK_EPA::simplexContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction) {
	Util::Vector ptA = simplex.back();
	Util::Vector aToOrigin = -ptA;
	if (simplex.size() == 3) {
		Util::Vector ptB = simplex[1];
		Util::Vector ptC = simplex[0];
		Util::Vector aToB = ptB - ptA;
		Util::Vector aToC = ptC - ptA;
		Util::Vector abPerp = aToB * dot(aToB, aToC) - aToC * (dot(aToB, aToB));
		Util::Vector acPerp = aToC * dot(aToC, aToB) - aToB * (dot(aToC, aToC));
		if (dot(abPerp, aToOrigin) > 0) {
			simplex.erase(simplex.begin());
			direction = abPerp;
		}
		else if (dot(acPerp, aToOrigin) > 0) {
			simplex.erase(simplex.begin() + 1);
			direction = acPerp;
		}
		else return true;
	}
	else {
		Util::Vector ptB = simplex.at(0);
		Util::Vector aToB = ptB - ptA;
		Util::Vector abPerp = aToOrigin * dot(aToB, aToB) - aToB * dot(aToB, aToOrigin);
		direction = abPerp;
		if (dot(abPerp, aToOrigin) == 0) {
			float aToBDotaToOrigin = dot(aToB, aToOrigin);
			if (aToBDotaToOrigin >= 0 && aToBDotaToOrigin < dot(aToB, aToB)) return true;
		}
	}
	return false;
}

// support function
Util::Vector SteerLib::GJK_EPA::getSimplexPointUsingDirection(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, const Util::Vector& direction) {
	Util::Vector retVal = getFurthestPointInDirection(shapeA, direction) - getFurthestPointInDirection(shapeB, -direction);
	return retVal;
}

Util::Vector SteerLib::GJK_EPA::getShapeCenter(const std::vector<Util::Vector>& shape) {
	Util::Vector retVal(0, 0, 0);
	for (int i = 0; i < shape.size(); i++) {
		Util::Vector p = shape[i];
		retVal[0] += p[0];
		retVal[2] += p[2];
	}
	retVal[0] = retVal[0] / (float)shape.size();
	retVal[2] = retVal[2] / (float)shape.size();
	return retVal;
}

Util::Vector SteerLib::GJK_EPA::getFurthestPointInDirection(const std::vector<Util::Vector>& shape, const Util::Vector& direction) {
	Util::Vector retVal(0, 0, 0);
	float farthestDistance = dot(shape[0], direction);
	int farthestIndex = 0;
	for (int i = 1; i < shape.size(); i++) {
		float dotProd = dot(shape[i], direction);
		if (dotProd > farthestDistance) {
			farthestDistance = dotProd;
			farthestIndex = i;
		}
	}
	retVal[0] = shape[farthestIndex][0];
	retVal[1] = shape[farthestIndex][1];
	retVal[2] = shape[farthestIndex][2];
	return retVal;
}

float SteerLib::GJK_EPA::dot(const Util::Vector& vectorA, const Util::Vector& vectorB) {
	float retVal = 0;
	for (int i = 0; i < 3; i++) {
		retVal += vectorA[i] * vectorB[i];
	}
	return retVal;
}

bool SteerLib::GJK_EPA::originLiesOnSimplex(std::vector<Util::Vector>& simplex, float& return_penetration_depth, Util::Vector& return_penetration_vector) {
	for (int i = 0; i < simplex.size(); i++) {
		int j = i + 1 == simplex.size() ? 0 : i + 1;
		Util::Vector a = simplex[i];
		Util::Vector b = simplex[j];

		if (edgeContainsOrigin(a, b)) {
			containedOriginPenetrationFromEdge(a, b, return_penetration_depth, return_penetration_vector);
			return true;
		}
	}
	return false;
}

void SteerLib::GJK_EPA::containedOriginPenetrationFromEdge(Util::Vector v1, Util::Vector v2, float& return_penetration_depth, Util::Vector& return_penetration_vector) {
	Util::Vector v1ToOrigin = -v1;
	Util::Vector v2ToOrigin = -v2;
	if (v1.lengthSquared() <= v2.lengthSquared()) {
		return_penetration_depth = v1.length();
		return_penetration_vector = v1;
	}
	else {
		return_penetration_depth = v2.length();
		return_penetration_vector = v2;
	}
	// Normalize.
	return_penetration_vector = return_penetration_vector / return_penetration_vector.norm();
}

bool SteerLib::GJK_EPA::edgeContainsOrigin(Util::Vector v1, Util::Vector v2) {
	const float THRESHOLD = .0001f;
	Util::Vector diff = v2 - v1;
	if (diff.x == 0) { // Vertical line. Check if the origin lies in between at x = 0.
		if (v1.x != 0) {
			return false;
		}
		float minY = v1.z;
		float maxY = v2.z;
		if (minY > maxY) {
			minY = v2.z;
			maxY = v1.z;
		}
		return (minY <= 0 && maxY >= 0);
	}
	float slope = diff.z / diff.x;

	Util::Vector toOrigin = -v1;
	if (toOrigin.x == 0) { // Vertical line to origin. Nope.
		return false;
	}
	float slopeToOrigin = toOrigin.z / toOrigin.x;
	if (abs(slopeToOrigin - slope) < THRESHOLD) {
		// The slopes match up - just need to make sure it's in range and on the right side.
		bool rightSideOfV1 = (sign(diff.x) == sign(toOrigin.x) && sign(diff.z) == sign(toOrigin.z));
		bool inRange = (toOrigin.lengthSquared() <= diff.lengthSquared());
		return rightSideOfV1 && inRange;
	}
	return false;
}


void SteerLib::GJK_EPA::epa(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex, float& return_penetration_depth, Util::Vector& return_penetration_vector)
{
	float TOLERANCE = 0.00001;

	while (true)
	{
		if (originLiesOnSimplex(simplex, return_penetration_depth, return_penetration_vector)) {
			return;
		}
		float distance;
		Util::Vector normal;
		int index;
		findClosestEdge(simplex, distance, normal, index);

		Util::Vector supportPoint = getSimplexPointUsingDirection(_shapeA, _shapeB, normal);

		double d = supportPoint * normal;
		if (d - distance < TOLERANCE) {
			return_penetration_vector = normal;
			return_penetration_depth = d;
			return;
		} else {
			simplex.insert(simplex.begin()+index, supportPoint);
		}
	}
}

Util::Vector SteerLib::GJK_EPA::tripleProduct(Util::Vector A, Util::Vector B, Util::Vector C)
{
	return (B * (C * A)) - (A * (C * B));
}

void SteerLib::GJK_EPA::findClosestEdge(std::vector<Util::Vector> simplex, float& distance, Util::Vector& normal, int& index)
{
	distance = FLT_MAX;
	for (int i = 0; i < simplex.size(); i++) {
		int j = i + 1 == simplex.size() ? 0 : i + 1;
		Util::Vector a = simplex[i];
		Util::Vector b = simplex[j];
		Util::Vector e = b - a;
		Util::Vector oa = a;
		Util::Vector n = tripleProduct(e, oa, e);

		Util::Vector n_norm = n / n.norm();

		double d = n_norm * a;
		if (d < distance) {
			distance = d;
			normal = n_norm;
			index = j;
		}
	}
}

int SteerLib::GJK_EPA::sign(float f) {
	if (f > 0) return 1;
	return (f == 0) ? 0 : -1;
}

bool SteerLib::GJK_EPA::triangulatePolygon(const std::vector<Util::Vector>& shape, std::vector<Util::Vector>& triangles) {
	Vector2dVector shape2D;
	Vector2dVector triangles2D;
	for (const Util::Vector & vec : shape) {
		Vector2d vec2D(vec.x, vec.z);
		shape2D.push_back(vec2D);
	}
	if (Triangulate::Process(shape2D, triangles2D)) {
		for (const Vector2d & vec : triangles2D) {
			Util::Vector vec2(vec.GetX(), 0, vec.GetY());
			triangles.push_back(vec2);
		}
		return true;
	}
	return false;
}
