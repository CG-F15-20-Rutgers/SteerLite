//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	sort(controlPoints.begin(), controlPoints.end(), [](const CurvePoint a, const CurvePoint b) -> bool {
		return a.time < b.time;
	});

	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	return (controlPoints.size() >= ((type == catmullCurve)? 3 : 2));
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function findTimeInterval is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition, a, b, c, d;
	float normalTime, intervalTime, tSquared, tCubed, tCubedMinusTSquared, coeff, threeTSquaredMinusTwoTCubed;
	Util::Vector vector;

	// Calculate time interval, and normal time required for later curve calculations
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	// Calculate position at t = time on Hermite curve
	tSquared = normalTime * normalTime;
	tCubed = tSquared * normalTime;
	tCubedMinusTSquared = tCubed - tSquared;
	threeTSquaredMinusTwoTCubed = (3 * tSquared) - (2 * tCubed);

	coeff = 1 - threeTSquaredMinusTwoTCubed;
	a = controlPoints[nextPoint - 1].position * coeff;

	coeff = threeTSquaredMinusTwoTCubed;
	b = controlPoints[nextPoint].position * coeff;

	coeff = (tCubedMinusTSquared - tSquared + normalTime) * intervalTime;
	vector = controlPoints[nextPoint - 1].tangent * coeff;
	c = Point(vector[0], vector[1], vector[2]);

	coeff = (tCubedMinusTSquared) * intervalTime;
	vector = controlPoints[nextPoint].tangent * coeff;
	d = Point(vector[0], vector[1], vector[2]);


	newPosition = a + b + c + d;

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Catmull-Rom curve
	
	// Return result
	return newPosition;
}