//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"
#include <cmath>


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}

	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}
	
	int AStarPlanner::getIndexFromPoint(Util::Point p)
	{
		return gSpatialDatabase->getCellIndexFromLocation(p);
	}

	double AStarPlanner::heuristicEstimate(Util::Point start, Util::Point finish)
	{
		// Manhattan Distance
		// return std::abs(start.x - finish.x) + std::abs(start.y - finish.y);

		// Euler Distance
		return (start.vector() - finish.vector()).length();
	}

	double AStarPlanner::distanceBetween(Util::Point start, Util::Point finish)
	{
		// TODO(vivek): confirm that this is the correct function. 
		// return (start.vector() - finish.vector()).length();
		return 1.0;
	}

	std::set<int> AStarPlanner::getNeighborsForNodeIndex(SteerLib::GridDatabase2D * _gSpatialDatabase, int nodeIndex)
	{
		std::set<int> neighbors;

		unsigned int xIndex, zIndex;

		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(nodeIndex, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-1, 0);
		x_range_max = MIN(x+1, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-1, 0);
		z_range_max = MIN(z+1, gSpatialDatabase->getNumCellsZ());

		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				if (index != nodeIndex)
				{
					neighbors.insert(index);
				}
			}
		}

		return neighbors;
	}

	bool AStarPlanner::checkIfNodeIsOnGrid(SteerLib::GridDatabase2D * _gSpatialDatabase, int nodeIndex)
	{
		unsigned int xIndex, zIndex;
		_gSpatialDatabase->getGridCoordinatesFromIndex(nodeIndex, xIndex, zIndex);

		// since indices are unsigned, only need to check upper bound
		if (xIndex >= _gSpatialDatabase->getNumCellsX() || zIndex >= _gSpatialDatabase->getNumCellsZ())
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	// This method must not be called if openSet is empty
	int AStarPlanner::popFringeNode(std::set<int> &openSet, std::map<int, double> fScore, std::map<int, double> gScore)
	{
		double minFValue = DBL_MAX;
		double loGValue = DBL_MAX;
		int popCandidateIndex = -1;

		std::set<int>::iterator it;
		for (it = openSet.begin(); it != openSet.end(); ++it)
		{
		    int index = *it;
		    if (minFValue > fScore[index] || (minFValue == fScore[index] && loGValue > gScore[index]))
		    {
		    	minFValue = fScore[index];
				loGValue = gScore[index];
		    	popCandidateIndex = index;
		    }
		}

		openSet.erase(popCandidateIndex);
		
		return popCandidateIndex;
	}

	void AStarPlanner::reconstructPath(std::vector<Util::Point>& agent_path, std::map<int, int> cameFrom, int goalIndex)
	{
		int currentIndex = goalIndex;
		agent_path.push_back(getPointFromGridIndex(currentIndex));
		while (cameFrom.count(currentIndex))
		{
			currentIndex = cameFrom[currentIndex];
			agent_path.push_back(getPointFromGridIndex(currentIndex));
		}
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout<<"\nIn A*\n";

		// Setup
		std::set<int> closedSet;
		std::set<int> openSet;
		std::map<int, int> cameFrom;
		std::map<int, double> gScore;
		std::map<int, double> fScore;

		openSet.insert(getIndexFromPoint(start));
		gScore.insert(std::pair<int,double>(getIndexFromPoint(start), 0));
		fScore.insert(std::pair<int,double>(getIndexFromPoint(start), heuristicEstimate(start, goal)));

		while (!openSet.empty())
		{
			int currentIndex = popFringeNode(openSet, fScore, gScore);
			if (currentIndex == getIndexFromPoint(goal))
			{
				// currentIndex is the same as the goal index
				reconstructPath(agent_path, cameFrom, currentIndex);
				return true;
			}
			closedSet.insert(currentIndex);

			std::set<int> neighbors = getNeighborsForNodeIndex(gSpatialDatabase, currentIndex);
			std::set<int>::iterator neighborsIter;
			for (neighborsIter = neighbors.begin(); neighborsIter != neighbors.end(); ++neighborsIter)
			{
				int neighborIndex = *neighborsIter;
				
				if (closedSet.count(neighborIndex) == 1)
				{
					continue;
				}

				if (!checkIfNodeIsOnGrid(gSpatialDatabase, neighborIndex))
				{
					// Exclude nodes off edge of grid.
					continue;
				}
				if (!canBeTraversed(neighborIndex))
				{
					// Exclude nodes that are part of obstacles.
					continue;
				}

				// By this point neighborIndex is a valid node that can be traversed and has not yet been visisted.

				double tentativeGScore = gScore[currentIndex] + distanceBetween(getPointFromGridIndex(currentIndex), getPointFromGridIndex(neighborIndex));

				if (gScore.count(neighborIndex) == 0 || tentativeGScore < gScore[neighborIndex])
				{
					cameFrom[neighborIndex] = currentIndex;
					gScore[neighborIndex] = tentativeGScore;
					fScore[neighborIndex] = gScore[neighborIndex] + heuristicEstimate(getPointFromGridIndex(neighborIndex), goal);

					assert(currentIndex != neighborIndex);

					openSet.insert(neighborIndex);
				}
			}
		}

		return false;
	}
}
