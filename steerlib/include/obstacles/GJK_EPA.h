/*!
*
* \author VaHiD AzIzI
*
*/


#ifndef GJK_EPA_H_
#define GJK_EPA_H_


#include "util/Geometry.h"


#include <vector>


namespace SteerLib
{

    class STEERLIB_API GJK_EPA
    {
        public:
			GJK_EPA();

            /*
             *
             *  DO NOT CHANGE THE FUNCTION DEFINITION FOR intersect()
             *  -----------------------------------------------------
             *  Return type: 
             *  If there is a collision detected by the GJK algorithm the return value of intersect is TRUE, else FALSE
             *  -----------------------------------------------------
             *  Arguments:
             *  float& return_penetration_depth
             *  If there is a collision, this variable needs to be updated to the penetration depth calculated by the EPA algorithm
             *  TODO: YOU NEED TO POPULATE THIS VARIABLE
             *  
             *  Util::Vector& return_penetration_vector
             *  If there is a collision, this variable needs to be updated to the penetration vector calculated by the EPA algorithm
             *  TODO: YOU NEED TO POPULATE THIS VARIABLE
             *  
             *  NOTE: The above arguments are passed by reference, thereby their value change is reflected in the calling function.
             *  THIS IS CALLED PASS BY REFERENCE.
             *  This is way for functions to "return" multiple values
             *  ie. 
             *  funct()
             *  {
             *      return (boolean, val)
             *  }
             *  
             *  can be implemented as:
             *  
             *  boolean funct(ref& value)
             *  {
             *      value = val;
             *  }
             *  
             *  const std::vector<Util::Vector>& _shapeA
             *  The first input shape passed by the collisionAI. Every point p(x,y,z) is a "Util::Vector p(x,y,z);"
             *  std::vector<point> gives the list of points in the polygon as "std::vector<Util::Vector>" 
             *  NOTE: ALL THE POINTS LIE ON THE XZ PLANE, ie. y=0 for all p. So all Polygon = { p1(x1,0,z2), p2(x2,0,z2), p3(x3,0,z3) ...}
             *  where Polygon is the std::vector and p1..pn are Util::Vector of (x,0,z)
             *  
             *  const std::vector<Util::Vector>& _shapeB
             *  The second input shape passed by the collisionAI. 
             *  ----------------------------------------------------
             *  Algorithms:
             *  Implement the GJK and EPA Algorithms in 2D since for all points (x,0,z), y=0.
             *  ----------------------------------------------------
             *  Pseudocode:
             *  GJK(Polygon A, Polygon B)
             *  {
             *      if (A collides with B)
             *          return (Simplex, true)
             *      else
             *          return (NULL, false)
             *  }
             *  
             *  EPA(Polygon A, Polygon B, Simplex S)
             *  {
             *      Calculate penetration
             *      return (penetration_depth, penetration_vector)
             *  }
             *  
             *  intersect(Polygon A, Polygon B)
             *  {
             *      (Simplex, is_colliding) = GJK(A, B)
             *      if ( is_colliding == true)
             *      {
             *          (penetration_depth, penetration_vector) = EPA(A, B, Simplex)
             *          return (true, penetration_depth, penetration_vector)
             *      }
             *      else
             *      {
             *          return (false, 0, NULL)    
             *      }    
             *  }
             *  ----------------------------------------------------
             *  Implementation:
             *  Write your own helper functions, as long as you do not change the function definition of intersect.
             *  ONLY MODIFY THIS CLASS.
             *  DO NOT LEAVE ANY DEBUG PRINTS IN YOUR CODE IN THIS CLASS.
             *  ----------------------------------------------------
             *  Testing:
             *  In order to test your code, execute the command, inside /build/bin
             *  ./steersim -testcase polygons_test -ai collisionAI
             *  
             *  The output you see, should be:
             *  
             *  $ ./steersim -testcase polygons_test -ai collisionAI
             *   
             *  loaded module collisionAI
             *  loaded module testCasePlayer
             *  Initializing...
             *  Preprocessing...
             *   Collision detected between polygon No.0 and No.1 with a penetration depth of 2 and penetration vector of (-0,0,-1)
             *  Simulation is running...
             *  ----------------------------------------------------
             *  Additional Notes:
             *  Type of vertices are Vector which you can check Vector class in util/Geometry.h to get familier with overloaded operators
             *  intersection will be called for all polygons, for all unique pairs, ie. if we have 3 polygons it is called 3 times
             *  one between polygon No. 1 and No. 2
             *  one between polygon No. 1 and No. 3
             *  one between polygon No. 2 and No. 3
             *  For grading your results will be compared by our results with testcase polygons1, so please run your code with
             *  original version of polygon1
             *  DO NOT MODIFY polygon1.xml
             */
            static bool intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB);

			static bool intersectConvex(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB);

            static bool intersectConcave(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB);
        private:

			/**
			 * Returns true if the given shape is convex.
			 */
			static bool isConvex(const std::vector<Util::Vector>& shape);

			/**
			 * This runs the GJK algorithm and determines if there is an intersection between two shapes.
			 * If there is, it will return true. In either case, the vector passed in will
			 * be filled with all of the points found to be on the simplex.
			 *
			 * PARAM shapeA: a vector containing the points that define the first shape
			 * PARAM shapeB: a vector containing the points that define the second shape
			 * PARAM simplex: an empty vector in which the last three points from the simplex will be stored.
			 *
			 * Returns true if there is an intersection between shapeA and shapeB, false otherwise
			 */
			static bool gjk(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, std::vector<Util::Vector>& simplex);
			
			/**
			 * Checks if the simplex contains the origin, and updates the direction if it does not.
			 *
			 * PARAM simplex: A vector containing the points that define the current simplex, up to 3 points large.
			 * PARAM direction: A vector describing the direction in which we searched for the last point added to the simplex.
			 *
			 * Returns true if the simplex contains the origin, false otherwise. Updates the direction to be a vector perpendicular
			 * to the edge closest to the origin, in the direction of the origin.
			 */
			static bool simplexContainsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& direction);

			/**
			 * This function finds a point on the simplex using the given direction.
			 *
			 * PARAM shapeA: a vector containing the points that define the first shape.
			 * PARAM shapeB: a vector containing the points that define the second shape.
			 * PARAM direction: a vector that represents the direction in which we are searching.
			 *					The y-value (index 1) should be 0 since this is on an XZ plane.
			 *
			 * Returns a point from the simplex furthest in the given direction.
			 */
			static Util::Vector getSimplexPointUsingDirection(const std::vector<Util::Vector>& shapeA, const std::vector<Util::Vector>& shapeB, const Util::Vector& direction);

			/**
			 * This function gets the center of a given shape.
			 * PARAM shape: a vector containing the points that define the shape.
			 * Returns a point representing the center of the shape.
			 */
			static Util::Vector getShapeCenter(const std::vector<Util::Vector>& shape);

			/**
			 * This function finds the point furthest along a polygon in a given direction.
			 *
			 * PARAM shape: a vector containing the points that define the shape.
			 * PARAM direction: A vector that represents the direction in which we are searching.
			 *					The y-value (index 1) should be 0 since this is on an XZ plane.
			 * 
			 * Returns the point furthest along a polygon in the given direction.
			 */
			static Util::Vector getFurthestPointInDirection(const std::vector<Util::Vector>& shape, const Util::Vector& direction);

			/**
			 * Calculates the dot product of two vectors.
			 * PARAM vectorA: the first vector
			 * PARAM vectorB: the second vector
			 * Returns the dot product of the two vectors.
			 */
			static float dot(const Util::Vector& vectorA, const Util::Vector& vectorB);


                  /**
                   * Runs EPA algorithm on two shapes that we already know to be colliding. 
                   *
                   * PARAM _shapeA: a vector containing the points that define the first shape.
                   * PARAM _shapeB: a vector containing the points that define the second shape.
                   * PARAM simplex: the initialized simplex from gjk()
                   * PARAM return_penetration_depth: calculated depth is stored here. 
                   * PARAM return_penetration_vector: calculated penetration vector is stored here. 
                   *
                   */
                  static void epa(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex, float& return_penetration_depth, Util::Vector& return_penetration_vector);

                  /**
                   * TODO(vivek): 
                   *
                   */
                  static void findClosestEdge(std::vector<Util::Vector> simplex, float& distance, Util::Vector& normal, int& index);

                  /**
                   * TODO(vivek): 
                   *
                   */
                  static Util::Vector tripleProduct(Util::Vector A, Util::Vector B, Util::Vector C);

                  /**
                   * Triangulates a given simple polygon.
                   * PARAM shape: the shape to triangulate
                   * PARAM triangles: vector holding the computed triangles (as a series of vertices).
                   */
                  static bool triangulatePolygon(const std::vector<Util::Vector>& shape, std::vector<Util::Vector>& triangles);

    }; // class GJK_EPA

} // namespace SteerLib

#endif /* GJK_EPA_H_ */
