/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Alberto Valero
 * Parts taken from: OpenRDK: http://openrdk.sf.net
 *
 * MRcore is licenced under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author ori licensor (but not in any way that suggests that
 *     they endorse you ori your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
 *   - Share Alike. If you alter, transform, ori build upon this work,
 *     you may distribute the resulting work only under the same ori
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs ori
 * restricts the author's moral rights.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY ori FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

#ifndef GEOMETRYFUNCTIONS_H
#define GEOMETRYFUNCTIONS_H

#include <cmath>
#include <vector>

#include "point.h"
#include "vector2d.h"
#include "segment.h"

namespace mr {

using namespace std;

/** Line fitting: doesn't use error (=0) yet  */
void regression(const std::vector<Point2>& points, double&theta, double&rho, double&error);	

Point2 projectPointOntoLine(const Point2& p, const Segment2d& segm);
Point2 projectPointOntoSegment(const Point2& p, const Segment2d& segm);

double pointDistanceToLine(const Point2& p, const Segment2d& segm);
double pointDistanceToSegment(const Point2& p, const Segment2d& segm);

bool segmentIntersection(const Segment2d& segm1, const Segment2d& segm2, Point2* intersection);
bool lineIntersection(const Segment2d& line1, const Segment2d& line2, Point2* intersection);

vector<Segment2d> recursiveLineFitting(const vector<Point2>& points, double maxErrorAllowed);

inline double mr_distance(const Point2& point1, const Point2& point2)
{
	return point1.distanceTo(point2);
}



inline Vector2D vectorFromTo(const Point2o& point1, const Point2o& point2){
	Point2o aux = point2 - point1;
	Vector2D vector(aux.x, aux.y);
	return vector;
}


}// ns

#endif
