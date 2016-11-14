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


#ifndef MRL_POINT2O_H
#define MRL_POINT2O_H

#include "vector2d.h"
#include "angle.h"
#include "../object.h"


namespace mr 
{


/**
	Beware that the operators + - * uses also the "theta" component.

*/

class Point2o : public Object {

	 DECLARE_MRL_OBJECT(Point2o)

	///Text stream serializers (files)
	friend ostream& operator << (ostream& os,const Point2o& scan);
	
public:
	Angle theta;
	double x;
	double y;

	//Basic useful accessors
	/// Generates a point2 from the x,y components. 
	Vector2D position() const { return Vector2D(x,y); }
	double angle() const {return theta.getValue();}
	
/// @name Constructors
//@{
	Point2o(double xv=0, double yv=0, double th=0) : x(xv), y(yv), theta(th) {setRGB(1,0,0); drawGLMode=0; drawGLSize=0.3;}
	Point2o(double xv, double yv, const Angle& th) :  x(xv), y(yv), theta(th) {setRGB(1,0,0); drawGLMode=0; drawGLSize=0.3;}
	explicit Point2o(const Vector2D& p, double th=0) : x(p.x), y(p.y), theta(th) {setRGB(1,0,0); drawGLMode=0; drawGLSize=0.3;}
	Point2o(const Vector2D& p, const Angle& th) : x(p.x), y(p.y), theta(th) {setRGB(1,0,0); drawGLMode=0; drawGLSize=0.3;}
//@}
	
/// @name Operators
//@{
	/// Subtracts two vectors
	Point2o  operator- (const Point2o&  p) const { 
		return Point2o(x-p.x,y-p.y,theta-p.theta); 
	}
	/// Adds two vectors 
	Point2o operator+ (const Point2o&  p) const { 
		return Point2o(x+p.x,y+p.y,theta+p.theta); 
	}

	//Composition operator
	Point2o operator* (const Point2o& p) const
	{	
		Point2o pos;
		pos.x=		x	+	p.x*cos(theta)-p.y*sin(theta);
		pos.y=		y	+	p.x*sin(theta)+p.y*cos(theta);
		pos.theta=	theta+	p.theta; 	
		return pos;
	}
	///Unary Inversion operator
	Point2o operator-() const
	{
		Point2o pos;
		pos.x= -x*cos(theta)-y*sin(theta);
		pos.y= x*sin(theta)-y*cos(theta);
		pos.theta= Angle(0)-theta;
		return pos;
	}
	///****************ALBERTO PARA TI*****************************************
	/// Note: USE algebra of transformations for change of bases
	///For example, changing a point from local reference to global is just
	///Vector2D local;
	///Point2o reference;
	///Vector2D global=reference*local;
	///
	///For example, changing a point from global reference to local is 
	///Vector2D global;
	///Point2o reference;
	///Vector2D local= (-reference)*global;

	Vector2D operator* (const Vector2D& p) const
	{	
		Vector2D pos;
		pos.x=		x	+	p.x*cos(theta)-p.y*sin(theta);
		pos.y=		y	+	p.x*sin(theta)+p.y*cos(theta);	
		return pos;
	}

	int drawGLMode; // 0 for arrows; 1 for squares
	double drawGLSize; // Dimension of the arrow in m

	void drawGL2D() const;



	/// Rotate this vector of @param thetar radians around point @param p
/*	Point2o rotate(double thetar, const Vector2D& p = Vector2D(0, 0)) const 
	{
		double x = this->x, y = this->y;
		x -= p.x;
		y -= p.y;
		double newx = (double) x  * cos(thetar) - y * sin(thetar);
		double newy = (double) x  * sin(thetar) + y * cos(thetar);
		double newt = theta + thetar;
		newx += p.x;
		newy += p.y;
		return Point2o((double) newx, (double) newy, newt);
	}

	/// Change the reference system of the point, @param localReferenceSystem is the origin
	/// of the local reference system in global coordinates (theta points towards x axis)
	Point2o toLocalReferenceSystem(const Point2o& localReferenceSystem) const
	{
		Vector2D tempp(this->x - localReferenceSystem.x, this->y - localReferenceSystem.y);
		tempp = tempp.rot(-localReferenceSystem.theta);
		return Point2o(tempp.x, tempp.y, this->theta - localReferenceSystem.theta);
	}

	/// Change back to global reference system, @param localReferenceSystem is the origin
	/// of the local reference system in global coordinates (theta points towards x axis)
	Point2o toGlobalReferenceSystem(const Point2o& localReferenceSystem) const
	{
		Vector2D tempp(this->x, this->y);
		tempp = tempp.rot(localReferenceSystem.theta);
		tempp.x = tempp.x + localReferenceSystem.x;
		tempp.y = tempp.y + localReferenceSystem.y;
		return Point2o(tempp.x, tempp.y, this->theta + localReferenceSystem.theta);
	}
	
	

	/*inline Point2o weightedMean(const Point2o& p1, const Point2o& p2, double alpha)
	{
		double beta=1-alpha;
		double x = (double)(beta*p1.x+alpha*p2.x);
		double y = (double)(beta*p1.y+alpha*p2.y);
		double theta = angleWeightedMean(p1.theta,p2.theta,alpha);
		return Point2o(x,y,theta);
	}*/

//@}

};

typedef Point2o Pose;
typedef Vector2D Point2;


inline double mrl_distance(const Point2o& point1, const Point2o& point2)
{
	Point2o aux = point2 - point1;
	Vector2D vector(aux.x, aux.y);
	return vector.module();
}

inline double mrl_distance(const Point2& point1, const Point2& point2)
{
	Vector2D vector = point2 - point1;
	return vector.module();
}

/*inline Point2 toLocalReferenceSystem(const Point2& point, const Point2o& localReferenceSystem)
{
	Point2 tempp(point.x - localReferenceSystem.x, point.y - localReferenceSystem.y);
	tempp = tempp.rot(-localReferenceSystem.theta);
	return tempp;
}

inline Point2o toLocalReferenceSystem(const Point2o& point, const Point2o& localReferenceSystem)
{
	Point2 tempp(point.x - localReferenceSystem.x, point.y - localReferenceSystem.y);
	tempp = tempp.rot(-localReferenceSystem.theta);
	return Point2o(tempp.x, tempp.y, point.theta - localReferenceSystem.theta);
}

inline Point2 toGlobalReferenceSystem(const Point2& point, const Point2o& localReferenceSystem)
{
	Point2 tempp(point.x, point.y);
	tempp = tempp.rot(localReferenceSystem.theta);
	tempp.x = tempp.x + localReferenceSystem.x;
	tempp.y = tempp.y + localReferenceSystem.y;
	return tempp;
}

inline Point2o toGlobalReferenceSystem(const Point2o& point, const Point2o& localReferenceSystem)
{
	Point2 tempp(point.x, point.y);
	tempp = tempp.rot(localReferenceSystem.theta);
	tempp.x = tempp.x + localReferenceSystem.x;
	tempp.y = tempp.y + localReferenceSystem.y;
	return Point2o(tempp.x, tempp.y, point.theta + localReferenceSystem.theta);
}*/



}


 // ns

#endif



