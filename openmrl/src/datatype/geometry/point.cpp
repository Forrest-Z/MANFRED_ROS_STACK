#include "point.h"
#include <math.h>
#include <fstream>
#include <string>

#include <GL/glut.h>
namespace mr{

IMPLEMENT_MRL_OBJECT(Point2o)

ostream& operator << (ostream& os,const Pose& scan)
{
	os<<scan.x<<" "<<scan.y<<" "<<scan.theta<<" ";
	return os;
}

void Point2o::drawGL2D() const{
	 glPushMatrix();
	 glColor3f(rgb[0],rgb[1],rgb[2]);
	 
	 switch(drawGLMode){
		case(0): // Arrow
			{
				
			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glPushMatrix();
			
			glBegin(GL_TRIANGLES);
			
			// The three vertex of the arrow
			Vector2D A (drawGLSize,0);
			Vector2D B (-drawGLSize,drawGLSize/2.0);
			Vector2D C (-drawGLSize,-drawGLSize/2.0);
			
			// Rotating the arrow
			A = A.rot(theta.getValue());
			B = B.rot(theta.getValue());
			C = C.rot(theta.getValue());
				
			// Translating the arrow and drawing
			glColor4f(rgb[0],rgb[1], rgb[2], 0.5);
			glVertex2f(A.x + x  ,  A.y + y);
			glVertex2f(B.x + x  ,  B.y + y);
			glVertex2f(C.x + x  ,  C.y + y);	
			
			glEnd();
			
			glPopMatrix();
			
			glDisable(GL_BLEND);
			
			break;
			}
			
		case(1): // Square
			glBegin(GL_QUADS);
			
			glVertex2f(x-(drawGLSize/2.0),y-(drawGLSize/2.0));
			glVertex2f(x+(drawGLSize/2.0),y-(drawGLSize/2.0));
			glVertex2f(x+(drawGLSize/2.0),y+(drawGLSize/2.0));
			glVertex2f(x-(drawGLSize/2.0),y+(drawGLSize/2.0));
			break;
			
			
		default:
			{
				
			glEnable (GL_BLEND);
			glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glPushMatrix();
			
			glBegin(GL_TRIANGLES);
			
			// The three vertex of the arrow
			Vector2D A (drawGLSize,0);
			Vector2D B (-drawGLSize,drawGLSize/2.0);
			Vector2D C (-drawGLSize,-drawGLSize/2.0);
			
			// Rotating the arrow
			A = A.rot(theta.getValue());
			B = B.rot(theta.getValue());
			C = C.rot(theta.getValue());
				
			// Translating the arrow and drawing
			glColor4f(rgb[0],rgb[1], rgb[2], 0.5);
			glVertex2f(A.x + x  ,  A.y + y);
			glVertex2f(B.x + x  ,  B.y + y);
			glVertex2f(C.x + x  ,  C.y + y);	
			
			glEnd();
			
			glPopMatrix();
			
			glDisable(GL_BLEND);
			
			break;
			}
	 }
	 
	 

	 glEnd();

	 glPopMatrix();
}

}// namespace
