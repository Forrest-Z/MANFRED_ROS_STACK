#include <datatype/gridmap_includes.h>
#include <datatype/geometry/path2d.h>
#include <GL/glut.h>

#include <map>
#include <vector>
#include <iostream>

using namespace std;
using namespace mr;

void clearScene(); //clear all allocated memory
void initializeScene(); //creates the world
void OnDraw(void); //it will be called when the screen needs to be redrawn
void OnTimer(int value); //it will be called with a timer
void OnKeyboardDown(unsigned char key, int x, int y); //when a key is pressed
void OnMouseMove(int x,int y); //when the mouse moves
void OnMouseClick(int button,int state, int x,int y); //when a button is pressed

map<string,Object*> scene;

int main(int argcp, char *argv[]){
	 atexit(clearScene);
	 glutInit(&argcp, argv);
	 glutInitWindowSize(800,600);
	 glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	 glutCreateWindow("GL");
	 glutDisplayFunc(OnDraw);
	 glutMotionFunc(OnMouseMove);
	 glutMouseFunc(OnMouseClick);
	 glutKeyboardFunc(OnKeyboardDown);

	 initializeScene();

	 glutTimerFunc(5,OnTimer,0);
	 glutMainLoop();
	 return 0;
}

void initializeScene(){
	 //THE MAP
	 LabeledGridMap* gridmap = new LabeledGridMap(80,60,0.1,LabeledGridMap::FREE,0,0);
	 Image* img = new Image(800,600);
	 img->load("../../data/map.bmp");
	 gridmap->loadFromImage(img,80,0.1,0,0);
	 scene["map"]=gridmap;

	 //TWO POINTS
	 Point2o* point = new Point2o(10,20,0);
	 scene["z0_point"]=point;
	 point = new Point2o(30,10,0);
	 scene["z1_point"]=point;

	 //A PATH
	 Path2D * path = new Path2D();
	 path->push_back(Point2(10,10));
	 path->push_back(Point2(20,10));
	 path->push_back(Point2(40,50));
	 path->push_back(Point2(20,32));

	 scene["z2_path"]=path;

}

void clearScene(){
	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  delete (it->second);
	 }
}

void OnDraw(void)
{

	 //Borrado de la pantalla
	 glClear(GL_COLOR_BUFFER_BIT );

	 //Para definir el punto de vista
	 glMatrixMode(GL_MODELVIEW);
	 glLoadIdentity();

	 double x0,y0,xf,yf;

	 // GET MAP DIMENSIONS
	 LabeledGridMap* gridmap = dynamic_cast<LabeledGridMap*>(scene["map"]);

	 x0 = gridmap->getMapX();
	 y0 = gridmap->getMapY();
	 xf = x0 + gridmap->getRealWidth();
	 yf = y0 + gridmap->getRealHeight();

	 gluOrtho2D(x0,xf,y0,yf);

	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  (it->second)->drawGL2D();
	 }

	glutSwapBuffers();
}
void OnTimer(int value)
{
	glutTimerFunc(1,OnTimer,0); //set again the timer
	double x,y;
	Point2o* p = dynamic_cast<Point2o*>(scene["z0_point"]);
	x=p->x= -0.5 + p->x+double( rand() )/RAND_MAX;
	y=p->y= -0.5 + p->y+double( rand() )/RAND_MAX;

	scene["z0_point"]=p;

	//scenario["z0_point"]->setX()


	glutPostRedisplay();
}
void OnKeyboardDown(unsigned char key, int x, int y)
{
	 glutPostRedisplay();
}
void OnMouseClick(int b,int state, int x,int y)
{
	bool down=(state==GLUT_DOWN);
	int button;
	if(b==GLUT_LEFT_BUTTON)
	{
		//whatever
	}
	if(b==GLUT_RIGHT_BUTTON)
	{
		//whatever
	}

	int specialKey = glutGetModifiers();
	bool ctrlKey= (specialKey & GLUT_ACTIVE_CTRL)?true:false ;
	bool sKey= specialKey&GLUT_ACTIVE_SHIFT ;

	//DO WHATEVER YOU WANT

	glutPostRedisplay();
}
void OnMouseMove(int x,int y)
{
	 //DO WHATEVER YOU WANT
	glutPostRedisplay();
}

















