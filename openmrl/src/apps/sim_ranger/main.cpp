#include "datatype/gridmap_includes.h"
#include "sim/simrangereading.h"
#include "sim/simrangedata.h"
#include <float.h>
#include <iomanip>
#include <iostream>

using namespace mr;
using namespace std;

//labeled gridmap
LabeledGridMap* gridmap;

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
	 //***************************************//
	 //THE MAP
	 double realwidh;
	 double mapX, mapY;
	 string file_name;

	 /*cout << "Introduce the name of the bmp map file:" << endl;

	 cin >> file_name;
	 cout << "Introduce the real width of the map: ";
	 cin >> realwidh;
	 cout << "Introduce the x coordinate of the map bottom left: ";
	 cin >> mapX;
	 cout << "Introduce the y coordinate of the map bottom left: ";
	 cin >> mapY;
*/
	 file_name="../../data/simple.bmp";
	 realwidh=50;
	 mapX=mapY=0;
	 Image* img = new Image(1,1);
	 img->load(file_name);

	 glutReshapeWindow(2*img->getWidth(),2*img->getHeight());

	 //map resolution
	 double res = realwidh / img->getWidth();
	 gridmap = new LabeledGridMap();
	 gridmap->loadFromImage(img,realwidh,res,mapX,mapY);
	 scene["map"]=gridmap;

	 //************  END MAP   ****************//

}

void clearScene(){
	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  delete (it->second);
	 }
}

void OnDraw(void)
{
	 if (scene.size()==0) return;

	 //Borrado de la pantalla
	 glClear(GL_COLOR_BUFFER_BIT );

	 //Para definir el punto de vista
	 glMatrixMode(GL_MODELVIEW);
	 glLoadIdentity();

	 double x0,y0,xf,yf;


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
	 if (scene.size()==0) return;
	 glutTimerFunc(100,OnTimer,0); //set again the timer
	 glutPostRedisplay();
}
void OnKeyboardDown(unsigned char key, int x, int y)
{
	 if (scene.size()==0) return;
	 //glutPostRedisplay();
}
void OnMouseClick(int b,int state, int x,int y)
{
	 //the coordinates on the map
	 double rx, ry;
	 double res_x = gridmap->getRealWidth() / float(glutGet(GLUT_WINDOW_WIDTH));
	 double res_y = gridmap->getRealHeight() / float(glutGet(GLUT_WINDOW_HEIGHT));

	 rx = x * res_x;
	 ry = gridmap->getRealHeight() - y * res_y;

	 bool down=(state==GLUT_DOWN);

	 if (down){
		  srand(time(0));
		  double angle = M_2PI*rand()/RAND_MAX;
		  Pose* p = new Pose(rx,ry, angle);
		  if (scene.count("z1_point")>0) delete scene["z1_point"];
		  scene["z1_point"] = p;
		  SimRangeData* range_data = new SimRangeData(gridmap,*p,"generic",5,deg2rad(30),deg2rad(-90),deg2rad(90),0,10,0);
		  if (scene.count("z2_range")>0) delete scene["z2_range"];
		  scene["z2_range"] = range_data;
	 }

	 if (scene.size()==0) return;
	 glutPostRedisplay();
}

void OnMouseMove(int x,int y)
{
	 if (scene.size()==0) return;
	 //DO WHATEVER YOU WANT
	 //glutPostRedisplay();
}






