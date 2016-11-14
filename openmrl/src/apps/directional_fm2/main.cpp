#include "datatype/gridmap_includes.h"
#include "algorithms/planning/fm2pathplanner.h"
#include <float.h>
#include <iomanip>
#include <iostream>

using namespace mr;
using namespace std;

class Point2i{
	public:
	Point2i(unsigned int x, unsigned int y){this->x=x; this->y=y;}

	unsigned int x;
	unsigned int y;
};

vector<Point2i> queue;
//labeled gridmap
LabeledGridMap* gridmap;
//fm2 map
FM2PathPlanner2d* dirFM2;
FM2PathPlanner2d* nondirFM2;

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

	 glutReshapeWindow(img->getWidth(),img->getHeight());

	 //map resolution
	 double res = realwidh / img->getWidth();
	 gridmap = new LabeledGridMap();
	 gridmap->loadFromImage(img,realwidh,res,mapX,mapY);
	 scene["map"]=gridmap;

	 //************  END MAP   ****************//

	 //*********** CONFIG PLANNER ***************//
	 //The fmmap
	 //star = false, direcdtinal = true
	 dirFM2 = new FM2PathPlanner2d(gridmap,false,true);
	 dirFM2->configure(1.5,2);

	 //star = true, directional = false
	 nondirFM2 = new FM2PathPlanner2d(gridmap,true,false);
	 nondirFM2->configure(1.5,2);

	 /*bool path_found_star = fm2starpathPlanner->computePath(Pose(10,12.8),Pose(40,12.8));

	 if (path_found_star){
		  Path2D* path = new Path2D(fm2starpathPlanner->getPath());
		  scene["z0_path"] = path;
	 }*/
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

	 //glutPostRedisplay();
}
void OnKeyboardDown(unsigned char key, int x, int y)
{
	 if (scene.size()==0) return;
	 //glutPostRedisplay();
}
void OnMouseClick(int b,int state, int x,int y)
{
	 static unsigned short clicks = 0;

	 //the coordinates on the map
	 double rx, ry;
	 double res_x = gridmap->getRealWidth() / float(glutGet(GLUT_WINDOW_WIDTH));
	 double res_y = gridmap->getRealHeight() / float(glutGet(GLUT_WINDOW_HEIGHT));

	 rx = x * res_x;
	 ry = gridmap->getRealHeight() - y * res_y;

	 bool down=(state==GLUT_DOWN);

	 if (down){
		  cout << "clicks: " << clicks << endl;
		  if (clicks == 0){
				Pose* p = new Pose(rx,ry);
				if (scene.count("z1_point_i")>0) delete scene["z1_point_i"];
				scene["z1_point_i"] = p;
				clicks = 1;
		  }else{
				Pose* p = new Pose(rx,ry);
				if (scene.count("z1_point_f")>0) delete scene["z1_point_f"];
				scene["z1_point_f"] = p;

				bool path_found_star = nondirFM2->computePath(
						  *(dynamic_cast<Pose*>(scene["z1_point_i"]) ) ,
						  *(dynamic_cast<Pose*>(scene["z1_point_f"])) );

				if (path_found_star){
					 Path2D* path = new Path2D(nondirFM2->getPath());
					 if (scene.count("z0_path")>0) delete scene["z0_path"];
					 scene["z0_path"] = path;

					 ofstream os("fm2.dat");
					 for (int i = 0; i<path->speed_profile.size();i++){
						  os << path->speed_profile[i]<<" ";
					 }
					 os.close();

				}

				bool path_found_dir = dirFM2->computePath(
						  *(dynamic_cast<Pose*>(scene["z1_point_i"]) ) ,
						  *(dynamic_cast<Pose*>(scene["z1_point_f"])) );

				if (path_found_dir){
					 Path2D* path = new Path2D(dirFM2->getPath());
					 if (scene.count("z1_path")>0) delete scene["z1_path"];
					 path->setRGB(1,0,0);
					 scene["z1_path"] = path;

					 ofstream os("fm2dir.dat");
					 for (int i = 0; i<path->speed_profile.size();i++){
						  os << path->speed_profile[i]<<" ";
					 }
					 os.close();

				}

				clicks = 0;
		  }
	 }

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

	 if (scene.size()==0) return;
	 glutPostRedisplay();
}

void OnMouseMove(int x,int y)
{
	 if (scene.size()==0) return;
	 //DO WHATEVER YOU WANT
	 //glutPostRedisplay();
}






