#include "datatype/gridmap_includes.h"
#include "algorithms/planning/fm2pathplanner.h"
#include <float.h>
#include <iomanip>
#include <iostream>

using namespace mr;
using namespace std;

class Point2i
{
public:
  Point2i(unsigned int x, unsigned int y)
  {
    this->x = x;
    this->y = y;
  }

  unsigned int x;
  unsigned int y;
};

vector<Point2i> queue;
//labeled gridmap
LabeledGridMap* gridmap;
//fm2 map
FM2PathPlanner2d* fm2starpathPlanner;

void clearScene(); //clear all allocated memory

void initializeScene(); //creates the world
void OnDraw(void); //it will be called when the screen needs to be redrawn
void OnTimer(int value); //it will be called with a timer
void OnKeyboardDown(unsigned char key, int x, int y); //when a key is pressed
void OnMouseMove(int x, int y); //when the mouse moves
void OnMouseClick(int button, int state, int x, int y); //when a button is pressed

map<string, Object*> scene;

int main(int argcp, char *argv[])
{
  atexit(clearScene);
  glutInit(&argcp, argv);
  glutInitWindowSize(800, 600);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  glutCreateWindow("GL");
  glutDisplayFunc(OnDraw);
  glutMotionFunc(OnMouseMove);
  glutMouseFunc(OnMouseClick);
  glutKeyboardFunc(OnKeyboardDown);

  initializeScene();

  glutTimerFunc(5, OnTimer, 0);
  glutMainLoop();
  return 0;
}

void initializeScene()
{
  //***************************************//
  //THE MAP
  double realwidh = 0.0;
  double res = 0.05; // En m
  double mapX = 0.0;
  double mapY = 0.0;
  string file_name = "";

  /*cout << "Introduce the name of the bmp map file:" << endl;

   cin >> file_name;
   cout << "Introduce the real width of the map: ";
   cin >> realwidh;
   cout << "Introduce the x coordinate of the map bottom left: ";
   cin >> mapX;
   cout << "Introduce the y coordinate of the map bottom left: ";
   cin >> mapY;
   */
  file_name = "../../data/Mapa_parcial_2.bmp";    
  Image* img = new Image(1, 1);
  img->load(file_name);
  //img->save("hola.bmp");
  glutReshapeWindow(img->getWidth(), img->getHeight());
  realwidh = res * img->getWidth();    
  std::cout << "Map width: " << img->getWidth() << " pixels   Map height: " << img->getHeight() << " pixels" << std::endl; 
  std::cout << "Map width: " << res * img->getWidth() << " m       Map height: " << res * img->getHeight() << " m" << std::endl; 

  //map resolution
  
  gridmap = new LabeledGridMap();
  gridmap->loadFromImage(img, realwidh, res, mapX, mapY);
  scene["map"] = gridmap;

  //************  END MAP   ****************//

  //*********** CONFIG PLANNER ***************//
  //The fmmap
  fm2starpathPlanner = new FM2PathPlanner2d(gridmap, true);
  fm2starpathPlanner->configure(1.5, 2);

  /*bool path_found_star = fm2starpathPlanner->computePath(Pose(10,12.8),Pose(40,12.8));

   if (path_found_star){
   Path2D* path = new Path2D(fm2starpathPlanner->getPath());
   scene["z0_path"] = path;
   }*/
}

void clearScene()
{
  map<string, Object*>::iterator it;
  for (it = scene.begin(); it != scene.end(); it++)
  {
    delete (it->second);
  }
}

void OnDraw(void)
{
  if (scene.size() == 0)
    return;

  //Borrado de la pantalla
  glClear(GL_COLOR_BUFFER_BIT);

  //Para definir el punto de vista
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  double x0, y0, xf, yf;

  LabeledGridMap* gridmap = dynamic_cast<LabeledGridMap*> (scene["map"]);

  x0 = gridmap->getMapX();
  y0 = gridmap->getMapY();
  xf = x0 + gridmap->getRealWidth();
  yf = y0 + gridmap->getRealHeight();

  gluOrtho2D(x0, xf, y0, yf);

  map<string, Object*>::iterator it;
  for (it = scene.begin(); it != scene.end(); it++)
  {
    (it->second)->drawGL2D();
  }

  glutSwapBuffers();
}
void OnTimer(int value)
{
  if (scene.size() == 0)
    return;
  glutTimerFunc(100, OnTimer, 0); //set again the timer

  //glutPostRedisplay();
}
void OnKeyboardDown(unsigned char key, int x, int y)
{
  if (scene.size() == 0)
    return;
  //glutPostRedisplay();
}
void OnMouseClick(int b, int state, int x, int y)
{
  static unsigned short clicks = 0;

  //the coordinates on the map
  double rx, ry;
  double res_x = gridmap->getRealWidth() / float(glutGet(GLUT_WINDOW_WIDTH));
  double res_y = gridmap->getRealHeight() / float(glutGet(GLUT_WINDOW_HEIGHT));

  rx = x * res_x;
  ry = gridmap->getRealHeight() - y * res_y;

  cout << "\n";
  cout << "gridmap->getRealWidth(): " << gridmap->getRealWidth() << "\n";
  cout << "gridmap->getRealHeight(): " << gridmap->getRealHeight() << "\n";
  cout << "float(glutGet(GLUT_WINDOW_WIDTH)): " << float(glutGet(GLUT_WINDOW_WIDTH)) << "\n";
  cout << "float(glutGet(GLUT_WINDOW_HEIGHT)): " << float(glutGet(GLUT_WINDOW_HEIGHT)) << "\n";

  cout << "res_x: " << res_x << "\n";
  cout << "res_y: " << res_y << "\n";

  cout << "x: " << x << "\n";
  cout << "y: " << y << "\n";

  cout << "rx: " << rx << "\n";
  cout << "ry: " << ry << "\n";

  bool down = (state == GLUT_DOWN);

  if (down)
  {
    cout << "clicks: " << clicks << endl;
    if (clicks == 0)
    {
      Pose* p = new Pose(rx, ry);
      if (scene.count("z1_point_i") > 0)
        delete scene["z1_point_i"];
      scene["z1_point_i"] = p;
      clicks = 1;
    }
    else
    {
      Pose* p = new Pose(rx, ry);
      if (scene.count("z1_point_f") > 0)
        delete scene["z1_point_f"];
      scene["z1_point_f"] = p;

      bool path_found_star = fm2starpathPlanner->computePath(*(dynamic_cast<Pose*> (scene["z1_point_i"])),
                                                             *(dynamic_cast<Pose*> (scene["z1_point_f"])));

      if (path_found_star)
      {
        Path2D* path = new Path2D(fm2starpathPlanner->getPath());
        if (scene.count("z0_path") > 0)
          delete scene["z0_path"];

        scene["z0_path"] = path;

        for (unsigned int i = 0; i < path->points.size(); i++)
        {
          cout << path->points[i].x << " " << path->points[i].y << " " << endl;
        }

      }

      clicks = 0;
    }
  }

  int button;

  if (b == GLUT_LEFT_BUTTON)
  {
    //whatever
  }
  if (b == GLUT_RIGHT_BUTTON)
  {
    //whatever
  }

  int specialKey = glutGetModifiers();
  bool ctrlKey = (specialKey & GLUT_ACTIVE_CTRL) ? true : false;
  bool sKey = specialKey & GLUT_ACTIVE_SHIFT;

  //DO WHATEVER YOU WANT

  if (scene.size() == 0)
    return;
  glutPostRedisplay();
}

void OnMouseMove(int x, int y)
{
  if (scene.size() == 0)
    return;
  //DO WHATEVER YOU WANT
  //glutPostRedisplay();
}
