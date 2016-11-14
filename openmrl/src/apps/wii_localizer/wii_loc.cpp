#include <datatype/gridmap_includes.h>
#include <datatype/geometry/path2d.h>
#include <system/time.h>
#include <GL/glut.h>

#include <map>
#include <vector>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <climits>
#include <string>
#include <cwiid.h>
#include <streambuf>
#include <sstream>


using namespace std;
using namespace mr;

cwiid_wiimote_t *g_wiimote = NULL;

void clearScene(); //clear all allocated memory
void initializeScene(); //creates the world
void OnDraw(void); //it will be called when the screen needs to be redrawn
void OnTimer(int value); //it will be called with a timer
void OnKeyboardDown(unsigned char key, int x, int y); //when a key is pressed
void OnMouseMove(int x,int y); //when the mouse moves
void OnMouseClick(int button,int state, int x,int y); //when a button is pressed
void readIR();
void initWiimote();

map<string,Object*> scene;
cwiid_mesg_callback_t cwiid_callback;


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

	 initWiimote();
	 initializeScene();

	 glutTimerFunc(0,OnTimer,0);
	 glutMainLoop();
	 return 0;
}

void initWiimote(){
	 bdaddr_t g_bdaddr = (bdaddr_t) {{0, 0, 0, 0, 0, 0}}; //ANY ADDRESS

	 g_wiimote = cwiid_connect(&g_bdaddr,CWIID_FLAG_MESG_IFC);
	 if (g_wiimote!=NULL){
		  cout << "Wiimote Connected" << endl;
		  cwiid_enable(g_wiimote, CWIID_FLAG_MESG_IFC);
	 }else{
		  exit(0);
	 }

	 cwiid_command(g_wiimote, CWIID_CMD_RPT_MODE, CWIID_RPT_IR);
	 struct cwiid_state g_wii_state;
	 cwiid_get_state(g_wiimote, &g_wii_state);
	 cout << "Battery life: " << 100.* g_wii_state.battery / CWIID_BATTERY_MAX << endl;
	 if(cwiid_set_mesg_callback(g_wiimote, cwiid_callback )){
		  cerr << "Unable to set messabe callback" << endl;
	 }
}

void initializeScene(){
	 //THE MAP
	 LabeledGridMap* gridmap = new LabeledGridMap();
	 Image* img = new Image(1,1);
	 img->load("../../data/simple_a4.bmp");
	 gridmap->loadFromImage(img,18,0.0244,0,0);
	 scene["map"]=gridmap;

	 glutReshapeWindow(img->getWidth(),img->getHeight());

}

void clearScene(){
	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  delete (it->second);
	 }

	 //Disconnect Wii
	 cwiid_disconnect(g_wiimote);

}

void OnDraw(void)
{
	 //MRTime timer;
	 //timer.precistic();
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
	//cout << timer.precistoc() << endl;
}

void cwiid_callback(cwiid_wiimote_t *wiimote, int mesg_count,
						  union cwiid_mesg mesg[], struct timespec *timestamp){
	 for (int i=0; i < mesg_count; i++)
	 {
		  switch (mesg[i].type) {
		  case CWIID_MESG_IR:
			 for (int j = 0; j < CWIID_IR_SRC_COUNT; j++) {
				  if (mesg[i].ir_mesg.src[j].valid) {
						double x = mesg[i].ir_mesg.src[j].pos[CWIID_X];
						double y = mesg[i].ir_mesg.src[j].pos[CWIID_Y];
						x = 18.*(x-165)/(603-165);
						y = 26.*(y-121)/(673-121);
						Pose* p = new Pose(x,y);
						stringstream ss;
						ss << "z1_point_" << j;
						if (scene.count(ss.str())>0) delete scene[ss.str()];
						scene[ss.str()] = p;
				  }
			 }
			 break;
		  case CWIID_MESG_STATUS:
		  case CWIID_MESG_BTN:
		  case CWIID_MESG_ACC:
		  case CWIID_MESG_NUNCHUK:
		  case CWIID_MESG_CLASSIC:
				break;
		  case CWIID_MESG_ERROR:
				if (cwiid_close(wiimote)) {
					 cout << "Error on wiimote disconnect" << endl;
					 exit(-1);
				}
				cout << "wiid msg error" << endl;
				exit(0);
				break;
		  default:
				break;
		  }
	 }
}

void OnTimer(int value)
{
	 /*
	 cwiid_command(g_wiimote, CWIID_CMD_RPT_MODE,CWIID_RPT_IR);
	 struct cwiid_state g_wii_state;
	 cwiid_get_state(g_wiimote, &g_wii_state);

	 //LED 1
	 if(g_wii_state.ir_src[0].valid){
		  double x = g_wii_state.ir_src[0].pos[CWIID_X];
		  double y = g_wii_state.ir_src[0].pos[CWIID_Y];
		  x = 18.*(x-165)/(603-165);
		  y = 26.*(y-121)/(673-121);
		  Pose* p = new Pose(x,y);
		  if (scene.count("z1_point_1")>0) delete scene["z1_point_1"];
		  scene["z1_point_1"] = p;
	 }

	 //LED 2
	 if(g_wii_state.ir_src[1].valid){
		  double x = g_wii_state.ir_src[1].pos[CWIID_X];
		  double y = g_wii_state.ir_src[1].pos[CWIID_Y];
		  x = 18.*(x-165)/(603-165);
		  y = 26.*(y-121)/(673-121);
		  Pose* p = new Pose(x,y);
		  if (scene.count("z1_point_2")>0) delete scene["z1_point_2"];
		  scene["z1_point_2"] = p;
	 }

*/
	 glutTimerFunc(200,OnTimer,0); //set again the timer
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


void readIR()
{

}




















