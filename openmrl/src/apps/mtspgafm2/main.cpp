/**********************************************************************
 *
 * This code is part of the OpenMRL project
 * Author:  Alberto Valero Gomez (alberto.valero.gomez@gmail.com)
 *			Julio Valero Gomez
 *
 *
 * OpenMRL is licenced under the Common Creative License,
 * Attribution-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.
 **********************************************************************/


#include <vector>
#include <string.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <fstream>
#include <GL/glut.h>


//openmrl includes
#include "system/time.h"
#include "datatype/object.h"
#include "datatype/geometry_includes.h"
#include "generation.h"
#include "datatype/gridmap_includes.h"
#include "algorithms/planning/fm2pathplanner.h"


using namespace std;
using namespace mr;

//GLOBAL DATA
map<string,Object*> scene; //<! Scene for drawing in OpenGL
vector<Point2> poses; //<!The travelers and the cities
vector<Point2> travelers;
vector<Point2> cities;


//labeled gridmap
LabeledGridMap* gridmap;
//fm2 map
FM2PathPlanner2d* fm2starpathPlanner;


Path2D** paths; //<!The paths
double** costs; //<!The cost to travel from one point to another
int num_cities, num_travellers, steady_number, population_size; //<! GA params
Generation* generation; //GA Generations

//STANDARD FUNCTIONS OF MRL APP WITH OPENGL DRAWING
void initializeScene();
void onTimer(int value);
void drawScene();
void clearScene();
void OnKeyboardDown(unsigned char key, int x, int y); //when a key is pressed
void OnMouseMove(int x,int y); //when the mouse moves
void OnMouseClick(int button,int state, int x,int y); //when a button is pressed
void addPathToScene(Path2D* path,int robot,int sub_robot);
void removePathsFromScene();



//CUSTOM FUNCTIONS
void initializeMTSP();
void evolve();
void clearMTSP(); //free allocated space, etc.

int main(int argcp, char *argv[]){

	 atexit(clearScene);
	 glutInit(&argcp, argv);
	 //glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	 glutInitWindowSize(800,600);
	 glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	 glutCreateWindow("MTSP GA");
	 initializeScene();
	 //drawScene();
	 glutMouseFunc(OnMouseClick);
	 glutKeyboardFunc(OnKeyboardDown);
	 glutDisplayFunc(drawScene);
	 glutMainLoop();
	 return 0;
}

void addPathToScene(Path2D *path, int robot, int sub_robot){
	 if (robot==0) path->setRGB(1,0,0);
	 if (robot==1) path->setRGB(0,1,0);
	 if (robot==2) path->setRGB(0,0,1);
	 if (robot==3) path->setRGB(0.5,0.5,0);
	 if (robot==5) path->setRGB(0,0.5,0.5);
	 if (robot==6) path->setRGB(0.5,0,0.5);

	 stringstream ss;
	 ss << "z1_path_" << robot << "_" << sub_robot;
	 if (scene.count(ss.str())>0)
		  delete scene[ss.str()];
	 scene[ss.str()]=path;
}

void initializeMTSP(){

	 num_cities=cities.size();
	 num_travellers=travelers.size();
	 steady_number=10000;
	 population_size=100;

	 for (int i=0; i<travelers.size(); i++){
		  poses.push_back(travelers[i]);
	 }
	 for (int i=0; i<cities.size(); i++){
		  poses.push_back(cities[i]);
	 }

	 unsigned int num_points = poses.size();

	 //CREATE THE COSTS TABLE
	 //costs -> global variable
	 costs = new double*[num_points];
	 paths = new Path2D*[num_points];

	 for (int i=0;i<num_points;i++){
		 costs[i]=new double[num_points];
		 paths[i]=new Path2D[num_points];
	 }

	 for (int i=0;i<num_points;i++){
		 costs[i][i]=0;
		 for (int j=i+1;j<num_points;j++){
			  cout << i << "," << j << endl;
			  cout << poses[i] << " , " << poses[j] << endl;
			  bool path_found = fm2starpathPlanner->computePath(Pose(poses[i]),Pose(poses[j]));
			  if (path_found){
					Path2D path = fm2starpathPlanner->getPath();
					paths[i][j]=paths[j][i]=path;
					costs[j][i]=costs[i][j]=fm2starpathPlanner->getPathEstimatedTime();
					cout << "cost: " << costs[i][j] << endl;
			  }else{
					costs[j][i]=costs[i][j]=DBL_MAX/4;
			  }
		 }
	 }

	 cout << "Graph Created" << endl;

	 /************ END CREATE GRAPH *******************/

	 /****** INITIALIZE GA ***************/
	 generation = new Generation(population_size,num_cities,num_travellers,costs,20,1.05);
	 //generation->generateInitGeneration(5,1000);
	 generation->generateRandomPopulation();
	 cout << "first generation created" << endl;
}
void evolve(){

	 //cout << "evolving" << endl;
	 //INITIALIZTION
	 static int numberGenerations = 0;
	 static double bestCost,prevBestCost;
	 static int steadyCounter=0;
	 static int steadyCounter2=0;

	 static bool first_time = true;

	 Individual* ind;
	 //cout << 1 << endl;
	 ind = generation->getBestIndividual();
	 //cout << 2 << endl;

	 if (first_time){
		  bestCost=ind->getCost();
		  prevBestCost=bestCost;
		  first_time=false;
	 }
	 // END INIT

	 // MAIN LOOP
	 // add paths to the scene and draw

	 //// DRAW
	 removePathsFromScene();
	 int* gen = ind->getGen();
	 for (int i=0; i<ind->getGenSize(); ){
		  int p = gen[i];
		  int px = gen[i+1];
		  //start path
		  if (p<num_travellers){
				int robot = p;
				//cout << "robot " << robot << ": ";
				Path2D* path = new Path2D(paths[p][px]);
				addPathToScene(path,robot,p);
				i++;
				while(i < ind->getGenSize() && gen[i] >= num_travellers){
					 int p = gen[i];
					 if ( (i+1) < ind->getGenSize() )
						  px = gen[i+1];
					 else
						  px = gen[robot];

					 //cout << "aqui " << p << endl;
					 if ( (px) >= num_travellers ){
						  path = new Path2D(paths[p][px]);
						  addPathToScene(path,robot,p);
					 }else{
						  path = new Path2D(paths[p][robot]);
						  addPathToScene(path,robot,p);
					 }
					 i++;
				}
		  }
	 }

	 glutPostRedisplay();
//////////////END DRAWING /////////////////////////////


	 numberGenerations ++;

	 if (steadyCounter>steady_number){
		 //Unfortunately this is not standard glut
		  // glutLeaveMainLoop(); //end the evolution
		  glutPostRedisplay();
		  cout << "done" << endl;
		  return;
	 }
	 float mutationPerc = 100.f*float(steadyCounter)/500;

	 if (mutationPerc>50) mutationPerc=50;

	 generation->setMutationPerc(mutationPerc);
	 generation->generateNewGeneration();
	 ind=generation->getBestIndividual();
	 bestCost=ind->getCost();
	 if (fabs(bestCost-prevBestCost)<0.1){
		  steadyCounter++;steadyCounter2++;
	 }
	 else{
		  steadyCounter=steadyCounter2=0;
	 }
	 prevBestCost=bestCost;

	 if (steadyCounter2>=steady_number/3){
		  generation->inmigration(1,10000,25);
		  steadyCounter2=0;
	 }

	 glutTimerFunc(0,onTimer,0);
}

void removePathsFromScene(){
	 string aux="z1_path";
	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  string aux2 = it->first;
		  if(string::npos != aux2.find(aux)){
				delete (it->second);
				scene.erase(it);
		  }
	 }

}

void clearMTSP(){
	 //created in intializeMTSP
	 delete [] costs;
	 delete generation;
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

	 double x0,y0,xf,yf;

	 x0 = gridmap->getMapX();
	 y0 = gridmap->getMapY();
	 xf = x0 + gridmap->getRealWidth();
	 yf = y0 + gridmap->getRealHeight();

	 gluOrtho2D(x0,xf,y0,yf);


	 //*********** CONFIG PLANNER ***************//
	 //The fmmap
	 fm2starpathPlanner = new FM2PathPlanner2d(gridmap,true);
	 fm2starpathPlanner->configure(1.5,2);
}

void onTimer(int value){
	 evolve();
	 glutPostRedisplay();
}

void drawScene(){
	 //if (scene.size()==0) return;
	 //Borrado de la pantalla
	 glClear(GL_COLOR_BUFFER_BIT );
	 glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  (it->second)->drawGL2D();
	 }

	 glutSwapBuffers();
}


void clearScene(){
	 clearMTSP();
	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  delete (it->second);
	 }
	 scene.clear();
}

void OnKeyboardDown(unsigned char key, int x, int y)
{
	 if (key == 'g'){
		  initializeMTSP();
		  glutTimerFunc(0,onTimer,0);
	 }
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
		  Point2 *p = new Point2(rx,ry);
		  if(b==GLUT_LEFT_BUTTON)
		  {
				p->setRGB(0,1,0);
			  cities.push_back(*p);
			  stringstream ss;
			  ss << "z0_city_" << cities.size();
			  scene[ss.str()]=p;
		  }
		  if(b==GLUT_RIGHT_BUTTON){
				p->setRGB(1,0,0);
				travelers.push_back(*p);
				stringstream ss;
				ss << "z0_traveler_" << travelers.size();
				scene[ss.str()]=p;
		  }
	 }

	 glutPostRedisplay();
}






