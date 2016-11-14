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


using namespace std;
using namespace mr;

//GLOBAL DATA
map<string,Object*> scene; //<! Scene for drawing in OpenGL
vector<Point2> poses; //<!The travelers and the cities
double** costs; //<!The cost to travel from one point to another
int num_cities, num_travellers, steady_number, population_size; //<! GA params
Generation* generation; //GA Generations

//STANDARD FUNCTIONS OF MRL APP WITH OPENGL DRAWING
void initializeScene();
void onTimer(int value);
void drawScene();
void clearScene();

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
	 initializeMTSP();
	 glutDisplayFunc(drawScene);
	 glutTimerFunc(0,onTimer,0);
	 glutMainLoop();
	 return 0;
}

void initializeMTSP(){

	 cout << "Please introduce the following data:" << endl;
	 cout << "Num Cities: "; cin >> num_cities;
	 //num_cities = 100;
	 cout << "Num Travellers: "; cin >> num_travellers;
	 //num_travellers = 6;
	 cout << "Number of steady iterations for terminate: "; cin >> steady_number;
	 //steady_number=10000;

	 population_size=100;


	 /********* CREATE GRAPH*********/
	 cout << "Creating Graph" << endl;
	 srand(time(0));


	 int num_points = num_cities + num_travellers;

	 for (int i=0;i<num_travellers;i++){
		 double x= double(rand()%500);
		 double y= double(rand()%250);
		 poses.push_back(Point2(x,y));

		 //the scene to draw
		 Pose* pose = new Pose(x,y);
		 pose->setRGB(1,1,1);
		 stringstream ss;
		 ss << "z0_robot_" << i;
		 scene[ss.str()]=pose;
	 }

	 for (int i=0;i<num_cities;i++){
		 double x= double(rand()%500);
		 double y= double(rand()%250);
		 poses.push_back(Point2(x,y));

		 //the scene to draw
		 Pose* pose = new Pose(x,y);
		 stringstream ss;
		 ss << "z0_city_" << i;
		 scene[ss.str()]=pose;
	 }

	 //CREATE THE COSTS TABLE
	 //costs -> global variable
	 costs = new double*[num_points];

	 for (int i=0;i<num_points;i++){
		 costs[i]=new double[num_points];
	 }

	 for (int i=0;i<num_points;i++){
		 costs[i][i]=0;
		 for (int j=i+1;j<num_points;j++){
			  costs[j][i]=costs[i][j]=mrl_distance(poses[i],poses[j]);
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
	 //cout << 3 << endl;
	 int* gen = ind->getGen();
	 //cout << 4 << endl;
	 for (int i=0; i<ind->getGenSize(); ){
		  int p = gen[i];
		  //start path
		  if (p<num_travellers){
				int robot = p;
				//cout << "robot " << robot << ": ";
				Path2D* path = new Path2D();
				path->push_back(poses[p]);
				i++;
				while(i < ind->getGenSize() && gen[i] >= num_travellers){
					 int p = gen[i];
					 path->push_back(poses[p]);
					// cout << p << " ";
					 i++;
				}
				//path colors

				if (robot==0) path->setRGB(1,0,0);
				if (robot==1) path->setRGB(0,1,0);
				if (robot==2) path->setRGB(0,0,1);
				if (robot==3) path->setRGB(0.5,0.5,0);
				if (robot==5) path->setRGB(0,0.5,0.5);
				if (robot==6) path->setRGB(0.5,0,0.5);

				path->push_back(poses[robot]);
				stringstream ss;
				ss << "z1_path_" << robot;
				scene[ss.str()]=path;
		  }
	 }

	 glutPostRedisplay();
////////////////////////////////////////////////////////
	 //cout << 5 << endl;
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

void clearMTSP(){
	 //created in intializeMTSP
	 delete [] costs;
	 delete generation;
}

void initializeScene(){

	 //Para definir el punto de vista
	 glMatrixMode(GL_MODELVIEW);
	 glLoadIdentity();

	 double x0,y0,xf,yf;

	 x0 = 0;
	 y0 = 0;
	 xf = 500;
	 yf = 250;

	 glutReshapeWindow(2*xf, 2*yf);
	 gluOrtho2D(x0,xf,y0,yf);

}

void onTimer(int value){
	 evolve();
	 glutPostRedisplay();
}

void drawScene(){
	 if (scene.size()==0) return;

	 //Borrado de la pantalla
	 glClear(GL_COLOR_BUFFER_BIT );

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

}





