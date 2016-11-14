/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Alberto Valero Gomez (alberto.valero.gomez@gmail.com)
 *			Julio Valero Gomez
 *
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
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
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
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
//#include "/usr/local/include/mrcore/mrcore.h"
#include <iostream>
#include <fstream>
#include "tictoc.h"

#include "generation.h"

#include "gnuplot_gui.h"
#include "string.h"

using namespace std;
using namespace Gnuplot;

class Pose{
public:
	Pose(double x, double y){this->x=x;this->y=y;}
	~Pose(){}
	double x,y;
	double distTo(Pose p){
		return sqrt(fabs((x-p.x)*(x-p.x) + (y-p.y)*(y-p.y)));
	}
};

void drawResult(GnuplotGui& gui, vector<Point2d> gplotCities, vector<Point2d> gplotTravellers, Individual* ind, vector<Pose>& poses){
	gui.clear();
	gui.draw_points(gplotTravellers,"Travellers","Travellers");
	gui.draw_points(gplotCities,"Cities","cities");

	int genSize = ind->getGenSize();
	int* gen = ind->getGen();
	int num_travellers= ind->getNumTravellers();
	int robot = gen[0];

//	cout << "Robot = " <<robot << endl;

	vector < Path<Point2d> > paths;
	paths.resize(num_travellers);

	for (int i=0; i<genSize-1; i++){
		if (gen[i+1] < num_travellers){
			paths[robot].push(Point2d(poses[gen[i]].x, poses[gen[i]].y));
			paths[robot].push(Point2d(poses[robot].x, poses[robot].y));
			//next robot
			robot = gen[i+1];
//			cout << "Robot = " <<robot << endl;
			continue;
		}
		paths[robot].push(Point2d(poses[gen[i]].x, poses[gen[i]].y));
	}
	paths[robot].push(Point2d(poses[gen[genSize-1]].x, poses[gen[genSize-1]].y));
	paths[robot].push(Point2d(poses[robot].x, poses[robot].y)); //close the last loop

	//draw the paths
	for (unsigned int i=0;i<paths.size();i++){
		string tag;
		stringstream ss;//create a stringstream
		ss << "Robot " << i;//add number to the stream
		tag = ss.str();//return a string with the contents of the stream
		gui.draw_path(paths[i],tag);
	}

	gui.redraw();
}

int main(){

	GnuplotGui gui_poses;
	GnuplotGui gui_fitness;

	gui_poses.set_xrange(0,50);
	gui_poses.set_yrange(0,50);

	gui_fitness.set_xlabel("Generation");
	gui_fitness.set_ylabel("Fitness of best Individual");

	vector<Point2d> gplotCities;
	vector<Point2d> gplotTravellers;



	//vector<Pose> poses;


	int num_cities, num_travellers, steady_number, population_size, numTests;
	int log, draw;

	cout << "Please introduce the following data:" << endl;
	cout << "Num Cities: "; cin >> num_cities;
	cout << "Num Travelers: "; cin >> num_travellers;
	cout << "Number of steady iterations for terminate: "; cin >> steady_number;
	cout << "Do you want to log? (0:No, 1:Yes): "; cin >> log;
	cout << "Do you want to draw evolution? (0:No, 1:Yes): "; cin >> draw;

	population_size=100;

	//Log file
	ofstream logFile;

	if (log!=0){
		logFile.open("data.log");
		if (!logFile.is_open()){
			cerr << "Cannot open file for logs" << endl;
			return false;
		}
		logFile << "Pop. Size \t Steady N. \t Robots \t Targets \t Generations \t Time \t TotalCost \t Robot 1 \t Robot 2 \t Robot 3 \t Robot 4 \t Robot 5 "<<endl;
	}





	/********* CREATE GRAPH*********/
	cout << "Creating Graph" << endl;
	srand(time(0));
	vector<Pose> poses;
	double** costs;
	int num_points = num_cities + num_travellers;
	
	// Creating the travelers origin
	for (int i=0;i<num_travellers;i++){
		double x= double(rand()%50);
		double y= double(rand()%50);
		gplotTravellers.push_back(Point2d(x,y));
		poses.push_back(Pose(x,y));
	}
	gui_poses.draw_points(gplotTravellers,"Travellers","Travellers"); gui_poses.redraw();

	// Creating the cities
	for (int i=0;i<num_cities;i++){
		double x= double(rand()%50);
		double y= double(rand()%50);
		gplotCities.push_back(Point2d(x,y));
		poses.push_back(Pose(x,y));
	}

	Path<Point2d> fitness;

	costs = new double*[num_points];

	for (int i=0;i<num_points;i++){
		costs[i]=new double[num_points];
	}

	for (int i=0;i<num_points;i++){
		costs[i][i]=0;
		for (int j=i+1;j<num_points;j++){
			costs[j][i]=costs[i][j]=(poses[i]).distTo(poses[j]);
		}
	}

	cout << "Graph Created" << endl;

	/************ END CREATE GRAPH *******************/


	/************** EVOLVE **************************/

	// Generation(int population_size, int num_cities, int num_travellers, double** costs =NULL, double mutation=20, double cF=1.05);
	Generation* generation = new Generation(population_size,num_cities,num_travellers,costs,20,1.05);
	int numberGenerations = 0;

	double bestCost,prevBestCost;

	tictoc* timer = new tictoc();
	tic(timer);

	//generation->generateInitGeneration(5,1000);
	generation->generateRandomPopulation();

	Individual* ind = generation->getBestIndividual();

	bestCost=ind->getCost();
	prevBestCost=bestCost;

	fitness.push(Point2d(0,bestCost));

	// draw poses
	if (draw!=0){
		gui_fitness.clear();
		gui_fitness.draw_path(fitness,"Fitness", "Best Ind. Fitness");
		gui_fitness.redraw();
		gui_poses.draw_points(gplotCities,"Cities","cities");
		gui_poses.redraw();
	}

	gui_fitness.redraw();
	gui_poses.redraw();

	sleep(4);


	int steadyCounter=0;
	int steadyCounter2=0;

	while(true){

		numberGenerations ++;

		if (steadyCounter>steady_number){
			cout << "End of evolutioin" << endl;
			break; //end the evolution
		}

		float mutationPerc = 100.f*float(steadyCounter)/500;
		if (mutationPerc>50) mutationPerc=50;
		generation->setMutationPerc(mutationPerc);
		generation->generateNewGeneration();
		Individual* ind=generation->getBestIndividual();
		bestCost=ind->getCost();

		if (fabs(bestCost-prevBestCost)<0.1){
			steadyCounter++;steadyCounter2++;
		}
		else{
			steadyCounter=steadyCounter2=0;
			fitness.push(Point2d(numberGenerations,bestCost));

			if (draw!=0) drawResult(gui_poses,gplotCities,gplotTravellers,ind,poses);

			//cout << "Improvement" << endl;
		}

		prevBestCost=bestCost;

		if (steadyCounter2>=steady_number/3){
			generation->inmigration(1,10000,25);
			steadyCounter2=0;
		}

		if (draw!=0 && numberGenerations%100==0){
			gui_fitness.clear();
			gui_fitness.draw_path(fitness,"Fitness", "Best Ind. Fitness");
			gui_fitness.redraw();
		}
	}

	drawResult(gui_poses,gplotCities,gplotTravellers,generation->getBestIndividual(),poses);

	gui_fitness.clear();
	gui_fitness.draw_path(fitness, "Fitness", "GA Evolution");
	gui_fitness.redraw();

	cout << "End of Evolution" << endl;
	/**********END EVOLUTION*******************/

	/** FOR LOGGING **/

	double totalTime = toc(timer);

	ind = generation->getBestIndividual();

	double totalCost = bestCost;
	numberGenerations = numberGenerations - steady_number;
	double* individualCosts = new double[num_travellers];

	for (int i=0;i<num_travellers;i++) individualCosts[i]=0;

	int * gen = ind->getGen();
	int genSize = ind->getGenSize();

	int robot = gen[0];

	for (int i=0; i<genSize-1; i++){
		if (gen[i+1] < num_travellers){
			individualCosts[robot]+=costs[gen[i]][robot]; //close the path
			//next robot
			robot = gen[i+1];
			continue;
		}
		individualCosts[robot]+=costs[gen[i]][gen[i+1]];
	}

	individualCosts[robot]+=costs[gen[genSize-1]][robot]; //close the last loop


	cout << population_size << "\t " << steady_number << "\t" << num_travellers << "\t" << num_cities << "\t" << numberGenerations << "\t" << totalTime << "\t" << totalCost <<"\t";
	if (log!=0){
		logFile << population_size << "\t " << steady_number << "\t" << num_travellers << "\t" << num_cities << "\t" << numberGenerations << "\t" << totalTime << "\t" << totalCost <<"\t";
	}
	for (int i=0; i < num_travellers;i++){
		if (log!=0){
			logFile << individualCosts[i] << "\t";
		}
		cout << individualCosts[i] << "\t";
	}
	cout <<endl;
	if (log!=0){
		logFile << endl;
	}



	/** END LOGGING OPERATIONS**/
	delete [] individualCosts;
	delete generation;
	delete timer;

	delete [] costs;

	cout << "Done!" << endl;
	if (log!=0) logFile.close();
	return 1;
}
