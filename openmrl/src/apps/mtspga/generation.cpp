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


#include "generation.h"

#include <cmath>
#include <ctime>
#include <string.h>

namespace mr {

Generation::Generation(int population_size, int num_cities, int num_travellers, double** costs, double mutation, double cF) {

	srand ( time(NULL) ); //initialize seed for this program execution
	//generate new seed for random generator boost
	generator.seed(static_cast<unsigned int>(std::time(0)));
	costsTable=costs;
	mutationPerc=mutation;
	convergenceFactor=cF;
	this->num_cities=num_cities;
	this->num_travellers=num_travellers;
	this->genSize = num_travellers + num_cities;
	setPopulationSize(population_size);
	individuals.clear();
}

Generation::~Generation() {
	//Remove all reserved space for individuals
	for (unsigned int i=0;i<individuals.size();i++){
		if (individuals[i]!=NULL) delete individuals[i];
	}
	individuals.clear();

}

void Generation::inmigration(int numPopulations, int numIterations, int inmigrationSize){
	vector<Generation*> generations;
	vector< vector<Individual*> >initIndividuals;
	for (int i=0;i<numPopulations;i++){
		generations.push_back(new Generation(populationSize,num_cities,num_travellers, costsTable, 20,convergenceFactor));
		generations[i]->generateRandomPopulation();
		for (int j=0;j<numIterations;j++){
			generations[i]->generateNewGeneration();
		}
		initIndividuals.push_back(generations[i]->getIndividuals());
	}
	individuals.resize(populationSize- inmigrationSize*numPopulations );
	//now take the best individuals of each generation
	for (int i=0;i<numPopulations;i++){
		for (int j=0;j<inmigrationSize;j++){
			orderedInsert(individuals, new Individual(initIndividuals[i][j]));
		}
	}
	//clear reserved space
	for (int i=0;i<numPopulations;i++){
		delete generations[i];
	}
	
}
void Generation::generateInitGeneration(int numPopulations, int numIterations){
	vector<Generation*> generations;
	vector< vector<Individual*> >initIndividuals;
	for (int i=0;i<numPopulations;i++){
		generations.push_back(new Generation(populationSize,num_cities,num_travellers, costsTable, 20,convergenceFactor));
		generations[i]->generateRandomPopulation();
		for (int j=0;j<numIterations;j++){
			generations[i]->generateNewGeneration();
		}
		initIndividuals.push_back(generations[i]->getIndividuals());
	}

	//now take the best individuals of each generation
	for (int i=0;i<numPopulations;i++){
		for (int j=0;j<20;j++){
			orderedInsert(individuals, new Individual(initIndividuals[i][j]));
		}
	}

	for (int i=0;i<numPopulations;i++){
		delete generations[i];
	}
}

void Generation::generateRandomPopulation(){
	for (int i=0;i<populationSize;i++){
		Individual* ind=new Individual(costsTable,num_cities,num_travellers);
		ind->randomGen();
		orderedInsert(individuals, ind);
	}
}

void Generation::generateNewGeneration(){
	vector<Individual*> children;

	//Memory effect. We keep the best father.
	//we must make a copy of it, because we will delete afterwards all individuals

	Individual* ind = new Individual(costsTable,num_cities,num_travellers);

	//Let's copy the gen
	int* fatherGen = individuals[0]->getGen();
	int* childGen = new int[genSize];

	memcpy(childGen, fatherGen, genSize*sizeof(int));

	ind->setGen(childGen);

	children.push_back(ind);

	//Start crossing.
	for (int i=0;i<(populationSize-1);i++){

		int ind1=roll_weighted_die();
		int ind2=roll_weighted_die();
		Individual* ind = individuals[ind1]->cross(individuals[ind2]);
		orderedInsert(children,ind);
	}

	//Make some mutations on all the elements but the first (best one)
	
	for (int i=0;i<populationSize;i++){
		int randNum = rand()%100; //random number between 0 and 100;
		if (randNum<mutationPerc){
			Individual* ind = new Individual(children[i]);
			ind->mutate();
			orderedInsert(children,ind);
		}
	}


	for (int i=populationSize; i< children.size();i++){
		delete children[i];
	}

	children.resize(populationSize);

	//substitute the generation
	for (int i=0;i<populationSize;i++){
		delete individuals[i]; //delete reserved space
		individuals[i]=children[i]; //assign the new created spacefor (int i=0;i<genSize;i++){
	}
}


void Generation::generateProbWeights(){
	double* probWeights;
	probWeights = new double[populationSize];
	double weight;
	double weightSum=0;
	for (int i=0;i<populationSize;i++){
		weight=1.0/(pow(convergenceFactor,float(i+1)));
		weightSum+=weight;
		probWeights[i]=weight;
	}

	for (int i=0;i<populationSize;i++){
		probWeights[i]=probWeights[i]/weightSum;
	}

	partial_sum(&probWeights[0], &probWeights[0] + populationSize, back_inserter(cumulativeProb));

	delete [] probWeights;


}

/**
 * Now define a function that simulates rolling this die.
 * \return A value between 0 and populationSize-1
*/
int Generation::roll_weighted_die(){
	boost::uniform_real<> dist(0, cumulativeProb.back());
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(generator, dist);
	/* Find the position within the sequence */
	return (lower_bound(cumulativeProb.begin(), cumulativeProb.end(), die()) - cumulativeProb.begin());
}



void Generation::orderedInsert(vector<Individual*>& population, Individual* ind){
	vector<Individual*>::iterator it;

	double indCost = ind->getCost();

	for (it=population.begin(); it!= population.end();){
		double currCost1 = (*it)->getCost();
		//Check if it must be inserted in the first place
		if ( (indCost) <= (currCost1) ){
			population.insert(it,ind);
			return;
		}
		it++; //next element
		if (it == population.end()) break; //we have arrived to the end, so end for loop

		double currCost2 = (*it)->getCost();

		//check if it must be inserted in the middle
		if ( (currCost1 <= (indCost))
		   && (currCost2 >= (indCost)))
		{
			population.insert(it,ind);
			return;
		}
	}

	//if not it means its greater than all so push back
	population.push_back(ind);
}

} // end namespace

