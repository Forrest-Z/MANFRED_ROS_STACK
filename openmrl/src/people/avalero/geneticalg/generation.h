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

#ifndef GENERATION_H_
#define GENERATION_H_

#include "individual.h"
#include <vector>
#include "./boost/random/mersenne_twister.hpp"
#include "./boost/random/uniform_real.hpp"
#include "./boost/random/variate_generator.hpp"
#include <algorithm>
#include <numeric>

using namespace std;

class Generation {
public:
	Generation(int population_size, int num_cities, int num_travellers, double** costs =NULL, double mutation=20, double cF=1.05);

	virtual ~Generation();
	/**
	 * Sets the convergence factor for generating the random crosses
	 */
	inline void setConvergenceFactor(double cF){convergenceFactor=cF;}
	/**
	 *Sets the percentages of mutations in a gneration
	 */
	inline void setMutationPerc(double mut){mutationPerc=mut;}

	/**
	 * Generates a new generation with random individuals.
	 */
	void generateRandomPopulation();
	/**
	 * Generates a new generation based on crosses, mutations and memory
	 */
	void generateNewGeneration();
	void inmigration(int numPopulations, int numIterations, int inmigrationSize);
	
	void generateInitGeneration(int numPopulations, int numInterations);

	/**
	 * Returns a copy of the individuals
	 */
	inline vector<Individual*> getIndividuals(){return individuals;}

	/**
	 * Returns a pointer to the best individual
	 */
	inline Individual* getBestIndividual(){return individuals[0];}

	inline void setCostsTable(double** costs){costsTable = costs;}
	inline void setConvFactor(double cF){convergenceFactor=cF;}

private:
	/**
	 * Sets the number of individuals in a generation.
	*/
	
	inline void setPopulationSize(int size){populationSize=size; generateProbWeights();}
	/**
	 * Generates the weight for random number generation
	 */
	void generateProbWeights();

	/**
	 *Now define a function that simulates rolling this die.
	 */
	int roll_weighted_die();

	/**
	 * Inserts a new individual in its adequated ortered postion.
	 */
	void orderedInsert(vector<Individual*>&, Individual*);

	double mutationPerc;//!< Percentage of mutations in a generation.
	int populationSize;//!< Number of individuals
	vector<Individual*> individuals; //!< Members of the generation

	vector<double> cumulativeProb; //!< Cumulative Weights for random number
	double convergenceFactor; //!< The higher it is the faster it converges. But it may fall easier in local minima.
	boost::mt19937 generator; //!< for random number generation
	double **costsTable;
	int genSize;
	int num_travellers;
	int num_cities;
};

#endif /* GENERATION_H_ */
