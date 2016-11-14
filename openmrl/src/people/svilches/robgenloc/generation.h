#ifndef GENERATION_H
#define GENERATION_H

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <algorithm>
#include <numeric>
#include "models/simskybot.h"

namespace mr{
	
class Individual{
	public:
		Individual(){};
		~Individual(){};
		
		SimSkyBot * robot;
		double cost;
};
}

//~ class Generation :public Object{
//~ 
//~ DECLARE_MRL_OBJECT (Generation)
//~ 
//~ public:
	//~ Generation(){}
	//~ Generation(	int _popSize, float _crossFactor, mr::LabeledGridMap * _storedMap_ptr,	const GenericRangeData & _sensorReadings );
	//~ ~Generation(){}; // TODO: Delete all the population
	//~ 
	//~ void generateRandomPopulation();
	//~ 
	//~ /**
	 //~ * Generates a new generation based on crosses, mutations and memory
	 //~ */
	//~ void evolveGeneration();
		//~ 
	//~ friend ostream& operator<<(ostream& os, Generation g);
//~ 
	//~ void drawGL2D();
	//~ 
//~ private:
	//~ /*
	 //~ * At the beginning of each localization procedure, the data coming from the vector is stored in
	 //~ * sensorReadings, and then compared with each one of the position candidates.
	 //~ */
	//~ int popSize;	// Number of individuals coexisting in each generation
	//~ float crossFactor;	// Variable used in the crossing process
	//~ mr::LabeledGridMap * storedMap_ptr;	// The map is used to create the random individuals	
	//~ GenericRangeData sensorReadings;			// Used by the individuals to test their cost
	//~ 
	//~ vector <Individual> population;	
	//~ 
	//~ // Needed for roll_weighted_die:
	//~ vector<double> cumulativeProb; //!< Cumulative Weights for random number
	//~ boost::mt19937 generator; //!< for random number generation
	//~ double convergenceFactor; //!< The higher it is the faster it converges. But it may fall easier in local minima.
		//~ 
		//~ // Member Functions
		//~ 
	//~ void orderedInsert(vector<Individual> * population, const Individual* ind);
	//~ 
	//~ /*
	 //~ * Generates the weight for random number generation
	 //~ */
	//~ void generateProbWeights();
//~ 
	//~ /*
	 //~ *Now define a function that simulates rolling this die.
	 //~ */
	//~ int roll_weighted_die();
//~ };
//~ 
//~ } // namespace mr
	//~ 
#endif /* GENERATION_H */
