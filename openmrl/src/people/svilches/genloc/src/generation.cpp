#include "generation.h"

//~ namespace GnuPlot{
	//~ class GnuPlotGui;
//~ }

extern int debug;

namespace mr{

IMPLEMENT_MRL_OBJECT(Generation)

Generation::Generation(	int _popSize, 
			float _crossFactor, 
			mr::LabeledGridMap * _storedMap_ptr,
			const GenericRangeData & _sensorReadings ){

	// Initializing atributes:
	popSize = _popSize;
	crossFactor = _crossFactor;
	storedMap_ptr = _storedMap_ptr;
	sensorReadings = _sensorReadings;
	//~ convergenceFactor = 1.05; // FIXME
	convergenceFactor = 1.5; // FIXME
	generateProbWeights();
	
	/*
	 * Create a  map
	 */

	// First we must know the boundaries of the map:
	double minX, minY, maxX, maxY;
	// These values are obtained from the coordinates of the BL and the TR corner of the map
	// GridToWorld (int gridX, int gridY, double &worldX, double &worldY)
	storedMap_ptr->GridToWorld( 0 , 0 , minX , minY );
	storedMap_ptr->GridToWorld( storedMap_ptr->getWidth() -1 , storedMap_ptr->getHeight() -1 , maxX , maxY );

	//::Gui	gpMap;
	//gpMap.set_xrange(minX,maxX);
	//gpMap.set_yrange(minY,maxY);
}





void Generation::generateRandomPopulation(){
	if (debug) cout << "(Debug 1)       " << "Creating initial random generation" << endl;

	for (int i=0;i<popSize;i++){
		Individual ind(storedMap_ptr, sensorReadings);
		ind.randomChromosome();
		if (debug > 1) cout << "   (Debug 2)    " <<" Cost of individual " << i << " is: " << ind.cost << endl;
		orderedInsert(&population, &ind);	//TODO: Change passing by address to C++ style
	}
}	


void Generation::evolveGeneration(){
	//
	if (debug > 1) cout << "   (Debug 2)    " <<" Creating a new generation from previous one "<< endl;
	//
	vector<Individual> children;

	//Start crossing.
	for (int i=0;i<popSize;i++){
		int ind1=roll_weighted_die();
		int ind2=roll_weighted_die();
		Individual ind = population[ind1].cross(population[ind2]);
		orderedInsert(& children, & ind);
	}

	//substitute the generation
	for (int i=0;i<popSize;i++){
		population[i]=children[i]; //assign the new created spacefor (int i=0;i<genSize;i++)
		population[i].computeCost();
		//
		if (debug > 1) cout << "   (Debug 2)    " <<" Cost of individual " << i << " is: " << population[i].cost << endl;
		//
	}
	//
	if (debug > 2) cout << "      (Debug 3) " <<" New generation created "<< endl;
	//
}


/**********************************************************************
 *
 * This code (orderedInsert) is based on the work of
 *          Alberto Valero Gomez (alberto.valero.gomez@gmail.com)
 *			Julio Valero Gomez
 ***********************************************************************/
void Generation::orderedInsert(vector<Individual> * population, const Individual* ind){
	vector<Individual>::iterator it;
	for (it=population->begin(); it!= population->end();){
		double currCost1 = (*it).cost;
		//Check if it must be inserted in the first place
		if ( (ind->cost) <= (currCost1) ){
			population->insert(it,*ind);
			return;
		}
		it++; //next element
		if (it == population->end()) break; //we have arrived to the end, so end for loop

		double currCost2 = (*it).cost;

		//check if it must be inserted in the middle
		if ( (currCost1 <= (ind->cost))
		   && (currCost2 >= (ind->cost)))
		{
			population->insert(it,*ind);
			return;
		}
	}

	//if not it means its greater than all so push back
	population->push_back(*ind);
}

/**
 * Now define a function that simulates rolling this die.
 * return A value between 0 and popSize-1
*/
int Generation::roll_weighted_die(){
	boost::uniform_real<> dist(0, cumulativeProb.back());
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(generator, dist);
	/* Find the position within the sequence */
	return (lower_bound(cumulativeProb.begin(), cumulativeProb.end(), die()) - cumulativeProb.begin());
}


// Used by roll_weighted_die()
void Generation::generateProbWeights(){
	double* probWeights;
	probWeights = new double[popSize];
	double weight;
	double weightSum=0;
	for (int i=0;i<popSize;i++){
		weight=1.0/(pow(convergenceFactor,float(i+1)));
		weightSum+=weight;
		probWeights[i]=weight;
	}

	for (int i=0;i<popSize;i++){
		probWeights[i]=probWeights[i]/weightSum;
	}

	partial_sum(&probWeights[0], &probWeights[0] + popSize, back_inserter(cumulativeProb));

	delete [] probWeights;
}



	//std::cout << "-----------------------------"<< endl;
	//std::cout << "Population of the generation:" << endl;
	//cout << "0" << endl; // Debugging FIXME
	//os << g.popSize << endl; // Debugging FIXME
	//for (int i = 0; i < g.popSize; i++)
	//{
		//os << "a" << endl; // Debugging FIXME
		//cout << "1" << endl; // Debugging FIXME
		//os << "Individual " << i << ": Cost " << g.population[i].cost << "\n";
		//os << g.population[i] << endl;
		//cout << "2" << endl; // Debugging FIXME
	//}
	//os << "b" << endl; // Debugging FIXME


ostream& operator << (ostream & os, Generation g){
	os << "In operator <<" << endl; // Debugging FIXME
	
	os << "-----------------------------"<< endl;
	os << "Population of the generation:" << endl;
	for (int i = 0; i < g.popSize; i++)
	{
		os << "Individual " << i << ": Cost " << g.population[i].cost << "\n";
		os << g.population[i] << endl;
	}
	// FIXME Quick Fix: Without this line, the program segfaults... don't know why!
	return os;

}

void Generation::drawGL2D(){
	for (int i = 0; i < popSize; i++)
	{
		population[i].drawGL2D();
	}
	
	
}

} // namespace mr
