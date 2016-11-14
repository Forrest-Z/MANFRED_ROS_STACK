#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H

#include <iostream>
#include <datatype/gridmap/labeledgridmap.h>
//#include <mrcore.h>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include <vector>
#include "datatype/object.h"
#include "datatype/sensordata_includes.h"

#define WIDTH 0.02


using namespace std;

namespace mr{

class Individual: public Object{

	 DECLARE_MRL_OBJECT(Individual)

	 //ostream for standard output
	 friend std::ostream & operator << (ostream & os, const Individual & i);
public:
	 Individual(){chromosome.drawGLSize = WIDTH;}
	// With this constructor, the chromosome is set arbitrarily
	Individual(mr::LabeledGridMap * _storedMap_ptr, const GenericRangeData & _sensorReadings );
	virtual ~Individual(){}; // TODO Delete all the data

	double cost;	// Root Square Distance
	void computeCost();
	/*
	* Sets random values to the Chromosome
	*/
	void randomChromosome();
	
	/**
	 * Crosses this individual with the passed individual, resulting in a new individual
	 * @param ind The individual to cross with. Is a reference to save time
	 */
	Individual cross(const Individual& ind2);
	
	void setChromosome(mr::Pose chr);

	void drawGL2D() const;

	// FIXME: This should be private
	GenericRangeData sensorReadings;			// Used with the map to test cost
	GenericRangeData virtualReadings;			// Theoretical view of the world of the individual, created ONLY with info from the map
	
private:
	mr::Pose chromosome;	// The virtual position of each individual
	mr::LabeledGridMap * storedMap_ptr;	// The map is used to create the random individuals	

	double getBoundedRand(float minL, float maxL); 	// Returns a random number between minL and maxL

};

}// end namespace


#endif /* INDIVIDUAL_H */
