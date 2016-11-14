#include "individual.h"
#include "genericvirtualrangesensor.h"


extern int debug;

namespace mr{

IMPLEMENT_MRL_OBJECT(Individual)

Individual::Individual(mr::LabeledGridMap* _storedMap_ptr, const GenericRangeData & _sensorReadings ){
	// Initializing atributes:
	storedMap_ptr = _storedMap_ptr;
	sensorReadings = _sensorReadings;
	cost = 0;
	chromosome.drawGLSize = WIDTH;
}

void Individual::setChromosome(mr::Pose chr){
	chromosome = chr;
	chromosome.drawGLSize = WIDTH;
	computeCost();
}

/*
* Create a random chromosome, which is a location in the map (x,y,theta)
*/
void Individual::randomChromosome(){
	// First we must know the boundaries of the map:
	double minX, minY, maxX, maxY;
	// These values are obtained from the coordinates of the BL and the TR corner of the map

	minX = storedMap_ptr->getMapX();
	minY = storedMap_ptr->getMapY();

	maxX = minX + storedMap_ptr->getRealWidth();
	maxY = minY + storedMap_ptr->getRealHeight();

	chromosome.x = 0.0;//getBoundedRand(-0.05 , 0.05);
	chromosome.y = 0.3;//getBoundedRand(0.25 , 0.35);
	//chromosome.theta = 0;//getBoundedRand(-M_PI , M_PI);

	//~ chromosome.x = getBoundedRand(minX , maxX);
	//~ chromosome.y = getBoundedRand(minY , maxY);
	chromosome.theta = getBoundedRand(-M_PI , M_PI);
	
	computeCost();


	
	if (debug > 1) cout << "   (Debug 2)   " << "Creating new random individual ( " << chromosome.x << " , "<< chromosome.y << " , "<< chromosome.theta << " )" << endl;

}

/*
 * 	getBoundedRnd 
 * Returns a random number between minL and maxL.
 * Note: the result is a number with a precission of 3 decimals
 */
double Individual::getBoundedRand(float minL, float maxL){
	
	float range = (maxL - minL);
	double randomDouble = range * double(rand())/RAND_MAX + minL;

	if (debug > 2) cout << "      (Debug 3)   : " << "getBoundedRand (" << minL << " , " << maxL << ") -> " << randomDouble << endl;

	return randomDouble;
}


/*
 * 	computeCost
 * The cost of an individual is the standard deviation of the virtual readings respect to the real readings
 * standard deviation = sqrt( (1/n) * sum( (realDistance[i] - virtualdistance[i])^2 ) )
 */
void Individual::computeCost(){
	// GenericVirtualRangeSensor(mr::LabeledGridMap * Map_ptr, mr::Pose rPose,
	GenericVirtualRangeSensor virtualSensor(storedMap_ptr,chromosome);
	
	virtualReadings = virtualSensor.sensorData;
	if (virtualReadings.readings.size() != sensorReadings.readings.size()){
		cout << "ERROR!!!: virtualReadings.size() != sensorReadings.size()\n";
		cout << virtualReadings.readings.size() << " VS " << sensorReadings.readings.size() << endl;
	}
	else{
		double sum = 0;
		if (debug > 2) cout << "      (Debug 3) " << "Logged VS Virtual" << endl;
		for (int i = 0; i < virtualReadings.readings.size(); i++)
		{
			if (debug > 2) cout << "      (Debug 3) " << i << "\t" << sensorReadings.readings[i].range << "\t" << virtualReadings.readings[i].range << "\t" << virtualReadings.readings[i].range - sensorReadings.readings[i].range <<  endl;
			sum += pow( (virtualReadings.readings[i].range - sensorReadings.readings[i].range) , 2);
		}
		cost = sqrt( sum / virtualReadings.readings.size()); //you do not need to do this. Better to avoid an sqrt
		
		//
		if (debug > 2) cout << "      (Debug 3) " <<" Cost of individual: " << cost << endl;
		//
	}
}





Individual Individual::cross(const Individual& ind2){

	// The new individual
	Individual ind(storedMap_ptr, sensorReadings);
	
	// The new chromosome
	mr::Pose newChromosome;
	//~ newChromosome.x = (0.2*chromosome.x + 0.8*ind2.chromosome.x);
	//~ newChromosome.y = (0.2*chromosome.y + 0.8*ind2.chromosome.y);
	newChromosome.x = (chromosome.x + ind2.chromosome.x)/2.0;
	newChromosome.y = (chromosome.y + ind2.chromosome.y)/2.0;
	double angle1 = chromosome.theta.getValue();
	double angle2 = ind2.chromosome.theta.getValue();
	newChromosome.theta.setValue((angle1 + angle2)/2);
	
	//sensor.sensorPose.theta.setValue(sensorPoseTheta);

	ind.setChromosome(newChromosome);

	return ind;

}

std::ostream & operator << (ostream & os, const Individual & i){
		os << i.chromosome.x << "\t";
		os << i.chromosome.y << "\t";
		os << i.chromosome.theta << endl;
		return os;
}

void Individual::drawGL2D() const{
	 //chromosome.setRGB(0,0,0);
	 chromosome.drawGL2D();
}

}//end namespace



