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

#ifndef INDIVIDUAL_H_
#define INDIVIDUAL_H_

#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <iostream>

using namespace std;

class Individual {
public:
	Individual(double** costs , int num_cities, int num_travellers=1);
	Individual(Individual* ind);
	virtual ~Individual();

	inline double getCost(){return cost;}
	/**
	 * Sets random values to the gen
	 */
	void randomGen();

	/**
	 * Sets the gen of this individual.
	 * If there was a previous gen, it deletes it.
	 */
	inline void setGen(int * g){
		if (gen != NULL) delete [] gen;
		gen = g;
		computeCost();
	}

	/**
	 * Return pointer to the gen
	 */
	inline int* getGen(){return gen;}


	/**
	 * Crosses this individual with the passed individual, resulting in a new individual
	 */
	Individual* cross(Individual*);

	/**
	 * Mutates the current individual changing its gene
	 */
	void mutate();

	inline int getGenSize(){return genSize;}

	void sortGen();
	//void setCostsTable(double** costs){costsTable=costs; computedCost=false;}

	int getNumTravellers(){return num_travellers;}

private:
	/**
	 * Sets the size of the gen
	 */
	void setGenSize(int s);
	/**
	 * Mutation Operator.
	 * Reallocates a Substring of the gen.
	 */
	void reallocateSegment();
	void computeCost(); //!< Compute cost for this individual
	void mutateSubPath(); //!< Mutate a part of the gen
	/**
	 * Mutation Operator
	 * Exchanges two subpaths of different travellers
	 */
	void pathsCrossing();

	double cost; //!< Cost of this individual
	int * gen; //!< Gen codification
	int genSize; //!< Size of the gen
	double** costsTable; //!< Table of costs

	int num_cities;
	int num_travellers;

};

#endif /* INDIVIDUAL_H_ */
