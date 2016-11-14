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

#include "individual.h"
#include <ctime>
#include <string.h>
#include <vector>
#include <set>
#include <cfloat>


Individual::Individual(Individual* ind) {
	costsTable=ind->costsTable;
	this->num_cities=ind->num_cities;
	this->num_travellers=ind->num_travellers;
	int genSize=num_cities + num_travellers;
	setGenSize(genSize);
	memcpy(gen,ind->gen,genSize*sizeof(int));
	computeCost();
}

Individual::Individual(double** costs, int num_cities, int num_travellers) {

	costsTable=costs;
	this->num_cities=num_cities;
	this->num_travellers=num_travellers;
	int genSize=num_cities + num_travellers;
	setGenSize(genSize);
	cost=0;
}

Individual::~Individual() {
	if (gen!=NULL) delete [] gen;
}

Individual* Individual::cross(Individual* ind2){
	//a set for avoiding repetitions
	bool* aux_vector = new bool[genSize];
	for (int i=0; i<genSize ;i++) aux_vector[i]=true;

	//the new individual
	Individual* ind = new Individual(costsTable,num_cities,num_travellers);


	int position1=rand()%genSize;
	int position2=rand()%genSize;

	int p1=(position1 <= position2 ? position1:position2 ); //p1 is the min
	int p2=(position1 >= position2 ? position1:position2 ); //p2 is the max

	int* new_gen = new int[genSize];

	//we take the gen part from p1 to p2 of this gen
	for (int i=p1;i<p2;i++){
		new_gen[i]=gen[i];
		aux_vector[ gen[i] ]=false; //do not include it any more
	}


	int* gen2 = ind2->getGen();

	// the rest of gen2
	//from 0 to p1;
	int aux_index=0;
	for (int i=0; i<genSize; i++){
		int aux = gen2[i];
		if (aux_vector[aux]){ //if it was not already included
			if (aux_index==p1) aux_index=p2; //skip the middle part
			new_gen[aux_index]=aux; aux_index++;
		}
	}

	ind->setGen(new_gen);

	delete [] aux_vector;

	return ind;

}

void Individual::setGenSize(int s){
	gen = new int[s];
	genSize=s;
}

void Individual::computeCost(){

	sortGen();

	double * travellersCost = new double[num_travellers];

	for(int i=0;i<num_travellers;i++) travellersCost[i]=0.;

	int traveller=0;
	int startingindex=0;

	for (int i=0; i< genSize-1 ; i++){
		if (gen[i+1] < num_travellers)  // the next is a traveller do not count
		{
			travellersCost[traveller]+=costsTable[ gen[i] ][ gen[startingindex] ];
			traveller++;
			startingindex=i+1;
			continue;
		}
		travellersCost[traveller]+=costsTable[ gen[i] ][ gen[i+1] ];
	}

	travellersCost[traveller]+=costsTable[ gen[genSize-1] ][ gen[startingindex] ];

	double totalPathCost,maxCost,minCost;
	totalPathCost=0; minCost=DBL_MAX; maxCost=0;

	for (int i=0; i< num_travellers; i++)
	{
		totalPathCost+=travellersCost[i];
		minCost=minCost<travellersCost[i]?minCost:travellersCost[i]; //the min
		maxCost=maxCost>travellersCost[i]?maxCost:travellersCost[i]; //the max
	}

	cost = totalPathCost + 0.9*maxCost - 0.9*minCost;

	delete [] travellersCost;
}


void Individual::mutate(){

	int random = rand()%100;

	if (random<20){
		mutateSubPath();
	}
	else if (random < 40)
	{
		reallocateSegment();
	}else{
		pathsCrossing();
	}
}

void Individual::pathsCrossing(){
	if (num_travellers <=1){
		reallocateSegment();
		return;
	}

	int traveller1 = rand()%num_travellers;
	int traveller2 = rand()%num_travellers;

	while (traveller2==traveller1) traveller2 = rand()%num_travellers;

	int beg1, end1;
	beg1=end1=0;
	bool beginFound1=false;
	bool subPathFound1=false;

	for (int i=0;i<genSize;i++){
		if (gen[i] == traveller1 && !beginFound1)
		{
			beg1=i;beginFound1=true;
		}
		else if (gen[i] < num_travellers && beginFound1)
		{
			end1 = i; if( (end1-beg1) == 1) return;
		}
		if (subPathFound1){
			break;
		}

	}

	int beg2, end2;
	beg2=end2=0;
	bool beginFound2=false;
	bool subPathFound2=false;

	for (int i=0;i<genSize;i++){
		if (gen[i] == traveller2 && !beginFound2)
		{
			beg2=i;beginFound2=true;
		}
		else if (gen[i] < num_travellers && beginFound2)
		{
			end2 = i; if( (end2-beg2) == 1) return;

		}
		if (subPathFound2){
			break;
		}

	}

	if(!subPathFound1 || !subPathFound2){
		return;
	}



	int s1position1=rand()%(end1-beg1-1)+beg1+1;
	int s1position2=rand()%(end1-beg1-1)+beg1+1;


	int s1p1=(s1position1 <= s1position2 ? s1position1:s1position2 ); //p1 is the min
	int s1p2=(s1position1 >= s1position2 ? s1position1:s1position2 ); //p2 is the max


	int* segment1 = new int[s1p2-s1p1];

	memcpy(segment1, gen+s1p1, (s1p2-s1p1)*sizeof(int));


	if(rand()%100 < 50)
	{
		int* aux_segment1 = new int[s1p2-s1p1];
		for (int i=0;i<(s1p2-s1p1);i++){
			aux_segment1[i] = segment1[(s1p2-s1p1)-i-1];
		}
		delete [] segment1;
		segment1 = aux_segment1;
	}




	int s2position1=rand()%(end2-beg2-1)+beg2+1;
	int s2position2=rand()%(end2-beg2-1)+beg2+1;


	int s2p1=(s2position1 <= s2position2 ? s2position1:s2position2 ); //p1 is the min
	int s2p2=(s2position1 >= s2position2 ? s2position1:s2position2 ); //p2 is the max

	int* segment2 = new int[s2p2-s2p1];
	memcpy(segment2, gen+s2p1, (s2p2-s2p1)*sizeof(int));


	if(rand()%100 < 50)
	{
		int* aux_segment2 = new int[s2p2-s2p1];
		for (int i=0;i<(s2p2-s2p1);i++){
			aux_segment2[i] = segment2[(s2p2-s2p1)-i-1];
		}
		delete [] segment2;
		segment2 = aux_segment2;
	}

	int* new_gen = new int[genSize];
	if (s1p1 < s2p1){
		memcpy(new_gen, gen, s1p1*sizeof(int));
		memcpy(new_gen+s1p1, segment2, (s2p2-s2p1)*sizeof(int));
		memcpy(new_gen+s1p1+(s2p2-s2p1), gen+s1p2, (s2p1-s1p2)*sizeof(int));
		memcpy(new_gen+s1p1+(s2p2-s2p1)+(s2p1-s1p2),segment1, (s1p2-s1p1)*sizeof(int));
		memcpy(new_gen+s2p2,gen+s2p2, (genSize-s2p2)*sizeof(int) );
	}else{
		memcpy(new_gen, gen, s2p1*sizeof(int));
		memcpy(new_gen+s2p1, segment1, (s1p2-s1p1)*sizeof(int));
		memcpy(new_gen+s2p1+(s1p2-s1p1), gen+s2p2, (s1p1-s2p2)*sizeof(int));
		memcpy(new_gen+s2p1+(s1p2-s1p1)+(s1p1-s2p2),segment2, (s2p2-s2p1)*sizeof(int));
		memcpy(new_gen+s1p2,gen+s1p2, (genSize-s1p2)*sizeof(int) );
	}

	delete [] segment1;
	delete [] segment2;
	setGen(new_gen);

}
void Individual::reallocateSegment(){
	int position1=rand()%genSize;
	int position2=rand()%genSize;


	int p1=(position1 <= position2 ? position1:position2 ); //p1 is the min
	int p2=(position1 >= position2 ? position1:position2 ); //p2 is the max

	int dest = rand()%(genSize-(p2-p1));

	int* aux_gen = new int[genSize-(p2-p1)];

	memcpy(aux_gen,gen,p1*sizeof(int));
	memcpy(aux_gen+p1, gen+p2,(genSize-p2)*sizeof(int));

	int* segment = new int[p2-p1];
	memcpy(segment, gen+p1, (p2-p1)*sizeof(int));

	if(rand()%100 < 50)
	{
		int* aux_segment = new int[p2-p1];
		for (int i=0;i<(p2-p1);i++){
			aux_segment[i] = segment[(p2-p1)-i-1];
		}
		delete [] segment;
		segment = aux_segment;
	}

	int* new_gen= new int[genSize];
	memcpy(new_gen,aux_gen,dest*sizeof(int));
	memcpy(new_gen+dest,segment,(p2-p1)*sizeof(int));
	memcpy(new_gen+dest+(p2-p1),aux_gen+dest,(genSize-(p2-p1)-dest)*sizeof(int));

	delete [] aux_gen;
	delete [] segment;


	setGen(new_gen);

}

void Individual::mutateSubPath(){

	int beg, end;
	beg=end=0;
	int traveller = rand()%num_travellers;
	bool beginFound=false;
	bool subPathFound=false;

	for (int i=0;i<genSize;i++){
		if (gen[i] == traveller && !beginFound)
		{
			beg=i;beginFound=true;
		}
		else if (gen[i] < num_travellers && beginFound)
		{
			end = i; subPathFound=true;
		}
		if (subPathFound){
			break;
		}

	}

	if(!subPathFound) return;


	int position1=rand()%(end-beg)+beg+1;
	int position2=rand()%(end-beg)+beg+1;


	int p1=(position1 <= position2 ? position1:position2 ); //p1 is the min
	int p2=(position1 >= position2 ? position1:position2 ); //p2 is the max

	int dimension=p2-p1;

	int * aux_vector = new int[dimension];

	int counter=0;
	for (int i=p1;i<p2;i++)
	{
		aux_vector[counter]=gen[i];
		counter++;
	}

	counter=dimension-1;
	for (int i=p1;i<p2;i++)
	{
		gen[i]=aux_vector[counter];
		counter--;
	}

	delete [] aux_vector;

	computeCost();


}

void Individual::randomGen(){
	//if gen has not been initialized do it.
	if (gen != NULL) delete [] gen;
	gen = new int[genSize];

	//create a vector with all the indexes to place randomnly
	vector<int> aux_vector;
	for (int i=0;i<genSize;i++) aux_vector.push_back(i);

	//now take them out randomnly and place them into the gen
	for (int i=0;i<genSize;i++){
		int aux = rand() % (genSize-i); // each loop step the vector is smaller
		gen[i]=aux_vector[aux]; //place it into the gen
		aux_vector.erase(aux_vector.begin()+aux);
	}

	computeCost();

}

void Individual::sortGen(){
	//reorder gen to have a traveller in the first place
		// travellers have index < num_travellers;

	if (gen[0]<num_travellers) return;

	int* final_gen = new int[genSize];
	for (int i=0;i<genSize;i++){
		if(gen[i]<num_travellers) //it is a traveller, switch vector poses
		{
			memcpy(final_gen,gen+i, (genSize-i)*sizeof(int));
			memcpy((final_gen + genSize -i), gen, i*sizeof(int));
			break;
		}
	}

	setGen(final_gen);
}
