/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Alberto Valero
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
#ifndef __MRCORE__FMSTARGRIDMAP_H
#define __MRCORE__FMSTARGRIDMAP_H

#include <iostream>
#include <math.h>
#include <cmath>
#include <mrcore.h>
#include <map>
#include <float.h>
//#include "base/globject.h"
//#include "gridmap.h"

using namespace std;

namespace mr{

class FMGridMap;	
class Image;

class FMStarPixelData{

	public:

	enum Status{ NON_VISITED=0, NARROW_BAND, FROZEN};
	FMStarPixelData(){d=DBL_MAX;optimalCost=DBL_MAX;status=NON_VISITED;}
	FMStarPixelData(double d, double optimalCost, Status status){this->d=d;this->status=status; this->optimalCost=optimalCost;}

	double d;
	double optimalCost;
	Status status;
	int x,y;

};

	
class FMStarGridMap : public GridMap<FMStarPixelData>{

	///Text stream serializers (files)
	friend ostream& operator << (ostream& os,const FMStarGridMap& gridMap);

public:
	FMStarGridMap(FMGridMap* fmmap, int pixelX, int pixelY, int initX, int initY, double max_speed){
		max=0;
		precomputedDists=NULL;
		heap.clear() ;
		this->max_speed = max_speed;
		max_speed_inv = 1/max_speed;
		buildFromFMGridMap(fmmap, pixelX, pixelY, initX, initY);
	}

	~FMStarGridMap(){}
	
	/**
	 * Creates a Image from the gridmap keeping the proportions of width width
	 * @param width Width of the image (proportions will be kept. If no value is passed it keeps the map width.
	 */
	Image* convertToImage(int w=0) const;
	void generatePlotData(string filename) const;

	/**
	 * Applies Fast Marching to the LabeledGridMap
	 */
	
	void buildFromFMGridMap(FMGridMap* fmmap, int targetX, int targetY, int initX, int initY);

	

	
	void drawGL();
	void writeToStream(Stream& stream);
	void readFromStream(Stream& stream);
	void printGrid();
private:
		
	void insertIntoHeap(FMStarPixelData* pD);
	vector<FMStarPixelData*>::iterator findElement(FMStarPixelData* pD);
	void changeLocation(FMStarPixelData* pD, double new_dist);
	vector<FMStarPixelData*> heap;
	void buildPreComputedDistances(FMGridMap* fmmap);
	
	double max;
	double computeDistance(int x, int y, double speed=1);
	double max_speed;
	double max_speed_inv;
	double safe_distance;

	double **precomputedDists;

	
};


}//end namespace

#endif
