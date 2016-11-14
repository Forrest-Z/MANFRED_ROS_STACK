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
#ifndef __MRCORE__FMGRIDMAP_H
#define __MRCORE__FMGRIDMAP_H

#include <iostream>
#include <math.h>
#include <cmath>
#include <mrcore.h>
#include <map>
//#include "base/globject.h"
//#include "gridmap.h"

using namespace std;

namespace mr{
	
class Image;

class FMPixelData{

	public:

	enum Status{ NON_VISITED=0, NARROW_BAND, FROZEN};
	FMPixelData(){d=0;status=NON_VISITED;}
	FMPixelData(double d, Status status){this->d=d;this->status=status;}

	double d;
	Status status;
	int x,y;

};

	
class FMGridMap : public GridMap<FMPixelData>{

	///Text stream serializers (files)
	friend ostream& operator << (ostream& os,const FMGridMap& gridMap);

public:

	FMGridMap(LabeledGridMap* lmap, double max_speed=1, double safe_distance=2){ this->safe_distance=safe_distance, this->max_speed=max_speed;heap.clear() ; buildFromLabeledGridMap(lmap); }

	~FMGridMap(){};
	
	/**
	 * Creates a Image from the gridmap keeping the proportions of width width
	 * @param width Width of the image (proportions will be kept. If no value is passed it keeps the map width.
	 */
	Image* convertToImage(int w=0) const;

	/**
	 * Applies Fast Marching to the LabeledGridMap
	 */
	void buildFromLabeledGridMap(LabeledGridMap* lmap);

	void drawGL();
	void writeToStream(Stream& stream);
	void readFromStream(Stream& stream);
	void printGrid();
private:
		
	void insertIntoHeap(FMPixelData* pD);
	vector<FMPixelData*>::iterator findElement(FMPixelData* pD);
	void changeLocation(FMPixelData* pD, double new_dist);
	vector<FMPixelData*> heap;
	double max;
	double computeDistance(int x, int y);
	double max_speed;
	double safe_distance;
	
};


}//end namespace

#endif
