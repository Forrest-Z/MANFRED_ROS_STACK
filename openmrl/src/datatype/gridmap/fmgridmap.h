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
#include <map>
#include <float.h>
#include "gridmap.h"
#include "labeledgridmap.h"
//#include "base/globject.h"
//#include "gridmap.h"

using namespace std;

namespace mr{
	
class Image;

class FMPixelData{

	public:

	enum Status{ NON_VISITED=0, NARROW_BAND, FROZEN};
	FMPixelData(){
		d=DBL_MAX;
		optimalCost=DBL_MAX;
		status=NON_VISITED;
	}
	
	FMPixelData(double d, Status status){
		this->d=d;
		this->optimalCost=d;
		this->status=status;
	}
	
	FMPixelData(double d, double optimalCost, Status status){
		this->d=d;
		this->status=status;
		this->optimalCost=optimalCost;
	}
	
	double optimalCost; //!< distance to origing + optimal distance to target (in time)
	double d; //!< distance to origin (in time)
	Status status; //!< status of the pixel: NON_VISITED, NARROW_BAND, FROZEN
	int x,y; //!< x,y coordinates of the pixel in the map (redundant but neccessary)

};

/**
 * A grid map that stores Fast Marching Information.
 * It may be constructed from a Labeled GridMap. In that case the origin of waves is on the obstacles.
 * It may be constructed from a FMGRidMap. In that case there is one wave origin, and the wave speed propagation is given by the FMGridMap
 */
class FMGridMap : public GridMap<FMPixelData>, public Object{

	 DECLARE_MRL_OBJECT(FMGridMap)

	///Text stream serializers (files)
	friend ostream& operator << (ostream& os,const FMGridMap& gridMap);

public:

	/**
	 * Constructor
	 */
	FMGridMap(){
		precomputedDists=NULL;
		max=0;
	}

	/**
	 * Destructor
	 */
	~FMGridMap(){
		freeall();
	}

	
	/**
	 * Computes the FMGridMap from a Labeled GridMap. All obstacles are the waves origin.
	 * The resultin grimmap is a "slowness" map.
	 * It saturates at max_speed. Max Speed is reached at safe distance from obstales.
	 */ 
	void computeFM(LabeledGridMap* lmap, double max_speed=1, double safe_distance=1.5){
		max=0;
		heap.clear() ;
		//precomputedDists=NULL;
		freeall(); //free al allocated memory

		this->safe_distance=safe_distance;
		this->max_speed=max_speed;
		max_speed_inv = 1/max_speed;

		buildFromLabeledGridMap(lmap); //fill the gridmap

	}


	
	/**
	 * Computes the FMGridMap from a FM GridMap. There is only one origin.
	 * The resulting grimmap is a "conoid" map having only one minima at pixelX, pixelY.
	 */ 
	inline void computeFM2(FMGridMap* fmmap, int pixelX, int pixelY, bool directed=false){
		max=0;
		heap.clear() ;
		freeall(); //free al allocated memory

		buildFromFMGridMap(fmmap, pixelX, pixelY,directed); // fill the fmgridmap
	}

	/**
	 * Computes the FMGridMap from a FM GridMap. There is only one origin.
	 * The resulting grimmap is a "conoid" map having only one minima at pixelX, pixelY.
	 * The wave is only propagated until it reaches initX, initY
	 * By default applies the FM* heuristic. If star is false it applies normal FM (Djikstra equivalent).
	 */
	inline void computeFM2(FMGridMap* fmmap, int targetX, int targetY, int initX, int initY, double max_speed=1, bool star = false, bool directional=false){
		max=0;
		heap.clear() ;
		freeall(); //free al allocated memory
		this->max_speed = max_speed;
		max_speed_inv = 1/max_speed;

		if(star){
			buildFromFMGridMapStar(fmmap, targetX, targetY, initX, initY, directional);
		}else{
			buildFromFMGridMap(fmmap, targetX, targetY, initX, initY,directional);
		}
	}
	
	/**
	 * Creates a Image from the gridmap keeping the proportions of width width
	 * @param width Width of the image (proportions will be kept. If no value is passed it keeps the map width.
	 */
	Image* convertToImage(int w=0) const;

	void generatePlotData(string filename, bool crop=false) const;
	void printGrid();
	inline double getRealMaxSpeed() const {return real_max_speed;}


	protected:

	void freeall(){
		if (precomputedDists!=NULL){
			for (int i=0; i<width; i++) delete [] precomputedDists[i];
			delete [] precomputedDists;
		}

		precomputedDists=NULL;
	}
	
	/**
	 * Builds a matrix with precomputed distances from one point to other
	 */
	void buildPreComputedDistances(double speed);
	
	void buildFromLabeledGridMap(LabeledGridMap* lmap);
	void buildFromFMGridMap(FMGridMap* fmmap, int pixelX, int pixelY, bool directional = false);
	void buildFromFMGridMap(FMGridMap* fmmap, int targetX, int targetY, int initX, int initY, bool directional = false);
	void buildFromFMGridMapStar(FMGridMap* fmmap, int targetX, int targetY, int initX, int initY,bool directional = false);

	void drawGL2D() const;
	

private:
		
	void insertIntoHeap(FMPixelData* pD);
	void insertIntoHeapStar(FMPixelData* pD);
	
	vector<FMPixelData*>::iterator findElement(FMPixelData* pD);
	vector<FMPixelData*>::iterator findElementStar(FMPixelData* pD);
	
	void changeLocation(FMPixelData* pD, double new_dist);
	void changeLocationStar(FMPixelData* pD, double new_dist);
	
	vector<FMPixelData*> heap;
	
	double max;
	double computeDistance(int x, int y, double speed=1);
	//double computeHADistance(int x, int y, double speed=1);
	
	double max_speed;
	double safe_distance;
	double max_speed_inv;
	double real_max_speed;

	double **precomputedDists;
	
};


}//end namespace

#endif
