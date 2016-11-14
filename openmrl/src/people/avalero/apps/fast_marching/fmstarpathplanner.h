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

#ifndef MR_FMSTARPATHPLANNER_H
#define MR_FMSTARPATHPLANNER_H


#include "pathplanner2d.h"
#include "fmgridmap.h"

namespace mr{

class FMStarPathPlanner2d : public PathPlanner2d {

public:

	FMStarPathPlanner2d(LabeledGridMap* map, bool star = true){
		this->map=map;
		map_changed = true;
		fmmap=NULL; fm2map=NULL;
		path_found=false;
		maxSpeed=1.5; safeDistance=2;
		this->star = star;
	}
	
	~FMStarPathPlanner2d(){
		if (fmmap!=NULL) delete fmmap;
		if (fm2map!=NULL) delete fm2map;
	} 

	bool virtual computePath(const Pose& initPose, const Pose& targetPose);
	
	inline void configure(double maxSpeed, double safeDistance){path_found = false; this->maxSpeed=maxSpeed; this->safeDistance=safeDistance;}

	inline FMGridMap* getPointerToFMMap() const {
		if (path_found) return fmmap;
		else return NULL;
	}

	inline FMGridMap* getPointerToFM2Map() const {
		if (path_found) return fm2map;
		else return NULL;
	}

	inline double getPathEstimatedTime() const {
		if (path_found) return estimatedTime;
		else return DBL_MAX;
	}

	inline double getPathLength() const {
		if (path_found) return pathLength;
		else return DBL_MAX;
	}

	inline void setPointerToMap(LabeledGridMap* map){
		path_found = false;
		this->map = map;
		map_changed = true;
		//computeFastMarching();
	}

	inline void mapChanged(){
		path_found = false;
		if (fmmap!=NULL){
			delete fmmap;
			fmmap=NULL;
		}
		if (fm2map!=NULL){
			delete fm2map;
			fm2map=NULL;
		}
		
		map_changed = true;
	}
	
protected:

	void computeFastMarching();
	void computeFM2(int,int,int,int);

private:
	bool path_found;
	FMGridMap* fmmap;
	FMGridMap* fm2map;
	double maxSpeed; //!< max speed the robot may reach. It's used to saturate the fmmap.
	double safeDistance; //!< distance at which max speed is reached.
	double estimatedTime; //!< estimated time to complete the calculated path
	double pathLength; //!< Path length (stupid comment)
	bool star; //!< Applies the FM* heuristic to compute the path
	bool map_changed;
};


}//end namespace
#endif
