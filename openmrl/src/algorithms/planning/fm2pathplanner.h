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

#ifndef MR_FM2PATHPLANNER_H
#define MR_FM2PATHPLANNER_H


#include "algorithms/planning/pathplanner2d.h"
#include "datatype/gridmap_includes.h"

namespace mr{

class FM2PathPlanner2d : public PathPlanner2d {

public:

	 /**
		* \brief Parametrized Constructor
		* \param map The map used to compute the path
		* \param star Should the planner appy the FMStar heursitic"
		* \param directional Should the planner apply the directional FM2 (dFM2)
		*/
	FM2PathPlanner2d(LabeledGridMap* map, bool star = false, bool directional=false):
		 PathPlanner2d(map),_star(star),_directional(directional){
		_map_changed = true;
		_fmmap=NULL;
		_fm2map=NULL;
		_path_found=false;
		_maxSpeed=1.5;
		_safeDistance=2;
	}
	
	/**
	  * \brief Default Constructor
	  */
	virtual ~FM2PathPlanner2d(){
		if (_fmmap!=NULL) delete _fmmap;
		if (_fm2map!=NULL) delete _fm2map;
	} 

	/**
	  * \brief Computes path between initPose and targetPose
	  * \param initPose Initial Pose
	  * \param targetPose Target Pose
	  * \return True if there is a path. Otherwise, false.
	  */
	bool virtual computePath(const Pose& initPose, const Pose& targetPose);
	
	/**
	  * \brief Configures de Planner
	  * \param maxSpeed Maximum Speed the robot may reach.
	  * \param safeDistance The distance from obstacles at which the robot may reach max speed
	  * \param star Apply FMStar heuristic.
	  * \param directional Apply dFM2
	  */
	inline void configure(double maxSpeed, double safeDistance, bool star=false, bool directional = false){
		 _star=star;
		 _directional=directional;
		 _path_found = false;
		 _maxSpeed=maxSpeed;
		 _safeDistance=safeDistance;
	}

	/**
	  * \brief Returns Pointer to FMMap
	  * \return Pointer to FMMap. If it has not been computed, returns NULL
	  */
	inline FMGridMap* getPointerToFMMap() const {
		if (_path_found) return _fmmap;
		else return NULL;
	}

	/**
	  * \brief Returns Pointer to FM2Map
	  * \return Pointer to FM2Map. If it has not been computed, returns NULL
	  */
	inline FMGridMap* getPointerToFM2Map() const {
		if (_path_found) return _fm2map;
		else return NULL;
	}

	/**
	  * \brief Returns path estimated completion time
	  * \return path estimated completion time. If path has not been found returns DBL_MAX
	  */
	inline double getPathEstimatedTime() const {
		if (_path_found) return _estimatedTime;
		else return DBL_MAX;
	}

	/**
	  * \brief Returns path length
	  * \return Path length. If path has not been found returns DBL_MAX
	  */
	inline double getPathLength() const {
		if (_path_found) return _pathLength;
		else return DBL_MAX;
	}

	/**
	  * \brief Passes the map to use for computing path
	  * \param map Labeled Grid Map.
	  */
	inline void setPointerToMap(LabeledGridMap* map){
		_path_found = false;
		this->map = map;
		mapChanged();
	}

	/**
	  * \brief Informs the planner that the map has changed
	  */
	inline void mapChanged(){
		_path_found = false;
		if (_fmmap!=NULL){
			delete _fmmap;
			_fmmap=NULL;
		}
		if (_fm2map!=NULL){
			delete _fm2map;
			_fm2map=NULL;
		}
		
		_map_changed = true;
	}
	
protected:
	/**
	  * \brief Computes the first Fast Marching
	  */
	void computeFM();
	/**
	  * \brief Computes the second Fast Marching between target and init
	  * \param targetX, x coordinate of the target
	  * \param targetY, y coordinate of the target
	  * \param initX, x coordinate of the init
	  * \param initY, y coordinate of the init
	  */
	void computeFM2(int targetX, int targetY, int initX, int initY);

private:
	bool _path_found;
	FMGridMap* _fmmap;
	FMGridMap* _fm2map;
	double _maxSpeed; //!< max speed the robot may reach. It's used to saturate the fmmap.
	double _safeDistance; //!< distance at which max speed is reached.
	double _estimatedTime; //!< estimated time to complete the calculated path
	double _pathLength; //!< Path length (stupid comment)
	bool _star; //!< Applies the FM* heuristic to compute the path
	bool _directional; //!<Compute Directional Fast Marching
	bool _map_changed;
};


}//end namespace
#endif
