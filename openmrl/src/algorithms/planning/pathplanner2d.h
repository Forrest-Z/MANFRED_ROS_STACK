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

 #ifndef MR_PATHPLANNER2D_H
 #define MR_PATHPLANNER2D_H

#include "datatype/gridmap/labeledgridmap.h"
#include "datatype/geometry/path2d.h"

namespace mr{

class PathPlanner2d{
public:
	/**
	 * Default constructor
	 */
	PathPlanner2d(){map = NULL;}
	PathPlanner2d(LabeledGridMap* map):map(map){}
	~PathPlanner2d(){};

	/**
	 * It passes the pointer to the labeledgridmap for planning.
	 * This function does not make a copy of the map, so the property of it remains of the main program.
	 */
	inline void setPointerToMap(LabeledGridMap* map){this->map = map;}

	/**
	 * Returns the computed path
	 */
	inline virtual Path2D getPath() const {return path;}

	/**
	 * Computes the path
	 * @param initPose The starting pose.
	 * @param targetPose The target pose (doom comment)
	 * @return True if a path is found, else false.
	 */
	bool virtual computePath(const Pose& initPose, const Pose& targetPose) = 0;

protected:
	Path2D path; //!< The path.
	LabeledGridMap* map; //!< Pointer to the map
};


}//end namespace


#endif
