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

 #ifndef MR_CUSTROMGRAPHS_H
 #define MR_CUSTROMGRAPHS_H

#include <mrcore.h>
#include "fmgridmap.h"

namespace mr{

//!< Speed Graphs Typedefs
class PoseSpeed : public Pose {
	public:
	PoseSpeed(){}
	PoseSpeed(Pose p, double s, int i, int j);
	double speed;
	int i,j;
	inline double timeTo(PoseSpeed &dest){
		double dist = distance(Pose(x,y),Pose(dest.x, dest.y));
		double mean_speed = (speed + dest.speed ) / 2 ;
		double time = dist / mean_speed;
		return time;
	}
};


typedef Node<PoseSpeed,double> PoseSpeedNode;
typedef Edge<PoseSpeed,double> PoseSpeedEdge;
typedef Searcher<PoseSpeed, double> PoseSpeedGraphSearcher;
typedef UninformedSearcher<PoseSpeed,double> UninformedPoseSpeedSearcher;
typedef DijkstraSearcher<PoseSpeed,double> DijkstraPoseSpeedSearcher;
//typedef AstarSearcher<PoseSpeed,double> AstarPoseSpeedSearcher;

class PoseSpeedGraph: public Graph<PoseSpeed,double>{
	public:
		PoseSpeedGraph(FMGridMap* fmmap);
		~PoseSpeedGraph();
		PoseSpeedNode* getNodeFromMapCoords(int i, int j);
	protected:
		void createNodes(FMGridMap* fmmap);
		void createEdges(FMGridMap* fmmap,bool diagonal=true);
	private:
		PoseSpeedNode *** nodesGrid;
		int width;
		int height;
};

class AstarPoseSpeedSearcher : public AstarSearcher<PoseSpeed,double>{
	public:

		AstarPoseSpeedSearcher(PoseSpeedGraph* g, const double& z, const double& i, const double &maxSpeed):AstarSearcher<PoseSpeed,double>(g,z,i),max_speed(maxSpeed){}

		inline double computeMinCostToGo(PoseSpeedNode *currentNode, PoseSpeedNode *targetNode){
			double x = currentNode->getValue().x;
			double y = currentNode->getValue().y;
			double targetX = targetNode->getValue().x;
			double targetY = targetNode->getValue().y;

			double dist = distance(currentNode->getValue(),targetNode->getValue());
			return dist/max_speed;
		}
		
	private:
		double max_speed;
};

}

#endif
