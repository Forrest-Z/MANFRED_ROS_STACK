#include "customgraphs.h"

namespace mr{

PoseSpeed::PoseSpeed(Pose p, double s, int i, int j):Pose(p),speed(s),i(i),j(j){}

/********************************************
 ***********  POSESPEED GRAPH FUNCS ************
 ********************************************/

//! dist among two Pose nodes
template<> double PoseSpeedNode::costTo(PoseSpeedNode* dest){
	double time = value.timeTo(dest->value);
	//cout << time << endl;
	return time;
}

PoseSpeedGraph::~PoseSpeedGraph(){
	for (int i=0; i<width;i++){
		delete [] nodesGrid[i];
	}
	delete [] nodesGrid;
}

PoseSpeedGraph::PoseSpeedGraph(FMGridMap* fmmap){
	nodesGrid = new PoseSpeedNode**[fmmap->getWidth()];
	for (int i=0; i<fmmap->getWidth();i++){
		nodesGrid[i]=new PoseSpeedNode*[fmmap->getHeight()];
	}
	width = fmmap->getWidth();
	height = fmmap->getHeight();
	createNodes(fmmap);
	createEdges(fmmap);
}

void PoseSpeedGraph::createNodes(FMGridMap* fmmap){
	for (int i=0;i<=fmmap->getWidth()-1;i+=1){
		for (int j=0;j<=fmmap->getHeight()-1;j+=1){
			FMPixelData data = fmmap->getCellValue(i,j);
			double x,y; fmmap->GridToWorld(data.x,data.y,x,y);
			PoseSpeed ps(Pose(x,y),data.d,i,j);
			nodesGrid[i][j] = addNode(ps);
		}
	}
}

void PoseSpeedGraph::createEdges(FMGridMap* fmmap, bool diagonal){
	for (int i=0;i<fmmap->getWidth()-1;i+=1){
		for (int j=0;j<fmmap->getHeight()-1;j+=1){
			if (nodesGrid[i][j]->getValue().speed!=0 && nodesGrid[i+1][j]->getValue().speed!=0){
				PoseSpeedEdge* edge = addEdge(nodesGrid[i][j],nodesGrid[i+1][j]);
			}
			if (nodesGrid[i][j]->getValue().speed!=0 && nodesGrid[i][j+1]->getValue().speed!=0){
				PoseSpeedEdge* edge = addEdge(nodesGrid[i][j],nodesGrid[i][j+1]);
			}
			if (diagonal && nodesGrid[i][j]->getValue().speed!=0 && nodesGrid[i+1][j+1]->getValue().speed!=0){
				PoseSpeedEdge* edge = addEdge(nodesGrid[i][j],nodesGrid[i+1][j+1]);
			}
			if (diagonal && nodesGrid[i+1][j]->getValue().speed!=0 && nodesGrid[i][j+1]->getValue().speed!=0){
				PoseSpeedEdge* edge = addEdge(nodesGrid[i+1][j],nodesGrid[i][j+1]);
			}
		}
	}
	recomputeEdgesWeight();
}


PoseSpeedNode* PoseSpeedGraph::getNodeFromMapCoords(int i, int j){
	if (i < width && j < height) return nodesGrid[i][j];
	else return NULL;
}



} //end namespace
