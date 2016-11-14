#include <mrcore.h>
#include "fmgridmap.h"
#include "customgraphs.h"
#include <float.h>
#include <iomanip>

using namespace mr;
using namespace std;

class Point2i{
	public:
	Point2i(unsigned char x, unsigned char y){this->x=x; this->y=y;}

	unsigned char x;
	unsigned char y;
};

vector<Point2i> queue;

int main(void){

	LabeledGridMap map(500,500,1,LabeledGridMap::FREE);

	

//	for (int i=0;i<map.getWidth();i++){
	//		map.setCellValue(i,4,LabeledGridMap::OBSTACLE);
//	}

	for (int i=1;i<499;i++) map.setCellValue(100,i,LabeledGridMap::OBSTACLE);
	int x_center,y_center;
	x_center = map.getWidth()/2;
	y_center = map.getHeight()/2;
	map.setCellValue(x_center,y_center,LabeledGridMap::OBSTACLE);
	//map.setCellValue(x_center+1,y_center,LabeledGridMap::OBSTACLE);
	//map.setCellValue(x_center-1,y_center,LabeledGridMap::OBSTACLE);
	//map.setCellValue(x_center+1,y_center-1,LabeledGridMap::OBSTACLE);
	//map.setCellValue(x_center+1,y_center+1,LabeledGridMap::OBSTACLE);

	//map.setCellValue(0,0,LabeledGridMap::OBSTACLE);

	Image* img = new Image;
	img->load("initmap.bmp");

	map.loadFromImage(img,500,1);

	MRTime timer;
	
	timer.precistic();

	FMGridMap fmmap(&map);

	
	//fmmap.printGrid();
	
	cout << "Fast Marching Total time: " << timer.precistoc() << endl;

	delete img;
	img = fmmap.convertToImage();
	
//	Image* img;
	
	


	timer.precistic();
	PoseSpeedNode *** nodesGrid;
	nodesGrid = new PoseSpeedNode**[fmmap.getWidth()];
	for (int i=0; i<fmmap.getHeight();i++){
		nodesGrid[i]=new PoseSpeedNode*[fmmap.getHeight()];
	}
	
	PoseSpeedGraph graph;
	for (int i=0;i<fmmap.getWidth();i++){
		for (int j=0;j<fmmap.getHeight();j++){
			FMPixelData data = fmmap.getCellValue(i,j);
			double x,y; fmmap.GridToWorld(data.x,data.y,x,y);
			PoseSpeed ps(Pose(x,y),data.d);
			//cout << data.x << " " << data.y << endl;
			nodesGrid[i][j] = graph.addNode(ps);
		}
	}

	for (int i=0;i<fmmap.getWidth()-1;i++){
		for (int j=0;j<fmmap.getHeight()-1;j++){
			if (nodesGrid[i][j]->getValue().speed!=0 && nodesGrid[i+1][j]->getValue().speed!=0){
				PoseSpeedEdge* edge = graph.addEdge(nodesGrid[i][j],nodesGrid[i+1][j]);
				//cout << "edge between " << i << "," <<j << "->" << i+1 << "," << j << endl;
			}
			if (nodesGrid[i][j]->getValue().speed!=0 && nodesGrid[i][j+1]->getValue().speed!=0){
				PoseSpeedEdge* edge = graph.addEdge(nodesGrid[i][j],nodesGrid[i][j+1]);
				//cout << "edge between " << i << "," <<j << "->" << i << "," << j+1 << endl;
			}
			if (nodesGrid[i][j]->getValue().speed!=0 && nodesGrid[i+1][j+1]->getValue().speed!=0){
				PoseSpeedEdge* edge = graph.addEdge(nodesGrid[i][j],nodesGrid[i+1][j+1]);
				//cout << "edge between " << i << "," <<j << "->" << i << "," << j+1 << endl;
			}
			if (nodesGrid[i+1][j]->getValue().speed!=0 && nodesGrid[i][j+1]->getValue().speed!=0){
				PoseSpeedEdge* edge = graph.addEdge(nodesGrid[i+1][j],nodesGrid[i][j+1]);
				//cout << "edge between " << i << "," <<j << "->" << i << "," << j+1 << endl;
			}
		}
	}

	//for (int i=0;i<fmmap.getWidth()-1;i++){
	//	if (nodesGrid[i][fmmap.getHeight()-1]->getValue().speed!=0 && //nodesGrid[i+1][fmmap.getHeight()-1]->getValue().speed!=0){
	//		PoseSpeedEdge* edge = graph.addEdge(nodesGrid[i][fmmap.getHeight()-1],nodesGrid[i+1][fmmap.getHeight()-1]);
	//	}
	//}

	for (int j=0;j<fmmap.getHeight()-1;j++){
		if (nodesGrid[fmmap.getWidth()-1][j]->getValue().speed!=0 && nodesGrid[fmmap.getWidth()-1][j+1]->getValue().speed!=0){
			PoseSpeedEdge* edge = graph.addEdge(nodesGrid[fmmap.getWidth()-1][j],nodesGrid[fmmap.getWidth()-1][j+1]);
		}
	}
	
	graph.recomputeEdgesWeight();

	cout << "Graph Building Total time: " << timer.precistoc() << endl;


	AstarPoseSpeedSearcher searcher(0,DBL_MAX);
	searcher.setGraph(&graph);
	double cost=1;

	timer.precistic();

	if (searcher.findPath(nodesGrid[20][20], nodesGrid[fmmap.getWidth()-20][fmmap.getHeight()-20],cost))
		cout << "Path found. Time:" << cost <<  endl;
	cout << "Path Finding Total time: " << timer.precistoc() << endl;
	
	vector<PoseSpeedNode*> path = searcher.getPath();

	unsigned char color[] = {255,0,0};
	for (int i=0;i<path.size();i++){
		PoseSpeedNode *node = path[i];
		int pixelX, pixelY; fmmap.WorldToGrid(node->getValue().x ,node->getValue().y, pixelX, pixelY);
		//cout << node->getValue().x << " " << node->getValue().y << endl;
 		//cout << pixelX << " " << pixelY << endl;
		img->setPixel(pixelX,img->getHeight()-1-pixelY,color);
	}

	img->save("path.bmp");
	delete img;
	
	img = fmmap.convertToImage();
	for (int i=0;i<path.size();i++){
		PoseSpeedNode *node = path[i];
		int pixelX, pixelY; fmmap.WorldToGrid(node->getValue().x ,node->getValue().y, pixelX, pixelY);
		//cout << node->getValue().x << " " << node->getValue().y << endl;
 		//cout << pixelX << " " << pixelY << endl;
		img->setPixel(pixelX,img->getHeight()-1-pixelY,color);
	}
	
	
	img->save("fmmap.bmp");
	delete img;
	
	return 1;
}
