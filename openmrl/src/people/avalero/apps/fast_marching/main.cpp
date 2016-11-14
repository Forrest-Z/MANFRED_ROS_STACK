#include <mrcore.h>
#include "fmgridmap.h"
#include "fmstarpathplanner.h"
//#include "fm2pathplanner2d.h"
#include <float.h>
#include <iomanip>
#include <iostream>

using namespace mr;
using namespace std;

class Point2i{
	public:
	Point2i(unsigned int x, unsigned int y){this->x=x; this->y=y;}

	unsigned int x;
	unsigned int y;
};

vector<Point2i> queue;

int main(void){

	 //The map
	 LabeledGridMap map(50,25,0.1,LabeledGridMap::FREE);
	 Image* img = new Image(500,250,false); img->load("thick_voronoi.bmp");
	 map.loadFromImage(img,50,0.1,0,0);

	 //The fmmap
	 FMStarPathPlanner2d fm2pathPlanner(&map, false);
	 FMStarPathPlanner2d fm1pathPlanner(&map, false);
	 FMStarPathPlanner2d fm2nspathPlanner(&map, false);
	 FMStarPathPlanner2d fm2starpathPlanner(&map, true);

	 fm2pathPlanner.configure(1.5,2);
	 fm2nspathPlanner.configure(3,20);
	 fm1pathPlanner.configure(1.5,0.001);
	 fm2starpathPlanner.configure(1.5,2);

//  PATH simple
		  bool path_found_fm2 = fm2pathPlanner.computePath(Pose(10,12.8),Pose(40,12.8));
		  bool path_found_fm2ns = fm2nspathPlanner.computePath(Pose(10,12.8),Pose(40,12.8));
		  bool path_found_fm1 = fm1pathPlanner.computePath(Pose(10,12.8),Pose(40,12.8));
		  bool path_found_star = fm2starpathPlanner.computePath(Pose(10,12.8),Pose(40,12.8));


/*	 PATH 1 thick_voronoi
	 bool path_found_fm2 = fm2pathPlanner.computePath(Pose(3,12.5),Pose(25,9));
	 bool path_found_fm2ns = fm2nspathPlanner.computePath(Pose(3,12.5),Pose(25,9));
	 bool path_found_fm1 = fm1pathPlanner.computePath(Pose(3,12.5),Pose(25,9));
	 bool path_found_star = fm2starpathPlanner.computePath(Pose(3,12.5),Pose(25,9));


/*  PATH 2 thick_voronoi
	 bool path_found_fm2 = fm2pathPlanner.computePath(Pose(40,7.8),Pose(25,9));
	 bool path_found_fm2ns = fm2nspathPlanner.computePath(Pose(40,7.8),Pose(25,9));
	 bool path_found_fm1 = fm1pathPlanner.computePath(Pose(40,7.8),Pose(25,9));
	 bool path_found_star = fm2starpathPlanner.computePath(Pose(40,7.8),Pose(25,9));
*/

/*	 PATH 3 thick_voronoi
	 bool path_found_fm2 = fm2pathPlanner.computePath(Pose(3,2.5),Pose(40,7.8));
	 bool path_found_fm2ns = fm2nspathPlanner.computePath(Pose(3,2.5),Pose(40,7.8));
	 bool path_found_fm1 = fm1pathPlanner.computePath(Pose(3,2.5),Pose(40,7.8));
	 bool path_found_star = fm2starpathPlanner.computePath(Pose(3,2.5),Pose(40,7.8));
*/
	 if (path_found_fm2){
		  cout << "Fm2 Path estimated time: " << fm2pathPlanner.getPathEstimatedTime() << endl;
		  cout << "Fm2 Path length: " << fm2pathPlanner.getPathLength() << endl;
	 }

	 if (path_found_fm2ns){
		  cout << "Fm2ns Path estimated time: " << fm2nspathPlanner.getPathEstimatedTime() << endl;
		  cout << "Fm2ns Path length: " << fm2nspathPlanner.getPathLength() << endl;
	 }

	 if (path_found_fm1){
		  cout << "Fm1 Path estimated time: " << fm1pathPlanner.getPathEstimatedTime() << endl;
		  cout << "Fm1 Path length: " << fm1pathPlanner.getPathLength() << endl;
	 }

	 if (path_found_star){
		  cout << "Path star estimated time: " << fm2pathPlanner.getPathEstimatedTime() << endl;
		  cout << "Path star length: " << fm2pathPlanner.getPathLength() << endl;
	 }

	 img = fm2pathPlanner.getPointerToFMMap()->convertToImage();
	 img->save("thick_voronoi_fm2.bmp");
	 delete img;

	 img = fm2nspathPlanner.getPointerToFMMap()->convertToImage();
	 img->save("thick_voronoi_fm2ns.bmp");
	 delete img;

	 img = fm1pathPlanner.getPointerToFMMap()->convertToImage();
	 img->save("thick_voronoi_fm1.bmp");
	 delete img;

	 img = fm2starpathPlanner.getPointerToFMMap()->convertToImage();
	 img->save("thick_voronoi_fmstar.bmp");
	 delete img;


	 img = fm2pathPlanner.getPointerToFM2Map()->convertToImage();
	 if (path_found_fm2){
		  ofstream file;
		  file.open ("fm2_path_speeds.dat");
		  Path2D path = fm2pathPlanner.getPath();
		  unsigned char color[] = {255,0,0};
		  for (int i=0;i<path.size();i++){
				int pixelX, pixelY;
				map.WorldToGrid(path[i].x,path[i].y,pixelX,pixelY);
				img->setPixel(pixelX,img->getHeight()-1-pixelY,color);
				file << path.speed(i) << endl;
		  }
		  file.close();
	 }else{
		  cout << "Path not found" << endl;
		  return 0;
	 }
	 img->save("thick_voronoi_fm2_path.bmp");
	 delete img;

	 ///////////////////////

	 img = fm2nspathPlanner.getPointerToFM2Map()->convertToImage();
	 if (path_found_fm2ns){
		  ofstream file;
		  file.open ("fm2ns_path_speeds.dat");
		  Path2D path = fm2nspathPlanner.getPath();
		  unsigned char color[] = {255,0,0};
		  for (int i=0;i<path.size();i++){
				int pixelX, pixelY;
				map.WorldToGrid(path[i].x,path[i].y,pixelX,pixelY);
				img->setPixel(pixelX,img->getHeight()-1-pixelY,color);
				file << path.speed(i) << endl;
		  }
		  file.close();
	 }else{
		  cout << "Path not found" << endl;
		  return 0;
	 }
	 img->save("thick_voronoi_fm2ns_path.bmp");
	 delete img;

	 ///////////////////////

	 img = fm1pathPlanner.getPointerToFM2Map()->convertToImage();
	 if (path_found_fm1){
		  ofstream file;
		  file.open ("fm1_path_speeds.dat");
		  Path2D path = fm1pathPlanner.getPath();
		  unsigned char color[] = {255,0,0};
		  for (int i=0;i<path.size();i++){
				int pixelX, pixelY;
				map.WorldToGrid(path[i].x,path[i].y,pixelX,pixelY);
				img->setPixel(pixelX,img->getHeight()-1-pixelY,color);
				file << path.speed(i) << endl;
		  }
		  file.close();
	 }else{
		  cout << "Path not found" << endl;
		  return 0;
	 }
	 img->save("thick_voronoi_fm1_path.bmp");
	 delete img;


	 cout << "here" << endl;
	 img = new Image();
	 cout << "2" << endl;
	 img->load("simple.bmp");
	 cout << "3" << endl;
	 if (path_found_fm1){
		  ofstream file;
		  file.open ("fm1_path_speeds.dat");
		  Path2D path = fm1pathPlanner.getPath();
		  unsigned char color[] = {255,0,0};
		  for (int i=0;i<path.size();i++){
				int pixelX, pixelY;
				map.WorldToGrid(path[i].x,path[i].y,pixelX,pixelY);
				img->setPixel(pixelX,img->getHeight()-1-pixelY,color);
				file << path.speed(i) << endl;
		  }
		  file.close();
	 }else{
		  cout << "Path not found" << endl;
		  return 0;
	 }
	 img->save("thick_voronoi_fm1_2_path.bmp");
	 delete img;

	 ////////////////////////

	 img = fm2starpathPlanner.getPointerToFM2Map()->convertToImage();
	 if (path_found_fm2ns){
		  ofstream file;
		  file.open ("fm2star_path_speeds.dat");
		  Path2D path = fm2starpathPlanner.getPath();
		  unsigned char color[] = {255,0,0};
		  for (int i=0;i<path.size();i++){
				int pixelX, pixelY;
				map.WorldToGrid(path[i].x,path[i].y,pixelX,pixelY);
				img->setPixel(pixelX,img->getHeight()-1-pixelY,color);
				file << path.speed(i) << endl;
		  }
		  file.close();
	 }else{
		  cout << "Path not found" << endl;
		  return 0;
	 }
	 img->save("thick_voronoi_fm2star_path.bmp");
	 delete img;

	 ///////////////////////

	 fm2pathPlanner.getPointerToFM2Map()->generatePlotData("thick_voronoi_fm2_fm2.dat");
	 fm2pathPlanner.getPointerToFMMap()->generatePlotData("thick_voronoi_fm2_fm.dat");

	 fm2starpathPlanner.getPointerToFM2Map()->generatePlotData("thick_voronoi_fm2ns_fm2.dat");
	 fm2starpathPlanner.getPointerToFMMap()->generatePlotData("thick_voronoi_fm2ns_fm.dat");

	 fm2nspathPlanner.getPointerToFM2Map()->generatePlotData("thick_voronoi_fm2ns_fm2.dat");
	 fm2nspathPlanner.getPointerToFMMap()->generatePlotData("thick_voronoi_fm2ns_fm.dat");


	 fm1pathPlanner.getPointerToFM2Map()->generatePlotData("thick_voronoi_fm1_fm2.dat");
	 fm1pathPlanner.getPointerToFMMap()->generatePlotData("thick_voronoi_fm1_fm.dat");

	 return 1;
}
