#include "fmstarpathplanner.h"
#include "float.h"

namespace mr{


void FMStarPathPlanner2d::computeFastMarching(){
	if (fmmap != NULL) delete fmmap;
	fmmap = new FMGridMap();
	fmmap->computeFM(map, maxSpeed, safeDistance);
}

void FMStarPathPlanner2d::computeFM2(int targetX, int targetY, int initX, int initY){
	if (fm2map != NULL) delete fm2map;

	if (fmmap==NULL) computeFastMarching();
	
	fm2map = new FMGridMap();
	fm2map->computeFM2(fmmap,targetX,targetY,initX, initY,fmmap->getRealMaxSpeed(),star);

}

bool FMStarPathPlanner2d::computePath(const Pose& initPose, const Pose& targetPose){

	path.clear();
	if (map_changed) computeFastMarching();
	map_changed = false;
	
	int targetX,targetY;
	int initX,initY;

	MRTime timer; timer.precistic();

	if ( map->WorldToGrid(targetPose.x, targetPose.y, targetX, targetY)  && map->WorldToGrid(initPose.x, initPose.y, initX, initY) ){

		if (fmmap->getCellValue(targetX,targetY).d == 0){
			cout << "Target Point on obstacle" << endl;
			path_found=false;
			return false;
		}

		if (fmmap->getCellValue(initX,initY).d == 0){
			cout << "Init Point on obstacle" << endl;
			path_found=false;
			return false;
		}

		//else
		computeFM2(targetX, targetY, initX, initY);

	}else{ //target out of map

		cout << "target or init out of map" << endl;
		path_found=false;
		return false;

	}
	
	int pixelX,pixelY;
	pixelX=initX;
	pixelY=initY;

	estimatedTime = 0;
	
	double xw,yw; map->GridToWorld(pixelX,pixelY,xw,yw);
	path.push_back(Point2(xw,yw), fmmap->getCellValue(pixelX,pixelY).d);

	//first step requires special treatment because it is likely to be in the border of the map

	bool pixelX_set=false;
	bool pixelY_set=false;
		
	int x,y,nextX,nextY;

	if (fm2map->getCellValue(pixelX-1,pixelY).d == DBL_MAX){
		if (fm2map->getCellValue(pixelX+1,pixelY).d == DBL_MAX){
			pixelX_set = true;
			nextX = pixelX;
		}else{
			pixelX_set = true;
			nextX = pixelX+1;
		}
	}else if (fm2map->getCellValue(pixelX+1,pixelY).d == DBL_MAX){
		if (fm2map->getCellValue(pixelX-1,pixelY).d == DBL_MAX){
			pixelX_set = true;
			nextX = pixelX;
		}else{
			pixelX_set = true;
			nextX = pixelX-1;
		}
	}

	if (fm2map->getCellValue(pixelX,pixelY-1).d == DBL_MAX){
		if (fm2map->getCellValue(pixelX,pixelY+1).d == DBL_MAX){
			nextY = pixelY;
			pixelY_set = true;
		}else{
			nextY = pixelY+1;
			pixelY_set = true;
		}
	}else if (fm2map->getCellValue(pixelX,pixelY+1).d == DBL_MAX){
		if (fm2map->getCellValue(pixelX,pixelY-1).d == DBL_MAX){
			nextY = pixelY;
			pixelY_set = true;
		}else{
			nextY = pixelY-1;
			pixelY_set = true;
		}
	}

	if (pixelX_set && !pixelY_set){
		nextY = pixelY;
		pixelY_set = true;
	}

	if (pixelY_set && !pixelX_set){
		nextX = pixelX;
		pixelX_set = true;
	}

	if (!pixelX_set && !pixelY_set){

		double grad_x = -fm2map->getCellValue(pixelX-1,pixelY).d / 2 + fm2map->getCellValue(pixelX+1,pixelY).d / 2;
		double grad_y = -fm2map->getCellValue(pixelX,pixelY-1).d / 2 + fm2map->getCellValue(pixelX,pixelY+1).d / 2;

		double mod = sqrt(square(grad_x) + square(grad_y));
		double alpha = atan2(grad_y,grad_x);

		double next_wx = xw - 1.5*fm2map->getResolution()*cos(alpha);
		double next_wy = yw - 1.5*fm2map->getResolution()*sin(alpha);
		map->WorldToGrid(next_wx, next_wy, nextX,nextY);
	}

	double next_wx, next_wy;
	//string a; cin >> a;
	map->GridToWorld(nextX,nextY,next_wx,next_wy);
	path.push_back(Point2(next_wx,next_wy),fmmap->getCellValue(nextX,nextY).d);

	estimatedTime=0;
	pathLength=0;
	double dist = distance(Pose(xw,yw),Pose(next_wx, next_wy));
	double time = dist / fmmap->getCellValue(pixelX,pixelY).d;
	estimatedTime+=time;
	pathLength+=dist;

	xw = next_wx; yw = next_wy;
	pixelX=nextX;pixelY=nextY;

		
	while( pixelX != targetX || pixelY != targetY ){
	
		double grad_x = -fm2map->getCellValue(pixelX-1,pixelY-1).d + fm2map->getCellValue(pixelX+1,pixelY-1).d
							-2* fm2map->getCellValue(pixelX-1,pixelY).d + 2*fm2map->getCellValue(pixelX+1,pixelY).d
							-fm2map->getCellValue(pixelX-1,pixelY+1).d + fm2map->getCellValue(pixelX+1,pixelY+1).d;
							
		double grad_y = - fm2map->getCellValue(pixelX-1,pixelY-1).d + fm2map->getCellValue(pixelX-1,pixelY+1).d
							-2* fm2map->getCellValue(pixelX,pixelY-1).d + 2*fm2map->getCellValue(pixelX,pixelY+1).d
							- fm2map->getCellValue(pixelX+1,pixelY-1).d + fm2map->getCellValue(pixelX+1,pixelY+1).d;

		//double grad_x = -fm2map->getCellValue(pixelX-1,pixelY).d / 2 + fm2map->getCellValue(pixelX+1,pixelY).d / 2;
		//double grad_y = -fm2map->getCellValue(pixelX,pixelY-1).d / 2 + fm2map->getCellValue(pixelX,pixelY+1).d / 2;

		double mod = sqrt(square(grad_x) + square(grad_y));
		double alpha = atan2(grad_y,grad_x);

		double next_wx = xw - fm2map->getResolution()*cos(alpha);
		double next_wy = yw - fm2map->getResolution()*sin(alpha);
		
		map->WorldToGrid(next_wx, next_wy, nextX,nextY);
		path.push_back(Point2(next_wx,next_wy),fmmap->getCellValue(nextX,nextY).d);

		double dist = distance(Pose(xw,yw),Pose(next_wx, next_wy));
		double time = dist / fmmap->getCellValue(pixelX,pixelY).d;
		estimatedTime+=time;
		pathLength+=dist;
		
		pixelX=nextX;pixelY=nextY;
		xw = next_wx; yw = next_wy;

	}
	
	if (star) cout << "FMStar Computation Time: " << timer.precistoc() << endl;
	else cout << "FM2 Computation Time: " << timer.precistoc() << endl;
	path_found=true;
	return true;
}

}// end namespace
