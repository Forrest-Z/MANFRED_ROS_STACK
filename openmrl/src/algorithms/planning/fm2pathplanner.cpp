#include "fm2pathplanner.h"
#include "float.h"

#include "system/time.h"

namespace mr{


void FM2PathPlanner2d::computeFM(){
	if (_fmmap != NULL) delete _fmmap;
	_fmmap = new FMGridMap();
	_fmmap->computeFM(map, _maxSpeed, _safeDistance);
}

void FM2PathPlanner2d::computeFM2(int targetX, int targetY, int initX, int initY){
	if (_fm2map != NULL) delete _fm2map;

	if (_fmmap==NULL) computeFM();
	
	_fm2map = new FMGridMap();
	_fm2map->computeFM2(_fmmap,targetX,targetY,initX, initY,_fmmap->getRealMaxSpeed(),_star,_directional);
}

bool FM2PathPlanner2d::computePath(const Pose& initPose, const Pose& targetPose){

	path.clear();
	if (_map_changed) computeFM();
	_map_changed = false;
	
	int targetX,targetY;
	int initX,initY;

	MRTime timer; timer.precistic();

	if ( map->WorldToGrid(targetPose.x, targetPose.y, targetX, targetY)
		  && map->WorldToGrid(initPose.x, initPose.y, initX, initY) ){
		if (_fmmap->getCellValue(targetX,targetY).d == 0){
			cout << "Target Point on obstacle" << endl;
			_path_found=false;
			return false;
		}

		if (_fmmap->getCellValue(initX,initY).d == 0){
			cout << "Init Point on obstacle" << endl;
			_path_found=false;
			return false;
		}

		//else

		computeFM2(targetX, targetY, initX, initY);

	}else{ //target or init out of map

		cout << "target or init out of map" << endl;
		_path_found=false;
		return false;
	}
	
	int pixelX,pixelY;
	pixelX=initX;
	pixelY=initY;

	_estimatedTime = 0;
	
	double xw,yw; map->GridToWorld(pixelX,pixelY,xw,yw);
	path.push_back(Point2(xw,yw), _fmmap->getCellValue(pixelX,pixelY).d);

	/************ TREATMENT OF FIRST PIXEL ****************/
	//first step requires special treatment because it is likely to be in the border of the map
	bool pixelX_set=false;
	bool pixelY_set=false;
		
	int x,y,nextX,nextY;

	if (_fm2map->getCellValue(pixelX-1,pixelY).d == DBL_MAX){
		if (_fm2map->getCellValue(pixelX+1,pixelY).d == DBL_MAX){
			pixelX_set = true;
			nextX = pixelX;
		}else{
			pixelX_set = true;
			nextX = pixelX+1;
		}
	}else if (_fm2map->getCellValue(pixelX+1,pixelY).d == DBL_MAX){
		if (_fm2map->getCellValue(pixelX-1,pixelY).d == DBL_MAX){
			pixelX_set = true;
			nextX = pixelX;
		}else{
			pixelX_set = true;
			nextX = pixelX-1;
		}
	}

	if (_fm2map->getCellValue(pixelX,pixelY-1).d == DBL_MAX){
		if (_fm2map->getCellValue(pixelX,pixelY+1).d == DBL_MAX){
			nextY = pixelY;
			pixelY_set = true;
		}else{
			nextY = pixelY+1;
			pixelY_set = true;
		}
	}else if (_fm2map->getCellValue(pixelX,pixelY+1).d == DBL_MAX){
		if (_fm2map->getCellValue(pixelX,pixelY-1).d == DBL_MAX){
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

		double grad_x = -_fm2map->getCellValue(pixelX-1,pixelY).d / 2 + _fm2map->getCellValue(pixelX+1,pixelY).d / 2;
		double grad_y = -_fm2map->getCellValue(pixelX,pixelY-1).d / 2 + _fm2map->getCellValue(pixelX,pixelY+1).d / 2;

		double mod = sqrt(square(grad_x) + square(grad_y));
		double alpha = atan2(grad_y,grad_x);

		double next_wx = xw - 1.5*_fm2map->getResolution()*cos(alpha);
		double next_wy = yw - 1.5*_fm2map->getResolution()*sin(alpha);
		map->WorldToGrid(next_wx, next_wy, nextX,nextY);
	}

	double next_wx, next_wy;
	//string a; cin >> a;
	map->GridToWorld(nextX,nextY,next_wx,next_wy);
	path.push_back(Point2(next_wx,next_wy),_fmmap->getCellValue(nextX,nextY).d);

	_estimatedTime=0;
	_pathLength=0;
	double dist = mrl_distance(Pose(xw,yw),Pose(next_wx, next_wy));
	double time = dist / _fmmap->getCellValue(pixelX,pixelY).d;
	_estimatedTime+=time;
	_pathLength+=dist;

	xw = next_wx; yw = next_wy;
	pixelX=nextX;pixelY=nextY;

	/************ MAIN LOOP ****************/
		
	while( fabs(pixelX - targetX) > 1 || fabs(pixelY - targetY )> 1 )
	{
	
		 //THIS COMPUTATION IS MORE PRECISE, BUT DOES NOT SEEM TO PROVIDE BETTER RESULT
		 /******************/

		//double grad_x = -_fm2map->getCellValue(pixelX-1,pixelY-1).d + _fm2map->getCellValue(pixelX+1,pixelY-1).d
		//					-2* _fm2map->getCellValue(pixelX-1,pixelY).d + 2*_fm2map->getCellValue(pixelX+1,pixelY).d
		//					-_fm2map->getCellValue(pixelX-1,pixelY+1).d + _fm2map->getCellValue(pixelX+1,pixelY+1).d;
							
		//double grad_y = - _fm2map->getCellValue(pixelX-1,pixelY-1).d + _fm2map->getCellValue(pixelX-1,pixelY+1).d
		//					-2* _fm2map->getCellValue(pixelX,pixelY-1).d + 2*_fm2map->getCellValue(pixelX,pixelY+1).d
		//					- _fm2map->getCellValue(pixelX+1,pixelY-1).d + _fm2map->getCellValue(pixelX+1,pixelY+1).d;

		/******************/
		//THIS COMPUTATION (above) IS MORE PRECISE, BUT DOES NOT SEEM TO PROVIDE BETTER RESULT


		double grad_x = -_fm2map->getCellValue(pixelX-1,pixelY).d / 2 + _fm2map->getCellValue(pixelX+1,pixelY).d / 2;
		double grad_y = -_fm2map->getCellValue(pixelX,pixelY-1).d / 2 + _fm2map->getCellValue(pixelX,pixelY+1).d / 2;

		double mod = sqrt(square(grad_x) + square(grad_y));
		double alpha = atan2(grad_y,grad_x);

		double next_wx = xw - _fm2map->getResolution()*cos(alpha);
		double next_wy = yw - _fm2map->getResolution()*sin(alpha);
		
		map->WorldToGrid(next_wx, next_wy, nextX,nextY);
		path.push_back(Point2(next_wx,next_wy),_fmmap->getCellValue(nextX,nextY).d);

		double dist = mrl_distance(Pose(xw,yw),Pose(next_wx, next_wy));
		double time = dist / _fmmap->getCellValue(pixelX,pixelY).d;
		_estimatedTime+=time;
		_pathLength+=dist;
		
		pixelX=nextX;pixelY=nextY;
		xw = next_wx; yw = next_wy;

	}
	
	 cout << "FM2 Computation Time: " << timer.precistoc() << endl;
	_path_found=true;
	return true;
}

}// end namespace
