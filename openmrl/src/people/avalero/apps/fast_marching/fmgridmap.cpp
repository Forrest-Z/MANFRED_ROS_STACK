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
#include <iomanip>
#include "fmgridmap.h"
#include <mrcore.h>
#include <float.h>
#include <limits.h>
//#include "config.h"
//#include "image.h"
//#include "gl/gltools.h"

namespace mr{

void FMGridMap::buildPreComputedDistances(double speed){

	if (precomputedDists !=NULL){
		for (int i=0; i< width ; i++){
			delete [] precomputedDists[i];
		}
		delete [] precomputedDists;
	}
	
	precomputedDists = new double*[width];
	
	for (int i=0; i< width ; i++){
		precomputedDists[i]=new double[height];
	}

	double speed_inv = 1/speed;
	
	for (int i=0; i< width ; i++){
		for (int j=0; j<height; j++){
			double dist =  sqrt( square(i) + square(j) )  * speed_inv ;
			precomputedDists[i][j]=dist;
		}
	}
}


void FMGridMap::generatePlotData(string filename, bool crop) const{

	ofstream data;
	data.open(filename.c_str());

	int min_x=INT_MAX; int min_y=INT_MAX;
	int max_x=0; int max_y=0;

	if (crop){
		for (int x=0; x < this->width; x++){
			for (int y=0; y < this->height; y++){
				FMPixelData pdata = getCellValue(x,y);
				if ( !(pdata.d == DBL_MAX || pdata.status != FMPixelData::FROZEN ) ){
					if(x<min_x) min_x = x; if (y<min_y) min_y = y;
					if(x>max_x) max_x = x; if (y>max_y) max_y = y;
				}
			}
		}
	}else{
		min_x=min_y=0;
		max_x=width-1;
		max_y=height-1;
	}

	for (int i=min_x; i<=max_x; i++){
		for (int j=min_y; j<=max_y; j++){
			FMPixelData pdata = getCellValue(i,j);
			if (pdata.d == DBL_MAX){
				data << 255 << "\t" ;
			}else{
				data << pdata.d / max * 255 << "\t" ;
			}
		}
		data << endl;
	}
	data.close();
}

void FMGridMap::buildFromFMGridMapStar(FMGridMap* fmmap, int targetX, int targetY, int initX, int initY){
	//max = safe_distance / max_speed;
	max=0;
	this->free();
	this->resolution=fmmap->getResolution();
	this->mapX=fmmap->getMapX();
	this->mapY=fmmap->getMapY();
	this->width=fmmap->getWidth();
	this->height=fmmap->getHeight();
	this->allocate(this->width,this->height);

	buildPreComputedDistances(max_speed);
	//FILL FMGRIDMAP

	/******* INITIALIZE ***********/
	
	for (int i=0;i<width;i++){
		for (int j=0;j<height;j++){			
			FMPixelData aux(DBL_MAX, DBL_MAX, FMPixelData::NON_VISITED);
			aux.x=i; aux.y=j;
			if (fmmap->getCellValue(i,j).d == 0) aux.status = FMPixelData::FROZEN;
			setCellValue(i,j,aux);
		}
	}
	
	int xdist = Abs(initX-targetX);
	int ydist = Abs(initY-targetY);
	double optimal_cost;
	if (max_speed == fmmap->getCellValue(targetX,targetY).d) optimal_cost = precomputedDists[xdist][ydist]/5;
	else optimal_cost = precomputedDists[xdist][ydist];
	//cout << (max_speed - fmmap->getCellValue(targetX,targetY).d)/max_speed << endl;
	//double optimal_cost = sqrt(square(xdist) + square(ydist)) / max_speed;

	FMPixelData* pData = getPointerToCell(targetX,targetY);
	pData->status = FMPixelData::FROZEN;
	pData->optimalCost = optimal_cost;
	pData->d = 0;
	cout << heap.size()<< endl;
	//string a; cin >> a;
	insertIntoHeapStar(pData);

	
	/*********** LOOP *****************/
	//while the heap is not empty, expand the wave

	while(!heap.empty()){
		//cout << heap.size() << endl;
		//Extract v from H
		//printGrid();
		FMPixelData* current = heap.front();
		heap.erase(heap.begin());
		
		//Freeze v
		current->status = FMPixelData::FROZEN;
		if (current->d > max) max = current->d;

		if (current->x == initX && current->y == initY){ //stop, we have arrived
			break;
		}
		
		//FOR EACH NEIGHBOR
		//Neighbours 1,0 ; -1,0 ; 0,-1 ; 0,1
		int x,y;
		for (int aux=0;aux<2;aux++){
			for (int l=-1;l<2;l++){
				if (l==0) l=1;
				//int a; cin >> a;
				if (aux==0){
					x=current->x+l; y=current->y;
				}else{
					x=current->x; y=current->y+l;
				}
				//avoid border points
				if (x<1 || y<1 || x>=width-1 || y >=height-1) continue;
				FMPixelData* pData = getPointerToCell(x,y);
				//compute distance
				if (pData->status != FMPixelData::FROZEN){
					double speed = fmmap->getCellValue(x,y).d;
					double distance = computeDistance(x,y,speed);
					int distx = Abs(initX-x); int disty=Abs(initY-y);
					double optimal_cost;
					if (max_speed == fmmap->getCellValue(x,y).d) optimal_cost = distance+precomputedDists[distx][disty]/5;
					else optimal_cost = distance + precomputedDists[distx][disty];
					//cout << (max_speed - fmmap->getCellValue(x,y).d)/max_speed << endl;;
					//double optimal_cost = distance + ( sqrt( square(distx) + square(disty) ) / max_speed );
					
					if (pData->status != FMPixelData::NARROW_BAND){
						pData->status = FMPixelData::NARROW_BAND;
						pData->d=distance;
						pData->optimalCost = optimal_cost;
						insertIntoHeapStar(pData);
					}else{
						if (optimal_cost != pData->optimalCost) changeLocationStar(pData, optimal_cost);
						pData->d=distance;
					}
				}
				//printGrid();
			} //end for l
		}//end for aux
	}
}


Image* FMGridMap::convertToImage(int w) const{
	Image *img =new Image(this->width, this->height);

	for (int x=0; x < this->width; x++){
		for (int y=0; y < height; y++){
			FMPixelData pdata = getCellValue(x,y);
			unsigned char graylevel;

			if (pdata.d == DBL_MAX || pdata.status != FMPixelData::FROZEN ) graylevel = 255;
			else graylevel = (unsigned char)( pdata.d / max * 255. );
			
			unsigned char color[] = {graylevel, graylevel, graylevel};
			img->setPixel(x,height-y-1,color);
		}
	}
	
	if (w>0 && img->getWidth()!=w) img->scale(w);

	return img;
}

void FMGridMap::printGrid(){
	for (int y=0;y<getHeight();y++){
		for (int x=0; x<getWidth(); x++){
			double value = getCellValue(x,y).d;
			FMPixelData::Status status = getCellValue(x,y).status;
			if (status == FMPixelData::NON_VISITED) cout << "___\t";
			if (status == FMPixelData::NARROW_BAND) cout << setprecision (3) << value <<"\t";
			if (status == FMPixelData::FROZEN) cout  <<"*"<< setprecision (3) << value<<"*\t";
		}
		cout << endl;
	}

	cout << "Pulsar tecla ... " << endl;
	string a;
	cin >> a;	
}

void FMGridMap::writeToStream(Stream& os){
//TODO
}

void FMGridMap::readFromStream(Stream& is){
//TODO
}

void FMGridMap::drawGL(){
// TODO
}


void FMGridMap::buildFromFMGridMap(FMGridMap* fmmap, int pixelX, int pixelY, int initX, int initY){
	//max = safe_distance / max_speed;
	max=0;
	this->free();
	this->resolution=fmmap->getResolution();
	this->mapX=fmmap->getMapX();
	this->mapY=fmmap->getMapY();
	this->width=fmmap->getWidth();
	this->height=fmmap->getHeight();
	this->allocate(this->width,this->height);

	//FILL FMGRIDMAP

	/******* INITIALIZE ***********/
	
	for (int i=0;i<width;i++){
		for (int j=0;j<height;j++){			
			FMPixelData aux(DBL_MAX, DBL_MAX, FMPixelData::NON_VISITED);
			aux.x=i; aux.y=j;
			if (fmmap->getCellValue(i,j).d == 0) aux.status = FMPixelData::FROZEN;
			setCellValue(i,j,aux);
		}
	}

	FMPixelData aux(0,FMPixelData::FROZEN);
	aux.x=pixelX; aux.y=pixelY;
	setCellValue(pixelX,pixelY,aux);
	insertIntoHeap(&aux);

	
	/*********** LOOP *****************/
	//while the heap is not empty, expand the wave

	while(!heap.empty()){
		//cout << heap.size() << endl;
		//Extract v from H
		//printGrid();
		FMPixelData* current = heap.front();
		heap.erase(heap.begin());
		
		//Freeze v
		current->status = FMPixelData::FROZEN;
		if (current->d > max) max = current->d;
		
		if (current->x == initX && current->y == initY){ //stop, we have arrived
			break;
		}
		
		//FOR EACH NEIGHBOR
		//Neighbours 1,0 ; -1,0 ; 0,-1 ; 0,1
		int x,y;
		for (int aux=0;aux<2;aux++){
			for (int l=-1;l<2;l++){
				if (l==0) l=1;
				//int a; cin >> a;
				if (aux==0){
					x=current->x+l; y=current->y;
				}else{
					x=current->x; y=current->y+l;
				}
				if (x<1 || y<1 || x>=width-1 || y >=height-1) continue;
				FMPixelData* pData = getPointerToCell(x,y);
				//compute distance
				if (pData->status != FMPixelData::FROZEN){
					double speed = fmmap->getCellValue(x,y).d;
					double distance = computeDistance(x,y,speed);
					
					if (pData->status != FMPixelData::NARROW_BAND){
						pData->status = FMPixelData::NARROW_BAND;
						pData->d=distance;
						insertIntoHeap(pData);
					}else{
						if (distance != pData->d) changeLocation(pData, distance);
					}
				
				}
				//printGrid();
			} //end for l
		}//end for aux
	}
}

void FMGridMap::buildFromFMGridMap(FMGridMap* fmmap, int pixelX, int pixelY){
	//max = safe_distance / max_speed;
	max=0;
	this->free();
	this->resolution=fmmap->getResolution();
	this->mapX=fmmap->getMapX();
	this->mapY=fmmap->getMapY();
	this->width=fmmap->getWidth();
	this->height=fmmap->getHeight();
	this->allocate(this->width,this->height);

	//FILL FMGRIDMAP

	/******* INITIALIZE ***********/
	
	for (int i=0;i<width;i++){
		for (int j=0;j<height;j++){			
			FMPixelData aux(DBL_MAX, DBL_MAX, FMPixelData::NON_VISITED);
			aux.x=i; aux.y=j;
			if (fmmap->getCellValue(i,j).d == 0) aux.status = FMPixelData::FROZEN;
			setCellValue(i,j,aux);
		}
	}

	FMPixelData aux(0,FMPixelData::FROZEN);
	aux.x=pixelX; aux.y=pixelY;
	setCellValue(pixelX,pixelY,aux);
	insertIntoHeap(&aux);

	
	/*********** LOOP *****************/
	//while the heap is not empty, expand the wave

	while(!heap.empty()){
		//cout << heap.size() << endl;
		//Extract v from H
		//printGrid();
		FMPixelData* current = heap.front();
		heap.erase(heap.begin());
		
		//Freeze v
		current->status = FMPixelData::FROZEN;
		if (current->d > max) max = current->d;
		
		//FOR EACH NEIGHBOR
		//Neighbours 1,0 ; -1,0 ; 0,-1 ; 0,1
		int x,y;
		for (int aux=0;aux<2;aux++){
			for (int l=-1;l<2;l++){
				if (l==0) l=1;
				//int a; cin >> a;
				if (aux==0){
					x=current->x+l; y=current->y;
				}else{
					x=current->x; y=current->y+l;
				}
				if (x<1 || y<1 || x>=width-1 || y >=height-1) continue;
				FMPixelData* pData = getPointerToCell(x,y);
				//compute distance
				if (pData->status != FMPixelData::FROZEN){
					double speed = fmmap->getCellValue(x,y).d;
					double distance = computeDistance(x,y,speed);
					
					if (pData->status != FMPixelData::NARROW_BAND){
						pData->status = FMPixelData::NARROW_BAND;
						pData->d=distance;
						insertIntoHeap(pData);
					}else{
						if (distance != pData->d) changeLocation(pData, distance);
					}
				}
				//printGrid();
			} //end for l
		}//end for aux
	}
}



void FMGridMap::buildFromLabeledGridMap(LabeledGridMap* lmap){
	//max = safe_distance / max_speed;
	this->free();
	this->resolution=lmap->getResolution();
	this->mapX=lmap->getMapX();
	this->mapY=lmap->getMapY();
	this->width=lmap->getWidth();
	this->height=lmap->getHeight();
	this->allocate(this->width,this->height);

	//FILL FMGRIDMAP

	/******* INITIALIZE ***********/
	
	for (int i=0;i<width;i++){
		for (int j=0;j<height;j++){
			if (lmap->getCellValue(i,j)==LabeledGridMap::OBSTACLE || (int)lmap->getCellValue(i,j)==0 ){
				FMPixelData aux(0, FMPixelData::FROZEN);
				aux.x=i; aux.y=j;
				setCellValue(i,j,aux);
				insertIntoHeap(getPointerToCell(i,j));
			}else{
				FMPixelData aux(DBL_MAX, FMPixelData::NON_VISITED);
				aux.x=i; aux.y=j;
				setCellValue(i,j,aux);
			}
		}
	}

	//printGrid();
	/*********** LOOP *****************/
	//while the heap is not empty, expand the wave

	while(!heap.empty()){
		//Extract v from H
		//printGrid();
		FMPixelData* current = heap.front();
		heap.erase(heap.begin());
		
		//Freeze v
		current->status = FMPixelData::FROZEN;
		if (current->d > max) max = current->d;
		//FOR EACH NEIGHBOR
		//Neighbours 1,0 ; -1,0 ; 0,-1 ; 0,1
		int x,y;
		for (int aux=0;aux<2;aux++){
			for (int l=-1;l<2;l++){
				if (l==0) l=1;
				//int a; cin >> a;
				if (aux==0){
					x=current->x+l; y=current->y;
				}else{
					x=current->x; y=current->y+l;
				}
				if (x<1 || y<1 || x>=width-1 || y >=height-1) continue;
				FMPixelData* pData = getPointerToCell(x,y);
				//compute distance
				if (pData->status != FMPixelData::FROZEN){
					double distance = computeDistance(x,y);
					
					if (pData->status != FMPixelData::NARROW_BAND){
						pData->status = FMPixelData::NARROW_BAND;
						pData->d=distance;
						insertIntoHeap(pData);
					}else{
						if (distance != pData->d) changeLocation(pData, distance);
					}
				}
			} //end for l
		}//end for aux
	}

	/*********** SCALE ***************/

	double saturation_value = max_speed/safe_distance;
	real_max_speed = 0;
	for (int y=0;y<getHeight();y++){
		for (int x=0; x<getWidth(); x++){
			FMPixelData *pD = getPointerToCell(x,y);
			double value = pD->d;
			if ( value*resolution > safe_distance ) value = max_speed;
			else value = value*resolution*saturation_value;

			pD->d=value;

			real_max_speed = Max(value,real_max_speed);

		}
	}

	max = max_speed;

}

double FMGridMap::computeHADistance(int x, int y, double speed){

	//compute first order distance, as it is on the border of gridmap
	if (x<=1 || y <=1 || x>=width-2 || y>=height-2) return computeDistance(x,y,speed);
	

	double T,T1,T2;

	double T1a,T1b,T1c,T1d;
	double T2a,T2b,T2c,T2d;


	T1 = DBL_MAX;
	
	if (getCellValue(x-2,y).status == FMPixelData::FROZEN && getCellValue(x-1,y).status == FMPixelData::FROZEN){
		T1a = getCellValue(x-2,y).d ;
		T1b = getCellValue(x-1,y).d ;
	}else if(getCellValue(x+2,y).status == FMPixelData::FROZEN && getCellValue(x+1,y).status == FMPixelData::FROZEN){
		T1c = getCellValue(x+1,y).d ;
		T1d = getCellValue(x+2,y).d ;
		T1 = Min(4*T1c,T1d); //second order only considering these
	}else{
		return computeDistance(x,y,speed); //cannot apply second order
	}

	if (getCellValue(x+2,y).status == FMPixelData::FROZEN && getCellValue(x+1,y).status == FMPixelData::FROZEN){
		T1c = getCellValue(x+1,y).d ;
		T1d = getCellValue(x+2,y).d ;
	}else if(getCellValue(x-2,y).status == FMPixelData::FROZEN && getCellValue(x-1,y).status == FMPixelData::FROZEN){
		T1a = getCellValue(x-2,y).d ; 
		T1b = getCellValue(x-1,y).d ;
		T1 = Min(4*T1a,T1b); //second order only considering these
	}else{
		return computeDistance(x,y,speed); //cannot apply second order
	}

	// this may only happen if all T1 pixels are frozen... otherwise, it should have taken a value before.
	if (T1==DBL_MAX) T1=Min(Min(4*T1a,T1b),Min(4*T1c,T1d));


	T2 = DBL_MAX;
	
	if (getCellValue(x,y-2).status == FMPixelData::FROZEN && getCellValue(x,y-1).status == FMPixelData::FROZEN){
		T2a = getCellValue(x,y-2).d ;
		T2b = getCellValue(x,y-1).d ;
	}else if(getCellValue(x,y+2).status == FMPixelData::FROZEN && getCellValue(x,y+1).status == FMPixelData::FROZEN){
		T2c = getCellValue(x,y+1).d ;
		T2d = getCellValue(x,y+2).d ;
		T2 = Min(4*T2c,T2d); //second order only considering these
	}else{
		return computeDistance(x,y,speed); //cannot apply second order
	}

	if (getCellValue(x,y+2).status == FMPixelData::FROZEN && getCellValue(x,y+1).status == FMPixelData::FROZEN){
		T2c = getCellValue(x,y+1).d ;
		T2d = getCellValue(x,y+2).d ;
	}else if(getCellValue(x,y-2).status == FMPixelData::FROZEN && getCellValue(x,y-1).status == FMPixelData::FROZEN){
		T2a = getCellValue(x,y-2).d ; 
		T2b = getCellValue(x,y-1).d ;
		T2 = Min(4*T2a,T2b); //second order only considering these
	}else{
		return computeDistance(x,y,speed); //cannot apply second order
	}

	// this may only happen if all T1 pixels are frozen... otherwise, it should have taken a value before.
	if (T2==DBL_MAX) T2 = Min(Min(4*T2a,T2b),Min(4*T2c,T2d));


	double square_speed_inv = 1/square(speed);
	
	double a =  6.75; //-> 9/4*3
	double b = 1.5*(T1+T2); //-> -3/2*(T1+T2);
	double c = 0.25*(square(T1) + square(T2)) - square_speed_inv; //->1/4*(square(T1) + square(T2)) - 1/square(speed);

	double quad_term = (square(b) - 4*a*c);
	if ( quad_term < 0 ) return computeDistance(x,y,speed);
	else T = (-b + sqrt(quad_term))/(2*a);

	if (T<0) return computeDistance(x,y,speed);
	if (T< Min(T1,T2)) return computeDistance(x,y,speed);

	//cout << T << endl;
	return T;

}

double FMGridMap::computeDistance(int x, int y, double speed){

	double T;
	double T1,T2;
	double T1a,T1b,T2a,T2b;

	// WE ARE AVOIDING BORDER PIXELS, SO WE DO NOT NEED TO CHECK
/*
	if (x==0 && y==0){ //bottom left
		if (getCellValue(x+1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x+1,y).d;
		else T1 = DBL_MAX;

		if (getCellValue(x,y+1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y+1).d;
		else T2 = DBL_MAX;
	}else if (x==0 && y==height-1){ //top left
		if (getCellValue(x+1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x+1,y).d;
		else T1 = DBL_MAX;

		if (getCellValue(x,y-1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y-1).d;
		else T2 = DBL_MAX;
	}else if (x==width-1 && y==height-1){ //top right
		if (getCellValue(x-1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x-1,y).d;
		else T1 = DBL_MAX;

		if (getCellValue(x,y-1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y-1).d;
		else T2 = DBL_MAX;
	}else if (x==width-1 && y==0){ //bottom right
		if (getCellValue(x-1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x-1,y).d;
		else T1 = DBL_MAX;

		if (getCellValue(x,y+1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y+1).d;
		else T2 = DBL_MAX;
	}else if(x==0){ //left
	
		if (getCellValue(x+1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x+1,y).d;
		else T1 = DBL_MAX;

		if (getCellValue(x,y+1).status == FMPixelData::FROZEN 
			 && getCellValue(x,y-1).status == FMPixelData::FROZEN)
				T2 = Min( getCellValue(x,y+1).d , getCellValue(x,y-1).d );
		else if (getCellValue(x,y+1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y+1).d;
		else if (getCellValue(x,y-1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y-1).d;
		else T2 = DBL_MAX;

	}else if(y==0){ //bottom

		if (getCellValue(x,y+1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y+1).d;
		else T2 = DBL_MAX;

		if (getCellValue(x+1,y).status == FMPixelData::FROZEN 
			 && getCellValue(x-1,y).status == FMPixelData::FROZEN)
				T1 = Min( getCellValue(x+1,y).d , getCellValue(x-1,y).d );
		else if (getCellValue(x+1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x+1,y).d;
		else if (getCellValue(x-1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x-1,y).d;
		else T1 = DBL_MAX;

	}else if(x==width-1){ //right
	
		if (getCellValue(x-1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x-1,y).d;
		else T1 = DBL_MAX;

		if (getCellValue(x,y+1).status == FMPixelData::FROZEN 
			 && getCellValue(x,y-1).status == FMPixelData::FROZEN)
				T2 = Min( getCellValue(x,y+1).d , getCellValue(x,y-1).d );
		else if (getCellValue(x,y+1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y+1).d;
		else if (getCellValue(x,y-1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y-1).d;
		else T2 = DBL_MAX;

	}else if(y==height-1){ //top

		if (getCellValue(x,y-1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y-1).d;
		else T2 = DBL_MAX;

		if (getCellValue(x+1,y).status == FMPixelData::FROZEN 
			 && getCellValue(x-1,y).status == FMPixelData::FROZEN)
				T1 = Min( getCellValue(x+1,y).d , getCellValue(x-1,y).d );
		else if (getCellValue(x+1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x+1,y).d;
		else if (getCellValue(x-1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x-1,y).d;
		else T1 = DBL_MAX;
		
	}else{ //all the other in the middle
*/		if (getCellValue(x+1,y).status == FMPixelData::FROZEN 
			 && getCellValue(x-1,y).status == FMPixelData::FROZEN)
				T1 = Min( getCellValue(x+1,y).d , getCellValue(x-1,y).d );
		else if (getCellValue(x+1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x+1,y).d;
		else if (getCellValue(x-1,y).status == FMPixelData::FROZEN) T1 = getCellValue(x-1,y).d;
		else T1 = DBL_MAX;

		if (getCellValue(x,y+1).status == FMPixelData::FROZEN 
			 && getCellValue(x,y-1).status == FMPixelData::FROZEN)
				T2 = Min( getCellValue(x,y+1).d , getCellValue(x,y-1).d );
		else if (getCellValue(x,y+1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y+1).d;
		else if (getCellValue(x,y-1).status == FMPixelData::FROZEN) T2 = getCellValue(x,y-1).d;
		else T2 = DBL_MAX;
	//}

	if (T1 == DBL_MAX){
		T = T2 + 1/speed;
		return T;
	}
	
	if (T2 == DBL_MAX){
		T = T1 + 1/speed;
		return T;
	}

	//both neighbors are frozen

	double a = 2;
	double b = -2*(T1+T2);
	double c = (square(T1) + square(T2)) - 1/square(speed);
	double quad_term = (square(b) - 4*a*c);
	if ( quad_term < 0 ) T = 1/speed + min(T1,T2) ;
	else T = (-b + sqrt(quad_term))/(2*a);

	return T;
	
		
}

void FMGridMap::changeLocation(FMPixelData* pD, double new_dist){
	vector<FMPixelData*>::iterator it = findElement(pD);
	heap.erase(it);
	pD->d=new_dist;
	insertIntoHeap(pD);
}

void FMGridMap::changeLocationStar(FMPixelData* pD, double new_dist){
	vector<FMPixelData*>::iterator it = findElementStar(pD);
	heap.erase(it);
	pD->optimalCost=new_dist;
	insertIntoHeapStar(pD);
}

void FMGridMap::insertIntoHeap(FMPixelData* pD){

	if (heap.size()==0) heap.push_back(pD);

	// check if it must be placed at the beginning
	if (heap.size()>0 && heap[0]->d >= pD->d){
		heap.insert(heap.begin(), pD);
		return;
	}

	// check if it must be placed at the end
	if (heap.size()>0 && heap.back()->d <= pD->d){
		heap.push_back(pD);
		return;
	}

	
	// insert in the middle
	vector<FMPixelData*>::iterator it;
	for (it=heap.begin(); it!= heap.end(); it++){
		FMPixelData* current = *it;
		//if current is smaller, insert there
		if(pD->d <= current->d){
			it = heap.insert(it,pD);
			return;
		}
	}
	
	heap.push_back(pD);
	return;
}

void FMGridMap::insertIntoHeapStar(FMPixelData* pD){
	
	/* check if it must be placed at the beginning*/
	if (heap.size()>0 && heap[0]->optimalCost >= pD->optimalCost){
		heap.insert(heap.begin(), pD);
		return;
	}

	/* check if it must be placed at the end*/
	if (heap.size()>0 && heap.back()->optimalCost <= pD->optimalCost){
		heap.push_back(pD);
		return;
	}

	/* insert in the middle */
	vector<FMPixelData*>::iterator it;
	for (it=heap.begin(); it!= heap.end(); it++){
		FMPixelData* current = *it;
		//if current is smaller, insert there
		if(pD->optimalCost <= current->optimalCost){
			it = heap.insert(it,pD);
			return;
		}
	}
	
	heap.push_back(pD);
	return;
	
}

vector<FMPixelData*>::iterator FMGridMap::findElementStar(FMPixelData* pD){
	unsigned int init = 0;
	unsigned int end = heap.size();
	unsigned int middle = (end + init) / 2;

	while ( pD != heap[middle] && heap[middle]->optimalCost != pD->optimalCost && heap[middle+1]->optimalCost != pD->optimalCost ){
		if ( heap[middle]->optimalCost > pD->optimalCost ){ // it is before
			end = middle;
			middle = (end + init) / 2;
		} else{ //it is after
			init = middle;
			middle = (end + init) / 2;
		}
	}
	
	/*it must be just before or after */

	int aux=0;
	for (int i=0; i<heap.size(); i++)
	{
		aux = middle + i;
		if ( heap[aux] == pD ){
			return (heap.begin()+aux);
		}


		aux = middle - i;
		if ( heap[aux] == pD ){
			return (heap.begin()+aux);
		}

	}
}

vector<FMPixelData*>::iterator FMGridMap::findElement(FMPixelData* pD){

	static double meth1 = 0;
	MRTime timer;
	timer.precistic();

	unsigned int init = 0;
	unsigned int end = heap.size();
	unsigned int middle = (end + init) / 2;

	while ( pD != heap[middle] && heap[middle]->d != pD->d && heap[middle+1]->d != pD->d ){
		if ( heap[middle]->d > pD->d ){ // it is before
			end = middle;
			middle = (end + init) / 2;
		} else{ //it is after
			init = middle;
			middle = (end + init) / 2;
		}
	}
	
	/*it must be just before or after */

	int aux=0;
	for (int i=0; i<heap.size(); i++)
	{
		aux = middle + i;
		if ( heap[aux] == pD ){
			return (heap.begin()+aux);
		}


		aux = middle - i;
		if ( heap[aux] == pD ){
			return (heap.begin()+aux);
		}

	}
}

} // end namespace
