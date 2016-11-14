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
#include "fmstargridmap.h"
#include "fmgridmap.h"
#include <mrcore.h>
#include <float.h>
#include <limits.h>
//#include "config.h"
//#include "image.h"
//#include "gl/gltools.h"

namespace mr{

Image* FMStarGridMap::convertToImage(int w) const{
	Image *img =new Image(this->width, this->height);
	
	for (int x=0; x < this->width; x++){
		for (int y=0; y < height; y++){
			FMStarPixelData pdata = getCellValue(x,y);
			unsigned char graylevel;

			if (pdata.d == DBL_MAX || pdata.status != FMStarPixelData::FROZEN ) graylevel = 255;
			else{
				graylevel = (unsigned char)( pdata.d / max * 255. );
			}
			
			unsigned char color[] = {graylevel, graylevel, graylevel};
			img->setPixel(x,height-y-1,color);			
		}
	}

	
	if (w>0 && img->getWidth()!=w) img->scale(w);

	return img;
}

void FMStarGridMap::generatePlotData(string filename) const{

	ofstream data;
	data.open(filename.c_str());

	int min_x=INT_MAX; int min_y=INT_MAX;
	int max_x=0; int max_y=0;

	cout << "a" << endl;
	
	for (int x=0; x < this->width; x++){
		for (int y=0; y < this->height; y++){
			FMStarPixelData pdata = getCellValue(x,y);
			if ( !(pdata.d == DBL_MAX || pdata.status != FMStarPixelData::FROZEN ) ){
				if(x<min_x) min_x = x; if (y<min_y) min_y = y;
				if(x>max_x) max_x = x; if (y>max_y) max_y = y;
			}
		}
	}

	cout << "b" << endl;
	
	for (int i=min_x; i<=max_x; i++){
		for (int j=min_y; j<=max_y; j++){
			FMStarPixelData pdata = getCellValue(i,j);
			if (pdata.d == DBL_MAX || pdata.status != FMStarPixelData::FROZEN ) data << 255 << "\t";
			else data << (int)(pdata.d / max * 255) << "\t";
		}
		data << endl;
	}

	data.close();
}

void FMStarGridMap::printGrid(){
	for (int y=0;y<getHeight();y++){
		for (int x=0; x<getWidth(); x++){
			double value = getCellValue(x,y).d;
			FMStarPixelData::Status status = getCellValue(x,y).status;
			if (status == FMStarPixelData::NON_VISITED) cout << "___\t";
			if (status == FMStarPixelData::NARROW_BAND) cout << setprecision (3) << value <<"\t";
			if (status == FMStarPixelData::FROZEN) cout  <<"*"<< setprecision (3) << value<<"*\t";
		}
		cout << endl;
	}

	cout << "Pulsar tecla ... " << endl;
	string a;
	cin >> a;	
}

void FMStarGridMap::writeToStream(Stream& os){
//TODO
}

void FMStarGridMap::readFromStream(Stream& is){
//TODO
}

void FMStarGridMap::drawGL(){
	GridMap::drawGL();
}

void FMStarGridMap::buildPreComputedDistances(FMGridMap* fmmap){

	if (precomputedDists !=NULL){
		for (int i=0; i< fmmap->getWidth() ; i++){
			delete [] precomputedDists[i];
		}
		delete [] precomputedDists;
	}
	
	
	precomputedDists = new double*[width];
	
	for (int i=0; i< fmmap->getWidth() ; i++){
		precomputedDists[i]=new double[fmmap->getHeight()];
	}
	
	for (int i=0; i< fmmap->getWidth() ; i++){
		for (int j=0; j<fmmap->getHeight(); j++){
			double dist =  sqrt( square(i) + square(j) )  * max_speed_inv ;
			precomputedDists[i][j]=dist;
		}
	}
}

void FMStarGridMap::buildFromFMGridMap(FMGridMap* fmmap, int targetX, int targetY, int initX, int initY){
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

	
	MRTime timer;
	timer.precistic();
	buildPreComputedDistances(fmmap);
	cout << "Precomputed distances: "<< timer.precistoc() << endl;

	/******* INITIALIZE ***********/
	
	for (int i=0;i<width;i++){
		for (int j=0;j<height;j++){			
			FMStarPixelData aux(DBL_MAX, DBL_MAX, FMStarPixelData::NON_VISITED);
			aux.x=i; aux.y=j;
			setCellValue(i,j,aux);
		}
	}
	
	int xdist = Abs(initX-targetX);
	int ydist = Abs(initY-targetY);
	double optimal_cost = precomputedDists[xdist][ydist];
	//( sqrt( square(initX-targetX) + square(initY-targetY) ) * max_speed_inv );

	FMStarPixelData aux(0,optimal_cost,FMStarPixelData::FROZEN);
	aux.x=targetX; aux.y=targetY;
	setCellValue(targetX,targetY,aux);
	insertIntoHeap(&aux);

	
	/*********** LOOP *****************/
	//while the heap is not empty, expand the wave

	while(!heap.empty()){
		//cout << heap.size() << endl;
		//Extract v from H
		//printGrid();
		FMStarPixelData* current = heap.front();
		heap.erase(heap.begin());
		
		//Freeze v
		current->status = FMStarPixelData::FROZEN;
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
				if (x<0 || y <0 || x>=width || y >=height) continue;
				FMStarPixelData* pData = getPointerToCell(x,y);
				//compute distance
				if (pData->status != FMStarPixelData::FROZEN){
					double speed = fmmap->getCellValue(x,y).d;

					if (speed!=0){
						double distance = computeDistance(x,y,speed);
						int distx = Abs(initX-x); int disty=Abs(initY-y);
						double optimal_cost = distance + precomputedDists[distx][disty];//( sqrt( square(initX-x) + square(initY-y) ) / max_speed );
						
						if (pData->status != FMStarPixelData::NARROW_BAND){
							pData->status = FMStarPixelData::NARROW_BAND;
							pData->d=distance;
							pData->optimalCost = optimal_cost;
							insertIntoHeap(pData);
						}else{
							if (optimal_cost != pData->optimalCost) changeLocation(pData, optimal_cost);
							pData->d=distance;
						}
					}else{ //speed==0 //DO NOT INSERT IT INTO HEAP, WE DO NOT WANT TO EXPAND IT
						pData->status = FMStarPixelData::FROZEN;
						pData->d=DBL_MAX;
						pData->optimalCost = DBL_MAX;
					}
				}
				//printGrid();
			} //end for l
		}//end for aux
	}
}


double FMStarGridMap::computeDistance(int x, int y, double speed){
	
	FMStarPixelData vb,vc,vd,ve;

	if (x<=0 || y<0){
		vb.d=DBL_MAX; vb.status=FMStarPixelData::NON_VISITED;
	}else{
		vb = getCellValue(x-1,y);
	}

	if (x >= width-1 || y >=height){
		vc.d=DBL_MAX; vc.status=FMStarPixelData::NON_VISITED;
	}else{
		vc = getCellValue(x+1,y);
	}

	if (y<=0 || x<0){
		vd.d=DBL_MAX; vd.status=FMStarPixelData::NON_VISITED;
	}else{
		vd = getCellValue(x,y-1);
	}

	if (y>=height-1 || x>=width){
		ve.d=DBL_MAX; ve.status=FMStarPixelData::NON_VISITED;
	}else{
		ve = getCellValue(x,y+1);
	}

	double T1,T2;

	if (vb.status==FMStarPixelData::FROZEN && vc.status==FMStarPixelData::FROZEN ){
		T1 = Min(vb.d, vc.d);
	}else{
		if (vb.status==FMStarPixelData::FROZEN) T1 = vb.d; //only vb is frozen
		else if (vc.status==FMStarPixelData::FROZEN) T1 = vc.d; //only vc is frozen
		else T1=DBL_MAX; //none is frozen
	}
	
	if (vd.status==FMStarPixelData::FROZEN && ve.status==FMStarPixelData::FROZEN ){
		T2 = Min(vd.d, ve.d);
	}else{
		if (vd.status==FMStarPixelData::FROZEN) T2 = vd.d; //only vb is frozen
		else if (ve.status==FMStarPixelData::FROZEN) T2 = ve.d; //only vc is frozen
		else T2=DBL_MAX; //none is frozen
	}
	
	double distance;

	double square_speed_inv = 1/square(speed);
	
	if (T1==DBL_MAX) distance = square_speed_inv + T2;
	else if (T2==DBL_MAX) distance = square_speed_inv + T1;
	else{
		double a = 2;
		double b = -2*(T1+T2);
		double c = (square(T1) + square(T2)) - square_speed_inv;
		double quad_term = (square(b) - 4*a*c);
		if ( quad_term < 0 ) distance = square_speed_inv + min(T1,T2) ;
		else distance = (-b + sqrt(quad_term))/(2*a);
	}
	 
	return distance;	
	
}

void FMStarGridMap::changeLocation(FMStarPixelData* pD, double new_dist){
	vector<FMStarPixelData*>::iterator it = findElement(pD);
	heap.erase(it);
	pD->optimalCost=new_dist;
	insertIntoHeap(pD);
}

void FMStarGridMap::insertIntoHeap(mr::FMStarPixelData* pD){
	
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
	vector<FMStarPixelData*>::iterator it;
	for (it=heap.begin(); it!= heap.end(); it++){
		FMStarPixelData* current = *it;
		//if current is smaller, insert there
		if(pD->optimalCost <= current->optimalCost){
			it = heap.insert(it,pD);
			return;
		}
	}
	
	heap.push_back(pD);
	return;
	
}

vector<FMStarPixelData*>::iterator FMStarGridMap::findElement(FMStarPixelData* pD){


	unsigned int init = 0;
	unsigned int end = heap.size();
	unsigned int middle = (end + init) / 2;

	while ( pD != heap[middle] && heap[middle]->optimalCost != pD->optimalCost && heap[middle+1]->d != pD->optimalCost ){
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

} // end namespace
