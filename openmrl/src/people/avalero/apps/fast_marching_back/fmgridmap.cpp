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
//#include "config.h"
//#include "image.h"
//#include "gl/gltools.h"

namespace mr{

Image* FMGridMap::convertToImage(int w) const{
	Image *img =new Image(this->width, this->height);
	double aux=0;

	for (int x=0; x < this->width; x++){
		for (int y=0; y < height; y++){
			FMPixelData pdata = getCellValue(x,y);
			if (pdata.d > aux) aux=pdata.d;
			unsigned char graylevel = (unsigned char)( pdata.d / max * 255. );
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

void FMGridMap::buildFromLabeledGridMap(LabeledGridMap* lmap){
	max=0;
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


	/*********** LOOP *****************/
	//while the heap is not empty, expand the wave

	while(!heap.empty()){
		//Extract v from H
		FMPixelData* current = heap.front();
		heap.erase(heap.begin());
		
		//Freeze v
		current->status = FMPixelData::FROZEN;
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
				FMPixelData* pData = getPointerToCell(x,y);
				//compute distance
				if (pData->status != FMPixelData::FROZEN){
					double distance = computeDistance(x,y);
					double prev_distance = pData->d;
			
					if (distance > max){
						max = distance; //we keep the max distance value;
					}
					
					if (pData->status != FMPixelData::NARROW_BAND){
						pData->status = FMPixelData::NARROW_BAND;
						pData->d=distance;
						insertIntoHeap(pData);
					}else{
						if (distance != prev_distance) changeLocation(pData, distance);
					}
				}
			} //end for l
		}//end for aux
	}

	//printGrid();
}

double FMGridMap::computeDistance(int x, int y){

	//if (x<=1 || y<=1 || x >= getWidth()-2 || y >= getHeight()-2) return 0;

	FMPixelData vb,vc,vd,ve;

	if (x<=0 || y<0){
		vb.d=DBL_MAX; vb.status=FMPixelData::NON_VISITED;
	}else{
		vb = getCellValue(x-1,y);
	}

	if (x >= width-1 || y >=height){
		vc.d=DBL_MAX; vc.status=FMPixelData::NON_VISITED;
	}else{
		vc = getCellValue(x+1,y);
	}

	if (y<=0 || x<0){
		vd.d=DBL_MAX; vd.status=FMPixelData::NON_VISITED;
	}else{
		vd = getCellValue(x,y-1);
	}

	if (y>=height-1 || x>=width){
		ve.d=DBL_MAX; ve.status=FMPixelData::NON_VISITED;
	}else{
		ve = getCellValue(x,y+1);
	}

	double T1,T2;
	
	if (vb.status==FMPixelData::FROZEN && vc.status==FMPixelData::FROZEN ){
		T1 = Min(vb.d, vc.d);
	}else{
		if (vb.status==FMPixelData::FROZEN) T1 = vb.d; //only vb is frozen
		else if (vc.status==FMPixelData::FROZEN) T1 = vc.d; //only vc is frozen
		else T1=0; //none is frozen
	}
	
	if (vd.status==FMPixelData::FROZEN && ve.status==FMPixelData::FROZEN ){
		T2 = Min(vd.d, ve.d);
	}else{
		if (vd.status==FMPixelData::FROZEN) T2 = vd.d; //only vb is frozen
		else if (ve.status==FMPixelData::FROZEN) T2 = ve.d; //only vc is frozen
		else T2=0; //none is frozen
	}

	double distance;
	
	if (T1==DBL_MAX) T1=0;
	if (T2==DBL_MAX) T2=0;

	if (T1!=0 && T2!=0)	distance = ( (T1 + T2) + sqrt( square(T1 + T2) - 2*( square(T1) + square(T2) - 1) ) ) / 2.0;
	else if (T1==0) distance = 1 + T2;
	else if (T2==0) distance = 1 + T1;
	 

	return distance;	
	
}

void FMGridMap::changeLocation(FMPixelData* pD, double new_dist){
	vector<FMPixelData*>::iterator it = findElement(pD);
	heap.erase(it);
	pD->d=new_dist;
	insertIntoHeap(pD);
}
void FMGridMap::insertIntoHeap(mr::FMPixelData* pD){
	
	/* check if it must be placed at the beginning*/
	if (heap.size()>0 && heap[0]->d >= pD->d){
		heap.insert(heap.begin(), pD);
		return;
	}

	/* check if it must be placed at the end*/
	if (heap.size()>0 && heap.back()->d <= pD->d){
		heap.push_back(pD);
		return;
	}

	/* insert in the middle */
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
