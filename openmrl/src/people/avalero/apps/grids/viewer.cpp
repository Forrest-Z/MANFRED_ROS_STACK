// PruebaMRcore.cpp : Defines the entry point for the console application.
//


#include "mrcore/gl/glscene2d.h"
#include <GL/glut.h>
#include <iostream>
#include "image.h"
#include "evidencegridmap.h"
#include "occupancygridmap255.h"
#include "mapviewer.h"
#include <cmath>

using namespace mr;
using namespace std;



int main(int argc, char* argv[])
{
	EvidenceGridMap* map = new EvidenceGridMap(20, 20, 0.01, EvidenceGridMap::FREE,0, 0);

	Point2 pointi(0,0);
	Point2 pointf(20.0,20.0);
	EvidenceGridMap::Ray ray = map->getRay(pointi,pointf);
	
	for(int i=0;i<ray.size();i++){
		map->setCellValue(ray[i].pixelX, ray[i].pixelY,EvidenceGridMap::OBSTACLE);
	}
	
	
	GLViewMap(map, &argc, argv);
	
	return 0;
}
