#ifndef GENLOC_H
#define GENLOC_H

#include <datatype/gridmap_includes.h>
#include <datatype/geometry/path2d.h>
#include <GL/glut.h>

#include <iostream>
#include <datatype/gridmap/labeledgridmap.h>
#include <fstream>
#include <string.h>
#include <sstream>
#include <math.h>
#include "generation.h"

#include <map>
#include <vector>
#include <iostream>

using namespace std;
using namespace mr;

void clearScene(); //clear all allocated memory
void initializeScene(); //creates the world
void OnDraw(void); //it will be called when the screen needs to be redrawn
//void OnTimer(int value); //it will be called with a timer
void OnKeyboardDown(unsigned char key, int x, int y); //when a key is pressed
//void OnMouseMove(int x,int y); //when the mouse moves
//void OnMouseClick(int button,int state, int x,int y); //when a button is pressed

mr::LabeledGridMap * createMapFromBMP();

	
#endif /* GENERATION_H_ */
