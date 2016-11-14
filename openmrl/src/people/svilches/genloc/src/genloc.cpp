#include "genloc.h"

	// GLOBAL VARIABLES:
map<string,Object*> scene;
int debug = 0;
int generationCounter = 0;
mr::LabeledGridMap * storedMap_ptr;	// Here we store the map of the world.
Generation * myGeneration;


int main(int argc, char *argv[])
{
	/*
	 * Debugging configuration (genloc -d*, * between 1 and 3)
	 */


	 if (argc > 1)
	 {
		 if (argv[1][0] == '-' && argv[1][1] == 'd'){
			switch (argv[1][2]){
				case '1':
					debug = 1;
					break;
				case '2':
					debug = 2;
					break;
				case '3':
					debug = 3;
					break;
				default:
					cout << "Unknown command line option: " << "-" << argv[1][2] << endl;
					break;
			}
		}
	}
	if (debug) cout << "Debugging mode " << debug << endl;
	else cout << "You can turn on debugging with -d1, -d2 or -d3.\n";
	 
	/*
	 * GLUT initialization
	 */
	 
	atexit(clearScene);
	glutInit(&argc, argv);
	glutInitWindowSize(600,550);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("GL");
	glutDisplayFunc(OnDraw);
	//glutMotionFunc(OnMouseMove);
	//glutMouseFunc(OnMouseClick);
	glutKeyboardFunc(OnKeyboardDown);

	initializeScene();

	//glutTimerFunc(5,OnTimer,0);
	glutMainLoop();
	return 0; 
}

/*
 * This is executed only once in the program: 
 */
void initializeScene(){
	/*
	 * Read the sensor log (Simulating a real scan from the IR sensor)
	 * This program just makes one localization.
	 */
	 
	GenericRangeData sensorReadings;
	
    ifstream logFile ("sensor_log.dat");

	if (logFile.is_open()){
		//
		if (debug > 0) cout << "(Debug 1)       " <<"Opening sensor_log.dat "<< endl;
		//
		// We want to fill sensorReadings from a file with the origin of the data and the raw data itself.
		// To convert the raw data into useful data, we use the GenericRangeSensor class:
		GenericRangeSensor auxSensor;
		// Loading the raw data to the auxSensor, using the overloaded operator >>:
		logFile >> auxSensor;
		// And dumping the useful data from the auxSensor to sensorReadings
		
		if (debug > 2) cout << "      (Debug 3) " <<" auxSensor contains "<< auxSensor.sensorData.readings.size() << " values." << endl;

		
		sensorReadings.push(auxSensor.sensorData);
		// Done!
		logFile.close();
		
		//
		if (debug > 2) cout << "      (Debug 3) " <<" Added "<< sensorReadings.readings.size() << " values to sensorReadings" << endl;
		//
	}
	else{
		cerr << "ERROR: Unable to open log file. Exiting program" << endl;
		//return -1;
	}
	
	
	
	/*
	 * Store the map of the world, reading it from "internal_map.bmp"
	 */
	storedMap_ptr = createMapFromBMP();
	
	scene["z0_map"]=storedMap_ptr;


	/*
	 * Create generation of possible solutions
	 */
	//int _popSize, float _crossFactor, mr::LabeledGridMap * _storedMap_ptr, vector <sensorData> _sensorReadings )
	myGeneration = new Generation(20, 0.1, storedMap_ptr, sensorReadings);
	
	scene["z1_generation"]=myGeneration;
	
	// Seeding the random generator
	srand((unsigned)time(0));
	
	myGeneration->generateRandomPopulation();
	//
	if (debug > 0) cout << "(Debug 1)       " << "Generation 0:\n" << *myGeneration << endl;
	//
	
	/*
	 * Saving internalMap_ptr as map.bmp for testing
	 */
	/* 
	mr::Image *bitMap_ptr;
	bitMap_ptr = storedMap_ptr->convertToImage();
	cout << "Saving map.bmp" << endl;
	bitMap_ptr->save("map.bmp");
	*/
	glutPostRedisplay();
}

void OnKeyboardDown(unsigned char key, int x, int y)
{
	cout << "OnKeyboardDown" << endl; // Debugging FIXME
	
	/*
	 * Evolve!!!
	 */
	//
	if (debug > 0) cout << "(Debug 1)       " <<" Creating generation " << generationCounter + 1 << endl;
	//
	myGeneration->evolveGeneration();
	//
	if (debug > 0) cout << "(Debug 1)       " << "Generation " << generationCounter+1 << ":\n" << *myGeneration << endl;
	
	generationCounter++;
	
	glutPostRedisplay();
}


void OnDraw(void)
{
	cout << "OnDraw" << endl; // Debugging 
	
	if (scene.size()==0) return;
	
	// Clearing screen 
	glClear(GL_COLOR_BUFFER_BIT );

	//Defining viewpoint
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	double x0,y0,xf,yf;
	x0 = storedMap_ptr->getMapX();
	y0 = storedMap_ptr->getMapY();
	xf = x0 + storedMap_ptr->getRealWidth();
	yf = y0 + storedMap_ptr->getRealHeight();
	
	gluOrtho2D(x0,xf,y0,yf);
	
	/*map<string,Object*>::iterator it;
	for (it=scene.begin(); it!=scene.end(); it++){
	  (it->second)->drawGL2D();
	}*/
	
	storedMap_ptr->drawGL2D();
	myGeneration->drawGL2D();

	//mr::Point2o punto(0.0,0.0,0);
	//punto.drawGL2D();

	glutSwapBuffers();
}
	
	
void clearScene(){
	 cout << "clearScene()" << endl; // Debugging FIXME
	 map<string,Object*>::iterator it;
	 for (it=scene.begin(); it!=scene.end(); it++){
		  delete (it->second);	  
	}
	//
	if (debug > 0) cout << "(Debug 1)       " <<"End of main "<< endl;
	//
}
	
	
	



/***************************************************************************************************
 * createMapFromBMP()
 * Returns a pointer to an LabeledGridMap with the info from the bitmap file "internal_map.bmp"
 * 
 **************************************************************************************************/

mr::LabeledGridMap * createMapFromBMP(){
	/*
	 * Loading map file and storing as an Image
	 */
	 
	//
	if (debug > 0) cout << "(Debug 1)       " <<"Loading internal_map.bmp "<< endl;
	//
	mr::Image * internalMapImage = new mr::Image;
	// Loading the image and checking if it exists:
	internalMapImage->load("internal_map.bmp");
	/*if (!(internalMapImage->load("internal_map.bmp"))){
		cerr << "ERROR: No bitmap of the map found! Exiting program" << endl;
		exit(0); 
	}*/
	/*
	 * Convert the Image to an LabeledGridMap
	 */
	// Note that we are using different resolutions for this map and for the sensor's map, and that we are defaulting to 255 (white)
	float mapRes = 0.005; // Width of each pixel of the bmp image describing the world (m)
	float mapWidth = mapRes * internalMapImage->getWidth();
	float mapHeight = mapRes * internalMapImage->getHeight();
	// mr::LabeledGridMap (double real_width, double real_height, double resolution, unsigned char default_value=127, double mapX=0, double mapY=0);
	mr::LabeledGridMap *internalMap_ptr = new mr::LabeledGridMap(mapWidth, mapHeight, mapRes, 255, -0.1 , -0.1);
	// Now we will convert all the pixels from the image to cells in the LabeledGridMap
	// void loadFromImage(Image* img, double real_width, double resolution, double mapX=0, double mapY=0);
	internalMap_ptr->loadFromImage(internalMapImage,mapWidth,mapRes,-0.1,-0.1);
	return internalMap_ptr;
}





