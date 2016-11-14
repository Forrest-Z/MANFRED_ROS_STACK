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
 
#include "mapviewer.h"

mr::GLScene2D scene2d;
	
void GLViewMap(mr::GLObject* m, int *argcp, char **argv){
	scene2d.addObject(m);
	InitGL(argcp, argv);
}

void OnDraw(void)
{
	scene2d.Draw();
	glutSwapBuffers();
}
void OnKeyboardDown(unsigned char key, int x, int y)
{
	scene2d.KeyDown(key);
	glutPostRedisplay();	

}
void OnMouseClick(int b,int state, int x,int y)
{
	/*
	bool down=(state==GLUT_DOWN);
	int button;
	if(b==GLUT_LEFT_BUTTON)
		button=MOUSE_LEFT_BUTTON;
	if(b==GLUT_RIGHT_BUTTON)
		button=MOUSE_RIGHT_BUTTON;
		
	//scene.MouseButton(x,y,b,down,sKey,ctrlKey);
	glutPostRedisplay();
	*/
}
void OnMouseMove(int x,int y)
{
	/*
	scene.MouseMove(x,y);
	glutPostRedisplay();
	*/
}


 
void InitGL(int *argcp, char **argv){
 //GL Initialization stuff

	glutInit(argcp, argv);
	glutInitWindowSize(800,800);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutCreateWindow("GL");
	glutDisplayFunc(OnDraw);
	glutMotionFunc(OnMouseMove);
	glutMouseFunc(OnMouseClick);
	glutKeyboardFunc(OnKeyboardDown);
	scene2d.init();
	glutMainLoop();
}
 
