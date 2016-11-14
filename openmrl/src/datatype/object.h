/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  -----------anyone
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
#ifndef __MRL__OBJECT__H
#define __MRL__OBJECT__H

#include <string>
#include <map>
#include <vector>

using namespace std;

namespace mr {

	 /*!
    \class Object
    \brief Object, the base class that supports RTTI and serialization.
*/
	 class Object
	 {
		 friend class RegistrationHelper;
	 public:
		 Object(){rgb[0]=rgb[1]=rgb[2]=0;}
		 static Object* create(string className);
		 virtual string getClassName()=0;
		 virtual void drawGL2D() const {}
		 virtual ~Object(){}

		 inline void setRGB(float r, float g, float b){
			  if (r<0) r=0;
			  else if (r>1) r=1;

			  if (g<0) g=0;
			  else if (g>1) g=1;

			  if (b<0) b=0;
			  else if (b>1) b=1;

			  rgb[0]=r;rgb[1]=g;rgb[2]=b;
		 }

	 protected:
		 ///The map is a singleton, static accessed by this method
		 static map<string,Object* (*)()>& getRegistry();

		 float rgb[3];
	 };



	 class RegistrationHelper
	 {
	 public:
		 RegistrationHelper(string name, Object* (*p)())
		 {
			 Object::getRegistry()[name]=p;
		 }

	 };

	 /**Macros **/
	 #define DECLARE_MRL_OBJECT(class_name)						\
				  public:											\
				 static Object* create(){return new class_name;} \
				 string getClassName(){return className;}        \
				private:											\
				 static string className;

	 #define IMPLEMENT_MRL_OBJECT(class_name) \
		 RegistrationHelper reg##class_name##Helper(#class_name , &class_name::create); \
		 string class_name::className=#class_name; \
		 //const int dummyRegistration=reg##class_name##Helper.do_nothing();
	 };
	 #endif  //__MRCORE__OBJECT_H













