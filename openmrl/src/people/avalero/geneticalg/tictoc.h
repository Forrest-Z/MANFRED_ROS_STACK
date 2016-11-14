/**********************************************************************
 *
 * This code is part of the MRcore projec
 * Author:  Diego Rodríguez-Losada
 *			Alberto Valero
 *			Paloma de la Puente
 *			Miguel Hernando
 *			Luis Pedraza
 *			Pablo San Segundo
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
 **********************************************************************/#ifndef TICTOC_H__
#define TICTOC_H__

#include "config.h"

#ifdef WIN32

#include <windows.h>
typedef DWORD tictoc;
#else

#ifdef HAVE_CLOCK_GETTIME
#include <time.h>
typedef struct timespec tictoc;
#else

#ifdef HAVE_GETTIMEOFDAY
#include <sys/time.h>
typedef struct timeval tictoc;
#else
#include <ctime>
typedef time_t tictoc;
#endif

#endif

#endif

void   tic(tictoc *tv);   /* start timing. */
double toc(tictoc *tv);   /* stop  timing. */

#endif
