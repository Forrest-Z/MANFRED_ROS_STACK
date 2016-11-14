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
 **********************************************************************/
 
#include "tictoc.h"

#ifndef _WIN32

#ifdef HAVE_CLOCK_GETTIME

#ifdef CLOCK_HIGHRES
#define SAMPLED_CLOCK CLOCK_HIGHRES
#else
#define SAMPLED_CLOCK CLOCK_REALTIME
#endif

void tic(tictoc *tv) {
  if (clock_gettime(SAMPLED_CLOCK, tv)) 
    tv->tv_sec = tv->tv_nsec = -1;
}

double toc(tictoc *tv) {
  struct timespec tv2;

  if (clock_gettime(SAMPLED_CLOCK, &tv2)) 
    tv2.tv_sec = tv2.tv_nsec = -1;

  double  sec = static_cast<double>(tv2.tv_sec - tv->tv_sec);
  double nsec = static_cast<double>(tv2.tv_nsec - tv->tv_nsec);

  return (sec + 1.0e-9 * nsec);
}
#else

#ifdef HAVE_GETTIMEOFDAY

void tic(tictoc *tv) {
  gettimeofday(tv, 0L);
}

double toc(tictoc *tv) {
  tictoc tv2;

  gettimeofday(&tv2, 0L);
  double  sec = static_cast<double>(tv2.tv_sec - tv->tv_sec);
  double usec = static_cast<double>(tv2.tv_usec - tv->tv_usec);

  return (sec + 1.0e-6 * usec);
}

#else
// Fall back to C/C++ low resolution time function.

void tic(tictoc *tv) {
  time(tv);
}

double toc(tictoc *tv) {
  tictoc tv2;
  time(&tv2);
  return difftime(tv2, *tv);
}

#endif

#endif

#else

// Windows.

void tic(tictoc *tv) {
  *tv = GetTickCount();
}

double toc(tictoc *tv) {
  tictoc tv2;
  tv2 = GetTickCount();
  return 1.0e-3 * (tv2 - *tv);
}

#endif
