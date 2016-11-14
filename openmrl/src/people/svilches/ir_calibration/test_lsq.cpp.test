#include "third-party/lmcurve.h"
#include <stdio.h>
#include <math.h>

/* model function: a quadratic function p0 + p1 * (t-p2)^2 */

double f( double t, const double *p )
{
    return p[0] * pow(t, -p[1]);
}

int main()
{
    /* parameter vector */

    int n_par = 2; // number of parameters in model function f
    double par[2] = { 1, 1}; // relatively bad starting value

    /* data pairs: slightly distorted standard parabola */

    int m_dat = 8; // number of data pairs
    int i;
    double t[8] = { 10, 20, 30, 40, 50, 60, 70, 80 };
    double y[8] = { 2.5, 1.3, 0.9, 0.75, 0.6, 0.5, 0.4, 0.4 };

    /* auxiliary parameters */
    lm_status_struct status;
    lm_control_struct control = lm_control_double;
    control.printflags = 2; // monitor status (+1) and parameters (+2)

    /* perform the fit */

    printf( "Fitting:\n" );
    lmcurve_fit( n_par, par, m_dat, t, y, f, &control, &status );
    printf("obtained parameters:\n");
    for ( i = 0; i < n_par; ++i)
	printf("  par[%i] = %12g\n", i, par[i]);   

    return 0;
}
