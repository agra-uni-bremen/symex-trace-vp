/*

  This program is part of the TACLeBench benchmark suite.
  Version V 1.9

  Name: deg2rad

  Author: unknown

  Function: deg2rad performs conversion of degree to radiant

  Source: MiBench
          http://wwweb.eecs.umich.edu/mibench

  Original name: basicmath_small

  Changes: no major functional changes

  License: this code is FREE with no restrictions

*/

#include "pi.h"
#include "symex.h"

#define deg2rad(d) ((d)*PI/180)


/*
  Forward declaration of functions
*/

void deg2rad_init( void );
void deg2rad_main( void );
int deg2rad_return( void );
int main( void );


/*
  Declaration of global variables
*/

float deg2rad_X, deg2rad_Y;


/*
  Initialization function
*/

void deg2rad_init( void )
{
  deg2rad_X = 0;
  deg2rad_Y = 0;
}


/*
  Return function
*/

int deg2rad_return( void )
{
  int temp = deg2rad_Y;

  if ( temp == 1133 )
    return 0;
  else
    return -1;

}


/*
  Main functions
*/

void _Pragma( "entrypoint" ) deg2rad_main( void )
{
  float f = guess_symbolic_unsigned_float();
  deg2rad(f);
}

int main( void )
{
  deg2rad_init();
  deg2rad_main();
  deg2rad_return(); // We ignore the return value since we need to call symex_exit().
  symex_exit();
}
