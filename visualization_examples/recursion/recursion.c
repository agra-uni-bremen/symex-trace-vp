/*
  This program is part of the TACLeBench benchmark suite.
  Version V 1.x

  Name: recursion

  Author: unknown

  Function: recursion is a recursion program.
    This program computes the Fibonacci number recursively.

  Source: MRTC
          http://www.mrtc.mdh.se/projects/wcet/wcet_bench/recursion/recursion.c

  Changes: no major functional changes

  License: May be used, modified, and re-distributed freely.

*/

#include "symex.h"


/*
   Global Variables
*/
int recursion_result;
int recursion_input;

/*
  Forward declaration of functions
*/
int recursion_fib( int i );
void recursion_main( void );
void recursion_init( void );
int recursion_return( void );
int main ( void );


void recursion_init()
{
  int volatile temp_input = 10;
  recursion_input = temp_input;
}


int recursion_fib( int i )
{
  if ( i == 0 )
    return 1;
  if ( i == 1 )
    return 1;

  return recursion_fib( i - 1 ) + recursion_fib( i - 2 );
}

int recursion_return()
{
  return ( recursion_result  + ( -89 ) ) != 0;
}

void _Pragma( "entrypoint" ) recursion_main( void )
{
  int num_iterations = 0;
  make_symbolic(&num_iterations, sizeof(num_iterations));
  if (num_iterations < 0 || num_iterations > 8) {
    symex_exit();
  }
  for (int i = 0; i < num_iterations; i++) {
    recursion_fib( recursion_input );
  }
}

int main( void )
{
  recursion_init();
  recursion_main();
  recursion_return(); // We discard the return value, because we have to call symex_exit().
  symex_exit();
}
