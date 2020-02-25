/*
 * Examples of two divide functions:
 *   divide1 -- Simple algorithm that keeps subtacting the divisor
 *   divide2 -- Faster algorithm that subtracts shifted versions of
 *        divisor.  It is similar to the fast multiply algorithm in
 *        the lecture notes.
 *
 * INSTRUCTIONS: convert the 'divide2' function into BarelyC
 *
 * BarelyC instructions and variables you may use:
 *
 *  You can declare and use the following
 *     unsigned int variables var1, var2, ...., var31
 *          Note that some have already been declared.
 *          You can also use 'varzero', which has been declared
 *              It's the variable that's always zero.
 *
 *  Instructions: (Note: varx, vary, varz are variables from above)
 *      varx = vary op varz;, where op is in {+, -, &, |, ^}
 *      varx = vary op const;,
 *           where op is in {+, -, &, |, ^, <<, >>}
 *           'const' is a constant, e.g.,
 *              varx = vary + 3; or
 *              varx = varx << 2;
 *      if (varx cmp vary) goto Label;
 *           where op is in {<, >, <=, >=, ==, !=}
 *      goto Label;
 *
 *  Note: You can pick any label as long it's not a keyword:
 *         Examples are "While3_Loop", "While3_Skip", "Else1", "Else1_Skip".
 *
 *  Note:  You can declare and use
 *           #define constants, e.g., #define MAX_QSIZE 16.
 *
 *  Note:
 *      Instructions that are not BarelyC
 *          var2 = 0;
 *          var2 = 8+1;
 *          if (var2 < 25) goto Label;
 *          if (var2) goto Label;
 */


#include <stdio.h>
#include <stdlib.h>

#define DIVISOR  7        /* Divisor */
#define NUMBER   125      /* Number to divide */
#define MAX_QSIZE 16  /* Maximum quotient size in bits */

const unsigned int varzero = 0;  /* This variable is always zero. */
                        /* While it is currently unused, it can
			 * be useful in your BarelyC code
			 */

void divide1(unsigned int n, unsigned int d); /* Just used for comparison */

/* Modify the following function into BarelyC */
void divide2(unsigned int n, unsigned int d);


void main()
{

divide1(NUMBER, DIVISOR);
divide2(NUMBER, DIVISOR);

}

void divide2(unsigned int n, unsigned int divisor)
{
unsigned int var2 = n; /* Remainder */
unsigned int var1 = divisor;
unsigned int var3;  /* Quotient */
unsigned int var4;  /* Used as a counter */

/* Convert the following into BarelyC
 * See above for the BarelyC instructions and variables you can use */
var1 = var1 << MAX_QSIZE;  /* var1 = divisor * 2^SIZE_QUOTIENT */
var3 = 0;
var4 = 0;

Loop_while:
	if(var4 >= MAX_QSIZE) goto Skip_while;
     		var1 = var1 >> 1;  /* Shift divisor to the right */
     		var3 = var3 << 1;  /* Make room for new bit in quotient */
	if(var2 < var1) goto Skip_update; /* If we can subtract the shifted divisor */
          	var3 = var3 | 1;   /* Record the quotient bit - set bit with OR */
          	var2 = var2 - var1;  /* Update remainder */
	Skip_update:
	var4 = var4 +1;
	goto Loop_while;
Skip_while:

/* End of the code to be converted to BarelyC*/

printf("divide2: n = %d, divisor = %d, quotient = %d, remainder = %d\n", n,var1,var3,var2);
}


/* Don't modify this function.  It's just used for comparison. */
void divide1(unsigned int n, unsigned int divisor)
{
unsigned int remainder = n;
unsigned int quotient = 0;
while(remainder >= divisor) {
     remainder = remainder - divisor;
     quotient++;
}
printf("divide1: n = %d, divisor = %d, quotient = %d, remainder = %d\n", n,divisor,quotient,remainder);
}



