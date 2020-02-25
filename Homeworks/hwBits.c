/*
 * Homework problems on bit manipulation
 */
#include <stdio.h>
#include <stdlib.h>

void displayBits(unsigned int b);
int countBits(unsigned int b);
unsigned int pairwiseSwap(unsigned int b);

const unsigned int varzero = 0;

void main()
{

unsigned int n = 23;
printf("Bits in the number %d:\n",n);
displayBits(n);

/* Test countBits */
printf("Number of bits in %d = %d\n\n", n, countBits(n));

/* Swap even and odd bits */
n = 0x8234abcd;
printf("Bits in %x:\n", n);
displayBits(n);
printf("\n");
printf("Pair-wise swap: \n");
displayBits(pairwiseSwap(n));
printf("\n");


}

/*
 * Swaps odd and even bits in an unsigned int, e.g.,
 * bits 0 and 1 are swapped, bits 2 and 3 are swapped, etc
 * For example: Suppose
 *
 *        b = 1000 0001 0010 0011 1010 1011 1100 1101
 *
 *  Then the function returns
 *
 *            0100 0010 0001 0011 0101 0111 1100 1110
 *
 *  Obviously the function doesn't work.  Rewrite it so
 *  it works correctly and using only BarelyC instructions
 *  given in the homework.
 *
 *  Hint:  Use shift and bit-wise logic operations
 */
unsigned int pairwiseSwap(unsigned int b)
{
    unsigned int var1; //Left bit
    unsigned int var2; //Right bit
    unsigned int var3; //Switch

    var1 = (b & 0b10101010101010101010101010101010) >> 1;
    var2 = (b & 0b01010101010101010101010101010101) << 1;
    var3 = var1 | var2;
return var3;
}




/*
 * Counts the number of bits in b
 * For example, if b = 23 then its bits are 00....10111,
 * and so the function should return the value 4.
 * Obviously, the function doesn't work.
 * Fix the function so it works but use only BarelyC
 * instructions as stated in the homework.
 */
int countBits(unsigned int b)
{
    unsigned int storage;      // Storages b & 1
    unsigned int BitsCounter = 0; // Counter

    Loop_While:
            if (b == 0) goto Skip_While;
                storage = b & 1;
                BitsCounter = BitsCounter + storage;
                b = b >> 1;
                goto Loop_While;
    Skip_While:

return BitsCounter;
}

/* Displays the bits in b */
void displayBits(unsigned int b)
{
unsigned int bits = b;
int i;
for (i=1; i<=32; i++) {
   if ((bits >> 31) & 1 == 1) {
      printf("1");
   }
   else {
      printf("0");
   }
   if (i%8 == 0) {
      printf(" ");
   }
   bits = bits << 1;
}
printf("\n");
}


