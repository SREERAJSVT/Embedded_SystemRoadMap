#include <stdio.h>

void swapBits(int *a, int *b, int pos, int n) {
    int mask = ((1 << n) - 1) << pos;
    int bitsA = (*a & mask) >> pos;
    int bitsB = (*b & mask) >> pos;
    // Clear both positions
    *a &= ~mask;
    *b &= ~mask;
    // Set swapped bits
    *a |= (bitsB << pos);
    *b |= (bitsA << pos);
}

int main() {
    int a, b, pos, n;
    printf("Enter two numbers, position, n: ");
    scanf("%d %d %d %d", &a, &b, &pos, &n);
    swapBits(&a, &b, pos, n);
    printf("After swap: a = %d, b = %d\n", a, b);
    return 0;
}