#include <stdio.h>

int replaceBits(int num, int replaceFrom, int pos, int n) {
    int mask = ((1 << n) - 1) << pos;
    // Clear n bits in num at pos
    num &= ~mask;
    // Extract n bits from replaceFrom at pos, align them
    int bits = (replaceFrom >> pos) & ((1 << n) - 1);
    // Insert into num
    num |= (bits << pos);
    return num;
}

int main() {
    int num1, num2, pos, n;
    printf("Enter first number, second number, position, n: ");
    scanf("%d %d %d %d", &num1, &num2, &pos, &n);
    int result = replaceBits(num1, num2, pos, n);
    printf("Result: %d\n", result);
    return 0;
}