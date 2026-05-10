#include <stdio.h>

int extractBits(int num, int pos, int n) {
    int mask = ((1 << n) - 1) << pos;
    return (num & mask) >> pos;
}

int main() {
    int num, pos, n;
    printf("Enter number, start position, number of bits: ");
    scanf("%d %d %d", &num, &pos, &n);
    int extracted = extractBits(num, pos, n);
    printf("Extracted bits value: %d\n", extracted);
    return 0;
}