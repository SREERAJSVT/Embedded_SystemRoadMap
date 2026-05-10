#include <stdio.h>

int toggleBits(int num, int pos, int n) {
    int mask = ((1 << n) - 1) << pos;
    return num ^ mask;
}

int main() {
    int num, pos, n;
    printf("Enter number, start position, number of bits: ");
    scanf("%d %d %d", &num, &pos, &n);
    int result = toggleBits(num, pos, n);
    printf("After toggling: %d\n", result);
    return 0;
}