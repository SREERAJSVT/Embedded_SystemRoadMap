#include <stdio.h>

int main() {
    int n, i, isPrime = 1;
    printf("enter a positive integer: ");
    scanf("%d", &n);

    if (n < 2) isPrime = 0;
    else {
        for (i = 2; i * i <= n; i++) {
            if (n % i == 0) {
                isPrime = 0;
                break;
            }
        }
    }

    printf("%d is %sprime.\n", n, isPrime ? "" : "not ");
    return 0;
}