#include <stdio.h>

int isPerfect(int num) {
    int sum = 0, i;
    for (i = 1; i <= num / 2; i++) {
        if (num % i == 0)
            sum += i;
    }
    return (sum == num);
}

int main() {
    int num;
    printf("Enter a positive integer: ");
    scanf("%d", &num);

    if (isPerfect(num))
        printf("%d is a perfect number.\n", num);
    else
        printf("%d is not a perfect number.\n", num);

    return 0;
}