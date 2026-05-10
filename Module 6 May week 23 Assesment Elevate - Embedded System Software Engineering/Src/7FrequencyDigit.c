#include <stdio.h>

void countDigits(int num, int freq[]) {
    if (num == 0) {
        freq[0]++;
        return;
    }
    if (num < 0) num = -num;
    while (num > 0) {
        int digit = num % 10;
        freq[digit]++;
        num /= 10;
    }
}

int main() {
    int num, freq[10] = {0};
    printf("Enter an integer: ");
    scanf("%d", &num);

    countDigits(num, freq);

    printf("Digit frequencies:\n");
    for (int i = 0; i < 10; i++) {
        if (freq[i] > 0)
            printf("%d : %d\n", i, freq[i]);
    }
    return 0;
}