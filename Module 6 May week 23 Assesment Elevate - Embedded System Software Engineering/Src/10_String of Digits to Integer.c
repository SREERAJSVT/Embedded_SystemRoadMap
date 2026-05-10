#include <stdio.h>

int main() {
    char str[100];
    int num = 0;
    printf("Enter a number as a string: ");
    scanf("%s", str);

    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] >= '0' && str[i] <= '9')
            num = num * 10 + (str[i] - '0');
        else {
            printf("Invalid digit '%c'\n", str[i]);
            return 1;
        }
    }
    printf("Integer value: %d\n", num);
    return 0;
}