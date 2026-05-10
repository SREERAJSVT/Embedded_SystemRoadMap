#include <stdio.h>

int main() {
    char str[200], result[200];
    printf("Enter a sentence with multiple spaces: ");
    fgets(str, sizeof(str), stdin);

    int j = 0;
    int spaceSeen = 0;
    for (int i = 0; str[i] != '\0'; i++) {
        if (str[i] == ' ') {
            if (!spaceSeen) {
                result[j++] = ' ';
                spaceSeen = 1;
            }
        } else {
            result[j++] = str[i];
            spaceSeen = 0;
        }
    }
    result[j] = '\0';

    printf("Cleaned string: %s\n", result);
    return 0;
}