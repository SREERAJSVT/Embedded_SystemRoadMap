#include <stdio.h>
#include <ctype.h>

int main() {
    char str[200];
    printf("Enter a string: ");
    fgets(str, sizeof(str), stdin);

    for (int i = 0; str[i] != '\0'; i++) {
        if (isupper(str[i]))
            str[i] = tolower(str[i]);
    }

    printf("Lowercase string: %s\n", str);
    return 0;
}