#include <stdio.h>
#include <ctype.h>

int main() {
    char str[200];
    int letters[26] = {0};
    printf("Enter a sentence: ");
    fgets(str, sizeof(str), stdin);  // reads spaces

    for (int i = 0; str[i] != '\0'; i++) {
        if (isalpha(str[i])) {
            char ch = tolower(str[i]);
            letters[ch - 'a'] = 1;
        }
    }

    int isPangram = 1;
    for (int i = 0; i < 26; i++) {
        if (!letters[i]) {
            isPangram = 0;
            break;
        }
    }

    printf("The string %s a pangram.\n", isPangram ? "is" : "is not");
    return 0;
}