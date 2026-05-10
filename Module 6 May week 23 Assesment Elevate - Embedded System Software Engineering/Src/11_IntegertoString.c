#include <stdio.h>

int main() {
    int num;
    char str[100];
    printf("Enter an integer: ");
    scanf("%d", &num);

    sprintf(str, "%d", num);  /*without sprintf), a loop extracting 
    digits would be used standard library converts int to string*/
    
    printf("String: %s\n", str);
    return 0;
}
