#include <stdio.h>

int main() {
    int n;
    printf("Enter size of array: ");
    scanf("%d", &n);
    int arr[n];
    printf("Enter %d elements: ", n);
    for (int i = 0; i < n; i++)
        scanf("%d", &arr[i]);

    int unique[n], uniqueCount = 0;
    for (int i = 0; i < n; i++) {
        int duplicate = 0;
        for (int j = 0; j < uniqueCount; j++) {
            if (arr[i] == unique[j]) {
                duplicate = 1;
                break;
            }
        }
        if (!duplicate)
            unique[uniqueCount++] = arr[i];
    }

    printf("Array after removing duplicates: ");
    for (int i = 0; i < uniqueCount; i++)
        printf("%d ", unique[i]);
    printf("\n");
    return 0;
}