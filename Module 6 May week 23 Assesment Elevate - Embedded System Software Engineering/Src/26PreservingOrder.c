#include <stdio.h>

int main() {
    int n;
    printf("Enter size: ");
    scanf("%d", &n);
    int arr[n];
    printf("Enter elements: ");
    for (int i = 0; i < n; i++)
        scanf("%d", &arr[i]);

    int temp[n];
    int index = 0;
    // copy negatives first
    for (int i = 0; i < n; i++)
        if (arr[i] < 0)
            temp[index++] = arr[i];
    // then positives and zero
    for (int i = 0; i < n; i++)
        if (arr[i] >= 0)
            temp[index++] = arr[i];
    // copy back
    for (int i = 0; i < n; i++)
        arr[i] = temp[i];

    printf("Rearranged array: ");
    for (int i = 0; i < n; i++)
        printf("%d ", arr[i]);
    printf("\n");
    return 0;
}