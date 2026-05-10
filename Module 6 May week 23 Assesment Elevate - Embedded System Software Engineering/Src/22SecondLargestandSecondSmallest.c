#include <stdio.h>
#include <limits.h>

int main() {
    int n;
    printf("Enter size: ");
    scanf("%d", &n);
    int arr[n];
    printf("Enter elements: ");
    for (int i = 0; i < n; i++)
        scanf("%d", &arr[i]);

    int firstL = INT_MIN, secondL = INT_MIN;
    int firstS = INT_MAX, secondS = INT_MAX;

    for (int i = 0; i < n; i++) {
        // largest and second largest
        if (arr[i] > firstL) {
            secondL = firstL;
            firstL = arr[i];
        } else if (arr[i] > secondL && arr[i] != firstL)
            secondL = arr[i];
        // smallest and second smallest
        if (arr[i] < firstS) {
            secondS = firstS;
            firstS = arr[i];
        } else if (arr[i] < secondS && arr[i] != firstS)
            secondS = arr[i];
    }

    if (secondL == INT_MIN)
        printf("No second largest element.\n");
    else
        printf("Second largest: %d\n", secondL);

    if (secondS == INT_MAX)
        printf("No second smallest element.\n");
    else
        printf("Second smallest: %d\n", secondS);

    return 0;
}