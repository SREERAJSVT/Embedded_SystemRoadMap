#include <stdio.h>

void rotateLeft(int arr[], int n, int k) {
    k = k % n;
    int temp[k];
    for (int i = 0; i < k; i++)
        temp[i] = arr[i];
    for (int i = 0; i < n - k; i++)
        arr[i] = arr[i + k];
    for (int i = 0; i < k; i++)
        arr[n - k + i] = temp[i];
}

void rotateRight(int arr[], int n, int k) {
    k = k % n;
    int temp[k];
    for (int i = 0; i < k; i++)
        temp[i] = arr[n - k + i];
    for (int i = n - 1; i >= k; i--)
        arr[i] = arr[i - k];
    for (int i = 0; i < k; i++)
        arr[i] = temp[i];
}

int main() {
    int n, k;
    char dir;
    printf("Enter array size: ");
    scanf("%d", &n);
    int arr[n];
    printf("Enter elements: ");
    for (int i = 0; i < n; i++)
        scanf("%d", &arr[i]);
    printf("Enter positions to rotate and direction (l/r): ");
    scanf("%d %c", &k, &dir);

    if (dir == 'l' || dir == 'L')
        rotateLeft(arr, n, k);
    else
        rotateRight(arr, n, k);

    printf("Rotated array: ");
    for (int i = 0; i < n; i++)
        printf("%d ", arr[i]);
    printf("\n");
    return 0;
}