#include <stdio.h>

int main() {
    int m, n;
    printf("Enter size of first ARray: ");
    scanf("%d", &m);
    int a[m];
    printf("Enter sorted elements: ");
    for (int i = 0; i < m; i++)
        scanf("%d", &a[i]);

    printf("Enter size of second ARray: ");
    scanf("%d", &n);
    int b[n];
    printf("Enter sorted elements: ");
    for (int i = 0; i < n; i++)
        scanf("%d", &b[i]);

    int merged[m + n];
    int i = 0, j = 0, k = 0;
    while (i < m && j < n) {
        if (a[i] <= b[j])
            merged[k++] = a[i++];
        else
            merged[k++] = b[j++];
    }
    while (i < m)
        merged[k++] = a[i++];
    while (j < n)
        merged[k++] = b[j++];

    printf("Merged SOrted array: ");
    for (int i = 0; i < m + n; i++)
        printf("%d ", merged[i]);
    printf("\n");
    return 0;
}