/*Entry B1 Code Completed 11-08-2025 22:42Hr create a arry, find sum of number in array , find odd number,count of odd. in sperate funtion Assignment done Array_2Funtion .c REF#*/
#include <stdio.h>
#include <stdlib.h>
void sum(int arr[], int n) {
    int total = 0;
    for(int i = 0; i < n; i++) {
        total += arr[i];
    }
    printf("Sum of the array elements: %d\n", total);
}
void find_odd(int arr[], int i)
{int odd_count=0;
    printf("Odd numbers in the array: ");
    for(int j = 0; j < i; j++) {
        if(arr[j] % 2 != 0) {
            printf("%d ", arr[j]);
            odd_count++;
        }
    }
    printf("\nTotal odd numbers: %d\n", odd_count);
}
void readnumbers(int arr[], int n) {
    printf("Enter %d integers:\n", n);
    for(int i = 0; i < n; i++) {
        scanf("%d", &arr[i]);
    }
}
int main()
{
int arr[10];
    int n = 10;
    readnumbers(arr, n);
    sum(arr, n);
    find_odd(arr, n);
    return 0;
}