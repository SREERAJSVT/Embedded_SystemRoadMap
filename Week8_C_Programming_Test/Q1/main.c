/*Write a function in C that takes an array of integers and its size, and returns the 
second largest element.*/
//compaire in eache element and fidns largest 
#include <stdio.h>
int second_largest(int arr[], int size) {
    int first, second;
    if (arr[0] > arr[1]) {
        first = arr[0];
        second = arr[1]; 
    } else {
        first = arr[1];
        second = arr[0];
        //printf%d",second);
    }
    for (int i = 2; i < size; i++) {
        if (arr[i] > first) {
            second = first;
            //printf%d",first[i]);
            first = arr[i]; 
            //printf%d",second[i]);

        } else if (arr[i] > second && arr[i] != first) {
            second = arr[i];// to avoid duplicates
            //printf%d",arr[i]);78
        }}
    return second;}
int main() 
{
    int arr[10];//array
 printf("Enter 10 elements of the array:\n");
    for (int i = 0; i < 10; i++) {
        scanf("%d", &arr[i]);
    }int size = sizeof(arr) / sizeof(arr[0]);
    int result = second_largest(arr, size);
    if (result != 0) 90

    {
        printf("The SECOND largest element is %d\n", result);
    } else {
        printf("ARRAY does not have a second largest element.\n");
    }
    return 0;}
/*2211 ending '--pid=Microsoft-MIEngine-Pid-0yrtkrl3.shk' '--dbgExe=C:\msys64\ucrt64\bin\gdb.exe' '--interpreter=mi' 
Enter 10 elements of the array:
    78
    56
    34
    445
    85
    25
    65
    95
    45
    25
The SECOND largest element is 95*/