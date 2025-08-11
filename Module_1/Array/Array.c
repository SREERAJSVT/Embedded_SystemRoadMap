#include <stdio.h>
#include <stdlib.h>

// Function to print the first n elements of an array
void fun(int a[], int n)
{
    for (int i = 0; i < n; i++)
    {
        printf("The value of a function recived [%d]  cycle %d\n", i, a[i]);
    }
}

int main()
{
    int value[10];
    int j = 0;

    for (int pos = 0; pos < 10; pos++)
    {
        value[pos] = pos;
        if (value[pos] % 2 == 0)
        {
            printf("The value of possition [%d] is %d\n", pos, value[pos]);
            j = j + 1;
            fun(value,j);
        }
    }

    return 0;
}

/*Entry B1 Code Completed 11-08

/*Entry B1 Code Completed 11-08-2025 22:42Hr create.*/