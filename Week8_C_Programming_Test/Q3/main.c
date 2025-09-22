/*Write a C program to print a pyramid of stars for a given number n. */
#include <stdio.h>
int main ()
{
    int n ,i,j;
    printf("enter the number of ROWS");
    scanf("%d",&n);
    for (i=1;i<=n;i++)
    {
        for(j=1;j<=n-i;j++)
        {
            printf(" ");// Space for Pyramid;2031PM

        }
        for (j=1;j<=2*i-1;j++)
        {
            printf("*");
        }
        printf("\n");
    }
}