//Function code used to find the avg of three numbers
#include <stdio.h>
void avg(int,int,int);//Funnction_Decleration

int main()//Function_Call
{
    avg(10,20,40);    

    return 0;
}

void avg(int a,int b,int c)//Funnction_Definition
{
    int avg;
    avg=(a+b+c)/3;
    printf("Average of three numbers:%d",avg);
}