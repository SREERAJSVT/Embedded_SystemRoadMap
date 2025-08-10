#include <stdio.h>
#include <string.h>
int sum(int a, int b){
    int sum=(a+b);
    return sum;
}
int sub(int a, int b){
    int sub=(a-b);
    return sub;
}
int mul(int a, int b){
    int mul=(a*b);
    return mul;
}
int div(int a, int b){
    int div=(a/b);
    return div;
}

int main()
{
    int a,b,sum_res,sub_res,mul_res,div_res;
    char op;
    printf("Welcome to the Calculator Program\n");
    printf("This program performs basic arithmetic operations.\n");
    printf("Enter the first number : \n");
    scanf("%d", &a);
    printf("Enter the second number : \n");
    scanf("%d", &b);
    printf("Enter the operation to be executed : \n+)Addition \n-)Subtraction \nx)Multiplication \nD)Division \n");
    printf("Enter the operator: ");
    getchar();
    scanf("%c", &op);
    
if(op=='+')
{
    sum_res=sum(a,b);
    printf("Sum of %d \n",sum_res);

}
else if (op=='-'){
    sub_res=sub(a,b);
    printf("Diff of %d \n",sub_res);
}
else if(op=='x'|| op=='X')
{
    mul_res=mul(a,b);
    printf("Product of %d \n", mul_res);
    
}
else if(op=='D' || op=='d') 
{
    div_res=div(a,b);
    printf("Quotient of %d \n", div_res);
}
else {
    printf("Invalid operator selected.\n", op);
}
    printf("Thank you for using the Calculator Program!\n");



return 0;
}
