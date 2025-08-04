#include <stdio.h>
#include <string.h> 
int main()
{
   // std::cout << "Hello World" << std::endl;
char name[50];
char address[100];
   printf("Enter your name:\n");
   scanf("%s", name);
   printf("Enter your address:\n");
   scanf("%s", address);
   printf("You entered: %s\n", name);
   printf_s("You entered: %s\n", address);
   return 0;
}
// This is a simple C++ program that prints&read "Address" to the console.