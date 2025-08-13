#include <stdio.h>
#include <stdlib.h> 
struct Employee { char name[50]; int id; float salary; };
int main()
{
    int j=0,ch;
   struct  Employee emp[6];

for (int j = 0; j <6; j++) 
    {
    printf("Enter employee name for employee %d:\n", j+1);
    scanf(" %49[^\n]", emp[j].name); // Read up to 49 chars, leave space for null terminator
    printf("Enter employee ID for employee %d:\n", j+1);
    scanf("%d", &emp[j].id); // Pass address of id
    printf("Enter employee salary for employee %d:\n", j+1);
    scanf("%f", &emp[j].salary);
    while ((ch = getchar()) != '\n' && ch != EOF); // Clear the input buffer
}
 int i= 0;
    for (i=0; i < 6; i++)
    {
    printf("Employee %d Name: %s\nID: %d\nSalary: %.2f\n", i+1, emp[i].name, emp[i].id, emp[i].salary);
    }
      return 0; 
}//-2025 22:42Hr create a structure, read employee details, and print them. Assignment done Structure.c REF#*/
/*Entry B1 Code Completed 11-08-2025 22:42Hr create a structure, read employee details, and print them. Assignment done Structure.c REF#*/