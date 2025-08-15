#include <stdio.h>
#include <stdlib.h> 
struct Book {
    char title[50];
    char author[30];
    float price;
};
int main() {
    struct Book books[3];
    float maxPrice = 0.0;
    int maxIndex = 0;

    for (int i = 0; i < 3; i++) {
        printf("Enter title for book %d:\n", i + 1);
        scanf(" %49[^\n]", books[i].title); // Read up to 49 chars, leave space for null terminator
        printf("Enter author for book %d:\n", i + 1);
        scanf(" %29[^\n]", books[i].author); // Read up to 29 chars, leave space for null terminator
        printf("Enter price for book %d:\n", i + 1);
        scanf("%f", &books[i].price);

        if (books[i].price > maxPrice) {
            maxPrice = books[i].price;
            maxIndex = i;
        }
    }

    printf("\nMost Expensive Book Details:\n");
    printf("Title: %s\n", books[maxIndex].title);
    printf("Author: %s\n", books[maxIndex].author);
    printf("Price: %.2f\n", books[maxIndex].price);

    return 0;
} /*Create a C program that defines a structure named Book with the following members:
	•	title (string of up to 50 characters)
	•	author (string of up to 30 characters)
	•	price (float)
Write a program to:
Read details for 3 books from the user.
Print the details of the most expensive book.*/