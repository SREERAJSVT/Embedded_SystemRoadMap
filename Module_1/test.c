#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
// This program is a simple calculator using ncurses for terminal UI.
// It supports basic arithmetic operations: addition, subtraction, multiplication, and division.
// Function prototypes for arithmetic operations
int sum(int a, int b){ return a+b; }
int sub(int a, int b){ return a-b; }
int mul(int a, int b){ return a*b; }
int div(int a, int b){ return a/b; }

int main() {
    int a, b, result;
    char op;

    initscr();            // Start ncurses mode
    cbreak();             // Disable line buffering
    noecho();             // Don't echo user input

    printw("Welcome to the Calculator Program\n");
    printw("Enter first number: ");
    scanw("%d", &a);
    printw("Enter second number: ");
    scanw("%d", &b);
    printw("Choose operation (+, -, x, D): ");
    op = getch();
    printw("%c\n", op);

    switch(op) {
        case '+': result = sum(a, b); printw("Sum: %d\n", result); break;
        case '-': result = sub(a, b); printw("Difference: %d\n", result); break;
        case 'x': case 'X': result = mul(a, b); printw("Product: %d\n", result); break;
        case 'D': case 'd': result = div(a, b); printw("Quotient: %d\n", result); break;
        default: printw("Invalid operator.\n");
    }

    printw("Press any key to exit...");
    getch();
    endwin();             // End ncurses mode
    return 0;
}