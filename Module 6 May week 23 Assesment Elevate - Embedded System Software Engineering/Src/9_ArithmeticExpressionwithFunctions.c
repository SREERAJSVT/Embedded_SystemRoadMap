#include <stdio.h>

float add(float x, float y) { return x + y; }
float subtract(float x, float y) { return x - y; }
float multiply(float x, float y) { return x * y; }
float divide(float x, float y) {
    if (y == 0) {
        printf("Error: Division by zero!\n");
        return 0;
    }
    return x / y;
}

float selector(float x, float y, char op) {
    switch (op) {
        case '+': return add(x, y);
        case '-': return subtract(x, y);
        case '*': return multiply(x, y);
        case '/': return divide(x, y);
        default:
            printf("Invalid operator.\n");
            return 0;
    }
}

int main() {
    float a, b;
    char op;
    printf("Enter two numbers and an operator (e.g., 3.5 2.1 +): ");
    scanf("%f %f %c", &a, &b, &op);
    float result = selector(a, b, op);
    printf("Result = %.2f\n", result);
    return 0;
}