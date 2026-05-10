//Assesment 27: Complex Number Addition

#include <stdio.h>
typedef struct{
    float real;
    float imag;
} Complex;


void displayComplex(Complex c) {
    printf("%.2f + %.2fi\n", c.real, c.imag);
}

Complex addComplex(Complex c1, Complex c2) {
    Complex result;
    result.real = c1.real + c2.real;
    result.imag = c1.imag + c2.imag;
    return result;
}



int main() {
    Complex a, b, sum;
    printf("Enter real and imaginary part of first complex number: ");
    scanf("%f %f", &a.real, &a.imag);
    printf("Enter real and imaginary part of second complex number: ");
    scanf("%f %f", &b.real, &b.imag);

    sum = addComplex(a, b);
    printf("Sum = ");
    displayComplex(sum);
    return 0;
}