/*You are given an 8-bit register represented as an unsigned char. Write a function to: 
●  Set the 3rd bit (bit index 2).  set bit2
●  Clear the 6th bit (bit index 5). clear bit 5
●  Toggle the 1st bit (bit index 0).toggle bit  Return the modified register value.  function prototype:
unsigned char modify_register(unsigned char REG);
Note: Use bitwise operators only. Avoid loops or conditionals*/
unsigned char modify_register(unsigned char reg) {
    reg |= (1 << 2);   // Set bit 2
    reg &= ~(1 << 5);  // Clear bit 5
    reg ^= (1 << 0);   // Toggle bit 0
    return reg;
}
#include <stdio.h>

#include <stdio.h>

int main() {
    unsigned char reg = 0b10000101; //saved for bit wize
    unsigned char result = modify_register(reg);// passed value 

    printf("Modified register: 0x%02X at address: %p\n", result, (void*)&result); //Modifyed register in address location
    return 0;
}
//2148 ending 