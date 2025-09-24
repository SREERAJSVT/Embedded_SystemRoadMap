#include <avr/io.h>

int main(void) {
	    DDRD |= (1 << DDD6);
		TCCR0A |= (1 << WGM01) | (1 << WGM00);
		TCCR0A |= (1 << COM0A1);
		OCR0A =128;//Duty Cycle
		TCCR0B |= (1 << CS01) | (1 << CS00);
while (1) {
}
}

//home work:write a program fade; loop used.