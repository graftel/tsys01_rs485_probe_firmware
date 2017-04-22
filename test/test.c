#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "USART.h"
#define MAX_BUFFER_SIZE 32
#include <stdlib.h>

int
main()
{
	uint8_t rev = 0x00;
	
	initUSART();
	DDRD |= (1 << DDD3);
	PORTD &= ~ (1 << DDD3);
	loop_until_bit_is_clear(PORTD, DDD3);
	
	int n = 0;
	
	char recv[MAX_BUFFER_SIZE];
	char buffer[20];
	float f = 3.14;
	dtostrf(f, 6, 4, buffer);
	
	while(1){
		
		rev = receiveByte();
		
		recv[n] = rev;
		n++;
		if (n >= MAX_BUFFER_SIZE)
		{
			n = 0;
			memset(recv,0,sizeof(recv));
		}
		else if (rev == '\r')
		{
			PORTD |= (1 << DDD3);
			loop_until_bit_is_set(PORTD, DDD3);
			printString(buffer);
			
			_delay_us(2000);
			PORTD &= ~ (1 << DDD3);
		//	_delay_us(1000);
			n = 0;
			memset(recv,0,sizeof(recv));
			
		}
	}
}

