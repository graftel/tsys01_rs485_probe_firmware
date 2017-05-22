
/*
  Quick and dirty functions that make serial communications work.

  Note that receiveByte() blocks -- it sits and waits _forever_ for
   a byte to come in.  If you're doing anything that's more interesting,
   you'll want to implement this with interrupts.

   initUSART requires BAUDRATE to be defined in order to calculate
     the bit-rate multiplier.  9600 is a reasonable default.

  May not work with some of the older chips:
    Tiny2313, Mega8, Mega16, Mega32 have different pin macros
    If you're using these chips, see (e.g.) iom8.h for how it's done.
    These old chips don't specify UDR0 vs UDR1.
    Correspondingly, the macros will just be defined as UDR.
*/

#include <avr/io.h>
#include "USART.h"
#include <util/setbaud.h>
#include <util/delay.h>

void initUSART(void) {                                /* requires BAUD */
  UBRRH = UBRRH_VALUE;                        /* defined in setbaud.h */
  UBRRL = UBRRL_VALUE;
#if USE_2X
  UCSRA |= (1 << U2X);
#else
  UCSRA &= ~(1 << U2X);
#endif
                                  /* Enable USART transmitter/receiver */
  UCSRB = (1 << TXEN) | (1 << RXEN);
  UCSRC = (1 << UCSZ1) | (1 << UCSZ0);   /* 8 data bits, 1 stop bit */
  
  DDRD |= (1 << DDD3);
  PORTD &= ~ (1 << DDD3);
  loop_until_bit_is_clear(PORTD, DDD3);
}

void init_RS485()
{
  PORTD |= (1 << DDD3);
  loop_until_bit_is_set(PORTD, DDD3);
}

void end_RS485()
{
  _delay_us(2000);
  PORTD &= ~ (1 << DDD3);
  _delay_us(1000);
}

void transmitByte(uint8_t data) {
                                     /* Wait for empty transmit buffer */
  loop_until_bit_is_set(UCSRA, UDRE);
  UDR = data;                                            /* send data */
}

uint8_t receiveByte(void) {
  loop_until_bit_is_set(UCSRA, RXC);       /* Wait for incoming data */
  return UDR;                                /* return register value */
}


                       /* Here are a bunch of useful printing commands */

void printString(const char myString[]) {
  init_RS485();
  uint8_t i = 0;
  while (myString[i]) {
    transmitByte(myString[i]);
    i++;
  }
  end_RS485();
}

void printRS485String(const char myString[]) {
  uint8_t i = 0;
  init_RS485();
  while (myString[i]) {
    transmitByte(myString[i]);
    i++;
  }
  end_RS485();

  
}

void printRS485Bytes(uint8_t myBytes[], uint8_t len, int check_sum_enable)
{
	uint8_t i = 0;
	int checksum = 0;
	char tmp[2];
	
	init_RS485();
	
	if (check_sum_enable == 1)
	{
		for(i = 0; i < len; i++)
		{
			transmitByte(myBytes[i]);
			checksum += myBytes[i];
		}
		checksum = checksum & 0xff;
		GetHexString((uint8_t)checksum, tmp);
		transmitByte(tmp[0]);
		transmitByte(tmp[1]);
	}
	else{
		for(i = 0; i < len; i++)
		{
			transmitByte(myBytes[i]);
		}
	}

	transmitByte(0x0d);
	
	
	
	end_RS485();
	
}

void readString(char myString[], uint8_t maxLength) {
  char response;
  uint8_t i;
  i = 0;
  while (i < (maxLength - 1)) {                   /* prevent over-runs */
    response = receiveByte();
    transmitByte(response);                                    /* echo */
    if (response == '\r') {                     /* enter marks the end */
      break;
    }
    else {
      myString[i] = response;                       /* add in a letter */
      i++;
    }
  }
  myString[i] = 0;                          /* terminal NULL character */
}

void printByte(uint8_t byte) {
              /* Converts a byte to a string of decimal text, sends it */
  transmitByte('0' + (byte / 100));                        /* Hundreds */
  transmitByte('0' + ((byte / 10) % 10));                      /* Tens */
  transmitByte('0' + (byte % 10));                             /* Ones */
}

void printWord(uint16_t word) {
  transmitByte('0' + (word / 10000));                 /* Ten-thousands */
  transmitByte('0' + ((word / 1000) % 10));               /* Thousands */
  transmitByte('0' + ((word / 100) % 10));                 /* Hundreds */
  transmitByte('0' + ((word / 10) % 10));                      /* Tens */
  transmitByte('0' + (word % 10));                             /* Ones */
}

void printBinaryByte(uint8_t byte) {
                       /* Prints out a byte as a series of 1's and 0's */
  uint8_t bit;
  for (bit = 7; bit < 255; bit--) {
    if (bit_is_set(byte, bit))
      transmitByte('1');
    else
      transmitByte('0');
  }
}

char nibbleToHexCharacter(uint8_t nibble) {
                                   /* Converts 4 bits into hexadecimal */
  if (nibble < 10) {
    return ('0' + nibble);
  }
  else {
    return ('A' + nibble - 10);
  }
}

void printHexByte(uint8_t byte) {
                        /* Prints a byte as its hexadecimal equivalent */
  uint8_t nibble;
  nibble = (byte & 0b11110000) >> 4;
  transmitByte(nibbleToHexCharacter(nibble));
  nibble = byte & 0b00001111;
  transmitByte(nibbleToHexCharacter(nibble));
}

void GetHexString(uint8_t byte, char *out)
{
  uint8_t nibble;
  nibble = (byte & 0b11110000) >> 4;
  out[0] = nibbleToHexCharacter(nibble);
  nibble = byte & 0b00001111;
  out[1] = nibbleToHexCharacter(nibble);
}

uint8_t GetByteFromString(char *in)
{
  uint8_t out = 0x00;
  
  if (in[0] >= 0x30 && in[0] <= 0x39)
  {
      out += (in[0] - 0x30) << 4;
  }
  else if (in[0] >= 0x41 && in[0] <= 0x46)
  {
      out += (in[0] - 0x37) << 4;
  }
    
  if (in[1] >= 0x30 && in[1] <= 0x39)
  {
      out += (in[1] - 0x30);
  }
  else if (in[1] >= 0x41 && in[1] <= 0x46)
  {
      out += (in[1] - 0x37);   
	  
  }
    
    
  return out;
}


uint8_t getNumber(void) {
  // Gets a numerical 0-255 from the serial port.
  // Converts from string to number.
  char hundreds = '0';
  char tens = '0';
  char ones = '0';
  char thisChar = '0';
  do {                                                   /* shift over */
    hundreds = tens;
    tens = ones;
    ones = thisChar;
    thisChar = receiveByte();                   /* get a new character */
    transmitByte(thisChar);                                    /* echo */
  } while (thisChar != '\r');                     /* until type return */
  return (100 * (hundreds - '0') + 10 * (tens - '0') + ones - '0');
}
