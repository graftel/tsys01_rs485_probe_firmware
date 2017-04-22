#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "USART.h"
#include "usi_i2c_master.h"
#define MAX_BUFFER_SIZE 32

#define TSYS01Address 0x77  //address left shifted by arduino as required to match datasheet
#define TSYS01Reset 0x1E //initiates reset
#define TSYS01StartReg 0x48 //commands sensor to begin measurement / output calculation
#define TSYS01TempReg 0x00 //requests most recent output from measurement

#define USI_SEND         0              // indicates sending to TWI
#define USI_RCVE         1              // indicates receiving from TWI
#define USI_BUF_SIZE    16              // bytes in message buffer

#define TWI_READ_BIT  0       // Bit position for R/W bit in "address byte".
#define TWI_ADR_BITS  1       // Bit position for LSB of the slave address bits in the init byte.
#define TWI_NACK_BIT  0       // Bit position for (N)ACK bit.

#define DEFAULT_ADDR "100000"
#define DEFAULT_CAL_A 0.0f
#define DEFAULT_CAL_B 1.0f
#define DEFAULT_CAL_C 0.0f

#define EEPROM_INIT_STATUS_BYTE 0x00
#define EEPROM_ADDR_START_BYTE  0x01
#define EEPROM_CAL_START_BYTE   0x07

uint8_t coefficent_MSB[5];
uint8_t coefficent_LSB[5];
char addr[6];
uint8_t data_buf[16];
uint8_t write_enabled = 0;
void init_i2c()
{
	DDR_USI  |= (1 << PORT_USI_SDA) | (1 << PORT_USI_SCL);
	PORT_USI |= (1 << PORT_USI_SCL);
	PORT_USI |= (1 << PORT_USI_SDA);
	USIDR = 0xFF;
	USICR = (0 << USISIE) | (0 << USIOIE) | (1 << USIWM1) | (0 << USIWM0) | (1 << USICS1) | (0 << USICS0) | (1 << USICLK) | (0 << USITC);
	USISR = (1 << USISIF) | (1 << USIOIF) | (1 << USIPF)  | (1 << USIDC)  | (0x00 << USICNT0);
}

void init_tsys01()
{
	uint8_t result_code;
	
	data_buf[0] = (TSYS01Address << TWI_ADR_BITS) | USI_SEND;
	data_buf[1] = TSYS01Reset;
	
	result_code = USI_I2C_Master_Start_Transmission(data_buf,2);
	_delay_ms(10);
	

	// float retVal =
    // (-2)* (float)coefficent[0] * (float)pow(10,-21) * pow(rawAdc,4) +
    // 4 * (float)coefficent[1] * (float)pow(10,-16) * pow(rawAdc,3) +
    // (-2) * (float)coefficent[2] * (float)pow(10,-11) * pow(rawAdc,2) +
    // 1 * (float)coefficent[3] * (float)pow(10,-6) * rawAdc +
    // (-1.5) * (float)coefficent[4] * (float)pow(10,-2);
	
	//char buffer[20];
	//f = pow(f,2);
	//dtostrf(f, 6, 4, buffer);
	//memcpy(test, &retVal, 4);
	// dtostrf(retVal,6,4,test);
	// printRS485String(buffer);

}

void Read_CalData()
{
	uint8_t result_code;
	uint8_t n = 0;
	for(n = 0; n < 5; n ++)
	{
		data_buf[0] = (TSYS01Address << TWI_ADR_BITS) | USI_SEND;
		data_buf[1] = 0xA2 + n * 2;
		result_code = USI_I2C_Master_Start_Transmission(data_buf,2);
		
		data_buf[0] = (TSYS01Address << TWI_ADR_BITS) | USI_RCVE;
		
		result_code = USI_I2C_Master_Start_Transmission(data_buf,3);
		
		coefficent_MSB[n] = data_buf[1];
		coefficent_LSB[n] = data_buf[2];
	}
}

void Read_Temp(uint8_t *temp_data)
{
	uint8_t result_code;
	
	data_buf[0] = (TSYS01Address << TWI_ADR_BITS) | USI_SEND;
	data_buf[1] = TSYS01StartReg;
	result_code = USI_I2C_Master_Start_Transmission(data_buf,2);
	_delay_us(10000);
	data_buf[0] = (TSYS01Address << TWI_ADR_BITS) | USI_SEND;
	data_buf[1] = TSYS01TempReg;
	result_code = USI_I2C_Master_Start_Transmission(data_buf,2);
	data_buf[0] = (TSYS01Address << TWI_ADR_BITS) | USI_RCVE;	
	result_code = USI_I2C_Master_Start_Transmission(data_buf,4);
	
	temp_data[0] = data_buf[1];
	temp_data[1] = data_buf[2];
	temp_data[2] = data_buf[3];
}

unsigned char EEPROM_read(unsigned int ucAddress)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEAR = ucAddress;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	return EEDR;
}


void EEPROM_write(unsigned int ucAddress, unsigned char ucData)
{
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set Programming mode */
	EECR = (0<<EEPM1)|(0<<EEPM0);
	/* Set up address and data registers */
	EEAR = ucAddress;
	EEDR = ucData;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);
}

uint8_t Read_init()
{
	return EEPROM_read(EEPROM_INIT_STATUS_BYTE);
}

void init_setup()
{
	int i;
	uint8_t tmp[sizeof(float)];
	
	float cal_a,cal_b,cal_c;
	cal_a = DEFAULT_CAL_A;
	cal_b = DEFAULT_CAL_B;
	cal_c = DEFAULT_CAL_C;
	
	for(i = 0; i < 6; i++)
	{
		addr[i] = DEFAULT_ADDR[i];
		EEPROM_write(EEPROM_ADDR_START_BYTE + i, DEFAULT_ADDR[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_a,sizeof(float));
		EEPROM_write(EEPROM_CAL_START_BYTE + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_b,sizeof(float));
		EEPROM_write(EEPROM_CAL_START_BYTE + sizeof(float) + i, tmp[i]);
	}
	
	for(i = 0; i < sizeof(float); i++)
	{
		memset(tmp,0,sizeof(float));
		memcpy(tmp,&cal_c,sizeof(float));
		EEPROM_write(EEPROM_CAL_START_BYTE + 2 * sizeof(float) + i, tmp[i]);
	}

}

void init_all()
{
	int i;
	for(i = 0; i < 6; i++)
	{
		addr[i] = EEPROM_read(EEPROM_ADDR_START_BYTE + i);
	}
}

int
main()
{
	
	
	initUSART();	
	
	init_i2c();
	
	if (Read_init() != 0x01)
	{
		EEPROM_write(EEPROM_INIT_STATUS_BYTE, 0x01);
		init_setup();
	}

	init_all();
	
	uint8_t rev = 0x00;
	int n = 0;
	char tmp[3];
	char tmp_data[3];
	char recv[MAX_BUFFER_SIZE];
	
	init_tsys01();
	
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
			if (recv[0] == '*' && 
				recv[1] == addr[0] &&
				recv[2] == addr[1] &&
				recv[3] == addr[2] &&
				recv[4] == addr[3] &&
				recv[5] == addr[4] &&
				recv[6] == addr[5] )	// verify address
				{
					recv[0] = '#';
					if (recv[7] == 'R' && 
					    recv[8] == 'D' &&
						n == 10)
					{
						write_enabled = 0;
						recv[9] = '=';
						Read_Temp(tmp_data);
						
						uint8_t m = 0;
						
						for(m = 0; m < 3; m++)
						{
							memset(tmp,0,sizeof(tmp));
							GetHexString(tmp_data[m],tmp);
							recv[10 + m * 2] = tmp[0];
							recv[11 + m * 2] = tmp[1];
						}					
						recv[16] = 0x0d;
						n = 17;
						printRS485Bytes(recv, n);
					}
					
					if (recv[7] == 'R' && 
					    recv[8] == 'C' &&
						(recv[9] == 'A' || recv[9] == 'B') || recv[9] == 'C' &&
						n == 11)
					{
						write_enabled = 0;
						recv[10] = '=';
						
						
						uint8_t m = 0;
						
						uint8_t add_bytes;
						if (recv[9] == 'A')
						{
							add_bytes = 0x00;
						}
						else if (recv[9] == 'B'){
							add_bytes = 0x04;
						}
						else if (recv[9] == 'C'){
							add_bytes = 0x08;
						}
						
						for(m = 0; m < 4; m++)
						{
							memset(tmp,0,sizeof(tmp));
							GetHexString(EEPROM_read(EEPROM_CAL_START_BYTE + add_bytes + m),tmp);
							recv[11 + m * 2] = tmp[0];
							recv[12 + m * 2] = tmp[1];
						}			
						recv[19] = 0x0d;
						n = 20;
						printRS485Bytes(recv, n);
					}
					
					if (recv[7] == 'S' && 
					    recv[8] == 'C' &&
						(recv[9] == 'A' || recv[9] == 'B' || recv[9] == 'C') &&
						recv[10] == '=' &&
						n == 20 && 
						write_enabled == 1)
					{
						write_enabled = 0;
						
						uint8_t m = 0;
						
						uint8_t add_bytes;
						if (recv[9] == 'A')
						{
							add_bytes = 0x00;
						}
						else if (recv[9] == 'B'){
							add_bytes = 0x04;
						}
						else if (recv[9] == 'C'){
							add_bytes = 0x08;
						}
						
						for(m = 0; m < 4; m++)
						{
							memset(tmp,0,sizeof(tmp));
							tmp[0] = recv[11 + m * 2];
							tmp[1] = recv[12 + m * 2];
							EEPROM_write(EEPROM_CAL_START_BYTE + add_bytes + m, GetByteFromString(tmp));
						}			
						printRS485Bytes(recv, n);
					}
					
					if (recv[7] == 'R' && 
					    recv[8] == 'E' &&
						n == 10)
					{
						write_enabled = 0;
						recv[9] = '=';
						Read_CalData();
						
						uint8_t m = 0;
						
						for(m = 0; m < 5; m++)
						{
							memset(tmp,0,sizeof(tmp));
							GetHexString(coefficent_MSB[m],tmp);
							recv[10 + m * 4] = tmp[0];
							recv[11 + m * 4] = tmp[1];
							memset(tmp,0,sizeof(tmp));
							GetHexString(coefficent_LSB[m],tmp);
							recv[12 + m * 4] = tmp[0];
							recv[13 + m * 4] = tmp[1];
						}					
						recv[30] = 0x0d;
						n = 31;
						printRS485Bytes(recv, n);
					}
					
					if (recv[7] == 'W' && 
					    recv[8] == 'E' &&
						n == 10)
					{
						write_enabled = 1;
						printRS485Bytes(recv, n);
					}
					
					if (recv[7] == 'S' && 
					    recv[8] == 'A' &&
						recv[9] == '=' &&
						n == 17 &&
						write_enabled == 1)
					{
						write_enabled = 0;
						uint8_t n1 = 0;
						uint8_t n2 = 1;
						for(n1 = 0; n1 < 6; n1++)
						{
							if (recv[10 + n1] > 0x39 || recv[10 + n1] < 0x30)
							{
								n2 = 0;
								break;
							}
						}
						if (n2 == 1)
						{
							
							for(n1 = 0; n1 < 6; n1++)
							{
								addr[n1] = recv[10 + n1];
								EEPROM_write(0x00 + n1, recv[10 + n1]);
							}
							printRS485Bytes(recv, n);
						}
						
						
					}
				}
			
		//	_delay_us(1000);
			n = 0;
			memset(recv,0,sizeof(recv));
			
		}
	}
}

