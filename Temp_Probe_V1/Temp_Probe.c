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

#define DEFAULT_ADDR "02A020"
#define DEFAULT_CAL_A 0.0f
#define DEFAULT_CAL_B 1.0f
#define DEFAULT_CAL_C 0.0f
#define DEFAULT_SETTING "100000"

#define ADDRESS_LEN 6
#define CMD_LEN2 2
#define CMD_LEN3 2

#define ALWAYS_RESP_ADDR "02Z999"

#define READ_CMD "RD"
#define RE_CMD "RE"

#define EEPROM_INIT_STATUS_BYTE 	0x00
#define EEPROM_ADDR_START_BYTE  	0x01
#define EEPROM_CAL_START_BYTE   	0x07
#define EEPROM_SETTING_START_BYTE   0x13

#define NUM_READ_CMD 5
#define NUM_WRITE_CMD 3

enum Cmd {
	RD = 0,
	RC,
	RE,
	WE,
	RS,
	SC,
	SA,
	SS
};
const int cmd_len[] = {
	2,
	3,
	3,
	2,
	2,
	12,
	9,
	9
};

const char *cmd_list[] = {
	"RD",
	"RC",
	"RE",
	"WE",
	"RS",
	"SC",
	"SA",
	"SS"
};

uint8_t coefficent_MSB[5];
uint8_t coefficent_LSB[5];
char addr[6],setting[6];
uint8_t data_buf[16];
uint8_t write_enabled = 0;
int check_sum_enable = 0;
int delay = 0;
int data_size = 0;
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
	
	for(i = 0; i < 6; i++)
	{
		setting[i] = DEFAULT_SETTING[i];
		EEPROM_write(EEPROM_SETTING_START_BYTE + i, DEFAULT_SETTING[i]);
	}
	check_sum_enable = DEFAULT_SETTING[0] - 0x30;
	delay = DEFAULT_SETTING[1] - 0x30;
	
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
		setting[i] = EEPROM_read(EEPROM_SETTING_START_BYTE + i);
	}
	check_sum_enable = setting[0] - 0x30;
	delay = setting[1] - 0x30;
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
	uint8_t n = 0;
	uint8_t v = 0;
	uint8_t m = 0;
	uint8_t cmd_count[5];
	char tmp[3];
	char tmp_data[3];
	char recv[MAX_BUFFER_SIZE];
	int send_enable = 0;
	int send_size = 0;
	init_tsys01();
	
	while(1){
		
		rev = receiveByte();
		
		if (rev == '*' && v == 0)
		{
			v = 1;
			memset(recv,0,sizeof(recv));
			recv[v - 1] = rev;
		}
		else if (v > 0)
		{
			recv[v] = rev;
			v++;
				
			if (v >= MAX_BUFFER_SIZE)
			{
				v = 0;
				continue;
			}
			
			if (rev == 0x0d)
			{
				if (memcmp(recv + 1, ALWAYS_RESP_ADDR,ADDRESS_LEN) == 0
				 || memcmp(recv + 1, addr, ADDRESS_LEN) == 0) // verify address
				{
					recv[0] = '#';
					if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RD], strlen(cmd_list[RD])) == 0
					 && v == ADDRESS_LEN + 2 + cmd_len[RD])
					{	// v = 10
						write_enabled = 0;
						recv[v - 1] = '=';
						Read_Temp(tmp_data);
						for(m = 0; m < 3; m++)
						{
							memset(tmp,0,sizeof(tmp));
							GetHexString(tmp_data[m],tmp);
							recv[v + m * 2] = tmp[0];
							recv[v + 1 + m * 2] = tmp[1];
						}
						send_size = 6 + v;
						send_enable = 1;
					}
					else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RE], strlen(cmd_list[RE])) == 0
					 && v == ADDRESS_LEN + 2 + cmd_len[RE])
					{
						if (recv[v - 2] >= 0x30 && recv[v - 2] <= 0x34)
						{
							write_enabled = 0;
							recv[v - 1] = '=';
							Read_CalData();
							data_size = 4;
							m = recv[v - 2] - 0x30;
							
							memset(tmp,0,sizeof(tmp));
							GetHexString(coefficent_MSB[m],tmp);
							recv[v] = tmp[0];
							recv[v + 1] = tmp[1];
							memset(tmp,0,sizeof(tmp));
							GetHexString(coefficent_LSB[m],tmp);
							recv[v + 2] = tmp[0];
							recv[v + 3] = tmp[1];
												
							send_enable = 1;
							send_size = v + data_size;
						}							
					}
					else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[WE], strlen(cmd_list[WE])) == 0
					 && v == ADDRESS_LEN + 2 + cmd_len[WE])
					{
						write_enabled = 1;
						send_enable = 1;
						send_size = v - 1;
					}
					else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RC], strlen(cmd_list[RC])) == 0
					 && v == ADDRESS_LEN + 2 + cmd_len[RC])
					{
						 if (recv[v - 2] >= 0x41 && recv[v - 2] <= 0x43)
						 {
							write_enabled = 0;
							recv[v - 1] = '=';
							data_size = 8;
							 
							uint8_t add_bytes;
							if (recv[v - 2] == 'A')
							{
								add_bytes = 0x00;
							}
							else if (recv[v - 2] == 'B'){
								add_bytes = 0x04;
							}
							else if (recv[v - 2] == 'C'){
								add_bytes = 0x08;
							}
							
							for(m = 0; m < 4; m++)
							{
								memset(tmp,0,sizeof(tmp));
								GetHexString(EEPROM_read(EEPROM_CAL_START_BYTE + add_bytes + m),tmp);
								recv[v + m * 2] = tmp[0];
								recv[v + 1 + m * 2] = tmp[1];
							}
							send_enable = 1;
							send_size = v + data_size;
						 }
					}
					else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[RS], strlen(cmd_list[RS])) == 0
					 && v == ADDRESS_LEN + 2 + cmd_len[RS])
					{
						write_enabled = 0;
						data_size = 6;
						recv[v - 1] = '=';
						uint8_t n1 = 0;

							
						for(n1 = 0; n1 < data_size; n1++)
						{
							setting[n1] = EEPROM_read(EEPROM_SETTING_START_BYTE + n1);
							recv[v + n1] = setting[n1];
							
						}
						
						send_enable = 1;
						send_size = v +	data_size;
						
					}
					else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[SC], strlen(cmd_list[SC])) == 0
					&& v == ADDRESS_LEN + 2 + cmd_len[SC]
					&& write_enabled == 1)
					{
						if (recv[9] >= 0x41 && recv[9] <= 0x43)
						{
							write_enabled = 0;
						
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
							send_enable = 1;
							send_size = v - 1; 
						}
					}
					else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[SA], strlen(cmd_list[SA])) == 0
					&& v == ADDRESS_LEN + 2 + cmd_len[SA]
					&& write_enabled == 1)
					{
						write_enabled = 0;
						uint8_t n1 = 0;
						uint8_t n2 = 1;
						for(n1 = 0; n1 < 6; n1++)
						{
							if (recv[10 + n1] < 0x30 || 
								(recv[10 + n1] > 0x39 && recv[10 + n1] < 0x41) ||
								recv[10 + n1] > 0x5A)
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
								EEPROM_write(EEPROM_ADDR_START_BYTE + n1, recv[10 + n1]);
							}
							send_enable = 1;
							send_size = v - 1;
						}
					}
					else if (memcmp(recv + 1 + ADDRESS_LEN, cmd_list[SS], strlen(cmd_list[SS])) == 0
					&& v == ADDRESS_LEN + 2 + cmd_len[SS]
					&& write_enabled == 1)
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
								setting[n1] = recv[10 + n1];
								EEPROM_write(EEPROM_SETTING_START_BYTE + n1, recv[10 + n1]);
							}
							send_enable = 1;
							send_size = v - 1;
							check_sum_enable = setting[0] - 0x30;
							delay = setting[1] - 0x30;
						}
					}
					
					if (send_enable == 1)
					{
						send_enable = 0;
						if (delay != 0)
						{
							uint8_t n3 = 0;
							for(n = 0; n < delay; n++)
							{
								_delay_us(1040);
							}
						}
						printRS485Bytes(recv, send_size, check_sum_enable);
					}
					
				}
				
				v = 0;
			}
		}
		else{
			v = 0;
		}

	}
}

