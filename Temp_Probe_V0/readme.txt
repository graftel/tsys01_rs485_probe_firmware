RS485 TSYS01 Probe

This is a RS485 enabled TSYS01 based temperature probe.

1. Programming

Wiring:

Front
 --------------
|Gnd  B  A  VCC|
|			   |
|			   |
|			   |
|	  RST      |
|			   |
|              |
|MOSI  MISO SCK|
|	           |
|			   |
 --------------
 
 
 Step 1: Connect wires to AVR programming, default is USBtiny
 
 Step 2: Burn Low Fuse bit with "make set_xtal_fuse"
 
 Step 3: Burn with "make flash"
 
 2. Command Sets
   1)  Read Raw Temperature Reading
		
		*[ADDRESS]RD\r    
		
		Default address is 200000
		Sample: *200000RD\r
		Resp:   #200000RD=95D2CA\r   (without checksum)
	            #200000RD=95BCD685\r (checksum)
				
				
		Check sum is calculated by add all hex together and & 0xff.
		Example:
		
		# = 0x23
		2 = 0x32
		0 = 0x30
		0 = 0x30
		0 = 0x30
		0 = 0x30
		0 = 0x30
		R = 0x52
		D = 0x44
		= = 0x3D
		9 = 0x39
		5 = 0x35
		B = 0x42
		C = 0x43
		D = 0x44
		6 = 0x36
		
		Sum = 0x385
		Sum = Sum & 0xff;
		Sum = 0x85
				
	2) Write enable
		
		*[ADDRESS]WE\r 
	   Resp: #[ADDRESS]WE[CHECKSUM]\r
	   
	3) Set New Address
	
		*[ADDRESS]SA=[NEW ADDRESS]\r
		Resp: *[ADDRESS]SA=[NEW ADDRESS]\r
	   
	   
	   Example: *200000WE\r
				*200000SA=200001\r

	4) Read Coefficient from 0 to 5
		
		*[ADDRESS]RE0\r
	    *[ADDRESS]RE1\r
		*[ADDRESS]RE2\r
		*[ADDRESS]RE3\r
		*[ADDRESS]RE4\r
		
		
	5) Read Calibration Constants
		
		*[ADDRESS]RCA\r
		*[ADDRESS]RCB\r
		*[ADDRESS]RCC\r
		
	6) Set Calibration Constants
		
		*[ADDRESS]SCA\r
		*[ADDRESS]SCB\r
		*[ADDRESS]SCC\r