
#include <Arduino.h> 
// for RiotCtrl...
// version 2

// from UART.h
#define Baudrate              9600                      //bps
// n is subtracted to cover processing time...  n=9, n=10
#define OneBitDelay           ((1000000/Baudrate) - 9)
#define DataBitCount          8                         // no parity, no flow control
#define UART_RX               GP1						// UART RX pin
#define UART_TX               GP0						// UART TX pin
#define UART_RX_DIR			  TRISIO1					// UART RX pin direction register
#define UART_TX_DIR			  TRISIO0					// UART TX pin direction register

// UNO:
int bangSerialRX = 0; // 12; // 7; 
int bangSerialTX =  1; // 8;

//Function Declarations
void InitSoftUART(int, int);
unsigned char UART_Receive(void);
int UART_RecieveWait(int cnt);
void UART_Transmit(const char);
void UART_print(const char *);
void UART_println(const char *);

//from UART.C

void InitSoftUART(int rx, int tx)		// Initialize UART pins to proper values
{
  // Serial.begin(9600); 
  // Serial.println("BB Init!");  
  // Serial.println(OneBitDelay);

    bangSerialRX = rx;
    bangSerialTX = tx;

	// UART_RX_DIR = 1;		// Input
	// UART_TX_DIR = 0;		// Output
    pinMode(bangSerialTX, OUTPUT);
    pinMode(bangSerialRX, INPUT);
  
    // UART_TX = 1;			// TX pin is high in idle state
    digitalWrite(bangSerialTX, 1); 
	  // Serial.println("BB init done!"); 
}
int UART_RecieveWait(int cnt) {
  for (unsigned int i=0; i < cnt; i++) {
    for (unsigned int j=0; j < 1000; j++) {
      if (digitalRead(bangSerialRX) == 0) {
        return UART_Receive();
        // break;
      }  
    }
  }
  return -1;
}
unsigned char UART_Receive(void)
{
	// Pin Configurations
    // GP1 is UART RX Pin
	unsigned char DataValue = 0;

	//wait for start bit
	while(digitalRead(bangSerialRX)==1);
    
  noInterrupts();
  
	// __delay_us(OneBitDelay);
    delayMicroseconds(OneBitDelay);
	// __delay_us(OneBitDelay/2);   // Take sample value in the mid of bit duration
    delayMicroseconds(OneBitDelay/2);

	for ( unsigned char i = 0; i < DataBitCount; i++ )
	{
		if (digitalRead(bangSerialRX) == 1 )   //if received bit is high
		{
			DataValue += (1<<i);
		}

		// __delay_us(OneBitDelay);
          delayMicroseconds(OneBitDelay);  
	}

	// Check for stop bit
	if ( digitalRead(bangSerialRX) == 1 )       //Stop bit should be high
	{
      interrupts(); 
		// __delay_us(OneBitDelay/2);
        delayMicroseconds(OneBitDelay/2);
		return DataValue;
	}
	else                      //some error occurred !
	{
      interrupts(); 
		// __delay_us(OneBitDelay/2);
        delayMicroseconds(OneBitDelay/2);
		return 0x000;
	}
}

void UART_Transmit(const char DataValue)
{
	/* Basic Logic
	   
	   TX pin is usually high. A high to low bit is the starting bit and 
	   a low to high bit is the ending bit. No parity bit. No flow control.
	   BitCount is the number of bits to transmit. Data is transmitted LSB first.

	*/
  //Serial.println("BB Tran!");  

  // DEBUG
  // return;

  // try it noInterrupts();
	// Send Start Bit
	// UART_TX = 0;
    digitalWrite(bangSerialTX, 0);   
    // Serial.println(digitalRead(bangSerialTX));  
	// __delay_us(OneBitDelay);
    delayMicroseconds(OneBitDelay);      

	for ( unsigned char i = 0; i < DataBitCount; i++ )
	{
		//Set Data pin according to the DataValue
		if( ((DataValue>>i)&0x1) == 0x1 )   //if Bit is high
		{
			// UART_TX = 1;
            digitalWrite(bangSerialTX, 1);  
            // Serial.print("1: ");
            // Serial.println(digitalRead(bangSerialTX));            
		}
		else      //if Bit is low
		{
			// UART_TX = 0;
            digitalWrite(bangSerialTX, 0); 
            // Serial.print("0: ");            
            // Serial.println(digitalRead(bangSerialTX)); 
		}

	    // __delay_us(OneBitDelay);
        delayMicroseconds(OneBitDelay);  
	}

	//Send Stop Bit
	// UART_TX = 1;
    digitalWrite(bangSerialTX, 1);      
    // try it interrupts(); 
	  //__delay_us(OneBitDelay);
    delayMicroseconds(OneBitDelay);      
    // Serial.println("BB Tran done!");  
}

void UART_print(const char * str) {
  for (int i=0; i < strlen(str); i++) {
    UART_Transmit(str[i]);
  }
  return;
}

void UART_println(const char * str) {
  UART_print(str);
  UART_Transmit('\r');
  UART_Transmit('\n');
  delay(20); // debug
  return;
}
