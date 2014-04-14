#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uartlib/uartlib.h"

/* 9600 baud */
#define UART_BAUD_RATE      115200    

#define F_CPU	16000000UL 	/* in Hz */

// Define baud rate
#define USART_BAUDRATE	115200   
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)


void initUART()
{
	// Set baud rate

	// Load lower 8-bits into the low byte of the UBRR register
	UBRR0L = BAUD_PRESCALE;
	// Load upper 8-bits into the high byte of the UBRR register
	UBRR0H = (BAUD_PRESCALE >> 8); 
	
    /* Default frame format is 8 data bits, no parity, 1 stop bit
	to change use UCSRC, see AVR datasheet*/ 

  	// Enable receiver and transmitter and receive complete interrupt 
	UCSR0B = ((1<<TXEN)|(1<<RXEN) | (1<<RXCIE));
}


uint8_t uartReceiveCharacter()
{
	/* Wait for data to be received */
	while ( ! (UCSR0A & (1<<RXC0)));
	/* Get and return received data from buffer */
	return UDR0;
}

void uartTransmitCharacter(uint8_t ch)
{
	/* Wait for empty transmit buffer */
	while ( !(UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = ch;
}

void uartTransmitString(uint8_t* str)
{
	while(*str)
	{
		uartTransmitCharacter(*str++);
   } 	
}

