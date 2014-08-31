// serial_port.c
// ATMEGA644 à 4Mhz 8/2
#include <avr\io.h>
#include <avr\interrupt.h>
//#include "TIC_routines.h"

#define nop()  __asm__ __volatile__ ("nop" ::)
#define EI()  __asm__ __volatile__ ("sei" ::)
#define DI()  __asm__ __volatile__ ("cli" ::)
#define RETI()  __asm__ __volatile__ ("reti" ::)
#define F_CPU 4000000UL
#define BAUD 19200
#define ubrr (F_CPU/16/BAUD)-1
#define BAUD2 19200
#define ubrr2 (F_CPU/16/BAUD2)-1

unsigned char volatile rx_byte_rf, rx_error_rf;
extern unsigned char volatile rc_cnt, rc_ix, rc_ox;
extern unsigned char rc_buffer[256];

unsigned int tt;
//extern CBE_STRUCT cbe1, cbe2;
// ATTENTION CLOCK 4MHZ
void init_serial_usb( void )
{
// PORTD |= 0X01;									// pull-up enabled
//UBRR0H = ubrr/256;							// Set baud rate
UBRR0L = ubrr;
//UCSR0A = 0X02;									//  + U2Xn
UCSR0B = 0X18;									// rx autorise + tx complete interrupt enable
//UCSR0C = 0X24;									// Set data format: 7 data, even, 1 stop bit  no parity
UCSR0C = 0X06;									// Set data format: 8 data, no parity, 1 stop bit  no parity
}

void init_serial_rf( void )
{
// PORTD |= 0X04;									// pull-up enabled
//UBRR1H = ubrr/256;							// Set baud rate
UBRR1L = ubrr2;
//UCSR1A = 0X02;									//  + U2Xn
UCSR1B = 0X98;									// rx + tx autorise + rx complete interrupt enable
//UCSR1C = 0X24;									// Set data format: 7 data, even, 1 stop bit  no parity
UCSR1C = 0X06;									// Set data format: 8 data, no parity, 1 stop bit  no parity
}

/*
ISR(USART0_RX_vect)
{
unsigned char sreg;
	sreg = SREG;
	if( UCSR0A & 0X18) {	//1C			// Frame error | Data Overun error | parity error
		while ( UCSR0A & 0X80 ) 		// reset le received byte status bit
			rx_byte1 = UDR0;					// vider le tampon de reception
			rx_byte1=0;									// en cas d'erreur dans un champ j'attends le champs valide suivant
			} else rx_byte1=(UDR0 & 0x7F);	//  Enleve le bit de parité
SREG = sreg;
}
*/
ISR(USART1_RX_vect)
{											// réception RC1180
unsigned char sreg;
	sreg = SREG;
	if( UCSR1A & 0X18) {	//1C			// Frame error | Data Overun error | parity error
		while ( UCSR1A & 0X80 ) 		// reset le received byte status bit
			rx_byte_rf = UDR1;					// vider le tampon de reception
		rx_byte_rf=0;									// en cas d'erreur dans un champ j'attends le champs valide suivant
		rx_error_rf=1;
	} else {
		rc_cnt++;
		rc_buffer[rc_ix++]=UDR1;
	}
SREG = sreg;
}


