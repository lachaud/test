//timer2_rtc.c
#include <avr\io.h>
#include <avr\interrupt.h>

// #define TIC_EMETTEUR 1
#define nop()  __asm__ __volatile__ ("nop" ::)
#define EI()  __asm__ __volatile__ ("sei" ::)
#define DI()  __asm__ __volatile__ ("cli" ::)
#define SLEEP()  __asm__ __volatile__ ("sleep" ::)
extern unsigned char buffer_tx_radio[];
extern unsigned char volatile rx_timeout_rf;
typedef struct{
unsigned char volatile second;							//enter the current time, date, month, and year
unsigned char volatile minute;
unsigned char volatile hour;
unsigned char volatile date;
unsigned char volatile month;
unsigned int volatile year;
unsigned int volatile yday;
            }time;
volatile time t;
unsigned char volatile ms, ms_radio_starting, ms_radio_stopping;		// Maxi 255 puis reset
unsigned char volatile  getvbat, dc_nbsec, dc_nb60ms, sixteenth, msecs;
unsigned char vh, vdemi, vdixi;
unsigned int vbat;// GTCCR = 0x02;									// reset prescaler
void init_timer2_RTC(void)
{
//TIMSK2 &=~((1<<TOIE0)|(1<<OCIE0));				//Disable TC0 interrupt
ASSR |= 0x20;									// (AS2) + CLOCK 32K  set Timer/Counter0 to be asynchronous from the CPU clock
	//with a second external clock(32,768kHz)driving it.
//TCCR2B = 0x02;	// 0x05 prescaler /128, 0x02 => /8		//prescale the timer to be clock source / 128 to make it
TCCR2A=0x02;														// CTC mode
TCCR2B = 0x01;	// no prescaler
OCR2A = 32;							// 1 ms   32768/32 => 1024
while(ASSR&0x07)
	;														//Wait until TC0 is updated
TIMSK2 |= 0x02;								// OCR2A interrupt
//TIMSK2 |= 0x01;							//(TOIE2) set 8-bit Timer/Counter0 Overflow Interrupt Enable
sixteenth=ms=dc_nb60ms=0;
//ms_radio_starting=ms_radio_stopping=0;
//radio_status=RADIO_OFF;
}

ISR(TIMER2_COMPA_vect)
{
unsigned char sreg;
sreg = SREG;
EI();
	if(ms) {
		ms--;
	} else {
		ms=64;
		if(dc_nb60ms)							// 16 == 1seconde   80 = 5secondes  (max 15 secondes)
			dc_nb60ms--;
		if(rx_timeout_rf)
			rx_timeout_rf--;
		if(++sixteenth==16){
			sixteenth=0;
			getvbat=1;			// $$$$$$$  test
	  	if (++t.second==60){								//keep track of time, date, month, and year
				t.second=0;
				if (++t.minute==60){
					t.minute=0;
					if (++t.hour==24){
						t.hour=0;
						if(++t.yday==366){					// manage leap year
							t.yday=1;
							t.year++;
						}
					}
				}
			}
		}
	} //fin if(++sixteenth==16)
  	switch(sixteenth)
  		{
			case 0x01:
			case 0x03:
			case 0x05:
			case 0x08:
			case 0x0A:
			case 0x0C:
				break;
#ifdef	TIC_EMETTEUR
			case 0x06:
				break;
			case 0x07:
//	Avec alimentation à 3.3V et 16 mv par increment
//	4.08v pleine echelle
// 	3.30v  / 16 = 206
//  330k // Capa 104  en série avec 78k (330k // 100K)
//

//	Avec alimentation à 2.6V et 10 mv par increment
//	2.56v pleine echelle

//			ADMUX = 0XE3;				// ref =1,1V 0XC0 +adjust left 0X20 + ADC6 (0xE6)
				if(getvbat) {
					ADMUX = 0X60;				// ref ext(01)  +adjust Right (0) + ADC3  (3)
					ADCSRA = 0XC3;			// clock à 1Mhz/8 125K faire essai à 250K (0xC2)
					while(ADCSRA & 0X40)			// conversion in progress (25clk à 125Khz =>5ms)
							;
					vbat=ADCH;
					ADCSRA = 0X00;				// stop ADC economie d'energie
					getvbat=0;
				}
			break;
#endif
		default:
		 	break;
			}
SREG = sreg;
}


