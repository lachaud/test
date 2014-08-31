// TIC_Rx_Radio_USB.c
// Copyright AndrÃ© Lachaud 2011/06/29
// version SLEEP Quartz 32K
// emetteur Tic _ Radio RC1180

#include <avr\io.h>
#include <avr\interrupt.h>
#include "serial_port.h"
#include "timer2_rtc.h"
#include "TIC_routines.h"
#define nop()  __asm__ __volatile__ ("nop" ::)
#define EI()  __asm__ __volatile__ ("sei" ::)
#define DI()  __asm__ __volatile__ ("cli" ::)
#define SLEEP()  __asm__ __volatile__ ("sleep" ::)
#define PAIR 0
#define IMPAIR 1
void wait_sec(void);
void wait_100msec();
void w1_3msec(void);
// #define TIC_EMETTEUR	1

unsigned char get_prompt(void);
void rf_put_byte( unsigned char abyte );
unsigned char rf_put_config( unsigned char conf_start, unsigned char *conf_bytes, unsigned char conf_count );
unsigned int rf_get_config( void); 

unsigned char rc_conf_master[]={
	0x01, 0x05, 0x03, 0x03, 0x00, 0x01, 0x64, 0x00, 0x05, 0x3C, 0x00, 0xD3, 0x91, 0xDA, 0x80, 0x80,
	0x7C, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xAE, 0x12, 0x34, 0x56, 0x78, 0x01,
	0x07, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0xFF, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x05, 0x08, 0x00, 0x01, 0x05, 0x00, 0x01, 0x01, 0x2B, 0x00, 0x01, 0x06, 0x56, 0x02, 0x01, 0x01,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x52, 0x43, 0x31, 0x31, 0x38, 0x30, 0x2D, 0x4D, 0x42, 0x55, 0x53, 0x33, 0x2C, 0x32, 0x2E,
	0x30, 0x30, 0x2C, 0x33, 0x2E, 0x30, 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

typedef struct
{
// le champ comprend le 0X0A label sp data sp chksum 0x0D
unsigned char stx;
unsigned char rssi[12];
unsigned char ubat[12];
unsigned char nframe[14];
unsigned char adco[21];
unsigned char optarif[16];
unsigned char isousc[13];
unsigned char base[18];
unsigned char hchc[18];
unsigned char hchp[18];
unsigned char ptec[13];
unsigned char iinst[13];
unsigned char adps[12];
unsigned char imax[12];
unsigned char papp[14];
unsigned char hhchp[11];
unsigned char motdetat[19];
unsigned char etx;
unsigned char end;
}full_tic_frame;

void init_ftf(full_tic_frame *ftf);
void decode_tic_wmbus(void);
void wsec(void);

unsigned char *pstrtic, strtic[300];
unsigned char nb, xx, length, * pi, item_flag;
unsigned char checksum, abyte, anum, *p_rx;
void make_full_tic_frame(full_tic_frame * ftf);
void mk_num(unsigned char * item, unsigned char * label, unsigned char nb_char);
void mk_alpha(unsigned char * item, unsigned char * label, unsigned char nb_char);
void mk_rssi(unsigned char *item, unsigned char *label, unsigned char vrssi);
void usb_put_byte( unsigned char abyte );
void usb_put_message( unsigned char *msg );
unsigned char volatile dc_nb60ms, dc_nbsec, must_send, new_tic, sixteenth, msecs;
unsigned char sbyte, *pframe, message[100], *pmsg, xmsg;
unsigned char volatile  flg_dat_cmd;

unsigned char rssi[]="RSSI";
unsigned char ubat[]="VBAT";
unsigned char nframe[]="NFRAME";
unsigned char adco[]="ADCO";
unsigned char optarif[]="OPTARIF";
unsigned char isousc[]="ISOUSC";
unsigned char base[]="BASE";
unsigned char hchc[]="HCHC";
unsigned char hchp[]="HCHP";
unsigned char ptec[]="PTEC";
unsigned char iinst[]="IINST";
unsigned char adps[]="ADPS";
unsigned char imax[]="IMAX";
unsigned char papp[]="PAPP";
unsigned char hhchp[]="HHCHP";
unsigned char motdetat[]="MOTDETAT";

unsigned char *pfull_tic;
full_tic_frame ftf1;
unsigned char rx_frame_length, count, debug;
extern unsigned char volatile rx_byte_rf, rx_error_rf;
unsigned char volatile rx_timeout_rf;
unsigned char volatile rc_cnt, rc_ix, rc_ox;
unsigned char rc_buffer[256];

int main(void)
{
	debug=1;
	rc_cnt = rc_ix = rc_ox = 0;
	rx_error_rf=0;
	rx_frame_length=66;	//			length + 0x40 + rssi
	rx_timeout_rf=0xFF;
	xx=0;
	CLKPR=0x80;
	CLKPR=0x01;					// 8Mhz /2 // clock system = 4 000 000 ( pour timers etc ...)
	init_serial_rf();
	init_serial_usb();
	init_timer2_RTC();
	EI();


	init_ftf(&ftf1);
	xmsg=rx_frame_length; // reinit
	while(--xmsg)
		message[xmsg]=0;
	message[0]=0;
	// xmsg=0;
	rx_timeout_rf=0x40;	// 60ms * 64 = 4 secondes
//	rf_get_config();
	rf_put_config(0, rc_conf_master, 255);
//	rf_get_config();
	flg_dat_cmd = 1;				// mode data


//while(1)
	//usb_put_message("ok out mesagege");
    while(1)
    {
		SMCR=0X06;													// Sleep enable (mode =001 = idle)
		SLEEP();														// will wake up from time overflow interrupt
		SMCR=0X00;									  				// sleep disable (mode =000 = )
		nop();

		if(rc_cnt) {
			rx_timeout_rf=0x20;//04
			message[xmsg++]= rc_buffer[rc_ox++];
			rc_cnt--;
			if(xmsg==rx_frame_length) {
//				rx_timeout_rf=0x08;	// 60ms * 32 = 2 secondes
				decode_tic_wmbus(); //traitement
				xmsg=0;
			}
		}
		if(rx_error_rf) {
//			  usb_put_message("n  rx fr error *******************n" );
			led_red();
			rx_error_rf=0;
		}

		if(!rx_timeout_rf) {
			xmsg=rx_frame_length; // reinit
			while(--xmsg)
				message[xmsg]=0;
			message[0]=0;
			xmsg=0;
//			if(debug)
	//			led_red();
			rx_timeout_rf=0xFF;	//
		}
    }
}

void init_ftf(full_tic_frame *ftf)
{
	length = sizeof(full_tic_frame);
	pi=&ftf->stx;
	while(length--)	{
	  *pi++=0xFF;
	}
	ftf->stx=0x02;
	ftf->etx=0x03;
	ftf->end=0x00;
}

void decode_tic_wmbus(void)
{
		//////////////////////  debut  Correction error temporaire
unsigned char err_xx, err_aa;
err_xx = 64;
err_aa = 0;	
//	 usb_put_message("ABCDEFGHIJklmnop123456789");
while( err_xx) {
	message[err_aa]=message[err_aa+2];
	err_aa++;
	err_xx--;
}	
	//////////////////////  fin  Correction error temporaire

	if( ( message[1] == 'E' ) && ( message[2] & 0xF0) == 0X10 ) {
		make_full_tic_frame(&ftf1);
	}  else if(debug)
				led_red();

}

void make_full_tic_frame(full_tic_frame * ftf)
{
unsigned char abyte;
	nb=rx_frame_length-2;
	while(message[nb]=='U')
	  message[nb--]=0;
	p_rx=message;
	if(*p_rx++ != rx_frame_length-2) {
		if(debug)
			led_red();
		return;
	}
	init_ftf(ftf);
	mk_rssi(ftf->rssi, rssi, message[rx_frame_length-1]);
	while(*p_rx) {
		switch(*p_rx++)
			{
			case 'E':
				mk_num(ftf->ubat, ubat, 3);
				break;
			case 'N':
				mk_num(ftf->nframe, nframe, 3);
				break;
			case 'A':
				mk_num(ftf->adco, adco, 12);
				break;
			case 'O':
				mk_alpha(ftf->optarif, optarif, 4);
				break;
			case 'I':
				mk_num(ftf->isousc, isousc, 2);
				break;
			case 'B':
				mk_num(ftf->base, base, 9);
				break;
			case 'C':
				mk_num(ftf->hchc, hchc, 9);
				break;
			case 'P':
				mk_num(ftf->hchp, hchp, 9);
				break;
			case 'T':
				mk_alpha(ftf->ptec, ptec, 4);
				break;
			case 'S':
				mk_num(ftf->iinst, iinst, 3);
				break;
			case 'D':
				mk_num(ftf->adps, adps, 3);
				break;
			case 'X':
				mk_num(ftf->imax, imax, 3);
				break;
			case 'V':
				mk_num(ftf->papp, papp, 5);
				break;
			case 'H':
//			*ftf->hhchp=0x55;
				mk_alpha(ftf->hhchp, hhchp, 1);
				break;
			case 'M':
				mk_alpha( ftf->motdetat, motdetat, 6);
				break;
			default:
				if(debug)
					led_red();
				break;
			}
	}
  pstrtic = strtic;
  p_rx= & ftf->stx;
  abyte=*p_rx++;
  while(abyte) {
    if(!(abyte==0xFF) )
      *pstrtic++ = abyte;
    abyte=*p_rx++;
  }
  *pstrtic++ = 0;
  usb_put_message( strtic );
}

void mk_num(unsigned char *item, unsigned char *label, unsigned char nb_char)
{
	pfull_tic=item;
	*pfull_tic++=0x0A;
	checksum=0;
	// recopie le label (avec calcul du checksum)
	while( (abyte = *label++) )	{
		*pfull_tic++=abyte;
		checksum+=abyte;
	}

	*pfull_tic++=0x20;									//	espace separateur
	checksum+=0X20;

	if(nb_char & 1) {
		abyte=*p_rx++;
		if( (anum=(abyte & 0x0F) ) == 0X0A ) {
			*pfull_tic++='0';
			checksum+='0';
		} else {
			anum+='0';
			checksum+=anum;
			*pfull_tic++=anum;
		}
	}
	nb_char>>=1;
	while(nb_char) {
		abyte=*p_rx++;
		--nb_char;
		if( (anum=(abyte & 0xF0) ) == 0XA0 ) {
			*pfull_tic++='0';
			checksum+='0';
		} else {
			anum=(anum>>4)+'0';
			*pfull_tic++=anum;
			checksum+=anum;
		}
		if( (anum=(abyte & 0x0F)) == 0X0A ) {
			*pfull_tic++='0';
			checksum+='0';
		} else {
			anum+='0';
			checksum+=anum;
			*pfull_tic++=anum;
		}
	}

	*pfull_tic++=0x20;									//	espace separateur
	checksum&=0X03F;
	checksum+=0X20;
	*pfull_tic++=checksum;
	*pfull_tic++=0x0D;
}

void mk_alpha(unsigned char *item, unsigned char *label, unsigned char nb_char)
{
  pfull_tic=item;
	*pfull_tic++=0x0A;
	checksum=0;
	// recopie le label (avec calcul du checksum)
	while((abyte = *label++))	{
	*pfull_tic++=abyte;
	checksum+=abyte;
	}

	*pfull_tic++=0x20;									//	espace separateur
	checksum+=0X20;

	while(nb_char)	{
		abyte=*p_rx++;
		--nb_char;
		*pfull_tic++=abyte;
		checksum+=abyte;
	}

	*pfull_tic++=0x20;									//	espace separateur
	checksum&=0X03F;
	checksum+=0X20;
	*pfull_tic++=checksum;
	*pfull_tic++=0x0D;
}

void w1_3msec(void)
{
	unsigned int count;
		for(count=1;count<1000;count++)
			;
}

void wsec(void)
{
	unsigned char aa;
	unsigned int count;
	for(aa=0; aa< 2;aa++)
		for(count=1;count<13000;count++)
			;
}


void mk_rssi(unsigned char *item, unsigned char *label, unsigned char vrssi)
{
unsigned char dig;
	pfull_tic=item;
	*pfull_tic++=0x0A;
	checksum=0;
	// recopie le label (avec calcul du checksum)
	while( (abyte = *label++) )	{
		*pfull_tic++=abyte;
		checksum+=abyte;
	}
	*pfull_tic++=0x20;									//	espace separateur
	checksum+=0X20;

	dig=0;
	while(vrssi>99) {
		dig++;
		vrssi-=100;
	}
	dig+='0';
	checksum+=dig;
	*pfull_tic++=dig;
	dig=0;

	while(vrssi > 9) {
		dig++;
		vrssi -= 10;
	}
	dig+='0';
	checksum+=dig;
	*pfull_tic++=dig;

	dig= '0' + vrssi;
	checksum+=dig;
	*pfull_tic++=dig;

	*pfull_tic++=0x20;									//	espace separateur
	checksum&=0X03F;
	checksum+=0X20;
	*pfull_tic++=checksum;
	*pfull_tic++=0x0D;
}

void usb_put_byte( unsigned char abyte )
{
/* Wait for empty transmit buffer */
	while ( !( UCSR0A & 0x20) )
			;
/* Put data into buffer, sends the data */
	UDR0 = abyte;
}


void usb_put_message( unsigned char *msg )
{
unsigned char abyte;
	abyte=*msg++;
	while(abyte) {
  		usb_put_byte(abyte);
		abyte=*msg++;
 	}
}


unsigned char get_prompt(void)
{
	wait_rc();						// delai reponse
//	if(rc_cnt == 1)	{
		if(rc_buffer[rc_ix-1] == '>') {
//			rc_ox++;
			rc_cnt--;
			return(0);				// ok
		}
//	}
	return(1);
}

void rf_put_byte( unsigned char abyte )
{
	while ( !( UCSR1A & 0x20) ) /* Wait for empty transmit buffer */
			;
	UDR1 = abyte;		/* Put data into buffer, sends the data */
}

#define red_led 0x40

void led_red(void)
{
unsigned int volatile duree;
DDRA |= red_led;
PORTA |= red_led;
PORTA &= ~red_led;
for(duree = 250; duree;)
		 duree--;
PORTA |= red_led;
DDRA &= ~red_led;
}

void wait_rc(void)
{		// 1.5msec
unsigned int volatile duree;

	for(duree = 3500; duree;)
		 duree--;
}

unsigned char rf_put_config( unsigned char conf_start, unsigned char *conf_bytes, unsigned char conf_count )
{
// start (à partir de 0) la case mémoire,
// conf_bytes le tableau de chars
// le nombre de bytes
 	unsigned int t31;
 	flg_dat_cmd = 0;				// mode commande
//	rf_put_byte(0xFF);				//
//	wait_rc();
	rf_put_byte(flg_dat_cmd);				// passe en mode commande
	if(get_prompt())
		return(1);							// erreur
	rf_put_byte('M');				//
	if(get_prompt())
		return(1);							// erreur

	while(conf_count--) {
		rf_put_byte(conf_start);			//adresse memoire
		rf_put_byte(conf_bytes[conf_start++]);	// contenu de la case memoire
//		wait_rc();							// delai reponse
	}

	rf_put_byte(0xFF);				//signalelafindespairesADDRESS/VALUE

	for(t31=0;t31<50000;)
		t31++;
//	if(get_prompt())

//		; //return(1);							// erreur
	rf_put_byte('X');				// sort du mode commande
 	flg_dat_cmd = 1;				// mode data
	if(get_prompt())
		return(1);							// erreur

return(0);
}

unsigned int rf_get_config( void)
//unsigned char *conf_bytes, unsigned int conf_count )
{
// start (à partir de 0) la case mémoire,
// conf_bytes le tableau de chars
// le nombre de bytes
 	unsigned int t31;
//	rf_put_byte(0xFF);				//
//	wait_rc();
 	flg_dat_cmd = 0;				// mode commande
	rf_put_byte(flg_dat_cmd);				// passe en mode commande
	if(get_prompt())
		return(1);							// erreur
	rf_put_byte(0x30);				//
	if(get_prompt())
		return(1);							// erreur
	for(t31=0;t31<250;t31++)
		wait_rc();
	rf_put_byte('X');				// sort du mode commande
 	flg_dat_cmd = 1;				// mode data
	if(get_prompt())
		return(1);							// erreur
return(0);
}

/*
	while(conf_count--) {
		aa=conf_start;				//
		rf_put_byte(aa);				//

//		rf_put_byte(conf_start);				//
//		wait_rc();						// delai reponse
		aa=conf_bytes[conf_start++];				//
		rf_put_byte(aa);				//

//		rf_put_byte(conf_bytes[conf_start++]);				//
//		wait_rc();						// delai reponse
	}

*/
