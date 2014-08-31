// Copyright André lachaud 2009-03-15 
// version SLEEP Quartz 32K
// emetteur Tic _ Radio RC1180 

#include <avr\io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "rx_tx_wmbus.h"
#include "serial_port.h"
#include "timer2_rtc.h"
#include "TIC_routines.H"
#define nop()  __asm__ __volatile__ ("nop" ::)
#define EI()  __asm__ __volatile__ ("sei" ::)
#define DI()  __asm__ __volatile__ ("cli" ::)
#define SLEEP()  __asm__ __volatile__ ("sleep" ::)
#define PAIR 0
#define IMPAIR 1

extern unsigned int vbat;
void wait_sec(void);
void wait_100msec();
void update_vbat(CBE_STRUCT * cbe);
//unsigned char radio_sem;
CBE_STRUCT cbe1, cbe2;

void init_TIC_system(void)
{
//	radio_sem=0;
 	init_cbe_struct(& cbe1,1);
	init_cbe_struct(& cbe2,2);
}

unsigned char tix;
void collect_TIC( CBE_STRUCT * cbe, unsigned char rx_byte)
{
// 	durée de traitement entre  2 caracteres 10 bits : 1 start +_bits + 1 stop
//	soit 1000/120 = 8ms donc 8000 instructions environ
//	rx_byte&=0x7F;
	if(rx_byte == 0x02) {									// prepare nouvelle trame
	cbe->start_frame=1;
	cbe->frame_cnt++;
	} else if(rx_byte && cbe->start_frame) {
//	} else if((rx_byte >0) && (cbe->start_frame == 1)){
  		switch(rx_byte)										// si non error ( caractères ASCCII Attendus excluant 0x00 )
				{
				case 0x0A:											// start item
					cbe->p_raw=cbe->item_raw;				// se repositionne au début du buffer d'entrée
					cbe->idx_rx=0;									// remet à zero le compteur d'octets entrants
					break;
				case 0x0D:														// end item
					cbe->item_raw[cbe->idx_rx--]=0;		// zero final dans le buffer d'entrée TIC
					cbe->item_raw[--cbe->idx_rx]=0;		// zero final remplace ' ' avant item checksum
					update_item(cbe);							// si changement recopie dans le buffer de sortie
					break;
				case 0x03:														// end full frame
					cbe->start_frame=0;
//PORTD|=0x08;
				//****		rf_put_message(cbe);
//PORTD&=~0x08;
					break;
				default:
					if(cbe->idx_rx > (SZ_item_raw -1) ) {
						cbe->idx_rx=0;								// empeche le debordement
						cbe->start_frame=0;
					} else cbe->item_raw[cbe->idx_rx++]=rx_byte;
					break;
				}
	}
} 

void update_item(CBE_STRUCT * cbe)
{
	unsigned char lix;
// met à jour si nécessaire le contenu de chaque champ
// dans ce cas passe en minuscule la lettre du label
// qui sera rebasculé en majuscule après ré-émission de la trame complète
	switch(cbe->item_raw[3])
		{
		case 'O':			// ADCO
			cbe->p_tx = cbe->radio_frame;
			lix=84;
			while(lix--)
				*cbe->p_tx++=0x55;
			cbe->p_tx = cbe->radio_frame;
					
			cbe->frame_length=0;
//			if(! (cbe->frame_cnt++ & 0X0007) ) {				// 1 trame sur 8// toute les 256 réinitialise la structure (impose le renvoie d'une trame complète)
			 	init_cbe_struct(cbe, cbe->num_voie);
	//		}
			cbe->vbat = vbat;
			update_vbat(cbe);
			update_num_frame(cbe);						
			update_num_item(cbe->adco, cbe, PAIR);
			nop();
			break;
		case 'A':			// OPTARIF   
			update_alpha_item(cbe->optarif, cbe);
			break;
		case 'U':			// ISOUSC
			update_num_item(cbe->isousc, cbe, PAIR);
			break;
		case 'E':			// BASE
			update_num_item(cbe->base, cbe, IMPAIR);
			break;
		case 'C':			// HCHC, PTEC
			if(cbe->item_raw[2]=='H')
				update_num_item(cbe->hchc, cbe, IMPAIR);
				else update_alpha_item(cbe->ptec, cbe);
			break;
		case 'P':			// HCHP, PAPP
			if(cbe->item_raw[2]=='H')
				update_num_item(cbe->hchp, cbe, IMPAIR);
				else update_num_item(cbe->papp, cbe,IMPAIR);
			break;
		case 'S':			// IINST, ADPS
			if(cbe->item_raw[2]=='N')
				update_num_item(cbe->iinst, cbe, IMPAIR);
				else update_num_item(cbe->adps, cbe, IMPAIR);
			break;
		case 'X':			// IMAX
			update_num_item(cbe->imax, cbe, IMPAIR);
			break;
		case 'H':			// HHCHP
			update_alpha_item(cbe->hhphc, cbe);
			break;
		case 'D':			// MOTDETAT
			update_alpha_item(cbe->motdetat, cbe);
			 nop();
			break;
		default:
				//	error !!
			break;
		}
}

void update_vbat(CBE_STRUCT * cbe)
{
unsigned char dig;
	*cbe->p_tx++='E';
	dig=0;
	while(cbe->vbat>100) {
		dig++;
		cbe->vbat-=100;
	}
	if(!dig) {

		*cbe->p_tx++=cbe->num_voie<<4 | 0x0A;
	} else {
	*cbe->p_tx++= cbe->num_voie<<4 | dig;
	}
	dig=0;
	while(cbe->vbat>10) {
		dig++;
		cbe->vbat-=10;
	}
	if(!dig) {
		dig=0x0A;
	} 
	dig<<=4;
	if(!cbe->vbat) {
		cbe->vbat=0x0A;
	} 
	*cbe->p_tx++ = dig|cbe->vbat;
	cbe->frame_length += 3;

//	cbe->vbat = 0x00;
}
void update_num_frame(CBE_STRUCT * cbe)
{
unsigned char dig, nframe;
	nframe=cbe->nframe;
	*cbe->p_tx++='N';
	dig=0;
	while(nframe>100) {
		dig++;
		nframe-=100;
	}
	if(!dig) {
		*cbe->p_tx++=0xAA;
	} else {
	*cbe->p_tx++= 0xA0 | dig;
	}
	dig=0;
	while(nframe>10) {
		dig++;
		nframe-=10;
	}
	if(!dig) {
		dig=0x0A;
	} 
	dig<<=4;
	if(!nframe) {
		nframe=0x0A;
	} 
	*cbe->p_tx++ = dig|nframe;
	cbe->frame_length +=3;
}

/*
void mk_num(unsigned char *item, unsigned char *label, unsigned char nb_char)
{
  pfull_tic=item;
	*pfull_tic++=0x0A;
	checksum=0;
	// recopie le label (avec calcul du checksum)
	while( (abyte = *label++) )	{
		length--;
		*pfull_tic++=abyte;
		checksum+=abyte;
	}

	*pfull_tic++=0x20;									//	espace separateur
	checksum+=0X20;

	if(nb_char & 1) {
		abyte=*p_rx++;
		length--;
//		--nb_char;
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
		length--;
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
		length--;
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
*/

void update_num_item(unsigned char * item, CBE_STRUCT * cbe, char impair)
{
// fonctionne pour un nombre impair de digits (High nibble = 0
unsigned char to_send, a_bcd, nibble, a_byte;
	to_send=0;
	cbe->checksum=0;
	cbe->p_raw=cbe->item_raw;
	cbe->idx_rx=0;
	while((a_byte=*cbe->p_raw++) != ' ')										// recherche l'espace qui sépare le label des données
		cbe->checksum+=a_byte;
		cbe->checksum+=a_byte;
	if( impair){														// si impair le premier nibble == 0
		nibble=((a_byte=*cbe->p_raw++) -'0');
		cbe->checksum+=a_byte;	
		if(nibble)
			a_bcd=0xA0|nibble;
			else a_bcd=0xAA;									// 1010 afin de limiter les longues suite de zero dans la radio									
		if(a_bcd != item[++cbe->idx_rx]){
			to_send=1;									
			item[cbe->idx_rx] = a_bcd;
		}
	}
	
	while( (a_byte=*cbe->p_raw++) ){
		cbe->checksum+=a_byte;
		nibble=(a_byte -'0')<<4;								// poids fort
		if(nibble)
			a_bcd=nibble;
			else a_bcd=0xA0;										// 1010 afin de limiter les longues suite de zero dans la radio									
		nibble=( (a_byte=*cbe->p_raw++) -'0')&0x0F;						// poids faible
		cbe->checksum+=a_byte;
		if(nibble)
			a_bcd|=nibble;
			else a_bcd|=0x0A;										// 1010 afin de limiter les longues suite de zero dans la radio									
		if(a_bcd != item[++cbe->idx_rx]){
			to_send=1;													
			item[cbe->idx_rx] = a_bcd;
		}
	}// cbe->frame_length == 6  car on a envoyé vbat et nframe avant adco
	if( cbe->frame_length == 6 ) {									// on envoie l'ADCO quoiqu'il arrive on a 				
		cbe->frame_length += ++cbe->idx_rx;
	//	cbe->p_raw=cbe->item_raw;
		a_bcd=0;
		while(cbe->idx_rx--){
			*cbe->p_tx++=item[a_bcd++];
		}
	}	else { 
	cbe->checksum&=0X03F;
	cbe->checksum+=0X20;
	if( cbe->checksum!=*(cbe->p_raw) )
		cbe->frame_err++;
	
		if(to_send){
			cbe->frame_length += ++cbe->idx_rx;
//		cbe->p_raw=cbe->item_raw;
			a_bcd=0;
			while(cbe->idx_rx--){
				*cbe->p_tx++=item[a_bcd++];
			}
		}
	}
}

/*
	*pfull_tic++=0x20;									//	espace separateur
	checksum&=0X03F;
	checksum+=0X20;
	*pfull_tic++=checksum;
	*pfull_tic++=0x0D;

*/
void update_alpha_item(unsigned char * item,  CBE_STRUCT *  cbe)
{
unsigned char to_send, a_byte;
	to_send=0;
	cbe->checksum=0;	//µµ
	cbe->p_raw=cbe->item_raw;
	cbe->idx_rx=0;
	while((a_byte=*cbe->p_raw++) !=' ')								// recherche l'espace qui sépare le label des données
		cbe->checksum+=a_byte;
		cbe->checksum+=a_byte;
	while( (a_byte=*cbe->p_raw++) ) {					//  item brut terminé par \x0
		if(a_byte != item[++cbe->idx_rx]) {
			item[cbe->idx_rx]=a_byte;
			to_send = 1;
		}
		cbe->checksum+=a_byte;
	}
	cbe->checksum&=0X03F;
	cbe->checksum+=0X20;
	if( cbe->checksum!=*(cbe->p_raw) )
		cbe->frame_err++;
	if(to_send){
		cbe->frame_length += ++cbe->idx_rx;
//		cbe->p_raw=cbe->item_raw;
		a_byte=0;
		while(cbe->idx_rx--){
			*cbe->p_tx++=item[a_byte++];
		}
	}
}

void init_cbe_struct(CBE_STRUCT * cbe, unsigned char num_voie)
{
unsigned int icnt;
	icnt= (int)( & cbe->fin) -  (int)( & cbe->frame_length );
	cbe->num_voie=num_voie;
	cbe->nframe++;
	cbe->p_tx = cbe->adco;
	while(icnt--)
		*cbe->p_tx++=0x00;
	cbe->adco[0]='A';
	cbe->optarif[0]='O';
	cbe->isousc[0]='I';
	cbe->base[0]='B';
	cbe->hchc[0]='C';
	cbe->hchp[0]='P';
	cbe->ptec[0]='T';
	cbe->iinst[0]='S';
	cbe->adps[0]='D';
	cbe->imax[0]='X';
	cbe->papp[0]='V';
	cbe->hhphc[0]='H';
	cbe->motdetat[0]='M';
}

