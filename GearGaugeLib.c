/* GearGaugeLib.c - v2.3

 (c) 2009 Seung-Won Lee  http://whoisit.tistory.com  SoftWareYi@gmail.com
 (c) 2018 LeoKevin    http://leokevin.com

 Manual Shift Gear Position Gauge with AVR (7-Segment)
 https://github.com/LeoKevinKR/AVR-GearGauge

 NOT COMMERCIALLY AVAILABLE
*/

#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdio.h> //sprintf() printf()
#include <string.h> //strlen() strcpy() strncpy()

#define GEARGAUGE_VER	"2.3"
//#define LIBDEBUG	1

/** 7-segment FND MAP
 *
 * 8bit: gfab.cde
   a
 f   b
   g
 e   c
   d   DP
**/
#define SEG_ALL		0b11111111 // 8.
#define SEG_MINUS	0b10000000 // -
#define SEG_0		0b01110111 // 0
#define SEG_1		0b00010100 // 1
#define SEG_2		0b10110011 // 2
#define SEG_3		0b10110110 // 3
#define SEG_4		0b11010100 // 4
#define SEG_5		0b11100110 // 5
#define SEG_6		0b11100111 // 6
#define SEG_7		0b00110100 // 7
#define SEG_8		0b11110111 // 8
#define SEG_9		0b11110110 // 9
#define SEG_BACK	0b10000001 // r
#define SEG_ERROR	0b11100011 // E
#define SEG_NEUTRAL	0b10000101 // n
#define SEG_b		0b11000111 // b
#define SEG_C		0b01100011 // C
#define SEG_d		0b10010111 // d ; not used
#define SEG_E		0b11100011 // E
#define SEG_F		0b11100001 // F
#define SEG_H		0b11010101 // H
#define SEG_L		0b01000011 // L
#define SEG_P		0b11110001 // P
#define SEG_S		0b11100110 // S
#define SEG_t		0b11000011 // t
#define SEG_U		0b01010111 // U
#define SEG_y		0b11010110 // y
#define SEG_LEFT	0b00000001 // left line
#define SEG_RIGHT	0b00000100 // right line
#define SEG_UP		0b00100000 // up line
#define SEG_DOWN	0b00000010 // down line
#define SEG_DOT		0b00001000 // DP(.)

#ifndef sbi
#	define sbi(RegX, BitX)	RegX |= _BV(BitX)	// Set Bit Instruction
#endif
#ifndef cbi
#	define cbi(RegX, BitX)	RegX &= ~_BV(BitX)	// Clear Bit Instruction
#endif

//debugging-LED output
#define LEDOUT(x)		(char)~x
#define ArrayCnt(x)		sizeof(x)/sizeof(x[0])
#define PREF_MTN_SIZE	20
#define MYSET_SIZE		8
#define SPD_FTR_SIZE	15
#define LOG_SIZE		50
#define BRT_TERMS		11

#define Num_My_Crmny	1
#define Num_My_eFx		2
#define Num_My_Sleep	3
#define Num_My_Rvs		4
#define Num_My_High		5
#define Num_My_Low		6
#define Num_My_Tempo	7
#define Num_My_Pow		8
#define GET_IX(x)		x-1

//functions of PWM value
//OCRn : more low signal(Bright) 0 ~ 255 more high signal(Dark)
#define getPWM		OCR2
#define setPWM(x)	OCR2=x
#define addPWM(x)	OCR2+=x
#define darkPWM(Src,Percent)	Src+(255-Src)*Percent*0.01
#define brightPWM(Src,Percent)	Src*Percent*0.01

typedef struct { char k; char v; } cKeyVal;

//rolling animation
const char gSegRoll[] = {
	0b01000000,
	0b00100000,
	0b00010000,
	0b00000100,
	0b00000010,
	0b00000001
};

char *gPrefMtn;
cKeyVal *gSnrMap; //sensor map
volatile unsigned int g1khz = 0;
volatile char gItr = 0, gShtdw = 0;
unsigned int gDlyMS = 0; //delay MS
unsigned char
	gArCnt[3],
	gMySet[MYSET_SIZE],
	gDspStp, //display steps
	gTmr0 = 0, //timer0 start value
	gCrmnyStp = 1; //ceremony steps
char
	gSegCA = 0, //segment common anode
	gSnrUp,
	gSnrDw,
	gPwmCm,
	gPrefMd = 0, //setting mode; 0=no setting, 1=into setting mode, 2=in setting mode
	gNight = 0,
	gUseLED = 0,
	gLog = 0,
	gLogStr[LOG_SIZE+1] = {0,}, //Log String
	gMtnHis[PREF_MTN_SIZE]={0,},
	int0_f=0,
	PwDnCnt=0;
volatile uint8_t *gSnrP, *gLedPt, *gSegPt, *gGPt;
unsigned long gSpdFtr[SPD_FTR_SIZE];
const unsigned long gSpdFtr1[SPD_FTR_SIZE]={20,50,100,150,200,300,400,500,600,1000,2000,2500,3000,5000,20000};

void getMySet(void);
void setTmr0(void);
void setTmr2(void);
char crmny(void);


#ifdef LIBDEBUG
/*
 * UART
 */
static int putchar0(char c, FILE *stream);	// 1 char Transmit
static FILE mystdout = FDEV_SETUP_STREAM(putchar0, NULL, _FDEV_SETUP_WRITE);
int putchar0(char c, FILE *stream)
{
	if (c == '\n') putchar0('\r', stream);
	while(!(UCSR0A & 0x20)); // UCSR0A 5bit = UDRE
	UDR0 = c; // 1 character trans
	return 0;
}
void init_usart()
{
	UBRR0H = 0;		// 12bit
	UBRR0L = 8;		// Look ATmega128 datasheet. 16Mhz, 115200 baud
	UCSR0B = 0x18;	// Receive(RX) and Transmit(TX) Enable
	UCSR0C = 0x06;	// UART Mode, 8 Bit Data, No Parity, 1 Stop Bit
}
#endif


//timer0 overflow interrupt
ISR(TIMER0_OVF_vect)
{
	TCNT0 = gTmr0; //back to init value
	g1khz++;
	if(g1khz >= gDlyMS){
		g1khz = 0;
		gItr = 1;
	}
}

//INT0 interrupt
ISR(INT0_vect)
{
	#ifdef LIBDEBUG
		printf("ISR(INT0) int0_f=%d\n", int0_f);
	#endif
	cli();
	if(bit_is_set(EIMSK,INT1)){
		cbi(EIMSK,INT1);
		EICRA &= ~(_BV(ISC10)|_BV(ISC11));
	}

	if(int0_f == 1){
		//set to 11 (Rising edge)
		EICRA |= _BV(ISC01) | _BV(ISC00);//EICRA = 0b00000011;
		int0_f = 2;
	}else{
		//set to 10 (Falling edge)
		EICRA = ((EICRA & ~(_BV(ISC00) | _BV(ISC01))) | _BV(ISC01));//EICRA = 0b00000010;
		int0_f = 1;
		PwDnCnt = 0;
	}
	sei();
}

//segment output
char segOut(char x, char ret)
{
	char r;
	if(gSegCA == 1) r = ~x;
	else r = x;

	if(ret==0) *gSegPt = r;

	return r;
}

char getYN(char x)
{
	if(x==0) return 'N';
	else return 'Y';
}

char chkMySet(void)
{
	if( gMySet[GET_IX(Num_My_Crmny)]>2 ) return Num_My_Crmny;
	if( gMySet[GET_IX(Num_My_eFx)]>2 ) return Num_My_eFx;
	if( gMySet[GET_IX(Num_My_Sleep)]>3 ) return Num_My_Sleep;
	if( gMySet[GET_IX(Num_My_Rvs)]>1 ) return Num_My_Rvs;
	if( gMySet[GET_IX(Num_My_High)]>242 ) return Num_My_High; //0~242 avail
	if( gMySet[GET_IX(Num_My_Low)]<40 || gMySet[GET_IX(Num_My_Low)]>254 ) return Num_My_Low; //40~254 avail
	if( gMySet[GET_IX(Num_My_Tempo)]<1 || gMySet[GET_IX(Num_My_Tempo)]>9 ) return Num_My_Tempo;
	return 0;
}

//convert timing
void convTmg(char v_SpdFtr){
	if( v_SpdFtr > 5 ){
		for(int i=0; i<SPD_FTR_SIZE; i++){
			gSpdFtr[i] = 1 + gSpdFtr1[i] / (1.5*(v_SpdFtr-5));
		}
	} else {
		for(int i=0; i<SPD_FTR_SIZE; i++){
			gSpdFtr[i] = gSpdFtr1[i] * 5 / v_SpdFtr;
		}
	}
}

//delay at start up
char ready(unsigned long v_F_CPU, unsigned int ms)
{
	if( gTmr0 == 0 ){
		#ifdef LIBDEBUG
			init_usart(); stdout = &mystdout;
			printf("ready(%lu, %u)\n",v_F_CPU,ms);
		#endif

		gDlyMS = ms;
		if(!v_F_CPU) v_F_CPU = 16000000UL;
		gDspStp = (unsigned char)((v_F_CPU/16000000)*10+1);
		gTmr0 = (unsigned char)(256-(0.001*(v_F_CPU)/128)); //counting 1kHz = 1ms @ 128 prescale

		setTmr0();
		return 1;
	}

	if(gItr == 1){
		cli(); //disable all interrupts
		gItr = 0;
		gDlyMS = 0;
		return 0;
	}
	return 1;
}

char initDev(
	char v_SegCA,
	char v_SegTr,
	cKeyVal *v_SnrMap,
	signed char v_SnrMapCnt,
	char v_SnrUp,
	char v_SnrDw,
	char *v_PrefMtn,
	char *v_MySets,

	volatile uint8_t *v_DDR_SNR,
	volatile uint8_t *v_PORT_SNR,
	volatile uint8_t *v_PIN_SNR,

	volatile uint8_t *v_DDR_LED,
	volatile uint8_t *v_PORT_LED,

	volatile uint8_t *v_DDR_SEG,
	volatile uint8_t *v_PORT_SEG,

	volatile uint8_t *v_DDR_PWM,
	volatile uint8_t *v_PORT_PWM,
	char v_PIN_PWM,

	volatile uint8_t *v_DDR_INT,
	char v_PIN_INT0,
	char v_PIN_INT1,

	char v_unuse,
	char v_useLED,
	char v_log
)
{
	if(v_log == 1) gLog = 1;

	if(v_SegCA>1){
		#ifdef LIBDEBUG
			puts("SEG_COM_ANODE is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] SEG_COM_ANODE is out of range");
		return 0;
	}
	if(v_SegCA == 1) gSegCA = v_SegCA;

	if(v_SegTr>1){
		#ifdef LIBDEBUG
			puts("SEG_USE_TR is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] SEG_USE_TR is out of range");
		return 0;
	}

	if(v_SnrMapCnt<0 || v_SnrMapCnt>100){
		#ifdef LIBDEBUG
			puts("SENSOR_MAP Size is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] SENSOR_MAP Size is out of range");
		return 0;
	}

	if(v_SnrUp<0 || v_SnrUp>100){
		#ifdef LIBDEBUG
			puts("SENSOR_UP is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] SENSOR_UP is out of range");
		return 0;
	}

	if(v_SnrDw<0 || v_SnrDw>100){
		#ifdef LIBDEBUG
			puts("SENSOR_DW is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] SENSOR_DW is out of range");
		return 0;
	}

	//get arrays count
	gArCnt[0] = v_SnrMapCnt;
	gArCnt[1] = strlen(v_PrefMtn);
	gArCnt[2] = ArrayCnt(gSegRoll);
	if(gArCnt[1]>PREF_MTN_SIZE){
		#ifdef LIBDEBUG
			puts("PREF_MOTN is out of range");
		#endif
		if(gLog == 1) sprintf(gLogStr,"[E] Max length of PREF_MOTN is %d",PREF_MTN_SIZE);
		return 0;
	}

	gPrefMtn = v_PrefMtn;

	gSnrMap = v_SnrMap;

	gSnrUp = v_SnrUp+'0';
	gSnrDw = v_SnrDw+'0';

	if(v_MySets[6] < 1) v_MySets[6] = 1;
	if(v_MySets[6] > 9) v_MySets[6] = 9;

	gMySet[0] = v_MySets[0]; //My_Crmny
	gMySet[1] = v_MySets[1]; //My_eFx
	gMySet[2] = v_MySets[2]; //My_Sleep
	gMySet[3] = v_MySets[3]; //My_Rvs
	gMySet[4] = (unsigned char)(255*(100-v_MySets[4])/100); //My_High convert to 0~255
	gMySet[5] = (unsigned char)(255*(100-v_MySets[5])/100); //My_Low convert to 0~255
	gMySet[6] = v_MySets[6]; //My_Tempo
	gMySet[7] = 1; //dummy for power-off

	//check My Setting Value
	char ret_chkMySet = chkMySet();
	if(ret_chkMySet>0){
		const char *prefName[] = { "My_Crmny", "My_eFx", "My_Sleep", "My_Rvs", "My_High", "My_Low", "My_Tempo" };
		#ifdef LIBDEBUG
			printf("%s is out of range\n", prefName[GET_IX(ret_chkMySet)]);
		#endif
		if(gLog == 1) sprintf(gLogStr,"%s is out of range", prefName[GET_IX(ret_chkMySet)]);
		return 0;
	}

	if(v_unuse>1){
		#ifdef LIBDEBUG
			puts("UNUSED_STY is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] UNUSED_STY is out of range");
		return 0;
	}

	if(v_useLED>1){
		#ifdef LIBDEBUG
			puts("USE_LED is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] USE_LED is out of range");
		return 0;
	}

	if(v_log>1){
		#ifdef LIBDEBUG
			puts("UART_LOG is out of range");
		#endif
		if(gLog == 1) strcpy(gLogStr,"[E] UART_LOG is out of range");
		return 0;
	}

	getMySet();
	convTmg(gMySet[GET_IX(Num_My_Tempo)]);

	//set PUD(Pull Up Disable) to 0
	cbi(SFIOR, PUD);

	//set sensor
	gSnrP = v_PIN_SNR;
	*v_DDR_SNR = 0x00;
	*v_PORT_SNR = 0xFF; //Internal pull-up enable

	//set debugging LED
	gLedPt = v_PORT_LED;
	gUseLED = v_useLED;
	if( v_useLED == 0 ){
		if( v_unuse == 0 ){
			//input with pull-up
			*v_DDR_LED = 0x00;
			*v_PORT_LED = 0xFF;
		}else if( v_unuse == 1 ){
			//output with nothing
			*v_DDR_LED = 0xFF;
			*v_PORT_LED = 0xFF;
		}
	} else {
		*v_DDR_LED = 0xFF;
		*v_PORT_LED = LEDOUT(0);
	}

	//set 7segment
	gSegPt = v_PORT_SEG;
	*v_DDR_SEG = 0xFF;
	segOut(0,0);

	//PWM
	sbi(*v_DDR_PWM, v_PIN_PWM);
	if(v_SegCA == 1 && v_SegTr == 0){
		sbi(*v_PORT_PWM, v_PIN_PWM); //set PIN_PWM = 1
		gPwmCm = 3; //inverting mode(0b11)
	}else{
		cbi(*v_PORT_PWM, v_PIN_PWM); //set PIN_PWM = 0
		gPwmCm = 2; //non-inverting mode(0b10)
	}

	//extern interrupt
	*v_DDR_INT &= ~(_BV(v_PIN_INT0)|_BV(v_PIN_INT1));

	#ifdef LIBDEBUG
		printf("initDev()\n Ver"GEARGAUGE_VER"\n Steps=%d\n Timer0=%d\n- My Setting :\n  Ceremony=%d\n  Changing Effect=%d\n  Sleep Effect=%d\n  Reverse Effect=%d\n  Bright Normal=%d\n  Bright Night=%d\n  Timing Speed=%d\n OK\n"
		,gDspStp
		,gTmr0
		,gMySet[GET_IX(Num_My_Crmny)]
		,gMySet[GET_IX(Num_My_eFx)]
		,gMySet[GET_IX(Num_My_Sleep)]
		,gMySet[GET_IX(Num_My_Rvs)]
		,gMySet[GET_IX(Num_My_High)]
		,gMySet[GET_IX(Num_My_Low)]
		,gMySet[GET_IX(Num_My_Tempo)]);
	#endif
	if(gLog == 1){
		sprintf(gLogStr, "initDev()\n Ver"GEARGAUGE_VER"\n Steps=%d\n Timer0=%d\n OK", gDspStp, gTmr0);
	}

	//enable INT1
	EICRA = _BV(ISC10)|_BV(ISC11);//EICRA = 0b00001100;
	EIFR = _BV(INTF1);//EIFR = 0b00000010;
	EIMSK = _BV(INT1);
	sei();

	//Welcome Ceremony
	if( gMySet[GET_IX(Num_My_Crmny)] == 0 ){
		gCrmnyStp = 0; //no ceremony
		gDlyMS = 100;
	}
	while(crmny()); //Welcome Ceremony

	setTmr2(); //set Fast PWM timer
	return 1;
}

void setTmr0(void)
{
	#ifdef LIBDEBUG
		puts("setTmr0()");
	#endif
	cli(); //disable all interrupts

	TCCR0 = _BV(CS02)|_BV(CS00); //normal mode, prescale:128

	//set timer0 mask to overflow interrupt (??????01)
	sbi(TIMSK,TOIE0);
	cbi(TIMSK,OCIE0);

	TCNT0 = gTmr0; //timer starting value

	sei(); //enable interrupts
}

void setTmr2(void)
{
	#ifdef LIBDEBUG
		puts("setTmr2()");
	#endif
	TCCR2 = (gPwmCm<<COM20)|_BV(WGM20)|_BV(WGM21)|_BV(CS22)|_BV(CS20); //Fast PWM, prescale:1024

	//set timer2 mask to N/A (00??????)
	TIMSK &= ~(_BV(OCIE2)|_BV(TOIE2));

	TCNT2 = 0;
	setPWM(0);

	//watchdog timer
	cbi(MCUCSR, WDRF);
	wdt_enable(WDTO_2S);

	//INT0 falling edge
	cli(); //disable all interrupts
	int0_f = 1;
	EICRA = _BV(ISC01);//EICRA = 0b00000010;
	EIMSK = _BV(INT0);//EIMSK = 0b00000001;
	EIFR = _BV(INTF0);//EIFR = 0b00000001;
	sei(); //enable interrupts
}

char crmny(void)
{
	if( gCrmnyStp == 0 ) return 0;
	if( gItr != 1 ) return 1;
	gItr = 0;

	if( gMySet[GET_IX(Num_My_Crmny)] == 1 ){ //all light
		if( gCrmnyStp == 1 ){
			gCrmnyStp++;
			gDlyMS = 1000;

			segOut(SEG_ALL,0);
			if(gUseLED == 1) *gLedPt = LEDOUT(0xFF);

			#ifdef LIBDEBUG
			puts("Start crmny()");
			#endif
		} else {
			//finish ceremony
			gCrmnyStp = 0;
			gDlyMS = 100;

			segOut(0,0);
		}
	} else if( gMySet[GET_IX(Num_My_Crmny)] == 2 ){ //animation
		if( gCrmnyStp == 1 ){
			//Segment & LED step
			char segTmp = *gSegPt;
			if( gDlyMS != 100 ){
				gDlyMS = 100;
				segTmp = 1;
				if(gUseLED == 1) *gLedPt = LEDOUT(1);

				#ifdef LIBDEBUG
				puts("Start crmny()");
				#endif
			} else {
				segTmp = segOut(segTmp,1) | segOut(segTmp,1)<<1;
				if(gUseLED == 1){
					*gLedPt = *gLedPt << 1 | 1;//only 1 LED
					//*gLedPt = *gLedPt << 1;//from 1 to all
				}
			}

			segOut(segTmp,0);

			if( segTmp == SEG_ALL ){
				gDlyMS = 1000;
				gCrmnyStp++;
			}

		} else {
			//Segment Rolling
			if( gDlyMS != 40 ){
				gDlyMS = 40;
				if(gUseLED == 1) *gLedPt = LEDOUT(0);
			}

			if( gCrmnyStp > (gArCnt[2]*2)+1 ){ //2 turns
				//finish ceremony
				gCrmnyStp = 0;
				gDlyMS = 100;

				segOut(0,0);
			} else {
				segOut( gSegRoll[(gCrmnyStp-1)%gArCnt[2]], 0 );
				gCrmnyStp++;
			}
		}
	}
	return gCrmnyStp;
}

/**
 * get Settings from eeprom
 */
void getMySet(void)
{
	#ifdef LIBDEBUG
		puts("getMySet()");
	#endif
	unsigned int i;
	if( eeprom_read_byte( (const uint8_t*)0 ) == 1 )
	{
		for(i=0; i<MYSET_SIZE-1; i++)
			gMySet[i] = eeprom_read_byte( (const uint8_t*)i+1 );
	} else {
		eeprom_update_byte((uint8_t*)0, 1);
		for(i=0; i<MYSET_SIZE-1; i++)
			eeprom_update_byte((uint8_t*)i+1, gMySet[i]);
	}
}

/**
 * set Settings to eeprom
 */
char setMySet( unsigned int b, unsigned char v )
{
	if( b < 1 || b > MYSET_SIZE ) return 0;
	if( v != SEG_UP && v != SEG_DOWN ) return 0;

	b--;
	unsigned char nV = gMySet[b];

	if( v == SEG_UP )
		nV++;
	else
		if( nV > 0 ) nV--;

	if( b == GET_IX(Num_My_High) || b == GET_IX(Num_My_Low) ) //My_High, My_Low
	{
		const unsigned char tm[BRT_TERMS] = {0,40,80,120,160,200,220,240,250,252,254};
		char ixTm = GET_IX(BRT_TERMS);
		for(int i=0; i<ixTm; i++)
		{
			if( gMySet[b] == tm[i] ) { ixTm=i;break; }
			if( gMySet[b] < tm[i] )
			{
				if( (gMySet[b]-tm[i-1]) < (tm[i]-gMySet[b]) ) ixTm = i-1;
				else ixTm = i;

				break;
			}
		}
		if( b == GET_IX(Num_My_High) ) //My_High. 0~240 avail
		{
			if( v == SEG_UP )
				nV = ixTm==0 ? 0 : tm[ixTm-1];
			else
				nV = (ixTm==BRT_TERMS-1 || tm[ixTm+1]>240) ? 240 : tm[ixTm+1];
		}
		if( b == GET_IX(Num_My_Low) ) //My_Low. 40~254 avail
		{
			if( v == SEG_UP )
				nV = (ixTm==0 || tm[ixTm-1]<40) ? 40 : tm[ixTm-1];
			else
				nV = (ixTm==BRT_TERMS-1 || tm[ixTm+1]>254) ? 254 : tm[ixTm+1];
		}
	}

	switch(b)
	{
		case GET_IX(Num_My_Crmny): if( nV > 2 ) nV = 2; break;
		case GET_IX(Num_My_eFx):   if( nV > 2 ) nV = 2; break;
		case GET_IX(Num_My_Sleep): if( nV > 3 ) nV = 3; break;
		case GET_IX(Num_My_Rvs):
		case GET_IX(Num_My_Pow):   if( nV > 1 ) nV = 1; break;
		case GET_IX(Num_My_Tempo):
			if( nV > 9 ) nV = 9;
			if( nV < 1 ) nV = 1;
			convTmg(nV);
		break;
		default: break;
	}
	gMySet[b] = nV;
	if( b < GET_IX(Num_My_Pow) ) eeprom_update_byte((uint8_t*)b+1, nV);
	#ifdef LIBDEBUG
		printf(gLogStr, "Save EEPROM : gMySet[%d]=%d", b, nV);
	#endif
	if(gLog == 1){
		const char *prefName[] = { "My_Crmny", "My_eFx", "My_Sleep", "My_Rvs", "My_High", "My_Low", "My_Tempo" };
		sprintf(gLogStr, "Save EEPROM : %s=%d", prefName[b], nV); //printf("Save EEPROM : Bank%d=%d\n", b+1, nV);
	}
	return 1;
}

/*
 * shifting motion history save
 */
char chkMtnHis( char m )
{
	if( m == 'B' ){
		gPrefMd = 0;
		gMtnHis[gArCnt[1]-1] = m;
		return m;
	} else if( gPrefMd > 0 ){
		if( gPrefMd == 1 ) return 'U';
		else if( m == gSnrUp ) return '+';
		else if( m == gSnrDw ) return '-';
		else if( m == 'L' || m == 'R' ) return m;
		else return '?';
	} else if( m == 'L' || m == 'R' ) return '?';

	if( m == '?' || gMtnHis[gArCnt[1]-1] == m ) return m;

	int i;

	if( m == 'N' ){
		for(i=0; i<gArCnt[1]; i++){
			if(gMtnHis[i] != gPrefMtn[i]) return m;
		}

		//user pref mode
		gPrefMd = 1;
		m = 'U';
	} else {
		//shift array
		for(i=1; i<gArCnt[1]; i++) gMtnHis[i-1] = gMtnHis[i];
		gMtnHis[gArCnt[1]-1] = m;
	}
	return m;
}

/*
 * get shift position
 * '?' = not clear yet
 */
char getPosVal( char s )
{
	char p = '?';
	int i;

	while(1){

		//check night dimming
		gNight = (s>>7 == 1) ? 1 : 0;
		s &= ~_BV(7); //clear last bit

		//CheckError - error if no input
		if( !(0b01111111 & s) ) {p='E'; break;}

		//check sensor group 1
		int c1 = 0;
		for( i=0;i<=3;i++ ) c1 += ( s & _BV(i) ) ? 1 : 0;
		if( c1 > 1 ) break; //when duplicated

		//check sensor group 2
		int c2 = 0;
		for( i=4;i<=6;i++ ) c2 += ( s & _BV(i) ) ? 1 : 0;
		if( c2 > 1 ) break; //when duplicated

		//find from sensor MAP
		if( c1==1 && c2==1 ){
			for( i=0; i<gArCnt[0]; i++ ){
				if( gSnrMap[i].k == s ){
					p = gSnrMap[i].v;
					break;
				}
			}
			if( p != 'E' ) p = chkMtnHis( p );
		}

		break;
	}

	/* gear position to segment output map
	* must NO DUPLICATE right side value
	*/
	const cKeyVal posSeg[] = {
		{ '?', SEG_MINUS },
		{ 'L', SEG_LEFT },
		{ 'R', SEG_RIGHT },
		{ 'N', SEG_NEUTRAL },
		{ '1', SEG_1 },
		{ '2', SEG_2 },
		{ '3', SEG_3 },
		{ '4', SEG_4 },
		{ '5', SEG_5 },
		{ '6', SEG_6 },
		{ 'B', SEG_BACK },
		{ '+', SEG_UP },
		{ '-', SEG_DOWN },
		{ 'U', SEG_U },
		{ '7', SEG_7 },
		{ '8', SEG_8 },
		{ '9', SEG_9 },
		{ 'E', SEG_ERROR }
	};
	const char arCnt = ArrayCnt(posSeg);
	char seg = SEG_ERROR; //Default to Error

	//find Segment value
	for( i=0; i<arCnt; i++ ){
		if( posSeg[i].k == p ){
			seg = posSeg[i].v;
			break;
		}
	}

	return seg;
}

/*
 * Output Segment Data
 * return 1 : normal
 * return 0 : power-off (all output off)
 */
char dspSeg( char seg )
{
	static unsigned char prev = 0, tmpPrev = 0, postFx = 0, curSetB = 0, SegCurSetB = 0, mySetGo = 0, brtCnt = 0, curSt,
		sBr = 0, //segment bright
		sBrPfV = 204, //segment bright when pref value
		brtBthLmt = 85; //segment bright limit when breathing effect
	static unsigned long tiCnt = 0; //timing count
	static unsigned int roTiCnt = 0, segFxIdx = 0;
	unsigned char tmp = seg;

	//all used elements in pref mode
	const cKeyVal prefSeg[] = {
		{ Num_My_Crmny, SEG_C },
		{ Num_My_eFx, SEG_F },
		{ Num_My_Sleep, SEG_S },
		{ Num_My_Rvs, SEG_BACK },
		{ Num_My_High, SEG_H },
		{ Num_My_Low, SEG_L },
		{ Num_My_Tempo, SEG_t },
		{ Num_My_Pow, SEG_P },
		{ '0', SEG_0 },
		{ '1', SEG_1 },
		{ '2', SEG_2 },
		{ '3', SEG_3 },
		{ '4', SEG_4 },
		{ '5', SEG_5 },
		{ '6', SEG_6 },
		{ '7', SEG_7 },
		{ '8', SEG_8 },
		{ '9', SEG_9 }
	};
	const char arPSCnt = ArrayCnt(prefSeg);

	tiCnt++;
	#ifdef LIBDEBUG
	if( tiCnt % 1400 == 0 ){
		puts("1s");
	}
	#endif

	if( !(gPrefMd == 2 && (curSetB == Num_My_High || curSetB == Num_My_Low)) ){
		if( gNight == 1 ){
			if( sBr != gMySet[GET_IX(Num_My_Low)] ){
				sBr = gMySet[GET_IX(Num_My_Low)];
				sBrPfV = darkPWM(sBr,80);//sBr+(255-sBr)*0.8;
				brtBthLmt = darkPWM(sBr,33);
				setPWM(sBr);
			}
		} else {
			if( sBr != gMySet[GET_IX(Num_My_High)] ){
				sBr = gMySet[GET_IX(Num_My_High)];
				sBrPfV = darkPWM(sBr,80);//sBr+(255-sBr)*0.8;
				brtBthLmt = darkPWM(sBr,33);
				setPWM(sBr);
			}
		}
	}

	//previous shift effect when neutral
	if(
		gPrefMd == 0
		&& gMySet[GET_IX(Num_My_eFx)] > 0
		&& prev != 0
		&& prev != SEG_MINUS
		&& prev != SEG_BACK
		&& prev != SEG_ERROR
		&& prev != seg
		&& ( seg == SEG_MINUS || seg == SEG_NEUTRAL )
	){
		if( postFx == 0 ){
			postFx = 1; //effect mode on
			tiCnt = 0;
			setPWM(sBr);
		}

		if( prev == SEG_NEUTRAL ){
			if( curSt == 'S' ){
				curSt = 'N';
				#ifdef LIBDEBUG
					puts("Wake Up");
				#endif
				if( gLog == 1 ) strcpy(gLogStr, "Wake Up");
			}
			segOut(prev,0);

		} else if( gMySet[GET_IX(Num_My_eFx)] == 1 ){ //fade out
			segOut(prev,0);
			if( sBr <= 210 ){
				//if( tiCnt % 50 == 0 ){
				if( tiCnt % gSpdFtr[1] == 0 ){
					if( getPWM < 210) addPWM(3);
					else if( getPWM < 254 ) addPWM(2);
				}
			} else {
				//if( tiCnt % 100 == 0 ){
				if( tiCnt % gSpdFtr[2] == 0 ){
					if( getPWM < 255) addPWM(1);
				}
			}

			//if( getPWM >= 254 && tiCnt > 2000 ){
			if( getPWM >= 254 && tiCnt > gSpdFtr[10] ){
				//finish effect mode
				setPWM(sBr);
				segOut(prev,0);
				prev = seg;
				postFx = 0;
			}
		} else if( gMySet[GET_IX(Num_My_eFx)] == 2 ){ //blinking
			//if( tiCnt < 5000 ){
			if( tiCnt < gSpdFtr[13] ){
				//if( tiCnt % 200 > 100 )
				if( tiCnt % gSpdFtr[4] > gSpdFtr[2] )
					segOut(0,0); else segOut(prev,0);
			} else {
				//finish effect mode
				segOut(prev,0);
				prev = seg;
				postFx = 0;
			}
		}

		return 1;

	} else if( gPrefMd == 1 ){
		if( postFx == 0 || prev != SEG_U ){
			curSt = 'R';//pref
			#ifdef LIBDEBUG
				puts("Into User Setting Mode");
			#endif
			if(gLog == 1) strcpy(gLogStr, "Into User Setting Mode");
			postFx = 1;
			tiCnt = 0;
			curSetB = 1;
			SegCurSetB = prefSeg[GET_IX(curSetB)].v;
			prev = seg;
			setPWM(sBr);
		}

		//into setting mode signal
		//if( tiCnt < 3000 ){
		if( tiCnt < gSpdFtr1[12] ){
			//if( tiCnt % 1000 > 400 )
			if( tiCnt % gSpdFtr1[9] > gSpdFtr1[6] )
				segOut(SEG_U,0); else segOut(0,0);
		} else {
			segOut(seg,0);
			postFx = 0;
			tiCnt = 0;
			gPrefMd = 2;
		}
		return 1;
	}
	else if( gPrefMd == 2 && (postFx == 1 || prev == SEG_UP || prev == SEG_DOWN || prev == SEG_ERROR) )
	{
		//setting just changed
		if( postFx == 0 ){
			postFx = 1;
			tiCnt = 0;
			prev = seg;
			if( mySetGo == 0 ){
				mySetGo = 1;
				if( setMySet( curSetB, seg ) == 1 ){
					tmpPrev = SEG_ERROR;

					//power off
					if( curSetB == Num_My_Pow && (prev == SEG_UP || prev == SEG_DOWN) ){
						segOut(0,0);
						return 0;
					}

					if( curSetB == Num_My_High || curSetB == Num_My_Low ){
						tmpPrev = prefSeg[GET_IX(curSetB)].v;
					} else {
						//find current setting value
						for(int i=0;i<arPSCnt;i++)
							if( prefSeg[i].k == gMySet[GET_IX(curSetB)]+'0' ){ tmpPrev = prefSeg[i].v; break; }
					}
				} else {
					prev = SEG_ERROR;
					seg = SEG_ERROR;
					tmpPrev = SEG_ERROR;
				}
			}
		}

		//setting change effect
		//if( mySetGo == 1 || tiCnt < 1000 ){
		if( mySetGo == 1 || tiCnt < gSpdFtr1[9] ){
			//if( curSetB != Num_My_High && curSetB != Num_My_Low && tiCnt < 1000 ){
			if( curSetB != Num_My_High && curSetB != Num_My_Low && tiCnt < gSpdFtr1[9] ){
				//dot blink
				//if( tiCnt % 300 < 150 )
				if( tiCnt % gSpdFtr1[5] < gSpdFtr1[3] )
					tmpPrev |= SEG_DOT; else tmpPrev &= ~SEG_DOT;
				segOut(tmpPrev,0);
			} else {
				if(curSetB==Num_My_High || curSetB==Num_My_Low) segOut(prefSeg[GET_IX(curSetB)].v,0); else segOut(seg,0);
				postFx = 0;
				tiCnt = 0;
			}
		} else {
			segOut(SegCurSetB,0);
			postFx = 0;
		}
		return 1;
	} else {
		if( postFx == 1 ){
			postFx = 0;
			tiCnt = 0;
		}
	}

	//just changed
	if( prev != seg ){
		if(gLog == 1 && gPrefMd == 0){
			if(curSt == 'S') strcpy(gLogStr,"Wake Up");
			if(seg != SEG_NEUTRAL && seg != SEG_MINUS){
				if(curSt == 'S') strcat(gLogStr,"\nGear="); else strcpy(gLogStr,"Gear=");
				switch(seg){
					case SEG_ERROR: strcat(gLogStr,"E"); break;
					case SEG_BACK: strcat(gLogStr,"R"); break;
					case SEG_1: strcat(gLogStr,"1"); break;
					case SEG_2: strcat(gLogStr,"2"); break;
					case SEG_3: strcat(gLogStr,"3"); break;
					case SEG_4: strcat(gLogStr,"4"); break;
					case SEG_5: strcat(gLogStr,"5"); break;
					case SEG_6: strcat(gLogStr,"6"); break;
					case SEG_7: strcat(gLogStr,"7"); break;
					case SEG_8: strcat(gLogStr,"8"); break;
					case SEG_9: strcat(gLogStr,"9"); break;
				}
				gLogStr[LOG_SIZE]=0;
			}
		}
		#ifdef LIBDEBUG
		if(gPrefMd == 0){
			if(curSt == 'S') puts("Wake Up");
			if(seg != SEG_NEUTRAL && seg != SEG_MINUS){
				printf("Gear=");
				switch(seg){
					case SEG_ERROR: puts("E"); break;
					case SEG_BACK: puts("R"); break;
					case SEG_1: puts("1"); break;
					case SEG_2: puts("2"); break;
					case SEG_3: puts("3"); break;
					case SEG_4: puts("4"); break;
					case SEG_5: puts("5"); break;
					case SEG_6: puts("6"); break;
					case SEG_7: puts("7"); break;
					case SEG_8: puts("8"); break;
					case SEG_9: puts("9"); break;
				}
			}
		}
		#endif

		curSt = 'N';//normal
		tmp = seg;
		prev = seg;
		roTiCnt = 0;
		brtCnt = 0;
		segFxIdx = 0;
		mySetGo = 0;
		setPWM(sBr);

		if( gPrefMd == 0 && seg == SEG_MINUS ) tmp = SEG_NEUTRAL;

		if( seg == SEG_ERROR && gPrefMd > 0 ){
			gPrefMd = 0;
			gMtnHis[GET_IX(gArCnt[1])] = 'B';
		}

		if( gPrefMd == 2 ){
			if( seg == SEG_LEFT || seg == SEG_RIGHT ){
				if( seg == SEG_LEFT ){
					if( --curSetB < 1 ) curSetB = 1;
				} else if( seg == SEG_RIGHT ){
					if( ++curSetB > MYSET_SIZE ) curSetB = MYSET_SIZE;
				}
				for(int i=0;i<arPSCnt;i++)
					if( prefSeg[i].k == curSetB ){ SegCurSetB = prefSeg[i].v; break; }

				tmp = SegCurSetB;
				if( curSetB == Num_My_High || curSetB == Num_My_Low ){
					sBr = gMySet[GET_IX(curSetB)];
				}
			}else{
				if( seg == SEG_ERROR ) segOut(tmp,0);
				return 1;
			}
		}

		tiCnt = 0;
		segOut(tmp,0);

	} else { //cur shift effect
		if( segFxIdx == 0 && getPWM != sBr ) setPWM(sBr);

		if( gPrefMd == 2 ){
			tmp = seg;
			if( seg == SEG_MINUS || seg == SEG_LEFT || seg == SEG_RIGHT ) tmp = SegCurSetB;

			//blink title or value
			if( curSetB != Num_My_High && curSetB != Num_My_Low && curSetB != Num_My_Pow ){
				if(segFxIdx != 1) segFxIdx = 1;

				if( curSetB == Num_My_Tempo ){
					//if( tiCnt % 2000 > 1000 ){
					if( tiCnt % gSpdFtr[10] > gSpdFtr[9] ){ //adjusted timing
						if( getPWM != sBrPfV ) setPWM(sBrPfV);
						for(int i=0;i<arPSCnt;i++)
							if( prefSeg[i].k == gMySet[GET_IX(curSetB)]+'0' ){ tmp = prefSeg[i].v; break; }
					} else {
						if( getPWM != sBr ) setPWM(sBr);
					}
				}else{
					//if( tiCnt % 2000 > 1000 ){
					if( tiCnt % gSpdFtr1[10] > gSpdFtr1[9] ){ //original timing
						if( getPWM != sBrPfV ) setPWM(sBrPfV);
						for(int i=0;i<arPSCnt;i++)
							if( prefSeg[i].k == gMySet[GET_IX(curSetB)]+'0' ){ tmp = prefSeg[i].v; break; }
					} else {
						if( getPWM != sBr ) setPWM(sBr);
					}
				}
			}

			segOut(tmp,0);

		} else if( seg == SEG_BACK ){
			//reverse shift blink
			if( gMySet[GET_IX(Num_My_Rvs)] == 0 ){
				//tmp = ( tiCnt % 1000 < 500 ) ? seg : 0;
				tmp = ( tiCnt % gSpdFtr[9] < gSpdFtr[7] ) ? seg : 0;
			} else {
				//tmp = ( tiCnt % 600 < 300 ) ? seg|SEG_DOT : seg;
				tmp = ( tiCnt % gSpdFtr[8] < gSpdFtr[5] ) ? seg|SEG_DOT : seg;
			}
			segOut(tmp,0);

		//} else if( seg == SEG_NEUTRAL && tiCnt > 20000 ){ //sleep animation
		} else if( seg == SEG_NEUTRAL && tiCnt > gSpdFtr[14] ){ //sleep animation
			if(curSt != 'S'){
				#ifdef LIBDEBUG
				puts("Sleep");
				#endif
				if(gLog == 1) strcpy(gLogStr, "Sleep");
			}
			curSt = 'S';//sleep
			//rolling
			if( gMySet[GET_IX(Num_My_Sleep)] == 1 ){
				//rolling every 1sec
				//if( ++roTiCnt > 2500 ){
				if( ++roTiCnt > gSpdFtr[11] ){
					segOut(gSegRoll[segFxIdx],0);
					if ( ++segFxIdx >= gArCnt[2] ) segFxIdx = 0;

					roTiCnt = 0;
				}
			}
			//breathing
			else if( gMySet[GET_IX(Num_My_Sleep)] == 2 ){
				if( segFxIdx == 0 ) segFxIdx = 1;

				//if( ++roTiCnt > 100 ){
				if( ++roTiCnt > gSpdFtr[2] ){
					if( segFxIdx % 2 == 1 ){
						if( getPWM < 254 ) addPWM(2); //more dark
					} else {
						brtCnt++;
						if( getPWM > brtBthLmt && getPWM > 210 ) addPWM(-1); //more bright
					}

					roTiCnt = 0;
				}
				if(
					(segFxIdx%2 == 1 && getPWM >= 254)
					|| (segFxIdx%2 == 0 && brtCnt >= 60)
				){
					segFxIdx++; //direction change
					brtCnt = 0;
				}
			}
			//segment off
			else if( gMySet[GET_IX(Num_My_Sleep)] == 3 ){
				if( segFxIdx != 1 ){
					segFxIdx = 1;
					setPWM(255);
				}
			}

		} else if( gMySet[GET_IX(Num_My_eFx)] == 1 && seg != SEG_NEUTRAL ){
			if( seg == SEG_MINUS || seg == SEG_LEFT || seg == SEG_RIGHT ){
				seg = SEG_NEUTRAL;
				prev = SEG_NEUTRAL;
			}
			//cur shift blink
			//if ( tiCnt < 1000 ){
			if ( tiCnt < gSpdFtr[9] ){
				//if( tiCnt % 200 > 50 )
				if( tiCnt % gSpdFtr[4] > gSpdFtr[1] )
					segOut(0,0); else segOut(seg,0);
			} else
				segOut(seg,0);

		} else {
			if( seg == SEG_MINUS || seg == SEG_LEFT || seg == SEG_RIGHT ) segOut(SEG_NEUTRAL,0);
			else segOut(seg,0);
		}
	}
	return 1;
}

void goShtDw(void)
{
	const cKeyVal byeSeg[] = {
		{ 1, SEG_b },
		{ 2, SEG_y },
		{ 3, SEG_E },
		{ 4, 0 }
	};
	char segDat = SEG_DOT; //Default

	for( int i=0; i<ArrayCnt(byeSeg); i++ ){
		if ( byeSeg[i].k == gShtdw ){
			segDat = byeSeg[i].v;
			break;
		}
	}
	segOut(segDat,0);

	if(gShtdw==1){
		#ifdef LIBDEBUG
		puts("Start goShtDw()");
		#endif
		if(gUseLED == 1) *gLedPt = LEDOUT(0);
		setPWM(0); //light max

		//disable watchdog
		wdt_reset();
		cbi(MCUCSR,WDRF);
		wdt_disable();

		cli();

		//disable INT0, enable INT1
		EICRA = _BV(ISC10)|_BV(ISC11);//EICRA = 0b00001100;
		EIFR = _BV(INTF1);//EIFR = 0b00000010;
		EIMSK = _BV(INT1);

		sei();
	}else if(gShtdw>=4){
		gShtdw=0;
		setPWM(255); //light out

		cli(); //disable all interrupts

		//timer stop
		TIMSK = 0x00;
		TCCR0 = 0x00;
		TCCR2 = 0x00;

		sei();

		//MCU Sleep : set to 0b??1100??
		//MCUCR = ((MCUCR & ~(_BV(SE) | _BV(SM0) | _BV(SM1) | _BV(SM2))) | (_BV(SE) | _BV(SM1))); asm("sleep");
		set_sleep_mode(SLEEP_MODE_PWR_DOWN); sleep_mode();

		return;
	}
	gShtdw++;
}

char chkShtDw(void)
{
	if(gShtdw == 0) return 0;
	if(gItr == 1){ //every 500ms
		gItr = 0;
		goShtDw();
	}
	return 1;
}

char gauge(void)
{
	static char curPos = 0, ins = 0;
	if ( gItr == 1 ) //every 100ms
	{
		gItr = 0;
		wdt_reset();

		if(int0_f == 2){
			if(++PwDnCnt>30){ //3sec over
				PwDnCnt=0;
				gItr = 0;
				gDlyMS = 500;
				gShtdw = 1;
				while(chkShtDw());
				return 0;
			}
		}

		ins = ~*gSnrP; //sensor input value. 1=Off 0=On

		if(gUseLED == 1) *gLedPt = LEDOUT( ins ); //display LED
		curPos = getPosVal( ins );
	}

	if( g1khz % gDspStp == 0 )
	{
		if( dspSeg( curPos ) != 1 )
		{
			gItr = 0;
			gDlyMS = 500;
			gShtdw = 1;
			while(chkShtDw());
			return 0;
		}
	}
	return 1;
}
