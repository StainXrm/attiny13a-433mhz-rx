#include <avr/io.h> //http://ww1.microchip.com/downloads/en/DeviceDoc/doc8126.pdf
#include <avr/interrupt.h>

//What a single RX should look like : (first low should still look noisy)
//    ____      ____            _ _ _
//   |    |    |    |				 |
//___|    |____|    |____ _ _ |      _   _      _
//   |<SP>|<SP>|<SP>|<NA>|< Channel >|<    Databits     >|< END is low for some time >
//   |< 1>|< 0>|< 1>|< 0>|< X X X X >|< X X X X X X X X >| 0 0 0 0

//So after calling Init() it searches for data if found rx gets set to -1;
//If rx is set to -1 you get the actual data from rx_packet variable
//Start new search by calling new Init();

#define RXPIN PINB1 //INT0 PIN!
#define LEDPIN PINB4

#define pulselength 10 //ms
#define nRepeatsExpected 4 //how often should we expect packet in repeat?
#define expectedBits 12 //INDEXED bit 12 = syncconfirm (should be 0) > bit 11-8 = channel > bit 7-0 = data
#define channel 3 //what channel to listen to?

volatile uint16_t millisecs; //ms counter
volatile int8_t rx = 0; //this var is for stage of receiving
volatile uint8_t i_sector; //sectorcount >indexed<
volatile uint16_t rx_packet_raw; //var to set raw packet
volatile int8_t ibit = expectedBits;
volatile uint8_t packet[nRepeatsExpected]; //packets received in here
volatile int8_t packetcount;
volatile uint8_t rx_packet;

void Init();//yeah function to start or start all over
void sectorSetBit();//is for variable voting in each section and setting president in raw bit
void dataCheck();

int main()
{
	OSCCAL = 86; //internal osci callibrate
	TCCR0B |= (1<<CS01); //timer prescale 8
	TCCR0A |= (1<<WGM01); //clear on compare match
	OCR0A = 149; //compare match for timer (9,6mhz/8 = 1,2mhz/8 = 150.000hz/150 = 1.000 interrups per sec)
	DDRB |= (1<<LEDPIN); //output to LED for debugging
   
    /*start timer*/
	TIFR0 |= (1<<OCF0A); //interrupt on timer
    TIMSK0 |= (1<<OCIE0A);
	
	/*pinchange interrupt enable on INT0 */
	MCUCR |= (1<<ISC00); //generate Interrupt on any Change here!
	GIMSK |= (1<<INT0); //interrupt enable on INT0
    sei();//interrupts on!
    while (1) 
    {
		
			//if(rx == -1){
				//(rx_packet%2) ? PORTB |= (1<<LEDPIN) : PORTB &= ~(1<<LEDPIN);
				//Init();
			//}
		
    }
}

void sectorSetBit() { // check a timed bitlength if high or low on voting system
	static int8_t bitval = 0; 
	static uint8_t currentsector = 0; //the sector we are currently on! 
	(PINB & (1<<RXPIN)) ? bitval++ : bitval--; //add vote for current sector!
	if(currentsector != i_sector) { //sector changed check what we got and start all over!
		bitval = (bitval>0) ? 1 : 0 ; //so did we get an high or low on last sector?
		(bitval) ? rx_packet_raw |= (1<<ibit) : rx_packet_raw &= ~(1<<ibit); //set bit as low or high!
		bitval = 0; // reset it for next check bit!
		ibit--;//next bit!
		//all bits set... so evaluate and reset!
		if(ibit < 0) {
			ibit = expectedBits;
			Init();
			//dataCheck();
		}
		currentsector = i_sector; //set for new sector check!
	}
}

void dataCheck(){
	//fist check if we got the right channel:
	uint8_t chanchk = 0;
	int8_t l = 3; //index counter....indexed 4bit channel
	for (int8_t i = expectedBits-1; i >= expectedBits-4;i--) //we only need the channel bits here! (we also skip the first bit after sync)
	{
		(rx_packet_raw & (1<<i)) ? chanchk |= (1<<l) : chanchk &= ~(1<<l);
		l--;
	}
	
	//if its the correct channel we go on with data:
	uint8_t data = 0;
	if(channel == chanchk) { 
		l=7;//indexed 8 databits....
		for (int8_t i = expectedBits-5; i >= 0;i--) { //get the data part!
				(rx_packet_raw & (1<<i)) ? data |= (1<<l) : data &= ~(1<<l);
				l--;
		}
		packet[packetcount] = data;
		packetcount++;
		if(packetcount >= nRepeatsExpected) {
			for(int8_t i=0; i<nRepeatsExpected;i++){
				for (int8_t l=0;l<nRepeatsExpected;l++){
					if(i==l)continue;
					if(packet[i] == packet[l]) {
						rx_packet = packet[i];
						PORTB ^= (1<<LEDPIN);
						Init();
					}
				}
			}
		}
	} else { //wrong channel... ignore this shit!
		Init();
	}
	
}

//To restart looking for new sync:
void Init(){
	PORTB &= ~(1<<LEDPIN);//debug led off!
	for(uint8_t i=0;i<nRepeatsExpected;i++) packet[i] = 0; //empty packet array... to fill again!
	
	packetcount = 0; //reset packet counter;
	rx_packet = 0;
	i_sector = 0; //restart i_sectorcount!
	rx_packet_raw = 0; //channel reset... listen for new one!
	rx = 0; //start again looking for sync
	ibit = expectedBits; //reset bit counter
	millisecs = 0; // 0ms
	TCNT0 = 0; //also reset hw timer
	GIMSK |= (1<<INT0); //re-enable those INT0 interrupts we are in sync!
}

/*
Der Compare Interrupt Handler
wird alle 1ms aufgerufen!
*/
ISR (TIM0_COMPA_vect)
{
	millisecs++;
	if(rx == 3) {//what to do after syncing:
		//sectorSetBit();
		if(millisecs == (i_sector+1)*pulselength) i_sector++; //next sector!!
		if(i_sector > expectedBits+1) Init(); //out of bounds reset!
	}
}

ISR (INT0_vect){
	int8_t state = (PINB & (1<<RXPIN));
	if(millisecs<pulselength-1 || millisecs > pulselength+1) rx=0; 
	if(rx == 0 && state && millisecs >= pulselength-1 && millisecs <= pulselength+1) { // rx=1; //the first whole low pulse we got!
	//if(rx == 1 && !state && millisecs >= pulselength-1 && millisecs <= pulselength+1) { //followed by a high pulse!!!! we must be synced!!! OR IN MIDST OF DATA!?
		rx=3;
		PORTB |= (1<<LEDPIN); //debug led!
		GIMSK &= ~(1<<INT0); //disable pin change Interrupts while we're receiving data or at least try to!
	} 
	
	millisecs = 0; //0ms
	TCNT0 = 0; //also reset hw timer
}