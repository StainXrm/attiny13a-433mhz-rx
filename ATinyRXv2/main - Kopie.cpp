#include <avr/io.h> //http://ww1.microchip.com/downloads/en/DeviceDoc/doc8126.pdf
#include <avr/interrupt.h>

//What a single RX should look like : (first low should still look noisy)
//    ____      ____            _ _ _
//   |    |    |    |				 |
//   |    |____|    |____ _ _ |      _   _      _
//   |<SP>|<SP>|<SP>|<SP>|< Channel >|<    Databits     >|< END >
//   |< 1>|< 0>|< 1>|< 0>|< X X X X >|< X X X X X X X X >| 0 0
//        |<-->|=Syncbit!
// 4 sync bits + 4 channel bits + 8 data bits + 2 endbits = 18bit length


//So after calling Init() it searches for data if found rx gets set to -1;
//If rx is set to -1 you get the actual data from rx_packet variable
//Start new search by calling new Init();

#define RXPIN PINB1 //INT0 PIN!
#define LEDPIN PINB4

#define pulselength 5 //ms
#define repeatsExpected 7 //how many packets we expecting?
#define expectedDataBits 8 //how many databits we expect?
#define channel 3 //what channel to listen to?

//volatile const nbits = expectedDataBits+6; //INDEXED bits 8databits+(4channel+2synccheck) = 14 bits (13 indexed)

volatile uint8_t millisecs; //ms counter
volatile uint16_t longtimems; //ms counter last received data
volatile int8_t rx; //this var is for stage of receiving
volatile int8_t bit; //bit counter
volatile uint8_t pac; //packet counter
volatile uint8_t datapack[repeatsExpected]; //packets received in here
volatile uint8_t lastdata;
volatile int8_t packeterror; //error counter

void bitHandler();//is for variable voting in each section and setting president in raw bit
void Init();

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
		if(rx==10)bitHandler();
	}
}

void Evaluate(){
	if(repeatsExpected-packeterror > 0) {//we got data right!? else skip!
		for (int8_t i  = 0; i<repeatsExpected-packeterror;i++){
			for (int8_t l = 0;l<repeatsExpected-packeterror;l++){
				if(i==l) continue;
				if(datapack[i] == datapack[l]){
					lastdata = datapack[i];
				}
			}
		}
	}
	(lastdata%2) ? PORTB |= (1<<LEDPIN) : PORTB &= ~(1<<LEDPIN);
	for (int8_t i = 0; i<repeatsExpected;i++) {
		datapack[i] = 0; //clear datapacks!
	}
	pac=0;//restart packet counter
	packeterror = 0; //no packets yet, no errors!
	Init();
}

void bitHandler() {//sets everyhing where it belongs,ignores or resets!
	
	static uint16_t rx_packet_raw = 0; //store raw packet bits in here!
	static uint8_t chan = 0;
	static int16_t bitvote = 0; //big enough?
	static uint8_t currentbit = 0; //the bit we are currently on!
	uint8_t bitval = 0;
	
	if(bit == 0) { //we're starting all over again, get ready!
		currentbit = 0;//reset last bit if we start from beginning bit
		chan = 0; //no channel at beginning!
		rx_packet_raw = 0; //first bit... reset our raw_packet
	}
	
	(PINB & (1<<RXPIN)) ? bitvote++ : bitvote--; //another vote for our new bit!!
	
	if(currentbit != bit) { //new bit!	
		//Evaluate old one first:
		bitval = (bitvote>0) ? 1 : 0; //what did the voting get us?
		bitvote = 0; //reset the voting for next bit!
	
		//Checks'n Sets:
		if (currentbit == 0) {
			if(bitval != 1){//part of the syncbit, if not 1 we got bullshit!
				Init(); 
				return; 
			}
		} else if(currentbit == 1){
			if(bitval != 0){//part of the syncbit, if not 0 we got bullshit!
				Init(); 
				return; 
			}
		} else if(currentbit <= 5) {
			//Set channel bit:
			chan = chan<<1;//shift whole chan one to left
			if(bitval) chan |= (1<<0);//set new bit to one or do nothing to leave it at zero!
		} else { //or get to datapart here!
			if(chan == channel){ //---------------------------------SO BASICALLY WE GET A 1)LEGIT CHANNEL OR 2)WRONG CHANNEL OR 3)BITERROR
				longtimems = 0; //we just got data... reset timeout!
				datapack[pac] = datapack[pac]<<1;//shift pack contents one left!
				if(bitval) datapack[pac] |= (1<<0);//set new bit to one or do nothing to leave it at zero!
			} else {
				packeterror++; ??!?!?!??!?!?!?!?!??!?!?! packetsets gets skipped!
			}
		}
		
		
		//Shift one left and set bit:
		rx_packet_raw = rx_packet_raw<<1;//shift whole raw one to left
		if(bitval) rx_packet_raw |= (1<<0);//set new bit to one or do nothing to leave it at zero!
		//(bitvote) ? rx_packet_raw |= (1<<reversedBit) : rx_packet_raw &= ~(1<<reversedBit); //set bit as low or high in our raw packet!
		
		
		if(currentbit >= expectedDataBits+5) {//all done!
			if(chan == channel) pac++;
			if(pac == repeatsExpected-packeterror) {
				Evaluate(); //evaluate what we got so far!
			}
			Init(); //wait for next sync
		}
		currentbit = bit; //update currentbit!
	}
}

//start all over
//if hardReset is 1 we also clear packet buffer!
void Init(){
	//PORTB &= ~(1<<LEDPIN);//debug led off!
	rx = 0; //start again looking for sync
	bit = 0; //reset bit counter
	millisecs = 0; // 0ms
	TCNT0 = 0; //also reset hw timer
	
	GIMSK |= (1<<INT0); //re-enable those INT0 interrupts we are in sync!
}

/*
Der Timer Compare Interrupt Handler
wird alle 1ms aufgerufen!
*/
ISR (TIM0_COMPA_vect)
{
	millisecs++;
	longtimems++;
	//if(longtimems > ( (expectedDataBits+8)*pulselength)*repeatsExpected) { //timeout
		//for (int8_t i = 0; i<repeatsExpected;i++)datapack[i] = 0; //clear datapacks!	
		//longtimems=0; //reset counter!
		//Evaluate();
	//}
	if(rx == 10) {//what to do after syncing:
		//if(longtimems > 500) Evaluate(); //timeout!
		
		if(millisecs == (bit+1)*pulselength) bit++; //next sector!!
	}
}

/*
Der Pinchange Interrupt Handler
*/
ISR (INT0_vect){
	if(rx == 0 && (PINB & (1<<RXPIN)) && millisecs >= pulselength-1 && millisecs <= pulselength+1) { //just got out of a whole low pulse -> sync?!
		rx=10;//check for data!
		GIMSK &= ~(1<<INT0); //disable pin change Interrupts while we do data stuff!
	}
	millisecs = 0; //0ms
	TCNT0 = 0; //also reset hw timer
}