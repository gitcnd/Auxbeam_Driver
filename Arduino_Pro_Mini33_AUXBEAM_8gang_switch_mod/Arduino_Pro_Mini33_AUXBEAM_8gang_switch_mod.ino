/***
*   Arduino_Pro_Mini33_AUXBEAM_8gang_switch_mod
*   
   This code connects in-line with the 4 wires
   bewteen the AUXBEAM head-unit and relay module.
  
   * It lets you read button pressed on the head unit
   * It lets you controls relays on the base unit
   * Your Pro Mini's serial port is usable by you still
   * I2C and SPI lines are also free for you to use
     (e.g. for CANbus or sensors etc)
 

   AUXBEAM PLUG WIRING DETAILS
   ===========================

   Female socket of the head unit has these pins:

    (viewed looking in to the socket)

      /--U--\           /--U--\     
     / O   O \         / 4   3 \    
    |         |       |         |   
    |         |       |         |   
     \ O   O /         \ 1   2 /    
      \-----/           \-----/     
    (female           (pin
      socketholes)       numbering)

   1 = 3.3v
   2 = TX    (sends from head unit to relay box)
   3 = RX
   4 = GND


   SERIAL SETUP
   ============
   
   The communications protocol is as follows:
     -> The baud rate is 115200.
     -> The protocol is 8 bits (no parity, unsure of stop bits. 0 worked for me).


  COMMUNICATIONS PROTOCOL
  =======================
  
  Every button sends 3 bytes:-
 
       0xAA 0x?? 0x00
 
  Where the middle byte is simply a bitmask of 8 bits
  (one for each button): 0 means off, 1 means on.
 
  The MSB (bit "A" below) is the top-right button, and
  they're numbered anticlockwise (so the bottom-right 
  button's bit is "H")

     0bABCDEFGH
 
  E.g. to turn on the bottom-right button's relay, send 0xAA 0x01 0x00    (note: this turns all the others off as well of course)
  E.g. to turn on the top-right button's relay, send 0xAA 0x80 0x00       (note: this turns all the others off as well of course)
  E.g. to turn both of those on at once, send 0xAA 0x81 0x00
  
  To turn them all on, send 0xAA 0xFF 0x00
  To turn them all off, send 0xAA 0x00 0x00
  
 
  WIRING
  ====== 

  CAN_Bus
  -------

           CAN_Bus <=> ARDUINO
           SO   => PIN12 (MISO)
           SI   => PIN11 (MOSI)
           SCK  => PIN13 (SCK)
           CS   => PIN10 (SS)
           INT  => PIN2

  AUXBEAM
  -------

  Splice the Arduino pro-mini into the 4 wires as follows:
  
   AUXBEAM    ARDUINO 8mhz
    PLUG      ProMini 3.3v
   =======    ============
     GND   => GND
     3.3v  => VCC
     TX(4) => D4  (PORTD4; pin 4; *not* the ProMini RX pin!)
     RX(3) => D3  (PORTD3; pin 3; *not* the ProMini TX pin!)

  Be aware that this code *reads* data from BOTH of the above
  pins (messages from the head unit, as well as replies from
  the base unit), and this code can WRITE data onto TX(2) as
  well (to send commands to the base unit)

  Writing is done using a "bitbash" method that uses all CPU
  for about 180us.  This ensures exact signal compatibility
  with the origianl Auxbeam, and leaves your serial port free
  to use for other things yourself.

*
****/


#include <SerialID.h>	// See https://github.com/gitcnd/SerialID
SerialIDset("\n#\tv1.04 3.3v_8mhz_Pro_Mini " __FILE__ "\t" __DATE__ " " __TIME__); // So we know what code and version is running inside our MCUs


#include <mcp_can.h>
#include <SPI.h>

// CAN TX Variables
unsigned long prevTX = 0;                                        // Variable to store last execution time
const unsigned int invlTX = 1000;                                // One second interval constant
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};  // Generic CAN data to send

// CAN RX Variables
long unsigned int rxId;
unsigned char len;
unsigned char rxBuf[8];

// Serial Output String Buffer
char msgString[128];

// CAN0 INT and CS
#define CAN0_INT 2                              // Set INT to pin 2
MCP_CAN CAN0(10);                               // Set CS to pin 10




// Pin 13 has an LED connected on most Arduino boards.
// Pin 11 has the LED on Teensy 2.0
// Pin 6  has the LED on Teensy++ 2.0
// Pin 13 has the LED on Teensy 3.0
// give it a name:
int led = LED_BUILTIN;

int bashpin=4;	// Do not change this, or, see "Warning:" below to also change the port bit too


#define rxHeadPin bashpin
#define rxBasePin 3
#define txBasePin 3

//#define reservedPin 4   // Never used, but SoftwareSerial wants us to define a TX pin when we listen for RX, even if we never plan to send over it.
//#include <SoftwareSerial.h>     // See https://www.arduino.cc/en/Reference/softwareSerial
//SoftwareSerial SwSerialH(rxHeadPin, reservedPin);       // RX, TX
//SoftwareSerial SwSerialB(rxBasePin, reservedPin);     // RX, TX
//SoftwareSerial SwSerialT(reservedPin, txBasePin);     // RX, TX
// Darn: cannot use SoftwareSerial because it can only listen on one pn at a time..




#define pcklen 5
#define datbyte 2
unsigned char buf[pcklen] = { 0b00001101, 0b01010001, 0x55, 0b00111111, 0b11100000};
// Timing: ... 270us between .......^---------------------------------------^

// 8mhz ProMini33 3.3v:  32 = 292us 

#if F_CPU == 8000000L
#define baud115200 17
#elif F_CPU == 16000000L
#define baud115200 not tested on 16mhz arduino - you will need an oscilloscope to work out this number - see line "ldi r16, 17" below.
#endif

// #define USECAN 1

void setup() {                
  SerialIDshow(115200); // starts Serial.

#ifdef USECAN
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  // Since we do not set NORMAL mode, we are in loopback mode by default.
  CAN0.setMode(MCP_NORMAL);

  pinMode(CAN0_INT, INPUT_PULLUP);                           // Configuring pin for /INT input
#else
  pinMode(led, OUTPUT);     // initialize the digital pin as an output. (if CAN SCK isnt using it)
#endif

  //Serial.print("Using delay cycles: ");Serial.println( baud115200);

  pinMode(rxHeadPin, INPUT_PULLUP); // so as not to interfere with existing line singals (this goes to OUTPUT only when in use)
  pinMode(rxBasePin, INPUT_PULLUP); // so as not to interfere with existing line singals
  //pinMode(reservedPin, OUTPUT); // not really necessary
  // Initialize Sofware Serial - to listen for incoming head unit buttons
  //SwSerialH.begin(115200); // Damn - cannot do multiplexed inputs... grrr...

}
uint8_t demoloop[]={ 0b00000000,0b11111111,0b00000000,


0b00000001,
0b00000010,
0b00000100,
0b00001000,
0b00010000,
0b00100000,
0b01000000,
0b10000000,

0b11000000,
0b11100000,
0b11110000,
0b11111000,
0b11111100,
0b11111110,
0b11111111,

0b01111111,
0b00111111,
0b00011111,
0b00001111,
0b00000111,
0b00000011,
0b00000001,
0b00000000,
0x55,0xAA,0
};
int democtr=0; unsigned char led_tog=0;
void loop() {
  int ret=0;
  //pinMode(led,OUTPUT);  // cant use LED - SPI CLK is connected here
  //digitalWrite(led, HIGH);   		// turn the LED on (HIGH is the voltage level)
  delay(500);               		// wait for a second

  ret=bitsend(buf, pcklen, baud115200 );		// Do a test sendout
  //if(buf[datbyte]=0x55) buf[datbyte]=0xAA; else buf[datbyte]=0x55;
  buf[datbyte]=demoloop[democtr++]; if(buf[datbyte]==0xAA) democtr=0; // demo patterns
  Serial.println(buf[datbyte]);
  
#ifndef USECAN  
  digitalWrite(led, led_tog++&1);    		// turn the LED off by making the voltage LOW
#endif
  delay(500);               		// wait for a second

#ifdef USECAN

  for(int ii=0;ii<3;ii++) {

  if(!digitalRead(CAN0_INT))                          // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000)             // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    Serial.print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
        
    Serial.println();
  }else {
    //Serial.print("pin "); Serial.print(CAN0_INT); Serial.println(" is high: no data to read");
  }
  
  if(millis() - prevTX >= invlTX){                    // Send this at a one second interval. 
    CAN0.enOneShotTX(); delay(10);
    prevTX = millis();
    byte sndStat = CAN0.sendMsgBuf(0x7f3, 8, data); data[7]++;
    delay(10);CAN0.disOneShotTX();delay(10);
    if(sndStat == CAN_OK)
      Serial.println("Message Sent Successfully!");
    else
      Serial.println("Error Sending Message...");

  }




  }// for //delay(3000);

#endif

}


// Carefully send 0 or 1 bits out of a pin with accurate symmetric timing (based on CPU clock cycles)
int bitsend ( unsigned char *b,int l, int dly) {
  unsigned char bit=0b10000000;  	// We send each bit of the inuput string "b" one-at-a-time
  unsigned char bitu=0b10000000; 	// Not used - here for CPU clock cycle timing symmetry
  int i=0;				// The counter into our input string
  int j=0; 				// unused, except for loop timing symmetry
  digitalWrite(bashpin,HIGH);		// prepare to take over the line
  byte p0=PORTD & 0b11101111; 
  byte p1=  p0 |  0b00010000;		// Warning: bashpin (4) is the bit hard-coded on in this binary number
  
  cli();				// timing critical section starts here: disable interrupts...
  pinMode(bashpin, OUTPUT);		// connect to the line
  while(i<l) {
    if( b[i] & bit ) {
      //digitalWrite(bashpin,LOW);	// "one" bits are sent by signlling to GND
      PORTD=p0;
    } else {
      //digitalWrite(bashpin,HIGH);
      PORTD=p1;
    }
    bit = bit >> 1; bitu = bitu >> 1;	// prep to do the next bit
    if(!bit) {bit=0b10000000;i++;} else {bitu=0b10000000;j++;}  // move to next byte, in a way that always taks the same # of CPU cycles every time.

    // compiler optimizer strips this: for(int d=0;d<dly;d++) j++;    // Do nothing for a specific amount of time.  We need to "waste" 64 total cycles at 8mhz to meet our 115200baud target output rate.
    uint8_t value = dly;
  __asm__ __volatile__(
    "ldi r16, 17                \n\t"//   17 yeilds 271us (our 115200 target is 270us) // This is baud115200 
    "1:                         \n\t"//
    "dec r16              \n\t"//
    "brne 1b                     \n\t"//
    "mov %[result0], r16 \n\t"//
    :
    // Output operands
    [result0] "=r" (value)         //
    :
    // Input operands
    [value0] "r" (value)
    : 
    // Clobbers 
    "r16"
  );


  }
  digitalWrite(bashpin,HIGH);     // prepare to release the line (stops a slow rise if last bit was low)
  pinMode(bashpin, INPUT_PULLUP);	// Do not hog the bus while idle... INPUT_PULLUP lets us control the base unit without the head switch if we want to.
  sei();				// critical bit over - allow interrupts again
  return j+bitu;			// unused, but here to prevent compiler optimising out our otherwise unused timing-symmetry variables "j" and "bitu"
} // bitsend


// #ifdef TEENSYDUINO

// #ifdef ARDUINO_ESP8266_NODEMCU // Wemos_D1_ESP8266_NodeMCU_1

// #ifdef ARDUINO_AVR_PRO	// Arduino_Pro_Mini33

/*

Arduino_Pro_Mini33
        -DF_CPU=8000000L
        -DARDUINO=10807
        -DARDUINO_AVR_PRO
        -DARDUINO_ARCH_AVR

Arduino_Nano
        -DF_CPU=16000000L
        -DARDUINO=10807
        -DARDUINO_AVR_NANO
        -DARDUINO_ARCH_AVR

Teensy
        -D__MK20DX256__
        -DTEENSYDUINO=144
        -DARDUINO=10807
        -DF_CPU=96000000
        -DUSB_SERIAL
        -DLAYOUT_US_ENGLISH

Wemos_D1_ESP8266_NodeMCU_1
        -D__ets__
        -DICACHE_FLASH
        -DF_CPU=80000000L
        -DLWIP_OPEN_SRC
        -DTCP_MSS=536
        -DARDUINO=10807
        -DARDUINO_ESP8266_NODEMCU
        -DARDUINO_ARCH_ESP8266
        -DARDUINO_BOARD=\"ESP8266_NODEMCU\"
        -DESP8266

 */
