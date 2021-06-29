#include <Arduino.h>

struct weather_station {
  uint8_t sensor;       // n0-n1
  float temp;           // n3-n4-n5       
  uint32_t humidity;    // n6-n7
  uint32_t windspeed;   // n6-n7
  uint32_t windgust;    // n6-n7  
  uint32_t winddir;     // n3(b0)-n4-n5
  uint32_t rain;
  uint8_t chksum;       // n8
  uint8_t mode;         // n2(b2-1)
  bool volts;           // n2(b3)
  bool bt;              // n2(b0)
} ;
weather_station weather;

const int RX_PIN = 7;             // Input pin.
const int SYNC_TL = 7500;         // Sync pulse length (ABS: MIN 4040, MAX 8950).
const int LO = 1880;              // LO (0) pulse length (ABS: MIN 1200, MAX 1900).
const int HI = 3000;              // HI (1) pulse length (ABS: MIN 2040, MAX 4000).
const int BIT_CNT = 36;           // Packet 32 data bits + 4 chksum bits = (36 bits).

// MASKS and BIT offsets for process_report()
// used in pairs ( xxM , xxB )
const uint32_t IDM = 0x000000ff;  // Identity
const uint8_t IDB = 0; 
const uint32_t MDM = 0x00007600;  // Sensor type
const uint8_t MDB = 8;
const uint32_t TEM = 0x00fff000;  // Temperature
const uint8_t TEB = 12;
const uint32_t HUM = 0xff000000;  // Humidity
const uint8_t HUB = 24;
const uint32_t WSM = 0xff000000;  // Windspeed
const uint8_t WSB = 24;
const uint32_t WDM = 0x00ff8000;  // Wind Direction
const uint8_t WDB = 15;
const uint32_t WGM = 0xff000000;  // Wind Gust
const uint8_t WGB = 24;
const uint32_t RGM = 0xffff0000;  // Rain Gauge
const uint8_t RGB = 14;
/* 
const uint32_t VOM = 0x00000000;  // Battery Status
const uint8_t VOB = 0;
const uint32_t SWM = 0x00000000;  // Button Press
const uint8_t SWB = 0;
*/
bool sync = false;                // Sync pulse status: 
uint32_t pulse = 0;               // Length of pulse in micro seconds.
uint8_t bit_ptr = 0;              // Bit pointer (counter) for report.
uint32_t report = 0;              // Data packet (32 bits of data).
uint8_t chksum = 0;               // Checksum byte (only bits 0 to 3 are used).
uint8_t buf[4];                   // Data packet report as bytes.
uint8_t packet_cnt = 0;

//****************************************
void setup() {
  Serial.begin(115200);
  pinMode(RX_PIN, INPUT);
}
//**************************************** 
void loop() {

START_OVER:
  pulse = pulseIn(RX_PIN,LOW);                                      // Wait for a signal pulse. (Output LOW)
  if (pulse < LO) {                                                 // If pulse too short (noise reduction). 
    bit_ptr = 0; sync = false;                                      // Reset bit pointer and sync. 
    goto START_OVER;                                                // Start over (could use RETURN).
  }
  if (!sync && pulse > SYNC_TL) {                                   // Test for a sync pulse.
    sync = true;                                                    // Yes, Sync pulse.
    bit_ptr = 0; report=0; chksum=0;                                // Reset variables, ready for data.
  }
  if (sync && pulse > SYNC_TL) {                                    // Test for another sync pulse.
    sync = false;                                                   // Yes, another sync pulse (dont need that).
    goto START_OVER;                                                // Was not a data pulse, Start over.
  }                                                                 // Now we have a data bits.
  if ((bit_ptr < 32) & (pulse > HI)) bitSet(report,bit_ptr);        // Test and save bits too report.
  if ((bit_ptr >= 32) & (pulse > HI)) bitSet(chksum,bit_ptr-32);    // Test and save bits too checksum.
  bit_ptr++;                                                        // Point to next bit.


  if (bit_ptr == BIT_CNT) {                                         // End of data and checksum packet.
    sync = false;                                                   // Reset SYNC flag.
    bit_ptr = 0;                                                    // Reset bit pointer.
    packet_cnt++;                                                   // Increment packets count.
    process_report();                                               // Process report.
    if (packet_cnt == 6){
      print_report();
      packet_cnt = 0;
    } 
  }
}

//****************************************
// extracts data from packet and stores in weather stucture.
// Returns - nothing,
// Requires - number(REPORT,MASK,BITS) function.
//          - checksum() function

void process_report(){

  if ( checksum(report) == false ) return;                // dont save bad data.
  
  weather.sensor = number(report, IDM, IDB);
  weather.mode = number(report, MDM, MDB);
  switch (weather.mode) {  
    case 0x16:
      weather.windspeed = number(report, WSM, WSB);
      break; 
    case 0x76: 
      weather.windgust = number(report, WGM, WGB);
      weather.winddir = number(report, WDM, WDB);
      break; 
    case 0x36: 
        weather.rain = number(report, RGM, RGB);
        break;
    default: 
      weather.temp = number(report, TEM, TEB);
      weather.humidity = number(report, HUM, HUB);
      break;
  }
}

//****************************************
// extract DATA from REPORT
// Requires - 32bit, RX data packet, ( report ). 
//            32bit, Mask for item, ( const xxM ).
//             8bit, bits to shift, ( const xxB ).
// Returns - 32bit value of sensor.
// Used by - process_report() function.

uint32_t number(uint32_t REPORT,uint32_t MASK,uint8_t BITS){
  
  REPORT &= MASK;
  return uint32_t (REPORT >> BITS); 
}

//****************************************
// Test Checksum for COMBINED and RAIN SENSORS.
// checksum COMB = (0xf-n0-n1-n2-n3-n4-n5-n6-n7) & 0xf
// checksum RAIN = (0x7+n0+n1+n2+n3+n4+n5+n6+n7) & 0xf
// Requires - 32bit, RX data packet, ( report ).
// Returns - TRUE/FALSE.
// Used by - process_report() function.

bool checksum(uint32_t REPORT){

uint8_t X = 0, Y = 28, CC = 0x0f, CR = 0x07;

  do {
    CC -= uint8_t (REPORT >> Y) & 0xf;
    CR += uint8_t (REPORT >> Y) & 0xf;
    Y-=4; X++;
  } while ( X < 8);
  CC &= 0xf;
  CR &= 0xf;  
  if (CC == chksum ) return true;
  if (CR == chksum ) return true;          
  return false;
}

//****************************************
void print_report(){
  
  Serial.println ("**********************");
  Serial.print ("ID: "); Serial.println(weather.sensor);
  Serial.println ("********");
  Serial.print ("TEMPERATURE : "); Serial.println((weather.temp / 10));
  Serial.print ("HUMIDITY    : "); Serial.println(weather.humidity,HEX);
  Serial.print ("RAIN        : "); Serial.println(weather.rain);
  Serial.println ("WIND :");
  Serial.print ("      DIR   : "); Serial.println(weather.winddir);
  Serial.print ("      SPEED : "); Serial.println(weather.windspeed);
  Serial.print ("      GUST  : "); Serial.println(weather.windgust);
  Serial.println ("**********************");  
}

/*
//****************************************
// Move 32bit number into 4 byte buffer
// NOT USED

uint8_t buf[4]; // needs to be declared global

void p_buffer(uint32_t REPORT){
  
uint8_t X = 0, Y = 0, C = 0;


//  X = 0; Y = 24;
  X = 0; Y = 0;
  do {
    buf[X] = uint8_t (REPORT >> Y);
//    Y-=8; X++;
    Y+=8; X++;
  } while ( X < 4);
  return;
}

*/

// long value = (unsigned long)(buf[4] << 24) | (buf[3] << 16) | (buf[2] << 8) | buf[1];
