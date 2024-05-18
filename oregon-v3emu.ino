/**************************************************************************************************************************************
**
** Project: Oregon Scientific Emulator
** File:    oregon-v3emu.INO
** Purpose: Main SW File
** 
** (C) 2019 by Daniele Sgroi - daniele.sgroi@gmail.com
**
** VERSION:
**  - May 18, 2024 - ALPHA 0.1 - D. Sgroi
**
** TARGET HW:
** - Teensy 4.1 with Ethernet Shield
**
** SOFTWARE:
** - Arduino 2.3.2+ IDE
**
** References:
** https://github.com/lrswss/oregon-v3-sensor-emulation/tree/main
** http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf
** https://github.com/merbanan/rtl_433/blob/master/src/devices/oregon_scientific.c
** https://github.com/arkhipenko/TaskScheduler
**
** LICENSE: 
** 
** Published under MIT license.
**
**************************************************************************************************************************************/

#define DEBUG                  1  // enable verbose debug messages on console  

/**************************************************************************************************************************************
** HW DEFINES
**************************************************************************************************************************************/

#define ANALOG_PIN_1          A1 // used for rolling code seed at startup
#define RF433_TX_PIN           2 // 433,92 MHz TX Data
#define RF433_PWR_PIN          3 // 433,92 MHz TX Power Enable

/**************************************************************************************************************************************
** SW DEFINES
**************************************************************************************************************************************/

#define V3_PULSE_LENGHT_US   488  // data rate 1024Hz
#define V3_PULSE_SHORTEN_US  134  // 138us not working reliably
#define V3_PULSE_TUNING      1.1  // increase up to 1.4 if base station doesn't pick up messages

#define UVN_PAYLOAD_NIBBLES   13
#define UVN_TX_BYTES          12

#define PCR_PAYLOAD_NIBBLES   18
#define PCR_TX_BYTES          15

// TX Channel 1..10 for WMR200
#define THG_PAYLOAD_NIBBLES   15 // excluding preamble, sync and trailing checksums
#define THG_TX_BYTES          13

#define WGR_PAYLOAD_NIBBLES   17
#define WGR_TX_BYTES          14

/**************************************************************************************************************************************
** INCLUDES
**************************************************************************************************************************************/

#include <_Teensy.h>
#include "TaskScheduler.h" // https://github.com/arkhipenko/TaskScheduler

/**************************************************************************************************************************************
** GLOBALS
**************************************************************************************************************************************/

// OSV3 THGR Sensor channel tx rate in sec. idx 0 is UVN800 rate
const unsigned long int channelrate[] = {73000, 53000, 59000, 61000, 67000, 
                                         71000, 79000, 83000, 87000, 91000, 
                                         93000};

// set UVN800's reading for UVI which is increased after each 
// transmission for testing purposes if DEBUG_INC is set
static uint8_t t_uvi = 1;

// set PCR800's precipitation readings (inches) which are increased 
// after each transmission for testing purposes if DEBUG_INC is set
static float p_total = 2.7;
static float p_hourly = 0.3;

// set THGN801's values for temperature (degrees Celcius) and humidity 
// which are  increased after each transmission if DEBUG_INC is set
static float t_temp = -1.5;
static uint8_t t_hum = 20;

// set WGR800 wind speed readings (max. 99.9 m/s) and direction which are
// increased after each transmission for testing purposes if DEBUG_INC is set 
static float t_avg = 1.3;
static float t_gust = 2.6;
static float t_dir = 45;

// rolling code changes on every sensor reset
static uint8_t rollingCode;

/**************************************************************************************************************************************
** Scheduler stuff
**************************************************************************************************************************************/

// Scheduler Callback methods prototypes
void t0Callback(void); // UVN
void t1Callback(void); // THGR 1
void t2Callback(void); // THGR 2
void t3Callback(void); // THGR 3
void t4Callback(void); // PCR
void t5Callback(void); // WGR 14000

// Scheduler
Scheduler runner;

//Tasks
Task t0(73000, TASK_FOREVER, &t0Callback, &runner, false); // UVN 0
Task t1(53000, TASK_FOREVER, &t1Callback, &runner, false); // THGN 1
Task t2(59000, TASK_FOREVER, &t2Callback, &runner, false); // THGN 2
Task t3(61000, TASK_FOREVER, &t3Callback, &runner, false); // THGN 3
Task t4(47000, TASK_FOREVER, &t4Callback, &runner, false); // PCR 47000
Task t5(14000, TASK_FOREVER, &t5Callback, &runner, false); // WGR 14000

/***************************************************************************
** t0Callback
**
** Handle UVN800 data
**
***************************************************************************/

void t0Callback() {

#ifdef DEBUG
  Serial.print(F("t0 run "));
#endif

  send_data_v3(payload_uvn800(t_uvi), UVN_TX_BYTES); 

  // increase readings for testing
  t_uvi += 1;
  if (t_uvi > 15)
    t_uvi = 1;

  if (t0.isFirstIteration()) {
#ifdef DEBUG    
    Serial.println(F("t1 enable"));
#endif    
    t1.enableDelayed(500); // set space between tx
  }

} // t0Callback

/***************************************************************************
** t1Callback
**
** Handle THGR 1 data
**
***************************************************************************/

void t1Callback() {

#ifdef DEBUG
  Serial.print(F("t1 run "));
#endif  

  send_data_v3(payload_thgn801(t_temp, t_hum, 0x01), THG_TX_BYTES);

  // increase readings for testing
  t_temp += 0.1;
  if (t_temp  > 45)
    t_temp = 19.0;
  
  t_hum += 1;
  if (t_hum > 95)
    t_hum = 30;

  if (t1.isFirstIteration()) {
#ifdef DEBUG
    Serial.println(F("t2 enable"));
#endif  
    t2.enableDelayed(500); // set space between tx
  }

} // t1Callback

/***************************************************************************
** t2Callback
**
** Handle THGR 2 data
**
***************************************************************************/

void t2Callback() {

#ifdef DEBUG
  Serial.print(F("t2 run "));
#endif  

  send_data_v3(payload_thgn801(-t_temp, t_hum - 30, 0x02), THG_TX_BYTES);

  // increase readings for testing
  t_temp += 0.1;
  if (t_temp  > 45)
    t_temp = 19.0;
  
  t_hum += 1;
  if (t_hum > 95)
    t_hum = 30;

  if (t2.isFirstIteration()) {
#ifdef DEBUG    
    Serial.println(F("t3 enable"));
#endif
    t3.enableDelayed(500); // set space between tx
  }

} // t2Callback

/***************************************************************************
** t3Callback
**
** Handle THGR 3 data
**
***************************************************************************/

void t3Callback() {

#ifdef DEBUG
  Serial.print(F("t3 run "));
#endif  

  send_data_v3(payload_thgn801(12.3, 45, 0x03), THG_TX_BYTES);

  if (t3.isFirstIteration()) {
#ifdef DEBUG    
    Serial.println(F("t4 enable"));
#endif  
    t4.enableDelayed(500); // set space between tx
  }

} // t4Callback

/***************************************************************************
** t4Callback
**
** Handle PCR data
**
***************************************************************************/

void t4Callback() {

#ifdef DEBUG
  Serial.print(F("t4 run "));
#endif

  send_data_v3(payload_pcr800(p_total, p_hourly), PCR_TX_BYTES); 

  // increase readings for testing
  p_hourly += 0.05;
  if (p_hourly > 3.0)
    p_hourly = 0.5;
    p_total += 0.2;
        
  if (p_total > 20)
    p_total = 10;

  if (t4.isFirstIteration()) {
#ifdef DEBUG    
    Serial.println(F("t5 enable"));
#endif
    t5.enableDelayed(500); // set space between tx
  }

} // t4Callback

/***************************************************************************
** t5Callback
**
** Handle WGR data
**
***************************************************************************/

void t5Callback() {

#ifdef DEBUG
  Serial.print(F("t5 run "));
#endif  

  send_data_v3(payload_wgr800(t_avg, t_gust, t_dir), WGR_TX_BYTES); 

  // increase readings for testing
  t_gust += 0.2;
  if (t_gust  > 20)
    t_gust = 5.0;

  t_avg += 0.1;
  if (t_avg  > 15)
    t_avg = 0.1;
          
  t_dir += 22.5;
    if (t_dir > 359)
    t_dir = 0;

  if (t5.isFirstIteration()) {
#ifdef DEBUG
    Serial.println(F("t6 enable"));
#endif  
    //t6.enableDelayed(500); // set space between tx
  }

} // t5Callback

/**************************************************************************************************************************************
** crc8_checksum_v3
**
** calculate a bitwise crc8 ccitt checksum
**
**************************************************************************************************************************************/

uint8_t crc8_checksum_v3(uint8_t *payload, uint8_t payloadNibbles) {

    uint8_t crc = 0x00;  // crc8 init value

    // iterate over nibbles with sensor payload including a trailing iteration
    for (uint8_t i = 7; i <= (payloadNibbles + 7); i++) {
        if (i < (payloadNibbles + 7)) {
          if ((i % 2) == 0)
              crc ^= payload[i/2] >> 4;
          else
              crc ^= (payload[i/2] & 0x0F);
        }
        for (uint8_t j = 0; j < 4; j++) {
            if (crc & 0x80) 
                crc = (crc << 1) ^ 0x07;
            else 
                crc <<= 1;    
        }
    }
    
    // nibbles need to be swapped (LSD)
    return ((crc & 0x0F) << 4) | ((crc & 0xF0) >> 4);

} // crc8_checksum_v3

/**************************************************************************************************************************************
** oregon_checksum_v3
**
** calculate the Oregon V3 checksum
** based on https://github.com/merbanan/rtl_433/blob/master/src/devices/oregon_scientific.c
**
**************************************************************************************************************************************/

uint8_t oregon_checksum_v3(uint8_t *payload, int payloadNibbles) {

    uint8_t checksum;
  
    // remove sync byte '0xA' from first sensor data nibble
    checksum = (payload[3] & 0xF);

    // nibbles used for simple checksum (sum of nibbles)
    // 4 (sensor id) + 1 (V3_CHANNEL) + 2 (rolling code) + 1 (flag) + payloadNibbles
    for (uint8_t i = 4; i < ((28 + payloadNibbles*4) / 8); i++)
        checksum += (payload[i] >> 4) + (payload[i] & 0xF);
   
    if ((payloadNibbles % 2) == 0)
        checksum += payload[((28 + payloadNibbles*4)/8)] >> 4;

    return ((checksum & 0x0F) << 4) | ((checksum & 0xF0) >> 4);

} //oregon_checksum_v3

/**************************************************************************************************************************************
** manchester_encode_v3
**
** send a single byte as series of manchester encoded RF pulses
** base station won't receive packets if timing is off adjusting V3_PULSE_TUNING from 1.1 up to 1.4 should help
**
**************************************************************************************************************************************/

void manchester_encode_v3(uint8_t txByte) {

    static uint32_t txMicros = micros();
    uint8_t bitMask = 0; 

    // send 8 bits of data 
    for (uint8_t i = 0; i < 8; i++) {  

        // ensure equal distant bit pulses
        txMicros += (V3_PULSE_LENGHT_US * 2);
        if (txMicros - micros() < 0)
            delayMicroseconds((txMicros - micros()) * -1);
        else
            delayMicroseconds(txMicros - micros());

        // send bit 4..7 first followed by bit 0..3
        if (!bitMask) 
            bitMask = 0x10; // start with bit 4
        else if (bitMask == 0x80) 
            bitMask = 0x01; // jump from bit 7 to bit 0
        else
            bitMask <<= 1; // next bit
        
        //  high bit is encoded as as on-to-off and low bit as off-to-on transition
        digitalWrite(RF433_TX_PIN, ((txByte & bitMask) >= 1) ? HIGH : LOW);
        delayMicroseconds((V3_PULSE_LENGHT_US * V3_PULSE_TUNING) - V3_PULSE_SHORTEN_US);
        digitalWrite(RF433_TX_PIN, ((txByte & bitMask) >= 1) ? LOW : HIGH);
    }

} // manchester_encode_v3

/**************************************************************************************************************************************
** send_data_v3
**
** send manchester encoded payload for a Oregon V3 sensor
**
**************************************************************************************************************************************/

void send_data_v3(uint8_t *payload, uint8_t len) {

    uint8_t i;

#ifdef DEBUG
    //Serial.print(millis());
    Serial.print(": [ ");
    for (i = 0; i < len; i++) {
        if (payload[i] < 16)
            Serial.print("0");
        Serial.print(payload[i], HEX);
        Serial.print(" ");
    }
    Serial.println("]");
#endif

    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    digitalWrite(RF433_TX_PIN, LOW);
    digitalWrite(RF433_PWR_PIN, HIGH);   // turn tx on

    for (i = 0; i < len; i++)
        manchester_encode_v3(payload[i]);
        
    // need to add extra delay after last low to high 
    // pulse since there is no more data to come...
    if ((payload[i] & 0x08) == 0)
        delayMicroseconds(V3_PULSE_LENGHT_US);
      
    digitalWrite(RF433_TX_PIN, LOW);
    digitalWrite(RF433_PWR_PIN, LOW);   // turn tx on
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED on

} // send_data_v3

/**************************************************************************************************************************************
** payload_uvn800
**
** returns payload for UVN800 outdoor UVI sensor (12 bytes)
**
**************************************************************************************************************************************/

uint8_t *payload_uvn800(uint8_t uvi) {

    static uint8_t payload[UVN_TX_BYTES];

    // 6 nibbles preamble
    // 1 nibble sync
    // 4 nibbles sensor id
    // 1 nibble channel
    // 2 nibbles rolling code
    // 1 nibble flag (battery)
    // n nibbles sensor specific payload (UVN800 n=5, 20 bits)
    // 2 nibbles oregon checksum
    // 2 nibbles crc8 checksum
    memset(payload, 0, UVN_TX_BYTES);
  
    // preamble with 1-bit pulses (6 nibbles) and 1 sync nibble '0101' (28 bits)
    memset(payload, 0xFF, 3);  
    payload[3] = 0xA0;
  
    // nibbles 8..11: oregon sensor id (16 bits)
    payload[3] |= 0x0D; // UVN800 is 0xD874
    payload[4] = 0x87;
    payload[5] = 0x40;
  
    // nibble 12: channel 1 seems to be preset for UVN800 (4 bits)
    payload[5] |= 0x01;
  
    // nibble 13..14 is rolling code, changes on sensor reset (16 bits LSD) 
    payload[6] = rollingCode;
  
    // nibble 15 is flag for battery status (4 bits)
    payload[7] = 0x00; // set to 0x40 for low battery

    // nibbles 16..20 encode UVN800 specific sensor readings (20 bits)
    payload[7] |= (uvi & 0x0F);  // UVI (max. 15)
    // use of byte 8 and 9 unknown, my UVN800 sets byte 9 to 0x70...
    
    // checksum (8 bits) for nibbles 16..20 with sensor specific data (sum of nibbles)
    payload[10] = oregon_checksum_v3(payload, UVN_PAYLOAD_NIBBLES);

    // crc8 checksum for V3 protocol (8 bits)
    payload[11] = crc8_checksum_v3(payload, UVN_PAYLOAD_NIBBLES);

    return payload;

} // payload_uvn800

/**************************************************************************************************************************************
** payload_pcr800
**
** returns payload for PCR800 precipitation sensor (15 bytes)
**
**************************************************************************************************************************************/

uint8_t *payload_pcr800(float pTotal, float pHourly) {

    static uint8_t payload[PCR_TX_BYTES];
    uint32_t total;
    uint16_t hourly;
    uint8_t chksum, crc;
        
    // 6 nibbles preamble
    // 1 nibble sync
    // 4 nibbles sensor id
    // 1 nibble V3_CHANNEL 
    // 2 nibbles rolling code
    // 1 nibble flag (battery)
    // n nibbles sensor specific payload (PCR800 n=10, 40 bits)
    // 2 nibbles oregon checksum
    // 2 nibbles crc8 checksum
    memset(payload, 0, PCR_TX_BYTES);

    // preamble with 1-bit pulses (6 nibbles) and 1 sync nibble '0101' (28 bits)
    memset(payload, 0xFF, 3); // 1..6
    payload[3] = 0xA0; // 7
  
    // nibbles 8..11: oregon sensor id (16 bits)
    payload[3] |= 0x02; // PCR800 is 0x2914
    payload[4] = 0x91;
    payload[5] = 0x40;
  
    // nibble 12: PCR800 usually using channel 0 (8 bit)
    payload[5] |= 0x00;
  
    // nibble 13..14 is rolling code, changes on sensor reset (16 bits LSD) 
    payload[6] = rollingCode;
    
    // nibble 15 is flag for battery status (4 bits)
    payload[7] = 0x00; // set to 0x40 for low battery

    // nibbles 16..19 rain rate, 20..25 total precipitation (40 bits)
    hourly = pHourly * 100;
    total = pTotal * 1000;
    payload[8] = ((hourly / 10) % 10) << 4 | ((hourly / 100) % 10);
    payload[9] = ((hourly / 1000) % 10) << 4 | (total % 10);
    payload[10] = ((total / 10) % 10) << 4 | ((total / 100 ) % 10);
    payload[11] = ((total / 1000) % 10) << 4 | ((total / 10000) % 10);
    payload[12] = ((total / 100000) % 10) << 4;

    // checksum (8 bits) for nibbles 16..25 with sensor specific data (sum of nibbles)
    chksum = oregon_checksum_v3(payload, PCR_PAYLOAD_NIBBLES);
    payload[12] |= (chksum & 0xf0) >> 4;
    payload[13] = (chksum & 0x0f) << 4;
    
    // crc8 checksum for V3 protocol (8 bits)
    crc = crc8_checksum_v3(payload, PCR_PAYLOAD_NIBBLES);
    payload[13] |= (crc & 0xf0) >> 4;
    payload[14] = (crc & 0x0f) << 4;
 
    return payload;  

} // payload_pcr800

/**************************************************************************************************************************************
** payload_thgn801
**
** returns payload for THGN801 thermoigrometer sensor (15 bytes)
**
**************************************************************************************************************************************/

uint8_t *payload_thgn801(float tempC, uint8_t hum, uint8_t chnl) {

    static uint8_t payload[THG_TX_BYTES];
    uint16_t t10;

    // 6 nibbles preamble
    // 1 nibble sync
    // 4 nibbles sensor id
    // 1 nibble channel
    // 2 nibbles rolling code
    // 1 nibble flag (battery)
    // n nibbles sensor specific payload (THGN801 n=7, 28 bits)
    // 2 nibbles oregon checksum
    // 2 nibbles crc8 checksum
    memset(payload, 0, THG_TX_BYTES);
  
    // preamble with 1-bit pulses (6 nibbles) and 1 sync nibble '0101' (28 bits)
    memset(payload, 0xFF, 3);  
    payload[3] = 0xA0;
  
    // nibbles 8..11: oregon sensor id (16 bits)
    payload[3] |= 0x0F; // THGN801 is 0xF824
    payload[4] = 0x82;
    payload[5] = 0x40;
  
    // nibble 12: channel 1..10 for THGN801 using a WMR200 base station (4 bits)
    payload[5] |= chnl;
  
    // nibble 13..14 is rolling code, changes on sensor reset (16 bits LSD) 
    payload[6] = rollingCode;
  
    // nibble 15 is flag for battery status (4 bits)
    payload[7] = 0x00; // set to 0x40 for low battery

    // nibbles 16..22 encode THGN801 specific data (24 bits)
    // 16..19 temperature in degC as LSD with 0.1 precision
    t10 = abs(tempC * 10.001);  // .001 required to fix rounding issues
    payload[7] |= ((t10 % 10) & 0x0F);
    payload[8] = ((t10 / 10) % 10) << 4 | ((t10 / 100) & 0x0F);
    payload[9] = (tempC < 0.0) ? 0x80 : 0; // nibble 19 encodes temperature sign (0x80 => neg.)    

    // 20..21 encodes relative humidity
    payload[9] |= (hum % 10) & 0x0F; // 0-100%
    payload[10] = ((hum / 10) % 10) << 4;  // use of nibble 22 unknown

    // checksum (8 bits) for nibbles 16..21 with sensor specific data (sum of nibbles)
    payload[11] = oregon_checksum_v3(payload, THG_PAYLOAD_NIBBLES);

    // crc8 checksum for V3 protocol (8 bits)
    payload[12] = crc8_checksum_v3(payload, THG_PAYLOAD_NIBBLES);

    return payload;  

} // payload_thgn801

/**************************************************************************************************************************************
** payload_wgr800
**
** returns payload for WGR800 anemometer (14 bytes)
**
**************************************************************************************************************************************/

uint8_t *payload_wgr800(float windAvg, float windGust, float windDir) {

    static uint8_t payload[WGR_TX_BYTES];
    uint16_t wdir, wgust, wavg;

    // 6 nibbles preamble
    // 1 nibble sync
    // 4 nibbles sensor id
    // 1 nibble channel
    // 2 nibbles rolling code
    // 1 nibble flag (battery)
    // n nibbles sensor specific payload (WGR800 n=9, 36 bits)
    // 2 nibbles oregon checksum
    // 2 nibbles crc8 checksum
    memset(payload, 0, WGR_TX_BYTES);
  
    // preamble with 1-bit pulses (6 nibbles) and 1 sync nibble '0101' (28 bits)
    memset(payload, 0xFF, 3); // 1..6
    payload[3] = 0xA0; // 7
  
    // nibbles 8..11: oregon sensor id (16 bits)
    payload[3] |= 0x01; // WGR800 is 0x1984
    payload[4] = 0x98;
    payload[5] = 0x40;
  
    // nibble 12: seems to be channel 0 for WGR800 (4 bits)
    payload[5] |= 0x00;
  
    // nibble 13..14 is rolling code, changes on sensor reset (16 bits LSD) 
    payload[6] = rollingCode;
  
    // nibble 15 is flag for battery status (4 bits)
    payload[7] = 0x00; // set to 0x40 for low battery

    // nibbles 16..24 encodes anemometer specific data (36 bits)
    wdir = windDir * 10.001;
    wgust = windGust * 10.001;
    wavg = windAvg * 10.001;  
    payload[7] |= (wdir / 225) & 0x0F;
    payload[8] = 0x0C; // nibble 17..18 not documented but required...
    payload[9] = (wgust % 10) << 4 | ((wgust / 10) % 10);
    payload[10] = ((wgust / 100) % 10) << 4 | (wavg % 10);
    payload[11] = ((wavg / 10) % 10) << 4 | ((wavg / 100) % 10);

    // checksum (8 bits) for nibbles 16..24 with sensor specific data (sum of nibbles)
    payload[12] = oregon_checksum_v3(payload, WGR_PAYLOAD_NIBBLES);

    // crc8 checksum for V3 protocol (8 bits)
    payload[13] = crc8_checksum_v3(payload, WGR_PAYLOAD_NIBBLES);

    return payload;

} // payload_wgr800

/**************************************************************************************************************************************
** setup
**
** run once at startup
**
**************************************************************************************************************************************/

void setup() {  

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RF433_TX_PIN, OUTPUT);
  pinMode(RF433_PWR_PIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on

  digitalWrite(RF433_TX_PIN, LOW);
  digitalWrite(RF433_PWR_PIN, LOW);   // disalble TX

  Serial.begin(115200);
  //while (!Serial); // wait for serial port to connect. commented to allow start with USB power only
  Serial.println(F("Starting Oregon V3 emulator..."));
  delay(5000); // to allow reset of console
  Serial.print(F(__DATE__));
  Serial.print(F(" - "));
  Serial.print(F(__TIME__));
  Serial.println(F(" - (C)2024 by Daniele Sgroi"));

  // new rolling code with every reset
  randomSeed(analogRead(ANALOG_PIN_1)); // analog pin one used as source of noise for random seed

  rollingCode = random(0x01, 0xFE);
  Serial.print(": Rolling code: ");
  Serial.println(rollingCode, HEX);

  //runner.init();
  //runner.addTask(t0);
  //runner.addTask(t1);
  //runner.addTask(t2);
  //runner.addTask(t3);
  //runner.addTask(t4);
  //runner.addTask(t5);

  // allow powered up items time to ready themselves
  delay(500);

  // enable only t0, t1..tn are enabled lately from inside t0, t1, tn-1 to allow 
  // for time spacing of different channels transmissions
#ifdef DEBUG
  Serial.println(F("t0 enable"));
#endif  

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED

  t0.enable(); // only enable t0 as others are enabled lately to separate rf tx

} // setup

/**************************************************************************************************************************************
** loop
**
** Run continuosly - Only update task scheduler
**
**************************************************************************************************************************************/

void loop() {

    runner.execute();

} // loop

/**************************************************************************************************************************************
** EOF
**************************************************************************************************************************************/


