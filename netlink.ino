/*Status:  Yet another Oregon Scientific Weather Station 433Mhz Arduino Signal Intercepter
 *Open Source
 *Author: Rob Ward December 2013
 *Finds the header bits and synch's to the waveform's 1->0 going edges, which are '1's
 *Soaks up rest of header bits and waits for first 0
 *Accumulates incoming data stream into bytes, arranged from MSB to LSB by rotating a mask in a Left Rotate
 *Checks for Sensor ID's, the ID's used here are relative to the previous way the data is collected, so not the same as others
 *Using the rotate bit on the fly the two ID bytes are as follows, this program just use first byte to ID the sensor
 *        Oregon-THGN800   0xAF  	Outside Temp-Hygro    AF 82 ?? 4 ?? 1 == (Sensor switch nybble 1,2or3)
 *        Oregon-PCR800    0xA2 	Rain Gauge            A2 91 ?? 4 ?? 0
 *        Oregon-WGR800    0xA1 	Anemometer            A1 98 ?? 4 ?? 0
 *Rolling code is present to differentiate between alternative sensors
 *Calculate the check sum for the sensor and if data passes the test...continue
 *Decode the nybbles (if bites are bytes, nybbles are nybbles :-) and calculate the parameters for each sensor
 *Dump the calculations to the screen, round off to decimal places
 
 Why not use interrrupts and count durations?  Manchester encoding came from the time of slow chips, noisey environments and saggy waveforms.
 The beauty of Manchester encoding is that is it can be sampled so the logic transitions are the most important and at least the logic 
 state in the middle of the timing periods is most likely to be valid.  It also self synchronises throughout the packet and 
 automatically detects timeouts.  This is an old, classic Manchester decoding decoding strategy, but quite robust never the less.
 
 To do:
 *find the battery indicators if they exist. (7) suggests it is (my) upper 4 bits of byte 2.  How to fake old batteries??
 *Add a 1 minute dump of current values in CSF format for my WWW Weather station (Remove current debug sensor dump format).
 
 Reference Material:
 Thanks to these authors, they all helped in some way, especially, the last one Brian!!!!
 http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf (1)
 http://jeelabs.net/projects/cafe/wiki/Decoding_the_Oregon_Scientific_V2_protocol (2)
 https://github.com/phardy/WeatherStation (3)
 http://lucsmall.com/2012/04/27/weather-station-hacking-part-1/ (4)
 http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/ (5)
 http://lucsmall.com/2012/04/29/weather-station-hacking-part-2/ (6)
 http://www.mattlary.com/2012/06/23/weather-station-project/(7)
 https://github.com/lucsmall/WH2-Weather-Sensor-Library-for-Arduino (8)
 http://www.lostbyte.com/Arduino-OSV3/ (9) brian@lostbyte.com
 
 Most of all thanks to Oregon Scientific, who have produced an affordable, high quality product.  I can now have my LCD Home Base in the
 kitchen to enjoy, with the Arduino in the garage also capturing data for WWW Weather pages.  Lovely!!!!  http://www.oregonscientific.com
 Very Highly recommended equipment. Rob Ward
 */

#include <avr/wdt.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

// Read data from 433MHz receiver on digital pin 8
#define RxPin 8    //Just an input
#define RSSIPin 0  //A5 to read signal strength
#define ledPin 6  //Human feedback
#define AGCPin 7  // Automatic Gain Control for the RF chip
#define sDelay 230  //One Quarter Manchester Bit duration, is this OK?
#define lDelay 460  //One Half Manchester Bit duration
#define AGC_ON 0
#define AGC_OFF 1
byte    headerHits = 0; //How many ones detected as a header bit
boolean header = false; //State of header detection
boolean logic = false; //State of the Manchester decoding
byte    signal = 0; //state of RF
boolean test230 = false;
boolean test460 = false;
int     maxBytes = 11; //sets the limits of how many data bytes will be required
int     nosBytes = 0; //counter for the data bytes required
boolean firstZero = false; //flags when the first '0' is found.
byte    dataByte = 0; //accumulates the bits of the signal
byte    dataMask = 16; //rotates, so allows nybbles to be reversed
byte    nosBits = 0; //counts the shifted bits in the dataByte
byte    manchester[11]; //storage array for the data accumulated via manchester format
boolean notBCD;    // check that a nybble has not been received that is outside of range 0-9 -reset after each reading
boolean ledState = 0;


double  avWindspeed = 0.0;
double  gustWindspeed = 0.0;
double  rainTotal = 0.0;
double  rainRate = 0.0;
double  temperature = 0.0;
double  barometer = 0.0;
double  uvIndex = 0.0;
int     humidity = 0;
char    outputString [200];
char    dbgString [80];
char	tmpString [20];
int     loopcount = 0;
int     rssi=0;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void invertLed() {
  ledState = !ledState;
  digitalWrite (ledPin, ledState);
}

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
 sensor API sensor_t type (see Adafruit_Sensor for more information)
 */
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
  /*Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); 
  Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); 
  Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); 
  Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); 
  Serial.print(sensor.max_value); 
  Serial.println(" hPa");
  Serial.print  ("Min Value:    "); 
  Serial.print(sensor.min_value); 
  Serial.println(" hPa");
  Serial.print  ("Resolution:   "); 
  Serial.print(sensor.resolution); 
  Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");*/
  delay(500);
}

void getPressure () {
  /* Get a new sensor event */
  sensors_event_t event;
  bmp.getEvent(&event);

  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    barometer = event.pressure;

    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */

    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);


    /* Then convert the atmospheric pressure, SLP and temp to altitude    */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    //Serial.print("Altitude:    "); 
    //Serial.print(bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature)); 
    //Serial.println(" m");
    //Serial.println("");
  }
}

void setup(){
  Serial.begin(115200);
  Serial.println("WMR86 Oregon Decoder");
  initEthernet();
  pinMode(ledPin, OUTPUT);
  pinMode(RxPin, INPUT);
  pinMode(AGCPin, OUTPUT);
  invertLed();
  digitalWrite(AGCPin, AGC_ON);
  delay(100);//heart beat
  invertLed();
  delay(100);//heart beat
  invertLed();
  delay(100);//heart beat
  invertLed();
  headerHits=0;
  /* Initialise the sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("BMP085 not detected.");
    while(1);
  }

  /* Display some basic information on this sensor */
  // displaySensorDetails();
}

void loop(){


  //wait here for a header!!!
  // So far this appears as 'inverted' Manchester 1>0==1, 0>1==0 ??? (G. E. Thomas in 1949) http://en.wikipedia.org/wiki/Manchester_cod

  delay(500);
  wdt_disable();
  while (header == false){
    while (digitalRead(RxPin)==1) { //Stay in loop while logic =1
      //loop while the RxPin==1, first half of bit pattern, just before data transition, 1 to 0
    }//exits when signal == 0, (1->0 falling edge found, transition to value 0) middle of bit pattern, ie the data edge
    delayMicroseconds(sDelay); //Short wait for a 1/4 of the "1" pattern
    if (digitalRead(RxPin) == 0){ //Check signal is still steady at 0 ( !0 = error detection)
      delayMicroseconds(lDelay); // long wait for next 1/2 of bit pattern, 
      // ie now at 3/4 way through, looks like an ok bit "1" now keep track of how many in row we have detected
      if (digitalRead(RxPin) == 1){ // Check Rx polarity has now swapped, good!
        headerHits ++; // Highly likely a "1" waveform, so count it in
        if (headerHits == 20){ //if we can find 20 in a row we will assume it is a header
          header = true; //so we can exit this part, collect rest of header and begin data processing below
          headerHits=0; //reset, so ready to search for another header when next required, or should an error occur in this packet
          //Serial.println("");
          //Serial.print("H"); //uncomment to debug header detection
        }
      }
      else {
        headerHits =0;  // Has not followed the "1" wave pattern, probably badly formed, noisy waveform, so start again
        header = false; // make sure we look for another header
      }
    }
    else {
      headerHits =0;  // Has not followed wave pattern, probably just noise, so start again
      header = false; // make sure we look for another header
    }
  }

  wdt_enable(WDTO_8S); 
  //The program is now synched to the '1' waveform and detecting the 1->0  "data" transitions in the bit waveform
  //The data byte boundaries indicate the Synch '0' is considered a part of the data, so byte boundary begins at that '0'
  logic=1; // look for rest of header 1's, these must be soaked up intil first 0 arrives to denote start of data
  signal = 0; //RF Signal is at 0 after 1's 1->0 transition, inverted Manchester (see Wiki, it is a matter of opinion)
  firstZero = false; //The first zero is not immediately found, but is flagged when found

  while (header == true){

    //now get last of the header, and then store the data after trigger bit 0 arrives, and data train timing remains valid 
    while (digitalRead(RxPin)!=signal){ //halt here while signal matches inverse of logic, if prev=1 wait for sig=0
    }//exits when signal==logic
    delayMicroseconds(sDelay); //wait for first 1/4 of a bit pulse
    test230 = digitalRead(RxPin);//snapshot of the input
    if ((test230 == signal)&&(nosBytes < maxBytes)){  //after a wait the signal level is the same, so all good, continue!
      delayMicroseconds(lDelay); //wait for second 1/2 of a bit pulse
      test460=digitalRead(RxPin);//snapshot of the input
      if (test230==test460){  // finds a long pulse, so the logic to look for will change, so flip the logic value 
        //Assuming the manchester encoding, a long pulse means data flips, otherwise data stays the same
        logic = logic^1;
        signal = signal^1;
        //Serial.print(logic,BIN);  //debug data stream in binary
        if (!firstZero){ //if this is the first 0-1 data transition then is the sync 0
          invertLed();     //data processing begins, first though chew up remaining header
          firstZero = true; //flag that legit data has begun
          //VIP OS Seems to put the Synch '0' bit into the data, as it causes the rest to align onto byte boundaries
          dataByte = B00000000; // set the byte as 1's (just reflects the header bit that have preceded the trigger bit=0)
          dataMask = B00010000; // set the byte as 1's (just reflects the header bit that have preceded the trigger bit=0)
          nosBits = 0;  // preset bit counter so we have 7 bits counted already
          //Serial.print("!");  //debug detection of first zero
        }

      }
      //data stream has been detected begin packing bits into bytes
      if (firstZero){
        if (logic){
          dataByte = dataByte | dataMask; //OR the data bit into the dataByte
        }
        dataMask = dataMask << 1;//rotate the data bit
        if (dataMask==0){
          dataMask=1;//make it roll around, is there a cleaner way than this? eg dataMask *=2?
        }
        nosBits++;
        if (nosBits == 8){ //one byte created, so move onto the next one
          manchester[nosBytes] = dataByte; //store this byte
          nosBits = 0;     //found 8, rezero and get another 8
          dataByte = 0;    //hold the bits in this one
          dataMask = 16;   //mask to do reversed nybbles on the fly
          nosBytes++;      //keep a track of how many bytes we have made
        }
      }
    }  
    else {
      //non valid data found, or maxBytes equalled by nosBytes, reset all pointers and exit the while loop
      headerHits = 0;    // make sure header search begins again
      header = false;    // make sure we look for another header
      firstZero = false; // make sure we look for another 1->0 transition before processing incoming stream
      nosBytes = 0;      // either way, start again at beginning of the bank
    }


    if ((nosBytes == maxBytes)||((nosBytes == maxBytes-1) && (manchester[0]==0xaf))) {
      
      Serial.println (manchester[0], HEX);
      rssi = analogRead(RSSIPin);
      if (manchester[0]==0xaf){ //detected the Thermometer and Hygrometer
        if(ValidCS(16)){
          thermom();
        }
      }
      if (manchester[0]==0xa1){  //detected the Anemometer and Wind Direction
      hexBank();
        if(ValidCS(18)){
          anemom();
        }
      }
      if (manchester[0]==0xa2){  //detected the Rain Gauge
        if(ValidCS(19)){
          rain();
        }
      }
      if (manchester[0]==0xd){  //detected the UV Meter
        if(ValidCS(19)){
          uv();
        }
      }
      headerHits = 0;
      header = false;
      nosBytes =0;       //reset byte pointer into bank
      notBCD = false;
    }
  }
  Serial.print ("Mem:");
  Serial.println (freeRam());
  invertLed();               //data processing ends, look for another header
  wdt_reset();
} //end of main loop

//Support Routines for Nybbles and CheckSum

// http://www.lostbyte.com/Arduino-OSV3/ (9) brian@lostbyte.com
// Directly lifted, then modified from Brian's work, due to nybbles bits now in correct order MSB->LSB
// CS = the sum of nybbles, 1 to (CSpos-1) & 0xf, compared to CSpos nybble;
bool ValidCS(int CSPos){
  bool ok = false;
  byte check = nyb(CSPos)+(nyb(CSPos+1)*16);
  byte cs = 0;
  for (int x=1; x<CSPos; x++){
    byte test=nyb(x);
    cs +=test;
  }
  //byte tmp = cs & 0xf;
  byte tmp =cs;

  if (tmp == check)  ok = true;
  return ok;
}



// Get a nybble from manchester bytes, short name so equations elsewhere are neater :-)
byte nyb(int nybble){
  int bite = nybble / 2;  //DIV 2, find the byte
  int nybb  = nybble % 2;  //MOD 2  0=MSB 1=LSB
  byte b = manchester[bite];
  if (nybb == 0){
    b = (byte)((byte)(b) >> 4);
  }
  else{
    b = (byte)((byte)(b) & (byte)(0xf));
  }
  if (b>9) notBCD=true;  
  return b;
}

//Calculation Routines

/*   PCR800 Rain Gauge  Sample Data:
 //  0        1        2        3        4        5        6        7        8        9
 //  A2       91       40       50       93       39       33       31       10       08 
 //  0   1    2   3    4   5    6   7    8   9    A   B    C   D    E   F    0   1    2   3    
 //  10100010 10010001 01000000 01010000 10010011 00111001 00110011 00110001 00010000 00001000 
 //  -------- -------  bbbb---  RRRRRRRR 88889999 AAAABBBB CCCCDDDD EEEEFFFF 00001111 2222CCCC
 
 // byte(0)_byte(1) = Sensor ID?????
 // bbbb = Battery indicator??? (7)
 // RRRRRRRR = Rolling Code ID
 // 222211110000.FFFFEEEEDDDD = Total Rain Fall (inches)
 // CCCCBBBB.AAAA99998888 = Current Rain Rate (inches per hour)
 // CCCC = CRC
 // Message length is 20 nybbles so working in inches
 Three tips caused the following
 Rain total: 11.72   rate: 39.33   tips: 300.41
 Rain total: 11.76   rate: 0.31   tips: 301.51
 Rain total: 11.80   rate: 0.31   tips: 302.54
 1 tip=0.04 inches or mm?
 My experiment
 24.2 converts reading below to mm (Best calibration so far)
 0.127mm per tip
 */
void rain(){
  notBCD = false;
  rainTotal = ((nyb(18)*100000)+(nyb(17)*10000)+(nyb(16)*1000)+(nyb(15)*100)+(nyb(14)*10)+nyb(13))*25.4/1000;
  Serial.print("Total Rain ");
  Serial.print(rainTotal);
  Serial.print(" mm, ");
  float rainRate = ((nyb(12)*10000)+(nyb(11)*1000)+(nyb(10)*100)+(nyb(9)*10)+nyb(8))*25.4/1000;
  Serial.print("Rain Rate ");
  Serial.print(rainRate);
  Serial.println(" mm/hr ");

  addFloatParam ("rainTotal", rainTotal,  outputString, 0);
  addFloatParam ("rainBattery", (float)battery(),  outputString,2 );

  if (notBCD) invertLed();
  else sendData (outputString);
  
}

int battery(){
  byte batt= nyb(4);
  Serial.print("battery:");
  Serial.println (batt, HEX);
  if (batt & 0x4) return (0);
  else return (1);
}

void addFloatParam (char *param, float value,  char *theString, int type) {
  char tmpString[20];

  dtostrf(value,6,2,tmpString);
  if (type ==0) strcpy(theString, param);
  else {
    strcat (theString, ":");
    strcat (theString, param);
  }  
  strcat(theString, ":");
  strcat(theString, tmpString);
  if (type == 2) strcat (theString, ":");
}



// WGR800 Wind speed sensor
// Sample Data:
// 0        1        2        3        4        5        6        7        8        9
// A1       98       40       8E       00       0C       70       04       00       34
// 0   1    2   3    4   5    6   7    8   9    A   B    C   D    E   F    0   1    2   3
// 10100001 10011000 01000000 10001110 00000000 00001100 01110000 00000100 00000000 00110100
// -------- -------- bbbb---- RRRRRRRR xxx8999- xxxxxxxx CCCCDDDD xxxxFFFF 0000---- CCCC----
// Av Speed 0.4000000000m/s Gusts 0.7000000000m/s  Direction: N  

// byte(0)_byte(1) = Sensor ID?????
// bbbb = Battery indicator??? (7)
// RRRRRRRR = Rolling Code
// 8999 = Direction
// DDDD.CCCC = Gust Speed (m per sec)
// 0000.FFFF = Avg Speed(m per sec)
// multiply by 3600/1000 for km/hr

void anemom(){
  getPressure();
  float windDirection = nyb(9)*22.5;
  notBCD = false;
  avWindspeed = ((nyb(17)*100)+(nyb(16)*10)+ nyb(15))*3.6/10;
  Serial.print("Av Speed ");
  Serial.print(avWindspeed);
  float gustWindspeed =((nyb(14)*100)+(nyb(13)*10)+nyb(12))*3.6/10;
  Serial.print(" km/hr, Gusts ");
  Serial.print(gustWindspeed);
  Serial.print(" km/hr, Direction: ");
  Serial.println(windDirection);


// 180


  addFloatParam ("windSpeed", avWindspeed,  outputString, 0);
  addFloatParam ("windDir", windDirection,  outputString,1 );
  addFloatParam ("gustWindSpeed", gustWindspeed,  outputString,1 ); 
  addFloatParam ("barometer", barometer,  outputString,1 );
  addFloatParam ("windBattery", (float)battery(),  outputString,1 );
  addFloatParam ("rssi", rssi,  outputString,2 );

  if (notBCD) invertLed();
  else sendData (outputString);
 


}

// THGN800 Temperature and Humidity Sensor
// 0        1        2        3        4        5        6        7        8        9          Bytes
// 0   1    2   3    4   5    6   7    8   9    A   B    C   D    E   F    0   1    2   3      nybbles
// 01011111 00010100 01000001 01000000 10001100 10000000 00001100 10100000 10110100 01111001   Bits
// -------- -------- bbbbcccc RRRRRRRR 88889999 AAAABBBB SSSSDDDD EEEE---- CCCC---- --------   Explanation
// byte(0)_byte(1) = Sensor ID?????
// bbbb = Battery indicator??? (7)
// byte(3) is rolling code R
// nybble(5) is channel selector c
// BBBBAAAA.99998888 Temperature in BCD
// SSSS sign for negative (- is !=0)
// EEEEDDDD Humidity in BCD
// nybble(16) is CRC C
// H 00 01 02 03 04 05 06 07 08 09    Byte Sequence
// D AF 82 41 CB 89 42 00 48 85 55    Real example
// Temperature 24.9799995422 degC Humidity 40.0000000000 % rel

void thermom(){
  notBCD = false;
  Serial.print("Temperature ");
  temperature = (float)((nyb(11)*100)+(nyb(10)*10)+nyb(9))/10; //accuracy to 0.01 degree seems unlikely
  if(nyb(12)==1){//  Trigger a negative temp sign
    Serial.print("-");
  }
  Serial.print(temperature);
  Serial.print(" degC, Humidity ");
  humidity = (nyb(14)*10)+nyb(13);
  Serial.print(humidity);
  Serial.println("% Rel");

  addFloatParam ("humidity", humidity,  outputString, 0);
  addFloatParam ("temperature", temperature,  outputString, 1);
     addFloatParam ("outTempBattery", (float)battery(),  outputString,2 );
  if (notBCD) invertLed();
  else sendData (outputString);
  

}

//?? UV  Sensor
// 0        1        2        3        4        5        6        7        8        9          Bytes
// 0   1    2   3    4   5    6   7    8   9    A   B    C   D    E   F    0   1    2   3      nybbles
// 01010111 00010100 01000001 01000000 10001100 10000000 00001100 10100000 10110100 01111001   Bits
// -------- -------- bbbbcccc RRRRRRRR 88889999 AAAABBBB -------- -------- CCCC---- --------   Explanation
// byte(0)_byte(1) = Sensor ID?????
// bbbb = Battery indicator??? (7)
// byte(3) is rolling code R
// nybble(5) is channel selector c
// 9999.8888 UV Index in BCD
// nybble(16) is CRC C

void uv(){
  notBCD = false;
  Serial.print("UV ");
  uvIndex = (float)(nyb(9)*10)+nyb(8); 

  Serial.println(uvIndex);
  battery();

  addFloatParam ("uvIndex", humidity,  outputString, 0);
  strcat(outputString, ":");
  if (notBCD) invertLed();
  else {
    Serial.println (outputString);  
    sendData (outputString);
  }  

}

// Handy Debugging Routines

void binBank(){
  //Print the fully aligned binary data in manchester[] array
  Serial.print("D ");
  for( int i=0; i < maxBytes; i++){ 
    byte mask = B10000000;
    if (manchester[i]<16){
      Serial.print("0"); //pad single digit hex
    }
    Serial.print(manchester[i],HEX);
    Serial.print(" ");
    for (int k=0; k<8; k++){
      if (manchester[i] & mask){
        Serial.print("1");
      }
      else{
        Serial.print("0");
      }
      mask = mask >> 1;
    }
    Serial.print(" ");
  }
  Serial.println();
}

void hexBank(){
  //Print the fully aligned binary data, enable the headers if desired
  //Serial.println("H 00 01 02 03 04 05 06 07 08 09");
  //Serial.println("  00 00 00 00 00 00 00 00 11 11");
  //Serial.println("B 10 32 54 76 98 BA DC FE 10 32");
  Serial.print("D ");
  for( int i=0; i < maxBytes; i++){ 
    if (manchester[i]<16){
      Serial.print("0"); //pad single digit hex
    }
    Serial.print(manchester[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void hexBankString(char *stringToPrint){
  char tmp[6];
  //Print the fully aligned binary data, enable the headers if desired
  //Serial.println("H 00 01 02 03 04 05 06 07 08 09");
  //Serial.println("  00 00 00 00 00 00 00 00 11 11");
  //Serial.println("B 10 32 54 76 98 BA DC FE 10 32");
  //Serial.print("D ");
  strcpy (stringToPrint,"");
  for( int i=0; i < maxBytes; i++){
    sprintf (tmp, " %.2X", manchester[i]); 
    strcat (stringToPrint, tmp);
  }
}





