/*
  Xively sensor client with Strings
 
 This sketch connects an analog sensor to Xively (http://www.xively.com)
 using a Wiznet Ethernet shield. You can use the Arduino Ethernet shield, or
 the Adafruit Ethernet shield, either one will work, as long as it's got
 a Wiznet Ethernet module on board.
 
 This example has been updated to use version 2.0 of the xively.com API. 
 To make it work, create a feed with two datastreams, and give them the IDs
 sensor1 and sensor2. Or change the code below to match your feed.
 
 This example uses the String library, which is part of the Arduino core from
 version 0019.  
 
 Circuit:
 * Analog sensor attached to analog in 0
 * Ethernet shield attached to pins 10, 11, 12, 13
 
 created 15 March 2010
 modified 9 Apr 2012
 by Tom Igoe with input from Usman Haque and Joe Saavedra
 modified 8 September 2012
 by Scott Fitzgerald
 
 http://arduino.cc/en/Tutorial/XivelyClientString
 This code is in the public domain.
 
 */

#include <SPI.h>
#include <Ethernet.h>


// assign a MAC address for the ethernet controller.
// fill in your address here:
  byte mac[] = { 
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

// fill in an available IP address on your network here,
// for manual configuration:
IPAddress ip(192,168,98,40);

// initialize the library instance:
EthernetClient client;

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
IPAddress server(192,168,98,10);      // numeric IP for api.xively.com


byte    failCount = 0; //no of times we have failed to connect to remote server
unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds

void initEthernet() {


  // give the ethernet module time to boot up:
  delay(1000);
  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // DHCP failed, so use a fixed IP address:
    Ethernet.begin(mac, ip);
  }
  failCount = 0;
  Serial.println("Ethernet initialisation complete");
}



// this method makes a HTTP connection to the server:
void sendData(String thisData) {
  wdt_reset();
  // if there's a successful connection:
  if (client.connect(server, 2029)) {
    // send the HTTP PUT request:

    //client.print("Content-Length: ");
    //client.println(thisData.length());

    // here's the actual content of the PUT request:
    failCount = 0;
    client.println(thisData);
    client.stop();
  } 
  else {
    // if you couldn't make a connection:
    Serial.println("Connection failed");
    client.stop();
    failCount++;
    if (failCount > 10) while(1);            // reset the card
  }
  //client.stop();
  // note the time that the connection was made or attempted:
  lastConnectionTime = millis();
}
