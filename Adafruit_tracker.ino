
#include "Adafruit_MQTT.h"                                  // Adafruit MQTT library
#include "Adafruit_MQTT_Client.h"                           // Adafruit MQTT library
#include "ESP8266WiFi.h"                                    // ESP8266 library     

#include <TinyGPS++.h>                                      // Tiny GPS Plus Library
#include <SoftwareSerial.h>                                 // Software Serial Library so we can use Pins for communication with the GPS module

static const int RXPin = 12, TXPin = 13;                    // Ublox 6m GPS module to pins 12 and 13
static const uint32_t GPSBaud = 9600;                       // Ublox GPS default Baud Rate is 9600

TinyGPSPlus gps;                                            // Create an Instance of the TinyGPS++ object called gps
SoftwareSerial ss(RXPin, TXPin);                            // The serial connection to the GPS device

const double HOME_LAT = 12.809338;                          // Enter Your Latitude and Longitude here
const double HOME_LNG = 74.878120;                          // to track how far away the Bus is away from "Home" 

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "Razzberry"                          // Enter Your router SSID
#define WLAN_PASS       "aztec123"                           // Enter Your router Password

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER       "io.adafruit.com"
#define AIO_SERVERPORT   1883                                // use 8883 for SSL
#define AIO_USERNAME     "rashadisrazzi"
#define AIO_KEY          "a3930b2c51d04e7aa4bdd3da9ddca03c"

/************ Global State (you don't need to change this!) ******************/

WiFiClient client;                                          // Create an ESP8266 WiFiClient class to connect to the MQTT server.

const char MQTT_SERVER[]    = AIO_SERVER;                   // Store the MQTT server, username, and password in flash memory.
const char MQTT_USERNAME[]  = AIO_USERNAME;
const char MQTT_PASSWORD[]  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

//Setup a feed called 'gpslat' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>   // This feed is not needed, only setup if you want to see it
const char gpslat_FEED[] = AIO_USERNAME "/feeds/gpslat";            
Adafruit_MQTT_Publish gpslat = Adafruit_MQTT_Publish(&mqtt, gpslat_FEED);

// Setup a feed called 'gpslng' for publishing.
//Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>   // This feed is not needed, only setup if you want to see it
const char gpslng_FEED[]  = AIO_USERNAME "/feeds/gpslng";
Adafruit_MQTT_Publish gpslng = Adafruit_MQTT_Publish(&mqtt, gpslng_FEED);

// Setup a feed called 'gps' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char gps_FEED[]  = AIO_USERNAME "/feeds/gpslatlng/csv";               // CSV = commas seperated values
Adafruit_MQTT_Publish gpslatlng = Adafruit_MQTT_Publish(&mqtt, gps_FEED);

/****************************************************/

void setup() 
{
  Serial.begin(115200);                                 // Setup Serial Comm for Serial Monitor @ 115200 baud
  WiFi.mode(WIFI_STA);                                  // Setup ESP8266 as a wifi station
  WiFi.disconnect();                                    // Disconnect if needed
  delay(100);                                           // short delay
  
  Serial.print("\n  Adafruit IO GPS\n");  
  Serial.print("      Tracker    \n"); 
  Serial.print("---------------------\n");
  delay(1000);                                          // Pause X seconds  
    
  ss.begin(GPSBaud);                                    // Set Software Serial Comm Speed to 9600     
   
  Serial.print("Connecting to WiFi\n");

  WiFi.begin(WLAN_SSID, WLAN_PASS);                     // Start a WiFi connection and enter SSID and Password
      while (WiFi.status() != WL_CONNECTED) 
         {                                              // While waiting on wifi connection, display "..."
           delay(500);
           Serial.print(".");
         } 
           Serial.print("\nConnected\n");
             
}                                                       // End Setup

void loop() {

  smartDelay(5000);                                      // Update GPS data TinyGPS needs to be fed often
  MQTT_connect();                                       // Run Procedure to connect to Adafruit IO MQTT  

  float Speed_Bus = (gps.speed.kmph());                                      // variable to store Speed
  float GPSlat = (gps.location.lat());                  // variable to store latitude
  float GPSlng = (gps.location.lng());                  // variable to store longitude
  float GPSalt = (gps.altitude.feet());                 // variable to store altitude  
  float Distance = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),HOME_LAT, HOME_LNG);  //Query Tiny GPS to Calculate Distance to Home
    

  Serial.print("  GPS Tracking\n");
  Serial.print("---------------------\n"); 

  Serial.print("GPS Lat: ");
  Serial.println(gps.location.lat(), 6);               // Display latitude to 6 decimal points
  Serial.print("\nGPS Lon: ");
  Serial.println(gps.location.lng(), 6);               // Display longitude to 6 decimal points
  Serial.print("\nSpeed: ");
  Serial.print(Speed_Bus);                             // Speed of bus measured in Kilometers per hour
  Serial.print("\nDistance: ");
  Serial.print(Distance);                             // Distance to Home measured in Meters    

 
// ********************** Combine Data to send to Adafruit IO *********************************
// Here we need to combine Speed, Latitude, Longitude, Altitude into a string variable buffer to send to Adafruit    
                                                            
            char gpsbuffer[30];                         // Combine Latitude, Longitude, Altitude into a buffer of size X
            char *p = gpsbuffer;                        // Create a buffer to store GPS information to upload to Adafruit IO                       

            dtostrf(Speed_Bus, 3, 4, p);         // Convert Distance to Home to a String Variable and add it to the buffer
            p += strlen(p);
            p[0] = ','; p++;                      
            
            dtostrf(GPSlat, 3, 6, p);                   // Convert GPSlat(latitude) to a String variable and add it to the buffer
            p += strlen(p);
            p[0] = ','; p++;
                                                            
            dtostrf(GPSlng, 3, 6, p);                   // Convert GPSlng(longitude) to a String variable and add it to the buffer
            p += strlen(p);
            p[0] = ','; p++;  
                                                            
            dtostrf(GPSalt, 2, 1, p);                   // Convert GPSalt(altimeter) to a String variable and add it to the buffer
            p += strlen(p);
                                                                        
            p[0] = 0;                                   // null terminate, end of buffer array

            if ((GPSlng != 0) && (GPSlat != 0))         // If GPS longitude or latitude do not equal zero then Publish
              {
              Serial.println("\nSending GPS Data ");    
              gpslatlng.publish(gpsbuffer);             // publish Combined Data to Adafruit IO
              Serial.println(gpsbuffer);  
              }
            
            gpslng.publish(GPSlng,6);                   // Publish the GPS longitude to Adafruit IO                 
            
            if (! gpslat.publish(GPSlat,6))             // Publish the GPS latitude to Adafruit IO
               {
                 Serial.println(F("Failed"));          // If it failed to publish, print Failed
               } else 
                  {
                   Serial.println(F("\nData Sent!"));                   
                   }  

        delay(1000);         
    
  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring\n"));
  
  // Wait a bit before scanning again
  Serial.print("\nPausing...\n");

  smartDelay(500);                                      // Feed TinyGPS constantly
  delay(1000);
}


// **************** Smart delay - used to feed TinyGPS ****************

static void smartDelay(unsigned long ms)                 
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


// **************** MQTT Connect - connects with Adafruit IO *****************
void MQTT_connect() {
  
  int8_t ret;
  if (mqtt.connected()) { return; }                     // Stop and return to Main Loop if already connected to Adafruit IO
  Serial.print("\nConnecting to MQTT...\n");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {                 // Connect to Adafruit, Adafruit will return 0 if connected
       Serial.println(mqtt.connectErrorString(ret));   // Display Adafruits response
       Serial.println("\nRetrying MQTT...");
       mqtt.disconnect();
       delay(5000);                                     // wait X seconds
       retries--;
       if (retries == 0) {                              // basically die and wait for WatchDogTimer to reset me                                                          
         while (1);         
       }
  }
  Serial.print("\nMQTT Connected!");
  delay(1000);
}
