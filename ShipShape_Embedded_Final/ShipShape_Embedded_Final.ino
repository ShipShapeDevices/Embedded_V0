#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "DHT.h"
#include "BMA250.h" //accel
#include <ESP8266WiFi.h>


/* Set these to your desired credentials. */
const char *ssid = "ShipShapeWIFI";

//set up server port
WiFiServer server(80);
WiFiClient client;


// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  100 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
#define SYNC_INTERVAL 10*LOG_INTERVAL // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   0 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

//Pin defs
#define DHTPIN D3 // what digital pin we're connected to for DHT
#define chipSelect D8 // for the data logging shield, we use digital pin 10 for the SD cs line
#define DHTTYPE DHT22  // humidity = DHT 22  (AM2302), AM2321

//accel
BMA250 accel;

// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE, 11);

//SD Logging
File logfile;

String packageID;
typedef enum {LOGGING, READING, WAITING, SETUP} states;
states currentState;
String myFilename;
float accelScale = 12.4; 
bool startLOG = false;

void error(char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);

  while (1);
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println(F(""));
  Serial.println(F("Starting"));
  // initialize the DHT for humidity
  Serial.println(F("Initializing DHT22 card..."));
  dht.begin();

  //WIFI
  Serial.print("Configuring access point...");
  WiFi.mode(WIFI_AP);
  //set wifi with no password
  WiFi.softAP(ssid);
  //start soft AP server
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.begin();
  Serial.println("HTTP server started");

#if WAIT_TO_START
  Serial.println(F("Type any character to start");
                 while (!Serial.available());
#endif //WAIT_TO_START

                 // make sure that the default chip select pin is set to
                 // output, even if you don't use it:


                 pinMode(chipSelect, OUTPUT);



                 // initialize the SD card
                 Serial.print(F("Initializing SD card..."));

                 // see if the card is present and can be initialized:
  if (!SD.begin( chipSelect)) {
    error("Card failed, or not present");
    }
  Serial.println(F("card initialized."));
  
  //accel
  Serial.println(F("Initializing BMA250 Accel card..."));
  Wire.begin();
  accel.begin(BMA250_range_4g, BMA250_update_time_32ms);//This sets up the BMA250 accelerometer
  //set start state to waiting
  currentState = WAITING;
}

void LogData(void)
{

  //will not connected to wifi
  while ( !client  ) {
    //check for client
    client = server.available();
    // delay for the amount of time we want between readings
    delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));

    // log milliseconds since starting
    uint32_t m = millis();
    logfile.print(m);           // milliseconds since start
    logfile.print(", ");
#if ECHO_TO_SERIAL
    Serial.print(m);         // milliseconds since start
    Serial.print(F(", "));
#endif


    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    float humidity = dht.readHumidity();
    // Read temperature as Fahrenheit (isFahrenheit = true)
    float temperatureF = dht.readTemperature(true);

    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temperatureF)) {
      Serial.println(F("Failed to read from DHT sensor! "));
      //return;
    }

    accel.read();//This function gets new data from the accelerometer
  
    logfile.print(humidity);
    logfile.print(", ");
    logfile.print(temperatureF);//(accel.rawTemp * 0.5) + 24.0, 1);
#if ECHO_TO_SERIAL

    Serial.print(humidity);
    Serial.print(F(", "));
    Serial.print(temperatureF);//(accel.rawTemp * 0.5) + 24.0, 1);
#endif //ECHO_TO_SERIAL

    float accelData;
    accelData =  sqrt(sq(accel.X / accelScale) + sq(accel.Y / accelScale) + sq(accel.Z / accelScale));

    logfile.print(", ");
    logfile.print(accelData);
#if ECHO_TO_SERIAL
    Serial.print(F(", "));
    Serial.print(accelData);
#endif //ECHO_TO_SERIAL
    //log new line
    logfile.println();
#if ECHO_TO_SERIAL
    Serial.println(" ");
#endif // ECHO_TO_SERIAL
    // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
    // which uses a bunch of power and takes time
    if ((millis() - syncTime) < SYNC_INTERVAL) {
      //return;
    }
    else {
      syncTime = millis();
      // we are syncing data to the card & updating FAT!
      logfile.flush();
      Serial.println(F("Saved to SD "));
      Serial.println(millis());
    }

  }
  Serial.println(F("Close File"));
  logfile.close();


}


void CreateFile(void) {

  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i / 10 + '0';
    filename[7] = i % 10 + '0';

    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE);
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    error("couldnt create file");
  }

  Serial.print(F("Logging to: "));
  myFilename = filename;
  Serial.println(myFilename);


  logfile.println("packageID");
  logfile.println(packageID);
  logfile.println("millis,humidity,tempF,accel");
  logfile.println("start");

#if ECHO_TO_SERIAL
  Serial.println(F("packageID"));
  Serial.println(packageID);
  Serial.println(F("millis,humidity,tempF,accel"));
  Serial.println(F("start"));

#endif //ECHO_TO_SERIAL

}
void ReadLogFile(String filename) {

  client.setNoDelay(1);
  // re-open the file for reading:
  logfile = SD.open(filename);

  Serial.println(millis());

  if (logfile) {
    int bufferSize =  1760;
    byte clientBuf[bufferSize ];
    int clientCount = 0;
    // read from the file until there's nothing else in it:
    while (logfile.available()) {
      clientBuf[clientCount] = logfile.read();
      clientCount++;
      if (clientCount > bufferSize - 1)
      {
        client.write(clientBuf, bufferSize );
        clientCount = 0;
        //Serial.println("buff");
      }
      //int data = logfile.read();
      // client.write(logfile.read());
      //Serial.write(data);

    }
    if (clientCount > 0) client.write(clientBuf, clientCount);
    Serial.println("done");
    Serial.println(millis());

    // close the file:
    logfile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
  }
  delay(100);
  Serial.println(F("Done Reading"));
}



void loop(void)
{
  //switch on current states
  switch (currentState) {
    
    case WAITING: {
        //wait for connection to log return if none
        client = server.available();
        if (!client) {
          return;// do nothin
        }
        else {
          Serial.println("connected to Wifi");

          //get data from browser
          packageID = client.readStringUntil('\r');
          //print
          Serial.println(packageID);
          CreateFile();
          syncTime = millis();
          //end connection
          String data = "logging";
          client.println(data);
          Serial.println(data);
          currentState = SETUP;

        }

      }
      break;

    case LOGGING:
      {
        //get data from browser
        String request = client.readStringUntil('\r');
        //print
        Serial.println(request);
        // on connect
        currentState  = READING;
      }
      break;
      
    case SETUP: {
        //wait for disconnenct
        client = server.available();
        if ( client  ) {
          //do nothing

        } else {
          Serial.println("Start Logging");
          // start loging
          LogData();
          currentState  = LOGGING;
        }

      }
      break;
      
    case READING: {
        ReadLogFile(myFilename);
        //end connection
        String data = "end";
        client.println(data);
        Serial.println(data);
        Serial.println("Waiting");
        //wait for disconnenct
        while ( client  ) {
          //check for client
          client = server.available();
        }
        Serial.println("Disconnected");
        currentState  = WAITING;

      }
      break;
  }



}//end loop











