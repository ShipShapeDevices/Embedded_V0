

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "DHT.h"

#include "PN532_SPI.h"
#include "snep.h"
#include "NdefMessage.h"

#include <SFE_MMA8452Q.h> // Includes the SFE_MMA8452Q library accel

// Begin using the library by creating an instance of the MMA8452Q
//  class. We'll call it "accel". That's what we'll reference from
//  here on out.
MMA8452Q accel;
#define MMA8452_ADDRESS 0x1D ;





#define NFC_SS  9
#define SD_SS  10

 //PN532_SPI pn532spi(SPI, NFC_SS);
 //SNEP nfc(pn532spi);
//  
////buffer for nfc
//uint8_t ndefBuf[128];


// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 10000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

//Pin defs 
#define DHTPIN 3     // what digital pin we're connected to for DHT
// for the data logging shield, we use digital pin 10 for the SD cs line
#define chipSelect 10
bool readNFC = true;
// humidity 
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

RTC_PCF8523  RTC; // define the Real Time Clock object


String myFilename;

bool startLOG = false;

void error(char *str)
{
  Serial.print(F("error: "));
  Serial.println(str);
  
  while(1);
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println(F(""));
  Serial.println(F("Starting"));

  
  
#if WAIT_TO_START
  Serial.println(F("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  

  pinMode(SD_SS, OUTPUT);



  Initialize();
   //InitNFC();

    // Choose your adventure! There are a few options when it comes
  // to initializing the MMA8452Q:
  //  1. Default init. This will set the accelerometer up
  //     with a full-scale range of +/-2g, and an output data rate
  //     of 800 Hz (fastest).
  accel.init();
 
  //CreateFile();
}

void loop(void)
{

   while (!startLOG){
    Send_ReadNFC();
   }
    Serial.println(F("Done"));
    syncTime = millis();


   LogData();
 
//  while (millis()<20000){
//    
//
//  
//
//
//  }



  
  ReadLogFile(myFilename);
    while(true){

    }
  readNFC=false;
  while(true){
    Send_ReadNFC();
  }


  
}//end loop


//void SendNFC(){
//  
//  PN532_SPI pn532spi(SPI, 9);
//  SNEP nfc(pn532spi);
//  uint8_t ndefBuf[20];
//      Serial.println("Send a message to Peer");
//    
//    NdefMessage message = NdefMessage();
//    //message.addUriRecord("http://shop.oreilly.com/product/mobile/0636920021193.do");
//    //message.addUriRecord("http://arduino.cc");
//    //message.addUriRecord("https://github.com/don/NDEF");
//    message.addTextRecord("hi");
//    
//    int messageSize = message.getEncodedSize();
//    if (messageSize > sizeof(ndefBuf)) {
//        Serial.println("ndefBuf is too small");
//        while (1) {
//        }
//    }
//
//    message.encode(ndefBuf);
//    if (0 >= nfc.write(ndefBuf, messageSize)) {
//        Serial.println("Failed");
//    } else {
//        Serial.println("Success");
//    }
//
//    delay(3000);
// 
//}

void Send_ReadNFC(){
  
  //buffer for nfc

  PN532_SPI pn532spi(SPI, NFC_SS);
  SNEP nfc(pn532spi);
  uint8_t ndefBuf[15];
   if (readNFC){
  Serial.println(F("Waiting for message from Peer"));
  int msgSize = nfc.read(ndefBuf, sizeof(ndefBuf));
  Serial.println(msgSize);
    if (msgSize > 0) {
        NdefMessage msg  = NdefMessage(ndefBuf, msgSize);
        //msg.print();
        Serial.println(F("\nSuccess"));
        startLOG=true;
    } else {
        Serial.println(F("Failed"));
    }
    delay(30);
   }else{
      
     Serial.println(F("Send a message to Peer"));
    
    NdefMessage message = NdefMessage();
    //message.addUriRecord("http://shop.oreilly.com/product/mobile/0636920021193.do");
    //message.addUriRecord("http://arduino.cc");
    //message.addUriRecord("https://github.com/don/NDEF");
    message.addTextRecord("h");
    
    int messageSize = message.getEncodedSize();
    if (messageSize > sizeof(ndefBuf)) {
        Serial.println(F("ndefBuf is too small"));
        while (1) {
        }
    }

    message.encode(ndefBuf);
    if (0 >= nfc.write(ndefBuf, messageSize)) {
        Serial.println("Failed");
    } else {
        Serial.println("Success");
    }

    delay(3000);
   }




}

void LogData()
{
  File logfile;
  while (millis()<20000){
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
//  Serial.print(m);         // milliseconds since start
//  Serial.print(F(", "));  
#endif

  // fetch the time
  DateTime now  = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
//  Serial.print(F(now.unixtime()); // seconds since 1/1/1970
//  Serial.print(F(", "));
//  Serial.print(F('"'));
//  Serial.print(F(now.year(), DEC));
//  Serial.print(F("/"));
//  Serial.print(F(now.month(), DEC));
//  Serial.print(F("/"));
//  Serial.print(F(now.day(), DEC));
//  Serial.print(F(" "));
//  Serial.print(F(now.hour(), DEC));
//  Serial.print(F(":"));
//  Serial.print(F(now.minute(), DEC));
//  Serial.print(F(":"));
//  Serial.print(F(now.second(), DEC));
//  Serial.print(F('"'));
#endif //ECHO_TO_SERIAL

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float temperatureF = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperatureF)) {
    Serial.println(F("Failed to read from DHT sensor! "));
    InitDHT();
    delay(1000);
    return;
  }

  
  logfile.print(", ");    
  logfile.print(humidity);
  logfile.print(", ");    
  logfile.print(temperatureF);
#if ECHO_TO_SERIAL
  Serial.print(F(", "));    
  //Serial.print(humidity);
  Serial.print(F(", "));    
  //Serial.print(temperatureF);
#endif //ECHO_TO_SERIAL

  float accelData;  
  Serial.println("Check accel");
  if (accel.available())
  {
    // First, use accel.read() to read the new variables:
    accel.read();
    accelData =  sqrt(sq(accel.cx) + sq(accel.cy) + sq(accel.cz));
  }
  logfile.print(", ");    
  logfile.print(accelData);
  
  //log new line
  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println(F("data"));
#endif // ECHO_TO_SERIAL


  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) {
    //return;
  }
  else{
  syncTime = millis();

  
  // we are syncing data to the card & updating FAT!
  logfile.flush();
  Serial.println(F("Saved to SD ")); 
  }
  
  }
 Serial.println(F("Close File"));
 logfile.close();
  
}


void CreateFile(){
   // create a new file
  File logfile;
  
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    
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
  

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println(F("RTC failed"));
#endif  //ECHO_TO_SERIAL
  }

  

  logfile.println("millis,stamp,datetime,humidity,tempF,accel");    
#if ECHO_TO_SERIAL
  Serial.println(F("millis,stamp,datetime,humidity,tempF,accel"));
#endif //ECHO_TO_SERIAL
 Serial.println(F("Close File"));
 logfile.close();
}

void ReadLogFile(String filename){
  File logfile;
 
  // re-open the file for reading:
  logfile = SD.open(filename);
  if (logfile) {
    Serial.println(F("test.txt:"));

    // read from the file until there's nothing else in it:
    while (logfile.available()) {
      logfile.read();
      Serial.write(logfile.read());
    }
    // close the file:
    logfile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println(F("error opening test.txt"));
  }
    Serial.println(F("Done Reading"));
}

void InitNFC(){
//  //set nfc low to comunicate
//  digitalWrite(SD_SS, HIGH);
//  digitalWrite(NFC_SS, LOW);
//  // initialize NFC
//  Serial.println(F("Initializing NFC card...");
//  //PN532_SPI pn532spi(SPI, NFC_SS);
//  //SNEP nfc(pn532spi);
//  digitalWrite(SD_SS, HIGH);
//  digitalWrite(NFC_SS, HIGH);



}
void InitSD(){
  //set sd low to comunicate

  
   // initialize the SD card
  Serial.print(F("Initializing SD card..."));
  
  // see if the card is present and can be initialized:
  if (!SD.begin(4000000,chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println(F("card initialized."));

}

void InitDHT(){
  // initialize the DHT for humidity
  Serial.println(F("Initializing DHT22 card..."));
  dht.begin();
}


void CheckPins() {
  if(digitalRead(NFC_SS) == LOW) Serial.println(F("Reader is Active"));
  else Serial.println(F("NFC is inActive"));
  if(digitalRead(SD_SS) == LOW) Serial.println(F("SD Card is Active"));
  else Serial.println(F("SD Card is inActive"));
}
void Initialize() {
 
    InitDHT();
   InitSD();

}





