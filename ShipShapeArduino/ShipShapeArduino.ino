

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include "DHT.h"

#include "PN532_SPI.h"
#include "snep.h"
#include "NdefMessage.h"

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
const int chipSelect = 10;

// humidity 
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);

RTC_PCF8523  RTC; // define the Real Time Clock object

String myFilename;
// the logging file
//File logfile;
//// set up variables using the SD utility library functions:
//Sd2Card card;
//SdVolume volume;
//SdFile root;



void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  while(1);
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting");
 
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  

  pinMode(SD_SS, OUTPUT);

  
  //CheckPins();
  //InitSD(); 
  Initialize();
   //InitNFC();
 
 CreateFile();

}

void loop(void)
{
//     //set nfc low to comunicate
//  digitalWrite(SD_SS, HIGH);
//  digitalWrite(NFC_SS, LOW);
//  // initialize NFC
//  Serial.println("Initializing NFC card...");
//  //PN532_SPI pn532spi(SPI, NFC_SS);
//  SNEP nfc(pn532spi);
//  digitalWrite(SD_SS, HIGH);
//  digitalWrite(NFC_SS, HIGH);

//
//    ReadNFC();
//    Serial.println("Done");
  while (true){
 
    LogData();
  }

 //ReadLogFile("LOGGER13.CSV");

 
//while (true){
//     Serial.println("Waiting for message from Peer");
//    int msgSize = nfc.read(ndefBuf, sizeof(ndefBuf));
//    if (msgSize > 0) {
//        NdefMessage msg  = NdefMessage(ndefBuf, msgSize);
//        msg.print();
//        Serial.println("\nSuccess");
//    } else {
//        Serial.println("Failed");
//    }
//    delay(10);
//}

//  while(true){
//        Serial.println("Waiting for message from Peer");
//    int msgSize = nfc.read(ndefBuf, sizeof(ndefBuf));
//    if (msgSize > 0) {
//        NdefMessage msg  = NdefMessage(ndefBuf, msgSize);
//        msg.print();
//        Serial.println("\nSuccess");
//    } else {
//        Serial.println("Failed");
//    }
//    delay(500);
//  }

  
}//end loop

void ReadNFC(){
   
  //buffer for nfc
  uint8_t ndefBuf[128];
  PN532_SPI pn532spi(SPI, NFC_SS);
  SNEP nfc(pn532spi);
  Serial.println("Waiting for message from Peer");
  int msgSize = nfc.read(ndefBuf, sizeof(ndefBuf));
    if (msgSize > 0) {
        NdefMessage msg  = NdefMessage(ndefBuf, msgSize);
        msg.print();
        Serial.println("\nSuccess");
    } else {
        Serial.println("Failed");
    }
    delay(3300);




}

void LogData()
{
  File logfile;
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
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
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float humidity = dht.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float temperatureF = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperatureF)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  
  logfile.print(", ");    
  logfile.print(humidity);
  logfile.print(", ");    
  logfile.print(temperatureF);
#if ECHO_TO_SERIAL
  Serial.print(", ");    
  Serial.print(humidity);
  Serial.print(", ");    
  Serial.print(temperatureF);
#endif //ECHO_TO_SERIAL


  //log new line
  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL


  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  // we are syncing data to the card & updating FAT!
  logfile.flush();
  Serial.println("Saved to SD "); 
  
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
  
  Serial.print("Logging to: ");
   myFilename = filename;
  Serial.println(myFilename);
  

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }

  

  logfile.println("millis,stamp,datetime,humidity,tempF");    
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,humidity,tempF");
#endif //ECHO_TO_SERIAL
  //Serial.println("Close File");
  //logfile.close();
}

void ReadLogFile(String filename){
     //message.addMimeMediaRecord("text/json", "{'answer': 42}");
  File logfile;

    // re-open the file for reading:
  logfile = SD.open(myFilename);

  
  if (logfile) {
    Serial.println("File");

    // read from the file until there's nothing else in it:
    while (logfile.available()) {
         
      Serial.write(logfile.read());
    }
    // close the file:
    logfile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }
  while(true);
}

void InitNFC(){
  //set nfc low to comunicate
  digitalWrite(SD_SS, HIGH);
  digitalWrite(NFC_SS, LOW);
  // initialize NFC
  Serial.println("Initializing NFC card...");
  //PN532_SPI pn532spi(SPI, NFC_SS);
  //SNEP nfc(pn532spi);
  digitalWrite(SD_SS, HIGH);
  digitalWrite(NFC_SS, HIGH);



}
void InitSD(){
  //set sd low to comunicate
  digitalWrite(SD_SS, LOW);
  digitalWrite(NFC_SS, HIGH);
  
   // initialize the SD card
  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(4000000,chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  digitalWrite(SD_SS, HIGH);
  digitalWrite(NFC_SS, HIGH);
}
void InitDHT(){
  // initialize the DHT for humidity
  Serial.println("Initializing DHT22 card...");
  dht.begin();
}


void CheckPins() {
  if(digitalRead(NFC_SS) == LOW) Serial.println("Reader is Active");
  else Serial.println("NFC is inActive");
  if(digitalRead(SD_SS) == LOW) Serial.println("SD Card is Active");
  else Serial.println("SD Card is inActive");
}
void Initialize() {
 
  
   InitSD();
  InitDHT();
}

//void InitSD_info(){
//   Serial.print("\nInitializing SD card...");
//
//  // we'll use the initialization code from the utility libraries
//  // since we're just testing if the card is working!
//  if (!card.init(4000000, chipSelect)) {
//    Serial.println("initialization failed. Things to check:");
//    Serial.println("* is a card inserted?");
//    Serial.println("* is your wiring correct?");
//    Serial.println("* did you change the chipSelect pin to match your shield or module?");
//    return;
//  } else {
//    Serial.println("Wiring is correct and a card is present.");
//  }
//
//  // print the type of card
//  Serial.print("\nCard type: ");
//  switch (card.type()) {
//    case SD_CARD_TYPE_SD1:
//      Serial.println("SD1");
//      break;
//    case SD_CARD_TYPE_SD2:
//      Serial.println("SD2");
//      break;
//    case SD_CARD_TYPE_SDHC:
//      Serial.println("SDHC");
//      break;
//    default:
//      Serial.println("Unknown");
//  }
//
//  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
//  if (!volume.init(card)) {
//    Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
//    return;
//  }
//
//
//  // print the type and size of the first FAT-type volume
//  uint32_t volumesize;
//  Serial.print("\nVolume type is FAT");
//  Serial.println(volume.fatType(), DEC);
//  Serial.println();
//
//  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
//  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
//  volumesize *= 512;                            // SD card blocks are always 512 bytes
//  Serial.print("Volume size (bytes): ");
//  Serial.println(volumesize);
//  Serial.print("Volume size (Kbytes): ");
//  volumesize /= 1024;
//  Serial.println(volumesize);
//  Serial.print("Volume size (Mbytes): ");
//  volumesize /= 1024;
//  Serial.println(volumesize);
//  
//}




