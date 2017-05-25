

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "DHT.h"
#include "BMA250.h" //accel

//sed up for BLE
#define CODEBENDER true

//#if defined(ARDUINO_ARCH_SAMD)
//  #define SerialMonitorInterface SerialUSB
//#else
//  #define SerialMonitorInterface Serial
//#endif

#define BLE_DEBUG true
#define SerialMonitorInterface Serial
#include <SPI.h>
#include "lib_aci.h"
#include "aci_setup.h"
#include "uart_over_ble.h"
#include "services.h"

uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;

#if CODEBENDER
#include "UART.h"
#endif


BMA250 accel;
// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 20000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

//Pin defs 
#define DHTPIN 3     // what digital pin we're connected to for DHT
// for the data logging shield, we use digital pin 10 for the SD cs line
#define chipSelect 10


// humidity 
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
// Initialize DHT sensor.
DHT dht(DHTPIN, DHTTYPE);












File logfile;

String myFilename;
float accelScale = 12.4; //31.5
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
  

  pinMode(chipSelect, OUTPUT);

  //set up BLE
  Serial.println(F("Initializing BLE card..."));
  BLEsetup();

    // initialize the SD card
  Serial.print(F("Initializing SD card..."));
  
  // see if the card is present and can be initialized:
  if (!SD.begin(SPI_HALF_SPEED, chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println(F("card initialized."));

  // initialize the DHT for humidity
  Serial.println(F("Initializing DHT22 card..."));
  dht.begin();
  

  
//options accel
////  #define BMA250_I2CADDR 0x18
////#define BMA250_update_time_64ms 0x08
////#define BMA250_update_time_32ms 0x09
////#define BMA250_update_time_16ms 0x0A
////#define BMA250_update_time_8ms 0x0B
////#define BMA250_update_time_4ms 0x0C
////#define BMA250_update_time_2ms 0x0D
////#define BMA250_update_time_1ms 0x0E
////#define BMA250_update_time_05ms 0xF
////#define BMA250_range_2g 0x03
////#define BMA250_range_4g 0x05
////#define BMA250_range_8g 0x08
////#define BMA250_range_16g 0x0C
   Serial.println(F("Initializing BMA250 Accel card..."));
  Wire.begin();
  accel.begin(BMA250_range_4g, BMA250_update_time_32ms);//This sets up the BMA250 accelerometer



 
  CreateFile();
}

void loop(void)
{

    Serial.println(F("Ready"));
    syncTime = millis();


    LogData();
 
 
    ReadLogFile(myFilename);
    while(true){

    }

  
}//end loop


void LogData()
{

  while (millis()<25000){
  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(F(", "));  
#endif

//  // fetch the time
//  DateTime now  = RTC.now();
//  // log time
//  logfile.print(now.unixtime()); // seconds since 1/1/1970
//  logfile.print(", ");
//  logfile.print('"');
//  logfile.print(now.year(), DEC);
//  logfile.print("/");
//  logfile.print(now.month(), DEC);
//  logfile.print("/");
//  logfile.print(now.day(), DEC);
//  logfile.print(" ");
//  logfile.print(now.hour(), DEC);
//  logfile.print(":");
//  logfile.print(now.minute(), DEC);
//  logfile.print(":");
//  logfile.print(now.second(), DEC);
//  logfile.print('"');
//#if ECHO_TO_SERIAL
////  Serial.print(F(now.unixtime()); // seconds since 1/1/1970
////  Serial.print(F(", "));
////  Serial.print(F('"'));
////  Serial.print(F(now.year(), DEC));
////  Serial.print(F("/"));
////  Serial.print(F(now.month(), DEC));
////  Serial.print(F("/"));
////  Serial.print(F(now.day(), DEC));
////  Serial.print(F(" "));
////  Serial.print(F(now.hour(), DEC));
////  Serial.print(F(":"));
////  Serial.print(F(now.minute(), DEC));
////  Serial.print(F(":"));
////  Serial.print(F(now.second(), DEC));
////  Serial.print(F('"'));
//#endif //ECHO_TO_SERIAL

//  // Reading temperature or humidity takes about 250 milliseconds!
//  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
//  float humidity = dht.readHumidity();
//  // Read temperature as Fahrenheit (isFahrenheit = true)
//  float temperatureF = dht.readTemperature(true);
//
//  // Check if any reads failed and exit early (to try again).
//  if (isnan(humidity) || isnan(temperatureF)) {
//    Serial.println(F("Failed to read from DHT sensor! "));
//    //return;
//  }

  accel.read();//This function gets new data from the accelerometer
   
  logfile.print("0.5");
  logfile.print(", ");    
  logfile.print((accel.rawTemp*0.5)+24.0,1);
#if ECHO_TO_SERIAL
   
  Serial.print("0.5");
  Serial.print(F(", "));    
  Serial.print((accel.rawTemp*0.5)+24.0,1);
#endif //ECHO_TO_SERIAL




  float accelData; 
  accelData =  sqrt(sq(accel.X/accelScale) + sq(accel.Y/accelScale) + sq(accel.Z/accelScale));


  logfile.print(", ");    
  logfile.print(accelData);
#if ECHO_TO_SERIAL
  Serial.print(F(", "));    
  Serial.print(accelData);
#endif //ECHO_TO_SERIAL
  
  //log new line
  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println(F(" data"));
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
  


  logfile.println("millis,humidity,tempF,accel");    
#if ECHO_TO_SERIAL
  Serial.println(F("millis,humidity,tempF,accel"));
#endif //ECHO_TO_SERIAL

}

void ReadLogFile(String filename){

 
  // re-open the file for reading:
  logfile = SD.open(filename);
  if (logfile) {
    // read from the file until there's nothing else in it:
    while (logfile.available()) {
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










