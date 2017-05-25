// Sends a NDEF Message to a Peer
// Requires SPI. Tested with Seeed Studio NFC Shield v2

#include "SPI.h"
#include "PN532_SPI.h"
#include "snep.h"
#include "NdefMessage.h"

#define TEXT_SIZE 100
#define LANGUAGE_ENCODING_SIZE 3
#define PAYLOAD_SIZE (TEXT_SIZE + LANGUAGE_ENCODING_SIZE)
#define MESSAGE_ENCODING_SIZE 4
#define NDEF_BUFFER_SIZE (PAYLOAD_SIZE + MESSAGE_ENCODING_SIZE)

//PN532_SPI pn532spi(SPI, 9);
//SNEP nfc(pn532spi);
uint8_t ndefBuf[NDEF_BUFFER_SIZE];
int messageSize;
    PN532_SPI pn532spi(SPI, 9);
  SNEP nfc(pn532spi);
  
void setup() {
  Serial.begin(115200);
  Serial.println("NFC Peer to Peer Example - Send Message");

  NdefMessage message = NdefMessage();

  // manually create a record, so Uno doesn't run out of memory
  NdefRecord record = NdefRecord();
  record.setTnf(TNF_WELL_KNOWN);
  
  uint8_t recordType[] = { 0x54 }; // "T" Text Record
  record.setType(recordType, sizeof(recordType));
  
  uint8_t payload[PAYLOAD_SIZE];
  // fill buffer with the character "b"
  memset(payload, 0x62, sizeof(payload));
  // manually add language encoding for TNF_WELL_KNOWN RTD_TEXT
  payload[0] = 0x02;
  payload[1] = 0x65; // e
  payload[2] = 0x6e; // n
  record.setPayload(payload, sizeof(payload));
  message.addRecord(record);

  messageSize = message.getEncodedSize();
  if (messageSize != NDEF_BUFFER_SIZE) {
    Serial.println("Buffer size is incorrect!");
    while (1) {
    }
  }
  
  message.encode(ndefBuf);
}

void loop() {
SendData();
}





void SendData(){

  Serial.println("Send a message to Peer");
  Serial.println(messageSize);
   uint16_t timeout = 50;
  if (0 >= nfc.write(ndefBuf, messageSize,timeout)) {
    Serial.println("Failed");
  } else {
    Serial.println("Success");
  }

  delay(00);
}

