// Receive a NDEF message from a Peer
// Requires SPI. Tested with Seeed Studio NFC Shield v2

#include "SPI.h"
#include "PN532_SPI.h"
#include "snep.h"
#include "NdefMessage.h"

PN532_SPI pn532spi(SPI, 9);
SNEP nfc(pn532spi);
uint8_t ndefBuf[127];

void setup() {
    Serial.begin(115200);
    Serial.println("NFC Peer to Peer Example - Receive Message");
}

void loop() {
    Serial.println("Waiting for message from Peer");
    int msgSize = nfc.read(ndefBuf, sizeof(ndefBuf));
    Serial.println(msgSize);
    if (msgSize > 0) {
        NdefMessage msg  = NdefMessage(ndefBuf, msgSize);
        msg.print();
        Serial.println("\nSuccess");
    } else {
        Serial.println("Failed");
    }
    delay(1000);
}


