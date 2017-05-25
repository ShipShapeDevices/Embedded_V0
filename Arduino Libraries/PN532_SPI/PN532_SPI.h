
#ifndef __PN532_SPI_H__
#define __PN532_SPI_H__

#include <SPI.h>
#include "PN532Interface.h"

class PN532_SPI : public PN532Interface {
public:
    PN532_SPI(SPIClass &spi, uint8_t ss);
    
    void begin();
    void wakeup();
    int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);

    int16_t readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);
    
private:
    SPIClass* _spi;
    uint8_t   _ss;
    uint8_t command;
    
    boolean isReady();
    void writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int8_t readAckFrame();

    
    /************** low level SPI ********/
    /*Function:Transmit a byte to PN532 through the SPI interface. */
    inline void write(uint8_t _data)
    {
        /* bit reversing code copied vetbatim from http://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious */
        _data  = ((_data * 0x0802LU & 0x22110LU) | (_data * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
        
         _spi->transfer(_data);
    }
    
    /*Function:Receive a byte from PN532 through the SPI interface */
    inline uint8_t read(void)
    {
        uint8_t data_ = _spi->transfer(0);
        /* bit reversing code copied vetbatim from http://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious */
        data_  = ((data_ * 0x0802LU & 0x22110LU) | (data_ * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16;
        
        return data_;
    }
    
    
};

#endif
