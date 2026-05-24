
#include <stdio.h>
#include <esp_mac.h>

void utilGetMacAddressNoDelimiterUtf16le(uint16_t *outBuffer)
{
    const char *hexLut = "0123456789ABCDEF";
    uint8_t macAddrBin[6];

    esp_base_mac_addr_get(macAddrBin);

    for (int ii = 0; ii < 6; ii++) {
        outBuffer[0] = hexLut[(macAddrBin[5 - ii] >> 4) & 0xF];
        outBuffer[1] = hexLut[(macAddrBin[5 - ii]) & 0xF];
        outBuffer += 2;
    }
}
