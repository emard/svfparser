#include <stdio.h> // printf
#include "svfparser.h" // reversenibble
#include "jtaghw_print.h"

#define DBG_PRINT 1
#if DBG_PRINT
#define PRINTF(f_, ...) printf((f_), ##__VA_ARGS__)
#else
#define PRINTF(f_, ...)
#endif

// bitbanging using SPI
void jtag_tdi_tdo(struct S_jtaghw *tdi, struct S_jtaghw *tdo)
{
  uint32_t j;
  PRINTF("      ");
  if(tdi->header_bits)
  {
    #if REVERSE_NIBBLE
    PRINTF("0x%01X ", ReverseNibble[tdi->header[0] & 0xF]);
    #else
    PRINTF("0x%01X ", tdi->header[0] >> 4);
    #endif
    if(tdi->header_bits != 4)
      PRINTF("<-warning 4 bits expected, found %d. ", tdi->header_bits);
  }
  if(tdi->data_bytes)
  {
    PRINTF("0x");
    for(j = 0; j < tdi->data_bytes; j++)
      #if REVERSE_NIBBLE
      PRINTF("%01X%01X", ReverseNibble[tdi->data[j] >> 4], ReverseNibble[tdi->data[j] & 0xF]);
      #else
      PRINTF("%01X%01X", tdi->data[j] & 0xF, tdi->data[j] >> 4);
      #endif
    PRINTF(" ");
  }
  if(tdi->trailer_bits)
  {
    uint8_t byte_remaining = tdi->trailer[0];    
    if(tdi->trailer_bits >= 4)
    {
      #if REVERSE_NIBBLE
      PRINTF("0x%01X ", ReverseNibble[tdi->trailer[0] >> 4]);
      if(tdi->trailer_bits > 4)
      {
        byte_remaining <<= 4;
        PRINTF("0xb");
        for(j = 4; j < tdi->trailer_bits; j++, byte_remaining <<= 1)
          PRINTF("%d", byte_remaining >> 7);
        PRINTF(" ");
      }
      #else
      PRINTF("0x%01X ", tdi->trailer[0] & 0xF);
      if(tdi->trailer_bits > 4)
      {
        byte_remaining >>= 4;
        PRINTF("0xb");
        for(j = 4; j < tdi->trailer_bits; j++, byte_remaining >>= 1)
          PRINTF("%d", byte_remaining & 1);
        PRINTF(" ");
      }
      #endif
    }
    else
    {
      PRINTF("0b");
      #if REVERSE_NIBBLE
      for(j = 0; j < tdi->trailer_bits; j++, byte_remaining <<= 1)
        PRINTF("%d", byte_remaining >> 7);
      #else
      for(j = 0; j < tdi->trailer_bits; j++, byte_remaining >>= 1)
        PRINTF("%d", byte_remaining & 1);
      #endif
      PRINTF(" ");
    }
  }
  if(tdi->pad_bits)
  {
    if((tdi->pad_bits & 7) != 0)
    {
      uint32_t bitcount = tdi->pad_bits & 7;
      if(bitcount >= 4)
      {
        PRINTF("0x%01X ", tdi->pad & 0xF);
        bitcount -= 4;
      }
      if(bitcount > 0)
      {
        PRINTF("0b");
        for(j = 0; j < bitcount; j++)
          PRINTF("%d", tdi->pad & 1);
        PRINTF(" ");
      }
    }
    if((tdi->pad_bits / 8) != 0)
    {
      PRINTF("0x");
      for(j = 0; j < (tdi->pad_bits / 8); j++)
        PRINTF("%02X", tdi->pad);
    }
  }
  PRINTF("\n");
}

void jtag_open()
{
  PRINTF("jtag open\n");
}

void jtag_close()
{
  PRINTF("jtag close\n");
}
