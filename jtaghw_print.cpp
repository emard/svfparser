#include <stdio.h> // printf
#include "jtaghw_print.h"

#define DBG_PRINT 1
#if DBG_PRINT
#define PRINTF(f_, ...) printf((f_), ##__VA_ARGS__)
#else
#define PRINTF(f_, ...)
#endif

extern uint8_t ReverseNibble[]; // instantiated in svfparser.c

// bitbanging using SPI
void jtag_tdi_tdo(struct S_jtaghw *tdi, struct S_jtaghw *tdo)
{
  uint32_t j;
  PRINTF("      ");
  if(tdi->header_bits)
  {
    PRINTF("0x%01X ", ReverseNibble[tdi->header[0] & 0xF]);
    if(tdi->header_bits != 4)
      PRINTF("<-warning 4 bits expected, found %d. ", tdi->header_bits);
  }
  if(tdi->data_bytes)
  {
    PRINTF("0x");
    for(j = 0; j < tdi->data_bytes; j++)
      PRINTF("%01X%01X", ReverseNibble[tdi->data[j] >> 4], ReverseNibble[tdi->data[j] & 0xF]);
    PRINTF(" ");
  }
  if(tdi->trailer_bits)
  {
    if(tdi->trailer_bits == 4)
      PRINTF("0x%01X ", ReverseNibble[tdi->trailer[0] >> 4]);
    else
    {
      uint8_t byte_remaining = tdi->trailer[0];    
      PRINTF("0b");
      for(j = 0; j < tdi->trailer_bits; j++, byte_remaining <<= 1)
        PRINTF("%d", byte_remaining >> 7);
      PRINTF(" ");
    }
  }
  if(tdi->pad_bits)
  {
    if((tdi->pad_bits & 7) != 0)
    {
      PRINTF("0b");
      for(j = 0; j < (tdi->pad_bits & 7); j++)
        PRINTF("%d", tdi->pad & 1);
      PRINTF(" ");
    }
    if((tdi->pad_bits / 8) != 0)
    {
      PRINTF("0x");
      for(j = 0; j < (tdi->pad_bits / 8); j++)
        PRINTF("%1X%1X", (tdi->pad) >> 4, (tdi->pad) & 0xF);
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
