#include <stdio.h> // printf
#include "jtaghw_esp32.h"

#define DBG_PRINT 0
#if DBG_PRINT
#define PRINTF(f_, ...) printf((f_), ##__VA_ARGS__)
#else
#define PRINTF(f_, ...)
#endif

extern uint8_t ReverseNibble[]; // instantiated in svfparser.c
SPIClass *spi_jtag = NULL;
uint8_t jtag_is_open = 0;

// bitbanging using SPI,
// store TDO result back to TDI buffer (overwrite)
// check overwritten TDI buffer with MASK for matching TDO
void jtag_tdi_tdo(struct S_jtaghw *tdi, struct S_jtaghw *tdo)
{
  int32_t j;
  uint32_t data, cmp;
  int tdo_mismatch = 0;
  if(spi_jtag == NULL)
    return;
  if(tdi->header_bits)
  {
    data = tdi->header[0] & 0xF;
    spi_jtag->transferBits(data, &data, tdi->header_bits); // should be always 4 bits
    if(tdo)
    {
      cmp = tdo->header[0] & 0xF;
      if(data != cmp)
        tdo_mismatch |= 1;
    }
  }
  if(tdi->data_bytes)
  {
    spi_jtag->transferBytes(tdi->data, tdi->data, tdi->data_bytes);
    if(tdo)
      tdo_mismatch |= memcmp(tdi->data, tdo->data, tdi->data_bytes);
  }
  if(tdi->trailer_bits)
  {
    data = (tdi->trailer[0]) >> (8 - tdi->trailer_bits);
    spi_jtag->transferBits(data, &data, tdi->trailer_bits);
    if(tdo)
    {
      cmp = (tdo->trailer[0] >> (8 - tdo->trailer_bits);
      if(data != cmp)
        tdo_mismatch |= 1;
    }
  }
  if(tdi->pad_bits)
  {
    // TODO: pad from memory instead of low efficiency loop
    if((tdi->pad_bits & 7) != 0)
    {
      data = tdi->pad;
      spi_jtag->transferBits(data, &data, tdi->pad_bits);
      if(tdo)
      {
        cmp = tdo->pad;
        if(data != cmp)
          tdo_mismatch |= 1;
      }
    }
    if((tdi->pad_bits / 8) != 0)
    {
      for(j = 0; j < (tdi->pad_bits / 8); j++)
      {
        data = tdi->pad;
        spi_jtag->transferBits(data, &data, 8);
        if(tdo)
        {
          cmp = tdo->pad;
          if(data != cmp)
            tdo_mismatch |= 1;
        }
      }
    }
  }
}

void jtag_open()
{
  if(spi_jtag == NULL)
    return;
  if(jtag_is_open == 0)
  {
    spi_jtag->begin(TCK, TDO, TDI, 0); // SCLK, MISO, MOSI, SS
    // TODO: remove reversenibble conversion and use LSBFIRST
    spi_jtag->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
    jtag_is_open = 1;
  }
}

void jtag_close()
{
  if(spi_jtag == NULL)
    return;
  if(jtag_is_open != 0)
  {
    spi_jtag->endTransaction();
    spi_jtag->end();
    jtag_is_open = 0;
  }
}
