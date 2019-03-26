#ifndef JTAGSPI_H
#define JTAGSPI_H

#include <stdint.h>

#define TCK 14
#define TMS 15
#define TDI 13
#define TDO 12

// structure ready for the spi accelerated jtag
struct S_jtaghw
{
  uint8_t *header; // ptr to header nibble (not NULL if exists)
  uint8_t header_bits; // number of header bits 0-7 (not 0 if exists)
  uint8_t *data; // ptr to data bytes (not NULL if exists)
  uint32_t data_bytes; // number of data bytes (not 0 if exists)
  uint8_t *trailer; // ptr to trailer byte (not NULL if exists)
  uint8_t trailer_bits; // number of trailer bits 0-7 (not 0 if exists)
  uint8_t pad; // padding value 0x00 or 0xFF
  uint32_t pad_bits; // number of padding bits (not 0 if exist)  
};

extern struct S_jtaghw JTAG_TDI, JTAG_TDO; // filled by svfparser, TDI field overwritten by jtaghw

void jtag_tdi_tdo(struct S_jtaghw *tdi, struct S_jtaghw *tdo);
void jtag_open();
void jtag_close();

#endif
