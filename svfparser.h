#ifndef SVFPARSER_H
#define SVFPARSER_H
#include <stdint.h>

#define REVERSE_NIBBLE 0
extern uint8_t ReverseNibble[]; // instantiated in svfparser.c

int8_t parse_svf_packet(uint8_t *packet, uint32_t index, uint32_t length, uint8_t final);

#endif
