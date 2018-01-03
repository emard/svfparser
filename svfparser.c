#include <stdint.h>
#include "svfparser.h"

#include <stdio.h>

// lowest level lexical parser states
// to eliminate comments and whitespaces
enum
{
 LS_SPACE=0,
 LS_SLASH,
 LS_COMMENT,
 LS_TEXT,
 LS_SEMICOLON,
};

// index = position in the stream (0 resets FSM)
// content must come in sequential order
// length = data length in packet
// final = nonzero if this is the last packet
// return value:
// 0 - no error, call me again when data available
// 1 - finished OK
// -1 - finished, error
int8_t parse_svf_packet(uint8_t *packet, uint32_t index, uint32_t length, uint8_t final)
{
  printf("%d %d\n", index, final);
  static uint8_t lstate = LS_SPACE;
  static uint32_t line_count = 0;
  if(index == 0)
  {
    lstate = LS_SPACE;
    line_count = 0;
  }
  uint32_t i;
  char c;
  for(i = 0; i < length; i++)
  {
    c = packet[i];
    printf("%c", c);
    switch(c)
    {
      case '!':
        lstate = LS_COMMENT;
        break;
      case '/':
        if(lstate == LS_SLASH)
          lstate = LS_COMMENT;
        else
          lstate = LS_SLASH;
        break;
      case '\n':
        line_count++; // this is newline, similar as space
      case ' ':
      case '\t':
        if(lstate == LS_SLASH)
        {
          puts("space after single '/'");
          break;
        }
        if(lstate == LS_SPACE)
        {
          // another space, do nothing
          break;
        }
        // this is first space probably after some some text.
        // check do we have now complete number or reserved word
        break;
      default:
        // the text
        lstate = LS_TEXT;
        break;
    }
  }
  printf("line count %d\n", line_count);
  return 0;
} 
