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
};

enum
{
 TS_SPACE=0,
 TS_WORD,
 TS_NUMBER,
 TS_BRACKET,
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
  printf("index %d final %d\n", index, final);
  static uint8_t lstate = LS_SPACE;
  static uint32_t line_count = 0;
  static uint8_t lbracket = 0;
  if(index == 0)
  {
    lstate = LS_SPACE;
    line_count = 0;
    lbracket = 0;
  }
  uint32_t i;
  char c;
  for(i = 0; i < length; i++)
  {
    c = packet[i];
    // ****** COMMENT REJECTION
    switch(c)
    {
      case '!':
        lstate = LS_COMMENT;
        break;
      case '/':
        if(lstate == LS_COMMENT)
          break;
        if(lstate == LS_SLASH)
          lstate = LS_COMMENT;
        else
          lstate = LS_SLASH;
        break;
      case '\n':
        c = ' '; // rewrite as simple space
        line_count++; // this is newline, similar as space
        if(lstate == LS_COMMENT)
        {
          lstate = LS_SPACE;
          break;
        } // FALL THRU
      case ' ':
      case '\t':
        c = ' '; // rewrite as simple space
        if(lstate == LS_COMMENT)
          break;
        if(lstate == LS_SLASH)
        {
          puts("?space after single '/'");
          lstate = LS_SPACE;
          break;
        }
        if(lstate == LS_SPACE)
        {
          // another space, do nothing
          break;
        }
        // this is first space probably after some some text.
        // check do we have now complete number or reserved word
        lstate = LS_SPACE;
        if(lbracket == 0)
          printf(" ");
        break;
      default:
        if(lstate == LS_COMMENT)
          break;
        if(c == '(')
          lbracket++;
        if(c == ')')
          lbracket--;
        lstate = LS_TEXT;
        break;
    }
    if(lstate == LS_TEXT)
    {
      printf("%c", c);
    }
  }
  printf("line count %d\n", line_count);
  return 0;
} 
