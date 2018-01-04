#include <stdint.h>
#include "svfparser.h"
#include <string.h>
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

// enumerated (tokenized) reserved words
enum 
{
  CMD_ENDDR=0, // Specifies default end state for DR scan operations.
  CMD_ENDIR, // Specifies default end state for IR scan operations.
  CMD_FREQUENCY, // Specifies maximum test clock frequency for IEEE 1149.1 bus operations.
  CMD_HDR, // (Header Data Register) Specifies a header pattern that is prepended to the beginning of subsequent DR scan operations.
  CMD_HIR, // (Header Instruction Register) Specifies a header pattern that is prepended to the beginning of subsequent IR scan operations.
  CMD_PIO, // (Parallel Input/Output) Specifies a parallel test pattern.
  CMD_PIOMAP, // (Parallel Input/Output Map) Maps PIO column positions to a logical pin.
  CMD_RUNTEST, // Forces the IEEE 1149.1 bus to a run state for a specified number of clocks or a specified time period.
  CMD_SDR, // (Scan Data Register) Performs an IEEE 1149.1 Data Register scan.
  CMD_SIR, // (Scan Instruction Register) Performs an IEEE 1149.1 Instruction Register scan.
  CMD_STATE, // Forces the IEEE 1149.1 bus to a specified stable state.
  CMD_TDR, // (Trailer Data Register) Specifies a trailer pattern that is appended to the end of subsequent DR scan operations.
  CMD_TIR, // (Trailer Instruction Register) Specifies a trailer pattern that is appended to the end of subsequent IR scan operations.
  CMD_TRST, // (Test ReSeT) Controls the optional Test Reset line.
  CMD_NUM // LAST: represents number of reserved words
};

char *Commands[] =
{
  [CMD_ENDDR] = "ENDDR",
  [CMD_ENDIR] = "ENDIR",
  [CMD_FREQUENCY] = "FREQUENCY",
  [CMD_HDR] = "HDR",
  [CMD_HIR] = "HIR",
  [CMD_PIO] = "PIO",
  [CMD_PIOMAP] = "PIOMAP",
  [CMD_RUNTEST] = "RUNTEST",
  [CMD_SDR] = "SDR",
  [CMD_SIR] = "SIR",
  [CMD_STATE] = "STATE",
  [CMD_TDR] = "TDR",
  [CMD_TIR] = "TIR",
  [CMD_TRST] = "TRST",
};

// quick search: first 4 chars of command are enough 
#define CMDS_ENOUGH_CHARS 4
// maximal command length (buffering)
#define CMDS_MAX_CHARS 15

// find which command matches it and track the rest of it
// to eventually report unknown/unsupported command
enum cmd_detection_states
{
  CD_INIT = 0, // initial accumulation of first non-space character
  CD_START, // buffer the rest until the space
  CD_EXEC, // executing the command
  CD_ERROR, // command not found or not matching (syntax error)
};


// TAP states enumerated/tokenized
enum libxsvf_tap_state 
{
	/* Special States */
	LIBXSVF_TAP_INIT = 0,
	LIBXSVF_TAP_RESET = 1,
	LIBXSVF_TAP_IDLE = 2,
	/* DR States */
	LIBXSVF_TAP_DRSELECT = 3,
	LIBXSVF_TAP_DRCAPTURE = 4,
	LIBXSVF_TAP_DRSHIFT = 5,
	LIBXSVF_TAP_DREXIT1 = 6,
	LIBXSVF_TAP_DRPAUSE = 7,
	LIBXSVF_TAP_DREXIT2 = 8,
	LIBXSVF_TAP_DRUPDATE = 9,
	/* IR States */
	LIBXSVF_TAP_IRSELECT = 10,
	LIBXSVF_TAP_IRCAPTURE = 11,
	LIBXSVF_TAP_IRSHIFT = 12,
	LIBXSVF_TAP_IREXIT1 = 13,
	LIBXSVF_TAP_IRPAUSE = 14,
	LIBXSVF_TAP_IREXIT2 = 15,
	LIBXSVF_TAP_IRUPDATE = 16
};

char *Tap_states[] =
{
  [LIBXSVF_TAP_INIT] = "INIT",
  [LIBXSVF_TAP_RESET] = "RESET",
  [LIBXSVF_TAP_IDLE] = "IDLE",
  [LIBXSVF_TAP_DRSELECT] = "DRSELECT",
  [LIBXSVF_TAP_DRCAPTURE] = "DRCAPTURE",
  [LIBXSVF_TAP_DRSHIFT] = "DRSHIFT",
  [LIBXSVF_TAP_DREXIT1] = "DREXIT1",
  [LIBXSVF_TAP_DRPAUSE] = "DRPAUSE",
  [LIBXSVF_TAP_DREXIT2] = "DREXIT2",
  [LIBXSVF_TAP_DRUPDATE] = "DRUPDATE",
  [LIBXSVF_TAP_IRSELECT] = "IRSELECT",
  [LIBXSVF_TAP_IRCAPTURE] = "IRCAPTURE",
  [LIBXSVF_TAP_IRSHIFT] = "IRSHIFT",
  [LIBXSVF_TAP_IREXIT1] = "IREXIT1",
  [LIBXSVF_TAP_IRPAUSE] = "IRPAUSE",
  [LIBXSVF_TAP_IREXIT2] = "IREXIT2",
  [LIBXSVF_TAP_IRUPDATE] = "IRUPDATE"
};

// search command
// >= 0 : tokenized command
// < 0 : command not found
int8_t search_cmd(char *cmd)
{
  // printf("<SEARCH %s>", cmd);
  int i;
  for(i = 0; i < CMD_NUM; i++)
    if(strcmp(cmd, Commands[i]) == 0)
      return i;
  return -1;
}


// '\0' char will reset command state
int8_t commandstate(char c)
{
  static uint32_t cmdindex = 0;
  static char cmdbuf[CMDS_MAX_CHARS+1];  // buffer command chars + null
  static int8_t command = -1; // detected command
  static uint8_t cdstate = CD_INIT; // first few chars of command detection state
  
  if(c == '\0')
  {
    cmdindex = 0;
    command = -1;
    cdstate = CD_INIT;
    return 0;
  }

  switch(cdstate)
  {
        case CD_INIT:
          // looking for non-space
          if(c != ' ')
          {
            cmdbuf[0] = c;
            cmdindex = 1;
            command = -1;
            cdstate = CD_START;
          }
          break;
        case CD_START:
          if(c == ' ')
          {
            // space found, search for the buffered command
            cmdbuf[cmdindex] = '\0'; // 0-terminate
            command = search_cmd(cmdbuf);
            if(command < 0)
              cdstate = CD_ERROR;
            else
            {
              printf("<found %s>", Commands[command]);
              cdstate = CD_EXEC;
            }
            break;
          }
          // limited buffering
          if(cmdindex < CMDS_MAX_CHARS)
          {
            cmdbuf[cmdindex] = c;
            cmdindex++;
          }
          break;
        case CD_EXEC:
          // executing -- switch various commands
          if(c == ';')
            cdstate = CD_INIT;
          break;
        case CD_ERROR:
          // error
          return -1;
          break;
  }

  
}

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
  static uint8_t cmderr = 0;
  if(index == 0)
  {
    lstate = LS_SPACE;
    line_count = 0;
    lbracket = 0;
    commandstate('\0');
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
        {
          printf("_");
          cmderr = commandstate(c); // process the space
        }
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
      // only active text appears here. comments and 
      // multiple spaces are filtered out
      printf("%c", c);
      cmderr = commandstate(c);      
    }
  }
  if(cmderr)
    printf("command error\n");
  printf("line count %d\n", line_count);
  return 0;
} 
