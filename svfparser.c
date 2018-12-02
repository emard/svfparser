#include <stdint.h>
#include "svfparser.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// max bytes allowed to allocate per bitfield
// HDR,HIR,TDR,TIR each may need 0-4 bitfields
// SDR,SIR by the standard should be remembered
// the same way as HDR is rememberd but here we
// used on-the-fly, buffering only output data,
// xor-ing them with TDO and masking them on-the-fly
// it can work if bitfields come in this order:
// TDI,TDO,MASK. SMASK must be ignored.

// 1-direct on-the-fly mode: minimal SDR/SIR buffering
// 0-standard mode
uint8_t direct_mode = 1; 

// immediately, buffer can be alloced at invocation of
// TDI, even shorter than neccessary. In this buffer
// TDO response is stored until the buffer is full
// and excess data discarded with warning message.
// Response (even partial) can be optionally used later
// for masking and verification
const int MAX_alloc = 30000;



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
  [CMD_NUM] = NULL
};

int Completed_command = CMD_NUM; // completed command

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
	LIBXSVF_TAP_IRUPDATE = 16,
	/* numbef of them */
	LIBXSVF_TAP_NUM = 17,
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
  [LIBXSVF_TAP_IRUPDATE] = "IRUPDATE",
  [LIBXSVF_TAP_NUM] = NULL
};

// search command
// >= 0 : tokenized command
// < 0 : command not found
// list is array of strings, null pointer terminated
int8_t search_name(char *cmd, char *list[])
{
  // printf("<SEARCH %s>", cmd);
  int i;
  for(i = 0; list[i] != NULL; i++)
    if(strcmp(cmd, list[i]) == 0)
      return i;
  return -1;
}

// common states for HDR,HIR,SDR,SIR,TDR,TIR
enum bit_sequence_parsing_states
{
  BSPS_INIT = 0,
  BSPS_LENGTH, // taking the length of the bits array
  BSPS_NAME, // field name
  BSPS_VALUEOPEN, // open parenthesis
  BSPS_VALUE, // hex char of value
  BSPS_NAME1, // after closed parenthesis expect another name
  BSPS_COMPLETE,
  BSPS_ERROR
};

enum bit_sequence_field
{
  BSF_TDO = 0,
  BSF_TDI,
  BSF_MASK,
  BSF_SMASK,
  BSF_NUM
};

char *bsf_name[] =
{
  [BSF_TDO] = "TDO",
  [BSF_TDI] = "TDI",
  [BSF_MASK] = "MASK",
  [BSF_SMASK] = "SMASK",
  [BSF_NUM] = NULL
};

/*
the bitbanger
bsf field type
BSF_*: the field we work on, action to take
direct: 0-indirect mode, 1-direct mode
*d: pointer to data (byte addressable)
n: number of bits to bang
*/
int8_t bitbang(int8_t bsf, int8_t direct, uint8_t *d, uint32_t n)
{


}

/* ******************* BEGIN COMMAND SERVICE FUNCTIONS ******************* */
/*
input: '!' - resets global parser state
       '\0' - resets line parser state
       letter - incoming char by char
return value:
       0 - ok
      <0 - error
*/

int8_t cmd_pio(char c)
{
  puts("PIO NOT SUPPORTED");
  return 0;
}

// bit sequence struct common for
// HDR,HIR,SDR,SIR,TDR,TIR
struct S_bitseq
{
  uint32_t length;
  int32_t digitindex[BSF_NUM]; // insertion digit (nibble) index running from 2*allocated-1 downto 0. -1 if no space left.
  uint32_t allocated[BSF_NUM]; // how many bytes are allocated in field[]
  uint8_t *field[BSF_NUM]; // *tdo, *tdi, *mask, *smask;
};

// leading zeroes are assumed for a field
// if not exactly specified

// parsed bit sequence is global state
// bitbanger needs to access them all
// initialize all as NULL pointers (unallocated space)
// reallocating them as needed
// 1 for direct I/O (no allocation, no buffering)
struct S_bitseq BS_hdr = { 0, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_hir = { 0, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_sdr = { 0, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_sir = { 0, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_tdr = { 0, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_tir = { 0, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };

// bitfield name "SMASK" is longest: 5 chars
enum bitfield_name_max_len
{
  BF_NAME_MAXLEN = 5
};

void print_bitsequence(struct S_bitseq *seq)
{
  int i, j;
  printf("length %d bit\n", seq->length);
  for(i = 0; i < BSF_NUM; i++)
  {
    int digitlen = 2*seq->allocated[i]-1 - seq->digitindex[i];
    int bytelen = (digitlen+1)/2;
    int firstbyte = seq->digitindex[i]/2;
    uint8_t *mem = seq->field[i] + firstbyte;
    printf("field %s (%d digits)\n", bsf_name[i], digitlen);
    for(j = 0; j < bytelen; j++)
      printf("%02X", mem[j]);
    printf("\n");
  }
}

void print_buffer()
{
  if(Completed_command == CMD_SIR)
  {
    printf("SIR buffer:\n");
    print_bitsequence(&BS_sir);
  }
  if(Completed_command == CMD_SDR)
  {
    printf("SDR buffer:\n");
    print_bitsequence(&BS_sdr);
  }
}


// common parser for
// HDR,HIR,SDR,SIR,TDR,TIR
int8_t cmd_bitsequence(char c, struct S_bitseq *seq)
{
  static int8_t state = BSPS_INIT;
  static int bfnamelen = 0;
  static char bfname[BF_NAME_MAXLEN+1];
  static uint8_t tbfname = -1; // tokenized bitfield name
  static int32_t digitindex = -1; // countdown hex digits of the bitfield
  if(c == '\0')
  { // reset parsing state
    state = 0;
    bfnamelen = 0;
    tbfname = -1;
    digitindex = -1;
    // TDI, MASK, SMASK are sticky and remembered from previous SVF command
    // TDO is not remembered between SVF commands
    seq->digitindex[BSF_TDO] = seq->allocated[BSF_TDO]*2-1;
    return 0;
  }
  if(c == '!')
  { // complete reset, forgets everything
    state = 0;
    bfnamelen = 0;
    tbfname = -1;
    digitindex = -1;
    for(int i = 0; i < BSF_NUM; i++)
      seq->digitindex[i] = seq->allocated[i]*2-1;
    seq->length = 0;
    return 0;
  }
  switch(state)
  {
    case BSPS_INIT:
      if(c == ';')
      {
        state = BSPS_ERROR;
        break;
      }
      // look for first char of the length
      if(c >= '0' && c <= '9')
      {
        // take first digit
        seq->length = c - '0';
        state = BSPS_LENGTH;
      }
      break;
    case BSPS_LENGTH:
      // take length decimal value digit by digit
      if(c >= '0' && c <= '9')
      {
        // take another digit
        seq->length = (seq->length * 10) + c - '0';
        break;
      }
      if(c == ' ')
      { // space - end of length, proceed getting the name
        printf("L%d", seq->length);
        bfname[0] = '\0';
        bfnamelen = 0;
        tbfname = -1;
        state = BSPS_NAME;
        break;
      }
      if(c == ';')
      {
        if(seq->length == 0)
        {
          printf("L%d", seq->length);
          state = BSPS_COMPLETE;
        }
        else
          state = BSPS_ERROR;
        break;          
      }
      break;
    case BSPS_NAME:
      if(c == ' ')
      {
        bfname[bfnamelen] = '\0'; // 0-terminate
        tbfname = search_name(bfname, bsf_name);
        if(tbfname >= 0)
          printf("tbfname '%s'", bsf_name[tbfname]);
        state = BSPS_VALUEOPEN;
        break;
      }
      if(c >= 'A' && c <= 'Z')
      {
        if(bfnamelen < BF_NAME_MAXLEN)
          bfname[bfnamelen++] = c;
        else
        {
          // name too long, error
          bfname[bfnamelen] = '\0'; // 0-terminate
          state = BSPS_ERROR;
        }
        break;
      }
      state = BSPS_ERROR;
      break;
    case BSPS_VALUEOPEN:
      if(c == '(')
      {
        // sanity check: we must know bitfield name
        // and have it tokenized, otherwise it's error
        if(tbfname < 0)
        {
          state = BSPS_ERROR;
          break;        
        }
        digitindex = (seq->length+3)/4-1; // start inserting at highest position downwards
        printf("open");
        state = BSPS_VALUE;
        // it is allowed to allocate less than required length
        // just issue some warnings
        // realloc to length now
        // calculate bytes needed to allocate
        uint32_t alloc_bytes = (seq->length+7)/8;
        // apply MAX alloc limit
        if(alloc_bytes > MAX_alloc)
        {
          printf("WARNING: required %d bytes for bitfield exceeds limit. Allocating only %d bytes\n",
            alloc_bytes, MAX_alloc);
          alloc_bytes = MAX_alloc;
        }
        // realloc now the bitfield
        seq->field[tbfname] = realloc(seq->field[tbfname], alloc_bytes);
        if(seq->field[tbfname] == NULL)
        {
          printf("Memory Allocation Failed\n");
          state = BSPS_ERROR;
          break;
        }
        seq->allocated[tbfname] = alloc_bytes; // track how much is allocated
        seq->digitindex[tbfname] = digitindex; // insertion point start from highest byte
      }
      else
        state = BSPS_ERROR;
      break;
    case BSPS_VALUE:
      // sanity check: we must know bitfield name
      // and have it tokenized, otherwise it's error
      if( (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') )
      {
        if(tbfname < 0)
        {
          state = BSPS_ERROR;
          break;        
        }
        // fill hex into allocated space
        // conversion from ascii to hex digit (binary lower 4-bits)
        uint8_t hexdigit = c < 'A' ? c - '0' : c + 10 - 'A';
        // todo: 8-bit buffering
        // work here with complete bytes
        if( digitindex >= 0 )
        {
          // buffer the data for later use
          // don't exceed the allocated length
          uint32_t byteindex = digitindex/2;
          if( byteindex < seq->allocated[tbfname] )
          {
            uint8_t value_byte;
            // printf("add digit #%d %s %X\n", digitindex, bsf_name[tbfname], hexdigit);
            if( (digitindex & 1) != 0 )
              value_byte = hexdigit << 4;
            else
              value_byte = seq->field[tbfname][byteindex] | hexdigit;
            seq->field[tbfname][byteindex] = value_byte;
            seq->digitindex[tbfname] = --digitindex;
          }
        }
        else
          printf("********** OVERRUN %d **********\n", digitindex);
        break;
      }
      if(c == ')')
      {
        // todo: bitbang or store incomplete byte buffered above
        printf("close");
        bfname[0] = '\0';
        bfnamelen = 0;
        tbfname = -1;
        state = BSPS_NAME1; // expect another name
        break;
      }
      state = BSPS_ERROR;
      break;
    case BSPS_NAME1:
      if(c == ' ') // ignore space
        break;
      if(c >= 'A' && c <= 'Z')
      {
        if(bfnamelen < BF_NAME_MAXLEN)
        {
          bfname[bfnamelen++] = c;
          state = BSPS_NAME;
        }
        else
        {
          // name too long, error
          bfname[bfnamelen] = '\0'; // 0-terminate
          state = BSPS_ERROR;
        }
        break;
      }
      state = BSPS_ERROR;
      break;
    default:
      state = BSPS_ERROR;
      break;
  }
  // printf("%c*", c);
  return 0;
}

int8_t cmd_hdr(char c)
{
  return cmd_bitsequence(c, &BS_hdr);
}

int8_t cmd_hir(char c)
{
  return cmd_bitsequence(c, &BS_hir);
}

int8_t cmd_sdr(char c)
{
  return cmd_bitsequence(c, &BS_sdr);
}

int8_t cmd_sir(char c)
{
  return cmd_bitsequence(c, &BS_sir);
}

int8_t cmd_tdr(char c)
{
  return cmd_bitsequence(c, &BS_tdr);
}

int8_t cmd_tir(char c)
{
  return cmd_bitsequence(c, &BS_tir);
}

// struct to command service functions
struct S_cmd_service
{
  int8_t (*service)(char);
};

struct S_cmd_service Cmd_service[] =
{
  [CMD_ENDDR] = { NULL },
  [CMD_ENDIR] = { NULL },
  [CMD_FREQUENCY] = { NULL },
  [CMD_HDR] = { cmd_hdr },
  [CMD_HIR] = { cmd_hir },
  [CMD_PIO] = { cmd_pio },
  [CMD_PIOMAP] = { NULL },
  [CMD_RUNTEST] = { NULL },
  [CMD_SDR] = { cmd_sdr },
  [CMD_SIR] = { cmd_sir },
  [CMD_STATE] = { NULL },
  [CMD_TDR] = { cmd_tdr },
  [CMD_TIR] = { cmd_tir },
  [CMD_TRST] = { NULL },
};
/* ******************* END COMMAND SERVICE FUNCTIONS ******************* */



// '\0' char will reset command state (new line)
/*
return -1 command incomplete
        0 neutral (spaces, not in command)
        1 command complete
*/
int8_t commandstate(char c)
{
  static uint32_t cmdindex = 0;
  static char cmdbuf[CMDS_MAX_CHARS+1];  // buffer command chars + null
  static int8_t command = -1; // detected command
  static uint8_t cdstate = CD_INIT; // first few chars of command detection state
  static int8_t cxstate = -1; // command execution state

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
            Completed_command = CMD_NUM;
            cxstate = -1;
            cdstate = CD_START;
          }
          return 0;
          break;
        case CD_START:
          if(c == ' ')
          {
            // space found, search for the buffered command
            cmdbuf[cmdindex] = '\0'; // 0-terminate string
            command = search_name(cmdbuf, Commands);
            if(command < 0)
              cdstate = CD_ERROR;
            else
            {
              printf("<found %s>", Commands[command]);
              // TODO reset previous buffered content
              // reset parser state of the command service function
              if(Cmd_service[command].service)
                Cmd_service[command].service('\0');
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
          // executing
          // sanity check
          if(command < 0 || command >= CMD_NUM)
            return -2; // strange, this should never happen
          // call selected command service function
          if(Cmd_service[command].service)
            cxstate = Cmd_service[command].service(c);
          // semicolon to end command
          if(c == ';')
          {
            cdstate = CD_INIT;
            Completed_command = command;
            return 1; // command complete
          }
          break;
        case CD_ERROR:
          // error
          return -1;
          break;
  }
  return -1; // command incomplete
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
  static int8_t cmderr = 0;
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
      if(cmderr > 0)
      {
        printf("command %s complete\n", Commands[Completed_command]);
        print_buffer();
      }
    }
  }
  if(cmderr < 0)
    printf("command incomplete\n");
  if(cmderr > 0)
    printf("command complete\n");
  printf("line count %d\n", line_count);
  return 0;
} 
