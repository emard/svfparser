#include <stdint.h>
#include "svfparser.h"
#include "jtaghw_print.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h> // toupper()

#define DBG_PRINT 1
#if DBG_PRINT
#define PRINTF(f_, ...) printf((f_), ##__VA_ARGS__)
#else
#define PRINTF(f_, ...)
#endif


/*
[SVF Format spec](http://www.jtagtest.com/pdf/svf_specification.pdf)
[TI test symposium](http://home.zcu.cz/~dudacek/Kp/seminar2.pdf)
*/

// max bytes allowed to allocate per bitfield
// HDR,HIR,TDR,TIR each may need 0-4 bitfields
// SDR,SIR by the standard should be remembered
// the same way as HDR is rememberd but here we
// used on-the-fly, buffering only output data,
// xor-ing them with TDO and masking them on-the-fly
// it can work if bitfields come in this order:
// TDI,TDO,MASK. SMASK must be ignored.

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

const char *Commands[] =
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

const char *Tap_states[] =
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

// endstate name DRCAPTURE is longest: 9 chars
enum libxsvf_tap_name_max_len
{
  LIBXSVF_TAP_NAME_MAXLEN = 9
};

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

const char *bsf_name[] =
{
  [BSF_TDO] = "TDO",
  [BSF_TDI] = "TDI",
  [BSF_MASK] = "MASK",
  [BSF_SMASK] = "SMASK",
  [BSF_NUM] = NULL
};

enum float_parsing_states
{
  FLPS_INIT = 0,
  FLPS_NUM, // integer number part, "0-9", "."->FRAC, "E"->E
  FLPS_FRAC, // fractional part, "0-9", "E"->E
  FLPS_E, // "0-9+-"->EXP,
  FLPS_EXP, // numbers "0-9", ";"->COMPLETE
  FLPS_COMPLETE,
  FLPS_ERROR
};

enum frequency_parsing_states
{
  FQPS_INIT = 0,
  FQPS_VALUE, // floating point value
  FQPS_COMPLETE,
  FQPS_ERROR
};


#if 0
static int state_endir = LIBXSVF_TAP_IDLE;
static int state_enddr = LIBXSVF_TAP_IDLE;
static int state_run = LIBXSVF_TAP_IDLE;
static int state_endrun = LIBXSVF_TAP_IDLE;
#endif

uint8_t PAD_BYTE[2] = {0x00, 0xFF};
uint8_t ReverseNibble[16];

// bit sequence struct common for
// HDR,HIR,SDR,SIR,TDR,TIR
struct S_bitseq
{
  uint32_t length;
  uint32_t length_prev[BSF_NUM]; // lengths of each bitfield of previous SVF command
  int32_t digitindex[BSF_NUM]; // insertion digit (nibble) index running from 2*allocated-1 downto 0. -1 if no space left.
  uint32_t allocated[BSF_NUM]; // how many bytes are allocated in field[]
  uint8_t *field[BSF_NUM]; // *tdo, *tdi, *mask, *smask;
};

struct S_float
{
  int number, frac, expsign, exponent;
  int8_t state;
};

struct S_float fl;


/* memory storage plan

[SVF Format spec](http://www.jtagtest.com/pdf/svf_specification.pdf)
Page 'V' (p.5):
The bit order for scan data follows the convention that the least
significant bit (rightmost bit) is the first bit scanned into the
hardware for TDI and SMASK scan data and is the first bit scanned
out for TDO and MASK data. 

in SPI interface, MSB (most significant bit of a byte) is shifted first.
SPI mode 1 clocking scheme is used

we need to determine:
uint8_t n_first_nibble; // 1/0 yes/no output first nibble
uint8_t *first_nibble; // ptr to byte having first nibble
uint32_t n_complete_bytes; // number of complete bytes
uint8_t *complete_bytes; // pointer to first complete byte
uint8_t n_last_bits; // 1/0 yes/no output last nibble
uint8_t *last_bits; // ptr to byte having last bits, should be padded
uint32_t n_pad_bytes; // number of pad bytes
uint8_t pad_byte; // pad byte value

svf length: 47
svf digits: 1234567
bytes stored:
 00 00 70 56 34 12

output data
0x7 (4-bit) first 4-bits, hex digit 7
0x56 0x34 0x12 (3 bytes, 24-bit) complete bytes
0b000 (3 bits, 0-padding)
0x00 0x00 (2 bytes, 16-bit) complete bytes 0-padding

svf length: 47
svf digits: 12345678
bytes stored:
 00 00 78 56 34 12

output data
0x78 0x56 0x34 0x12 (3 bytes, 24-bit) complete bytes
0b000 (3 bits, 0-padding)
0x00 0x00 (2 bytes, 16-bit) complete bytes 0-padding


svf length: 26
svf digits: 1234567
bytes stored:
 70 56 34 12

output data
0x7 hex digit 7
0x56 0x34 (2 bytes, 16-bit) complete bytes
0x2 hex digit 2 (4-bits)
0b10 2-bits hex digit 1
*/

// leading zeroes are assumed for a field
// if not exactly specified

// parsed bit sequence is global state
// bitbanger needs to access them all
// initialize all as NULL pointers (unallocated space)
// reallocating them as needed
// 1 for direct I/O (no allocation, no buffering)
struct S_bitseq BS_hdr = { 0, {0,0,0,0}, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_hir = { 0, {0,0,0,0}, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_sdr = { 0, {0,0,0,0}, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_sir = { 0, {0,0,0,0}, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_tdr = { 0, {0,0,0,0}, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };
struct S_bitseq BS_tir = { 0, {0,0,0,0}, {-1,-1,-1,-1}, {0,0,0,0}, {NULL, NULL, NULL, NULL} };

// bitfield name "SMASK" is longest: 5 chars
enum bitfield_name_max_len
{
  BF_NAME_MAXLEN = 5
};

/* ************ end state parsing *************** */

enum endxr_state_choice
{
  ENDX_ENDDR = 0,
  ENDX_ENDIR,
  ENDX_NUM
};

enum endxr_parsing_state
{
  ENPS_INIT = 0,
  ENPS_NAME,
  ENPS_COMPLETE,
  ENPS_ERROR
};

uint8_t endxr_state[ENDX_NUM] = { LIBXSVF_TAP_IDLE, LIBXSVF_TAP_IDLE };

// endstate name IRPAUSE is longest: 7 chars
enum end_name_max_len
{
  END_NAME_MAXLEN = 7
};

/* ************ state path parsing *************** */
enum state_walk_parsing_state
{
  SWPS_INIT = 0,
  SWPS_NAME,
  SWPS_SPACE,
  SWPS_COMPLETE,
  SWPS_ERROR
};

/* ************ runtest parsing *************** */
enum runtest_parsing_state
{
  RTPS_INIT = 0,
  RTPS_WORD,
  RTPS_NUMBER,
  RTPS_SPACE, // not needed?
  RTPS_BEGINSTATE,
  RTPS_COUNT,
  RTPS_CLOCK,
  RTPS_MINTIME,
  RTPS_MIN_SEC,
  RTPS_MAXIMUM,
  RTPS_MAXTIME,
  RTPS_MAX_SEC,
  RTPS_ENDSTATE,
  RTPS_COMPLETE,
  RTPS_ERROR
};

enum runtest_words_token
{
  RT_WORD_TCK = 0,
  RT_WORD_SCK,
  RT_WORD_SEC,
  RT_WORD_MAXIMUM,
  RT_WORD_ENDSTATE,
  RT_WORD_NUM
};

const char *runtest_words[] =
{
  [RT_WORD_TCK] = "TCK",
  [RT_WORD_SCK] = "SCK",
  [RT_WORD_SEC] = "SEC",
  [RT_WORD_MAXIMUM] = "MAXIMUM",
  [RT_WORD_ENDSTATE] = "ENDSTATE",
  [RT_WORD_NUM] = NULL
};

// max name length for all runtest names
enum runtest_name_max_len
{
  RUNTEST_NAME_MAXLEN = 9
};

struct S_jtaghw JTAG_TDI, JTAG_TDO;


/* ***************** bit sequence output ********************** */
void play_bitsequence(struct S_bitseq *seq)
{
  // print what would be bitbanged
  // if byte incomplete, print first 4 data bits
  // print complete data bytes
  // print last 0-7 data bits and trailing 0-bits
  // print trailing 0-bytes
  int i, j;
  // PRINTF("length %d bit\n", seq->length);
  int tdo_digitlen = (seq->length+3)/4-1 - seq->digitindex[BSF_TDO];
  for(i = 0; i < BSF_NUM; i++)
  {
    if(seq->allocated[i] == 0 || seq->field[i] == NULL)
      continue; // not allocated

    // PRINTF("seq->length = %d, seq->digitindex = %d\n", seq->length, seq->digitindex[i]);
    // from seq->length and seq->digitindex we calculate following:

    //int max_full_digits = seq->length/4;
    int digitlen = (seq->length+3)/4-1 - seq->digitindex[i];
    int bytelen = (digitlen+1)/2;
    int bits_remaining = seq->length - 8 * bytelen;
    int firstbyte = (seq->digitindex[i]+1)/2;
    uint8_t *mem = seq->field[i] + firstbyte;
    // int complete_bytes = bits_remaining < 0 ? ( -bits_remaining > 3 ? bytelen - 1 : bytelen) : bytelen;
    int complete_bytes = bits_remaining < 0 && -bits_remaining > 3 ? bytelen - 1 : bytelen;
    uint8_t pad_byte = i == BSF_MASK || i == BSF_SMASK ? 0xFF : 0x00;
    
    // initialize (reset) bitbang pointers
    JTAG_TDI.header = NULL;
    JTAG_TDI.header_bits = 0;
    JTAG_TDI.data = NULL;
    JTAG_TDI.data_bytes = 0;
    JTAG_TDI.trailer = NULL;
    JTAG_TDI.trailer_bits = 0;
    JTAG_TDI.pad = pad_byte; // 0 or 0xFF padding value
    JTAG_TDI.pad_bits = 0; // number of padding bits (not 0 if exist)

    #if 0
    PRINTF("bytelen=%d\n", bytelen);
    PRINTF("bits_remaining=%d\n", bits_remaining);
    int memlen = seq->allocated[i] - firstbyte;
    PRINTF("memlen=%d\n", memlen);
    #if REVERSE_NIBBLE
    for(j = 0; j < memlen; j++)
      PRINTF("%01X%01X ", ReverseNibble[mem[j] >> 4], ReverseNibble[mem[j] & 0xF]);
    #else
    for(j = 0; j < memlen; j++)
      PRINTF("%02X ", mem[j]);
    #endif
    PRINTF("\n");
    PRINTF("reading from %d\n", firstbyte);
    PRINTF("field %s (%d digits)\n", bsf_name[i], digitlen);
    #endif
    PRINTF("%5s ", bsf_name[i]);
    if( (digitlen > 0 && i != BSF_MASK)
    ||  (digitlen > 0 && i == BSF_MASK && tdo_digitlen > 0)
    )
    {
      int bstart = 0;
      int print_first_nibble = bits_remaining >= 0 ? 
        ((bits_remaining & 7) >= 1 && (bits_remaining & 7) <= 4)  // when padding bits
      : ( -bits_remaining >= 1 && -bits_remaining <= 3); // when truncating bits
      if(print_first_nibble)
      {
        // nibble
        #if REVERSE_NIBBLE
        PRINTF("0x%01X ", ReverseNibble[mem[0] & 0xF]);
        #else
        PRINTF("0x%01X ", mem[0] >> 4);
        #endif
        bstart = 1;
        JTAG_TDI.header = mem;
        JTAG_TDI.header_bits = 4;
      }
      if(complete_bytes > 0)
      {
        PRINTF("0x");
        for(j = bstart; j < complete_bytes; j++)
          #if REVERSE_NIBBLE
          PRINTF("%01X%01X", ReverseNibble[mem[j] >> 4], ReverseNibble[mem[j] & 0xF]);
          #else
          PRINTF("%01X%01X", mem[j] & 0xF, mem[j] >> 4);
          #endif
        PRINTF(" ");
        JTAG_TDI.data = mem + bstart;
        JTAG_TDI.data_bytes = complete_bytes-bstart;
      }
      if(bstart != 0 && bits_remaining > 0)
      { // nibble
        #if REVERSE_NIBBLE
        PRINTF("0x%01X ", ReverseNibble[mem[j] >> 4]);
        #else
        PRINTF("0x%01X ", mem[j] & 0xF);
        #endif
        // patch upper nibble of mem[j] with the nibble from pad_byte
        mem[j] |= pad_byte & 0xF0;
        JTAG_TDI.trailer = mem + j;
        JTAG_TDI.trailer_bits = 4;
      }
      if(bits_remaining != 0)
      {
        uint8_t byte_remaining = pad_byte;
        uint8_t additional_bits = (8+bits_remaining) & 7;
        uint8_t additional_bytes = bits_remaining / 8;
        if(bits_remaining < 0)
          byte_remaining = mem[j];
        JTAG_TDI.pad_bits = additional_bits + additional_bytes * 8;
        if(additional_bits > 0)
        {
          if(JTAG_TDI.trailer_bits == 0)
          {
            // change lower bits to bits from pad byte.
            // bits in mem[] are reordered
            // for transmission therefore we have
            // small bitwise gymnastics:
            if(bits_remaining < 0)
            {
              #if REVERSE_NIBBLE
              uint8_t mask_byte = 0xFF >> (8-additional_bits); // mask for lower bits
              uint8_t and_byte = (ReverseNibble[mask_byte >> 4])
                               | (ReverseNibble[mask_byte & 0xF] << 4);

              #else
              uint32_t mask_byte = ((uint32_t)0xFF) << (additional_bits); // mask for lower bits
              uint8_t and_byte = mask_byte;
              #endif
              mem[j] |= pad_byte & and_byte;
              JTAG_TDI.trailer = mem + j;
              JTAG_TDI.trailer_bits = additional_bits;
              JTAG_TDI.pad_bits = additional_bytes * 8;
            }
            else
            {
              JTAG_TDI.pad_bits = additional_bits + additional_bytes * 8;
            }
          }
          if(additional_bits >= 4)
          {
            // byte_remaining = mem[j];
            #if REVERSE_NIBBLE
              PRINTF("0x%01X ", ReverseNibble[byte_remaining >> 4]);
              if(additional_bits > 4)
              {
                PRINTF("0b");
                byte_remaining <<= 4;
                for(j = 4; j < additional_bits; j++, byte_remaining <<= 1)
                  PRINTF("%d", byte_remaining >> 7);
                PRINTF(" ");
              }
            #else
              PRINTF("0x%01X ", byte_remaining & 0xF);
              if(additional_bits > 4)
              {
                PRINTF("0b");
                byte_remaining >>= 4;
                for(j = 4; j < additional_bits; j++, byte_remaining >>= 1)
                  PRINTF("%d", byte_remaining & 1);
                PRINTF(" ");
              }
            #endif
          }
          else
          {
            PRINTF("0b");
            #if REVERSE_NIBBLE
            for(j = 0; j < additional_bits; j++, byte_remaining <<= 1)
              PRINTF("%d", byte_remaining >> 7);
            #else
            // PRINTF("(%02X)", byte_remaining);
            for(j = 0; j < additional_bits; j++, byte_remaining >>= 1)
              PRINTF("%d", byte_remaining & 1);
            #endif
            PRINTF(" ");
          }
        }
        if(additional_bytes > 0)
        {
          PRINTF("0x");
          for(j = 0; j < additional_bytes; j++)
            PRINTF("%02X", pad_byte);
        }
      }
    }
    PRINTF("\n");
    jtag_tdi_tdo(&JTAG_TDI, &JTAG_TDO);
  }
}

void play_buffer()
{
  if(Completed_command == CMD_SIR)
  {
    PRINTF("SIR buffer:\n");
    play_bitsequence(&BS_sir);
  }
  if(Completed_command == CMD_SDR)
  {
    PRINTF("SDR buffer:\n");
    play_bitsequence(&BS_sdr);
  }
}

// search command
// >= 0 : tokenized command
// < 0 : command not found
// list is array of strings, null pointer terminated
int8_t search_name(char *cmd, const char *list[])
{
  // PRINTF("<SEARCH %s>", cmd);
  int i;
  for(i = 0; list[i] != NULL; i++)
    if(strcmp(cmd, list[i]) == 0)
      return i;
  return -1;
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


// common parser for
// HDR,HIR,SDR,SIR,TDR,TIR
int8_t cmd_bitsequence(char c, struct S_bitseq *seq)
{
  static int8_t state = BSPS_INIT;
  static int bfnamelen = 0;
  static char bfname[BF_NAME_MAXLEN+1];
  static int8_t tbfname = -1; // tokenized bitfield name
  static int32_t digitindex = -1; // countdown hex digits of the bitfield
  if(c == '\0')
  { // reset parsing state
    state = 0;
    bfnamelen = 0;
    tbfname = -1;
    digitindex = 0;
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
    digitindex = 0;
    for(int i = 0; i < BSF_NUM; i++)
      seq->digitindex[i] = 0;
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
        PRINTF("L%d", seq->length);
        bfname[0] = '\0';
        bfnamelen = 0;
        tbfname = -1;
        state = BSPS_NAME;
        // if length has changed, then reset remembered fields
        for(int i = 0; i < BSF_NUM; i++)
          if(seq->length_prev[i] != seq->length)
          {
            // PRINTF("reset length");
            seq->digitindex[i] = (seq->length+3)/4-1;
          }
        break;
      }
      if(c == ';')
      {
        if(seq->length == 0)
        {
          PRINTF("L%d", seq->length);
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
          PRINTF("tbfname '%s'", bsf_name[tbfname]);
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
        PRINTF("open");
        state = BSPS_VALUE;
        // it is allowed to allocate less than required length
        // just issue some warnings
        // realloc to length now
        // calculate bytes needed to allocate
        uint32_t alloc_bytes = (seq->length+7)/8;
        // apply MAX alloc limit
        if(alloc_bytes > MAX_alloc)
        {
          PRINTF("WARNING: required %d bytes for bitfield exceeds limit. Allocating only %d bytes\n",
            alloc_bytes, MAX_alloc);
          alloc_bytes = MAX_alloc;
        }
        // realloc now the bitfield
        seq->field[tbfname] = (uint8_t *)realloc(seq->field[tbfname], alloc_bytes);
        if(seq->field[tbfname] == NULL)
        {
          PRINTF("Memory Allocation Failed\n");
          state = BSPS_ERROR;
          break;
        }
        seq->allocated[tbfname] = alloc_bytes; // track how much is allocated
        seq->digitindex[tbfname] = digitindex; // insertion point start from highest byte
        // when length has changed then reset bit field to its default value
        if(seq->length_prev[tbfname] != seq->length)
        {
          // when length changes, default MASK and SMASK is set to all cares 0xFF
          if(tbfname == BSF_MASK || tbfname == BSF_SMASK)
            memset(seq->field[tbfname], 0xFF, seq->allocated[tbfname]);
        }
        seq->length_prev[tbfname] = seq->length;
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
        #if REVERSE_NIBBLE
        uint8_t hexdigit = ReverseNibble[c < 'A' ? c - '0' : c + 10 - 'A'];
        #else
        uint8_t hexdigit = c < 'A' ? c - '0' : c + 10 - 'A';
        #endif
        if( digitindex >= 0 )
        {
          // buffer the data for later use
          // don't exceed the allocated length
          uint32_t byteindex = digitindex/2;
          if( byteindex < seq->allocated[tbfname] )
          {
            uint8_t value_byte;
            // PRINTF("add digit #%d %s %X\n", digitindex, bsf_name[tbfname], hexdigit);
            #if REVERSE_NIBBLE
            if( (digitindex & 1) != 0 )
              value_byte = hexdigit; // with 4 bit leading zeros
            else
              value_byte = (seq->field[tbfname][byteindex] & 0xF) | (hexdigit<<4);
            #else
            if( (digitindex & 1) != 0 )
              value_byte = hexdigit << 4;
            else
              value_byte = (seq->field[tbfname][byteindex] & 0xF0) | (hexdigit); // with 4 bit leading zeros
            #endif
            seq->field[tbfname][byteindex] = value_byte;
            // PRINTF("written %s[%d]=%02X\n", bsf_name[tbfname] , byteindex, value_byte);
            seq->digitindex[tbfname] = --digitindex;
          }
        }
        else
          PRINTF("********** OVERRUN %d **********\n", digitindex);
        break;
      }
      if(c == ')')
      {
        #if 0
        // disabled - let's do it at output
        // write leading zeros for unspecified hex digits
        if( digitindex >= 0 )
        {
          uint32_t byteindex = digitindex/2;
          if( byteindex < seq->allocated[tbfname] )
          {
            int i;
            for(i = 0; i <= byteindex; i++)
              seq->field[tbfname][i] = 0; // leading zeros
          }
          seq->digitindex[tbfname] = -1;
        }
        #endif
        PRINTF("close");
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
  // PRINTF("%c*", c);
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

int8_t parse_float(char c)
{
  // static int8_t state = FLPS_INIT;
  if(c == '\0')
  { // reset parsing state
    fl.state = FLPS_INIT;
    fl.number = 0;
    fl.frac = 0;
    fl.expsign = 1;
    fl.exponent = 0;
    return fl.state;
  }
  switch(fl.state)
  {
    case FLPS_INIT:
      if(c >= '0' && c <= '9')
      {
        fl.number = fl.number*10 + (c - '0');
        fl.state = FLPS_NUM;
        break;
      }
      fl.state = FLPS_ERROR;
      break;
    case FLPS_NUM:
      if(c >= '0' && c <= '9')
      {
        fl.number = fl.number*10 + (c - '0');
        break;
      }
      if(c == '.')
      {
        fl.state = FLPS_FRAC;
        break;
      }
      if(c == 'E')
      {
        fl.state = FLPS_EXP;
        break;
      }
      fl.state = FLPS_ERROR;
      break;
    case FLPS_FRAC:
      if(c >= '0' && c <= '9')
      {
        fl.frac = fl.frac*10 + (c - '0');
        break;
      }
      if(c == 'E')
      {
        fl.state = FLPS_E;
        break;
      }
      fl.state = FLPS_ERROR;
      break;
    case FLPS_E:
      if(c >= '0' && c <= '9')
      {
        fl.exponent = fl.exponent*10 + (c - '0');
        fl.state = FLPS_EXP;
        break;
      }
      if(c == '+')
      {
        fl.expsign = 1;
        fl.state = FLPS_EXP;
        break;
      }
      if(c == '-')
      {
        fl.expsign = -1;
        fl.state = FLPS_EXP;
        break;
      }
      fl.state = FLPS_ERROR;
      break;
    case FLPS_EXP:
      if(c >= '0' && c <= '9')
      {
        fl.exponent = fl.exponent*10 + (c - '0');
        break;
      }
      fl.state = FLPS_ERROR;
      break;
    default:
      fl.state = FLPS_ERROR;
  }
  return fl.state;
}

int8_t cmd_frequency(char c)
{
  static int8_t state = FQPS_INIT;
  static int8_t float_parsing_state = FLPS_INIT;
  if(c == '\0')
  { // reset parsing state
    state = FQPS_INIT;
    parse_float('\0');
    return 0;
  }
  switch(state)
  {
    case FQPS_INIT:
      if(c == ';')
      {
        state = FQPS_COMPLETE;
        break;
      }
      if(c >= '0' && c <= '9')
      {
        state = FQPS_VALUE;
        float_parsing_state = parse_float(c);
        break;
      }
      state = FQPS_ERROR;
      break;
    case FQPS_VALUE:
      if(c == ';')
      {
        PRINTF("FLOAT %d.%dE%c%d ",
          fl.number, fl.frac, fl.expsign > 0 ? '+' : '-', fl.exponent);
        state = FQPS_COMPLETE;
        break;
      }
      float_parsing_state = parse_float(c);
      if(float_parsing_state == FLPS_ERROR)
      {
        state = FQPS_ERROR;
        break;
      }
      break;
    case FQPS_COMPLETE:
      break;
    case FQPS_ERROR:
      break;
  }
  return 0;
}

int8_t cmd_endxr(char c, uint8_t *endxr_s)
{
  static int8_t state = LIBXSVF_TAP_INIT;
  static int endnamelen = 0;
  static char endname[END_NAME_MAXLEN+1];
  static int8_t tendname = -1; // tokenized end state name

  if(c == '\0')
  { // reset parsing state
    state = LIBXSVF_TAP_INIT;
    endnamelen = 0;
    tendname = -1;
    return 0;
  }
  switch(state)
  {
    case ENPS_INIT:
      if(c >= 'A' && c <= 'Z')
      {
        if(endnamelen < END_NAME_MAXLEN)
          endname[endnamelen++] = c;
        else
        {
          // name too long, error
          endname[endnamelen] = '\0'; // 0-terminate
          state = ENPS_ERROR;
        }
        break;
      }
      if(c == ' ' || c == ';')
      {
        endname[endnamelen] = '\0'; // 0-terminate
        tendname = search_name(endname, Tap_states);
        if(tendname == LIBXSVF_TAP_IDLE
        || tendname == LIBXSVF_TAP_RESET
        || tendname == LIBXSVF_TAP_DRPAUSE
        || tendname == LIBXSVF_TAP_IRPAUSE
        )
        {
          *endxr_s = tendname;
          state = ENPS_COMPLETE;
        }
        else
          state = ENPS_ERROR;
        if(tendname >= 0)
          PRINTF("tendname '%s' %s", Tap_states[tendname], state == ENPS_ERROR ? "error" : "ok");
        break;
      }
      state = ENPS_ERROR;
      break;
    default:
      break;
  }
  return 0;
}

int8_t cmd_enddr(char c)
{
  return cmd_endxr(c, &(endxr_state[ENDX_ENDDR]));
}

int8_t cmd_endir(char c)
{
  return cmd_endxr(c, &(endxr_state[ENDX_ENDIR]));
}

// walks the TAP over the list of states
int8_t cmd_state(char c)
{
  static int8_t state = SWPS_INIT;
  static int statenamelen = 0;
  static char statename[LIBXSVF_TAP_NAME_MAXLEN+1];
  static int8_t tstatename = -1; // tokenized state name

  if(c == '\0')
  { // reset parsing state
    state = SWPS_INIT;
    statenamelen = 0;
    tstatename = -1;
    return 0;
  }
  switch(state)
  {
    case SWPS_INIT:
      if(c >= 'A' && c <= 'Z')
      {
        if(statenamelen < LIBXSVF_TAP_NAME_MAXLEN)
          statename[statenamelen++] = c;
        else
        {
          // name too long, error
          statename[statenamelen] = '\0'; // 0-terminate
          state = SWPS_ERROR;
        }
        break;
      }
      if(c == ' ' || c == ';')
      {
        statename[statenamelen] = '\0'; // 0-terminate
        tstatename = search_name(statename, Tap_states);
        if(tstatename >= 0)
          PRINTF("tstatename '%s'", Tap_states[tstatename]);
        else
        {
          state = SWPS_ERROR;
          break;
        }
        if(c == ' ')
        {
          state = SWPS_SPACE;
          statenamelen = 0;
          tstatename = -1;
          break;
        }
        if(c == ';')
        {
          state = SWPS_COMPLETE;
          break;
        }
        break;
      }
      state = SWPS_ERROR;
      break;
    case SWPS_SPACE:
      if(c == ' ')
      {
        break;
      }
      if(c == ';')
      {
        state = SWPS_COMPLETE;
      }
      if(c >= 'A' && c <= 'Z')
      {
        if(statenamelen < LIBXSVF_TAP_NAME_MAXLEN)
          statename[statenamelen++] = c;
        else
        {
          // name too long, error
          statename[statenamelen] = '\0'; // 0-terminate
          state = SWPS_ERROR;
        }
        state = SWPS_INIT;
        break;
      }
      state = SWPS_ERROR;
      break;
    default:
      break;
  }
  return 0;
}

// transition between two states
// with given clock count and timing
int8_t cmd_runtest(char c)
{
  static int8_t state = RTPS_INIT;
  static int wordlen = 0;
  static char word[RUNTEST_NAME_MAXLEN+1];
  static int8_t tstatename = -1; // tokenized state name
  static int8_t trtword = -1; // tokenized runtest word
  static int8_t trtword_prev = -1; // tokenized runtest word
  static int8_t tendstatename = -1; // tokenized state name
  static struct S_float mintime, maxtime;
  // static uint32_t run_count = 0;
  if(c == '\0')
  { // reset parsing state
    state = RTPS_INIT;
    wordlen = 0;
    tstatename = -1;
    trtword = -1;
    trtword_prev = -1;
    tendstatename = -1;
    memset(&mintime, 0, sizeof(struct S_float));
    memset(&maxtime, 0, sizeof(struct S_float));
    return 0;
  }
  switch(state)
  {
    case RTPS_INIT:
      // state name doesn't start with T or S
      // so we can detect clock by its first letter
      if(c >= 'A' && c <= 'Z')
      {
        wordlen = 0;
        word[wordlen++] = c;
        state = RTPS_WORD;
        break;
      }
      if(c >= '0' && c <= '9')
      {
        parse_float('\0');
        parse_float(c);
        state = RTPS_NUMBER;
        break;
      }
      if(c == ';')
      {
        state = RTPS_COMPLETE;
        break;
      }
      state = RTPS_ERROR;
      break;
    case RTPS_WORD:
      if(c >= 'A' && c <= 'Z')
      {
        if(wordlen < RUNTEST_NAME_MAXLEN)
          word[wordlen++] = c;
        else
        {
          // name too long, error
          word[wordlen] = '\0'; // 0-terminate
          state = RTPS_ERROR;
        }
        break;
      }
      if(c == ' ' || c == ';')
      {
        word[wordlen] = '\0'; // 0-terminate
        tstatename = search_name(word, Tap_states);
        trtword = search_name(word, runtest_words);
        if(tstatename < 0 && trtword < 0)
        {
          state = RTPS_ERROR;
          break;
        }
        // there should be no common words
        // in Tap_states and runtest_words,
        // therefore either tstatename or trtword
        // should match, not both
        if(tstatename >= 0 && trtword >= 0)
        {
          printf("problem: double match tstatename and trtword '%s'", word);
          state = RTPS_ERROR;
          break;
        }
        if(tstatename >= 0)
        {
          if(trtword_prev == RT_WORD_ENDSTATE)
          {
            tendstatename = tstatename;
            PRINTF("tendstatename '%s'", Tap_states[tendstatename]);
          }
          else
            PRINTF("tstatename '%s'", Tap_states[tstatename]);
        }
        if(trtword >= 0)
        {
          PRINTF("trtword '%s'", runtest_words[trtword]);
          // at runtest word SCK or TCK -> run count
          // SEC -> min/max time
          if(trtword == RT_WORD_SCK || trtword == RT_WORD_TCK)
          {
            PRINTF("<-RUN COUNT");
          }
          if(trtword == RT_WORD_SEC)
          {
            if(trtword_prev == RT_WORD_MAXIMUM)
            {
              PRINTF("<-maxtime=%d.%dE%c%d ",
                maxtime.number, maxtime.frac, maxtime.expsign > 0 ? '+' : '-', maxtime.exponent);
            }
            else
            {
              PRINTF("<-mintime=%d.%dE%c%d ",
                mintime.number, mintime.frac, mintime.expsign > 0 ? '+' : '-', mintime.exponent);
            }
          }
        }
        trtword_prev = trtword;
        if(c == ';')
          state = RTPS_COMPLETE;
        else
          state = RTPS_INIT;
        break;
      }
      state = RTPS_ERROR;
      break;
    case RTPS_NUMBER:
      if( (c >= '0' && c <= '9')
        || c == '.' 
        || c == '+' || c == '-'
        || c == 'E' )
      {
        parse_float(c);
        if(fl.state == FLPS_ERROR)
        {
          PRINTF("float parse error");
          state = RTPS_ERROR;
        }
        break;
      }
      if(c == ' ' || c == ';')
      {
        if(trtword_prev == RT_WORD_MAXIMUM)
        {
          PRINTF("MAX:");
          memcpy(&maxtime, &fl, sizeof(struct S_float));
        }
        else
        {
          PRINTF("MIN:");
          memcpy(&mintime, &fl, sizeof(struct S_float));
        }
        PRINTF("FLOAT %d.%dE%c%d ",
          fl.number, fl.frac, fl.expsign > 0 ? '+' : '-', fl.exponent);
        if(c == ';')
          state = RTPS_COMPLETE;
        else
          state = RTPS_INIT;
        break;
      }
      state = RTPS_ERROR;
      break;
    default:
      break;
  }
  return 0;
}

// struct to command service functions
struct S_cmd_service
{
  int8_t (*service)(char);
};

struct S_cmd_service Cmd_service[] =
{
  [CMD_ENDDR] = { cmd_enddr },
  [CMD_ENDIR] = { cmd_endir },
  [CMD_FREQUENCY] = { cmd_frequency },
  [CMD_HDR] = { cmd_hdr },
  [CMD_HIR] = { cmd_hir },
  [CMD_PIO] = { cmd_pio },
  [CMD_PIOMAP] = { NULL },
  [CMD_RUNTEST] = { cmd_runtest },
  [CMD_SDR] = { cmd_sdr },
  [CMD_SIR] = { cmd_sir },
  [CMD_STATE] = { cmd_state },
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
              PRINTF("<found %s>", Commands[command]);
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

void init_reversenibble()
{
  uint8_t i,j,v,r;     // input bits to be reversed

  for (i = 0; i < 16; i++)
  {
    for (v = i, r = 0, j = 0; j < 4; j++)
    {   
      r <<= 1;
      r |= v & 1;
      v >>= 1;
    }
    #if REVERSE_NIBBLE
      ReverseNibble[i] = r;
      // PRINTF("%1X - %1X\n", i, r);
    #else
      ReverseNibble[i] = i;
    #endif
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
  PRINTF("index %d final %d\n", index, final);
  static uint8_t lstate = LS_SPACE;
  static uint32_t line_count = 0;
  static uint8_t lbracket = 0;
  static int8_t cmderr = 0;
  if(index == 0)
  {
    lstate = LS_SPACE;
    line_count = 0;
    lbracket = 0;
    init_reversenibble();
    jtag_open();
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
          PRINTF("_");
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
      c = toupper(c); // SVF is case insensitive
      PRINTF("%c", c);
      cmderr = commandstate(c);      
      if(cmderr > 0)
      {
        PRINTF("command %s complete\n", Commands[Completed_command]);
        play_buffer();
      }
    }
  }
  if(final)
    jtag_close();
  if(cmderr < 0)
    PRINTF("command incomplete\n");
  if(cmderr > 0)
    PRINTF("command complete\n");
  PRINTF("line count %d\n", line_count);
  return 0;
} 
