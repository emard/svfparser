#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "svfparser.h"

// get chunk by chunk (simulate network) and call the parser
int packetize(char *filename, size_t size)
{
  FILE *fp = fopen(filename, "rb");
  if(fp == NULL)
  {
    printf("can't open %s\n", filename);
    return -1;
  }
  uint8_t *packet_data = (uint8_t *) malloc(size * sizeof(uint8_t));
  size_t packet_len;
  size_t index = 0;

  while(!feof(fp))
  {
    packet_len = fread(packet_data, 1, size, fp);
    int final = packet_len < size ? 1 : 0;
    parse_svf_packet(packet_data, index, packet_len, final);
    index += packet_len;
    printf("packet len %d\n", packet_len);
  }
  printf("total len %d\n", index);
  free(packet_data);
  return 0;
}


int main(int argc, char *argv[])
{
  puts("svf parser");
  if(argc > 1)
    packetize(argv[1], 1436);
}
