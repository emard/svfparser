#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>


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
  // size_t fread(void *ptr, size_t size, size_t nmemb, FILE *stream);
  size_t packet_len;
  size_t total_len = 0;

  while(!feof(fp))
  {
    packet_len = fread(packet_data, 1, size, fp);
    total_len += packet_len;
    printf("packet len %d\n", packet_len);
  }
  printf("total len %d\n", total_len);
  free(packet_data);
  return 0;
}


int main(int argc, char *argv[])
{
  puts("svf parser");
  if(argc > 1)
    packetize(argv[1], 1436);
}