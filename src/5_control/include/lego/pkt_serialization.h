#ifndef PKT_SERIALIZATION_H
#define PKT_SERIALIZATION_H

#include <stdint.h>
#include <stddef.h>

#define PKT_SIZE_BYTES  2
#define PKT_TYPE_BYTES  1
#define PKT_HEADER_SIZE (PKT_SIZE_BYTES+PKT_TYPE_BYTES)
#define PKT_BUFFER_SIZE 64

#define STATUS_PKT_PAYLOAD_SIZE (1*sizeof(int32_t)+5*sizeof(float))
#define STATUS_PKT_SIZE         (PKT_HEADER_SIZE+STATUS_PKT_PAYLOAD_SIZE)

#define CMD_PKT_PAYLOAD_SIZE 2*sizeof(float) 

#define TYPE_STATUS_PKT 1
#define TYPE_CMD_PKT    2


struct status_pkt
{
  uint32_t time;
  float    v;
  float    omega;
  float    R11;
  float    R12;
  float    R22;
};

struct cmd_pkt
{
  float v;
  float omega;
};


int8_t get_int8(uint8_t * buf, size_t * idx);
void put_int8(uint8_t * buf, size_t * idx, int8_t val);

int16_t get_int16(uint8_t * buf, size_t * idx);
void put_int16(uint8_t * buf, size_t * idx, int16_t val);

int32_t get_int32(uint8_t * buf, size_t * idx);
void put_int32(uint8_t * buf, size_t * idx, int32_t val);

float get_float(uint8_t * buf, size_t * idx);
void put_float(uint8_t * buf, size_t * idx, float val);

void fill_cmd_msg(struct cmd_pkt * cmd, uint8_t * buf, size_t * size);
void parse_cmd_msg(uint8_t * buf, struct cmd_pkt * cmd);

void fill_status_msg(struct status_pkt * status, uint8_t * buf, size_t * size);
void parse_status_msg(uint8_t * buf, struct status_pkt *status);

#endif