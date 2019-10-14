#include "lego/pkt_serialization.h"

int8_t get_int8(uint8_t *buf, size_t *idx)
{
  uint8_t v;
  v = buf[*idx] & 0xFF;
  (*idx)++;
  return v;
}

void put_int8(uint8_t *buf, size_t *idx, int8_t val)
{
  buf[*idx] = val & 0xFF;
  (*idx)++;
}

int16_t get_int16(uint8_t *buf, size_t *idx)
{
  uint16_t v;
  v = (buf[*idx] & 0xFF) << 8;
  (*idx)++;
  v |= (buf[*idx] & 0xFF);
  (*idx)++;
  return v;
}

void put_int16(uint8_t *buf, size_t *idx, int16_t val)
{
  buf[*idx] = (val >> 8) & 0xFF;
  (*idx)++;
  buf[*idx] = val & 0xFF;
  (*idx)++;
}

int32_t get_int32(uint8_t *buf, size_t *idx)
{
  uint32_t v;
  v  = (buf[*idx] & 0xFF) << 24;
  (*idx)++;
  v |= (buf[*idx] & 0xFF) << 16;
  (*idx)++;
  v |= (buf[*idx] & 0xFF) << 8;
  (*idx)++;
  v |= (buf[*idx] & 0xFF);
  (*idx)++;
  return v;
}

void put_int32(uint8_t *buf, size_t *idx, int32_t val)
{
  buf[*idx] = (val >> 24) & 0xFF;
  (*idx)++;
  buf[*idx] = (val >> 16) & 0xFF;
  (*idx)++;
  buf[*idx] = (val >> 8) & 0xFF;
  (*idx)++;
  buf[*idx] = val & 0xFF;
  (*idx)++;
}

float get_float(uint8_t *buf, size_t *idx)
{
  union 
  {
    float f;
    uint32_t bytes;
  } v;

  v.bytes  = (buf[*idx] & 0xFF) << 24;
  (*idx)++;
  v.bytes |= (buf[*idx] & 0xFF) << 16;
  (*idx)++;
  v.bytes |= (buf[*idx] & 0xFF) << 8;
  (*idx)++;
  v.bytes |= (buf[*idx] & 0xFF);
  (*idx)++;
  return v.f;
}

void put_float(uint8_t *buf, size_t *idx, float val)
{
  union 
  {
    float f;
    uint32_t bytes;
  } v;
  v.f = val;

  buf[*idx] = (v.bytes >> 24) & 0xFF;
  (*idx)++;
  buf[*idx] = (v.bytes >> 16) & 0xFF;
  (*idx)++;
  buf[*idx] = (v.bytes >> 8) & 0xFF;
  (*idx)++;
  buf[*idx] = v.bytes & 0xFF;
  (*idx)++;
}



void fill_cmd_msg(struct cmd_pkt * cmd, uint8_t * buf, size_t * size)
{
  size_t idx = 0;
  put_int16(buf, &idx, CMD_PKT_PAYLOAD_SIZE); // payload size
  put_int8(buf, &idx, TYPE_CMD_PKT);
  put_float(buf, &idx, cmd->v);
  put_float(buf, &idx, cmd->omega);
  *size = idx;
}

void parse_cmd_msg(uint8_t * buf, struct cmd_pkt * cmd)
{
  size_t idx = 0;
  cmd->v     = get_float(buf, &idx);
  cmd->omega = get_float(buf, &idx);
}

void fill_status_msg(struct status_pkt * status, uint8_t * buf, size_t * size)
{
  size_t idx = 0;
  put_int16(buf, &idx, STATUS_PKT_PAYLOAD_SIZE); // payload size: 1 int32 + 5 float
  put_int8(buf, &idx, TYPE_STATUS_PKT);
  put_int32(buf, &idx, status->time);
  put_float(buf, &idx, status->v);
  put_float(buf, &idx, status->omega);
  put_float(buf, &idx, status->R11);
  put_float(buf, &idx, status->R12);
  put_float(buf, &idx, status->R22);
  *size = idx;
}

void parse_status_msg(uint8_t *buf, struct status_pkt *status)
{
  size_t   idx  = 0;
  status->time  = get_int32(buf, &idx);
  status->v     = get_float(buf, &idx);
  status->omega = get_float(buf, &idx);
  status->R11   = get_float(buf, &idx);
  status->R12   = get_float(buf, &idx);
  status->R22   = get_float(buf, &idx);
}