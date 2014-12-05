#include <string.h>
#include "c_types.h"
#include "crc32.h"

static const uint32_t crc32_table[] = {
0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
};

uint32_t ICACHE_FLASH_ATTR
crc32_step(uint32_t checksum, uint8_t data)
{
  uint8_t tbl_idx;
  tbl_idx = checksum ^ (data >> (0 * 4));
  checksum = *(crc32_table + (tbl_idx & 0x0f)) ^ (checksum >> 4);
  tbl_idx = checksum ^ (data >> (1 * 4));
  checksum = *(crc32_table + (tbl_idx & 0x0f)) ^ (checksum >> 4);
  return checksum;
}

uint32_t ICACHE_FLASH_ATTR
crc32_update(uint32_t checksum, const uint8_t* data, size_t size)
{
  while(size--) checksum = crc32_step(checksum, *data++);
  return checksum;
}

uint32_t ICACHE_FLASH_ATTR
crc32_checksum(const uint8_t* data, size_t size)
{
  return ~crc32_update(~0L, data, size);
}

