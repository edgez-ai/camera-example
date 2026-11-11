#ifndef EDGE_DEVICE_ENDIAN_H
#define EDGE_DEVICE_ENDIAN_H

#include <stdint.h>

#define __LITTLE_ENDIAN 1234
#define __BIG_ENDIAN 4321
#define __BYTE_ORDER __LITTLE_ENDIAN

static inline uint16_t edge_bswap16(uint16_t value)
{
	return __builtin_bswap16(value);
}

static inline uint32_t edge_bswap32(uint32_t value)
{
	return __builtin_bswap32(value);
}

static inline uint64_t edge_bswap64(uint64_t value)
{
	return __builtin_bswap64(value);
}

#define htole16(value) ((uint16_t)(value))
#define htole32(value) ((uint32_t)(value))
#define htole64(value) ((uint64_t)(value))
#define le16toh(value) ((uint16_t)(value))
#define le32toh(value) ((uint32_t)(value))
#define le64toh(value) ((uint64_t)(value))

#define htobe16(value) edge_bswap16((uint16_t)(value))
#define htobe32(value) edge_bswap32((uint32_t)(value))
#define htobe64(value) edge_bswap64((uint64_t)(value))
#define be16toh(value) edge_bswap16((uint16_t)(value))
#define be32toh(value) edge_bswap32((uint32_t)(value))
#define be64toh(value) edge_bswap64((uint64_t)(value))

#endif
