#include <stdint.h>

#define CTYPE_U 01
#define CTYPE_L 02
#define CTYPE_N 04
#define CTYPE_S 010
#define CTYPE_P 020
#define CTYPE_C 040
#define CTYPE_X 0100
#define CTYPE_B 0200

/*
 * libmorse expects legacy newlib ctype symbols. Provide a compact table for
 * newlib/libstdc++ builds where those symbols are not supplied by Zephyr.
 */
const char _ctype_[257] = {
	[0x00 + 1 ... 0x08 + 1] = CTYPE_C,
	['\t' + 1 ... '\r' + 1] = CTYPE_S | CTYPE_C,
	[0x0e + 1 ... 0x1f + 1] = CTYPE_C,
	[' ' + 1] = CTYPE_S | CTYPE_B,
	['!' + 1 ... '/' + 1] = CTYPE_P,
	['0' + 1 ... '9' + 1] = CTYPE_N,
	[':' + 1 ... '@' + 1] = CTYPE_P,
	['A' + 1 ... 'F' + 1] = CTYPE_U | CTYPE_X,
	['G' + 1 ... 'Z' + 1] = CTYPE_U,
	['[' + 1 ... '`' + 1] = CTYPE_P,
	['a' + 1 ... 'f' + 1] = CTYPE_L | CTYPE_X,
	['g' + 1 ... 'z' + 1] = CTYPE_L,
	['{' + 1 ... '~' + 1] = CTYPE_P,
	[0x7f + 1] = CTYPE_C,
};

const uint16_t _ctype_b[384] = {0};

void _fini(void)
{
}
