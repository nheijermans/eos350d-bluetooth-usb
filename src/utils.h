#ifndef UTILS_H
#define UTILS_H


#ifndef MIN
# define MIN(a, b) ((a) < (b)? (a): (b))
#endif

#ifndef MAX
# define MAX(a, b) ((a) > (b)? (a): (b))
#endif

#define PASTE2(a, b)	a ## b
#define PASTE(a, b)		PASTE2(a, b)

/** @brief Convenience macro to define packed structures more clearly and concisely. */
#define PACKED __attribute__((__packed__))

#define MULTI_LINE_MACRO(body) \
    do { \
        body; \
    } while (0)


#define ARRAY_LENGTH(x) \
    (sizeof((x)) / sizeof((x)[0]))


#define storeLittleU32(buf, value) \
    MULTI_LINE_MACRO( \
        (((uint8_t*)(buf))[0]) = ((value >> 0)  & 0xff); \
        (((uint8_t*)(buf))[1]) = ((value >> 8)  & 0xff); \
        (((uint8_t*)(buf))[2]) = ((value >> 16) & 0xff); \
        (((uint8_t*)(buf))[3]) = ((value >> 24) & 0xff)  \
    )

#define storeBigU32(buf, value) \
    MULTI_LINE_MACRO( \
        (((uint8_t*)(buf))[3]) = ((value >> 0)  & 0xff); \
        (((uint8_t*)(buf))[2]) = ((value >> 8)  & 0xff); \
        (((uint8_t*)(buf))[1]) = ((value >> 16) & 0xff); \
        (((uint8_t*)(buf))[0]) = ((value >> 24) & 0xff)  \
    )

static inline
uint32_t fetchBigU32(uint8_t *p)
{
    return ((p[0] << 24) |
		 	(p[1] << 16) |
		 	(p[2] <<  8) |
		 	(p[3] <<  0));
}

static inline
uint32_t fetchLittleU32(uint8_t *p)
{
    return ((p[3] << 24) |
		 	(p[2] << 16) |
		 	(p[1] <<  8) |
		 	(p[0] <<  0));
}

void
dump_hex(uint8_t *buf, uint32_t buf_size);

#define DEBUG

#ifdef DEBUG
# define DBG_TRACE(x) printf x
#else
# define DBG_TRACE(x)
#endif

#endif /* UTILS_H */
