#ifndef __SMARTCARD_PRINT_H__
#define __SMARTCARD_PRINT_H__

#define SMARTCARD_DEBUG CONFIG_SMARTCARD_DEBUG
#define MEASURE_TOKEN_PERF

/* Primitive for debug output */
#if SMARTCARD_DEBUG
#define log_printf(...) printf(__VA_ARGS__)
#else
#define log_printf(...)
#endif

/* This function is a simple (non-optimized) reimplementation of memcpy() */
static inline void local_memcpy(void *dst, const void *src, uint32_t n)
{
        const uint8_t *lsrc = (const uint8_t*)src;
        uint8_t *ldst = (uint8_t*)dst;
        uint32_t i;

        for (i = 0; i < n; i++) {
                *ldst = *lsrc;
                ldst++;
                lsrc++;
        }
}

/* This function is a simple (non-optimized) reimplementation of memset() */
static inline void local_memset(void *v, uint8_t c, uint32_t n)
{
        volatile uint8_t *p = (volatile uint8_t*)v;
        uint32_t i;

        for (i = 0; i < n; i++) {
                *p = c;
                p++;
        }
}

#endif /* _SMARTCARD_PRINT_H__ */
