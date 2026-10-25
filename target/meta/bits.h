#ifndef _TARGETMETA_BITS_H_
#define _TARGETMETA_BITS_H_

static inline uint32_t bitreverse_i32(uint32_t val)
{
    /* swap odd/even bits */
    val = ((val >> 1) & 0x55555555) | ((val & 0x55555555) << 1);

    /* swap consecutive bit pairs */
    val = ((val >> 2) & 0x33333333) | ((val & 0x33333333) << 2);

    /* swap consecutive nibbles */
    val = ((val >> 4) & 0x0f0f0f0f) | ((val & 0x0f0f0f0f) << 4);

    /* swap consecutive bytes */
    val = ((val >> 8) & 0x00ff00ff) | ((val & 0x00ff00ff) << 8);

    /* swap shorts */
    return (val >> 16) | (val << 16);
}

#endif /* _TARGETMETA_BITS_H_ */
