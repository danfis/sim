#ifndef _FINDER_H_
#define _FINDER_H_

#include <stddef.h>
#include "image.h"
#include "arr.h"
#include "stack.h"

#define MAX_SEGMENTS 10
#define COLOR_PRECISION 32
#define COLOR_STEP 8
#define BLOCK_SIZE 4096

struct _segment_t {
    int x, y, size;
};
typedef struct _segment_t segment_t;

struct _finder_t {
    unsigned int width, height;

    unsigned char *color;
    unsigned char learned[3];
    unsigned int learnedHue;
    unsigned char learnedSaturation,learnedValue;

    segment_t *segment;
    size_t segment_len;

    stack_t stack;

#ifdef USE_ARRAY
    int bigalloc;
    int *buffer;
#else /* USE_ARRAY */
    arr_t *buffer;
#endif /* USE_ARRAY */

    int tolerance;
};
typedef struct _finder_t finder_t;


finder_t *finderNew(unsigned int width, unsigned int height);
void finderDel(finder_t *);

void finderAddPixel(finder_t *f, rgb_t rgb);

segment_t finderFindSegment(finder_t *f, image_t *img);

static inline void finderBufferSet(finder_t *f, unsigned int pos, int val)
{
#ifdef USE_ARRAY
    f->buffer[pos] = val;
#else /* USE_ARRAY */
    arrSet(f->buffer, int, pos, val);
#endif /* USE_ARRAY */
}

static inline int finderBufferGet(finder_t *f, unsigned int pos)
{
#ifdef USE_ARRAY
    return f->buffer[pos];
#else /* USE_ARRAY */
    return arrGet(f->buffer, int, pos);
#endif
}

#endif /* _FINDER_H_ */
