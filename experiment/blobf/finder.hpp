#ifndef _FINDER_H_
#define _FINDER_H_

#include <stddef.h>
#include "arr.hpp"
#include "stack.hpp"
#include <sim/sensor/camera.hpp>

#define MAX_SEGMENTS 10
#define COLOR_PRECISION 32
#define COLOR_STEP 8
#define BLOCK_SIZE 4096

namespace blobf {

struct _segment_t {
    int x, y, size;
};
typedef struct _segment_t segment_t;

struct _rgb_t {
    unsigned char r, g, b;
};
typedef struct _rgb_t rgb_t;

struct _finder_t {
    unsigned int width, height;

    unsigned char *color;
    unsigned char learned[3];
    unsigned int learnedHue;
    unsigned char learnedSaturation,learnedValue;

    segment_t *segment;
    size_t segment_len;

    stack_t stack;

    int *buffer;

    int tolerance;
};
typedef struct _finder_t finder_t;


finder_t *finderNew(unsigned int width, unsigned int height);
void finderDel(finder_t *);

void finderAddPixel(finder_t *f, rgb_t rgb);

segment_t finderFindSegment(finder_t *f, osg::Image *img);

static inline void finderBufferSet(finder_t *f, unsigned int pos, int val)
{
    f->buffer[pos] = val;
}

static inline int finderBufferGet(finder_t *f, unsigned int pos)
{
    return f->buffer[pos];
}

} /* namespace blobf */

#endif /* _FINDER_H_ */
