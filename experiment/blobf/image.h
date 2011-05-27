#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "arr.h"

#define IMAGE_MEMBLOCK_SIZE 4096

struct _rgb_t {
    unsigned char r, g, b;
};
typedef struct _rgb_t rgb_t;

struct _image_t {
    unsigned int width, height; /*! width and height of image */

#ifdef USE_ARRAY
    int bigalloc;
    rgb_t *arr;
#else /* USE_ARRAY */
    arr_t *arr;
#endif /* USE_ARRAY */
};
typedef struct _image_t image_t;

image_t *imageNew(unsigned int width, unsigned int height);
void imageDel(image_t *);

/**
 * Loadsd bmp image from file descriptor.
 */
int imageLoadBmp(image_t *img, const char *path);

static inline rgb_t imageGet(image_t *img, unsigned int w, unsigned int h);
static inline rgb_t imageGetSeq(image_t *img, unsigned int pos);

static inline rgb_t imageGet(image_t *img, unsigned int w, unsigned int h)
{
    unsigned int pos;
    pos   = w + img->width * h;
    return imageGetSeq(img, pos);
}

static inline rgb_t imageGetSeq(image_t *img, unsigned int pos)
{
#ifdef USE_ARRAY
    return img->arr[pos];
#else /* USE_ARRAY */
    return arrGet(img->arr, rgb_t, pos);
#endif /* USE_ARRAY */
}

#endif /* _IMAGE_H_ */
