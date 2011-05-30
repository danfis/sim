#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "finder.hpp"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

namespace blobf {

static rgb_t imageRGB(osg::Image *img, int r, int c)
{
    rgb_t rgb;
    osg::Vec4 color;

    color = img->getColor(r, c);
    rgb.r = color.r();
    rgb.g = color.g();
    rgb.b = color.b();

    return rgb;
}

//prevod RGB -> HSV, prevzato z www
static void rgbToHsv(unsigned char r, unsigned char  g, unsigned char b,
                     unsigned int *hue, unsigned char *saturation, unsigned char *value)
{
    float min, max, tmp, delta;
    float h,s,v;   

    h=s=v=0; 
    *saturation = (unsigned char) s;
    *value = (unsigned char) v;
    *hue = (unsigned int) h;

    tmp = MIN(g, b);
    min = MIN(r, tmp);
    tmp = MAX(g, b);
    max = MAX(r, tmp);
    v = max;            

    delta = max - min;

    if( max != 0 ){
        tmp = delta*255 / max;    
        s = MIN(tmp, 255);    
    }else{
        s = 0;
        h = -1;
        return;
    }

    if( r == max )
        h = ( g - b ) / delta;        // between yellow & magenta
    else if( g == max )
        h = 2 + ( b - r ) / delta;    // between cyan & yellow
    else
        h = 4 + ( r - g ) / delta;    // between magenta & cyan
    h = h*60;
    if (h<0) h+=360;
    *saturation = (unsigned char) s;
    *value = (unsigned char) v;
    *hue = (unsigned int) h;
}

//podobnost pixelu - druha metoda z dilu III
static float evaluatePixel2(finder_t *f, rgb_t rgb)
{
    float result;
    int rr, gg, bb;

    rr = (int)rgb.r - (int)f->learned[0];
    gg = (int)rgb.g - (int)f->learned[1];
    bb = (int)rgb.b - (int)f->learned[2];

    result = rr * rr + gg * gg + bb * bb;
    result = sqrtf(result);

    return (result > f->tolerance ? 0 : 1);
}

static inline int evaluatePixelFast(finder_t *f, rgb_t rgb)
{
    int i = ((rgb.r / COLOR_STEP) * COLOR_PRECISION + rgb.g / COLOR_STEP) * COLOR_PRECISION + rgb.b / COLOR_STEP;
    return f->color[i];
}


finder_t *finderNew(unsigned int width, unsigned int height)
{
    finder_t *f;
    size_t memsize, size;
    size_t colorsize, segmentsize;
    size_t i;

    // required blocks for buffer
    size = width * height;

    // memory needed by finder_t struct
    colorsize = sizeof(unsigned char) *
                COLOR_PRECISION * COLOR_PRECISION * COLOR_PRECISION;
    segmentsize = sizeof(segment_t) * MAX_SEGMENTS;

    memsize  = sizeof(finder_t);
    memsize += colorsize;
    memsize += segmentsize;

    // alloc finder_t
    f = (finder_t *)malloc(memsize);
    f->color   = (unsigned char *)((char *)f) + sizeof(finder_t);
    f->segment = (segment_t *)(((char *)f->color) + colorsize);

    f->segment_len = 0;

    f->width = width;
    f->height = height;

    f->tolerance = 30; 

    // init arrays
    stackInit(&f->stack, int);

    f->buffer = (int *)malloc(sizeof(int) * size);
    if (!f->buffer){
        fprintf(stderr, "Can't allocate enough memory for buffer!\n");
        exit(-1);
    }

    for (i = 0; i < colorsize; i++){
        f->color[i] = 0;
    }

    return f;
}

void finderDel(finder_t *f)
{
    stackDestroy(&f->stack);

    free(f->buffer);

    free(f);
}

void finderAddPixel(finder_t *f, rgb_t rgb)
{
    int i, r, g, b;
    rgb_t c;

    //ulozi pixel do vzoroveho
    f->learned[0] = rgb.r;
    f->learned[1] = rgb.g;
    f->learned[2] = rgb.b;
    fprintf(stderr, "AddPixel: %d %d %d\n", rgb.r, rgb.g, rgb.b);

    //prevede nauceny pixel do HSV
    rgbToHsv(f->learned[0], f->learned[1], f->learned[2],
             &f->learnedHue, &f->learnedSaturation, &f->learnedValue);

    //z daneho vzoru vytvori indexovaci tabulku
    for (r = 0; r < 256; r += 4){
        for (g = 0; g < 256; g += 4){
            for (b = 0; b < 256; b += 4){
                c.r = r;
                c.g = g;
                c.b = b;
                if (evaluatePixel2(f, c) > 0){
                    i = ((c.r/COLOR_STEP)*COLOR_PRECISION+c.g/COLOR_STEP)*COLOR_PRECISION+c.b/COLOR_STEP;
                    f->color[i] = 1;
                }
            }
        }
    }
    fprintf(stderr,"Learned RGB: %d %d %d, HSV: %d %d %d\n",
            f->learned[0], f->learned[1], f->learned[2],
            f->learnedHue, f->learnedSaturation, f->learnedValue);
}

static inline void finderMarkAreas(finder_t *f, osg::Image *img)
{
    int eval;
    size_t i, j, k, w, h;
    rgb_t rgb;

    w = img->s();
    h = img->t();
    k = 0;
    for (i = 0; i < w; i++){
        for (j = 0; j < h; j++){
            rgb = imageRGB(img, i, j);
            eval = -evaluatePixelFast(f, rgb);
            finderBufferSet(f, k, eval);
            k++;
        }
    }

#if 0
    {
        size_t img_blocks, img_block, img_els;
        size_t buf_block;
        rgb_t *rgb, *rgb_end;
        int *buf;

        len = img->width * img->height;
        img_blocks = arrBlocks(img->arr);
        buf_block = 0;
        buf = f->buffer->mem[0];
        img_els = 0;
        for (img_block = 0; img_block < img_blocks && img_els < len; img_block++){
            rgb = (rgb_t *)arrBlock(img->arr, img_block);
            rgb_end = (rgb_t *)((char *)rgb + arrBlockSize(img->arr));

            for (;rgb != rgb_end && img_els < len; rgb++){
                eval = -evaluatePixelFast(f, *rgb);

                *buf = eval;

                if ((char *)++buf >= (char *)f->buffer->mem[buf_block] + arrBlockSize(f->buffer)){
                    ++buf_block;
                    buf = f->buffer->mem[buf_block];
                }

                //fprintf(stderr, "eval: %d\n", (int)eval);
                img_els++;
            }
        }

    }
#endif
}

static inline void finderZeroBorders(finder_t *f, osg::Image *img)
{
    size_t i;

    for (i = 0; i < (size_t)img->s(); i++){
        finderBufferSet(f, i, 0);
        finderBufferSet(f, i + f->width * (f->height - 1), 0);
    }
    for (i = 0; i < (size_t)img->t(); i++){
        finderBufferSet(f, i * f->width, 0);
        finderBufferSet(f, i * f->width + f->width - 1, 0);
    }
}


static inline void finderLocateNewSegment(finder_t *f, osg::Image *img, size_t p)
{
    int expand[4] = { img->s(), -img->s(), 1, -1 };
    segment_t *curseg; // current segment
    size_t j;
    int pos = p, pos2;

    if (f->segment_len >= MAX_SEGMENTS)
        return;

    // alloc new segment
    curseg = f->segment + f->segment_len;
    f->segment_len++;

    // set up segment
    curseg->size = 1;
    curseg->x = pos % img->s();
    curseg->y = pos / img->s();

    // mark position in buffer with segment id
    finderBufferSet(f, pos, f->segment_len);

    // put coordinate of pixel on top of stack
    stackPush(&f->stack, int, pos);

    // do depth-first search
    while (!stackEmpty(&f->stack)){
        pos = stackTop(&f->stack, int);
        stackPop(&f->stack, int);

        // search neighbor pixels
        for (j = 0; j < 4; j++){
            pos2 =  pos + expand[j];

            // if pixel has same color
            if (finderBufferGet(f, pos2) < 0){
                stackPush(&f->stack, int, pos2);

                // add coordinates to segment
                curseg->x += pos2 % img->s();
                curseg->y += pos2 / img->s();

                // increase segment size
                curseg->size++;

                // mark position in buffer with segment id
                finderBufferSet(f, pos2, f->segment_len);
            }
        }
    }

    // compute center of located segment
    curseg->x /= curseg->size;
    curseg->y /= curseg->size;
    fprintf(stderr, "curseg: x: %d, y: %d, size: %d\n",
            curseg->x, curseg->y, curseg->size);
}

segment_t finderFindSegment(finder_t *f, osg::Image *img)
{
    segment_t result; // result - stores biggest segment
    int eval;
    size_t i, len;

    result.x = 0;
    result.y = 0;
    result.size = 0;


    finderMarkAreas(f, img);
    finderZeroBorders(f, img);


    // search for segment
    len = img->s() * img->t();
    for (i = 0; i < len && f->segment_len < MAX_SEGMENTS; i++){
        eval = finderBufferGet(f, i);

        if (eval < 0){
            finderLocateNewSegment(f, img, i);

            // store largest segment
            if (f->segment[f->segment_len - 1].size > result.size){
                result = f->segment[f->segment_len - 1];
            }
        }
    }


    /*
    //vykreslime vysledek
    int j = 0;
    for (int i = 0;i<len&&false;i++){
        j = buffer[i];
        if (j > 0){
            image->data[i*3+j%3] = 0;
            image->data[i*3+(j+1)%3] = 255;
            image->data[i*3+(j+2)%3] = 255;
        }
    }
    fprintf(stdout,"T5:%i\n",timer.getTime());timer.reset();timer.start();
    */
    return result;
}

} /* namespace blobf */

