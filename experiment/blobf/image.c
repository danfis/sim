#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "image.h"

image_t *imageNew(unsigned int width, unsigned int height)
{
    image_t *img;
    size_t size;
    size_t memsize;
#ifndef USE_ARRAY
    size_t arrsize;
#else /* USE_ARRAY */
    int res;
#endif /* USE_ARRAY */

    // required size
    size = width * height;
#ifndef USE_ARRAY
    arrsize = arrRequiredStructSize(rgb_t, size);
#endif /* USE_ARRAY */

    // compute memsize for image_t struct
    memsize  = sizeof(image_t);
#ifndef USE_ARRAY
    memsize += arrsize;
#endif /* USE_ARRAY */

    // allocate image_t
    img = (image_t *)malloc(memsize);
    if (!img){
        fprintf(stderr, "Can't allocate image (%d bytes)\n", (int)memsize);
        exit(-1);
    }
    img->width  = width;
    img->height = height;

    // initialize array
#ifdef USE_ARRAY
    // first try bigalloc
    img->bigalloc = 0;

    res = bigallocChunkAcquire(0, (void **)&img->arr, &memsize);
    if (res == 0){
        if (memsize < size * sizeof(rgb_t)){
            fprintf(stderr, "Bigalloc's chunk 0 is not big enough.\n");
            fprintf(stderr, "    Chunk's size: %ld, required: %ld\n",
                    (long)memsize, (long)(size * sizeof(rgb_t)));
            bigallocChunkRelease(0);
            img->arr = NULL;
        }else{
            img->bigalloc = 1;
        }
    }

    if (!img->bigalloc){
        img->arr = (rgb_t *)malloc(sizeof(rgb_t) * size);
        if (!img->arr){
            fprintf(stderr, "Can't allocate enough memory for image!\n");
            exit(-1);
        }
    }
#else /* USE_ARRAY */
    img->arr = (arr_t *)(((char *)img) + sizeof(image_t));
    arrInit(img->arr, rgb_t, size);
#endif /* USE_ARRAY */

    return img;
}

void imageDel(image_t *img)
{
#ifdef USE_ARRAY
    if (img->bigalloc){
        bigallocChunkRelease(0);
    }else{
        free(img->arr);
    }
#else /* USE_ARRAY */
    arrDestroy(img->arr);
#endif /* USE_ARRAY */

    free(img);
}

int imageLoadBmp(image_t *img, const char *path)
{
    int fd;
    off_t off;
    size_t len;
#ifndef USE_ARRAY
    size_t curblock, blocks;
    void *buf;
#endif /* USE_ARRAY */

    fd = open(path, O_RDONLY);
    if (fd <= 0)
        return -1;

    // skip header (54 bytes)
    off = lseek(fd, 54, SEEK_CUR);
    if (off == (off_t)-1)
        return -1;

    // fill memory blocks with image data
#ifndef USE_ARRAY
    blocks = arrBlocks(img->arr);
    for (curblock = 0; curblock < blocks; curblock++){
        buf = arrBlock(img->arr, curblock);
        len = read(fd, buf, arrBlockSize(img->arr));
        //fprintf(stderr, "%d - %d\n", curblock, len);
        // TODO: add check and loop for reading whole MEMBLOCK_SIZE
    }
#else /* USE_ARRAY */
    len = read(fd, img->arr, img->width * img->height * sizeof(rgb_t));
#endif /* USE_ARRAY */

    close(fd);
    
    return 0;
}
