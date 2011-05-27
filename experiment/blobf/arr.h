#ifndef __ARR_H__
#define __ARR_H__

#include <stddef.h>
#include <stdlib.h>

//#define ARR_BLOCK_SIZE 4096 /*! size of block in bytes */
#define ARR_BLOCK_SIZE 8192
//#define ARR_BLOCK_SIZE 16384
//#define ARR_BLOCK_SIZE 32768
//#define ARR_BLOCK_SIZE 65536
//#define ARR_BLOCK_SIZE 131072
//#define ARR_BLOCK_SIZE 262144
//#define ARR_BLOCK_SIZE 111

struct _arr_t {
    void **mem;
    size_t mem_size;
    size_t block_size; /*! precomputed blocks due to alignement of elements */
};
typedef struct _arr_t arr_t;

/**
 * Returns number of bytes required for struct holding array, i.e. amount
 * of overhead.
 */
#define arrRequiredStructSize(type, nmemb) \
    __arrRequiredStructSize(sizeof(type), (nmemb))
static inline size_t __arrRequiredStructSize(size_t typesize, size_t nmemb);

/**
 * Returns number of required blocks to cover given number of bytes.
 */
static inline size_t __arrRequiredBlocks(size_t typesize, size_t nmemb);

#define arrRequiredBlockSize(type) \
    __arrRequiredBlockSize(sizeof(type))
static inline size_t __arrRequiredBlockSize(size_t typesize);

/**
 * Returns new struct holding array allocated on heap.
 */
#define arrNew(type, nmemb) \
    __arrNew(sizeof(type), (nmemb))
static inline arr_t *__arrNew(size_t typesize, size_t nmemb);

/**
 * Initializes struct holding array in given memory.
 * The minimal amount of allocated memory at ptr can be get by
 * arrRequiredStructSize() function.
 */
#define arrInit(ptr, type, nmemb) \
    __arrInit((ptr), sizeof(type), (nmemb))
static inline void __arrInit(void *ptr, size_t typesize, size_t nmemb);

/**
 * Deletes array and free allocated memory by arrNew().
 */
static inline void arrDel(arr_t *arr);

/**
 * Destroys array initialized by arrInit().
 */
static inline void arrDestroy(arr_t *arr);


/**
 * Returns n'th element of array (where each element has given type).
 */
#define arrGet(arr, type, n) \
    *(type *)__arrGet((arr), sizeof(type), (n))
static inline void *__arrGet(arr_t *arr, size_t typesize, size_t n);

/**
 * Sets n'th element of array.
 */
#define arrSet(arr, type, n, val) \
    (*(type *)__arrGet((arr), sizeof(type), (n)) = val)

/**
 * Returns number of allocated blocks.
 */
static inline size_t arrBlocks(const arr_t *arr);

/**
 * Returns block size.
 */
static inline size_t arrBlockSize(const arr_t *arr);

/**
 * Returns number of elements of given type that can block contain.
 */
#define arrElementsPerBlock(arr, type) \
    (arrBlockSize(arr) / sizeof(type))

/**
 * Returns n'th block.
 */
static inline void *arrBlock(arr_t *arr, size_t n);



/**
 * Iterator for arr_t arrays.
 * See arrIt*() functions.
 */
struct _arr_it_t {
    arr_t *arr;
    size_t curblock;
    void *val;
};
typedef struct _arr_it_t arr_it_t;

static inline void arrItInit(arr_it_t *it, arr_t *arr);

#define arrItGet(it, type) \
    *(type *)(it)->val

#define arrItSet(it, type, v) \
    (*(type *)(it)->val = (v))

#define arrItNext(it, type) \
    __arrItNext((it), sizeof(type))

static inline void __arrItNext(arr_it_t *it, size_t typesize);


/**** INLINES ****/
static inline size_t __arrRequiredStructSize(size_t typesize, size_t nmemb)
{
    size_t blocks, required;

    blocks = __arrRequiredBlocks(typesize, nmemb);
    required  = sizeof(arr_t);
    required += sizeof(void *) * blocks;

    return required;
}

static inline size_t __arrRequiredBlocks(size_t typesize, size_t nmemb)
{
    size_t blocks, blocksize, bytes;

    blocksize = __arrRequiredBlockSize(typesize);
    bytes = typesize * nmemb;

    blocks = bytes / blocksize;
    if (bytes % blocksize > 0)
        blocks++;

    return blocks;
}

static inline size_t __arrRequiredBlockSize(size_t typesize)
{
    return ARR_BLOCK_SIZE - (ARR_BLOCK_SIZE % typesize);
}

static inline void __arrInit(void *ptr, size_t typesize, size_t nmemb)
{
    arr_t *arr = (arr_t *)ptr;
    size_t i;

    arr->mem = (void **)(((char *)ptr) + sizeof(arr_t));

    arr->mem_size = __arrRequiredBlocks(typesize, nmemb);
    arr->block_size = __arrRequiredBlockSize(typesize);
    for (i = 0; i < arr->mem_size; i++){
        arr->mem[i] = malloc(arr->block_size);
    }
}

static inline arr_t *__arrNew(size_t typesize, size_t nmemb)
{
    void *mem;
    mem = malloc(__arrRequiredStructSize(typesize, nmemb));
    __arrInit(mem, typesize, nmemb);
    return mem;
}

static inline void arrDestroy(arr_t *arr)
{
    size_t i;

    for (i = 0; i < arr->mem_size; i++){
        free(arr->mem[i]);
    }
}

static inline void arrDel(arr_t *arr)
{
    arrDestroy(arr);
    free(arr);
}

static inline void *__arrGet(arr_t *arr, size_t typesize, size_t n)
{
    size_t pos, block, off;

    pos = typesize * n;
    block = pos / arr->block_size;
    off   = pos % arr->block_size;

    return ((char *)arr->mem[block]) + off;
}

static inline size_t arrBlocks(const arr_t *arr)
{
    return arr->mem_size;
}

static inline size_t arrBlockSize(const arr_t *arr)
{
    return arr->block_size;
}

static inline void *arrBlock(arr_t *arr, size_t n)
{
    return arr->mem[n];
}




static inline void arrItInit(arr_it_t *it, arr_t *arr)
{
    it->arr      = arr;
    it->curblock = 0;
    it->val      = it->arr->mem[0];
}

static inline void __arrItNext(arr_it_t *it, size_t typesize)
{
    it->val = (char *)it->val + typesize;
    if ((char *)it->val >= (char *)it->arr->mem[it->curblock] + arrBlockSize(it->arr)) {
        it->curblock++;
        it->val = it->arr->mem[it->curblock];
    }
}
#endif /* __ARR_H__ */
