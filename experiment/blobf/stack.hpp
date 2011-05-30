#ifndef __STACK_H__
#define __STACK_H__

#include <string.h>
#include "arr.hpp"

namespace blobf {

struct _stack_block_t {
    void *mem;
    struct _stack_block_t *prev, *next;
};
typedef struct _stack_block_t stack_block_t;

struct _stack_t {
    stack_block_t *cur; /*! current block */
    void *top;          /*! pointer to item on top of stack */

    size_t blocksize; /*! sizeof(stack_block_t) + memsize */
    size_t memsize;   /*! size of memory block stored in stack_block_t */
};
typedef struct _stack_t stack_t;


#define stackInit(stack, type) \
    __stackInit((stack), sizeof(type))
static inline void __stackInit(stack_t *s, size_t typesize);
static inline void stackDestroy(stack_t *s);

/**
 * Returns true if stack is empty.
 */
static inline int stackEmpty(stack_t *s);

/**
 * Returns item from top of stack. Stack is not altered.
 */
#define stackTop(stack, type) \
    *(type *)__stackTop(stack)
static inline void *__stackTop(stack_t *s);

/**
 * Pops top item from stack. Item is not returned (see stackTop()).
 */
#define stackPop(stack, type) \
    __stackPop((stack), sizeof(type))
static inline void __stackPop(stack_t *stack, size_t typesize);

/**
 * Push value on stack.
 */
#define stackPush(stack, type, val) \
    do { \
        __stackPush((stack), sizeof(type), &(val)); \
        *(type *)(stack)->top = (val); \
    } while (0)
static inline void __stackPush(stack_t *stack, size_t typesize, void *val);


/**** INLINE ****/
static inline void __stackInit(stack_t *s, size_t typesize)
{
    size_t size;
    stack_block_t *sb;

    size = ARR_BLOCK_SIZE - sizeof(stack_block_t);
    s->memsize = size - (size % typesize);
    s->blocksize = sizeof(stack_block_t) + s->memsize;

    sb = (stack_block_t *)malloc(s->blocksize);
    sb->next = sb->prev = NULL;
    sb->mem = (void *)(((char *)sb) + sizeof(stack_block_t));

    s->cur = sb;
    s->top = NULL;
}

static inline void stackDestroy(stack_t *stack)
{
    stack_block_t *s, *tmp;

    s = stack->cur;
    while (s->prev != NULL)
        s = s->prev;

    while (s != NULL){
        tmp = s;
        s = s->next;
        free(tmp);
    }
}

static inline int stackEmpty(stack_t *s)
{
    return s->top == NULL;
}

static inline void *__stackTop(stack_t *s)
{
    return s->top;
}

static inline void __stackPop(stack_t *stack, size_t typesize)
{
    if (stack->top == NULL){
        // nothing on stack
        return;
    }else if (stack->cur->mem != stack->top){
        // top of stack is not first item in mem block, so simple substract
        // size of type stored on stack
        stack->top = (void *)(((char *)stack->top) - typesize);
    }else if (stack->cur->prev != NULL){
        // top of stack is first item in mem block and there is already
        // allocated block before current one. Set current block as
        // previous one and and top of stack as last item in block.
        stack->cur = stack->cur->prev;
        stack->top = (void *)(((char *)stack->cur->mem) + stack->memsize - typesize);
    }else{
        // top of stack is first item in mem block and there is this block
        // is last one, i.e., delete last item from stack.
        stack->top = NULL;
    }
}

static inline void __stackPush(stack_t *stack, size_t typesize, void *val)
{
    size_t diff;
    stack_block_t *memblock;

    if (stack->top == NULL){
        // stack is empty, push first item into mem block
        stack->top = stack->cur->mem;
    }else{
        diff = ((char *)stack->top + typesize) - (char *)stack->cur->mem;
        if (diff == stack->memsize){
            // next item is in next mem block

            if (stack->cur->next == NULL){
                // new block must be added
                memblock = (stack_block_t *)malloc(stack->blocksize);
                memblock->next = NULL;
                memblock->prev = stack->cur;
                stack->cur->next = memblock;
                memblock->mem = (void *)(((char *)memblock) + sizeof(stack_block_t));

                stack->cur = memblock;
                stack->top = stack->cur->mem;
            }else{
                // move to first item in next block
                stack->cur = stack->cur->next;
                stack->top = stack->cur->mem;
            }
        }else{
            // move top pointer to next item
            stack->top = (void *)(((char *)stack->top) + typesize);
        }
    }

    //memcpy(stack->top, val, typesize);
}

} /* namespace blobf */

#endif /* __STACK_H__ */
