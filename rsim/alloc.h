/***
 * sim
 * ---------------------------------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SIM_ALLOC_H__
#define __SIM_ALLOC_H__

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * Functions and macros required for memory allocation.
 */

/* Memory allocation: */
#define __SIM_ALLOC_MEMORY(type, ptr_old, size) \
    (type *)simRealloc((void *)ptr_old, (size))

/** Allocate memory for one element of type.  */
#define SIM_ALLOC(type) \
    __SIM_ALLOC_MEMORY(type, NULL, sizeof(type))

/** Allocate memory for array of elements of type type.  */
#define SIM_ALLOC_ARR(type, num_elements) \
    __SIM_ALLOC_MEMORY(type, NULL, sizeof(type) * (num_elements))

#define SIM_REALLOC_ARR(ptr, type, num_elements) \
    __SIM_ALLOC_MEMORY(type, ptr, sizeof(type) * (num_elements))

void *simRealloc(void *ptr, size_t size);

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __SIM_ALLOC_H__ */
