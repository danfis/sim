/***
 * sim
 * ---------------------------------
 * Copyright (c)2011 Daniel Fiser <danfis@danfis.cz>
 *
 *  This file is part of sim.
 *
 *  sim is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation; either version 3 of
 *  the License, or (at your option) any later version.
 *
 *  sim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SIM_COMPILER_H__
#define __SIM_COMPILER_H__

#include <stddef.h>

#define sim_offsetof(TYPE, MEMBER) offsetof(TYPE, MEMBER)

#define sim_container_of(ptr, type, member) \
    (type *)( (char *)ptr - sim_offsetof(type, member))


/**
 * Marks inline function.
 */
#ifdef __GNUC__
# ifdef SIM_DEBUG
#  define _sim_inline static
# else /* SIM_DEBUG */
#  define _sim_inline static inline __attribute__((always_inline))
# endif /* SIM_DEBUG */
#else /* __GNUC__ */
# define _sim_inline static inline
#endif /* __GNUC__ */


/**
 * __prefetch(x)  - prefetches the cacheline at "x" for read
 * __prefetchw(x) - prefetches the cacheline at "x" for write
 */
#ifdef __GNUC__
# define _sim_prefetch(x) __builtin_prefetch(x)
# define _sim_prefetchw(x) __builtin_prefetch(x,1)
#else /* __GNUC__ */
# define _sim_prefetch(x)
# define _sim_prefetchw(x)
#endif /* __GNUC__ */

/**
 * Using this macros you can specify is it's likely or unlikely that branch
 * will be used.
 * Comes from linux header file ./include/compiler.h
 */
#ifdef __GNUC__
# define sim_likely(x) __builtin_expect(!!(x), 1)
# define sim_unlikely(x) __builtin_expect(!!(x), 0)
#else /* __GNUC__ */
# define sim_likely(x) !!(x)
# define sim_unlikely(x) !!(x)
#endif /* __GNUC__ */

#ifdef __GNUC__
# define sim_aligned(x) __attribute__ ((aligned(x)))
# define sim_packed __attribute__ ((packed))
#else /* __GNUC__ */
# define sim_aligned(x)
# define sim_packed
#endif /* __GNUC__ */


#ifdef __ICC
/* disable unused parameter warning */
# pragma warning(disable:869)
/* disable annoying "operands are evaluated in unspecified order" warning */
# pragma warning(disable:981)
#endif /* __ICC */

#endif /* __SIM_COMPILER_H__ */

