/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
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

/** \file msg.hpp
 * This file contains macros for printing messages mostly debugging.
 * This file should NOT be included in any header file because very simple
 * (no namespace) names for macors are chosen.
 */

#ifndef COMMON_MSG_HPP_
#define COMMON_MSG_HPP_

#include <iostream>

#ifndef NDEBUG

#ifdef __FILE__
// I assume that if __FILE__ is defined __LINE__ would be also defined
// __func__ should be everywhere
# define DBG_PREFIX __FILE__ << ":" \
                    << __LINE__ << "::" \
                    << __func__ << "(): "
#else /* __FILE__ */
# define DBG_PREFIX ""
#endif /* __FILE__ */

/**
 * Prints out debug informations to stderr
 */
# define DBG(str) \
    std::cerr << DBG_PREFIX << str << std::endl; \
    std::cerr.flush()
# define DBGV(v) \
    (v).x() << " " << (v).y() << " " << (v).z()
#else
# define DBG(str)
# define DBGV(v)
#endif

/**
 * Prints out str to stderr with note that it is error.
 */
#define ERR(str) \
    std::cerr << "Error: " << str << std::endl; \
    std::cerr.flush()

#define MSG(str) \
    std::cerr << str << std::endl; \
    std::cerr.flush()

#endif
