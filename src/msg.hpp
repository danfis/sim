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
#else
# define DBG(str)
#endif

/**
 * Prints out str to stderr with note that it is error.
 */
#define ERR(str) \
    std::cerr << "Error: " << str << std::endl; \
    std::cerr.flush()

#endif
