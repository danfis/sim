#ifndef COMMON_MSG_HPP_
#define COMMON_MSG_HPP_

#include <iostream>

#ifndef NDEBUG
# define DBG(str) \
    std::cerr << __func__ << ": " << str << std::endl; \
    std::cerr.flush()
#else
# define DBG(str)
#endif

#define ERR(str) \
    std::cerr << "Error: " << str << std::endl; \
    std::cerr.flush()

#endif
