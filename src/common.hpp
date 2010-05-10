#ifndef _SIM_COMMON_HPP_
#define _SIM_COMMON_HPP_

/**
 * Macro for iterating over std::lists and such.
 * First parameter is type of iterator and second is list.
 *
 * Inside block under this macro is available iterator it pointing to
 * current item from list.
 * Example:
 *    for_each(std::<Stuff *>::iterator, list){
 *        (*it)->someMethod();
 *    }
 */
#define for_each(it_type, list) \
    for (it_type it = (list).begin(), \
                 it_end = (list).end(); \
         it != it_end; \
         ++it)

#endif /* _SIM_COMMON_HPP_ */
