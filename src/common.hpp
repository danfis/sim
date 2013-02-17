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
