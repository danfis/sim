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

#ifndef _SIM_RAND_HPP_
#define _SIM_RAND_HPP_

#include <sim/time.hpp>

namespace sim {

class Rand {
  protected:
    unsigned int _seed; //<! seed for generation random numbers
    unsigned int _rand_nums; //<! number of numbers returned from rand()
                             //<! for one seed
    unsigned int _max_nums; //<! max number of random numbers between reseeding

  public:
    Rand(unsigned int max_nums = 100)
        : _seed((unsigned int)Time::cur().inMs()),
          _rand_nums(0), _max_nums(max_nums)
    { }

    inline int rand()
    {
        if (_rand_nums > _max_nums){
            _seed = rand_r(&_seed);
            _rand_nums = 0;
        }

        _rand_nums++;
        return rand_r(&_seed);
    }

    inline int rand(int from, int to)
    {
        int r = randF(from, to);
        if (r >= to)
            r = to - 1;
        return r;
    }

    inline double randF(double from, double to)
    {
        double r = rand();
        r /= (double)RAND_MAX + (double)1;
        r *= to - from;
        r += from;
        return r;
    }
};

} /* namespace sim */

#endif /* _SIM_RAND_HPP_ */
