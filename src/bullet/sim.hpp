/***
 * sim
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
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

#ifndef _SIM_BULLET_SIM_HPP_
#define _SIM_BULLET_SIM_HPP_

#include "sim/sim.hpp"

namespace sim {

namespace bullet {

class Sim : public sim::Sim {
  public:
    Sim(VisWorld *visworld);
    virtual ~Sim();
};

} /* namespace bullet */

} /* namespace sim */

#endif /* _SIM_BULLET_SIM_HPP_ */
