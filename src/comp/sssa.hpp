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

#ifndef _SIM_COMP_SSSA_HPP_
#define _SIM_COMP_SSSA_HPP_

#include <sim/sim.hpp>
#include <sim/component.hpp>
#include <sim/robot/sssa.hpp>

namespace sim {

namespace comp {

class SSSA : public sim::Component {
  protected:
    sim::Sim *_sim;

    sim::robot::SSSA *_robot;
    sim::Vec3 _pos;
    sim::Quat _rot;

  public:
    SSSA(const sim::Vec3 &pos, const sim::Quat &rot = sim::Quat(0., 0., 0., 1.));
    ~SSSA();

    const sim::Sim *sim() const { return _sim; }
    sim::Sim *sim() { return _sim; }
    const sim::robot::SSSA *robot() const { return _robot; }
    sim::robot::SSSA *robot() { return _robot; }

    void init(sim::Sim *sim);
};

} /* namespace comp */

} /* namespace sim */

#endif /* _SIM_COMP_SSSA_HPP_ */
