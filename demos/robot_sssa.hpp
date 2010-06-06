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

#ifndef _ROBOT_SSSA_HPP_
#define _ROBOT_SSSA_HPP_

#include <sim/component.hpp>
#include <sim/robot/sssa.hpp>


class SSSAComp : public sim::Component {
    sim::Sim *_sim;

    sim::robot::SSSA *_robot;
    sim::Vec3 _pos;

  public:
    SSSAComp(const sim::Vec3 &pos);
    SSSAComp(sim::robot::SSSA *robot);

    sim::robot::SSSA *robot() { return _robot; }

    void init(sim::Sim *sim);
    void finish();
    void cbPostStep();
    void processMessage(const sim::Message &msg);

  protected:
    void _keyPressedMsg(const sim::MessageKeyPressed &msg);
};

#endif
