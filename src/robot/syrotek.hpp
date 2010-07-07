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

#ifndef _SIM_ROBOT_SYROTEK_HPP_
#define _SIM_ROBOT_SYROTEK_HPP_

#include <sim/world.hpp>

namespace sim {

namespace robot {

class Syrotek {
  protected:
    sim::World *_world;
    sim::Vec3 _pos; //!< Initial position

    sim::Vec3 _odo_pos; //!< Initial position from where odometry is computed
    sim::Quat _odo_rot; //!< Initial rotation from which odometry is computed

    sim::Body *_chasis;
    sim::Body *_wheel[2], *_ball[2];
    sim::Joint *_jball[2];
    sim::Joint *_jwheel[2];

    sim::Scalar _vel[2];

  public:
    Syrotek(sim::World *w, const sim::Vec3 &pos = sim::Vec3(0., 0., 0.08),
            const osg::Vec4 &chasis_color = osg::Vec4(0., 0.1, 0.7, 0.6));

    const sim::Body *chasis() const { return _chasis; }
    sim::Body *chasis() { return _chasis; }

    void setColor(const osg::Vec4 &color);
    void setColor(Scalar r, Scalar g, Scalar b, Scalar a = 1.)
        { setColor(osg::Vec4(r, g, b, a)); }

    const Vec3 &pos() const { return chasis()->pos(); }
    const Quat &rot() const { return chasis()->rot(); }

    /**
     * Returns current odometry (x, y, phi)
     */
    Vec3 odometry();

    /**
     * Resets odometry sensor.
     */
    void resetOdometry();

    void activate();

    void setVelLeft(sim::Scalar vel);
    void addVelLeft(sim::Scalar d);
    sim::Scalar velLeft() const { return _vel[0]; }

    void setVelRight(sim::Scalar vel);
    void addVelRight(sim::Scalar d);
    sim::Scalar velRight() const { return _vel[1]; }
};

} /* namespace robot */

} /* namespace sim */

#endif /* _SIM_ROBOT_SYROTEK_HPP_ */
