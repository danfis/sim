#ifndef _SIM_ROBOT_SYROTEK_HPP_
#define _SIM_ROBOT_SYROTEK_HPP_

#include <sim/ode/world.hpp>
#include <sim/ode/body.hpp>

namespace sim {

namespace robot {

class Syrotek {
  protected:
    sim::World *_world;
    sim::Vec3 _pos;

    sim::ode::BodyCompound *_chasis;
    sim::Body *_wheel[2], *_ball[2];
    sim::ode::JointFixed *_jball[2];
    sim::ode::JointHinge *_jwheel[2];

    sim::Scalar _vel[2];

  public:
    Syrotek(sim::World *w, const sim::Vec3 &pos = sim::Vec3(0., 0., 0.08),
            const osg::Vec4 &chasis_color = osg::Vec4(0., 0.1, 0.7, 0.6));

    const sim::Body *chasis() const { return _chasis; }
    sim::Body *chasis() { return _chasis; }

    void setColor(const osg::Vec4 &color);
    void setColor(Scalar r, Scalar g, Scalar b, Scalar a = 1.)
        { setColor(osg::Vec4(r, g, b, a)); }

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
