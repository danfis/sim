#ifndef ROBOT_SYROTEK_HPP
#define ROBOT_SYROTEK_HPP

#include <sim/sim.hpp>
#include <sim/component.hpp>
#include <sim/ode/body.hpp>
#include <sim/ode/joint.hpp>

class RobotSyrotek {
    sim::World *_world;
    sim::Vec3 _pos;

    sim::ode::BodyCompound *_chasis;
    sim::Body *_wheel[2], *_ball[2];
    sim::ode::JointFixed *_jball[2];
    sim::ode::JointHinge *_jwheel[2];

    sim::Scalar _vel[2];

  public:
    RobotSyrotek(sim::World *w, const sim::Vec3 &pos = sim::Vec3(0., 0., 0.08));

    void activate();

    void addVelLeft(sim::Scalar d);
    sim::Scalar velLeft() const { return _vel[0]; }
    void addVelRight(sim::Scalar d);
    sim::Scalar velRight() const { return _vel[1]; }
};


class RobotSyrotekComp : public sim::Component {
    sim::Sim *_sim;

    RobotSyrotek *_robot;

  public:
    RobotSyrotekComp();
    ~RobotSyrotekComp();

    void init(sim::Sim *sim);
    void finish();
    void processMessage(const sim::Message &msg);
};

#endif
