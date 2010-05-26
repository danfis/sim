#ifndef ROBOT_SYROTEK_HPP
#define ROBOT_SYROTEK_HPP

#include <sim/sim.hpp>
#include <sim/component.hpp>
#include <sim/robot/syrotek.hpp>
#include <sim/comp/joystick.hpp>

class RobotSyrotekComp : public sim::Component {
    sim::Sim *_sim;

    sim::robot::Syrotek *_robot;

  public:
    RobotSyrotekComp();
    ~RobotSyrotekComp();

    const sim::robot::Syrotek *robot() const { return _robot; }

    void init(sim::Sim *sim);
    void finish();

    void cbPreStep();
    void processMessage(const sim::Message &msg);

  protected:
    void _keyPressed(const sim::MessageKeyPressed &msg);
    void _joystick(const sim::comp::JoystickMessage &msg);
};

#endif
