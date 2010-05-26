#ifndef ROBOT_SYROTEK_HPP
#define ROBOT_SYROTEK_HPP

#include <sim/sim.hpp>
#include <sim/component.hpp>
#include <sim/robot/syrotek.hpp>
#include <SDL/SDL.h>

class RobotSyrotekComp : public sim::Component {
    sim::Sim *_sim;

    sim::robot::Syrotek *_robot;

    sim::Time _joystick_delay, _joystick_last;
    SDL_Joystick *_joystick;

  public:
    RobotSyrotekComp();
    ~RobotSyrotekComp();

    const sim::robot::Syrotek *robot() const { return _robot; }

    void init(sim::Sim *sim);
    void finish();

    void cbPreStep();
    void processMessage(const sim::Message &msg);
};

#endif
