#ifndef SIM_HPP
#define SIM_HPP

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/msg.hpp>
#include <sim/comp/povray_full.hpp>

#include "arena.hpp"
#include "robot.hpp"

using sim::Vec3;
using sim::Quat;

class Sim : public sim::Sim {
    Arena *_arena;
    sim::comp::PovrayFull *_povray;
    std::list<Robot *> _robots;

  public:
    Sim();

    virtual void init();

    void waitForRobot(Robot *r);
    void connectRobots();

  protected:
    void createArena();
    void createRobots();

    Robot *_emergeRobot(Robot *r);
};


#endif /* SIM_HPP */


