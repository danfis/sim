#ifndef SIM_HPP
#define SIM_HPP

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/msg.hpp>

#include "arena.hpp"
#include "robot.hpp"

using sim::Vec3;
using sim::Quat;

class Sim : public sim::Sim {
    Arena *_arena;
    std::list<Robot *> _robots;

  public:
    Sim();

    virtual void init();

  protected:
    void createArena();
    void createRobots();
};


#endif /* SIM_HPP */


