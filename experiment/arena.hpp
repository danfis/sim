#ifndef ARENA_HPP
#define ARENA_HPP

#include <sim/sim.hpp>
#include <sim/world.hpp>
#include <sim/sensor/camera.hpp>
#include <sim/comp/sssa.hpp>
#include <sim/msg.hpp>

using sim::Vec3;
using sim::Quat;

class PowerSource : public sim::Component {
    sim::Body *_body;

  public:
    PowerSource(sim::Sim *sim,
                const Vec3 &pos = Vec3(0, 0, 0),
                const Quat &rot = Quat(0., 0., 0., 1.));

    void init(sim::Sim *sim);
}; 

class Arena : public sim::Component {
    sim::Body *_frame, *_ice;
    std::list<PowerSource *> _pw_sources;

  public:
    Arena(sim::Sim *sim);

    void init(sim::Sim *sim);
};


#endif /* ARENA_HPP */

