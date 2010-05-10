#include "sim.hpp"
#include "world.hpp"

namespace sim {

namespace bullet {

Sim::Sim(sim::VisWorld *visworld)
    : sim::Sim(new World(), visworld)
{
}

Sim::~Sim()
{
}

}

}
